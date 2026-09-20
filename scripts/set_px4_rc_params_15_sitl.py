#!/usr/bin/env python3
"""15 机 SITL：把"无遥控器（RC 丢失）"相关的 PX4 参数写入每架飞控并回读校验。

参数（与 SITL rcS 中的无条件 param set 保持一致）：
  COM_RC_IN_MODE  = 1  # Joystick only：不处理 RC 输入、跳过 RC 相关检查
  NAV_RCL_ACT     = 0  # 0 = link_loss_actions_t::DISABLED：RC 丢失不切换模式
  COM_RCL_EXCEPT  = 7  # Mission(1) | Hold(2) | Offboard(4)：这些模式下忽略 RC 丢失

为什么需要本脚本：rcS 在每次 SITL 启动时写入这三个参数；但已启动的仿真、或
eeprom 未被清空（QGC 改过参数）时，需要显式再写一遍。arm 前写入即生效。

仅适用于本机 15 机 SITL 仿真流程（各机独立 master 11311..11325）。

用法：
  python3 set_px4_rc_params_15_sitl.py             # 默认 1..15
  python3 set_px4_rc_params_15_sitl.py 1 2 3       # 指定机号
  python3 set_px4_rc_params_15_sitl.py --no-push   # 不调用 param/push 持久化
"""
import argparse
import os
import subprocess
import sys

import rospy
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamGet, ParamPush, ParamSet

ROS_SETUP = "/opt/ros/noetic/setup.bash"
WS = "/home/yjq/catkin_swarm6-2"

# (参数名, 整数取值)：COM_RC_IN_MODE/NAV_RCL_ACT/COM_RCL_EXCEPT 均为 INT32
RC_PARAMS = (
    ("COM_RC_IN_MODE", 1),
    ("NAV_RCL_ACT", 0),
    ("COM_RCL_EXCEPT", 7),
)


class RcParamsUAV:
    def __init__(self, idx, push=True):
        self.idx = idx
        self.master_port = 11310 + idx
        self.push = push

    @staticmethod
    def _int_value(param):
        """ParamValue -> int（这三个参数都是 INT32，直接读 integer 字段）。"""
        return int(param.integer)

    def run(self):
        os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % self.master_port
        os.environ["ROS_HOSTNAME"] = "localhost"
        rospy.init_node("set_rc_params_uav%d" % self.idx, anonymous=True)

        try:
            rospy.wait_for_service("/mavros/param/set", timeout=10)
            rospy.wait_for_service("/mavros/param/get", timeout=10)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: param 服务不可用: %s", self.idx, exc)
            return 1
        param_set = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        param_get = rospy.ServiceProxy("/mavros/param/get", ParamGet)

        failed = []
        for name, value in RC_PARAMS:
            pv = ParamValue()
            pv.integer = value
            pv.real = 0.0
            try:
                resp = param_set(name, pv)
            except rospy.ServiceException as exc:
                rospy.logerr("UAV%d: %s 设置异常: %s", self.idx, name, exc)
                failed.append("%s(set异常)" % name)
                continue
            if not resp.success:
                rospy.logerr("UAV%d: %s=%d 被拒绝", self.idx, name, value)
                failed.append("%s(被拒)" % name)
                continue
            back = self._int_value(param_get(name).value)
            if back != value:
                rospy.logerr("UAV%d: %s 回读不符 (%d != %d)",
                             self.idx, name, back, value)
                failed.append("%s(回读%d)" % (name, back))
            else:
                rospy.loginfo("UAV%d: %s=%d 写入并回读一致", self.idx, name, value)

        if self.push and not failed:
            try:
                rospy.wait_for_service("/mavros/param/push", timeout=10)
                presp = rospy.ServiceProxy("/mavros/param/push", ParamPush)()
                rospy.loginfo("UAV%d: param push transfered=%s",
                              self.idx, presp.param_transfered)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                # 本次已生效，仅影响掉电持久化
                rospy.logwarn("UAV%d: param push 失败（本次已生效）: %s",
                              self.idx, exc)

        if failed:
            rospy.logerr("UAV%d: 失败项 %s", self.idx, ",".join(failed))
            return 1
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="15 机 SITL 写入无 RC（RC 丢失）相关 PX4 参数")
    parser.add_argument("uavs", nargs="*", type=int, metavar="UAV",
                        help="UAV 编号，例如：1 2 3；不写则运行 1..15")
    parser.add_argument("--no-push", action="store_true",
                        help="不调用 /mavros/param/push 持久化")
    args = parser.parse_args()

    idxs = args.uavs or list(range(1, 16))
    if any(idx < 1 or idx > 15 for idx in idxs):
        parser.error("UAV 编号必须在 1..15")
    if len(set(idxs)) != len(idxs):
        parser.error("UAV 编号不能重复")

    results = {}
    for idx in idxs:
        # 逐机串行：每个 UAV 独立 master，子进程内设置 ROS_MASTER_URI
        cmd = [
            "bash", "-c",
            "source %s && source %s/devel/setup.bash && "
            "exec python3 %s --single %d%s"
            % (ROS_SETUP, WS, __file__, idx, "" if not args.no_push else " --no-push"),
        ]
        r = subprocess.run(cmd)
        results[idx] = "OK" if r.returncode == 0 else "FAIL(%d)" % r.returncode

    print("=== PX4 无 RC 参数汇总 ===")
    for idx in idxs:
        print("UAV%-2d (%d): %s" % (idx, 11310 + idx, results.get(idx, "?")))
    ok = [i for i, v in results.items() if v == "OK"]
    print("成功 %d/%d: %s" % (len(ok), len(idxs), sorted(ok)))
    return 0 if len(ok) == len(idxs) else 1


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        p = argparse.ArgumentParser()
        p.add_argument("--single", type=int)
        p.add_argument("--no-push", action="store_true")
        a, _ = p.parse_known_args()
        sys.exit(0 if RcParamsUAV(a.single, push=not a.no_push).run() == 0 else 1)
    sys.exit(main())
