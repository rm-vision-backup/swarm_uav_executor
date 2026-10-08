#!/usr/bin/env python3
"""
15 机无头 SITL 批量 arm + OFFBOARD 脚本（每机独立 ROS Master 11311..11325）。

迁移自 safe_valley_exp/scripts/offboard_takeoff_15.py（implementation_plan_26083018 §4.3）。
**仅适用于本机 15 机 SITL 仿真流程**；实机 takeoff 使用独立 onboard agent 方案，
不复用本脚本（固定端口 11311..11325、本机执行）。

职责（配合正确启动流程）：
  0. 先启动机载层（offboard 程序）：MAVROS + ego driver + executor + bridge，
     统一坐标系并待命；ego driver 初态为 HOLD，经 setpoint_relay
     以 30Hz 独占发布 /mavros/setpoint_raw/local（唯一 MAVROS setpoint 发布者）。
  1. 本脚本对选中的每架 UAV：
     a. 先切 HOLD（Auto 模式，arm 时不要求 RC 输入）
     b. arm 无人机
     c. 切 OFFBOARD
     d. 成功进入 OFFBOARD 后立即退出；ego_planner_driver 在 armed + OFFBOARD
         上升沿进入 TAKEOFF，起飞完成后回到 HOLD

用法：
  python3 offboard_takeoff_15_sitl.py 1 2 3
  不写编号 -> 默认 1..15；默认逐机运行。
  加 --parallel -> 所有选中 UAV 并发运行。

注意:
  - 本脚本会自动注入无 RC SITL 必需参数：COM_RCL_EXCEPT=4、NAV_RCL_ACT=0。
    任一参数设置失败即中止本机，避免 RC 丢失触发 failsafe（RTL）
  - arm 前先切 HOLD（Auto 模式，arm 不要求 RC 输入）：仿真无遥控可直接 arm，
    实机遥控仅作应急接管；若 HOLD 被拒则回退 AUTO.LOITER（同为 Auto 模式）
  - 起飞高度由 ego_planner_driver 的 takeoff_height_m 参数控制（默认 5.0m）
  - 本脚本不再调用 /mavros/cmd/takeoff（MAV_CMD_NAV_TAKEOFF），
    避免 PX4 preflight 在 set_gp_origin 修改 EKF2 origin 后拦截
"""
import argparse
import os
import sys

import rospy
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandBool, ParamPush, ParamSet, SetMode

ROS_SETUP = "/opt/ros/noetic/setup.bash"
# 工作空间根 = 本脚本所在包的三级父目录（不写死某个用户的 home）
WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))


class TakeoffUAV:
    def __init__(self, idx):
        self.idx = idx
        self.master_port = 11310 + idx

    def _inject_rc_params(self):
        """注入无 RC SITL 必需参数：COM_RCL_EXCEPT=4、NAV_RCL_ACT=0。

        任一参数设置失败即返回非零，调用方应中止本机起飞，避免参数
        未就绪时 RC 丢失触发 failsafe（RTL）。
        """
        try:
            rospy.wait_for_service("/mavros/param/set", timeout=10)
            param_set = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("UAV%d: param/set 服务不可用: %s", self.idx, exc)
            return 1
        for pid, ival in (("COM_RCL_EXCEPT", 4), ("NAV_RCL_ACT", 0)):
            pv = ParamValue()
            pv.integer = ival
            pv.real = 0.0
            try:
                resp = param_set(pid, pv)
            except rospy.ServiceException as exc:
                rospy.logerr("UAV%d: param %s 设置异常: %s", self.idx, pid, exc)
                return 1
            if not resp.success:
                rospy.logerr("UAV%d: param %s=%d 被拒绝", self.idx, pid, ival)
                return 1
            rospy.loginfo("UAV%d: param %s=%d success=%s",
                          self.idx, pid, ival, resp.success)
        try:
            rospy.wait_for_service("/mavros/param/push", timeout=10)
            push = rospy.ServiceProxy("/mavros/param/push", ParamPush)
            presp = push()
            rospy.loginfo("UAV%d: param push transfered=%s",
                          self.idx, presp.param_transfered)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("UAV%d: param push 失败（本次已生效，仅影响持久化）: %s",
                          self.idx, exc)
        return 0

    def run(self):
        os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % self.master_port
        os.environ["ROS_HOSTNAME"] = "localhost"
        rospy.init_node("takeoff_uav%d" % self.idx, anonymous=True)
        rospy.loginfo("UAV%d: master=%d", self.idx, self.master_port)

        # 0) 注入无 RC SITL 必需参数（任一失败即中止，避免 RC failsafe -> RTL）
        if self._inject_rc_params() != 0:
            return 1

        # 1) 准备 set_mode 服务（HOLD/LOITER 预选 + OFFBOARD 共用）
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=10)
            set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: set_mode 服务不可用: %s", self.idx, exc)
            return 1

        # 2) 先切 HOLD（Auto 模式，arm 时不要求 RC 输入）。
        #    仿真无遥控：不依赖虚拟摇杆即可 arm；
        #    实机有遥控：遥控仅作应急接管，不参与常规起飞流程。
        #    若 HOLD 被 PX4 拒绝则回退 AUTO.LOITER（同为 Auto 模式，arm 同样免 RC）。
        hold_ok = False
        for mode in ("HOLD", "AUTO.LOITER"):
            resp = set_mode(0, mode)
            rospy.loginfo("UAV%d: %s mode_sent=%s", self.idx, mode, resp.mode_sent)
            if resp.mode_sent:
                hold_ok = True
                break
        if not hold_ok:
            rospy.logerr("UAV%d: HOLD/AUTO.LOITER 预选全部失败", self.idx)
            return 1

        # 3) arm。ego_planner_driver 等待 armed + OFFBOARD 上升沿触发 TAKEOFF。
        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=10)
            arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: arm 服务不可用: %s", self.idx, exc)
            return 1

        resp = arm(True)
        rospy.loginfo("UAV%d: arm result=%s", self.idx, resp.success)
        if not resp.success:
            rospy.logerr("UAV%d: arm 被拒绝", self.idx)
            return 1

        # 4) 切 OFFBOARD。只看服务响应，不等状态回报。
        resp = set_mode(0, "OFFBOARD")
        rospy.loginfo("UAV%d: OFFBOARD mode_sent=%s", self.idx, resp.mode_sent)
        if not resp.mode_sent:
            rospy.logerr("UAV%d: OFFBOARD 请求失败", self.idx)
            return 1
        rospy.loginfo("UAV%d: HOLD -> arm -> OFFBOARD 请求完成；脚本退出", self.idx)
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="选定 UAV arm + OFFBOARD（默认逐机，ego driver 自动软起飞）")
    parser.add_argument("uavs", nargs="*", type=int, metavar="UAV",
                        help="UAV 编号，例如：1 2 3；不写则运行 1..15")
    parser.add_argument("--parallel", action="store_true",
                        help="并发运行；默认逐机运行")
    parser.add_argument("--individual", action="store_true",
                        help="兼容旧命令；默认就是逐机运行")
    args = parser.parse_args()

    results = {}
    idxs = args.uavs or list(range(1, 16))
    if any(idx < 1 or idx > 15 for idx in idxs):
        parser.error("UAV 编号必须在 1..15")
    if len(set(idxs)) != len(idxs):
        parser.error("UAV 编号不能重复")

    if not args.parallel:
        # 逐机串行：每个 UAV 在独立子进程内运行 --single，避免
        # rospy.init_node 同一进程重复调用的问题。
        import subprocess
        for idx in idxs:
            cmd = [
                "bash", "-c",
                "source %s && source %s/devel/setup.bash && "
                "export ROS_MASTER_URI=http://localhost:%d && "
                "exec python3 %s --single %d"
                % (ROS_SETUP, WS, 11310 + idx, __file__, idx),
            ]
            r = subprocess.run(cmd)
            results[idx] = "OK" if r.returncode == 0 else "FAIL(%d)" % r.returncode
    else:
        import subprocess
        procs = {}
        for idx in idxs:
            cmd = [
                "bash", "-c",
                "source %s && source %s/devel/setup.bash && "
                "export ROS_MASTER_URI=http://localhost:%d && "
                "exec python3 %s --single %d"
                % (ROS_SETUP, WS, 11310 + idx, __file__, idx),
            ]
            p = subprocess.Popen(cmd)
            procs[idx] = p
        for idx in idxs:
            p = procs[idx]
            p.wait()
            results[idx] = "OK" if p.returncode == 0 else "FAIL(%d)" % p.returncode

    print("=== arm+OFFBOARD 汇总 ===")
    for idx in idxs:
        print("UAV%-2d (%d): %s" % (idx, 11310 + idx, results.get(idx, "?")))
    ok = [i for i, v in results.items() if v == "OK"]
    print("成功 %d/%d: %s" % (len(ok), len(idxs), sorted(ok)))
    return 0 if len(ok) == len(idxs) else 1


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        p = argparse.ArgumentParser()
        p.add_argument("--single", type=int)
        a, _ = p.parse_known_args()
        sys.exit(0 if TakeoffUAV(a.single).run() == 0 else 1)
    sys.exit(main())