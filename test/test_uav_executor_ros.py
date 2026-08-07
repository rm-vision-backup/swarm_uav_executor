#!/usr/bin/env python3
import unittest, rospy, rostest
from swarm_uav_interfaces.msg import TaskAssignment, UavTaskState
from swarm_uav_interfaces.srv import UavTask, UavTaskRequest
class RosTest(unittest.TestCase):
    def test_service_and_terminal_topic(self):
        messages=[]; rospy.Subscriber('/UAV1/uav_task_state',UavTaskState,messages.append)
        rospy.wait_for_service('/UAV1/uav_task',5); proxy=rospy.ServiceProxy('/UAV1/uav_task',UavTask)
        req=UavTaskRequest(protocol_version='1.0',mission_id='rostest',group_id='GroupA',command_id='c1',uav_id='A01',exec_target='UAV1',command='MOVE_TO',timeout_s=2,leader_id='')
        req.assignment=TaskAssignment(uav_id='A01'); response=proxy(req); self.assertTrue(response.accepted)
        end=rospy.Time.now()+rospy.Duration(3)
        while rospy.Time.now()<end and not any(m.status=='COMPLETED' for m in messages): rospy.sleep(.02)
        self.assertTrue(any(m.status=='ACCEPTED' for m in messages)); self.assertTrue(any(m.status=='COMPLETED' for m in messages))
if __name__=='__main__': rospy.init_node('test_uav_executor_ros'); rostest.rosrun('swarm_uav_executor','uav_executor_ros',RosTest)
