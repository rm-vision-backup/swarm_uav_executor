#!/usr/bin/env python3
import unittest, rospy, rostest
from swarm_uav_interfaces.msg import TaskAssignment, UavTaskState
from swarm_uav_interfaces.srv import UavSafetyLease, UavSafetyLeaseRequest, UavTask, UavTaskRequest
class RosTest(unittest.TestCase):
    def test_service_and_terminal_topic(self):
        messages=[]; subscriber=rospy.Subscriber('/UAV1/uav_task_state',UavTaskState,messages.append)
        rospy.wait_for_service('/UAV1/uav_safety_lease',5)
        lease=rospy.ServiceProxy('/UAV1/uav_safety_lease',UavSafetyLease)
        start=UavSafetyLeaseRequest('1.0','START','rostest','epoch-rostest',1,5.0,'A01','UAV1','rostest')
        self.assertTrue(lease(start).accepted)
        rospy.wait_for_service('/UAV1/uav_task',5); proxy=rospy.ServiceProxy('/UAV1/uav_task',UavTask)
        connection_deadline=rospy.Time.now()+rospy.Duration(2)
        while rospy.Time.now()<connection_deadline and subscriber.get_num_connections()==0: rospy.sleep(.02)
        self.assertGreater(subscriber.get_num_connections(),0)
        req=UavTaskRequest(protocol_version='1.0',mission_id='rostest',group_id='GroupA',command_id='c1',uav_id='A01',exec_target='UAV1',command='MOVE_TO',timeout_s=2,leader_id='')
        req.assignment=TaskAssignment(uav_id='A01'); response=proxy(req); self.assertTrue(response.accepted)
        end=rospy.Time.now()+rospy.Duration(3)
        while rospy.Time.now()<end and not any(m.status=='COMPLETED' for m in messages): rospy.sleep(.02)
        self.assertTrue(any(m.status=='ACCEPTED' for m in messages)); self.assertTrue(any(m.status=='COMPLETED' for m in messages))
        end=UavSafetyLeaseRequest('1.0','END','rostest','epoch-rostest',2,0.0,'A01','UAV1','complete')
        self.assertTrue(lease(end).accepted)
if __name__=='__main__': rospy.init_node('test_uav_executor_ros'); rostest.rosrun('swarm_uav_executor','uav_executor_ros',RosTest)
