#!/usr/bin/env python3
import threading, time, unittest
from swarm_uav_executor.drivers.mock import MockMotionDriver
from swarm_uav_executor.models import MotionGoal
class MockDriverTest(unittest.TestCase):
    def test_cancel(self):
        driver=MockMotionDriver(result_delay_s=1); event=threading.Event(); event.set()
        result=driver.start_move_to(MotionGoal(0,0,0,0),event,time.time()+2)
        self.assertEqual(result.error_code,"COMMAND_HELD")
    def test_failure_injection(self):
        result=MockMotionDriver(0,False,"TEST_FAILED","bad").start_move_to(MotionGoal(0,0,0,0),threading.Event(),time.time()+1)
        self.assertFalse(result.success); self.assertEqual(result.error_code,"TEST_FAILED")
if __name__ == "__main__": unittest.main()
