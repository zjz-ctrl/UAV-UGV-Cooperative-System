#!/usr/bin/env python3
"""TDD for OFFBOARD chain fix.

Covers:
 A. No valid setpoint stream -> must not request/claim OFFBOARD success
 B. Stable setpoint stream -> can request OFFBOARD
 C. set_mode temporarily rejected -> bounded retry
 D. OFFBOARD success confirmed before arm
 E. OFFBOARD not success -> must not arm/takeoff
 F. Mode drops out of OFFBOARD after success -> must not pretend success
"""
import unittest
from unittest import mock
import sys
import pathlib
import importlib.util

mock_rospy = mock.MagicMock()
mock_rospy.ROSException = Exception
mock_rospy.ServiceException = Exception
mock_rospy.logerr = mock.MagicMock()
mock_rospy.logwarn = mock.MagicMock()
mock_rospy.loginfo = mock.MagicMock()
mock_rospy.wait_for_service = mock.MagicMock()
mock_rospy.sleep = mock.MagicMock()
mock_rospy.is_shutdown = mock.MagicMock(return_value=False)
class FakeTimeObj:
    def __init__(self, sec=0):
        self.sec = sec
    def to_sec(self):
        return self.sec
    def __add__(self, other):
        # other is Duration mock with sec
        dur = other.sec if hasattr(other, 'sec') else 0.5
        return FakeTimeObj(self.sec + dur)
    def __lt__(self, other):
        return self.sec < other.sec
    def __sub__(self, other):
        return FakeTimeObj(self.sec - other.sec)

class FakeDuration:
    def __init__(self, sec):
        self.sec = sec

mock_rospy.Time.now = mock.MagicMock(side_effect=lambda: FakeTimeObj(0))
mock_rospy.Duration = mock.MagicMock(side_effect=lambda s: FakeDuration(s))

class FakeParamValue:
    def __init__(self, integer=0, real=0.0):
        self.integer = integer
        self.real = real
class FakeRate:
    def __init__(self, hz): pass
    def sleep(self): pass

mock_rospy.Rate = FakeRate

sys.modules['rospy'] = mock_rospy
sys.modules['mavros_msgs'] = mock.MagicMock()
sys.modules['mavros_msgs.msg'] = mock.MagicMock(ParamValue=FakeParamValue, State=mock.MagicMock)
sys.modules['mavros_msgs.srv'] = mock.MagicMock(ParamGet=mock.MagicMock, ParamSet=mock.MagicMock, ParamPull=mock.MagicMock, CommandBool=mock.MagicMock, SetMode=mock.MagicMock)
sys.modules['nav_msgs'] = mock.MagicMock()
sys.modules['nav_msgs.msg'] = mock.MagicMock()
sys.modules['quadrotor_msgs'] = mock.MagicMock()
sys.modules['quadrotor_msgs.msg'] = mock.MagicMock()

MODULE_PATH = pathlib.Path(__file__).parents[1] / "scripts" / "auto_takeoff_trigger.py"
spec = importlib.util.spec_from_file_location("auto_takeoff_trigger_offboard", str(MODULE_PATH))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
CxrAutoTakeoff = mod.CxrAutoTakeoff

def make_state(mode="AUTO.LOITER", armed=False, connected=True, guided=False):
    s = mock.MagicMock()
    s.mode = mode
    s.armed = armed
    s.connected = connected
    s.guided = guided
    return s

def make_setmode_resp(success=True, mode_sent=True):
    r = mock.MagicMock()
    r.success = success
    r.mode_sent = mode_sent
    return r

class OffboardCompatTest(unittest.TestCase):
    def setUp(self):
        mock_rospy.wait_for_service = mock.MagicMock()
        mock_rospy.logerr = mock.MagicMock()
        mock_rospy.logwarn = mock.MagicMock()
        mock_rospy.loginfo = mock.MagicMock()
        mock_rospy.is_shutdown = mock.MagicMock(return_value=False)
        # Mock Time to allow loop progress
        self.time_val = [0.0]
        def now():
            self.time_val[0] += 0.6
            m = mock.MagicMock()
            m.to_sec = lambda: self.time_val[0]
            m.__sub__ = lambda s,o: mock.MagicMock(to_sec=lambda: self.time_val[0]-o.to_sec() if hasattr(o,'to_sec') else 0)
            return m
        # Use simple time mock
        mock_rospy.Time.now = mock.MagicMock(side_effect=lambda: mock.MagicMock(to_sec=lambda: self.time_val[0], __sub__=lambda s,o: mock.MagicMock(to_sec=lambda: 0.6)))
        # Instead patch Duration to return a mock
        mock_rospy.Duration = mock.MagicMock(side_effect=lambda s: mock.MagicMock(to_sec=lambda: s))
        # Make is_shutdown return False except when we want
        with mock.patch.object(mock_rospy, 'Publisher', mock.MagicMock()), \
             mock.patch.object(mock_rospy, 'Subscriber', mock.MagicMock()), \
             mock.patch.object(mock_rospy, 'ServiceProxy', mock.MagicMock()), \
             mock.patch.object(mock_rospy, 'get_param', side_effect=lambda n,d: d):
            self.node = CxrAutoTakeoff.__new__(CxrAutoTakeoff)
            self.node.connected = True
            self.node.has_odom = True
            self.node.state = make_state("AUTO.LOITER", False, True)
            self.node.odom = mock.MagicMock()
            self.node.odom.pose.pose.position.x = -3.0
            self.node.odom.pose.pose.position.y = 0.0
            self.node.odom.pose.pose.position.z = 0.25
            self.node.odom.pose.pose.orientation.x = 0
            self.node.odom.pose.pose.orientation.y = 0
            self.node.odom.pose.pose.orientation.z = 0
            self.node.odom.pose.pose.orientation.w = 1
            self.node.publisher = mock.MagicMock()
            self.node.param_set = mock.MagicMock(return_value=mock.MagicMock(success=True))
            self.node.param_get = mock.MagicMock(return_value=mock.MagicMock(success=True))
            self.node.param_pull = mock.MagicMock(return_value=mock.MagicMock(success=True))
            self.node.set_mode = mock.MagicMock(return_value=make_setmode_resp(True, True))
            self.node.arm = mock.MagicMock(return_value=mock.MagicMock(success=True))
            # Patch publish to not fail
            self.node.publish_command = mock.MagicMock()

    def test_A_no_valid_setpoint_stream_must_not_claim_offboard(self):
        """A: 无有效 setpoint stream 时不得宣称 OFFBOARD 成功"""
        self.node.connected = False
        self.node.has_odom = False
        rate = mock.MagicMock()
        rate.sleep = mock.MagicMock()
        # Mock Time to allow loop to exit quickly
        with mock.patch.object(mock_rospy, 'is_shutdown', return_value=False):
            # Make Time.now return increasing values to avoid infinite loop
            mock_rospy.Time.now = mock.MagicMock(return_value=mock.MagicMock(to_sec=lambda: 0, __sub__=lambda s,o: mock.MagicMock(to_sec=lambda: 1)))
            mock_rospy.Duration = mock.MagicMock(return_value=mock.MagicMock())
            result = self.node.request_offboard(-3,0,0.25,0, rate)
        self.assertFalse(result)
        # Should have logged no valid stream
        logs = [str(c) for c in mock_rospy.logerr.call_args_list]
        self.assertTrue(any("no valid setpoint stream" in s for s in logs))

    def test_B_stable_stream_can_request_offboard(self):
        """B: 有稳定 setpoint stream 后才能请求 OFFBOARD"""
        self.node.connected = True
        self.node.has_odom = True
        self.node.state.mode = "AUTO.LOITER"
        def set_mode_side(base_mode, custom_mode):
            self.node.state.mode = "OFFBOARD"
            return make_setmode_resp(True, True)
        self.node.set_mode.side_effect = set_mode_side
        rate = mock.MagicMock()
        mock_rospy.is_shutdown = mock.MagicMock(return_value=False)
        fake_now = mock.MagicMock()
        fake_now.__lt__ = mock.MagicMock(return_value=False)
        fake_now.__add__ = mock.MagicMock(return_value=fake_now)
        mock_rospy.Time.now = mock.MagicMock(return_value=fake_now)
        mock_rospy.Duration = mock.MagicMock(return_value=fake_now)
        result = self.node.request_offboard(-3,0,0.25,0, rate)
        self.assertTrue(result)

    def test_C_set_mode_temporarily_rejected_bounded_retry(self):
        """C: set_mode 暂时拒绝时 bounded retry"""
        self.node.connected = True
        self.node.has_odom = True
        self.node.state.mode = "AUTO.LOITER"
        calls = [0]
        def side(base_mode, custom_mode):
            calls[0] += 1
            if calls[0] < 3:
                return make_setmode_resp(False, False)
            self.node.state.mode = "OFFBOARD"
            return make_setmode_resp(True, True)
        self.node.set_mode.side_effect = side
        rate = mock.MagicMock()
        mock_rospy.is_shutdown = mock.MagicMock(return_value=False)
        fake_now = mock.MagicMock()
        fake_now.__lt__ = mock.MagicMock(return_value=False)
        fake_now.__add__ = mock.MagicMock(return_value=fake_now)
        mock_rospy.Time.now = mock.MagicMock(return_value=fake_now)
        mock_rospy.Duration = mock.MagicMock(return_value=fake_now)
        result = self.node.request_offboard(-3,0,0.25,0, rate)
        self.assertTrue(result)
        self.assertGreaterEqual(calls[0], 3)

    def test_D_offboard_must_be_confirmed_before_arm(self):
        """D: OFFBOARD 成功确认后才进入 arm"""
        self.node.connected = True
        self.node.has_odom = True
        self.node.state.mode = "OFFBOARD"
        self.node.state.armed = False
        rate = mock.MagicMock()
        mock_rospy.Time.now = mock.MagicMock(return_value=mock.MagicMock(to_sec=lambda: 0))
        mock_rospy.Duration = mock.MagicMock(return_value=mock.MagicMock())
        mock_rospy.is_shutdown = mock.MagicMock(return_value=True)
        # arm service succeeds and then armed True
        def arm_side(val):
            self.node.state.armed = True
            return mock.MagicMock(success=True)
        self.node.arm.side_effect = arm_side
        result = self.node.request_arm(-3,0,0.25,0, rate)
        self.assertTrue(result)

    def test_E_offboard_not_success_must_not_arm(self):
        """E: OFFBOARD 未成功时不得 arm/起飞"""
        self.node.state.mode = "AUTO.LOITER"
        rate = mock.MagicMock()
        mock_rospy.is_shutdown = mock.MagicMock(return_value=True)
        mock_rospy.Time.now = mock.MagicMock(return_value=mock.MagicMock(to_sec=lambda: 0))
        mock_rospy.Duration = mock.MagicMock(return_value=mock.MagicMock())
        result = self.node.request_arm(-3,0,0.25,0, rate)
        self.assertFalse(result)
        # arm should not have been called
        self.node.arm.assert_not_called()

    def test_F_mode_drops_out_after_success_must_not_pretend(self):
        """F: mode 后续掉出 OFFBOARD 时不能继续假装成功"""
        # Simulate run() check after OFFBOARD success then mode drops
        self.node.state.mode = "OFFBOARD"
        # After OFFBOARD, if mode drops to AUTO.LOITER before arm, request_arm should fail
        self.node.state.mode = "AUTO.LOITER"
        rate = mock.MagicMock()
        mock_rospy.is_shutdown = mock.MagicMock(return_value=True)
        mock_rospy.Time.now = mock.MagicMock(return_value=mock.MagicMock(to_sec=lambda: 0))
        mock_rospy.Duration = mock.MagicMock(return_value=mock.MagicMock())
        result = self.node.request_arm(-3,0,0.25,0, rate)
        self.assertFalse(result)

if __name__ == "__main__":
    unittest.main()
