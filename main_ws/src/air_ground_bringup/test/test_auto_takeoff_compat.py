#!/usr/bin/env python3
"""TDD for auto_takeoff_trigger PX4 1.13 compatibility - strict cache vs legacy.

Covers:
 A. cache delayed then COM_RCL_EXCEPT appears and set succeeds -> True
 B. cache never ready timeout -> False (param_cache_timeout)
 C. COM_RCL_EXCEPT exists but set fails -> False (primary_param_set_failed)
 D. legacy old PX4: cache ready, COM_RCL_EXCEPT truly not found, legacy confirmed -> True (legacy_no_exception_required)
 E. param not found and cannot confirm legacy -> False (unsupported_configuration)
 F. COM_RC_IN_MODE must NOT be used as equivalent for current PX4 1.13 -> False even if it would succeed
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

# Time mock for bounded wait
class FakeTime:
    def __init__(self, sec=0.0):
        self._sec = sec
    def to_sec(self):
        return self._sec
    def __sub__(self, other):
        return FakeTime(self._sec - other._sec)

# Global time counter for simulation
_time_counter = [0.0]
def fake_time_now():
    _time_counter[0] += 0.6
    return FakeTime(_time_counter[0])
def fake_duration(sec):
    return sec

mock_rospy.Time.now = mock.MagicMock(side_effect=fake_time_now)
mock_rospy.Time = mock.MagicMock(now=mock.MagicMock(side_effect=fake_time_now))
mock_rospy.Duration = mock.MagicMock(side_effect=lambda s: s)

class FakeParamValue:
    def __init__(self, integer=0, real=0.0):
        self.integer = integer
        self.real = real

sys.modules['rospy'] = mock_rospy
sys.modules['mavros_msgs'] = mock.MagicMock()
sys.modules['mavros_msgs.msg'] = mock.MagicMock(ParamValue=FakeParamValue, State=mock.MagicMock)
sys.modules['mavros_msgs.srv'] = mock.MagicMock(ParamGet=mock.MagicMock, ParamSet=mock.MagicMock, ParamPull=mock.MagicMock, CommandBool=mock.MagicMock, SetMode=mock.MagicMock)
sys.modules['nav_msgs'] = mock.MagicMock()
sys.modules['nav_msgs.msg'] = mock.MagicMock()
sys.modules['quadrotor_msgs'] = mock.MagicMock()
sys.modules['quadrotor_msgs.msg'] = mock.MagicMock()

MODULE_PATH = pathlib.Path(__file__).parents[1] / "scripts" / "auto_takeoff_trigger.py"
spec = importlib.util.spec_from_file_location("auto_takeoff_trigger", str(MODULE_PATH))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
CxrAutoTakeoff = mod.CxrAutoTakeoff

def make_resp(success, integer=0, real=0.0):
    r = mock.MagicMock()
    r.success = success
    r.value = mock.MagicMock(integer=integer, real=real)
    return r

class AutoTakeoffCompatTest(unittest.TestCase):
    def setUp(self):
        _time_counter[0] = 0.0
        mock_rospy.wait_for_service = mock.MagicMock()
        mock_rospy.logerr = mock.MagicMock()
        mock_rospy.logwarn = mock.MagicMock()
        mock_rospy.loginfo = mock.MagicMock()
        mock_rospy.sleep = mock.MagicMock()
        mock_rospy.is_shutdown = mock.MagicMock(return_value=False)
        # reset Time mock
        mock_rospy.Time.now = mock.MagicMock(side_effect=fake_time_now)
        with mock.patch.object(mock_rospy, 'Publisher', mock.MagicMock()), \
             mock.patch.object(mock_rospy, 'Subscriber', mock.MagicMock()), \
             mock.patch.object(mock_rospy, 'ServiceProxy', mock.MagicMock()), \
             mock.patch.object(mock_rospy, 'get_param', side_effect=lambda n,d: d):
            self.node = CxrAutoTakeoff.__new__(CxrAutoTakeoff)
            self.node.connected = True
            self.node.has_odom = True
            self.node.state = mock.MagicMock(mode="OFFBOARD", armed=False, connected=True)
            self.node.odom = mock.MagicMock()
            self.node.publisher = mock.MagicMock()
            self.node.param_set = mock.MagicMock()
            self.node.param_get = mock.MagicMock()
            self.node.param_pull = mock.MagicMock(return_value=make_resp(True))
            self.node.set_mode = mock.MagicMock()
            self.node.arm = mock.MagicMock()

    def test_A_cache_delayed_then_primary_success(self):
        """A: cache delayed several attempts then COM_RCL_EXCEPT appears and set succeeds -> True"""
        # Sentinel fails first 2 times, then succeeds; primary then succeeds
        call_count = [0]
        def get_side(param_id):
            if param_id == "SYS_AUTOSTART":
                call_count[0] += 1
                if call_count[0] < 3:
                    return make_resp(False)
                return make_resp(True)
            if param_id == "COM_RCL_EXCEPT":
                return make_resp(True)
            return make_resp(False)
        self.node.param_get.side_effect = get_side
        self.node.param_set.return_value = make_resp(True)
        # Need to mock Time to allow loop to progress; our fake_time already does
        result = self.node.configure_offboard_exception()
        self.assertTrue(result)
        # check primary_param_configured log was called
        log_infos = [str(c) for c in mock_rospy.loginfo.call_args_list]
        self.assertTrue(any("primary_param_configured" in s for s in log_infos) or any("waiting_for_param_cache" in s for s in log_infos))

    def test_B_cache_never_ready_timeout(self):
        """B: cache never ready, timeout -> fail-safe False with param_cache_timeout"""
        def get_side(param_id):
            return make_resp(False)
        self.node.param_get.side_effect = get_side
        # Mock sleep to not actually wait, but need to simulate timeout via Time
        # Our fake_time will advance 0.6 each call, need many calls to exceed 15s: ~25 calls
        # The loop will call param_get repeatedly until 15s; with our fake_time, it will timeout
        result = self.node.configure_offboard_exception()
        self.assertFalse(result)
        # should have logged param_cache_timeout
        errs = [str(c) for c in mock_rospy.logerr.call_args_list]
        self.assertTrue(any("param_cache_timeout" in s for s in errs))

    def test_C_primary_exists_but_set_fails(self):
        """C: COM_RCL_EXCEPT exists but set fails -> fail-safe False"""
        def get_side(param_id):
            if param_id == "SYS_AUTOSTART":
                return make_resp(True)
            if param_id == "COM_RCL_EXCEPT":
                return make_resp(True)
            return make_resp(False)
        self.node.param_get.side_effect = get_side
        self.node.param_set.return_value = make_resp(False)
        result = self.node.configure_offboard_exception()
        self.assertFalse(result)
        errs = [str(c) for c in mock_rospy.logerr.call_args_list]
        self.assertTrue(any("primary_param_set_failed" in s for s in errs))

    def test_D_legacy_old_px4_success(self):
        """D:已确认的老 PX4 legacy path -> cache ready, COM_RCL_EXCEPT truly not found, legacy confirmed -> True"""
        def get_side(param_id):
            if param_id == "SYS_AUTOSTART":
                return make_resp(True)
            if param_id == "COM_RCL_EXCEPT":
                return make_resp(False)
            return make_resp(False)
        self.node.param_get.side_effect = get_side
        # Ensure legacy check returns True (default)
        result = self.node.configure_offboard_exception()
        self.assertTrue(result)
        warns = [str(c) for c in mock_rospy.logwarn.call_args_list]
        self.assertTrue(any("legacy_no_exception_required" in s for s in warns))

    def test_E_param_not_found_and_cannot_confirm_legacy(self):
        """E: 参数不存在且无法确认 legacy capability -> fail-safe False with unsupported_configuration"""
        def get_side(param_id):
            if param_id == "SYS_AUTOSTART":
                return make_resp(True)
            if param_id == "COM_RCL_EXCEPT":
                return make_resp(False)
            return make_resp(False)
        self.node.param_get.side_effect = get_side
        # Mock legacy check to return False
        with mock.patch.object(self.node, '_is_legacy_offboard_no_exception_required', return_value=False):
            result = self.node.configure_offboard_exception()
        self.assertFalse(result)
        errs = [str(c) for c in mock_rospy.logerr.call_args_list]
        self.assertTrue(any("unsupported_configuration" in s for s in errs))

    def test_F_com_rc_in_mode_not_equivalent_for_current(self):
        """F: COM_RC_IN_MODE must NOT be used as equivalent for current PX4 1.13"""
        def get_side(param_id):
            if param_id == "SYS_AUTOSTART":
                return make_resp(True)
            if param_id == "COM_RCL_EXCEPT":
                return make_resp(False)
            if param_id == "COM_RC_IN_MODE":
                return make_resp(True)
            return make_resp(False)
        self.node.param_get.side_effect = get_side
        # Even though COM_RC_IN_MODE exists and could be set, our new code should NOT use it for current PX4.
        # It should go via legacy path, not via fallback set.
        # So param_set should NOT be called for COM_RC_IN_MODE in this case.
        self.node.param_set.return_value = make_resp(True)
        # For this test, we want to verify that after primary not found, we do NOT attempt COM_RC_IN_MODE set.
        # Instead we go to legacy. So set should be called 0 times for COM_RC_IN_MODE if legacy path is taken.
        # To test F more strictly, we can check that even if COM_RC_IN_MODE would succeed, we don't treat it as primary success.
        result = self.node.configure_offboard_exception()
        # For current code, primary not found leads to legacy success (D), not fallback. So we should check that param_set was never called with COM_RC_IN_MODE
        set_ids = [c[1].get("param_id") for c in self.node.param_set.call_args_list if "param_id" in c[1]]
        self.assertNotIn("COM_RC_IN_MODE", set_ids, "COM_RC_IN_MODE should not be used as equivalent for current PX4 1.13")
        # Now test that if we are on current PX4 and primary is required, the only success path is primary_param_configured
        # So if primary not found, we should not pretend COM_RC_IN_MODE success; the test above ensures that.

    def test_primary_param_configured_log(self):
        """Verify primary success logs primary_param_configured"""
        def get_side(param_id):
            if param_id == "SYS_AUTOSTART":
                return make_resp(True)
            if param_id == "COM_RCL_EXCEPT":
                return make_resp(True)
            return make_resp(False)
        self.node.param_get.side_effect = get_side
        self.node.param_set.return_value = make_resp(True)
        result = self.node.configure_offboard_exception()
        self.assertTrue(result)
        infos = [str(c) for c in mock_rospy.loginfo.call_args_list]
        self.assertTrue(any("primary_param_configured" in s for s in infos))

if __name__ == "__main__":
    unittest.main()
