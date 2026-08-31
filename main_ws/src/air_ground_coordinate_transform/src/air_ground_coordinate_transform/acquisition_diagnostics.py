"""Read-only observability for visual-registration acquisition pipelines.

The helper only counts outcomes that the surrounding pipeline already
computed; it never changes acceptance decisions.
"""

ACCEPTED = "accepted"

REASON_NO_ODOM = "no_odom"
REASON_STAMP_ZERO = "stamp_zero"
REASON_ODOM_BRACKET = "odom_bracket"
REASON_BELOW_HEIGHT = "below_height"
REASON_UAV_FAST = "uav_fast"
REASON_UAV_YAW_FAST = "uav_yaw_fast"
REASON_UGV_FAST = "ugv_fast"
REASON_INTERPOLATION = "interpolation"

KNOWN_REASONS = (
    REASON_NO_ODOM,
    REASON_STAMP_ZERO,
    REASON_ODOM_BRACKET,
    REASON_BELOW_HEIGHT,
    REASON_UAV_FAST,
    REASON_UAV_YAW_FAST,
    REASON_UGV_FAST,
    REASON_INTERPOLATION,
)


class AcquisitionDiagnostics:
    """Tally why nadir observations did (or did not) become samples."""

    def __init__(self, throttle_seconds=5.0):
        throttle_seconds = float(throttle_seconds)
        if not throttle_seconds > 0.0:
            raise ValueError("throttle_seconds must be positive")
        self._throttle_seconds = throttle_seconds
        self.received = 0
        self.sampled = 0
        self.drops = {reason: 0 for reason in KNOWN_REASONS}
        self.last_drop = None
        self._last_report_stamp = None

    def observe(self):
        """Record one candidate observation entering the sample gates."""
        self.received += 1

    def drop(self, reason):
        if reason not in KNOWN_REASONS:
            raise ValueError("unknown acquisition drop reason: {}".format(reason))
        self.drops[reason] += 1
        self.last_drop = reason

    def accept(self):
        self.sampled += 1

    def summary(self):
        total_drops = sum(self.drops.values())
        active = [
            "{}={}".format(reason, count)
            for reason, count in sorted(self.drops.items())
            if count
        ]
        parts = ["received={}".format(self.received)]
        if active:
            parts.append("dropped={}".format(total_drops))
            parts.extend(active)
        parts.append("sampled={}".format(self.sampled))
        if self.last_drop is not None:
            parts.append("last_drop={}".format(self.last_drop))
        return "acquisition observations: {}".format(", ".join(parts))

    def should_report(self, now):
        """True at most once per throttle window once any activity exists."""
        if self.received == 0:
            return False
        if (
            self._last_report_stamp is not None
            and now - self._last_report_stamp < self._throttle_seconds
        ):
            return False
        self._last_report_stamp = now
        return True
