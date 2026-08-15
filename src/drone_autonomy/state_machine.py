"""Landing steps with clear safe exits."""

from dataclasses import dataclass

from .interfaces import TargetProvider, VehicleInterface, VelocitySetpoint
from .models import LandingState, VisualTarget


ALLOWED_TRANSITIONS: dict[LandingState, set[LandingState]] = {
    LandingState.IDLE: {LandingState.TAKEOFF, LandingState.HOLD, LandingState.ABORT},
    LandingState.TAKEOFF: {LandingState.SEARCH, LandingState.HOLD, LandingState.ABORT},
    LandingState.SEARCH: {LandingState.TARGET_ACQUIRED, LandingState.HOLD, LandingState.ABORT},
    LandingState.TARGET_ACQUIRED: {LandingState.SEARCH, LandingState.ALIGN, LandingState.HOLD, LandingState.ABORT},
    LandingState.ALIGN: {LandingState.DESCEND, LandingState.TARGET_LOST, LandingState.HOLD, LandingState.ABORT},
    LandingState.DESCEND: {LandingState.TARGET_LOST, LandingState.LAND, LandingState.HOLD, LandingState.ABORT},
    LandingState.TARGET_LOST: {LandingState.ALIGN, LandingState.HOLD, LandingState.ABORT},
    LandingState.HOLD: {LandingState.ABORT},
    LandingState.ABORT: {LandingState.HOLD},
    LandingState.LAND: {LandingState.HOLD, LandingState.ABORT},
}


@dataclass(frozen=True)
class LandingConfig:
    takeoff_altitude_m: float = 2.0
    takeoff_settle_s: float = 2.0
    takeoff_timeout_s: float = 15.0
    search_timeout_s: float = 30.0
    acquisition_dwell_s: float = 0.5
    acquisition_timeout_s: float = 3.0
    alignment_dwell_s: float = 1.0
    alignment_timeout_s: float = 15.0
    target_reacquire_timeout_s: float = 2.0
    descent_timeout_s: float = 30.0
    final_land_altitude_m: float = 0.30
    center_error_limit: float = 0.10
    lateral_gain: float = 0.10
    max_image_speed: float = 0.15
    descent_rate_mps: float = 0.10


class LandingStateMachine:
    """Uses fresh targets and sends only small, high-level requests."""

    def __init__(self, target_provider: TargetProvider, vehicle: VehicleInterface, config: LandingConfig = LandingConfig()):
        self.target_provider = target_provider
        self.vehicle = vehicle
        self.config = config
        self.state = LandingState.IDLE
        self.state_since_s = 0.0
        self._acquired_since_s: float | None = None
        self._centered_since_s: float | None = None
        self.transitions: list[tuple[LandingState, LandingState, str]] = []

    def start(self, now_s: float) -> None:
        if self.state != LandingState.IDLE:
            raise RuntimeError("landing autonomy can only start from IDLE")
        self._transition(LandingState.TAKEOFF, now_s, "operator_start")

    def update(self, now_s: float, *, manual_override: bool = False, abort: bool = False) -> LandingState:
        if manual_override:
            if self.state != LandingState.HOLD:
                self._transition(LandingState.HOLD, now_s, "manual_override")
            self.vehicle.hold("manual_override")
            return self.state
        if abort and self.state != LandingState.ABORT:
            self._transition(LandingState.ABORT, now_s, "operator_abort")

        target = self.target_provider.latest_target()
        usable = target is not None and target.usable
        elapsed = now_s - self.state_since_s

        if self.state == LandingState.IDLE:
            return self.state
        if self.state == LandingState.TAKEOFF:
            if elapsed > self.config.takeoff_timeout_s:
                self._transition(LandingState.ABORT, now_s, "takeoff_timeout")
            elif elapsed >= self.config.takeoff_settle_s:
                self._transition(LandingState.SEARCH, now_s, "takeoff_settled")
        elif self.state == LandingState.SEARCH:
            self.vehicle.hold("search_no_pattern_configured")
            if usable:
                self._acquired_since_s = now_s
                self._transition(LandingState.TARGET_ACQUIRED, now_s, "fresh_stable_target")
            elif elapsed > self.config.search_timeout_s:
                self._transition(LandingState.ABORT, now_s, "search_timeout")
        elif self.state == LandingState.TARGET_ACQUIRED:
            if not usable:
                self._transition(LandingState.SEARCH, now_s, "acquisition_lost")
            elif self._acquired_since_s is not None and now_s - self._acquired_since_s >= self.config.acquisition_dwell_s:
                self._transition(LandingState.ALIGN, now_s, "acquisition_confirmed")
            elif elapsed > self.config.acquisition_timeout_s:
                self._transition(LandingState.SEARCH, now_s, "acquisition_timeout")
        elif self.state == LandingState.ALIGN:
            if not usable:
                self._transition(LandingState.TARGET_LOST, now_s, "visual_target_lost")
            else:
                self._guide_laterally(target)
                if self._centered(target):
                    self._centered_since_s = self._centered_since_s or now_s
                    if now_s - self._centered_since_s >= self.config.alignment_dwell_s:
                        self._transition(LandingState.DESCEND, now_s, "alignment_confirmed")
                else:
                    self._centered_since_s = None
                if elapsed > self.config.alignment_timeout_s:
                    self._transition(LandingState.TARGET_LOST, now_s, "alignment_timeout")
        elif self.state == LandingState.DESCEND:
            if not usable:
                self._transition(LandingState.TARGET_LOST, now_s, "target_lost_during_descent")
            elif elapsed > self.config.descent_timeout_s:
                self._transition(LandingState.TARGET_LOST, now_s, "descent_timeout")
            elif self._centered(target) and self.vehicle.telemetry().altitude_m <= self.config.final_land_altitude_m:
                self._transition(LandingState.LAND, now_s, "low_altitude_centered")
            else:
                self._guide_laterally(target)
                self.vehicle.descend(self.config.descent_rate_mps)
        elif self.state == LandingState.TARGET_LOST:
            self.vehicle.hold("target_lost")
            if usable:
                self._centered_since_s = None
                self._transition(LandingState.ALIGN, now_s, "target_reacquired")
            elif elapsed > self.config.target_reacquire_timeout_s:
                self._transition(LandingState.ABORT, now_s, "target_reacquire_timeout")
        elif self.state == LandingState.HOLD:
            self.vehicle.hold("manual_hold")
        elif self.state == LandingState.ABORT:
            self.vehicle.hold("abort")
        elif self.state == LandingState.LAND:
            self.vehicle.land()
        return self.state

    def _transition(self, state: LandingState, now_s: float, reason: str) -> None:
        old = self.state
        if state not in ALLOWED_TRANSITIONS[old]:
            raise RuntimeError(f"unsafe transition refused: {old.value} -> {state.value}")
        self.state = state
        self.state_since_s = now_s
        self.transitions.append((old, state, reason))
        if state == LandingState.TAKEOFF:
            self.vehicle.takeoff(self.config.takeoff_altitude_m)
        elif state in (LandingState.TARGET_LOST, LandingState.HOLD, LandingState.ABORT):
            self.vehicle.hold(reason)
        elif state == LandingState.LAND:
            self.vehicle.land()

    def _guide_laterally(self, target: VisualTarget) -> None:
        cap = self.config.max_image_speed
        x = max(-cap, min(cap, target.horizontal_error * self.config.lateral_gain))
        y = max(-cap, min(cap, target.vertical_error * self.config.lateral_gain))
        self.vehicle.velocity(VelocitySetpoint(x, y))

    def _centered(self, target: VisualTarget) -> bool:
        limit = self.config.center_error_limit
        return abs(target.horizontal_error) <= limit and abs(target.vertical_error) <= limit
