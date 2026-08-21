"""Mission steps for GPS travel, obstacle checks, marker landing, and safe exits."""

from dataclasses import dataclass
from math import asin, atan2, cos, degrees, isfinite, radians, sin, sqrt

from .interfaces import NavigationProvider, TargetProvider, VehicleInterface, VelocitySetpoint
from .models import GpsPosition, LandingState, NavigationReadings, VisualTarget


ALLOWED_TRANSITIONS: dict[LandingState, set[LandingState]] = {
    LandingState.IDLE: {LandingState.PREFLIGHT, LandingState.TAKEOFF, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.PREFLIGHT: {LandingState.TAKEOFF, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.TAKEOFF: {LandingState.GPS_NAVIGATE, LandingState.SEARCH, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.GPS_NAVIGATE: {LandingState.YAW_SCAN, LandingState.SEARCH_MOVE, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.YAW_SCAN: {LandingState.GPS_NAVIGATE, LandingState.SEARCH_MOVE, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.SEARCH_MOVE: {LandingState.YAW_SCAN, LandingState.SEARCH, LandingState.TARGET_ACQUIRED, LandingState.HOLD, LandingState.RETURN_HOME},
    LandingState.SEARCH: {LandingState.SEARCH_MOVE, LandingState.TARGET_ACQUIRED, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.TARGET_ACQUIRED: {LandingState.SEARCH, LandingState.ALIGN, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.ALIGN: {LandingState.DESCEND, LandingState.DROP_READY, LandingState.TARGET_LOST, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.DESCEND: {LandingState.TARGET_LOST, LandingState.LAND, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.DROP_READY: {LandingState.ALIGN, LandingState.DROP_PAYLOAD, LandingState.TARGET_LOST, LandingState.HOLD, LandingState.RETURN_HOME},
    LandingState.DROP_PAYLOAD: {LandingState.RETURN_HOME, LandingState.HOLD},
    LandingState.TARGET_LOST: {LandingState.ALIGN, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.HOLD: {LandingState.ABORT},
    LandingState.ABORT: {LandingState.HOLD},
    LandingState.LAND: {LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME},
    LandingState.RETURN_HOME: {LandingState.HOLD},
}
"""The only state changes the program is allowed to make.

Every state that can still make a decision may reach ``RETURN_HOME``, because
any mission health check can decide to come home from any of them. Leaving one
of them out does not make the aircraft safer: it makes the safety check itself
raise instead of acting.
"""


@dataclass(frozen=True)
class LandingConfig:
    """Mission limits and waits. Defaults are only for mock tests.

    The payload mission is GPS target → 5 m grid search → marker alignment →
    servo release → return home. All settings must be measured and tested on
    the actual aircraft before real use.
    """

    target_position: GpsPosition | None = None
    arrival_radius_m: float = 3.0
    heading_tolerance_deg: float = 10.0
    yaw_scan_offsets_deg: tuple[float, ...] = (0.0, -30.0, 30.0, -60.0, 60.0)
    side_clearance_m: float = 2.0
    top_clearance_m: float = 1.0
    landing_distance_m: float = 0.30
    forward_speed_mps: float = 0.25
    forward_step_s: float = 0.50
    search_altitude_m: float = 5.0
    search_area_side_m: float = 10.0
    search_grid_spacing_m: float = 2.0
    search_point_dwell_s: float = 2.0
    payload_alignment_dwell_s: float = 1.0
    return_home_battery_percent: float = 40.0
    takeoff_altitude_m: float = 2.0
    takeoff_settle_s: float = 2.0
    takeoff_timeout_s: float = 15.0
    search_timeout_s: float = 600.0
    mission_timeout_s: float | None = None
    require_armed: bool = True
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
    """Makes safe mock requests for GPS travel, payload drop, and return home."""

    def __init__(self, target_provider: TargetProvider, vehicle: VehicleInterface, config: LandingConfig = LandingConfig(), navigation_provider: NavigationProvider | None = None):
        """Create the mission controller."""
        self.target_provider = target_provider
        self.navigation_provider = navigation_provider
        self.vehicle = vehicle
        self.config = config
        self.state = LandingState.IDLE
        self.state_since_s = 0.0
        self._acquired_since_s: float | None = None
        self._centered_since_s: float | None = None
        self._scan_index = 0
        self._scan_return_state = LandingState.GPS_NAVIGATE
        self._temporary_heading_deg: float | None = None
        self._search_points: list[GpsPosition] = []
        self._search_index = 0
        self._active_navigation_goal: GpsPosition | None = None
        self._payload_centered_since_s: float | None = None
        self._payload_release_requested = False
        self._mission_started_s: float | None = None
        self._search_phase_since_s: float | None = None
        self.transitions: list[tuple[LandingState, LandingState, str]] = []

    def start(self, now_s: float) -> None:
        """Start the mission from IDLE."""
        if self.state != LandingState.IDLE:
            raise RuntimeError("autonomy can only start from IDLE")
        if self.config.target_position is not None and self.navigation_provider is None:
            raise RuntimeError("a GPS target requires a navigation provider")
        self._mission_started_s = now_s
        first = LandingState.PREFLIGHT if self.config.target_position else LandingState.TAKEOFF
        self._transition(first, now_s, "operator_start")

    def update(self, now_s: float, *, manual_override: bool = False, abort: bool = False) -> LandingState:
        """Run one decision cycle and return the resulting state."""
        if manual_override:
            if self.state != LandingState.HOLD:
                self._transition(LandingState.HOLD, now_s, "manual_override")
            self.vehicle.hold("manual_override")
            return self.state
        if abort and self.state not in (LandingState.ABORT, LandingState.RETURN_HOME):
            self._return_or_abort(now_s, "operator_abort")

        telemetry = self.vehicle.telemetry()
        if self.state not in (LandingState.IDLE, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME) and telemetry.failsafe_active:
            self._transition(LandingState.HOLD, now_s, "pixhawk_failsafe_active")
            return self.state
        if self.state not in (LandingState.IDLE, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME) and not telemetry.telemetry_fresh:
            self._return_or_abort(now_s, "pixhawk_telemetry_stale")
            return self.state
        if self.state not in (LandingState.IDLE, LandingState.PREFLIGHT, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME) and telemetry.battery_remaining_percent <= self.config.return_home_battery_percent:
            self._return_or_abort(now_s, "battery_low")
            return self.state

        if (
            self.config.mission_timeout_s is not None
            and self._mission_started_s is not None
            and self.state not in (LandingState.IDLE, LandingState.HOLD, LandingState.ABORT, LandingState.RETURN_HOME)
            and now_s - self._mission_started_s > self.config.mission_timeout_s
        ):
            self._return_or_abort(now_s, "mission_timeout")
            return self.state

        if self._search_expired(now_s):
            self._return_or_abort(now_s, "search_timeout")
            return self.state

        target = self.target_provider.latest_target()
        visual_ok = target is not None and target.usable
        nav = self._navigation()
        elapsed = now_s - self.state_since_s

        if self.state == LandingState.PREFLIGHT:
            if not telemetry.telemetry_fresh or not telemetry.position_hold_ready or telemetry.failsafe_active:
                self._transition(LandingState.HOLD, now_s, "preflight_vehicle_not_ready")
            elif self.config.require_armed and not telemetry.armed:
                self._transition(LandingState.HOLD, now_s, "preflight_not_armed")
            elif telemetry.battery_remaining_percent <= self.config.return_home_battery_percent:
                self._transition(LandingState.HOLD, now_s, "preflight_battery_low")
            elif not self._navigation_ok(nav):
                self._transition(LandingState.HOLD, now_s, "preflight_navigation_unavailable")
            elif not self._top_clear(nav):
                self._transition(LandingState.HOLD, now_s, "preflight_top_blocked")
            else:
                self._transition(LandingState.TAKEOFF, now_s, "preflight_passed")
        elif self.state == LandingState.TAKEOFF:
            if self.navigation_provider and not self._top_clear(nav):
                self._transition(LandingState.HOLD, now_s, "top_sensor_not_clear")
            elif elapsed > self.config.takeoff_timeout_s:
                self._return_or_abort(now_s, "takeoff_timeout")
            elif elapsed >= self.config.takeoff_settle_s:
                self._active_navigation_goal = self.config.target_position
                self._transition(LandingState.GPS_NAVIGATE if self.config.target_position else LandingState.SEARCH, now_s, "takeoff_settled")
        elif self.state == LandingState.GPS_NAVIGATE:
            self._navigate(now_s, nav)
        elif self.state == LandingState.YAW_SCAN:
            self._scan(now_s, nav)
        elif self.state == LandingState.SEARCH_MOVE:
            if visual_ok:
                self._acquired_since_s = now_s
                self._transition(LandingState.TARGET_ACQUIRED, now_s, "marker_seen_during_search")
            else:
                self._navigate(now_s, nav)
        elif self.state == LandingState.SEARCH:
            self.vehicle.hold("search_waiting_for_marker")
            if visual_ok:
                self._acquired_since_s = now_s
                self._transition(LandingState.TARGET_ACQUIRED, now_s, "fresh_stable_target")
            elif self.config.target_position is None and elapsed > self.config.search_timeout_s:
                self._return_or_abort(now_s, "search_timeout")
            elif self.config.target_position is not None and elapsed >= self.config.search_point_dwell_s:
                self._next_search_point_or_return(now_s)
        elif self.state == LandingState.TARGET_ACQUIRED:
            if not visual_ok:
                self._transition(LandingState.SEARCH, now_s, "acquisition_lost")
            elif self._acquired_since_s is not None and now_s - self._acquired_since_s >= self.config.acquisition_dwell_s:
                self._transition(LandingState.ALIGN, now_s, "acquisition_confirmed")
            elif elapsed > self.config.acquisition_timeout_s:
                self._return_or_abort(now_s, "acquisition_timeout")
        elif self.state == LandingState.ALIGN:
            if not visual_ok:
                self._transition(LandingState.TARGET_LOST, now_s, "visual_target_lost")
            else:
                self._guide(target)
                if self._centered(target):
                    self._centered_since_s = self._centered_since_s or now_s
                    if now_s - self._centered_since_s >= self.config.alignment_dwell_s:
                        next_state = LandingState.DROP_READY if self.config.target_position else LandingState.DESCEND
                        self._transition(next_state, now_s, "alignment_confirmed")
                else:
                    self._centered_since_s = None
                if elapsed > self.config.alignment_timeout_s:
                    self._return_or_abort(now_s, "alignment_timeout")
        elif self.state == LandingState.DROP_READY:
            self.vehicle.hold("payload_alignment_check")
            if not visual_ok:
                self._transition(LandingState.TARGET_LOST, now_s, "marker_lost_before_drop")
            elif not self._centered(target):
                self._payload_centered_since_s = None
                self._transition(LandingState.ALIGN, now_s, "marker_moved_before_drop")
            else:
                self._payload_centered_since_s = self._payload_centered_since_s or now_s
                if now_s - self._payload_centered_since_s >= self.config.payload_alignment_dwell_s:
                    self._transition(LandingState.DROP_PAYLOAD, now_s, "payload_drop_authorized")
        elif self.state == LandingState.DROP_PAYLOAD:
            if not self._payload_release_requested:
                self.vehicle.release_payload()
                self._payload_release_requested = True
            self._transition(LandingState.RETURN_HOME, now_s, "payload_release_requested")
        elif self.state == LandingState.DESCEND:
            if not visual_ok:
                self._transition(LandingState.TARGET_LOST, now_s, "target_lost_during_descent")
            elif self.navigation_provider and not self._bottom_usable(nav):
                self._transition(LandingState.HOLD, now_s, "bottom_sensor_unavailable")
            elif elapsed > self.config.descent_timeout_s:
                self._transition(LandingState.TARGET_LOST, now_s, "descent_timeout")
            elif self._centered(target) and self._ready_to_land(nav):
                self._transition(LandingState.LAND, now_s, "low_altitude_centered")
            else:
                self._guide(target)
                self.vehicle.descend(self.config.descent_rate_mps)
        elif self.state == LandingState.TARGET_LOST:
            self.vehicle.hold("target_lost")
            if visual_ok:
                self._centered_since_s = None
                self._transition(LandingState.ALIGN, now_s, "target_reacquired")
            elif elapsed > self.config.target_reacquire_timeout_s:
                self._return_or_abort(now_s, "target_reacquire_timeout")
        elif self.state == LandingState.HOLD:
            self.vehicle.hold("hold")
        elif self.state == LandingState.ABORT:
            self.vehicle.hold("abort")
        elif self.state == LandingState.LAND:
            self.vehicle.land()
        return self.state

    def _navigate(self, now_s: float, nav: NavigationReadings | None) -> None:
        if not self._navigation_ok(nav):
            self._transition(LandingState.HOLD, now_s, "navigation_unavailable")
            return
        assert nav and nav.position and nav.heading_deg is not None
        assert self._active_navigation_goal is not None
        if self._distance_to_goal_m(nav.position, self._active_navigation_goal) <= self.config.arrival_radius_m:
            self._temporary_heading_deg = None
            if self.state == LandingState.GPS_NAVIGATE:
                self._begin_search(now_s)
            else:
                self._transition(LandingState.SEARCH, now_s, "search_point_reached")
            return
        desired = self._temporary_heading_deg
        if desired is None:
            desired = self._bearing_to_goal_deg(nav.position, self._active_navigation_goal)
        if abs(self._heading_error_deg(desired, nav.heading_deg)) > self.config.heading_tolerance_deg:
            self.vehicle.yaw_to(desired)
        elif not self._side_clear(nav):
            self._scan_index = 0
            self._scan_return_state = self.state
            self._temporary_heading_deg = None
            self._transition(LandingState.YAW_SCAN, now_s, "forward_path_blocked")
        else:
            self.vehicle.forward(self.config.forward_speed_mps, self.config.forward_step_s)
            self._temporary_heading_deg = None

    def _scan(self, now_s: float, nav: NavigationReadings | None) -> None:
        if not self._navigation_ok(nav) or not self._side_usable(nav):
            self._return_or_abort(now_s, "scan_sensor_unavailable")
            return
        assert nav and nav.position and nav.heading_deg is not None
        if self._scan_index >= len(self.config.yaw_scan_offsets_deg):
            self._return_or_abort(now_s, "no_clear_scan_direction")
            return
        assert self._active_navigation_goal is not None
        candidate = self._normalize_heading(self._bearing_to_goal_deg(nav.position, self._active_navigation_goal) + self.config.yaw_scan_offsets_deg[self._scan_index])
        if abs(self._heading_error_deg(candidate, nav.heading_deg)) > self.config.heading_tolerance_deg:
            self.vehicle.yaw_to(candidate)
        elif self._side_clear(nav):
            self._temporary_heading_deg = candidate
            self._transition(self._scan_return_state, now_s, "clear_scan_direction")
        else:
            self._scan_index += 1

    def _transition(self, state: LandingState, now_s: float, reason: str) -> None:
        if state not in ALLOWED_TRANSITIONS[self.state]:
            raise RuntimeError(f"unsafe transition refused: {self.state.value} -> {state.value}")
        self.transitions.append((self.state, state, reason))
        self.state, self.state_since_s = state, now_s
        if state == LandingState.TAKEOFF:
            altitude = self.config.search_altitude_m if self.config.target_position else self.config.takeoff_altitude_m
            self.vehicle.takeoff(altitude)
        elif state in (LandingState.YAW_SCAN, LandingState.TARGET_LOST, LandingState.HOLD, LandingState.ABORT):
            self.vehicle.hold(reason)
        elif state == LandingState.LAND:
            self.vehicle.land()
        elif state == LandingState.RETURN_HOME:
            self.vehicle.return_home(reason)

    def _return_or_abort(self, now_s: float, reason: str) -> None:
        """Return home for a GPS mission; retain ABORT for legacy landing tests."""
        state = LandingState.RETURN_HOME if self.config.target_position is not None else LandingState.ABORT
        self._transition(state, now_s, reason)

    def _begin_search(self, now_s: float) -> None:
        """Create the configured grid around the destination and visit its first point."""
        assert self.config.target_position is not None
        self._search_points = self._make_search_grid(self.config.target_position)
        self._search_index = 0
        self._search_phase_since_s = now_s
        if not self._search_points:
            self._return_or_abort(now_s, "search_grid_empty")
            return
        self._active_navigation_goal = self._search_points[0]
        self._transition(LandingState.SEARCH_MOVE, now_s, "gps_target_reached")

    def _search_expired(self, now_s: float) -> bool:
        """Return True when the whole marker-search phase has run out of time.

        The clock starts when the grid search begins and keeps running across
        every grid point, so it is a real limit on the search as a whole rather
        than a limit on one dwell.
        """
        return (
            self._search_phase_since_s is not None
            and self.state in (LandingState.SEARCH, LandingState.SEARCH_MOVE, LandingState.YAW_SCAN)
            and now_s - self._search_phase_since_s > self.config.search_timeout_s
        )

    def _next_search_point_or_return(self, now_s: float) -> None:
        """Move to the next grid point, or return home when all points were checked."""
        self._search_index += 1
        if self._search_index >= len(self._search_points):
            self._return_or_abort(now_s, "marker_not_found_in_search_area")
            return
        self._active_navigation_goal = self._search_points[self._search_index]
        self._transition(LandingState.SEARCH_MOVE, now_s, "next_search_point")

    def _make_search_grid(self, centre: GpsPosition) -> list[GpsPosition]:
        """Return a serpentine grid centred on the GPS destination.

        This simple local-earth approximation is suitable only for a small
        search area such as 10 m by 10 m. It does not replace full geodesy.
        """
        if self.config.search_area_side_m <= 0 or self.config.search_grid_spacing_m <= 0:
            return []
        half = self.config.search_area_side_m / 2
        offsets: list[float] = []
        offset = -half
        while offset < half:
            offsets.append(offset)
            offset += self.config.search_grid_spacing_m
        offsets.append(half)
        latitude_scale = 111_111.0
        longitude_scale = latitude_scale * cos(radians(centre.latitude_deg))
        points: list[GpsPosition] = []
        for row, north_m in enumerate(offsets):
            east_offsets = offsets if row % 2 == 0 else list(reversed(offsets))
            for east_m in east_offsets:
                points.append(GpsPosition(
                    latitude_deg=centre.latitude_deg + north_m / latitude_scale,
                    longitude_deg=centre.longitude_deg + east_m / longitude_scale,
                ))
        return points

    def _navigation(self) -> NavigationReadings | None:
        return self.navigation_provider.latest_navigation() if self.navigation_provider else None

    @staticmethod
    def _navigation_ok(nav: NavigationReadings | None) -> bool:
        return nav is not None and nav.navigation_usable

    @staticmethod
    def _side_usable(nav: NavigationReadings | None) -> bool:
        return bool(nav and nav.side_fresh and nav.side_distance_m is not None and isfinite(nav.side_distance_m) and nav.side_distance_m >= 0)

    def _side_clear(self, nav: NavigationReadings | None) -> bool:
        return self._side_usable(nav) and nav.side_distance_m >= self.config.side_clearance_m  # type: ignore[union-attr]

    def _top_clear(self, nav: NavigationReadings | None) -> bool:
        return bool(nav and nav.top_fresh and nav.top_distance_m is not None and isfinite(nav.top_distance_m) and nav.top_distance_m >= self.config.top_clearance_m)

    @staticmethod
    def _bottom_usable(nav: NavigationReadings | None) -> bool:
        return bool(nav and nav.bottom_fresh and nav.bottom_distance_m is not None and isfinite(nav.bottom_distance_m) and nav.bottom_distance_m >= 0)

    def _ready_to_land(self, nav: NavigationReadings | None) -> bool:
        if self.navigation_provider:
            return self._bottom_usable(nav) and nav.bottom_distance_m <= self.config.landing_distance_m  # type: ignore[union-attr]
        return self.vehicle.telemetry().altitude_m <= self.config.final_land_altitude_m

    def _distance_to_target_m(self, position: GpsPosition) -> float:
        assert self.config.target_position is not None
        return self._distance_to_goal_m(position, self.config.target_position)

    @staticmethod
    def _distance_to_goal_m(position: GpsPosition, goal: GpsPosition) -> float:
        """Return approximate ground distance between two close GPS positions."""
        radius = 6_371_000.0
        lat1, lon1 = radians(position.latitude_deg), radians(position.longitude_deg)
        lat2, lon2 = radians(goal.latitude_deg), radians(goal.longitude_deg)
        a = sin((lat2 - lat1) / 2) ** 2 + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) ** 2
        return radius * 2 * asin(sqrt(a))

    def _bearing_to_target_deg(self, position: GpsPosition) -> float:
        assert self.config.target_position is not None
        return self._bearing_to_goal_deg(position, self.config.target_position)

    def _bearing_to_goal_deg(self, position: GpsPosition, goal: GpsPosition) -> float:
        """Return compass bearing from a GPS position to a chosen GPS goal."""
        lat1, lat2 = radians(position.latitude_deg), radians(goal.latitude_deg)
        delta_lon = radians(goal.longitude_deg - position.longitude_deg)
        y = sin(delta_lon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
        return self._normalize_heading(degrees(atan2(y, x)))

    @staticmethod
    def _normalize_heading(heading_deg: float) -> float:
        return heading_deg % 360.0

    @staticmethod
    def _heading_error_deg(desired_deg: float, current_deg: float) -> float:
        return (desired_deg - current_deg + 180.0) % 360.0 - 180.0

    def _guide(self, target: VisualTarget) -> None:
        cap = self.config.max_image_speed
        self.vehicle.velocity(VelocitySetpoint(
            max(-cap, min(cap, target.horizontal_error * self.config.lateral_gain)),
            max(-cap, min(cap, target.vertical_error * self.config.lateral_gain)),
        ))

    def _centered(self, target: VisualTarget) -> bool:
        return abs(target.horizontal_error) <= self.config.center_error_limit and abs(target.vertical_error) <= self.config.center_error_limit
