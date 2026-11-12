"""Pure mission state machine used by the ROS node and offline tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class MissionState(str, Enum):
    INIT = "INIT"
    GROUND_SCAN = "GROUND_SCAN"
    AERIAL_SCAN = "AERIAL_SCAN"
    RETURN = "RETURN"
    COMPLETE = "COMPLETE"
    FAILSAFE = "FAILSAFE"


@dataclass
class InspectionFSM:
    phase_timeout_s: float = 60.0
    state: MissionState = MissionState.INIT
    phase_started_at: float | None = None
    fault_reason: str = ""
    transitions: list[tuple[MissionState, MissionState, float]] = field(default_factory=list)

    def _transition(self, state: MissionState, now: float) -> None:
        if self.state is state:
            return
        previous = self.state
        self.state = state
        self.phase_started_at = now
        self.transitions.append((previous, state, now))

    def _failsafe(self, reason: str, now: float) -> None:
        self.fault_reason = reason
        self._transition(MissionState.FAILSAFE, now)

    def tick(
        self,
        now: float,
        *,
        ready: bool,
        pose_valid: bool,
        control_ok: bool,
    ) -> MissionState:
        """Advance safety checks and start the mission when all prerequisites exist."""
        if self.state in (MissionState.COMPLETE, MissionState.FAILSAFE):
            return self.state
        # During startup, the first timer callback can precede odometry.  Keep
        # the robot stopped in INIT until a fresh pose is available, even when
        # the other readiness signal has already arrived.  A prolonged gap is
        # still a safety fault rather than an endless wait.
        if self.state is MissionState.INIT:
            if self.phase_started_at is None:
                self.phase_started_at = now
            if now - self.phase_started_at > self.phase_timeout_s:
                self._failsafe("timeout:init", now)
                return self.state
            if not ready or not pose_valid:
                return self.state
        if not pose_valid:
            self._failsafe("pose_unavailable", now)
            return self.state
        if not control_ok:
            self._failsafe("control_unavailable", now)
            return self.state
        if self.phase_started_at is None:
            self.phase_started_at = now
        if now - self.phase_started_at > self.phase_timeout_s:
            self._failsafe(f"timeout:{self.state.value.lower()}", now)
            return self.state
        if self.state is MissionState.INIT and ready:
            self._transition(MissionState.GROUND_SCAN, now)
        return self.state

    def mark_ground_complete(self, now: float) -> MissionState:
        if self.state is MissionState.GROUND_SCAN:
            self._transition(MissionState.AERIAL_SCAN, now)
        return self.state

    def mark_aerial_complete(self, now: float) -> MissionState:
        if self.state is MissionState.AERIAL_SCAN:
            self._transition(MissionState.RETURN, now)
        return self.state

    def mark_return_complete(self, now: float) -> MissionState:
        if self.state is MissionState.RETURN:
            self._transition(MissionState.COMPLETE, now)
        return self.state

    def fail(self, reason: str, now: float) -> MissionState:
        self._failsafe(reason, now)
        return self.state

