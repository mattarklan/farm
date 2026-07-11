import uuid

from core.models import Event
from core.severity import Severity


class Pipeline:
    def __init__(self, event_engine, confidence_monitor, staleness_monitor, range_monitor, anomaly_engine,
                 rule_engine, goal_engine, constraint_engine, executor, derived_engine,
                 window_store, event_deduplicator):
        self.event_engine = event_engine
        self.confidence_monitor = confidence_monitor
        self.staleness_monitor = staleness_monitor
        self.range_monitor = range_monitor
        self.anomaly_engine = anomaly_engine
        self.rule_engine = rule_engine
        self.goal_engine = goal_engine
        self.constraint_engine = constraint_engine
        self.executor = executor
        self.derived_engine = derived_engine
        self.window_store = window_store
        self.event_deduplicator = event_deduplicator

    def run(self, state):
        for sensor_id, sensor in state["sensors"].items():
            self.window_store.append(sensor_id, sensor.value, sensor.timestamp)

        derived = self.derived_engine.compute(list(state["sensors"].keys()))
        state["derived"] = derived

        # Health checks run FIRST and deliberately BEFORE event_engine/
        # goal_engine -- they can downgrade a sensor's confidence to 0
        # (stuck, stale, out-of-range), and that needs to take effect on
        # THIS cycle's threshold/until checks, not next cycle's.
        events = self.confidence_monitor.process(state, derived)
        events += self.staleness_monitor.process(state)
        events += self.range_monitor.process(state)

        events += self.event_engine.process(state, derived)

        goal_start_actions, goal_stop_actions, goal_events = self.goal_engine.process(
            events, state, derived
        )
        events += goal_events

        events += self.anomaly_engine.process(state, derived)

        instant_actions = self.rule_engine.process(events, state)
        candidate_actions = instant_actions + goal_start_actions

        allowed, blocked, delayed = self.constraint_engine.evaluate(candidate_actions, state, derived)

        blocked_events = [self._blocked_event(b, state) for b in blocked]
        events += blocked_events

        # Goal-stop actions bypass the constraint filter on purpose: a
        # safety/goal-completion stop should never itself be blockable by
        # the same constraint system that might be why it's stopping.
        executable = allowed + goal_stop_actions
        results = self.executor.execute(executable)

        action_by_id = {a.id: a for a in executable}
        failed_events = []
        for r in results:
            action = action_by_id.get(r.action_id)
            if r.success:
                self.anomaly_engine.on_action_executed(action, state)
            else:
                failed_events.append(self._failed_event(r, action, state))
        events += failed_events

        events = self.event_deduplicator.process(events, state["timestamp"])

        return {
            "events": events,
            "derived": derived,
            "actions": candidate_actions,
            "allowed": allowed,
            "blocked": blocked,
            "delayed": delayed,
            "goal_stop_actions": goal_stop_actions,
            "results": results,
        }

    @staticmethod
    def _blocked_event(blocked_entry, state) -> Event:
        action = blocked_entry["action"]
        return Event(
            id=str(uuid.uuid4()),
            event_type="ACTION_BLOCKED",
            asset_id=action.target or "",
            sensor_id="",
            severity=action.severity,
            confidence=1.0,
            timestamp=state["timestamp"],
            data={
                "action_id": action.id,
                "action_type": action.action_type,
                "reason": blocked_entry["reason"],
            },
            source=action.source,
            proposal_id=action.proposal_id,
        )

    @staticmethod
    def _failed_event(result, action, state) -> Event:
        return Event(
            id=str(uuid.uuid4()),
            event_type="ACTION_EXECUTION_FAILED",
            asset_id=(action.target if action else None) or "",
            sensor_id="",
            severity=action.severity if action else Severity.CRITICAL,
            confidence=1.0,
            timestamp=state["timestamp"],
            data={
                "action_id": result.action_id,
                "message": result.message,
            },
            source=action.source if action else None,
            proposal_id=action.proposal_id if action else None,
        )
