class Pipeline:
    def __init__(self,
                 event_engine,
                 rule_engine,
                 goal_engine,
                 constraint_engine,
                 executor,
                 derived_engine,
                 window_store,
                 decision_memory):
        self.event_engine = event_engine
        self.rule_engine = rule_engine
        self.goal_engine = goal_engine
        self.constraint_engine = constraint_engine
        self.executor = executor
        self.derived_engine = derived_engine
        self.window_store = window_store
        self.memory = decision_memory

    def run(self, state):
        for sensor_id, sensor in state["sensors"].items():
            self.window_store.append(
                sensor_id,
                sensor.value,
                sensor.timestamp
            )

        derived = self.derived_engine.compute(
            list(state["sensors"].keys())
        )
        state["derived"] = derived

        events = self.event_engine.process(state, derived)
        for e in events:
            self.memory.record_event(e)

        # Goals: end anything whose `until` condition has tripped, then
        # start any new goal-rules whose trigger event just fired. Goal
        # completions are themselves events (GOAL_STOPPED) - folded into
        # the same `events` list so they get logged and are visible to
        # rule_engine this same cycle (e.g. a rule that reacts to a goal
        # finishing).
        goal_start_actions, goal_stop_actions, goal_events = self.goal_engine.process(
            events, state, derived
        )
        events = events + goal_events
        for e in goal_events:
            self.memory.record_event(e)

        instant_actions = self.rule_engine.process(events, state)
        candidate_actions = instant_actions + goal_start_actions

        allowed, blocked, delayed = self.constraint_engine.evaluate(candidate_actions, state, derived)

        for b in blocked:
            self.memory.record_constraint(b)

        for a in allowed:
            self.memory.record_action(a)

        # Goal-stop actions bypass the constraint filter on purpose: a
        # safety/goal-completion stop should never itself be blockable by
        # the same constraint system that might be the reason it's
        # stopping (e.g. don't let "battery too low" also prevent the
        # action that's shutting things off because the battery is too low).
        for a in goal_stop_actions:
            self.memory.record_action(a)

        executable = allowed + goal_stop_actions
        results = self.executor.execute(executable)

        return {
            "events": events,
            "derived": derived,
            "actions": candidate_actions,
            "allowed": allowed,
            "blocked": blocked,
            "delayed": delayed,
            "goal_stop_actions": goal_stop_actions,
            "results": results
        }
