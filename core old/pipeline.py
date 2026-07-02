#pipeline.py
class Pipeline:
    def __init__(self,
                 event_engine,
                 rule_engine,
                 constraint_engine,
                 executor,
                 derived_engine,
                 window_store,
                 decision_memory):
        self.event_engine = event_engine
        self.rule_engine = rule_engine
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
        actions = self.rule_engine.process(events, state)
        allowed, blocked, delayed = self.constraint_engine.evaluate(actions, state, derived)
        for b in blocked:
            self.memory.record_constraint(b)
        results = self.executor.execute(allowed)
        for r in results:
            self.memory.record_action(r)
        return {
            "events": events,
            "derived": derived,
            "actions": actions,
            "allowed": allowed,
            "blocked": blocked,
            "delayed": delayed,
            "results": results
        }