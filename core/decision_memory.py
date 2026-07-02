from collections import defaultdict, deque
from datetime import datetime, timedelta


class DecisionMemory:
    def __init__(self, max_history=200):
        # key -> deque of timestamps
        self.event_history = defaultdict(lambda: deque(maxlen=max_history))
        self.action_history = defaultdict(lambda: deque(maxlen=max_history))
        self.constraint_history = defaultdict(lambda: deque(maxlen=max_history))

    # -----------------------
    # EVENT TRACKING
    # -----------------------
    def record_event(self, event):
        key = event.event_type + ":" + event.sensor_id
        self.event_history[key].append(datetime.utcnow())

    # -----------------------
    # ACTION TRACKING
    # -----------------------
    def record_action(self, action):
        key = action.action_type + ":" + str(action.target)
        self.action_history[key].append(datetime.utcnow())

    # -----------------------
    # CONSTRAINT TRACKING
    # -----------------------
    def record_constraint(self, blocked_entry):
        action = blocked_entry["action"]
        key = action.action_type + ":" + str(action.target)
        self.constraint_history[key].append(datetime.utcnow())

    # -----------------------
    # FREQUENCY ANALYSIS
    # -----------------------
    def get_frequency(self, history, window_seconds=300):
        now = datetime.utcnow()
        cutoff = now - timedelta(seconds=window_seconds)
        return sum(1 for t in history if t >= cutoff)

    # -----------------------
    # ESCALATION SCORE
    # -----------------------
    def get_escalation_score(self, key, window_seconds=300):
        event_count = self.get_frequency(self.event_history[key], window_seconds)
        constraint_count = self.get_frequency(self.constraint_history[key], window_seconds)
        score = 0.0
        score += event_count * 0.2
        score += constraint_count * 0.5
        return min(1.0, score)
