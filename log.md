### Architecture established

* Registry-based asset/sensor loading (modular farm topology)
* Central `State` snapshot system for runtime sensor values
* Rolling window time-series store per sensor (bounded memory history)
* Derived metrics engine producing:

  * slope (rate of change)
  * rolling average
  * volatility
* Event engine v2 upgraded to:

  * combine raw + derived signals
  * support confidence scoring per event
  * evaluate multi-signal conditions (threshold + trend + stability)
* Rule engine producing structured actions from events
* Constraint engine introduced as a safety filter layer:

  * blocks unsafe actions
  * supports delayed execution rules
  * enforces runtime safety conditions before execution
* Executor layer implemented as a strict action dispatcher (no decision logic)
* Pipeline orchestrator introduced to enforce deterministic execution order:

  * Events → Actions → Constraints → Execution

### Key system improvements

* Shifted from stateless threshold detection → temporal + behavioural inference
* Added separation between:

  * detection (events)
  * decision (rules)
  * safety (constraints)
  * actuation (executor)
* Introduced early-stage confidence scoring for events
* Eliminated ad-hoc script wiring via unified pipeline controller

### Current capability

System can now:

* detect sensor threshold violations
* detect early trend-based anomalies (via derived slope)
* filter unsafe actions before execution
* execute only validated control actions via a unified interface