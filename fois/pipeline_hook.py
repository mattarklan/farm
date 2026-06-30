"""
fois/pipeline_hook.py

How the governance layer plugs into the existing operator pipeline.

The operator's Pipeline.run(state) already exists. This hook is called
at the top of that run — after state is built, before events fire.

It:
    1. Loads any accepted proposals from fois/proposals/accepted/
    2. Makes their resource demands visible in state so the constraint
       engine can account for them during the current cycle
    3. Processes any pending proposals (validates, decides, moves them)

The pipeline does not change its structure. This is additive.

Usage in pipeline.py:
    from fois.pipeline_hook import FoisHook

    class Pipeline:
        def run(self, state):
            FoisHook.run(state)   # ← add this line at the top
            events = self.event_engine.detect(state)
            ...
"""

import logging
from pathlib import Path

import yaml

from fois.governance import GovernanceLayer

logger = logging.getLogger(__name__)

SCHEMA_PATH = "config/rule_schema.yaml"
ACCEPTED_DIR = "fois/proposals/accepted"


class FoisHook:

    @staticmethod
    def run(state: dict) -> dict:
        """
        Run governance on any pending proposals, then inject accepted
        proposal resource demands into state for the constraint engine.

        Returns a summary dict for logging/debugging.
        """
        summary = {
            "proposals_evaluated": 0,
            "accepted": [],
            "rejected": [],
            "active_resource_claims": [],
        }

        # Load already-accepted proposals so policy checker knows about them
        active_proposals = FoisHook._load_accepted_proposals()

        # Run governance on anything new in pending
        try:
            governor = GovernanceLayer(
                schema_path=SCHEMA_PATH,
                state=state,
                active_proposals=active_proposals,
            )
            results = governor.evaluate_all_pending()
            summary["proposals_evaluated"] = len(results)
            for r in results:
                if r.accepted:
                    summary["accepted"].append(r.proposal_id)
                else:
                    summary["rejected"].append({"id": r.proposal_id, "gate": r.gate, "reasons": r.reasons})
        except Exception as e:
            logger.error(f"FoisHook: governance failed: {e}")

        # Reload accepted proposals (may include ones just accepted above)
        active_proposals = FoisHook._load_accepted_proposals()

        # Inject active resource claims into state
        # The constraint engine reads these to account for committed water/power
        claims = FoisHook._build_resource_claims(active_proposals)
        state["fois_resource_claims"] = claims
        summary["active_resource_claims"] = [c["proposal_id"] for c in claims]

        if claims:
            logger.info(f"FoisHook: {len(claims)} active resource claim(s) injected into state")

        return summary

    @staticmethod
    def _load_accepted_proposals() -> list[dict]:
        accepted = []
        path = Path(ACCEPTED_DIR)
        if not path.exists():
            return accepted
        for f in path.glob("*.yaml"):
            try:
                with open(f) as fp:
                    proposal = yaml.safe_load(fp)
                    if proposal:
                        accepted.append(proposal)
            except Exception as e:
                logger.warning(f"FoisHook: could not load accepted proposal {f}: {e}")
        return accepted

    @staticmethod
    def _build_resource_claims(proposals: list[dict]) -> list[dict]:
        """
        Flatten accepted proposals into a simple list of resource claims.
        These are injected into state for the constraint engine.
        """
        claims = []
        for p in proposals:
            resources = p.get("resources", {})
            water = resources.get("water", {})
            power = resources.get("power", {})
            crop = p.get("crop", {})

            claims.append({
                "proposal_id": p.get("proposal_id"),
                "zone_id": crop.get("zone_id"),
                "crop": crop.get("name"),
                "planting_date": crop.get("planting_date"),
                "harvest_date": crop.get("harvest_date"),
                "daily_water_litres": water.get("daily_litres", 0),
                "peak_water_litres": water.get("peak_litres", 0),
                "daily_power_kwh": power.get("daily_kwh", 0),
                "peak_power_kw": power.get("peak_kw", 0),
            })
        return claims
