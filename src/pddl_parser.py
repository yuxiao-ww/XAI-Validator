"""
PDDL Parser and Data Structures.

Parses PDDL domain/problem files and planning traces into
Python objects consumed by the CNF encoder.

Supports a subset of PDDL 2.1:
  - :strips actions (preconditions, add/delete effects)
  - :typing
  - initial state and goal conditions
  - custom :safety-invariants extension (for AV validation)
"""

from __future__ import annotations
import re
from dataclasses import dataclass, field
from pathlib import Path


# ─────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────

@dataclass
class Fluent:
    """A grounded fluent with a boolean value."""
    fluent: str
    value: bool = True

    def __repr__(self):
        prefix = "" if self.value else "¬"
        return f"{prefix}{self.fluent}"


@dataclass
class PDDLAction:
    """A grounded action with preconditions and effects."""
    name: str
    preconditions: list[Fluent] = field(default_factory=list)
    add_effects: list[str] = field(default_factory=list)
    del_effects: list[str] = field(default_factory=list)
    params: dict[str, str] = field(default_factory=dict)  # param → value

    def __repr__(self):
        return (f"Action({self.name}, pre={self.preconditions}, "
                f"+{self.add_effects}, -{self.del_effects})")


@dataclass
class SafetyInvariant:
    """A global safety constraint that must hold across all timesteps."""
    name: str
    fluent: str
    must_be_false: bool = True  # True means fluent must NOT hold
    horizon: int = 20

    def __repr__(self):
        sign = "¬" if self.must_be_false else ""
        return f"Invariant({self.name}: □{sign}{self.fluent})"


@dataclass
class PlanningTrace:
    """
    A complete planning trace: initial state → sequence of actions → goal.
    This is the main input to the XAI validator.
    """
    trace_id: str
    initial_state: dict[str, bool]
    actions: list[PDDLAction]
    goal_conditions: list[Fluent]
    safety_invariants: list[SafetyInvariant] = field(default_factory=list)
    metadata: dict = field(default_factory=dict)

    def __repr__(self):
        return (f"PlanningTrace(id={self.trace_id}, "
                f"steps={len(self.actions)}, "
                f"fluents={len(self.initial_state)})")


# ─────────────────────────────────────────────
# PDDL Parser
# ─────────────────────────────────────────────

class PDDLParser:
    """
    Parses PDDL domain + problem files into PlanningTrace objects.

    Usage:
        parser = PDDLParser()
        domain = parser.parse_domain("domain.pddl")
        trace = parser.parse_problem("problem.pddl", domain, plan="plan.txt")
    """

    def parse_domain(self, path: str | Path) -> dict:
        """Parse a PDDL domain file. Returns action schemas."""
        text = Path(path).read_text()
        text = self._strip_comments(text)
        return {
            "actions": self._parse_actions(text),
            "predicates": self._parse_predicates(text),
            "requirements": self._parse_requirements(text),
        }

    def parse_problem(self, path: str | Path, domain: dict,
                      plan_path: str | Path | None = None,
                      trace_id: str | None = None) -> PlanningTrace:
        """
        Parse a PDDL problem file + optional plan file into a PlanningTrace.
        If plan_path is None, an empty action sequence is used.
        """
        text = Path(path).read_text()
        text = self._strip_comments(text)

        problem_name = self._extract_name(text, "problem")
        init = self._parse_init(text)
        goal = self._parse_goal(text)
        invariants = self._parse_safety_invariants(text)

        actions = []
        if plan_path is not None:
            actions = self._parse_plan(plan_path, domain)

        return PlanningTrace(
            trace_id=trace_id or problem_name,
            initial_state=init,
            actions=actions,
            goal_conditions=goal,
            safety_invariants=invariants,
        )

    def parse_trace_from_dict(self, data: dict) -> PlanningTrace:
        """
        Build a PlanningTrace directly from a Python dict.
        Useful for programmatic trace generation in tests.

        Format:
        {
          "trace_id": "T001",
          "initial_state": {"at_intersection": True, "speed_high": False, ...},
          "actions": [
            {
              "name": "accelerate",
              "preconditions": [{"fluent": "lane_clear", "value": True}],
              "add_effects": ["speed_high"],
              "del_effects": ["speed_low"]
            }, ...
          ],
          "goal_conditions": [{"fluent": "at_destination", "value": True}],
          "safety_invariants": [
            {"name": "no_collision", "fluent": "collision", "must_be_false": True}
          ]
        }
        """
        actions = []
        for a in data.get("actions", []):
            actions.append(PDDLAction(
                name=a["name"],
                preconditions=[Fluent(**p) for p in a.get("preconditions", [])],
                add_effects=a.get("add_effects", []),
                del_effects=a.get("del_effects", []),
            ))

        invariants = []
        for inv in data.get("safety_invariants", []):
            invariants.append(SafetyInvariant(
                name=inv["name"],
                fluent=inv["fluent"],
                must_be_false=inv.get("must_be_false", True),
                horizon=inv.get("horizon", len(actions) + 1),
            ))

        return PlanningTrace(
            trace_id=data["trace_id"],
            initial_state=data["initial_state"],
            actions=actions,
            goal_conditions=[Fluent(**g) for g in data.get("goal_conditions", [])],
            safety_invariants=invariants,
            metadata=data.get("metadata", {}),
        )

    # ── private helpers ──────────────────────

    @staticmethod
    def _strip_comments(text: str) -> str:
        return re.sub(r";[^\n]*", "", text)

    @staticmethod
    def _extract_name(text: str, keyword: str) -> str:
        m = re.search(rf"\({keyword}\s+(\S+)", text, re.IGNORECASE)
        return m.group(1) if m else "unknown"

    def _parse_requirements(self, text: str) -> list[str]:
        m = re.search(r":requirements(.*?)\)", text, re.DOTALL)
        if not m:
            return []
        return re.findall(r":\S+", m.group(1))

    def _parse_predicates(self, text: str) -> list[str]:
        m = re.search(r":predicates(.*?)\)(?=\s*\()", text, re.DOTALL)
        if not m:
            return []
        return re.findall(r"\((\S+)", m.group(1))

    def _parse_actions(self, text: str) -> dict[str, PDDLAction]:
        actions = {}
        for m in re.finditer(r":action\s+(\S+)(.*?)(?=:action|\Z)",
                             text, re.DOTALL):
            name = m.group(1)
            body = m.group(2)
            actions[name] = self._parse_action_body(name, body)
        return actions

    def _parse_action_body(self, name: str, body: str) -> PDDLAction:
        pre = self._extract_section(body, ":precondition")
        eff = self._extract_section(body, ":effect")
        return PDDLAction(
            name=name,
            preconditions=self._parse_fluent_list(pre),
            add_effects=self._parse_add_effects(eff),
            del_effects=self._parse_del_effects(eff),
        )

    @staticmethod
    def _extract_section(text: str, keyword: str) -> str:
        pattern = rf"{re.escape(keyword)}\s*(\(.*)"
        m = re.search(pattern, text, re.DOTALL)
        if not m:
            return ""
        # Extract balanced parentheses
        s = m.group(1)
        depth, end = 0, 0
        for i, ch in enumerate(s):
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        return s[:end]

    def _parse_fluent_list(self, text: str) -> list[Fluent]:
        fluents = []
        # Handle (and ...) or single predicate
        text = re.sub(r"\(and\s*", "", text).strip("() ")
        for m in re.finditer(r"\(not\s+\((\S+)[^)]*\)\s*\)|\((\S+)[^)]*\)",
                             text):
            if m.group(1):
                fluents.append(Fluent(fluent=m.group(1), value=False))
            elif m.group(2):
                fluents.append(Fluent(fluent=m.group(2), value=True))
        return fluents

    def _parse_add_effects(self, text: str) -> list[str]:
        # Positive effects (not wrapped in (not ...))
        dels = set(self._parse_del_effects(text))
        all_preds = re.findall(r"\((\w+)(?:\s+\??\w+)*\)", text)
        return [p for p in all_preds if p not in dels and p != "not" and p != "and"]

    def _parse_del_effects(self, text: str) -> list[str]:
        return re.findall(r"\(not\s+\((\w+)", text)

    def _parse_init(self, text: str) -> dict[str, bool]:
        m = re.search(r":init(.*?)\(:goal", text, re.DOTALL)
        if not m:
            return {}
        state = {}
        for pred in re.findall(r"\((\w+)[^)]*\)", m.group(1)):
            state[pred] = True
        return state

    def _parse_goal(self, text: str) -> list[Fluent]:
        m = re.search(r":goal\s*(\(.*)", text, re.DOTALL)
        if not m:
            return []
        return self._parse_fluent_list(m.group(1))

    def _parse_safety_invariants(self, text: str) -> list[SafetyInvariant]:
        """Parse custom :safety-invariants section (AV extension)."""
        invs = []
        m = re.search(r":safety-invariants(.*?)(?=\(:|$)", text, re.DOTALL)
        if not m:
            return invs
        for im in re.finditer(r"\((\S+)\s+(must-not-hold|must-hold)\s+(\S+)\)",
                               m.group(1)):
            invs.append(SafetyInvariant(
                name=im.group(1),
                fluent=im.group(3),
                must_be_false=(im.group(2) == "must-not-hold"),
            ))
        return invs

    def _parse_plan(self, path: str | Path, domain: dict) -> list[PDDLAction]:
        """
        Parse a plan file (one action per line, e.g. output by Fast Downward).
        Lines look like: (move vehicle_1 loc_A loc_B)
        """
        actions = []
        for line in Path(path).read_text().splitlines():
            line = line.strip().strip("()")
            if not line or line.startswith(";"):
                continue
            parts = line.split()
            action_name = parts[0]
            schema = domain["actions"].get(action_name, PDDLAction(name=action_name))
            # Ground the schema with the given parameters
            grounded = PDDLAction(
                name=action_name,
                preconditions=list(schema.preconditions),
                add_effects=list(schema.add_effects),
                del_effects=list(schema.del_effects),
                params={f"?p{i}": v for i, v in enumerate(parts[1:])},
            )
            actions.append(grounded)
        return actions
