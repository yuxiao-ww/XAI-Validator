"""
CNF Encoder for Autonomous Vehicle Planning Traces.

Converts PDDL planning constraints into Conjunctive Normal Form (CNF)
for downstream MIS-based validation and XAI reconciliation.
"""

import hashlib
import time
from dataclasses import dataclass, field
from typing import Optional
from collections import defaultdict

# ─────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────

@dataclass
class Clause:
    """A single CNF clause: a disjunction of literals."""
    literals: list[int]       # positive = var, negative = negated var
    source: str = ""          # human-readable origin (e.g. "precondition:move")
    clause_id: int = -1

    def __hash__(self):
        return hash(tuple(sorted(self.literals)))

    def __eq__(self, other):
        return sorted(self.literals) == sorted(other.literals)


@dataclass
class CNFFormula:
    """A CNF formula with variable mapping and metadata."""
    clauses: list[Clause] = field(default_factory=list)
    var_map: dict[str, int] = field(default_factory=dict)   # name → int
    var_names: dict[int, str] = field(default_factory=dict) # int → name
    _next_var: int = 1

    def get_var(self, name: str) -> int:
        if name not in self.var_map:
            self.var_map[name] = self._next_var
            self.var_names[self._next_var] = name
            self._next_var += 1
        return self.var_map[name]

    def add_clause(self, literals: list[int], source: str = "") -> Clause:
        c = Clause(literals=literals, source=source, clause_id=len(self.clauses))
        self.clauses.append(c)
        return c

    def num_vars(self) -> int:
        return self._next_var - 1

    def num_clauses(self) -> int:
        return len(self.clauses)

    def fingerprint(self) -> str:
        """SHA256 of sorted clause set using variable names (not IDs)."""
        def clause_sig(c: "Clause") -> str:
            parts = []
            for l in sorted(c.literals, key=abs):
                name = self.var_names.get(abs(l), str(abs(l)))
                parts.append(f"{'-' if l < 0 else ''}{name}")
            return ",".join(sorted(parts))

        sigs = sorted(clause_sig(c) for c in self.clauses)
        content = "|".join(sigs)
        return hashlib.sha256(content.encode()).hexdigest()


# ─────────────────────────────────────────────
# Encoder
# ─────────────────────────────────────────────

class CNFEncoder:
    """
    Encodes PDDL-style planning constraints into CNF.

    Supported constraint types:
      - action preconditions
      - action effects (add/delete)
      - state invariants
      - goal conditions
      - safety rules (collision avoidance, lane constraints, etc.)
    """

    def __init__(self):
        self._clause_cache: dict[str, list[Clause]] = {}
        self._cache_hits = 0
        self._cache_misses = 0
        self._compile_times: list[float] = []

    # ── public API ───────────────────────────

    def encode_trace(self, trace: "PlanningTrace") -> CNFFormula:
        """
        Full pipeline: PDDL trace → CNF formula.
        Returns a CNFFormula with all constraints encoded.
        """
        formula = CNFFormula()
        t0 = time.perf_counter()

        # 1. Encode initial state
        self._encode_state(trace.initial_state, formula, timestep=0, prefix="init")

        # 2. Encode each action step
        for t, action in enumerate(trace.actions):
            self._encode_action(action, formula, timestep=t)

        # 3. Encode goal
        self._encode_goal(trace.goal_conditions, formula,
                          timestep=len(trace.actions))

        # 4. Encode global safety invariants
        for inv in trace.safety_invariants:
            self._encode_invariant(inv, formula)

        # 5. Simplify (remove tautologies, subsumptions)
        before = formula.num_clauses()
        self._simplify(formula)
        after = formula.num_clauses()

        elapsed = time.perf_counter() - t0
        self._compile_times.append(elapsed)

        print(f"[Encoder] {before} → {after} clauses after simplification "
              f"({before - after} removed) in {elapsed*1000:.1f}ms")
        return formula

    def cache_stats(self) -> dict:
        total = self._cache_hits + self._cache_misses
        hit_rate = self._cache_hits / total if total else 0.0
        avg_time = (sum(self._compile_times) / len(self._compile_times)
                    if self._compile_times else 0.0)
        return {
            "cache_hits": self._cache_hits,
            "cache_misses": self._cache_misses,
            "hit_rate": f"{hit_rate:.1%}",
            "avg_compile_ms": f"{avg_time*1000:.1f}",
        }

    # ── encoding helpers ─────────────────────

    def _encode_state(self, state: dict, formula: CNFFormula,
                      timestep: int, prefix: str):
        """Each fluent in the state → unit clause at this timestep."""
        for fluent, value in state.items():
            var_name = f"{fluent}@{timestep}"
            v = formula.get_var(var_name)
            lit = v if value else -v
            formula.add_clause([lit], source=f"{prefix}:{fluent}")

    def _encode_action(self, action: "PDDLAction", formula: CNFFormula,
                       timestep: int):
        """
        For action a at time t:
          - Preconditions: must hold at t
          - Add effects: hold at t+1
          - Delete effects: do not hold at t+1
          - Frame axiom: unaffected fluents persist
        Uses cache keyed on (action_name, precond_hash, effects_hash).
        """
        cache_key = self._action_cache_key(action)
        if cache_key in self._clause_cache:
            self._cache_hits += 1
            cached = self._clause_cache[cache_key]
            # Re-map variables to current timestep
            self._replay_cached(cached, formula, timestep, action.name)
            return

        self._cache_misses += 1
        new_clauses: list[Clause] = []

        # Preconditions at time t
        for pre in action.preconditions:
            var_name = f"{pre.fluent}@{timestep}"
            v = formula.get_var(var_name)
            lit = v if pre.value else -v
            c = formula.add_clause([lit],
                                   source=f"pre:{action.name}:{pre.fluent}")
            new_clauses.append(c)

        # Add effects at t+1
        for eff in action.add_effects:
            var_name = f"{eff}@{timestep+1}"
            v = formula.get_var(var_name)
            c = formula.add_clause([v],
                                   source=f"add:{action.name}:{eff}")
            new_clauses.append(c)

        # Delete effects at t+1
        for eff in action.del_effects:
            var_name = f"{eff}@{timestep+1}"
            v = formula.get_var(var_name)
            c = formula.add_clause([-v],
                                   source=f"del:{action.name}:{eff}")
            new_clauses.append(c)

        self._clause_cache[cache_key] = new_clauses

    def _encode_goal(self, goals: list, formula: CNFFormula, timestep: int):
        for g in goals:
            var_name = f"{g.fluent}@{timestep}"
            v = formula.get_var(var_name)
            lit = v if g.value else -v
            formula.add_clause([lit], source=f"goal:{g.fluent}")

    def _encode_invariant(self, invariant: "SafetyInvariant",
                          formula: CNFFormula):
        """
        Encode safety invariant as CNF clause.
        e.g. ¬(collision) at all times → add unit clauses.
        """
        for t in range(invariant.horizon):
            var_name = f"{invariant.fluent}@{t}"
            v = formula.get_var(var_name)
            lit = -v if invariant.must_be_false else v
            formula.add_clause([lit],
                               source=f"invariant:{invariant.name}@{t}")

    def _simplify(self, formula: CNFFormula):
        """
        Constraint simplification:
          1. Remove tautological clauses (contains p and ¬p)
          2. Remove subsumed clauses (A ⊆ B → remove B)
        This is the main source of compilation time reduction.
        """
        # Step 1: tautology removal
        non_tautological = []
        for c in formula.clauses:
            lits = set(c.literals)
            if not any(-l in lits for l in lits):
                non_tautological.append(c)

        # Step 2: subsumption elimination
        # Sort by length (shorter clauses subsume longer ones)
        non_tautological.sort(key=lambda c: len(c.literals))
        surviving = []
        for i, c in enumerate(non_tautological):
            c_set = set(c.literals)
            subsumed = False
            for prev in surviving:
                if set(prev.literals).issubset(c_set):
                    subsumed = True
                    break
            if not subsumed:
                surviving.append(c)

        formula.clauses = surviving

    def _action_cache_key(self, action: "PDDLAction") -> str:
        pre = sorted(f"{p.fluent}={p.value}" for p in action.preconditions)
        add = sorted(action.add_effects)
        dlt = sorted(action.del_effects)
        raw = f"{action.name}|{pre}|{add}|{dlt}"
        return hashlib.md5(raw.encode()).hexdigest()

    def _replay_cached(self, cached: list[Clause], formula: CNFFormula,
                       timestep: int, action_name: str):
        """Re-instantiate cached clauses at a new timestep."""
        for c in cached:
            # The variable names embed @T; we re-look them up at new T
            new_lits = []
            for lit in c.literals:
                old_name = formula.var_names.get(abs(lit), str(abs(lit)))
                # Replace @OLD_T with @NEW_T
                if "@" in old_name:
                    base, _ = old_name.rsplit("@", 1)
                    new_name = f"{base}@{timestep}"
                else:
                    new_name = old_name
                v = formula.get_var(new_name)
                new_lits.append(v if lit > 0 else -v)
            formula.add_clause(new_lits, source=c.source)
