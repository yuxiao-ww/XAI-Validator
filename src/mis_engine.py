"""
MIS-Based XAI Reconciliation Engine.

Finds Minimal Infeasible Subsets (MIS) of CNF clauses to explain
WHY an autonomous vehicle planning trace is invalid.

Algorithm: MARCO-style MIS enumeration with GROW/SHRINK operators.
  - GROW: extend a satisfiable subset toward a Maximal Satisfiable Subset (MSS)
  - SHRINK: reduce an infeasible subset toward a Minimal Infeasible Subset (MIS)
  
Each MIS is an "explanation": the smallest set of constraints that are
mutually contradictory, enabling human-readable XAI output.
"""

from __future__ import annotations
import itertools
import time
from dataclasses import dataclass, field
from typing import Generator

from .cnf_encoder import CNFFormula, Clause


# ─────────────────────────────────────────────
# Result types
# ─────────────────────────────────────────────

@dataclass
class MISResult:
    """A single Minimal Infeasible Subset."""
    clauses: list[Clause]
    explanation: str = ""
    severity: str = "ERROR"      # ERROR | WARNING | INFO
    category: str = "unknown"    # collision | goal_unreachable | precondition | invariant

    def clause_ids(self) -> list[int]:
        return [c.clause_id for c in self.clauses]

    def sources(self) -> list[str]:
        return list(dict.fromkeys(c.source for c in self.clauses))


@dataclass
class ValidationResult:
    """Full validation result for a planning trace."""
    trace_id: str
    is_valid: bool
    mis_list: list[MISResult] = field(default_factory=list)
    sat_time_ms: float = 0.0
    mis_time_ms: float = 0.0
    num_clauses: int = 0
    num_vars: int = 0
    summary: str = ""

    def has_errors(self) -> bool:
        return any(m.severity == "ERROR" for m in self.mis_list)

    def report(self) -> str:
        lines = [
            f"{'='*60}",
            f"Trace: {self.trace_id}",
            f"Valid: {'✓ YES' if self.is_valid else '✗ NO'}",
            f"Clauses: {self.num_clauses} | Variables: {self.num_vars}",
            f"SAT check: {self.sat_time_ms:.1f}ms | MIS search: {self.mis_time_ms:.1f}ms",
        ]
        if self.mis_list:
            lines.append(f"\nFound {len(self.mis_list)} infeasible subset(s):\n")
            for i, mis in enumerate(self.mis_list, 1):
                lines.append(f"  [{mis.severity}] MIS #{i} ({mis.category})")
                lines.append(f"    Explanation: {mis.explanation}")
                lines.append(f"    Sources: {', '.join(mis.sources())}")
                lines.append(f"    Clauses: {mis.clause_ids()}")
                lines.append("")
        lines.append(f"{'='*60}")
        return "\n".join(lines)


# ─────────────────────────────────────────────
# Lightweight SAT solver (DPLL)
# ─────────────────────────────────────────────

class DPLLSolver:
    """
    Lightweight DPLL SAT solver for small CNF formulas.
    Used internally by the MIS engine.
    For production, swap with pysat or Z3.
    """

    def solve(self, formula: CNFFormula,
              subset: list[int] | None = None) -> bool:
        """
        Check satisfiability of (a subset of) the formula.
        subset: list of clause indices; None = all clauses.
        Returns True if SAT, False if UNSAT.
        """
        if subset is None:
            clauses = [set(c.literals) for c in formula.clauses]
        else:
            clauses = [set(formula.clauses[i].literals) for i in subset]

        assignment: dict[int, bool] = {}
        return self._dpll(clauses, assignment)

    def _dpll(self, clauses: list[set[int]],
              assignment: dict[int, bool]) -> bool:
        # Propagate unit clauses
        changed = True
        while changed:
            changed = False
            for clause in clauses:
                unresolved = [l for l in clause
                              if abs(l) not in assignment]
                resolved_true = any(
                    (l > 0 and assignment.get(abs(l)) is True) or
                    (l < 0 and assignment.get(abs(l)) is False)
                    for l in clause
                )
                if resolved_true:
                    continue
                if len(unresolved) == 0:
                    return False  # Empty clause → conflict
                if len(unresolved) == 1:
                    lit = unresolved[0]
                    assignment[abs(lit)] = (lit > 0)
                    changed = True

        # Check if all clauses satisfied
        all_sat = True
        for clause in clauses:
            sat = any(
                (l > 0 and assignment.get(abs(l)) is True) or
                (l < 0 and assignment.get(abs(l)) is False)
                for l in clause
            )
            if not sat:
                all_sat = False
                break
        if all_sat:
            return True

        # Choose unassigned variable (VSIDS-lite: pick first unresolved)
        var = None
        for clause in clauses:
            for l in clause:
                if abs(l) not in assignment:
                    var = abs(l)
                    break
            if var:
                break
        if var is None:
            return True

        for val in [True, False]:
            new_assign = dict(assignment)
            new_assign[var] = val
            if self._dpll(clauses, new_assign):
                return True
        return False


# ─────────────────────────────────────────────
# MIS Engine
# ─────────────────────────────────────────────

class MISEngine:
    """
    Enumerates Minimal Infeasible Subsets of a CNF formula.

    Algorithm: MARCO (Liffiton & Malik 2016) adapted for planning traces.
    - Maps clause indices into a seed space
    - Alternates GROW (find MSS) and SHRINK (find MIS)
    - Terminates after max_mis results or timeout
    """

    def __init__(self, max_mis: int = 10, timeout_s: float = 30.0):
        self.max_mis = max_mis
        self.timeout_s = timeout_s
        self._solver = DPLLSolver()

    def find_all_mis(self, formula: CNFFormula) -> list[MISResult]:
        """Find all (up to max_mis) MIS in the formula."""
        n = formula.num_clauses()
        if n == 0:
            return []

        # Quick check: is the full formula UNSAT?
        if self._solver.solve(formula, list(range(n))):
            return []  # SAT globally — no MIS

        t0 = time.perf_counter()
        results: list[MISResult] = []
        found_mis_sets: list[frozenset] = []

        # Try deletion-based approach: start from all clauses, shrink to MIS
        mis_indices = self._shrink(formula, list(range(n)))
        if mis_indices:
            mis_clauses = [formula.clauses[i] for i in mis_indices]
            mis = self._build_mis_result(mis_clauses, formula)
            results.append(mis)
            found_mis_sets.append(frozenset(mis_indices))

        # Try to find additional MIS by rotating out one clause from known MIS
        for _ in range(self.max_mis - 1):
            if not results or time.perf_counter() - t0 > self.timeout_s:
                break
            last_mis = found_mis_sets[-1]
            found_new = False
            for exclude_idx in last_mis:
                # Try a seed that excludes this clause but includes everything else
                seed = [i for i in range(n) if i != exclude_idx]
                if not self._solver.solve(formula, seed):
                    new_mis = self._shrink(formula, seed)
                    fs = frozenset(new_mis)
                    if fs and fs not in found_mis_sets:
                        mis_clauses = [formula.clauses[i] for i in new_mis]
                        mis = self._build_mis_result(mis_clauses, formula)
                        results.append(mis)
                        found_mis_sets.append(fs)
                        found_new = True
                        break
            if not found_new:
                break

        return results

    def validate(self, formula: CNFFormula,
                 trace_id: str = "trace") -> ValidationResult:
        """Full validation pipeline: SAT check + MIS enumeration."""
        t0 = time.perf_counter()
        is_sat = self._solver.solve(formula)
        sat_ms = (time.perf_counter() - t0) * 1000

        t1 = time.perf_counter()
        mis_list = [] if is_sat else self.find_all_mis(formula)
        mis_ms = (time.perf_counter() - t1) * 1000

        result = ValidationResult(
            trace_id=trace_id,
            is_valid=is_sat,
            mis_list=mis_list,
            sat_time_ms=sat_ms,
            mis_time_ms=mis_ms,
            num_clauses=formula.num_clauses(),
            num_vars=formula.num_vars(),
        )
        result.summary = self._summarize(result)
        return result

    # ── MARCO operators ──────────────────────

    def _grow(self, formula: CNFFormula,
              seed: list[int],
              universe: set[int]) -> list[int]:
        """Extend seed to a Maximal Satisfiable Subset."""
        current = list(seed)
        current_set = set(current)
        for idx in sorted(universe - current_set):
            candidate = current + [idx]
            if self._solver.solve(formula, candidate):
                current = candidate
                current_set.add(idx)
        return current

    def _shrink(self, formula: CNFFormula,
                seed: list[int]) -> list[int]:
        """
        Reduce seed to a Minimal Infeasible Subset via
        deletion-based shrinking (Liffiton & Malik 2016).
        """
        if self._solver.solve(formula, seed):
            return []  # Seed is SAT — shouldn't happen
        current = list(seed)
        for idx in list(current):
            candidate = [i for i in current if i != idx]
            if not candidate:
                break
            if not self._solver.solve(formula, candidate):
                current = candidate  # Still UNSAT without idx → drop it
        return current

    # ── XAI explanation builder ──────────────

    def _build_mis_result(self, clauses: list[Clause],
                          formula: CNFFormula) -> MISResult:
        """Generate a human-readable explanation for a MIS."""
        sources = [c.source for c in clauses]
        category, severity = self._classify(sources)
        explanation = self._explain(clauses, category)
        return MISResult(
            clauses=clauses,
            explanation=explanation,
            severity=severity,
            category=category,
        )

    @staticmethod
    def _classify(sources: list[str]) -> tuple[str, str]:
        joined = " ".join(sources).lower()
        if "invariant" in joined and "collision" in joined:
            return "collision", "ERROR"
        if "invariant" in joined:
            return "safety_invariant", "ERROR"
        if "goal" in joined and "del" in joined:
            return "goal_unreachable", "ERROR"
        if "pre:" in joined:
            return "precondition_violation", "WARNING"
        if "add:" in joined and "del:" in joined:
            return "effect_conflict", "WARNING"
        return "constraint_conflict", "INFO"

    @staticmethod
    def _explain(clauses: list[Clause], category: str) -> str:
        unique_sources = list(dict.fromkeys(c.source for c in clauses))
        n = len(clauses)

        templates = {
            "collision": (
                f"Safety invariant violated: {n} constraints mutually require "
                f"a collision state. Conflicting sources: {unique_sources[:3]}"
            ),
            "goal_unreachable": (
                f"Goal is unreachable: an action deletes a fluent required by "
                f"the goal. Check: {unique_sources[:3]}"
            ),
            "precondition_violation": (
                f"Action precondition unsatisfied: {n} constraints conflict. "
                f"Action sequence may be invalid. Sources: {unique_sources[:3]}"
            ),
            "effect_conflict": (
                f"Conflicting effects detected across {n} constraints: "
                f"an add-effect and delete-effect contradict. "
                f"Sources: {unique_sources[:3]}"
            ),
            "safety_invariant": (
                f"Safety invariant breached across {n} timesteps. "
                f"Sources: {unique_sources[:3]}"
            ),
            "constraint_conflict": (
                f"General constraint conflict among {n} clauses. "
                f"Sources: {unique_sources[:3]}"
            ),
        }
        return templates.get(category, f"Infeasible subset of size {n}.")

    @staticmethod
    def _summarize(result: ValidationResult) -> str:
        if result.is_valid:
            return f"✓ Trace {result.trace_id} validated successfully."
        cats = [m.category for m in result.mis_list]
        return (f"✗ Trace {result.trace_id} INVALID. "
                f"{len(result.mis_list)} conflict(s): {', '.join(set(cats))}")
