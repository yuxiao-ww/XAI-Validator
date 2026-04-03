"""
XAI Validator — Main Orchestrator.

Coordinates PDDL parsing → CNF encoding → MIS validation.
Supports single-trace and batch validation with caching and metrics.
"""

from __future__ import annotations
import json
import time
from pathlib import Path
from dataclasses import dataclass, field, asdict

from .pddl_parser import PDDLParser, PlanningTrace
from .cnf_encoder import CNFEncoder
from .mis_engine import MISEngine, ValidationResult


@dataclass
class BatchReport:
    """Aggregated report for a batch of planning traces."""
    total: int = 0
    valid: int = 0
    invalid: int = 0
    success_rate: float = 0.0
    avg_sat_ms: float = 0.0
    avg_mis_ms: float = 0.0
    results: list[ValidationResult] = field(default_factory=list)

    def print_summary(self):
        print(f"\n{'='*60}")
        print(f"BATCH VALIDATION REPORT")
        print(f"{'='*60}")
        print(f"Total traces:   {self.total}")
        print(f"Valid:          {self.valid} ({self.success_rate:.1%})")
        print(f"Invalid:        {self.invalid}")
        print(f"Avg SAT time:   {self.avg_sat_ms:.1f}ms")
        print(f"Avg MIS time:   {self.avg_mis_ms:.1f}ms")
        print(f"{'='*60}\n")
        for r in self.results:
            status = "✓" if r.is_valid else "✗"
            print(f"  {status} {r.trace_id:30s}  {r.summary}")


class XAIValidator:
    """
    Main entry point for AV planning trace validation.

    Example:
        validator = XAIValidator()
        result = validator.validate_file("domain.pddl", "problem.pddl", "plan.txt")
        print(result.report())

        # Batch mode
        report = validator.validate_batch(traces)
        report.print_summary()
    """

    def __init__(self, max_mis: int = 5, timeout_s: float = 10.0):
        self._parser = PDDLParser()
        self._encoder = CNFEncoder()
        self._mis_engine = MISEngine(max_mis=max_mis, timeout_s=timeout_s)
        self._formula_cache: dict[str, ValidationResult] = {}
        self._total_traces = 0
        self._valid_traces = 0

    # ── public API ───────────────────────────

    def validate_file(self, domain_path: str, problem_path: str,
                      plan_path: str | None = None,
                      trace_id: str | None = None) -> ValidationResult:
        """Validate a single trace from PDDL files."""
        domain = self._parser.parse_domain(domain_path)
        trace = self._parser.parse_problem(problem_path, domain,
                                           plan_path=plan_path,
                                           trace_id=trace_id)
        return self._validate(trace)

    def validate_trace(self, trace: PlanningTrace) -> ValidationResult:
        """Validate a PlanningTrace object directly."""
        return self._validate(trace)

    def validate_dict(self, data: dict) -> ValidationResult:
        """Validate a trace from a Python dict (see PDDLParser.parse_trace_from_dict)."""
        trace = self._parser.parse_trace_from_dict(data)
        return self._validate(trace)

    def validate_batch(self, traces: list[PlanningTrace | dict],
                       verbose: bool = True) -> BatchReport:
        """Validate a list of traces and return aggregate statistics."""
        results = []
        for item in traces:
            if isinstance(item, dict):
                result = self.validate_dict(item)
            else:
                result = self.validate_trace(item)
            results.append(result)
            if verbose:
                print(f"  {'✓' if result.is_valid else '✗'} {result.trace_id}")

        valid = sum(1 for r in results if r.is_valid)
        total = len(results)
        report = BatchReport(
            total=total,
            valid=valid,
            invalid=total - valid,
            success_rate=valid / total if total else 0.0,
            avg_sat_ms=sum(r.sat_time_ms for r in results) / total if total else 0,
            avg_mis_ms=sum(r.mis_time_ms for r in results) / total if total else 0,
            results=results,
        )
        return report

    def load_traces_from_json(self, path: str) -> list[PlanningTrace]:
        """Load a JSON file containing a list of trace dicts."""
        data = json.loads(Path(path).read_text())
        traces = [self._parser.parse_trace_from_dict(d) for d in data]
        print(f"[Validator] Loaded {len(traces)} traces from {path}")
        return traces

    def cache_stats(self) -> dict:
        enc_stats = self._encoder.cache_stats()
        return {
            "formula_cache_size": len(self._formula_cache),
            "encoder": enc_stats,
            "total_traces": self._total_traces,
            "valid_traces": self._valid_traces,
            "success_rate": (f"{self._valid_traces/self._total_traces:.1%}"
                             if self._total_traces else "N/A"),
        }

    # ── internal ─────────────────────────────

    def _validate(self, trace: PlanningTrace) -> ValidationResult:
        self._total_traces += 1

        # Encode to CNF
        formula = self._encoder.encode_trace(trace)

        # Check formula cache (avoid re-running SAT on identical CNF)
        fp = formula.fingerprint()
        if fp in self._formula_cache:
            cached = self._formula_cache[fp]
            print(f"[Validator] Cache hit for {trace.trace_id} "
                  f"(same CNF as cached result)")
            result = ValidationResult(
                trace_id=trace.trace_id,
                is_valid=cached.is_valid,
                mis_list=cached.mis_list,
                sat_time_ms=0.0,
                mis_time_ms=0.0,
                num_clauses=formula.num_clauses(),
                num_vars=formula.num_vars(),
                summary=cached.summary.replace(cached.trace_id, trace.trace_id),
            )
            if result.is_valid:
                self._valid_traces += 1
            return result

        # Run validation
        result = self._mis_engine.validate(formula, trace_id=trace.trace_id)
        self._formula_cache[fp] = result

        if result.is_valid:
            self._valid_traces += 1
        return result
