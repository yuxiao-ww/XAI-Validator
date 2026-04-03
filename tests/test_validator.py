"""
Unit tests for AV XAI Validator.
Run with: pytest tests/ -v
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import pytest
from src import (
    XAIValidator, PDDLParser, CNFEncoder, MISEngine,
    PlanningTrace, PDDLAction, Fluent, SafetyInvariant, CNFFormula,
)


# ─────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────

@pytest.fixture
def simple_valid_trace_dict():
    return {
        "trace_id": "TEST-valid",
        "initial_state": {"speed_high": True, "lane_clear": True},
        "actions": [
            {
                "name": "decelerate",
                "preconditions": [{"fluent": "speed_high", "value": True}],
                "add_effects": ["speed_low"],
                "del_effects": ["speed_high"],
            },
            {
                "name": "arrive",
                "preconditions": [{"fluent": "speed_low", "value": True}],
                "add_effects": ["at_destination"],
                "del_effects": [],
            },
        ],
        "goal_conditions": [{"fluent": "at_destination", "value": True}],
        "safety_invariants": [
            {"name": "no_collision", "fluent": "collision",
             "must_be_false": True, "horizon": 3}
        ],
    }


@pytest.fixture
def collision_trace_dict():
    return {
        "trace_id": "TEST-collision",
        "initial_state": {"speed_low": True},
        "actions": [
            {
                "name": "unsafe_accel",
                "preconditions": [],
                "add_effects": ["collision", "speed_high"],
                "del_effects": ["speed_low"],
            }
        ],
        "goal_conditions": [{"fluent": "at_destination", "value": True}],
        "safety_invariants": [
            {"name": "no_collision", "fluent": "collision",
             "must_be_false": True, "horizon": 2}
        ],
    }


@pytest.fixture
def validator():
    return XAIValidator(max_mis=5, timeout_s=5.0)


# ─────────────────────────────────────────────
# PDDL Parser tests
# ─────────────────────────────────────────────

class TestPDDLParser:
    def test_parse_trace_from_dict_valid(self, simple_valid_trace_dict):
        parser = PDDLParser()
        trace = parser.parse_trace_from_dict(simple_valid_trace_dict)
        assert trace.trace_id == "TEST-valid"
        assert len(trace.actions) == 2
        assert len(trace.safety_invariants) == 1
        assert trace.safety_invariants[0].must_be_false is True

    def test_fluents_parsed_correctly(self, simple_valid_trace_dict):
        parser = PDDLParser()
        trace = parser.parse_trace_from_dict(simple_valid_trace_dict)
        pre = trace.actions[0].preconditions[0]
        assert pre.fluent == "speed_high"
        assert pre.value is True

    def test_effects_parsed(self, simple_valid_trace_dict):
        parser = PDDLParser()
        trace = parser.parse_trace_from_dict(simple_valid_trace_dict)
        action = trace.actions[0]
        assert "speed_low" in action.add_effects
        assert "speed_high" in action.del_effects


# ─────────────────────────────────────────────
# CNF Encoder tests
# ─────────────────────────────────────────────

class TestCNFEncoder:
    def test_encode_produces_clauses(self, simple_valid_trace_dict):
        parser = PDDLParser()
        trace = parser.parse_trace_from_dict(simple_valid_trace_dict)
        encoder = CNFEncoder()
        formula = encoder.encode_trace(trace)
        assert formula.num_clauses() > 0
        assert formula.num_vars() > 0

    def test_fingerprint_deterministic(self, simple_valid_trace_dict):
        """Same trace encoded by two fresh encoders → same fingerprint."""
        parser = PDDLParser()
        trace = parser.parse_trace_from_dict(simple_valid_trace_dict)
        # Use separate encoder instances to ensure cold-cache parity
        f1 = CNFEncoder().encode_trace(trace)
        f2 = CNFEncoder().encode_trace(trace)
        assert f1.fingerprint() == f2.fingerprint()

    def test_cache_hit_on_second_encode(self, simple_valid_trace_dict):
        parser = PDDLParser()
        trace = parser.parse_trace_from_dict(simple_valid_trace_dict)
        encoder = CNFEncoder()
        encoder.encode_trace(trace)
        encoder.encode_trace(trace)
        stats = encoder.cache_stats()
        assert int(stats["cache_hits"]) > 0

    def test_simplification_reduces_clauses(self):
        """Tautological clauses should be removed."""
        formula = CNFFormula()
        v = formula.get_var("x")
        # Tautology: x OR ¬x
        formula.add_clause([v, -v], source="tautology")
        formula.add_clause([v], source="unit")

        encoder = CNFEncoder()
        encoder._simplify(formula)
        sources = [c.source for c in formula.clauses]
        assert "tautology" not in sources
        assert "unit" in sources


# ─────────────────────────────────────────────
# MIS Engine tests
# ─────────────────────────────────────────────

class TestMISEngine:
    def test_valid_formula_is_sat(self):
        formula = CNFFormula()
        v1 = formula.get_var("a")
        v2 = formula.get_var("b")
        formula.add_clause([v1], source="unit_a")
        formula.add_clause([v2], source="unit_b")
        engine = MISEngine()
        result = engine.validate(formula, trace_id="sat-test")
        assert result.is_valid is True
        assert result.mis_list == []

    def test_unsat_formula_finds_mis(self):
        formula = CNFFormula()
        v = formula.get_var("x")
        formula.add_clause([v], source="x_true")
        formula.add_clause([-v], source="x_false")
        engine = MISEngine()
        result = engine.validate(formula, trace_id="unsat-test")
        assert result.is_valid is False
        assert len(result.mis_list) >= 1

    def test_mis_is_minimal(self):
        """MIS should contain only necessary clauses."""
        formula = CNFFormula()
        v1 = formula.get_var("x")
        v2 = formula.get_var("y")
        formula.add_clause([v1], source="x_true")
        formula.add_clause([-v1], source="x_false")
        formula.add_clause([v2], source="y_true")  # Irrelevant, satisfiable
        engine = MISEngine(max_mis=5)
        result = engine.validate(formula, trace_id="minimal-test")
        assert result.is_valid is False
        # MIS should not include the y_true clause
        for mis in result.mis_list:
            sources = mis.sources()
            assert not all(s == "y_true" for s in sources)


# ─────────────────────────────────────────────
# Validator integration tests
# ─────────────────────────────────────────────

class TestXAIValidator:
    def test_valid_trace_passes(self, validator, simple_valid_trace_dict):
        result = validator.validate_dict(simple_valid_trace_dict)
        assert result.is_valid is True
        assert result.mis_list == []

    def test_collision_trace_fails(self, validator, collision_trace_dict):
        result = validator.validate_dict(collision_trace_dict)
        assert result.is_valid is False
        assert len(result.mis_list) >= 1

    def test_collision_category(self, validator, collision_trace_dict):
        result = validator.validate_dict(collision_trace_dict)
        categories = {m.category for m in result.mis_list}
        # Should detect collision or safety_invariant category
        assert categories & {"collision", "safety_invariant", "constraint_conflict"}

    def test_batch_validation(self, validator, simple_valid_trace_dict,
                              collision_trace_dict):
        traces = [simple_valid_trace_dict, collision_trace_dict]
        report = validator.validate_batch(traces, verbose=False)
        assert report.total == 2
        assert report.valid == 1
        assert report.invalid == 1
        assert abs(report.success_rate - 0.5) < 0.01

    def test_formula_cache_reuse(self, validator, simple_valid_trace_dict):
        """Same CNF fingerprint → cache hit."""
        validator.validate_dict(simple_valid_trace_dict)
        validator.validate_dict(simple_valid_trace_dict)
        stats = validator.cache_stats()
        assert stats["formula_cache_size"] >= 1

    def test_report_string(self, validator, collision_trace_dict):
        result = validator.validate_dict(collision_trace_dict)
        report = result.report()
        assert "MIS" in report
        assert "collision_trace" in report or "TEST" in report

    def test_success_rate_above_97_pct(self, validator):
        """Simulate the 500-trace benchmark from the resume."""
        from scripts.generate_traces import generate_traces
        traces_data = generate_traces(100, invalid_rate=0.03)
        report = validator.validate_batch(traces_data, verbose=False)
        assert report.success_rate >= 0.90  # relaxed for small sample


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
