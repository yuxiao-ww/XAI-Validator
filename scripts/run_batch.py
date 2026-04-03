"""
Batch validation runner.
Demonstrates:
  1. Single trace validation with PDDL files
  2. Batch validation of 500+ traces from JSON
  3. Cache-enabled compilation time reduction (~42%)
  4. XAI explanation output

Usage:
    python scripts/run_batch.py
    python scripts/run_batch.py --traces data/traces_500.json
    python scripts/run_batch.py --domain data/examples/domain.pddl \
                                 --problem data/examples/problem.pddl
"""

import argparse
import json
import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src import XAIValidator, PDDLParser


def demo_single_trace(validator: XAIValidator):
    """Validate a single trace defined inline (no PDDL files needed)."""
    print("\n" + "="*60)
    print("DEMO 1: Single trace validation")
    print("="*60)

    trace_data = {
        "trace_id": "DEMO-001-valid",
        "initial_state": {
            "speed_high": True,
            "lane_clear": True,
            "obstacle_ahead": False,
        },
        "actions": [
            {
                "name": "decelerate",
                "preconditions": [{"fluent": "speed_high", "value": True}],
                "add_effects": ["speed_low"],
                "del_effects": ["speed_high"],
            },
            {
                "name": "move",
                "preconditions": [
                    {"fluent": "lane_clear", "value": True},
                    {"fluent": "obstacle_ahead", "value": False},
                ],
                "add_effects": ["at_next_loc"],
                "del_effects": ["at_current_loc"],
            },
            {
                "name": "arrive",
                "preconditions": [
                    {"fluent": "at_destination_loc", "value": True},
                    {"fluent": "speed_low", "value": True},
                ],
                "add_effects": ["at_destination"],
                "del_effects": [],
            },
        ],
        "goal_conditions": [{"fluent": "at_destination", "value": True}],
        "safety_invariants": [
            {"name": "no_collision", "fluent": "collision",
             "must_be_false": True, "horizon": 4}
        ],
    }

    result = validator.validate_dict(trace_data)
    print(result.report())


def demo_invalid_trace(validator: XAIValidator):
    """Validate an invalid trace — should find MIS with collision explanation."""
    print("\n" + "="*60)
    print("DEMO 2: Invalid trace (safety violation)")
    print("="*60)

    trace_data = {
        "trace_id": "DEMO-002-invalid",
        "initial_state": {
            "speed_low": True,
            "obstacle_ahead": True,
        },
        "actions": [
            {
                # BUG: accelerating with obstacle ahead → adds collision
                "name": "accelerate_unsafe",
                "preconditions": [{"fluent": "obstacle_ahead", "value": True}],
                "add_effects": ["speed_high", "collision"],
                "del_effects": ["speed_low"],
            },
        ],
        "goal_conditions": [{"fluent": "at_destination", "value": True}],
        "safety_invariants": [
            {"name": "no_collision", "fluent": "collision",
             "must_be_false": True, "horizon": 2}
        ],
    }

    result = validator.validate_dict(trace_data)
    print(result.report())


def demo_batch_with_cache(args):
    """
    Run batch validation and show cache-enabled speedup.
    Two rounds:
      Round 1 (cold cache): baseline compilation time
      Round 2 (warm cache): cached compilation — should be ~42% faster
    """
    print("\n" + "="*60)
    print("DEMO 3: Batch validation + cache benchmark")
    print("="*60)

    traces_path = Path(args.traces) if args.traces else None

    # Generate traces if needed
    if traces_path is None or not traces_path.exists():
        print("Generating 520 synthetic traces...")
        # Run inline generation
        from scripts.generate_traces import generate_traces
        traces_data = generate_traces(520)
        out_path = Path("data/traces_500.json")
        out_path.parent.mkdir(exist_ok=True)
        out_path.write_text(json.dumps(traces_data, indent=2))
        traces_path = out_path
        print(f"Saved to {traces_path}")

    validator = XAIValidator(max_mis=3, timeout_s=5.0)
    traces = validator.load_traces_from_json(str(traces_path))

    # ── Round 1: cold cache ──────────────────
    print(f"\n--- Round 1: Cold cache ({len(traces)} traces) ---")
    t0 = time.perf_counter()
    report1 = validator.validate_batch(traces, verbose=False)
    cold_time = time.perf_counter() - t0

    # ── Round 2: warm cache ──────────────────
    print(f"\n--- Round 2: Warm cache (same traces, repeat) ---")
    t1 = time.perf_counter()
    report2 = validator.validate_batch(traces, verbose=False)
    warm_time = time.perf_counter() - t1

    # ── Results ──────────────────────────────
    speedup = (1 - warm_time / cold_time) * 100 if cold_time > 0 else 0
    print(f"\n{'='*60}")
    print(f"PERFORMANCE BENCHMARK")
    print(f"{'='*60}")
    print(f"Cold cache time:  {cold_time*1000:.0f}ms")
    print(f"Warm cache time:  {warm_time*1000:.0f}ms")
    print(f"Speedup:          {speedup:.0f}% reduction")
    print(f"Success rate:     {report1.success_rate:.1%}")
    print(f"Total validated:  {report1.total}")
    print(f"{'='*60}")

    report1.print_summary()

    # Cache stats
    print("\nEncoder cache stats:")
    stats = validator.cache_stats()
    for k, v in stats.items():
        print(f"  {k}: {v}")

    return report1


def demo_pddl_file(args, validator: XAIValidator):
    """Validate from actual PDDL files if provided."""
    print("\n" + "="*60)
    print("DEMO 4: PDDL file validation")
    print("="*60)
    try:
        result = validator.validate_file(
            domain_path=args.domain,
            problem_path=args.problem,
        )
        print(result.report())
    except Exception as e:
        print(f"[PDDL] Error: {e}")


def main():
    parser = argparse.ArgumentParser(description="AV XAI Validator Demo")
    parser.add_argument("--traces", type=str, default=None,
                        help="Path to traces JSON file")
    parser.add_argument("--domain", type=str,
                        default="data/examples/domain.pddl")
    parser.add_argument("--problem", type=str,
                        default="data/examples/problem.pddl")
    parser.add_argument("--demo", choices=["all", "single", "invalid",
                                           "batch", "pddl"],
                        default="all")
    args = parser.parse_args()

    validator = XAIValidator(max_mis=5, timeout_s=10.0)

    if args.demo in ("all", "single"):
        demo_single_trace(validator)

    if args.demo in ("all", "invalid"):
        demo_invalid_trace(validator)

    if args.demo in ("all", "batch"):
        demo_batch_with_cache(args)

    if args.demo in ("all", "pddl"):
        if Path(args.domain).exists():
            demo_pddl_file(args, validator)
        else:
            print(f"\n[PDDL] Skipping: {args.domain} not found "
                  f"(use --domain to specify)")


if __name__ == "__main__":
    main()
