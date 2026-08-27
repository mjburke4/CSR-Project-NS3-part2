#!/usr/bin/env python3
"""Focused tests for the OPNET/ns-3 aggregate-series comparator."""

from __future__ import annotations

import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "utils" / "compare-opnet-ns3-aggregates.py"


def load_script():
    specification = importlib.util.spec_from_file_location(
        "csr_aggregate_comparator", SCRIPT
    )
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


COMPARATOR = load_script()
FIELDS = (
    "schema",
    "scenario",
    "statistic",
    "time_s",
    "value",
    "source",
    "unit",
    "aggregation",
    "raw_value",
    "value_status",
)


class AggregateComparatorTests(unittest.TestCase):
    def write_rows(self, path: Path, rows: list[dict[str, object]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
            writer.writeheader()
            for row in rows:
                complete = {
                    "schema": COMPARATOR.AGGREGATE_SCHEMA,
                    "scenario": "two-node",
                    "statistic": "Sink.Traffic Received (packets/sec)",
                    "time_s": 1.0,
                    "value": 10.0,
                    "source": "opnet",
                    "unit": "packets/s",
                    "aggregation": "bucket_sum_per_second",
                    "raw_value": "",
                    "value_status": "observed",
                }
                complete.update(row)
                writer.writerow(complete)

    def load(self, rows: list[dict[str, object]]):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "aggregate.csv"
        self.write_rows(path, rows)
        return COMPARATOR.read_aggregates([path])

    def compare(
        self,
        rows: list[dict[str, object]],
        absolute: float = 0.0,
        relative: float = 0.0,
        time: float = 0.0,
    ):
        return COMPARATOR.compare(
            self.load(rows), absolute, relative, time, max_differences=100
        )

    def test_exact_pass(self) -> None:
        report = self.compare(
            [
                {},
                {"source": "ns3"},
                {"time_s": 2.0, "value": 20.0},
                {"source": "ns3", "time_s": 2.0, "value": 20.0},
            ]
        )
        self.assertTrue(report["pass"])
        self.assertEqual(report["counts"]["matched_points"], 2)
        self.assertEqual(report["counts"]["numeric_mismatches"], 0)

    def test_absolute_relative_and_time_tolerances_pass(self) -> None:
        report = self.compare(
            [
                {"value": 100.0, "time_s": 5.0},
                {"source": "ns3", "value": 101.0, "time_s": 5.05},
            ],
            absolute=0.1,
            relative=0.02,
            time=0.1,
        )
        self.assertTrue(report["pass"])
        self.assertAlmostEqual(
            report["series"][0]["maximum_time_delta_s"], 0.05
        )

    def test_input_order_does_not_change_alignment(self) -> None:
        report = self.compare(
            [
                {"time_s": 2.0, "value": 20.0},
                {"source": "ns3", "time_s": 1.0, "value": 10.0},
                {"time_s": 1.0, "value": 10.0},
                {"source": "ns3", "time_s": 2.0, "value": 20.0},
            ]
        )
        self.assertTrue(report["pass"])
        self.assertEqual(report["counts"]["matched_points"], 2)

    def test_missing_and_extra_points_are_distinct(self) -> None:
        report = self.compare(
            [
                {"time_s": 1.0},
                {"time_s": 2.0, "value": 20.0},
                {"source": "ns3", "time_s": 2.0, "value": 20.0},
                {"source": "ns3", "time_s": 3.0, "value": 30.0},
            ]
        )
        self.assertFalse(report["pass"])
        self.assertEqual(report["counts"]["missing_in_ns3"], 1)
        self.assertEqual(report["counts"]["extra_in_ns3"], 1)
        self.assertEqual(
            {difference["kind"] for difference in report["differences"]},
            {"missing_in_ns3", "extra_in_ns3"},
        )

    def test_numeric_mismatch_is_reported(self) -> None:
        report = self.compare(
            [{"value": 10.0}, {"source": "ns3", "value": 12.0}],
            absolute=1.0,
        )
        self.assertFalse(report["pass"])
        self.assertEqual(report["counts"]["numeric_mismatches"], 1)
        difference = report["differences"][0]
        self.assertEqual(difference["kind"], "numeric_mismatch")
        self.assertEqual(difference["absolute_delta"], 2.0)

    def test_extreme_finite_values_do_not_overflow_into_a_false_pass(self) -> None:
        report = self.compare(
            [
                {"value": 1.0e308},
                {"source": "ns3", "value": -1.0e308},
            ],
            relative=1.8,
        )
        self.assertFalse(report["pass"])
        self.assertEqual(report["counts"]["numeric_mismatches"], 1)
        difference = report["differences"][0]
        self.assertTrue(difference["absolute_delta_capped"])
        self.assertTrue(difference["allowed_absolute_delta_capped"])
        self.assertAlmostEqual(difference["relative_delta"], 2.0)
        self.assertTrue(
            all(
                math.isfinite(value)
                for value in (
                    difference["absolute_delta"],
                    difference["relative_delta"],
                    difference["allowed_absolute_delta"],
                    report["series"][0]["maximum_absolute_delta"],
                    report["series"][0]["maximum_relative_delta"],
                )
            )
        )
        # Strict JSON encoding is also an invariant: no NaN/Infinity tokens.
        json.dumps(report, allow_nan=False)

    def test_semantic_mismatch_is_never_compared_numerically(self) -> None:
        report = self.compare(
            [
                {"unit": "s", "aggregation": "bucket_sample_mean"},
                {
                    "source": "ns3",
                    "unit": "ms",
                    "aggregation": "bucket_sample_mean",
                },
            ]
        )
        self.assertFalse(report["pass"])
        self.assertEqual(report["counts"]["noncomparable_series"], 1)
        self.assertEqual(report["counts"]["numeric_mismatches"], 0)
        self.assertEqual(report["differences"][0]["kind"], "noncomparable_series")

    def test_filter_can_select_only_source_comparable_statistics(self) -> None:
        points = self.load(
            [
                {},
                {"source": "ns3"},
                {
                    "statistic": "MAC.Transmit Queue Size (packets)",
                    "aggregation": "bucket_sample_mean",
                },
            ]
        )
        unfiltered = COMPARATOR.compare(points, 0.0, 0.0, 0.0, 100)
        self.assertFalse(unfiltered["pass"])
        self.assertEqual(unfiltered["counts"]["unmatched_opnet_series"], 1)
        selected = COMPARATOR.filter_points(
            points,
            set(),
            {"Sink.Traffic Received (packets/sec)"},
        )
        report = COMPARATOR.compare(selected, 0.0, 0.0, 0.0, 100)
        self.assertTrue(report["pass"])
        self.assertEqual(report["counts"]["comparable_series"], 1)

    def test_filter_can_select_a_closed_time_window(self) -> None:
        points = self.load(
            [
                {"time_s": 1.0},
                {"source": "ns3", "time_s": 1.0},
                {"time_s": 2.0, "value": 20.0},
                {"source": "ns3", "time_s": 2.0, "value": 20.0},
                {"time_s": 3.0, "value": 30.0},
                {"source": "ns3", "time_s": 3.0, "value": 30.0},
            ]
        )
        selected = COMPARATOR.filter_points(
            points,
            set(),
            set(),
            time_start_s=2.0,
            time_stop_s=2.0,
        )
        self.assertEqual(len(selected), 2)
        self.assertEqual({point.time_s for point in selected}, {2.0})

    def test_modeler_missing_bucket_is_skipped_and_reported(self) -> None:
        report = self.compare(
            [
                {
                    "value": "",
                    "raw_value": "2e+100",
                    "value_status": "missing",
                },
                {"source": "ns3", "value": 7.0},
                {"time_s": 2.0, "value": 8.0},
                {"source": "ns3", "time_s": 2.0, "value": 8.0},
            ]
        )
        self.assertTrue(report["pass"])
        self.assertEqual(report["counts"]["skipped_missing_values"], 1)
        self.assertEqual(report["counts"]["numeric_points_compared"], 1)
        self.assertEqual(report["counts"]["numeric_mismatches"], 0)
        self.assertEqual(report["skipped"][0]["opnet_raw_value"], "2e+100")
        self.assertEqual(
            report["skipped"][0]["reason"], "opnet_reference_missing"
        )

    def test_ns3_missing_for_observed_reference_is_a_failure(self) -> None:
        report = self.compare(
            [
                {"value": 10.0},
                {
                    "source": "ns3",
                    "value": "",
                    "value_status": "missing",
                },
            ]
        )
        self.assertFalse(report["pass"])
        self.assertEqual(report["counts"]["missing_ns3_values"], 1)
        self.assertEqual(report["counts"]["skipped_missing_values"], 0)
        self.assertEqual(report["counts"]["numeric_mismatches"], 0)
        self.assertEqual(report["differences"][0]["kind"], "missing_ns3_value")
        self.assertFalse(report["series"][0]["pass"])

    def test_all_reference_values_missing_is_indeterminate(self) -> None:
        report = self.compare(
            [
                {
                    "value": "",
                    "raw_value": "2e+100",
                    "value_status": "missing",
                },
                {"source": "ns3", "value": 7.0},
            ]
        )
        self.assertFalse(report["pass"])
        self.assertEqual(report["counts"]["indeterminate_series"], 1)
        self.assertEqual(report["counts"]["skipped_missing_values"], 1)
        self.assertEqual(report["counts"]["numeric_points_compared"], 0)
        self.assertEqual(report["series"][0]["status"], "indeterminate")
        self.assertEqual(
            report["differences"][0]["kind"], "indeterminate_series"
        )

    def test_malformed_inputs_are_rejected(self) -> None:
        cases = (
            ({"source": "invalid"}, "source must be"),
            ({"value": "not-a-number"}, "value is not numeric"),
            ({"schema": ""}, "unsupported schema ''"),
            ({"schema": "unknown-v9"}, "unsupported schema"),
            ({"value": "", "value_status": "observed"}, "value is empty"),
        )
        for row, expected in cases:
            with self.subTest(row=row):
                with self.assertRaisesRegex(COMPARATOR.AggregateError, expected):
                    self.load([row])

    def test_missing_required_column_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.csv"
            path.write_text(
                "scenario,statistic,time_s,value,source\n"
                "case,metric,1,2,opnet\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                COMPARATOR.AggregateError, "missing required columns: schema"
            ):
                COMPARATOR.read_aggregates([path])

    def test_mixed_schema_rows_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mixed.csv"
            self.write_rows(
                path,
                [
                    {},
                    {"schema": "unknown-v9", "time_s": 2.0},
                ],
            )
            with self.assertRaisesRegex(
                COMPARATOR.AggregateError,
                r"mixed\.csv:3: unsupported schema 'unknown-v9'",
            ):
                COMPARATOR.read_aggregates([path])

    def test_cli_writes_summary_json_and_normalized_csv(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            input_path = directory_path / "input.csv"
            report_path = directory_path / "report.json"
            normalized_path = directory_path / "normalized.csv"
            self.write_rows(input_path, [{}, {"source": "ns3"}])
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(input_path),
                    "--report",
                    str(report_path),
                    "--normalized",
                    str(normalized_path),
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertTrue(report["pass"])
            self.assertEqual(report["schema"], COMPARATOR.REPORT_SCHEMA)
            self.assertEqual(len(report["inputs"]), 1)
            with normalized_path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["source"] for row in rows], ["opnet", "ns3"])

    def test_cli_rejects_input_and_output_path_collisions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            input_path = directory_path / "input.csv"
            self.write_rows(input_path, [{}, {"source": "ns3"}])
            original_input = input_path.read_bytes()

            cases = (
                (["--report", str(input_path)], "--report output"),
                (["--normalized", str(input_path)], "--normalized output"),
            )
            for extra_arguments, expected in cases:
                with self.subTest(arguments=extra_arguments):
                    completed = subprocess.run(
                        [sys.executable, str(SCRIPT), str(input_path), *extra_arguments],
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        check=False,
                    )
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(expected, completed.stdout)
                    self.assertIn("aliases aggregate input", completed.stdout)
                    self.assertEqual(input_path.read_bytes(), original_input)

            shared_output = directory_path / "comparison.json"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(input_path),
                    "--report",
                    str(shared_output),
                    "--normalized",
                    str(directory_path / "." / shared_output.name),
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("resolve to the same path", completed.stdout)
            self.assertFalse(shared_output.exists())

            symlink_path = directory_path / "input-alias.csv"
            symlink_path.symlink_to(input_path.name)
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(input_path),
                    "--report",
                    str(symlink_path),
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("aliases aggregate input", completed.stdout)
            self.assertEqual(input_path.read_bytes(), original_input)

    def test_cli_rejects_hard_link_output_collisions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            input_path = directory_path / "input.csv"
            self.write_rows(input_path, [{}, {"source": "ns3"}])
            original_input = input_path.read_bytes()

            report_link = directory_path / "report-hard-link"
            normalized_link = directory_path / "normalized-hard-link"
            report_link.hardlink_to(input_path)
            normalized_link.hardlink_to(input_path)
            for option, output_path in (
                ("--report", report_link),
                ("--normalized", normalized_link),
            ):
                with self.subTest(option=option):
                    completed = subprocess.run(
                        [
                            sys.executable,
                            str(SCRIPT),
                            str(input_path),
                            option,
                            str(output_path),
                        ],
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        check=False,
                    )
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn("aliases aggregate input", completed.stdout)
                    self.assertEqual(input_path.read_bytes(), original_input)

            shared_file = directory_path / "shared-output-data"
            shared_file.write_bytes(b"must remain unchanged\n")
            report_output = directory_path / "report-output"
            normalized_output = directory_path / "normalized-output"
            report_output.hardlink_to(shared_file)
            normalized_output.hardlink_to(shared_file)
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(input_path),
                    "--report",
                    str(report_output),
                    "--normalized",
                    str(normalized_output),
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("alias the same file", completed.stdout)
            self.assertEqual(shared_file.read_bytes(), b"must remain unchanged\n")


if __name__ == "__main__":
    unittest.main()
