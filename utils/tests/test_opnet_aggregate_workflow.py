#!/usr/bin/env python3
"""End-to-end tests for run-opnet-aggregate-differential.py."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "utils" / "run-opnet-aggregate-differential.py"
EXTRACTOR_TEST = Path(__file__).with_name("test_opnet_ov_extractor.py")


def load_module(name: str, path: Path):
    specification = importlib.util.spec_from_file_location(name, path)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


WORKFLOW = load_module("csr_opnet_aggregate_workflow", SCRIPT)
OV_FIXTURE = load_module("csr_opnet_ov_fixture_helpers", EXTRACTOR_TEST)


SCENARIO_COLUMNS = (
    "record",
    "schema",
    "scenario",
    "source_sha256",
    "duration_s",
    "seed",
    "tmm",
    "coordinate_scale_m_per_unit",
    "node_id",
    "name",
    "node_type",
    "x_m",
    "y_m",
    "height_m",
    "min_speed_kbps",
    "max_speed_kbps",
    "min_power_dbm",
    "max_power_dbm",
    "link_margin_db",
    "ecc_threshold",
    "rx_frequency_hz",
    "tx_frequency_hz",
    "interarrival_s",
    "packet_bytes",
    "start_s",
    "flow_src",
    "flow_dst",
    "flow_start_s",
    "flow_interval_s",
    "flow_packet_bytes",
    "flow_dscp",
)


def write_scenario(
    path: Path,
    duration_s: float = 60000.0,
    seed: int | float | str = 128,
    source_sha256: str = "0" * 64,
) -> None:
    rows = [
        {
            "record": "run",
            "schema": WORKFLOW.SCENARIO_SCHEMA,
            "scenario": "fixture",
            "source_sha256": source_sha256,
            "duration_s": str(duration_s),
            "seed": str(seed),
            "tmm": "0",
            "coordinate_scale_m_per_unit": "1",
        },
        {
            "record": "node",
            "schema": WORKFLOW.SCENARIO_SCHEMA,
            "node_id": "1",
            "name": "gateway",
            "node_type": "gateway",
            "x_m": "0",
            "y_m": "0",
            "height_m": "1",
            "min_speed_kbps": "8",
            "max_speed_kbps": "128",
            "min_power_dbm": "-36",
            "max_power_dbm": "33",
            "link_margin_db": "12",
            "ecc_threshold": "0.1",
            "rx_frequency_hz": "400000000",
            "tx_frequency_hz": "400000000",
        },
        {
            "record": "node",
            "schema": WORKFLOW.SCENARIO_SCHEMA,
            "node_id": "2",
            "name": "ordinary",
            "node_type": "ordinary",
            "x_m": "1",
            "y_m": "0",
            "height_m": "1",
            "min_speed_kbps": "8",
            "max_speed_kbps": "128",
            "min_power_dbm": "-36",
            "max_power_dbm": "33",
            "link_margin_db": "12",
            "ecc_threshold": "0.1",
            "rx_frequency_hz": "400000000",
            "tx_frequency_hz": "400000000",
        },
        {
            "record": "flow",
            "schema": WORKFLOW.SCENARIO_SCHEMA,
            "flow_src": "2",
            "flow_dst": "1",
            "flow_start_s": "1",
            "flow_interval_s": "600",
            "flow_packet_bytes": "16",
            "flow_dscp": "0",
        },
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SCENARIO_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_fake_runner(path: Path, mode: str = "exact") -> None:
    if mode not in {"exact", "fifo", "unmatched"}:
        raise ValueError(mode)
    program = textwrap.dedent(
        """\
            #!/usr/bin/env python3
            import csv
            from pathlib import Path
            import sys

            mode = __MODE__

            options = {}
            for argument in sys.argv[1:]:
                if not argument.startswith("--") or "=" not in argument:
                    raise SystemExit(64)
                name, value = argument[2:].split("=", 1)
                options[name] = value
            required = {
                "scenario", "trace", "stop", "flowLimit", "dutyCycling",
                "gatewayDiscovery", "opnetAppGating", "aggregateTraceOnly",
                "quietModelLogs",
            }
            if set(options) != required:
                raise SystemExit(65)
            if options["aggregateTraceOnly"] != "1" or options["quietModelLogs"] != "1":
                raise SystemExit(66)

            fields = (
                "schema", "event_index", "time_s", "event", "src", "dst",
                "sequence", "size_bytes", "reason",
            )
            stop = float(options["stop"])
            trace = Path(options["trace"])
            with trace.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\\n")
                writer.writeheader()
                event_index = 0
                for bucket_end in range(600, int(stop) + 1, 600):
                    sequence = str(bucket_end // 600)
                    send_sequence = "" if mode == "fifo" else sequence
                    delivery_sequence = (
                        "" if mode == "fifo" else
                        ("wrong-" + sequence if mode == "unmatched" else sequence)
                    )
                    writer.writerow({
                        "schema": "csr-differential-trace-v1",
                        "event_index": event_index,
                        "time_s": bucket_end - 2,
                        "event": "app_send",
                        "src": "2", "dst": "1", "sequence": send_sequence,
                        "size_bytes": "16", "reason": "",
                    })
                    event_index += 1
                    writer.writerow({
                        "schema": "csr-differential-trace-v1",
                        "event_index": event_index,
                        "time_s": bucket_end - 1,
                        "event": "nwk_delivery",
                        "src": "2", "dst": "1", "sequence": delivery_sequence,
                        "size_bytes": "16", "reason": "",
                    })
                    event_index += 1
            print(f"fake runner wrote {event_index} events")
        """
    ).replace("__MODE__", repr(mode))
    path.write_text(program, encoding="utf-8")
    os.chmod(path, 0o755)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class AggregateWorkflowTests(unittest.TestCase):
    def prepare(
        self,
        root: Path,
        values: list[float] | None = None,
        runner_mode: str = "exact",
    ):
        scenario = root / "scenario.csv"
        ov = root / "fixture-DES-1.ov"
        pb = root / "fixture.pb.m"
        runner = root / "fake-runner"
        write_scenario(scenario)
        ov.write_bytes(
            OV_FIXTURE.synthetic_ov([1.0] * 100 if values is None else values)
        )
        pb.write_bytes(OV_FIXTURE.synthetic_pb())
        write_fake_runner(runner, runner_mode)
        return scenario, ov, pb, runner

    def command(
        self,
        scenario: Path,
        ov: Path,
        runner: Path,
        output: Path,
        *extra: str,
    ) -> list[str]:
        probe_definition = ov.parent / "fixture.pb.m"
        return [
            sys.executable,
            str(SCRIPT),
            "--scenario",
            str(scenario),
            "--runner",
            str(runner),
            "--opnet-ov",
            str(ov),
            "--probe-definition",
            str(probe_definition),
            "--expected-opnet-ov-sha256",
            sha256(ov),
            "--expected-probe-definition-sha256",
            sha256(probe_definition),
            "--output-dir",
            str(output),
            "--statistic",
            "Sink.End-to-End Delay (seconds)",
            *extra,
        ]

    def test_fake_runner_end_to_end_records_reproducible_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, pb, runner = self.prepare(root)
            output = root / "run"
            completed = subprocess.run(
                self.command(
                    scenario,
                    ov,
                    runner,
                    output,
                    "--stop",
                    "60000",
                    "--flow-limit",
                    "7",
                    "--no-duty-cycling",
                    "--no-gateway-discovery",
                    "--no-opnet-app-gating",
                ),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)

            manifest_path = output / WORKFLOW.MANIFEST_NAME
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["schema"], WORKFLOW.RUN_SCHEMA)
            self.assertEqual(manifest["status"], "selected_comparison_passed")
            self.assertEqual(manifest["evidence_scope"], WORKFLOW.EVIDENCE_SCOPE)
            self.assertEqual(manifest["profile"], "diagnostic")
            self.assertEqual(
                manifest["noncertifying_overrides"],
                [
                    "statistics_not_complete_core_set",
                    "flow_limit",
                    "duty_cycling_disabled",
                    "gateway_discovery_disabled",
                    "opnet_app_gating_disabled",
                ],
            )
            self.assertIsNone(manifest["failure"])
            self.assertEqual(
                manifest["evidence_identity"],
                {
                    "opnet_ov": {
                        "expected_sha256": sha256(ov),
                        "actual_sha256": sha256(ov),
                        "match": True,
                    },
                    "probe_definition": {
                        "expected_sha256": sha256(pb),
                        "actual_sha256": sha256(pb),
                        "match": True,
                    },
                },
            )
            configuration = manifest["configuration"]
            self.assertEqual(configuration["stop_s"], 60000.0)
            self.assertEqual(configuration["comparison_time_start_s"], 600.0)
            self.assertEqual(configuration["comparison_time_stop_s"], 60000.0)
            self.assertEqual(configuration["opnet_axis"]["bucket_width_s"], 600.0)
            self.assertEqual(
                configuration["identities"]["canonical_scenario"],
                {
                    "scenario": "fixture",
                    "duration_s": 60000.0,
                    "seed": 128,
                    "source_sha256": "0" * 64,
                },
            )
            opnet_identity = configuration["identities"]["opnet_execution"]
            self.assertEqual(opnet_identity["scenario"], "fixture")
            self.assertEqual(opnet_identity["duration_s"], 60000.0)
            self.assertEqual(opnet_identity["seed"], 128)
            self.assertEqual(
                opnet_identity["source_identity"]["basis"],
                "modeler_des_result_filename",
            )
            self.assertEqual(
                configuration["legacy_trace_size_exclusion_bits"], 0
            )
            self.assertEqual(
                configuration["statistics"],
                ["Sink.End-to-End Delay (seconds)"],
            )
            self.assertEqual(
                configuration["runner"],
                {
                    "aggregate_trace_only": True,
                    "duty_cycling": False,
                    "flow_limit": 7,
                    "gateway_discovery": False,
                    "opnet_app_gating": False,
                    "quiet_model_logs": True,
                },
            )
            self.assertTrue(
                all(stage["exit_code"] == 0 for stage in manifest["stages"].values())
            )
            self.assertNotIn(
                "--scenario", manifest["stages"]["extract_opnet"]["command"]
            )
            self.assertEqual(
                manifest["stages"]["aggregate_ns3"]["exact_sequence_validation"],
                {
                    "matched_count": 100,
                    "matched_by_method": {"exact_sequence": 100},
                    "unmatched_delivery_count": 0,
                    "matched_size_compared_count": 100,
                    "matched_size_mismatch_count": 0,
                },
            )
            runner_command = manifest["stages"]["run_ns3"]["command"]
            self.assertIn("--stop=60000", runner_command)
            self.assertIn("--aggregateTraceOnly=1", runner_command)
            self.assertIn("--quietModelLogs=1", runner_command)
            compare_command = manifest["stages"]["compare"]["command"]
            self.assertEqual(
                compare_command[compare_command.index("--time-stop") + 1], "60000"
            )
            aggregate_command = manifest["stages"]["aggregate_ns3"]["command"]
            adjustment_option = "--legacy-trace-size-exclusion-bits"
            self.assertEqual(
                aggregate_command[aggregate_command.index(adjustment_option) + 1],
                "0",
            )
            self.assertIn("--require-zero-size-mismatches", aggregate_command)
            self.assertTrue(
                configuration["require_zero_matched_packet_size_mismatches"]
            )

            report = json.loads(
                (output / "aggregate-report.json").read_text(encoding="utf-8")
            )
            self.assertTrue(report["pass"])
            self.assertEqual(report["counts"]["matched_points"], 100)
            self.assertEqual(report["filters"]["time_stop_s"], 60000.0)
            for identity, artifact in manifest["artifacts"].items():
                self.assertIsNotNone(artifact, identity)
                artifact_path = output / artifact["path"]
                self.assertEqual(artifact["sha256"], sha256(artifact_path))

    def test_comparison_exit_status_is_returned_without_hiding_report(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root, [2.0] * 100)
            output = root / "run"
            completed = subprocess.run(
                self.command(scenario, ov, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 1)
            manifest = json.loads(
                (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["status"], "selected_comparison_failed")
            self.assertEqual(manifest["stages"]["compare"]["exit_code"], 1)
            self.assertEqual(manifest["failure"]["exit_code"], 1)
            report = json.loads(
                (output / "aggregate-report.json").read_text(encoding="utf-8")
            )
            self.assertFalse(report["pass"])
            self.assertEqual(report["counts"]["numeric_mismatches"], 100)

    def test_default_stop_is_the_imported_scenario_duration(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            completed = subprocess.run(
                self.command(scenario, ov, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            manifest = json.loads(
                (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["configuration"]["stop_s"], 60000.0)
            self.assertIn(
                "--stop=60000", manifest["stages"]["run_ns3"]["command"]
            )

    def test_partial_extraction_stops_before_runner(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, _ov, _pb, runner = self.prepare(root)
            partial = root / "fixture-DES-2.ov"
            partial.write_bytes(
                OV_FIXTURE.synthetic_ov(None, include_duration=False)
            )
            output = root / "run"
            completed = subprocess.run(
                self.command(scenario, partial, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 3)
            manifest = json.loads(
                (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["failure"]["stage"], "extract_opnet")
            self.assertEqual(manifest["failure"]["exit_code"], 3)
            self.assertNotIn("run_ns3", manifest["stages"])

    def test_shorter_stop_is_rejected_without_creating_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            completed = subprocess.run(
                self.command(scenario, ov, runner, output, "--stop", "600"),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("authoritative prefix runs", completed.stderr)
            self.assertFalse(output.exists())

    def test_later_time_start_is_rejected_before_runner(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            completed = subprocess.run(
                self.command(
                    scenario, ov, runner, output, "--time-start", "1200"
                ),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("authoritative suffix comparisons", completed.stderr)
            manifest = json.loads(
                (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            self.assertNotIn("run_ns3", manifest["stages"])

    def test_wrong_ov_scenario_name_cannot_be_relabelled(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            wrong = root / "different-DES-1.ov"
            wrong.write_bytes(ov.read_bytes())
            output = root / "run"
            completed = subprocess.run(
                self.command(scenario, wrong, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("identifies scenario 'different'", completed.stderr)
            self.assertFalse(output.exists())

    def test_plain_ov_name_is_probe_corroborated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            plain = root / "fixture.ov"
            plain.write_bytes(ov.read_bytes())
            accepted_output = root / "accepted"
            accepted = subprocess.run(
                self.command(scenario, plain, runner, accepted_output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(accepted.returncode, 0, accepted.stderr)
            manifest = json.loads(
                (accepted_output / WORKFLOW.MANIFEST_NAME).read_text(
                    encoding="utf-8"
                )
            )
            basis = manifest["configuration"]["identities"]["opnet_execution"][
                "source_identity"
            ]["basis"]
            self.assertEqual(
                basis, "plain_ov_filename_corroborated_by_probe_definition"
            )

    def test_probe_and_expected_hash_arguments_are_required(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            command = [
                sys.executable,
                str(SCRIPT),
                "--scenario",
                str(scenario),
                "--runner",
                str(runner),
                "--opnet-ov",
                str(ov),
                "--output-dir",
                str(output),
            ]
            completed = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            for option in (
                "--probe-definition",
                "--expected-opnet-ov-sha256",
                "--expected-probe-definition-sha256",
            ):
                self.assertIn(option, completed.stderr)
            self.assertFalse(output.exists())

    def test_expected_hash_mismatch_precedes_output_mutation(self) -> None:
        flags = (
            "--expected-opnet-ov-sha256",
            "--expected-probe-definition-sha256",
        )
        for flag in flags:
            with self.subTest(flag=flag), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                scenario, ov, _pb, runner = self.prepare(root)
                output = root / "run"
                output.mkdir()
                sentinel = output / "ns3-trace.csv"
                sentinel.write_text("preserve\n", encoding="utf-8")
                command = self.command(scenario, ov, runner, output)
                command[command.index(flag) + 1] = "f" * 64
                completed = subprocess.run(
                    command,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                self.assertEqual(completed.returncode, 2)
                self.assertIn("SHA-256 mismatch", completed.stderr)
                self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve\n")
                self.assertFalse((output / WORKFLOW.MANIFEST_NAME).exists())

    def test_expected_hash_arguments_normalize_uppercase(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            command = self.command(scenario, ov, runner, output)
            for flag in (
                "--expected-opnet-ov-sha256",
                "--expected-probe-definition-sha256",
            ):
                index = command.index(flag) + 1
                command[index] = command[index].upper()
            completed = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            manifest = json.loads(
                (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            for identity in manifest["evidence_identity"].values():
                self.assertEqual(
                    identity["expected_sha256"], identity["expected_sha256"].lower()
                )

    def test_unowned_output_directory_is_untouched(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            output.mkdir()
            managed = output / "ns3-trace.csv"
            unrelated = output / "notes.txt"
            managed.write_text("keep managed sentinel\n", encoding="utf-8")
            unrelated.write_text("keep unrelated sentinel\n", encoding="utf-8")
            completed = subprocess.run(
                self.command(scenario, ov, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("not workflow-owned", completed.stderr)
            self.assertEqual(
                managed.read_text(encoding="utf-8"), "keep managed sentinel\n"
            )
            self.assertEqual(
                unrelated.read_text(encoding="utf-8"), "keep unrelated sentinel\n"
            )
            self.assertFalse((output / WORKFLOW.MANIFEST_NAME).exists())

    def test_preflight_failure_does_not_clear_owned_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            output.mkdir()
            ownership = output / WORKFLOW.MANIFEST_NAME
            ownership_text = json.dumps({"schema": WORKFLOW.RUN_SCHEMA}) + "\n"
            ownership.write_text(ownership_text, encoding="utf-8")
            managed = output / "ns3-trace.csv"
            managed.write_text("stale but preserved\n", encoding="utf-8")
            scenario.unlink()

            completed = subprocess.run(
                self.command(scenario, ov, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("scenario does not exist", completed.stderr)
            self.assertEqual(managed.read_text(encoding="utf-8"), "stale but preserved\n")
            self.assertEqual(ownership.read_text(encoding="utf-8"), ownership_text)

    def test_input_collision_is_rejected_before_output_handling(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            output.mkdir()
            scenario = output / "ns3-trace.csv"
            write_scenario(scenario)
            before = scenario.read_bytes()
            completed = subprocess.run(
                self.command(scenario, ov, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("collides with managed output target", completed.stderr)
            self.assertEqual(scenario.read_bytes(), before)
            self.assertFalse((output / WORKFLOW.MANIFEST_NAME).exists())

    def test_owned_rerun_replaces_only_managed_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            first = subprocess.run(
                self.command(scenario, ov, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(first.returncode, 0, first.stderr)
            unrelated = output / "notes.txt"
            unrelated.write_text("user note\n", encoding="utf-8")
            (output / "ns3-trace.csv").write_text("stale\n", encoding="utf-8")
            second = subprocess.run(
                self.command(scenario, ov, runner, output),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(second.returncode, 0, second.stderr)
            self.assertEqual(unrelated.read_text(encoding="utf-8"), "user note\n")
            self.assertNotEqual(
                (output / "ns3-trace.csv").read_text(encoding="utf-8"), "stale\n"
            )

    def test_relative_runner_is_resolved_instead_of_searched_on_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, _runner = self.prepare(root)
            output = root / "run"
            completed = subprocess.run(
                self.command(scenario, ov, Path("fake-runner"), output),
                cwd=root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            manifest = json.loads(
                (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            self.assertEqual(
                manifest["stages"]["run_ns3"]["command"][0],
                str((root / "fake-runner").resolve()),
            )

    def test_workflow_rejects_fifo_and_unmatched_delivery_provenance(self) -> None:
        cases = {
            "fifo": "non-exact delay matching",
            "unmatched": "unmatched deliveries",
        }
        for mode, message in cases.items():
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                scenario, ov, _pb, runner = self.prepare(
                    root, runner_mode=mode
                )
                output = root / "run"
                completed = subprocess.run(
                    self.command(scenario, ov, runner, output),
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                self.assertEqual(completed.returncode, 2)
                self.assertIn(message, completed.stderr)
                manifest = json.loads(
                    (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
                )
                self.assertEqual(manifest["failure"]["stage"], "aggregate_ns3")
                self.assertNotIn("compare", manifest["stages"])

    def test_seed_and_duration_must_match_opnet_execution(self) -> None:
        cases = (
            ("seed", {"seed": 129}, "seed does not exactly match"),
            (
                "duration",
                {"duration_s": 1200.0},
                "duration does not exactly match",
            ),
        )
        for label, overrides, message in cases:
            with self.subTest(label=label), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                scenario, ov, _pb, runner = self.prepare(root)
                write_scenario(scenario, **overrides)
                output = root / "run"
                completed = subprocess.run(
                    self.command(scenario, ov, runner, output),
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                self.assertEqual(completed.returncode, 2)
                self.assertIn(message, completed.stderr)
                manifest = json.loads(
                    (output / WORKFLOW.MANIFEST_NAME).read_text(encoding="utf-8")
                )
                self.assertEqual(manifest["failure"]["stage"], "extract_opnet")
                self.assertNotIn("run_ns3", manifest["stages"])

    def test_seed_must_fit_ns3_rng_domain_before_output_creation(self) -> None:
        for seed in (0, WORKFLOW.NS3_RNG_SEED_MAX + 1, 4294967295):
            with self.subTest(seed=seed), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                scenario, ov, _pb, runner = self.prepare(root)
                write_scenario(scenario, seed=seed)
                output = root / "run"
                completed = subprocess.run(
                    self.command(scenario, ov, runner, output),
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                self.assertEqual(completed.returncode, 2)
                self.assertIn(
                    f"seed must be a finite integer in 1..{WORKFLOW.NS3_RNG_SEED_MAX}",
                    completed.stderr,
                )
                self.assertFalse(output.exists())

    def test_seed_ns3_rng_boundaries_are_valid_scenario_identities(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.csv"
            for seed in (1, WORKFLOW.NS3_RNG_SEED_MAX):
                with self.subTest(seed=seed):
                    write_scenario(path, seed=seed)
                    self.assertEqual(WORKFLOW.load_scenario_run(path)["seed"], seed)

    def test_inconsistent_vector_axes_are_rejected(self) -> None:
        vector = {
            "status": "complete",
            "statistic": "one",
            "time_axis": {"encoded_start_s": 0.0, "bucket_width_s": 10.0},
            "buckets": [{"time_s": 10.0}, {"time_s": 20.0}],
        }
        different = {
            "status": "complete",
            "statistic": "two",
            "time_axis": {"encoded_start_s": 0.0, "bucket_width_s": 5.0},
            "buckets": [{"time_s": 5.0}, {"time_s": 10.0}],
        }
        with self.assertRaisesRegex(WORKFLOW.WorkflowError, "inconsistent"):
            WORKFLOW.validate_vector_axes(
                {"status": "complete", "vectors": [vector, different]}
            )

    def test_canonical_profile_requires_complete_exact_behavior_set(self) -> None:
        arguments = type(
            "Arguments",
            (),
            {
                "abs_tolerance": 0.0,
                "rel_tolerance": 0.0,
                "time_tolerance": 0.0,
                "flow_limit": 0,
                "no_duty_cycling": False,
                "no_gateway_discovery": False,
                "no_opnet_app_gating": False,
                "legacy_trace_size_exclusion_bits": 0,
            },
        )()
        profile, overrides = WORKFLOW._comparison_profile(
            WORKFLOW.DEFAULT_STATISTICS, arguments
        )
        self.assertEqual(profile, "canonical")
        self.assertEqual(overrides, [])


if __name__ == "__main__":
    unittest.main()
