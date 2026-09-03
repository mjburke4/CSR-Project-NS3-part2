#!/usr/bin/env python3
"""End-to-end tests for run-opnet-aggregate-differential.py."""

from __future__ import annotations

import csv
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap
import types
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
    "application_profile",
    "mac_profile",
    "hop_security_profile",
    "ack_envelope_profile",
    "source_executable_sha256",
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
    "flow_destination_mode",
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
    application_profile: str = "current-send-only",
    mac_profile: str | None = "current-fine-free-slot",
    hop_security_profile: str | None = "production-pairwise16",
    ack_envelope_profile: str | None = None,
    source_executable_sha256: str | None = None,
) -> None:
    if source_executable_sha256 is None:
        source_executable_sha256 = (
            WORKFLOW.HIST_ADB97C54_EXECUTABLE_SHA256
            if hop_security_profile
            == WORKFLOW.HOP_SECURITY_PROFILE_HIST_ADB97C54_BARE
            else ""
        )
    rows = [
        {
            "record": "run",
            "schema": WORKFLOW.SCENARIO_SCHEMA,
            "scenario": "fixture",
            "application_profile": application_profile,
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
            "flow_destination_mode": "fixed",
            "flow_start_s": "1",
            "flow_interval_s": "600",
            "flow_packet_bytes": "16",
            "flow_dscp": "0",
        },
    ]
    if mac_profile is not None:
        rows[0]["mac_profile"] = mac_profile
    if hop_security_profile is not None:
        rows[0]["hop_security_profile"] = hop_security_profile
    if ack_envelope_profile is not None:
        rows[0]["ack_envelope_profile"] = ack_envelope_profile
    if source_executable_sha256:
        rows[0]["source_executable_sha256"] = source_executable_sha256
    columns = tuple(
        column
        for column in SCENARIO_COLUMNS
        if not (
            (column == "mac_profile" and mac_profile is None)
            or (
                column == "hop_security_profile"
                and hop_security_profile is None
            )
            or (
                column == "ack_envelope_profile"
                and ack_envelope_profile is None
            )
            or (
                column == "source_executable_sha256"
                and not source_executable_sha256
            )
        )
    )
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
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
                "opnetAlignedDutyCycle",
                "gatewayDiscovery", "opnetAppGating", "aggregateTraceOnly",
                "quietModelLogs", "appDiagnostics",
            }
            if set(options) != required:
                raise SystemExit(65)
            if options["aggregateTraceOnly"] != "1" or options["quietModelLogs"] != "1":
                raise SystemExit(66)

            with Path(options["scenario"]).open(encoding="utf-8") as stream:
                scenario_rows = list(csv.DictReader(stream))
            application_profile = next(
                row.get("application_profile", "") or "unspecified"
                for row in scenario_rows
                if row["record"] == "run"
            )

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
            diagnostics = Path(options["appDiagnostics"])
            diagnostics.write_text(
                "schema,scenario,application_profile,flow_index,source,"
                "configured_destination,"
                "destination_mode,attempts,admitted,blocked_discovery,"
                "blocked_topology,blocked_gateway_route,blocked_destination,"
                "blocked_nsdp,first_admitted_s,last_admitted_s\\n"
                "csr-app-admission-diagnostics-v1,fixture," + application_profile + ","
                "0,2,1,fixed,100,100,"
                "0,0,0,0,0,598,599\\n",
                encoding="utf-8",
            )
            print(f"fake runner wrote {event_index} events")
        """
    ).replace("__MODE__", repr(mode))
    path.write_text(program, encoding="utf-8")
    os.chmod(path, 0o755)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def instrument_python_utility(source: Path, destination: Path) -> None:
    """Copy one child utility and record the interpreter flags it observes."""

    marker = '\nif __name__ == "__main__":\n'
    program = source.read_text(encoding="utf-8")
    if marker not in program:
        raise RuntimeError(f"cannot instrument {source}")
    probe = textwrap.dedent(
        """

        import sys as _probe_sys
        from pathlib import Path as _ProbePath
        _probe_sys.path.insert(0, str(_ProbePath(__file__).resolve().parent))
        import child_execution_probe
        child_execution_probe.record(_ProbePath(__file__).name)
        """
    )
    destination.write_text(program.replace(marker, probe + marker), encoding="utf-8")


def load_uncached_module(name: str, path: Path):
    """Execute a temporary workflow copy without generating import bytecode."""

    module = types.ModuleType(name)
    module.__file__ = str(path)
    module.__package__ = ""
    sys.modules[name] = module
    try:
        exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    finally:
        del sys.modules[name]
    return module


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
                    "application_profile": "current-send-only",
                    "mac_profile": "current-fine-free-slot",
                    "hop_security_profile": "production-pairwise16",
                    "hop_security_profile_origin": "explicit",
                    "hop_security_behavior": {
                        "ordinary_data": "pairwise16",
                        "ack_dack": "pairwise16",
                    },
                    "ack_envelope_profile": "unspecified",
                    "source_executable_sha256": "",
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
                    "app_admission_diagnostics_schema": (
                        WORKFLOW.APP_ADMISSION_DIAGNOSTICS_SCHEMA
                    ),
                    "application_profile": "current-send-only",
                    "mac_profile": "current-fine-free-slot",
                    "hop_security_profile": "production-pairwise16",
                    "hop_security_profile_origin": "explicit",
                    "hop_security_behavior": {
                        "ordinary_data": "pairwise16",
                        "ack_dack": "pairwise16",
                    },
                    "ack_envelope_profile": "unspecified",
                    "source_executable_sha256": "",
                    "aggregate_trace_only": True,
                    "duty_cycling": False,
                    "opnet_aligned_duty_cycle": True,
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
                    "source_exact_dack_retry_duplicate_proof_count": 0,
                    "source_exact_dack_retry_duplicate_delivery_count": 0,
                    "unmatched_delivery_count": 0,
                    "matched_size_compared_count": 100,
                    "matched_size_mismatch_count": 0,
                },
            )

    def test_nested_python_utilities_are_isolated_and_bytecode_free(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tools = root / "tools"
            tools.mkdir()
            workflow_path = tools / SCRIPT.name
            shutil.copy2(SCRIPT, workflow_path)
            utility_names = (
                "extract-opnet-ov.py",
                "aggregate-ns3-trace.py",
                "compare-opnet-ns3-aggregates.py",
            )
            for utility_name in utility_names:
                instrument_python_utility(
                    ROOT / "utils" / utility_name,
                    tools / utility_name,
                )
            (tools / "child_execution_probe.py").write_text(
                textwrap.dedent(
                    """\
                    import json
                    from pathlib import Path
                    import sys

                    def record(tool):
                        state = {
                            "tool": tool,
                            "isolated": sys.flags.isolated,
                            "dont_write_bytecode": sys.dont_write_bytecode,
                        }
                        with Path(__file__).with_name("child-state.jsonl").open(
                            "a", encoding="utf-8"
                        ) as stream:
                            stream.write(json.dumps(state, sort_keys=True) + "\\n")
                    """
                ),
                encoding="utf-8",
            )
            copied_workflow = load_uncached_module(
                "csr_opnet_aggregate_workflow_isolation_test",
                workflow_path,
            )
            scenario, ov, _pb, runner = self.prepare(root)
            output = root / "run"
            result = copied_workflow.main(
                self.command(scenario, ov, runner, output)[2:]
            )
            self.assertEqual(
                result,
                0,
                (output / "ns3-aggregate.log").read_text(encoding="utf-8"),
            )

            manifest = json.loads(
                (output / copied_workflow.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            stage_tools = {
                "extract_opnet": "extract-opnet-ov.py",
                "aggregate_ns3": "aggregate-ns3-trace.py",
                "compare": "compare-opnet-ns3-aggregates.py",
            }
            for stage, utility_name in stage_tools.items():
                command = manifest["stages"][stage]["command"]
                self.assertEqual(command[:3], [sys.executable, "-B", "-I"])
                self.assertEqual(Path(command[3]), tools / utility_name)

            states = [
                json.loads(line)
                for line in (tools / "child-state.jsonl")
                .read_text(encoding="utf-8")
                .splitlines()
            ]
            self.assertEqual([state["tool"] for state in states], list(utility_names))
            self.assertTrue(all(state["isolated"] == 1 for state in states))
            self.assertTrue(all(state["dont_write_bytecode"] for state in states))
            self.assertEqual(list(tools.rglob("*.pyc")), [])
            self.assertEqual(list(tools.rglob("__pycache__")), [])

    def test_historical_manifest_binds_binary_and_projects_data_and_ack(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            write_scenario(
                scenario,
                application_profile="legacy-send-only-no-dscp",
                mac_profile="hist-2014-next-tslot-modulo-probe",
                hop_security_profile="hist-adb97c54-bare",
            )
            output = root / "historical-run"
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
            canonical = manifest["configuration"]["identities"][
                "canonical_scenario"
            ]
            self.assertEqual(canonical["hop_security_profile"], "hist-adb97c54-bare")
            self.assertEqual(canonical["hop_security_profile_origin"], "explicit")
            self.assertEqual(
                canonical["source_executable_sha256"],
                WORKFLOW.HIST_ADB97C54_EXECUTABLE_SHA256,
            )
            self.assertEqual(
                canonical["hop_security_behavior"],
                {"ordinary_data": "bare", "ack_dack": "bare"},
            )
            configuration = manifest["configuration"]
            runner_configuration = configuration["runner"]
            self.assertEqual(
                runner_configuration["hop_security_behavior"],
                canonical["hop_security_behavior"],
            )
            runner_command = manifest["stages"]["run_ns3"]["command"]
            self.assertIn("--stop=60000", runner_command)
            self.assertIn("--aggregateTraceOnly=1", runner_command)
            self.assertIn("--quietModelLogs=1", runner_command)
            self.assertIn("--opnetAlignedDutyCycle=1", runner_command)
            self.assertTrue(
                any(value.startswith("--appDiagnostics=") for value in runner_command)
            )
            self.assertEqual(
                manifest["stages"]["run_ns3"]["application_admission_totals"],
                {
                    "admitted": 100,
                    "attempts": 100,
                    "blocked_destination": 0,
                    "blocked_discovery": 0,
                    "blocked_gateway_route": 0,
                    "blocked_nsdp": 0,
                    "blocked_topology": 0,
                },
            )
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
            "fifo": "unsupported delay matching",
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

    def test_provenance_accepts_only_accounted_dack_retry_duplicates(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            aggregates = root / "aggregates.csv"
            provenance_path = root / "provenance.json"
            trace.write_text("trace evidence\n", encoding="utf-8")
            aggregates.write_text("aggregate evidence\n", encoding="utf-8")

            provenance = {
                "schema": WORKFLOW.NS3_PROVENANCE_SCHEMA,
                "source": "ns3",
                "scenario": "fixture",
                "input": {
                    "path": str(trace.resolve()),
                    "sha256": WORKFLOW.digest(trace),
                    "size_bytes": trace.stat().st_size,
                },
                "output": {
                    "sha256": WORKFLOW.digest(aggregates),
                    "size_bytes": aggregates.stat().st_size,
                },
                "delay_matching": {
                    "matched_count": 2,
                    "unmatched_delivery_count": 0,
                    "matched_by_method": {
                        "exact_sequence": 1,
                        "source_exact_dack_retry_duplicate": 1,
                    },
                },
                "source_exact_dack_retry_duplicates": {
                    "repeated_feedback_proof_count": 1,
                    "matched_duplicate_delivery_count": 1,
                    "unused_proof_count": 0,
                    "lineages": [
                        {
                            "src": "0",
                            "dst": "2",
                            "sequence": "77",
                            "relay": "1",
                            "ingress_peer": "0",
                            "hop_sequence": "9",
                            "qualifying_feedback_count": 2,
                            "dack_feedback_count": 1,
                            "proven_extra_delivery_budget": 1,
                            "source_ordered_pairs": [
                                {
                                    "enqueue_event_index": 2,
                                    "feedback_event_index": 4,
                                },
                                {
                                    "enqueue_event_index": 5,
                                    "feedback_event_index": 7,
                                },
                            ],
                        }
                    ],
                },
                "window": {"in_window_event_counts": {"nwk_delivery": 2}},
                "matched_packet_size_integrity": {
                    "compared_count": 2,
                    "mismatch_count": 0,
                },
            }
            provenance_path.write_text(
                json.dumps(provenance), encoding="utf-8"
            )

            result = WORKFLOW._load_and_validate_ns3_provenance(
                provenance_path, trace, aggregates, "fixture"
            )

            self.assertEqual(
                result["matched_by_method"],
                {
                    "exact_sequence": 1,
                    "source_exact_dack_retry_duplicate": 1,
                },
            )
            self.assertEqual(
                result[
                    "source_exact_dack_retry_duplicate_delivery_count"
                ],
                1,
            )

            valid_provenance = copy.deepcopy(provenance)
            malformed_cases = {}

            overlapping = copy.deepcopy(valid_provenance)
            overlapping["source_exact_dack_retry_duplicates"]["lineages"][0][
                "source_ordered_pairs"
            ][1]["enqueue_event_index"] = 3
            malformed_cases["overlapping-pairs"] = overlapping

            impossible_budget = copy.deepcopy(valid_provenance)
            impossible_lineage = impossible_budget[
                "source_exact_dack_retry_duplicates"
            ]["lineages"][0]
            impossible_lineage["qualifying_feedback_count"] = 3
            impossible_lineage["dack_feedback_count"] = 2
            impossible_lineage["source_ordered_pairs"].append(
                {"enqueue_event_index": 8, "feedback_event_index": 9}
            )
            malformed_cases["impossible-budget"] = impossible_budget

            reused_index = copy.deepcopy(valid_provenance)
            second_lineage = copy.deepcopy(
                reused_index["source_exact_dack_retry_duplicates"]["lineages"][0]
            )
            second_lineage["sequence"] = "78"
            reused_index["source_exact_dack_retry_duplicates"]["lineages"].append(
                second_lineage
            )
            duplicate_evidence = reused_index[
                "source_exact_dack_retry_duplicates"
            ]
            duplicate_evidence["repeated_feedback_proof_count"] = 2
            duplicate_evidence["unused_proof_count"] = 1
            malformed_cases["cross-lineage-reused-index"] = reused_index

            for label, malformed in malformed_cases.items():
                with self.subTest(label=label):
                    provenance_path.write_text(
                        json.dumps(malformed), encoding="utf-8"
                    )
                    with self.assertRaisesRegex(
                        WORKFLOW.WorkflowError,
                        "lineage.*malformed",
                    ):
                        WORKFLOW._load_and_validate_ns3_provenance(
                            provenance_path, trace, aggregates, "fixture"
                        )

            provenance = valid_provenance

            provenance["source_exact_dack_retry_duplicates"]["lineages"] = []
            provenance_path.write_text(
                json.dumps(provenance), encoding="utf-8"
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "lineage budget is inconsistent"
            ):
                WORKFLOW._load_and_validate_ns3_provenance(
                    provenance_path, trace, aggregates, "fixture"
                )
            provenance["source_exact_dack_retry_duplicates"]["lineages"] = [
                {
                    "src": "0",
                    "dst": "2",
                    "sequence": "77",
                    "relay": "1",
                    "ingress_peer": "0",
                    "hop_sequence": "9",
                    "qualifying_feedback_count": 2,
                    "dack_feedback_count": 1,
                    "proven_extra_delivery_budget": 1,
                    "source_ordered_pairs": [
                        {"enqueue_event_index": 2, "feedback_event_index": 4},
                        {"enqueue_event_index": 5, "feedback_event_index": 7},
                    ],
                }
            ]

            provenance["source_exact_dack_retry_duplicates"][
                "repeated_feedback_proof_count"
            ] = 0
            provenance_path.write_text(
                json.dumps(provenance), encoding="utf-8"
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "duplicate evidence is inconsistent"
            ):
                WORKFLOW._load_and_validate_ns3_provenance(
                    provenance_path, trace, aggregates, "fixture"
                )

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

    def test_unknown_application_profile_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.csv"
            write_scenario(path, application_profile="invented-profile")
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "unsupported application_profile"
            ):
                WORKFLOW.load_scenario_run(path)

    def test_unknown_mac_profile_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.csv"
            write_scenario(path, mac_profile="invented-profile")
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "unsupported mac_profile"
            ):
                WORKFLOW.load_scenario_run(path)

    def test_supported_mac_profiles_are_accepted(self) -> None:
        profiles = (
            "current-fine-free-slot",
            "hist-2014-coarse-inclusive-no-avoid",
            "hist-2014-zero-based-rebuild-list",
            "hist-2015-fine-one-based-table-no-avoid",
            "hist-2014-next-tslot-modulo-probe",
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.csv"
            for profile in profiles:
                with self.subTest(profile=profile):
                    write_scenario(path, mac_profile=profile)
                    self.assertEqual(
                        WORKFLOW.load_scenario_run(path)["mac_profile"], profile
                    )

    def test_hop_security_profile_is_atomic_and_version_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.csv"
            write_scenario(path)
            loaded = WORKFLOW.load_scenario_run(path)
            self.assertEqual(
                loaded["hop_security_profile"],
                WORKFLOW.HOP_SECURITY_PROFILE_PRODUCTION_PAIRWISE16,
            )
            self.assertEqual(loaded["hop_security_profile_origin"], "explicit")
            self.assertEqual(
                loaded["hop_security_behavior"],
                {"ordinary_data": "pairwise16", "ack_dack": "pairwise16"},
            )
            self.assertEqual(loaded["ack_envelope_profile"], "unspecified")
            self.assertEqual(loaded["source_executable_sha256"], "")

            write_scenario(
                path,
                application_profile="legacy-send-only-no-dscp",
                mac_profile="hist-2014-next-tslot-modulo-probe",
                hop_security_profile=(
                    WORKFLOW.HOP_SECURITY_PROFILE_HIST_ADB97C54_BARE
                ),
            )
            loaded = WORKFLOW.load_scenario_run(path)
            self.assertEqual(
                loaded["hop_security_profile"],
                WORKFLOW.HOP_SECURITY_PROFILE_HIST_ADB97C54_BARE,
            )
            self.assertEqual(
                loaded["source_executable_sha256"],
                WORKFLOW.HIST_ADB97C54_EXECUTABLE_SHA256,
            )
            self.assertEqual(
                loaded["hop_security_behavior"],
                {"ordinary_data": "bare", "ack_dack": "bare"},
            )

            write_scenario(
                path,
                hop_security_profile=(
                    WORKFLOW.HOP_SECURITY_PROFILE_HIST_ADB97C54_BARE
                ),
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError,
                "adb97 executable tuple must select",
            ):
                WORKFLOW.load_scenario_run(path)

            write_scenario(
                path,
                application_profile="legacy-send-only-no-dscp",
                mac_profile="hist-2014-next-tslot-modulo-probe",
                hop_security_profile="production-pairwise16",
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "adb97 executable tuple must select"
            ):
                WORKFLOW.load_scenario_run(path)

            write_scenario(path, hop_security_profile="invented-profile")
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "unsupported hop_security_profile"
            ):
                WORKFLOW.load_scenario_run(path)

            write_scenario(
                path,
                hop_security_profile="production-pairwise16",
                ack_envelope_profile="hist-adb97c54-bare",
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "ack_envelope_profile disagree"
            ):
                WORKFLOW.load_scenario_run(path)

    def test_legacy_ack_alias_is_resolved_but_diagnostic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.csv"
            write_scenario(
                path,
                application_profile="legacy-send-only-no-dscp",
                mac_profile="hist-2014-next-tslot-modulo-probe",
                hop_security_profile=None,
                ack_envelope_profile="hist-adb97c54-bare",
                source_executable_sha256=(
                    WORKFLOW.HIST_ADB97C54_EXECUTABLE_SHA256
                ),
            )
            loaded = WORKFLOW.load_scenario_run(path)
            self.assertEqual(loaded["hop_security_profile"], "hist-adb97c54-bare")
            self.assertEqual(
                loaded["hop_security_profile_origin"], "legacy_ack_alias"
            )

            write_scenario(
                path,
                application_profile="legacy-send-only-no-dscp",
                mac_profile="hist-2014-next-tslot-modulo-probe",
                hop_security_profile=None,
                ack_envelope_profile="hist-adb97c54-bare",
                source_executable_sha256="",
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError,
                "effective hist-adb97c54-bare requires",
            ):
                WORKFLOW.load_scenario_run(path)

            write_scenario(
                path,
                hop_security_profile=None,
                ack_envelope_profile="production-pairwise16",
                source_executable_sha256=(
                    WORKFLOW.HIST_ADB97C54_EXECUTABLE_SHA256
                ),
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError,
                "effective production-pairwise16 cannot claim",
            ):
                WORKFLOW.load_scenario_run(path)

    def test_missing_hop_security_profile_is_compatible_but_diagnostic(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            write_scenario(
                scenario,
                hop_security_profile=None,
                ack_envelope_profile=None,
            )

            loaded = WORKFLOW.load_scenario_run(scenario)
            self.assertEqual(loaded["hop_security_profile"], "production-pairwise16")
            self.assertEqual(loaded["hop_security_profile_origin"], "missing")
            self.assertEqual(loaded["ack_envelope_profile"], "unspecified")

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
            self.assertEqual(manifest["profile"], "diagnostic")
            self.assertIn(
                "hop_security_profile_missing",
                manifest["noncertifying_overrides"],
            )
            self.assertEqual(
                manifest["configuration"]["identities"]["canonical_scenario"][
                    "hop_security_profile"
                ],
                "production-pairwise16",
            )
            self.assertEqual(
                manifest["configuration"]["runner"][
                    "hop_security_profile_origin"
                ],
                "missing",
            )

    def test_missing_mac_profile_is_backward_compatible_but_diagnostic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario, ov, _pb, runner = self.prepare(root)
            write_scenario(scenario, mac_profile=None)

            loaded = WORKFLOW.load_scenario_run(scenario)
            self.assertEqual(loaded["mac_profile"], "unspecified")

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
            self.assertEqual(manifest["profile"], "diagnostic")
            self.assertIn(
                "mac_profile_unspecified", manifest["noncertifying_overrides"]
            )
            self.assertEqual(
                manifest["configuration"]["identities"]["canonical_scenario"][
                    "mac_profile"
                ],
                "unspecified",
            )
            self.assertEqual(
                manifest["configuration"]["runner"]["mac_profile"],
                "unspecified",
            )

    def test_application_diagnostics_counters_must_partition_attempts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "app.csv"
            path.write_text(
                "schema,scenario,application_profile,flow_index,attempts,"
                "admitted,blocked_discovery,blocked_topology,"
                "blocked_gateway_route,blocked_destination,blocked_nsdp,"
                "first_admitted_s,last_admitted_s\n"
                "csr-app-admission-diagnostics-v1,fixture,current-send-only,"
                "0,2,1,0,0,0,0,0,1,1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                WORKFLOW.WorkflowError, "do not partition attempts"
            ):
                WORKFLOW.validate_app_admission_diagnostics(
                    path, "fixture", "current-send-only", 1
                )

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
