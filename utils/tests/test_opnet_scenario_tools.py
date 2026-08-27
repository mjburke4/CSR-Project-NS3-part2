#!/usr/bin/env python3
"""Focused tests for the OPNET scenario and differential-trace utilities."""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import importlib.util
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[2]


def load_script(module_name: str, filename: str):
    specification = importlib.util.spec_from_file_location(module_name, ROOT / "utils" / filename)
    module = importlib.util.module_from_spec(specification)
    sys.modules[module_name] = module
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


IMPORTER = load_script("csr_opnet_importer", "import-opnet-scenario.py")
COMPARATOR = load_script("csr_trace_comparator", "compare-opnet-ns3-traces.py")
INSTRUMENTER = load_script(
    "csr_opnet_instrumenter", "instrument-opnet-reservation-trace.py"
)


def string_value(value: str) -> bytes:
    encoded = value.encode("ascii")
    return b"J" + struct.pack(">I", len(encoded)) + encoded


def enum_value(value: str) -> bytes:
    encoded = value.encode("ascii")
    return b"J" + struct.pack(">II", 2, len(encoded)) + encoded


def double_value(value: float) -> bytes:
    return b"J" + struct.pack(">d", value)


def integer_value(value: int) -> bytes:
    return b"J" + struct.pack(">I", value)


def promoted(attribute_id: int, value: bytes) -> bytes:
    return b"\x00\x01" + bytes((attribute_id,)) + b"\x00" * 5 + value + b"\xff" * 4


def synthetic_network(
    include_node_type: bool = True, include_mobile_fields: bool = False
) -> bytes:
    names = [
        "name",
    ]
    if include_mobile_fields:
        names.append("color")
    names.extend(
        (
            "altitude",
            "model",
            "x position",
            "y position",
        )
    )
    if include_mobile_fields:
        names.extend(
            (
                "trajectory",
                "bearing",
                "trajectory speed override",
                "ground speed",
                "ascent rate",
                "pitch",
                "yaw",
                "roll",
            )
        )
    names.extend(
        (
            "Node ID",
            "Min Power",
            "Max Power",
            "Link Margin",
            "rx.ecc threshold",
            "rx.channel [0].min frequency",
            "tx.channel [0].min frequency",
        )
    )
    if include_node_type:
        names.append("Node Type")
    names.extend(
        ("app.Packet Interarrival Time", "app.Packet Size", "app.Start Time")
    )
    identifiers = {name: index + 1 for index, name in enumerate(names)}
    schema = b"".join(
        struct.pack(">I", len(name.encode("ascii"))) + name.encode("ascii")
        for name in names
    )
    data = IMPORTER.MODEL_HEADER + b" synthetic"
    data += b"\x00" * (512 - len(data)) + schema + b"\x00" * 32
    attributes = {
        "name": string_value("node_7"),
        "altitude": double_value(1.7),
        "model": string_value("br_node_v1"),
        "x position": double_value(1.2),
        "y position": double_value(-0.5),
        "Node ID": integer_value(7),
        "Min Power": double_value(-36.0),
        "Max Power": double_value(33.0),
        "Link Margin": double_value(12.0),
        "rx.ecc threshold": double_value(0.1),
        "rx.channel [0].min frequency": double_value(400.0),
        "tx.channel [0].min frequency": double_value(400.0),
        "app.Packet Interarrival Time": string_value("constant (0.02)"),
        "app.Packet Size": integer_value(600),
        "app.Start Time": double_value(300.0),
    }
    if include_node_type:
        attributes["Node Type"] = enum_value("Gateway")
    for name in names:
        value = attributes.get(name)
        if value is not None:
            data += promoted(identifiers[name], value)
    return data


class ScenarioImporterTests(unittest.TestCase):
    def test_promoted_schema_and_units(self) -> None:
        (node,) = IMPORTER.parse_network_model(synthetic_network(), 1000.0)
        self.assertEqual(node.node_id, 7)
        self.assertEqual(node.name, "node_7")
        self.assertEqual(node.node_type, "gateway")
        self.assertEqual(node.x_m, 1200.0)
        self.assertEqual(node.y_m, -500.0)
        self.assertEqual(node.height_m, 1.7)
        self.assertEqual(node.min_speed_kbps, 8)
        self.assertEqual(node.max_speed_kbps, 128)
        self.assertEqual(node.rx_frequency_hz, 400.0e6)
        self.assertEqual(node.interarrival_s, 0.02)
        self.assertEqual(node.packet_bytes, 600)

    def test_shifted_schema_defaults_to_ordinary(self) -> None:
        (node,) = IMPORTER.parse_network_model(
            synthetic_network(include_node_type=False), 1.0
        )
        self.assertEqual(node.node_type, "ordinary")
        self.assertEqual(node.packet_bytes, 600)
        self.assertEqual(node.start_s, 300.0)

    def test_unknown_mobile_fields_do_not_shift_promoted_ids(self) -> None:
        (node,) = IMPORTER.parse_network_model(
            synthetic_network(include_mobile_fields=True), 1.0
        )
        self.assertEqual(node.node_id, 7)
        self.assertEqual(node.node_type, "gateway")
        self.assertEqual(node.min_power_dbm, -36.0)
        self.assertEqual(node.rx_frequency_hz, 400.0e6)
        self.assertEqual(node.interarrival_s, 0.02)

    def test_des_environment(self) -> None:
        environment = (
            b'"duration":\t"60,000.0"\n'
            b'"seed":\t"128"\n'
            b'"tmm_simulate":\t"false"\n'
        )
        parsed = IMPORTER.parse_environment(environment)
        self.assertEqual(parsed["duration_s"], 60000.0)
        self.assertEqual(parsed["seed"], 128)
        self.assertFalse(parsed["tmm"])

    def test_seed_range_matches_runner_contract(self) -> None:
        maximum = IMPORTER.NS3_RNG_SEED_MAX
        for value in ("0", str(maximum + 1), "4294967295", "1.5"):
            with self.subTest(value=value), self.assertRaisesRegex(
                IMPORTER.ImportErrorDetail,
                f"DES seed must be an integer in 1..{maximum}",
            ):
                IMPORTER.parse_environment(
                    f'"seed":\t"{value}"\n'.encode("ascii")
                )
        self.assertEqual(
            IMPORTER.parse_environment(
                f'"seed":\t"{maximum}"\n'.encode("ascii")
            )["seed"],
            maximum,
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "scenario.nt.m"
            output = root / "scenario.csv"
            source.write_bytes(synthetic_network())
            arguments = IMPORTER.build_parser().parse_args(
                [str(source), str(output), "--seed", "0"]
            )
            with self.assertRaisesRegex(
                IMPORTER.ImportErrorDetail, f"--seed must be in 1..{maximum}"
            ):
                IMPORTER.import_scenario(arguments)
            self.assertFalse(output.exists())
        self.assertIn(f"1..{maximum}", IMPORTER.build_parser().format_help())

    def test_reservation_slot_override(self) -> None:
        self.assertEqual(
            IMPORTER.parse_reservation_slot_override("0x2:5"),
            (2, 5),
        )
        with self.assertRaises(argparse.ArgumentTypeError):
            IMPORTER.parse_reservation_slot_override("2:0")

    def test_gateway_flow_inference_matches_supplied_application_profile(self) -> None:
        (gateway,) = IMPORTER.parse_network_model(synthetic_network(), 1.0)
        ordinary = replace(
            gateway,
            node_id=8,
            name="node_8",
            node_type="ordinary",
        )
        (flow,) = IMPORTER.infer_gateway_flows([gateway, ordinary])
        self.assertEqual((flow.source, flow.destination), (8, 7))
        self.assertEqual(flow.interval_s, 0.02)
        self.assertEqual(flow.packet_bytes, 600)
        self.assertEqual(flow.dscp, 5)

    def test_gateway_flow_inference_uses_explicit_dscp_assumption(self) -> None:
        (gateway,) = IMPORTER.parse_network_model(synthetic_network(), 1.0)
        ordinary = replace(
            gateway,
            node_id=8,
            name="node_8",
            node_type="ordinary",
        )
        (flow,) = IMPORTER.infer_gateway_flows([gateway, ordinary], dscp=2)
        self.assertEqual(flow.dscp, 2)
        arguments = IMPORTER.build_parser().parse_args(
            ["scenario.nt.m", "scenario.csv", "--gateway-flow-dscp", "0x3"]
        )
        self.assertEqual(arguments.gateway_flow_dscp, 3)

    def test_gateway_flow_dscp_range(self) -> None:
        for value in ("-1", "8", "garbage"):
            with self.subTest(value=value), self.assertRaises(
                argparse.ArgumentTypeError
            ):
                IMPORTER.parse_gateway_flow_dscp(value)
        with self.assertRaisesRegex(
            IMPORTER.ImportErrorDetail, "gateway flow DSCP must be in 0..7"
        ):
            IMPORTER.infer_gateway_flows([], dscp=8)

    def test_gateway_flow_inference_requires_promoted_app_fields(self) -> None:
        (gateway,) = IMPORTER.parse_network_model(synthetic_network(), 1.0)
        ordinary = replace(
            gateway,
            node_id=8,
            name="node_8",
            node_type="ordinary",
        )
        for field in ("interarrival_s", "packet_bytes", "start_s"):
            with self.subTest(field=field), self.assertRaisesRegex(
                IMPORTER.ImportErrorDetail,
                r"node 'node_8' lacks promoted application attribute.*app\.",
            ):
                IMPORTER.infer_gateway_flows(
                    [gateway, replace(ordinary, **{field: None})]
                )

    def test_gateway_flow_inference_requires_one_gateway(self) -> None:
        (gateway,) = IMPORTER.parse_network_model(synthetic_network(), 1.0)
        with self.assertRaisesRegex(
            IMPORTER.ImportErrorDetail, "exactly one gateway"
        ):
            IMPORTER.infer_gateway_flows([])
        with self.assertRaisesRegex(
            IMPORTER.ImportErrorDetail, "exactly one gateway"
        ):
            IMPORTER.infer_gateway_flows([gateway, gateway])

    def test_output_aliases_preserve_source_and_environment_inputs(self) -> None:
        environment_data = (
            b'"duration":\t"60.0"\n'
            b'"seed":\t"128"\n'
            b'"tmm_simulate":\t"false"\n'
        )
        for target_label in ("source", "environment"):
            for alias_kind in ("direct", "symlink", "hardlink"):
                with self.subTest(target=target_label, alias=alias_kind):
                    with tempfile.TemporaryDirectory() as directory:
                        root = Path(directory)
                        source = root / "scenario.nt.m"
                        environment = root / "scenario-DES-1.ef"
                        source_data = synthetic_network()
                        source.write_bytes(source_data)
                        environment.write_bytes(environment_data)
                        target = source if target_label == "source" else environment
                        if alias_kind == "direct":
                            output = target
                        else:
                            output = root / "scenario.csv"
                            if alias_kind == "symlink":
                                output.symlink_to(target)
                            else:
                                os.link(target, output)
                        arguments = IMPORTER.build_parser().parse_args(
                            [
                                str(source),
                                str(output),
                                "--environment",
                                str(environment),
                            ]
                        )
                        with self.assertRaisesRegex(
                            IMPORTER.ImportErrorDetail,
                            rf"output path aliases .*{target_label}.*input",
                        ):
                            IMPORTER.import_scenario(arguments)
                        self.assertEqual(source.read_bytes(), source_data)
                        self.assertEqual(environment.read_bytes(), environment_data)

    def test_archive_environment_member_is_not_a_filesystem_input(self) -> None:
        environment_data = (
            b'"duration":\t"60.0"\n'
            b'"seed":\t"128"\n'
            b'"tmm_simulate":\t"false"\n'
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "project.zip"
            output = root / "scenario-DES-1.ef"
            with zipfile.ZipFile(archive, "w") as stream:
                stream.writestr("scenario.nt.m", synthetic_network())
                stream.writestr("scenario-DES-1.ef", environment_data)
            arguments = IMPORTER.build_parser().parse_args(
                [
                    str(archive),
                    str(output),
                    "--scenario",
                    "scenario.nt.m",
                    "--environment",
                    str(output),
                ]
            )
            nodes, flows = IMPORTER.import_scenario(arguments)
            self.assertEqual(len(nodes), 1)
            self.assertEqual(flows, [])
            self.assertTrue(output.read_text(encoding="utf-8").startswith("record,"))


class TraceComparatorTests(unittest.TestCase):
    def write_trace(self, path: Path, rows: list[dict[str, object]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=(
                    "Simulation Time",
                    "Action",
                    "Node ID",
                    "Tx Node",
                    "Pkt Type",
                    "Seq",
                    "SNR",
                    "Accepted",
                ),
            )
            writer.writeheader()
            writer.writerows(rows)

    def test_alias_normalization_and_time_tolerance(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            expected_path = Path(directory) / "opnet.csv"
            actual_path = Path(directory) / "ns3.csv"
            base = [
                {
                    "Simulation Time": 1.0,
                    "Action": "TX",
                    "Node ID": 1,
                    "Tx Node": 2,
                    "Pkt Type": "data",
                    "Seq": 4,
                    "SNR": 8.0,
                    "Accepted": 1,
                }
            ]
            actual = [dict(base[0], **{"Simulation Time": 1.0000005})]
            self.write_trace(expected_path, base)
            self.write_trace(actual_path, actual)
            expected = COMPARATOR.normalize_trace(expected_path)
            observed = COMPARATOR.normalize_trace(actual_path)
            report = COMPARATOR.compare(
                expected,
                observed,
                COMPARATOR.DEFAULT_KEY_FIELDS,
                dict(COMPARATOR.DEFAULT_TOLERANCES),
                set(),
                20,
            )
            self.assertTrue(report["pass"])
            self.assertEqual(report["counts"]["matched_events"], 1)

    def test_extra_event_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            expected_path = Path(directory) / "opnet.csv"
            actual_path = Path(directory) / "ns3.csv"
            row = {
                "Simulation Time": 1.0,
                "Action": "TX",
                "Node ID": 1,
                "Tx Node": 2,
                "Pkt Type": "data",
                "Seq": 4,
                "SNR": 8.0,
                "Accepted": 1,
            }
            self.write_trace(expected_path, [row])
            self.write_trace(actual_path, [row, dict(row, **{"Simulation Time": 2.0})])
            report = COMPARATOR.compare(
                COMPARATOR.normalize_trace(expected_path),
                COMPARATOR.normalize_trace(actual_path),
                COMPARATOR.DEFAULT_KEY_FIELDS,
                dict(COMPARATOR.DEFAULT_TOLERANCES),
                set(),
                1,
            )
            self.assertFalse(report["pass"])
            self.assertEqual(report["counts"]["extra_in_ns3"], 1)
            self.assertFalse(report["differences_truncated"])

    def test_event_and_time_filter(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.csv"
            rows = [
                {
                    "Simulation Time": 1.0,
                    "Action": "TX",
                    "Node ID": 1,
                    "Tx Node": 2,
                    "Pkt Type": "data",
                    "Seq": 4,
                    "SNR": 8.0,
                    "Accepted": 1,
                },
                {
                    "Simulation Time": 2.0,
                    "Action": "drop",
                    "Node ID": 1,
                    "Tx Node": 2,
                    "Pkt Type": "data",
                    "Seq": 4,
                    "SNR": 0.0,
                    "Accepted": 0,
                },
            ]
            self.write_trace(trace_path, rows)
            selected = COMPARATOR.filter_trace(
                COMPARATOR.normalize_trace(trace_path),
                {"rx_drop"},
                1.5,
                2.0,
                {("rx_drop", "1")},
            )
            self.assertEqual(len(selected), 1)
            self.assertEqual(selected[0].values["event"], "rx_drop")
            self.assertEqual(
                COMPARATOR.parse_selector("drop:0x1"),
                ("rx_drop", "1"),
            )

    def test_coincident_event_order_is_canonical(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            expected_path = Path(directory) / "opnet.csv"
            actual_path = Path(directory) / "ns3.csv"
            first = {
                "Simulation Time": 1.0,
                "Action": "TX",
                "Node ID": 1,
                "Tx Node": 3,
                "Pkt Type": "data",
                "Seq": 4,
                "SNR": 8.0,
                "Accepted": 1,
            }
            second = dict(first, **{"Node ID": 2, "Tx Node": 4})
            self.write_trace(expected_path, [first, second])
            self.write_trace(actual_path, [second, first])
            expected = COMPARATOR.order_coincident_events(
                COMPARATOR.normalize_trace(expected_path), 1.0e-6
            )
            actual = COMPARATOR.order_coincident_events(
                COMPARATOR.normalize_trace(actual_path), 1.0e-6
            )
            report = COMPARATOR.compare(
                expected,
                actual,
                COMPARATOR.DEFAULT_KEY_FIELDS,
                dict(COMPARATOR.DEFAULT_TOLERANCES),
                set(),
                20,
            )
            self.assertTrue(report["pass"])


class InstrumenterTests(unittest.TestCase):
    def test_numbered_upload_source_resolution(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "09-br_app.pr.c"
            source.write_text("fixture", encoding="utf-8")
            self.assertEqual(
                INSTRUMENTER.resolve_source(Path(directory), "br_app.pr.c"),
                source,
            )


if __name__ == "__main__":
    unittest.main()
