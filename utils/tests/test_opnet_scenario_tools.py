#!/usr/bin/env python3
"""Focused tests for the OPNET scenario and differential-trace utilities."""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
import unittest


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


def synthetic_network(include_node_type: bool = True) -> bytes:
    names = [
        "name",
        "altitude",
        "model",
        "x position",
        "y position",
        "Node ID",
        "Min Power",
        "Max Power",
        "Link Margin",
        "rx.ecc threshold",
        "rx.channel [0].min frequency",
        "tx.channel [0].min frequency",
    ]
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


if __name__ == "__main__":
    unittest.main()
