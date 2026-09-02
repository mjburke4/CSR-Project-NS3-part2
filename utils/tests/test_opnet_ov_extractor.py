#!/usr/bin/env python3
"""Focused tests for the conservative historical OPNET .ov extractor."""

from __future__ import annotations

import csv
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import tracemalloc
import unittest


ROOT = Path(__file__).resolve().parents[2]


def load_script():
    specification = importlib.util.spec_from_file_location(
        "csr_opnet_ov_extractor", ROOT / "utils" / "extract-opnet-ov.py"
    )
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


EXTRACTOR = load_script()


def u32(value: int) -> bytes:
    return struct.pack(">I", value)


def f64(value: float) -> bytes:
    return struct.pack(">d", value)


def string(value: str) -> bytes:
    encoded = value.encode("ascii")
    return u32(len(encoded)) + encoded


def model_header(producer: str = "op_runsim_dev") -> bytes:
    value = (
        f"MIL_3_Tfile_Hdr_ 171A 150A {producer} 1D synthetic"
    ).encode("ascii")
    return value + b" " * (511 - len(value)) + b"\0"


def vector_block(values: list[float], *, start: float, width: float) -> bytes:
    cookie = bytes.fromhex("d4b249ad2594c37d")
    encoded_count = len(values) + 1
    return (
        b"\x82"
        + u32(0x100)
        + cookie
        + b"\0" * 8
        + u32(encoded_count)
        + b"".join(f64(value) for value in (start, width, *values))
        + bytes.fromhex("54d249ad2594c37d")
    )


def partial_vector_block() -> bytes:
    cookie = bytes.fromhex("d4b249ad2594c37d")
    return (
        b"\x82"
        + u32(0x100)
        + cookie
        + b"\0" * 8
        + u32(0)
        + bytes.fromhex("54b249ad2594c37d")
        + f64(3.0)
    )


def synthetic_ov(
    values: list[float] | None,
    include_duration: bool = True,
    *,
    duration_s: float = 60000.0,
    duplicate_scalar: bool = False,
    description_ids: tuple[int, ...] = (9,),
    vector_specs: tuple[tuple[str, int, list[float] | None], ...] | None = None,
    declared_metadata_unit_count: int | None = None,
    details_flags: int | None = None,
    filter_value: str = "none",
    aggregation_description: str = "Bucket: sample mean: total of 100 values",
    vector_start_s: float = 0.0,
    vector_width_s: float = 600.0,
    scope_value: str = "Sink",
) -> bytes:
    body = bytearray()
    body += u32(0) + string("Aug 09 2011") + string("14:13:02.000")
    scalars = [("Execution.seed", 128.0)]
    if include_duration:
        scalars.append(("Execution.simulated duration (sec)", duration_s))
    if duplicate_scalar:
        scalars.append(("Execution.seed", 256.0))
    body += u32(len(scalars))
    for name, value in scalars:
        body += string(name) + u32(1) + f64(value) + u32(0xFFFFFFFF)
    description = "End-to-end delay of packets received by traffic sinks."
    body += u32(0) + u32(len(description_ids))
    for description_id in description_ids:
        body += u32(description_id) + u32(1) + string(description)

    if vector_specs is None:
        vector_specs = (("End-to-End Delay (seconds)", 9, values),)
    if declared_metadata_unit_count is None:
        declared_metadata_unit_count = 1 + 2 * len(vector_specs)
    body += u32(declared_metadata_unit_count)

    body += u32(0x23) + string(scope_value)
    body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
    group_link_position = len(body)
    body += u32(0)

    data_offset_positions: list[int] = []
    for name_value, description_id, vector_values in vector_specs:
        body += u32(0x124) + string(name_value)
        body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
        statistic_link_position = len(body)
        body += u32(0)
        record_details_flags = (
            details_flags
            if details_flags is not None
            else (0x165 if vector_values is None else 0x145)
        )
        body += u32(record_details_flags) + string(filter_value) + u32(0xFFFFFFFF)
        body += string(aggregation_description)
        body += u32(description_id) + u32(0)
        data_offset_positions.append(len(body))
        body += u32(0) + u32(0)
        struct.pack_into(
            ">I", body, statistic_link_position, 512 + len(body)
        )

    body += u32(0)
    struct.pack_into(">I", body, group_link_position, 512 + len(body))
    body += u32(0)

    for data_offset_position, (_, _, vector_values) in zip(
        data_offset_positions, vector_specs
    ):
        struct.pack_into(">I", body, data_offset_position, 512 + len(body))
        if vector_values is None:
            body += partial_vector_block()
        else:
            body += vector_block(
                vector_values,
                start=vector_start_s,
                width=vector_width_s,
            )
    return model_header() + body


def synthetic_nested_ov(
    *,
    nested_statistic_flags: int = 0x224,
    nested_details_flags: int = 0x245,
    nested_aggregation: str = "All values",
    nested_marker: int = 0x81,
    global_filter: str = "none",
    nested_filter: str = "none",
    nested_event_count: int = 1,
    nested_event_values: tuple[float, ...] | None = None,
    nested_event_times: tuple[float, ...] | None = None,
    duplicate_nested_statistic: bool = False,
    declared_metadata_unit_count: int | None = None,
    nested_scope: str = "MAC",
    nested_statistic_name: str = "Tx Queuing Delay (sec)",
) -> bytes:
    """Build the minimal exact network -> node nested latency layout."""
    if nested_event_values is None:
        nested_event_values = (0.25,) * nested_event_count
    if nested_event_times is None:
        nested_event_times = (12.0,) * nested_event_count
    if (
        len(nested_event_values) != nested_event_count
        or len(nested_event_times) != nested_event_count
    ):
        raise ValueError("synthetic nested event arrays must match event count")
    body = bytearray()
    body += u32(0) + string("Mar 18 2013") + string("08:40:20.000")
    scalars = [
        ("Execution.seed", 128.0),
        ("Execution.simulated duration (sec)", 1200.0),
    ]
    body += u32(len(scalars))
    for name, value in scalars:
        body += string(name) + u32(1) + f64(value) + u32(0xFFFFFFFF)
    descriptions = (
        (9, "End-to-end delay of packets received by traffic sinks."),
        (14, "Queuing delay that packets incur at the MAC Tx queue."),
    )
    body += u32(0) + u32(len(descriptions))
    for description_id, description in descriptions:
        body += u32(description_id) + u32(1) + string(description)
    nested_statistic_count = 2 if duplicate_nested_statistic else 1
    if declared_metadata_unit_count is None:
        declared_metadata_unit_count = 8 + 2 * (nested_statistic_count - 1)
    body += u32(declared_metadata_unit_count)

    # One canonical bucket aggregate.
    body += u32(0x23) + string("Sink")
    body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
    global_group_link = len(body)
    body += u32(0)
    body += u32(0x124) + string("End-to-End Delay (seconds)")
    body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
    global_stat_link = len(body)
    body += u32(0)
    body += u32(0x145) + string(global_filter) + u32(0xFFFFFFFF)
    body += string("Bucket: sample mean: total of 100 values")
    body += u32(9) + u32(0)
    global_data_offset = len(body)
    body += u32(0) + u32(0)
    struct.pack_into(">I", body, global_stat_link, 512 + len(body))
    body += u32(0)
    struct.pack_into(">I", body, global_group_link, 512 + len(body))

    # Exact two-level 0x22 hierarchy with one per-node All-values record.
    body += u32(0x22) + string("Campus Network")
    body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
    network_link = len(body)
    body += u32(0)
    body += u32(0x22) + string("node_1")
    body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
    node_link = len(body)
    body += u32(0)
    body += u32(0x23) + string(nested_scope)
    body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
    nested_group_link = len(body)
    body += u32(0)
    nested_data_offsets: list[int] = []
    for _ in range(nested_statistic_count):
        body += u32(nested_statistic_flags) + string(nested_statistic_name)
        body += u32(0xFFFFFFFF) * 2 + u32(0) * 2
        nested_stat_link = len(body)
        body += u32(0)
        body += u32(nested_details_flags) + string(nested_filter)
        body += u32(0xFFFFFFFF)
        body += string(nested_aggregation)
        body += u32(14) + u32(0)
        nested_data_offsets.append(len(body))
        body += u32(0) + u32(0)
        struct.pack_into(">I", body, nested_stat_link, 512 + len(body))
    body += u32(0)
    struct.pack_into(">I", body, nested_group_link, 512 + len(body))
    body += u32(0)
    struct.pack_into(">I", body, node_link, 512 + len(body))
    body += u32(0)
    struct.pack_into(">I", body, network_link, 512 + len(body))
    body += u32(0)

    struct.pack_into(">I", body, global_data_offset, 512 + len(body))
    body += vector_block([0.25] * 100, start=0.0, width=12.0)
    for nested_data_offset in nested_data_offsets:
        struct.pack_into(">I", body, nested_data_offset, 512 + len(body))
        encoded_count = nested_event_count + 2
        nested_block = (
            b"\x81"
            + u32(0x100)
            + EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE
            + EXTRACTOR.RECOVERED_VECTOR_RESERVED
            + u32(encoded_count)
            + f64(EXTRACTOR.MISSING_SENTINEL)
            + b"".join(f64(value) for value in nested_event_values)
            + f64(EXTRACTOR.ALL_VALUES_END_SENTINEL)
            + f64(0.0)
            + b"".join(f64(time_s) for time_s in nested_event_times)
            + f64(1200.0)
        )
        if nested_marker != 0x81:
            nested_block = bytes((nested_marker,)) + nested_block[1:]
        body += nested_block
    return model_header() + body


def tagged_string(value: str, tag: int = 0x0A) -> bytes:
    encoded = value.encode("ascii")
    return u32(tag) + u32(len(encoded)) + encoded


def synthetic_pb(*, duplicate_probe_id: bool = False) -> bytes:
    values = [
        "pb2",
        "",
        "linear",
        "",
        "Sink",
        "",
        "End-to-End Delay (seconds)",
        "",
        "sample mean",
        "bucket/default total/sample mean",
    ]
    if duplicate_probe_id:
        values += values
    body = string("fixture") + u32(2 if duplicate_probe_id else 1)
    body += b"".join(tagged_string(value) for value in values)
    return model_header("modeler") + body


def synthetic_nested_pb() -> bytes:
    aggregate = [
        "pb0",
        "linear",
        "Sink",
        "End-to-End Delay (seconds)",
        "sample mean",
        "bucket/default total/sample mean",
    ]
    all_values = [
        "pb1",
        "linear",
        "MAC",
        "Tx Queuing Delay (sec)",
    ]
    body = string("fixture") + u32(2)
    body += b"".join(tagged_string(value) for value in (*aggregate, *all_values))
    return model_header("modeler") + body


def linked_record_offsets(data: bytes, flags: int, name: str) -> tuple[int, int]:
    needle = u32(flags) + string(name)
    record_offset = data.index(needle, 512)
    link_offset = record_offset + len(needle) + 16
    return record_offset, link_offset


def statistic_data_offset_field(data: bytes, flags: int, name: str) -> int:
    record_offset, _ = linked_record_offsets(data, flags, name)
    offset = record_offset + 4
    name_length = struct.unpack_from(">I", data, offset)[0]
    offset += 4 + name_length + 20
    offset += 4
    filter_length = struct.unpack_from(">I", data, offset)[0]
    offset += 4 + filter_length + 4
    aggregation_length = struct.unpack_from(">I", data, offset)[0]
    offset += 4 + aggregation_length
    offset += 8
    return offset


class OvExtractorTests(unittest.TestCase):
    def assert_ov_rejected(self, data: bytes, pattern: str) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid-DES-1.ov"
            path.write_bytes(data)
            with self.assertRaisesRegex(EXTRACTOR.OvFormatError, pattern):
                EXTRACTOR.extract_ov(path)

    def run_main(self, arguments: list[str]) -> tuple[int, str]:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            return_code = EXTRACTOR.main(arguments)
        return return_code, stderr.getvalue()

    def test_complete_vector_json_csv_and_probe_crosscheck(self) -> None:
        values = [float(index) / 10.0 for index in range(100)]
        values[4] = EXTRACTOR.MISSING_SENTINEL
        ov_data = synthetic_ov(values)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            pb_path = root / "fixture.pb.m"
            ov_path.write_bytes(ov_data)
            pb_path.write_bytes(synthetic_pb())
            manifest = EXTRACTOR.extract_ov(
                ov_path, probe_definition=pb_path
            )

        self.assertEqual(manifest["schema"], EXTRACTOR.MANIFEST_SCHEMA)
        self.assertEqual(manifest["status"], "complete")
        self.assertEqual(manifest["scenario"], "fixture")
        self.assertEqual(manifest["execution"]["duration_s"], 60000.0)
        self.assertEqual(
            manifest["provenance"]["source_file_sha256"],
            hashlib.sha256(ov_data).hexdigest(),
        )
        vector = manifest["vectors"][0]
        self.assertEqual(vector["statistic"], "Sink.End-to-End Delay (seconds)")
        self.assertEqual(vector["unit"], "s")
        self.assertEqual(vector["aggregation"], "bucket_sample_mean")
        self.assertEqual(vector["decoded_value_count"], 100)
        self.assertEqual(vector["time_axis"]["bucket_width_s"], 600.0)
        self.assertEqual(vector["buckets"][0]["time_s"], 600.0)
        self.assertEqual(vector["buckets"][-1]["time_s"], 60000.0)
        self.assertIsNone(vector["buckets"][4]["value"])
        self.assertEqual(vector["buckets"][4]["raw_value"], 2.0e100)
        self.assertEqual(vector["buckets"][4]["value_status"], "missing")
        self.assertEqual(
            vector["data_record"]["record_cookie"],
            EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE.hex(),
        )
        self.assertEqual(
            vector["data_record"]["reserved"],
            EXTRACTOR.RECOVERED_VECTOR_RESERVED.hex(),
        )
        self.assertEqual(
            vector["data_record"]["trailer"],
            EXTRACTOR.RECOVERED_COMPLETE_VECTOR_TRAILER.hex(),
        )
        self.assertEqual(
            vector["probe_definition_match"]["status"], "exact"
        )

        stream = io.StringIO()
        EXTRACTOR.write_csv(manifest, stream)
        rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(rows), 100)
        self.assertEqual(tuple(rows[0]), EXTRACTOR.CSV_COLUMNS)
        self.assertEqual(rows[0]["time_s"], "600")
        self.assertEqual(rows[4]["value"], "")
        self.assertEqual(rows[4]["raw_value"], "2e+100")
        self.assertEqual(rows[4]["value_status"], "missing")

    def test_aggregate_rejects_all_values_end_sentinel(self) -> None:
        values = [0.0] * 100
        values[4] = EXTRACTOR.ALL_VALUES_END_SENTINEL
        self.assert_ov_rejected(
            synthetic_ov(values),
            "aggregate bucket 5 reuses structural end sentinel",
        )

    def test_nested_latency_layout_exports_only_bucket_aggregates(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            pb_path = root / "fixture.pb.m"
            ov_path.write_bytes(synthetic_nested_ov())
            pb_path.write_bytes(synthetic_nested_pb())
            manifest = EXTRACTOR.extract_ov(
                ov_path,
                probe_definition=pb_path,
            )

        self.assertEqual(manifest["status"], "complete")
        self.assertEqual(
            manifest["format"]["declared_metadata_unit_count"], 8
        )
        self.assertEqual(
            manifest["format"]["decoded_metadata_unit_count"], 8
        )
        self.assertEqual(
            manifest["format"]["metadata_counts"],
            {
                "hierarchy_containers": 2,
                "statistic_groups": 2,
                "statistics": 2,
            },
        )
        self.assertEqual(len(manifest["vectors"]), 1)
        self.assertEqual(
            manifest["vectors"][0]["probe_definition_match"]["status"],
            "exact",
        )
        excluded = manifest["excluded_nonaggregate_vectors"]
        self.assertEqual(len(excluded), 1)
        self.assertEqual(excluded[0]["hierarchy"], ["Campus Network", "node_1"])
        self.assertEqual(excluded[0]["statistic"], "MAC.Tx Queuing Delay (sec)")
        self.assertEqual(
            excluded[0]["status"], "validated_excluded_nonaggregate"
        )
        self.assertEqual(excluded[0]["data_record"]["marker"], "0x81")
        self.assertEqual(excluded[0]["data_record"]["encoded_count"], 3)
        self.assertEqual(excluded[0]["data_record"]["event_value_count"], 1)
        self.assertEqual(
            excluded[0]["data_record"]["event_semantics"],
            "per_statistic_write_record_not_packet_trace",
        )
        self.assertFalse(excluded[0]["data_record"]["numeric_contents_emitted"])
        self.assertEqual(excluded[0]["unit"], "s")
        self.assertNotIn("events", excluded[0])
        self.assertEqual(
            manifest["vector_completeness"][
                "validated_nonaggregate_event_count"
            ],
            1,
        )
        self.assertEqual(
            manifest["vector_completeness"][
                "emitted_nonaggregate_event_count"
            ],
            0,
        )
        self.assertEqual(
            excluded[0]["probe_definition_match"]["status"], "exact"
        )
        rows = EXTRACTOR.aggregate_rows(manifest)
        self.assertEqual(len(rows), 100)
        self.assertTrue(
            all(row["statistic"] == "Sink.End-to-End Delay (seconds)" for row in rows)
        )

    def test_nested_all_values_opt_in_emits_validated_pairs_in_order(
        self,
    ) -> None:
        data = synthetic_nested_ov(
            nested_event_count=3,
            nested_event_values=(0.25, 0.5, 0.75),
            nested_event_times=(12.0, 12.0, 13.0),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixture-DES-1.ov"
            path.write_bytes(data)
            manifest = EXTRACTOR.extract_ov(
                path,
                include_all_values=True,
            )

        excluded = manifest["excluded_nonaggregate_vectors"][0]
        self.assertTrue(excluded["data_record"]["numeric_contents_emitted"])
        self.assertEqual(
            excluded["data_record"]["event_semantics"],
            "per_statistic_write_record_not_packet_trace",
        )
        self.assertEqual(
            excluded["events"],
            [
                {"record_index": 0, "time_s": 12.0, "value": 0.25},
                {"record_index": 1, "time_s": 12.0, "value": 0.5},
                {"record_index": 2, "time_s": 13.0, "value": 0.75},
            ],
        )
        self.assertEqual(
            manifest["vector_completeness"][
                "emitted_nonaggregate_event_count"
            ],
            3,
        )
        self.assertEqual(len(EXTRACTOR.aggregate_rows(manifest)), 100)

    def test_nested_all_values_opt_in_preserves_empty_record(self) -> None:
        data = synthetic_nested_ov(nested_event_count=0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixture-DES-1.ov"
            path.write_bytes(data)
            manifest = EXTRACTOR.extract_ov(
                path,
                include_all_values=True,
            )

        excluded = manifest["excluded_nonaggregate_vectors"][0]
        self.assertTrue(excluded["data_record"]["numeric_contents_emitted"])
        self.assertEqual(excluded["events"], [])
        self.assertEqual(
            manifest["vector_completeness"][
                "emitted_nonaggregate_event_count"
            ],
            0,
        )

    def test_all_values_emission_total_is_bounded_before_allocation(
        self,
    ) -> None:
        vectors = [
            EXTRACTOR.ExcludedNonAggregateVector(
                hierarchy=("Campus Network", f"node_{index}"),
                scope="MAC",
                name="Tx Queuing Delay (sec)",
                description_id=14,
                description="fixture",
                data_offset=0,
                metadata_details_flags=0x245,
                data_record={
                    "encoded_count": 11_019,
                    "event_value_count": 11_017,
                },
            )
            for index in range(3)
        ]
        tracemalloc.start()
        try:
            with self.assertRaisesRegex(
                EXTRACTOR.OvFormatError,
                "event count 33051 exceeds the source-observed total .*31758",
            ):
                EXTRACTOR._emit_excluded_nonaggregate_events(
                    b"",
                    Path("fixture.ov"),
                    vectors,
                )
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        self.assertLess(peak, 1_000_000)
        self.assertTrue(all(vector.events == [] for vector in vectors))

    def test_cli_include_all_values_changes_json_only(self) -> None:
        data = synthetic_nested_ov()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            json_path = root / "fixture.json"
            csv_path = root / "fixture.csv"
            default_csv_path = root / "fixture-default.csv"
            ov_path.write_bytes(data)
            default_return_code = EXTRACTOR.main(
                [
                    str(ov_path),
                    "--csv",
                    str(default_csv_path),
                ]
            )
            return_code = EXTRACTOR.main(
                [
                    str(ov_path),
                    "--include-all-values",
                    "--json",
                    str(json_path),
                    "--csv",
                    str(csv_path),
                ]
            )
            manifest = json.loads(json_path.read_text(encoding="utf-8"))
            rows = list(
                csv.DictReader(
                    io.StringIO(csv_path.read_text(encoding="utf-8"))
                )
            )
            csv_unchanged = (
                csv_path.read_bytes() == default_csv_path.read_bytes()
            )

        self.assertEqual(default_return_code, 0)
        self.assertEqual(return_code, 0)
        self.assertEqual(
            manifest["excluded_nonaggregate_vectors"][0]["events"],
            [{"record_index": 0, "time_s": 12.0, "value": 0.25}],
        )
        self.assertTrue(csv_unchanged)
        self.assertEqual(len(rows), 100)
        self.assertTrue(
            all(row["statistic"] == "Sink.End-to-End Delay (seconds)" for row in rows)
        )

    def test_cli_include_all_values_rejects_csv_only_output(self) -> None:
        data = synthetic_nested_ov()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            csv_path = root / "fixture.csv"
            ov_path.write_bytes(data)
            return_code, stderr = self.run_main(
                [
                    str(ov_path),
                    "--include-all-values",
                    "--csv",
                    str(csv_path),
                ]
            )
            csv_exists = csv_path.exists()

        self.assertEqual(return_code, 2)
        self.assertIn("requires --json", stderr)
        self.assertFalse(csv_exists)

    def test_nested_latency_layout_rejects_unknown_statistic_flags(self) -> None:
        self.assert_ov_rejected(
            synthetic_nested_ov(nested_statistic_flags=0x225),
            "unsupported statistic flags 0x225",
        )

    def test_nested_latency_layout_rejects_unknown_details_flags(self) -> None:
        self.assert_ov_rejected(
            synthetic_nested_ov(nested_details_flags=0x225),
            "unsupported statistic details 0x225",
        )

    def test_nested_latency_layout_rejects_unproven_aggregation(self) -> None:
        self.assert_ov_rejected(
            synthetic_nested_ov(
                nested_aggregation="Bucket: sample mean: total of 100 values"
            ),
            "nested statistic MAC.Tx Queuing Delay.*uses unsupported aggregation",
        )

    def test_unknown_global_aggregation_is_rejected(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(
                [0.0] * 100,
                aggregation_description="X" * 40,
            ),
            "unsupported aggregate description",
        )

    def test_global_aggregation_requires_declared_value_count(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(
                [0.0] * 100,
                aggregation_description="Bucket: sample mean",
            ),
            "unsupported aggregate description",
        )

    def test_global_aggregation_requires_exact_recovered_grammar(self) -> None:
        descriptions = (
            "garbage sample mean total of 100 values",
            "evil/sum/time x total of 100 values",
            "xxx sum: total of 100 values",
            "Bucket: sum: total of 100 values trailing",
        )
        for description in descriptions:
            with self.subTest(description=description):
                self.assert_ov_rejected(
                    synthetic_ov(
                        [0.0] * 100,
                        aggregation_description=description,
                    ),
                    "unsupported aggregate description",
                )

    def test_huge_declared_value_count_is_format_error(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(
                [0.0] * 100,
                aggregation_description=(
                    "Bucket: sample mean: total of " + "9" * 5000 + " values"
                ),
            ),
            "implausible aggregate expected-value count with 5000 decimal digits",
        )

    def test_non_source_aggregate_bucket_count_is_rejected(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(
                [0.0] * 100,
                aggregation_description=(
                    "Bucket: sample mean: total of 101 values"
                ),
            ),
            "unsupported aggregate expected-value count 101",
        )

    def test_global_and_nested_statistic_filters_must_be_none(self) -> None:
        cases = (
            synthetic_ov([0.0] * 100, filter_value="xxxx"),
            synthetic_nested_ov(global_filter="xxxx"),
            synthetic_nested_ov(nested_filter="xxxx"),
        )
        for index, data in enumerate(cases):
            with self.subTest(index=index):
                self.assert_ov_rejected(
                    data,
                    "unsupported statistic filter 'xxxx'",
                )

    def test_nested_latency_layout_rejects_wrong_record_marker(self) -> None:
        self.assert_ov_rejected(
            synthetic_nested_ov(nested_marker=0x82),
            "unsupported nested vector marker 0x82",
        )

    def test_nested_latency_layout_rejects_extent_mismatch(self) -> None:
        data = bytearray(synthetic_nested_ov())
        cookie_offset = data.rindex(EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE)
        encoded_count_offset = cookie_offset + 16
        struct.pack_into(">I", data, encoded_count_offset, 4)
        self.assert_ov_rejected(
            bytes(data),
            "nested vector count 4 exceeds the block-derived maximum 3",
        )

    def test_nested_all_values_count_is_source_bounded_before_iteration(
        self,
    ) -> None:
        data = synthetic_nested_ov(nested_event_count=20_000)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "large-DES-1.ov"
            path.write_bytes(data)
            tracemalloc.start()
            try:
                with self.assertRaisesRegex(
                    EXTRACTOR.OvFormatError,
                    "exceeds the largest source-observed latency record",
                ):
                    EXTRACTOR.extract_ov(path)
                _, peak = tracemalloc.get_traced_memory()
            finally:
                tracemalloc.stop()
        self.assertLess(peak, 2_000_000)

    def test_nested_latency_layout_rejects_unknown_child_flags(self) -> None:
        data = bytearray(synthetic_nested_ov())
        node_record = u32(0x22) + string("node_1")
        node_offset = data.index(node_record, 512)
        struct.pack_into(">I", data, node_offset, 0x24)
        self.assert_ov_rejected(
            bytes(data),
            "unsupported nested hierarchy child flags 0x24",
        )

    def test_nested_latency_layout_rejects_declared_count_plus_or_minus_one(
        self,
    ) -> None:
        cases = (
            (7, "decoded metadata units 8 exceed limit 7"),
            (9, "does not equal decoded count 8"),
        )
        for declared_count, pattern in cases:
            with self.subTest(declared_count=declared_count):
                self.assert_ov_rejected(
                    synthetic_nested_ov(
                        declared_metadata_unit_count=declared_count
                    ),
                    pattern,
                )

    def test_metadata_count_is_incrementally_source_bounded(self) -> None:
        counts = {
            "hierarchy_containers": 0,
            "statistic_groups": 1,
            "statistics": 56,
        }
        with self.assertRaisesRegex(
            EXTRACTOR.OvFormatError,
            "decoded metadata units 115 exceed limit 113",
        ):
            EXTRACTOR._increment_metadata_count(
                path=Path("fixture.ov"),
                offset=512,
                metadata_counts=counts,
                field="statistics",
                declared_limit=113,
            )

    def test_nested_latency_layout_rejects_duplicate_identity(self) -> None:
        self.assert_ov_rejected(
            synthetic_nested_ov(duplicate_nested_statistic=True),
            "duplicate nested vector identity",
        )

    def test_nested_latency_layout_rejects_backward_and_aliased_links(
        self,
    ) -> None:
        original = synthetic_nested_ov()
        network_offset, network_link_offset = linked_record_offsets(
            original, 0x22, "Campus Network"
        )
        _, node_link_offset = linked_record_offsets(original, 0x22, "node_1")
        node_link = struct.unpack_from(">I", original, node_link_offset)[0]
        mutations = (
            (network_offset, "invalid nested hierarchy link"),
            (node_link, "nested hierarchy link"),
        )
        for link, pattern in mutations:
            with self.subTest(link=link):
                data = bytearray(original)
                struct.pack_into(">I", data, network_link_offset, link)
                self.assert_ov_rejected(bytes(data), pattern)

    def test_nested_latency_layout_rejects_missing_statistic_terminator(
        self,
    ) -> None:
        data = bytearray(synthetic_nested_ov())
        _, nested_group_link_offset = linked_record_offsets(
            data, 0x23, "MAC"
        )
        nested_group_end = struct.unpack_from(">I", data, nested_group_link_offset)[0]
        struct.pack_into(">I", data, nested_group_end - 4, 0x224)
        self.assert_ov_rejected(
            bytes(data), "decoded metadata units 10 exceed limit 8"
        )

    def test_nested_latency_layout_rejects_second_root_and_group_after_root(
        self,
    ) -> None:
        original = synthetic_nested_ov()
        _, network_link_offset = linked_record_offsets(
            original, 0x22, "Campus Network"
        )
        top_terminator = struct.unpack_from(">I", original, network_link_offset)[0]
        for flags, pattern in (
            (0x22, "multiple top-level nested hierarchy containers"),
            (0x23, "top-level statistic group follows nested hierarchy"),
        ):
            with self.subTest(flags=flags):
                data = bytearray(original)
                struct.pack_into(">I", data, top_terminator, flags)
                self.assert_ov_rejected(bytes(data), pattern)

    def test_nested_latency_layout_rejects_aliased_data_offsets(self) -> None:
        data = bytearray(synthetic_nested_ov())
        global_offset_field = statistic_data_offset_field(
            data, 0x124, "End-to-End Delay (seconds)"
        )
        nested_offset_field = statistic_data_offset_field(
            data, 0x224, "Tx Queuing Delay (sec)"
        )
        global_offset = struct.unpack_from(">I", data, global_offset_field)[0]
        struct.pack_into(">I", data, nested_offset_field, global_offset)
        self.assert_ov_rejected(
            bytes(data),
            "vector data offsets are not unique and ordered",
        )

    def test_nested_latency_layout_rejects_record_header_corruption(self) -> None:
        original = synthetic_nested_ov()
        cookie_offset = original.rindex(EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE)
        marker_offset = cookie_offset - 5
        corruptions = (
            (marker_offset + 1, b"\0\0\1\1", "encoding tag"),
            (cookie_offset, b"\0", "record cookie"),
            (cookie_offset + 8, b"\1", "reserved field"),
        )
        for offset, replacement, pattern in corruptions:
            with self.subTest(offset=offset):
                data = bytearray(original)
                data[offset : offset + len(replacement)] = replacement
                self.assert_ov_rejected(bytes(data), pattern)

    def test_nested_latency_layout_rejects_count_below_boundaries(self) -> None:
        data = bytearray(synthetic_nested_ov())
        cookie_offset = data.rindex(EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE)
        struct.pack_into(">I", data, cookie_offset + 16, 1)
        self.assert_ov_rejected(
            bytes(data),
            "implausible nested vector encoded count 1",
        )

    def test_nested_latency_layout_rejects_boundary_sentinel_corruption(
        self,
    ) -> None:
        data = bytearray(synthetic_nested_ov())
        cookie_offset = data.rindex(EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE)
        payload_offset = cookie_offset + 20
        struct.pack_into(">d", data, payload_offset, 0.0)
        self.assert_ov_rejected(bytes(data), "boundary sentinels")

    def test_nested_latency_layout_rejects_interior_boundary_sentinels(
        self,
    ) -> None:
        for sentinel in (
            EXTRACTOR.MISSING_SENTINEL,
            EXTRACTOR.ALL_VALUES_END_SENTINEL,
        ):
            with self.subTest(sentinel=sentinel):
                data = bytearray(synthetic_nested_ov())
                cookie_offset = data.rindex(
                    EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE
                )
                first_event_offset = cookie_offset + 20 + 8
                struct.pack_into(">d", data, first_event_offset, sentinel)
                self.assert_ov_rejected(
                    bytes(data), "interior value 1 reuses boundary sentinel"
                )

    def test_nested_latency_layout_rejects_nonmonotonic_time_axis(self) -> None:
        data = bytearray(synthetic_nested_ov())
        cookie_offset = data.rindex(EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE)
        payload_offset = cookie_offset + 20
        time_array_offset = payload_offset + 3 * 8
        struct.pack_into(">d", data, time_array_offset + 8, 800.0)
        struct.pack_into(">d", data, time_array_offset + 16, 600.0)
        self.assert_ov_rejected(bytes(data), "time axis is not nondecreasing")

    def test_zero_count_vector_is_marked_partial_without_samples(self) -> None:
        data = synthetic_ov(None, include_duration=False)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fragment-DES-1.ov"
            path.write_bytes(data)
            manifest = EXTRACTOR.extract_ov(path)
        self.assertEqual(manifest["status"], "partial")
        self.assertIsNone(manifest["execution"]["duration_s"])
        self.assertEqual(manifest["vectors"][0]["status"], "partial")
        self.assertEqual(manifest["vectors"][0]["buckets"], [])
        self.assertEqual(EXTRACTOR.aggregate_rows(manifest), [])
        self.assertIn(
            "contains no time axis or samples",
            manifest["vectors"][0]["warnings"][0],
        )

    def test_truncated_metadata_is_rejected(self) -> None:
        data = synthetic_ov([0.0] * 100)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "truncated.ov"
            path.write_bytes(data[:700])
            with self.assertRaises(EXTRACTOR.OvFormatError):
                EXTRACTOR.extract_ov(path)

    def test_duplicate_scalar_identifier_is_rejected(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov([0.0] * 100, duplicate_scalar=True),
            "duplicate execution scalar identifier",
        )

    def test_statistic_scope_and_name_must_be_nonempty(self) -> None:
        values = [0.0] * 100
        cases = (
            (synthetic_ov(values, scope_value=" "), "empty statistic scope"),
            (
                synthetic_ov(
                    values,
                    vector_specs=((" ", 9, values),),
                ),
                "empty statistic name",
            ),
            (synthetic_nested_ov(nested_scope=" "), "empty statistic scope"),
            (
                synthetic_nested_ov(nested_statistic_name=" "),
                "empty statistic name",
            ),
        )
        for data, pattern in cases:
            with self.subTest(pattern=pattern):
                self.assert_ov_rejected(data, pattern)

    def test_complete_vector_requires_duration(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov([0.0] * 100, include_duration=False),
            "cannot validate a complete vector without",
        )

    def test_complete_vector_requires_positive_duration(self) -> None:
        for duration_s in (0.0, -1.0):
            with self.subTest(duration_s=duration_s):
                self.assert_ov_rejected(
                    synthetic_ov([0.0] * 100, duration_s=duration_s),
                    "simulated duration .* must be positive",
                )

    def test_complete_vector_requires_source_observed_zero_start(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(
                [0.0] * 100,
                vector_start_s=-1.0,
                vector_width_s=600.01,
            ),
            "unsupported compressed time axis start -1",
        )

    def test_reconstructed_aggregate_axis_must_remain_finite(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(
                [0.0] * 100,
                duration_s=1.0e308,
                vector_width_s=1.0e308,
            ),
            "reconstructed aggregate bucket end is not finite",
        )

    def test_aggregate_count_mismatch_rejected_before_allocation(self) -> None:
        data = bytearray(synthetic_ov([0.0] * 100))
        cookie_offset = data.index(
            EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE, 512
        )
        struct.pack_into(">I", data, cookie_offset + 16, 100_001)
        tracemalloc.start()
        try:
            with self.assertRaisesRegex(
                EXTRACTOR.OvFormatError,
                "aggregate encoded count 100001 does not equal",
            ):
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "large-count-DES-1.ov"
                    path.write_bytes(data)
                    EXTRACTOR.extract_ov(path)
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        self.assertLess(peak, 2_000_000)

    def test_incorrect_declared_metadata_unit_count_is_rejected(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(
                [0.0] * 100,
                declared_metadata_unit_count=0x22,
            ),
            "declared vector metadata unit count 34 does not equal decoded count 3",
        )

    def test_complete_record_with_observed_0x165_details_is_supported(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "complete-165-DES-1.ov"
            path.write_bytes(
                synthetic_ov([0.0] * 100, details_flags=0x165)
            )
            manifest = EXTRACTOR.extract_ov(path)
        self.assertEqual(manifest["status"], "complete")
        self.assertEqual(
            manifest["vectors"][0]["metadata_details_flags"], "0x165"
        )

    def test_zero_count_record_requires_observed_0x165_details(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov(None, include_duration=False, details_flags=0x145),
            "zero-count vector record uses details flags 0x145",
        )

    def test_unobserved_details_flags_are_rejected(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov([0.0] * 100, details_flags=0x125),
            "unsupported statistic details 0x125",
        )

    def test_duplicate_description_identifier_is_rejected(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov([0.0] * 100, description_ids=(9, 9)),
            "duplicate description identifier 9",
        )

    def test_duplicate_vector_description_identifier_is_rejected(self) -> None:
        values = [0.0] * 100
        self.assert_ov_rejected(
            synthetic_ov(
                values,
                vector_specs=(
                    ("First (seconds)", 9, values),
                    ("Second (seconds)", 9, values),
                ),
            ),
            "duplicate vector description identifier 9",
        )

    def test_duplicate_canonical_vector_identity_is_rejected(self) -> None:
        values = [0.0] * 100
        self.assert_ov_rejected(
            synthetic_ov(
                values,
                description_ids=(9, 10),
                vector_specs=(
                    ("Repeated (seconds)", 9, values),
                    ("Repeated (seconds)", 10, values),
                ),
            ),
            "duplicate canonical vector identity",
        )

    def test_duplicate_probe_vector_identifier_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixture.pb.m"
            path.write_bytes(synthetic_pb(duplicate_probe_id=True))
            with self.assertRaisesRegex(
                EXTRACTOR.OvFormatError,
                "duplicate probe/vector identifier 'pb2'",
            ):
                EXTRACTOR.parse_probe_definition(path)

    def test_probe_definition_source_bounds_are_enforced(self) -> None:
        declared_data = bytearray(synthetic_pb())
        declared_count_offset = 512 + 4 + len("fixture")
        struct.pack_into(">I", declared_data, declared_count_offset, 28)

        tag_flood = (
            model_header("modeler")
            + string("fixture")
            + u32(0)
            + (u32(0x0A) + u32(0)) * 490
        )
        oversized = synthetic_pb() + b"\0" * (
            EXTRACTOR.RECOVERED_MAX_PB_FILE_SIZE + 1 - len(synthetic_pb())
        )
        cases = (
            (bytes(declared_data), "declared probe count 28 exceeds"),
            (tag_flood, "printable tagged-string count exceeds"),
            (oversized, "probe-definition size 15124 exceeds"),
        )
        for data, pattern in cases:
            with self.subTest(pattern=pattern):
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "bounded.pb.m"
                    path.write_bytes(data)
                    with self.assertRaisesRegex(
                        EXTRACTOR.OvFormatError, pattern
                    ):
                        EXTRACTOR.parse_probe_definition(path)

    def test_unobserved_record_cookie_is_rejected(self) -> None:
        data = bytearray(synthetic_ov([0.0] * 100))
        offset = data.index(EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE, 512)
        data[offset] ^= 0x01
        self.assert_ov_rejected(bytes(data), "unexpected vector record cookie")

    def test_nonzero_record_reserved_field_is_rejected(self) -> None:
        data = bytearray(synthetic_ov([0.0] * 100))
        cookie_offset = data.index(
            EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE, 512
        )
        data[cookie_offset + len(EXTRACTOR.RECOVERED_VECTOR_RECORD_COOKIE)] = 1
        self.assert_ov_rejected(bytes(data), "nonzero vector reserved field")

    def test_unobserved_complete_record_trailer_is_rejected(self) -> None:
        data = bytearray(synthetic_ov([0.0] * 100))
        offset = data.rindex(EXTRACTOR.RECOVERED_COMPLETE_VECTOR_TRAILER)
        data[offset + 1] ^= 0x01
        self.assert_ov_rejected(
            bytes(data), "unexpected complete vector trailer"
        )

    def test_partial_extraction_retains_json_diagnostics_but_no_csv_rows(
        self,
    ) -> None:
        data = synthetic_ov([0.0] * 100, duration_s=60001.0)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            json_path = root / "fixture.json"
            csv_path = root / "fixture.csv"
            ov_path.write_bytes(data)

            manifest = EXTRACTOR.extract_ov(ov_path)
            self.assertEqual(manifest["status"], "partial")
            self.assertEqual(
                manifest["vectors"][0]["decoded_value_count"], 0
            )
            self.assertEqual(EXTRACTOR.aggregate_rows(manifest), [])

            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                return_code = EXTRACTOR.main(
                    [
                        str(ov_path),
                        "--json",
                        str(json_path),
                        "--csv",
                        str(csv_path),
                        "--strict",
                    ]
                )
            diagnostic = json.loads(json_path.read_text(encoding="utf-8"))
            rows = list(
                csv.DictReader(
                    io.StringIO(csv_path.read_text(encoding="utf-8"))
                )
            )

        self.assertEqual(return_code, 3)
        self.assertIn("extraction status is partial", stderr.getvalue())
        self.assertEqual(diagnostic["status"], "partial")
        self.assertEqual(
            diagnostic["vectors"][0]["decoded_value_count"], 0
        )
        self.assertEqual(rows, [])

    def test_probe_scenario_mismatch_is_not_used(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "different-DES-1.ov"
            pb_path = root / "fixture.pb.m"
            ov_path.write_bytes(synthetic_ov([0.0] * 100))
            pb_path.write_bytes(synthetic_pb())
            manifest = EXTRACTOR.extract_ov(
                ov_path, probe_definition=pb_path
            )
        self.assertEqual(
            manifest["vectors"][0]["probe_definition_match"]["status"],
            "scenario_mismatch",
        )

    def test_json_output_cannot_alias_input(self) -> None:
        data = synthetic_ov([0.0] * 100)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixture-DES-1.ov"
            path.write_bytes(data)
            return_code, stderr = self.run_main(
                [str(path), "--json", str(path)]
            )
            retained = path.read_bytes()
        self.assertEqual(return_code, 2)
        self.assertIn("JSON output", stderr)
        self.assertIn("aliases input", stderr)
        self.assertEqual(retained, data)

    def test_csv_output_cannot_alias_probe_definition(self) -> None:
        probe_data = synthetic_pb()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            pb_path = root / "fixture.pb.m"
            ov_path.write_bytes(synthetic_ov([0.0] * 100))
            pb_path.write_bytes(probe_data)
            return_code, stderr = self.run_main(
                [
                    str(ov_path),
                    "--probe-definition",
                    str(pb_path),
                    "--csv",
                    str(pb_path),
                ]
            )
            retained = pb_path.read_bytes()
        self.assertEqual(return_code, 2)
        self.assertIn("CSV output", stderr)
        self.assertIn("aliases probe definition", stderr)
        self.assertEqual(retained, probe_data)

    def test_json_and_csv_outputs_must_be_distinct(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            output_path = root / "same-output"
            ov_path.write_bytes(synthetic_ov([0.0] * 100))
            return_code, stderr = self.run_main(
                [
                    str(ov_path),
                    "--json",
                    str(output_path),
                    "--csv",
                    str(output_path),
                ]
            )
            output_exists = output_path.exists()
        self.assertEqual(return_code, 2)
        self.assertIn("outputs must be distinct", stderr)
        self.assertFalse(output_exists)

    def test_symlink_output_alias_cannot_overwrite_input(self) -> None:
        data = synthetic_ov([0.0] * 100)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            output_link = root / "output.json"
            ov_path.write_bytes(data)
            output_link.symlink_to(ov_path)
            return_code, stderr = self.run_main(
                [str(ov_path), "--json", str(output_link)]
            )
            retained = ov_path.read_bytes()
        self.assertEqual(return_code, 2)
        self.assertIn("aliases input", stderr)
        self.assertEqual(retained, data)

    def test_hard_link_output_alias_cannot_overwrite_input(self) -> None:
        data = synthetic_ov([0.0] * 100)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ov_path = root / "fixture-DES-1.ov"
            output_link = root / "output.csv"
            ov_path.write_bytes(data)
            output_link.hardlink_to(ov_path)
            return_code, stderr = self.run_main(
                [str(ov_path), "--csv", str(output_link)]
            )
            retained = ov_path.read_bytes()
        self.assertEqual(return_code, 2)
        self.assertIn("aliases input", stderr)
        self.assertEqual(retained, data)


if __name__ == "__main__":
    unittest.main()
