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
    vector_section_tag: int = 0x14,
    details_flags: int | None = None,
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
    body += u32(vector_section_tag)

    if vector_specs is None:
        vector_specs = (("End-to-End Delay (seconds)", 9, values),)

    body += u32(0x23) + string("Sink")
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
        body += u32(record_details_flags) + string("none") + u32(0xFFFFFFFF)
        body += string("Bucket: sample mean: total of 100 values")
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
            body += vector_block(vector_values, start=0.0, width=600.0)
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

    def test_unobserved_vector_section_tag_is_rejected(self) -> None:
        self.assert_ov_rejected(
            synthetic_ov([0.0] * 100, vector_section_tag=0x22),
            "unsupported vector section tag 0x22",
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
                manifest["vectors"][0]["decoded_value_count"], 100
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
            diagnostic["vectors"][0]["decoded_value_count"], 100
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
