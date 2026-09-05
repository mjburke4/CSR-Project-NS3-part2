#!/usr/bin/env python3
"""Fail-closed tests for the source-derived reservation/collision validator."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]


def load_script(module_name: str, filename: str):
    specification = importlib.util.spec_from_file_location(
        module_name, ROOT / "utils" / filename
    )
    module = importlib.util.module_from_spec(specification)
    sys.modules[module_name] = module
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


VALIDATOR = load_script(
    "csr_opnet_reservation_collision_validator",
    "validate-opnet-reservation-collision.py",
)


class ReservationCollisionValidatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.trace = self.root / "trace.csv"
        self.manifest = self.root / "manifest.json"
        self.generated_payloads = {
            name: f"generated fixture for {name}\n".encode("utf-8")
            for name in VALIDATOR.EXPECTED_PATCHED_OUTPUT_SHA256
        }
        self.output_digest_patch = mock.patch.dict(
            VALIDATOR.EXPECTED_PATCHED_OUTPUT_SHA256,
            {
                name: VALIDATOR.hashlib.sha256(payload).hexdigest()
                for name, payload in self.generated_payloads.items()
            },
            clear=True,
        )
        self.output_digest_patch.start()

    def tearDown(self) -> None:
        self.output_digest_patch.stop()
        self.temporary.cleanup()

    @staticmethod
    def row(time_s: float, event: str, node: int, **values: object) -> dict[str, str]:
        row = {name: "" for name in VALIDATOR.OPNET_TRACE_COLUMNS}
        row.update(
            {
                "schema": VALIDATOR.COMPARATOR.TRACE_SCHEMA,
                "time_s": repr(time_s),
                "event": event,
                "node": str(node),
            }
        )
        row.update({name: str(value) for name, value in values.items()})
        return row

    def base_rows(
        self,
        *,
        holdoff: bool = False,
        rate_kbps: int = 8,
        size_bytes: int = 600,
        preamble: str = "long",
    ) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        prepare_time = 59.9 if holdoff else 60.0
        holdoff_time = prepare_time + 0.3
        tick_start = math.ceil(
            (
                prepare_time
                + (VALIDATOR.HOLDOFF_S if holdoff else 0.0)
                + VALIDATOR.SLOT_CYCLE_S
            )
            / VALIDATOR.CONTROL_EPOCH_S
        ) * VALIDATOR.CONTROL_EPOCH_S
        tx_time = tick_start + 5 * VALIDATOR.SLOT_CYCLE_S
        completion = tx_time + VALIDATOR.source_airtime_s(
            rate_kbps, size_bytes, preamble
        )
        for node in (2, 3):
            rows.append(
                self.row(
                    prepare_time,
                    "reservation_prepare",
                    node,
                    reservation_slot=5,
                    reservation_counter=5,
                    reason="controlled_wait" if holdoff else "controlled_ready",
                )
            )
            if holdoff:
                rows.append(
                    self.row(
                        holdoff_time,
                        "reservation_holdoff",
                        node,
                        reservation_slot=5,
                        reservation_counter=5,
                    )
                )
            for offset, counter in enumerate(range(4, -2, -1)):
                rows.append(
                    self.row(
                        tick_start + 0.013 * offset,
                        "reservation_tick",
                        node,
                        reservation_slot=5,
                        reservation_counter=counter,
                    )
                )
            rows.append(
                self.row(
                    tx_time,
                    "reservation_advertise",
                    node,
                    reservation_slot=5,
                    reservation_counter=5,
                )
            )
            rows.append(
                self.row(
                    tx_time,
                    "tx_start",
                    node,
                    peer=1,
                    src=node,
                    dst=1,
                    sequence=100 + node,
                    rate_kbps=rate_kbps,
                    size_bytes=size_bytes,
                    reservation_slot=5,
                    reservation_counter=5,
                    detail=f"preamble={preamble}",
                )
            )
            rows.append(
                self.row(
                    completion,
                    "rx_drop",
                    1,
                    peer=node,
                    src=node,
                    rate_kbps=rate_kbps,
                    size_bytes=size_bytes,
                    success=0,
                    reason="ecc",
                    detail="collisions=1",
                )
            )
        return sorted(rows, key=lambda item: float(item["time_s"]))

    def write_inputs(self, rows: list[dict[str, str]]) -> argparse.Namespace:
        with self.trace.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=VALIDATOR.OPNET_TRACE_COLUMNS,
                lineterminator="\n",
            )
            writer.writeheader()
            writer.writerows(sorted(rows, key=lambda item: float(item["time_s"])))
        for name, payload in self.generated_payloads.items():
            (self.root / name).write_bytes(payload)
        forced = {"2": 5, "3": 5}
        header = VALIDATOR.render_expected_header(forced, 59.9)
        (self.root / VALIDATOR.HEADER_NAME).write_bytes(header)
        sources = {
            name: {
                "original_sha256": original,
                "output_sha256": (
                    VALIDATOR.hashlib.sha256(self.generated_payloads[name]).hexdigest()
                    if name in self.generated_payloads
                    else None
                ),
            }
            for name, original in VALIDATOR.EXPECTED_ORIGINAL_SHA256.items()
        }
        template_path = (
            ROOT / "utils" / "opnet" / VALIDATOR.HEADER_NAME
        )
        sources[VALIDATOR.HEADER_NAME] = {
            "original_sha256": VALIDATOR.digest(template_path),
            "output_sha256": VALIDATOR.hashlib.sha256(header).hexdigest(),
        }
        generator_path = ROOT / "utils" / VALIDATOR.GENERATOR_NAME
        manifest = {
            "schema": VALIDATOR.INSTRUMENTATION_SCHEMA,
            "trace_schema": VALIDATOR.TRACE_SCHEMA,
            "generator": {
                "name": VALIDATOR.GENERATOR_NAME,
                "sha256": VALIDATOR.digest(generator_path),
            },
            "forced_reservation_slots": forced,
            "activation_time_s": 59.9,
            "behavioral_control": VALIDATOR.EXPECTED_BEHAVIORAL_CONTROL,
            "sources": sources,
        }
        self.manifest.write_text(
            json.dumps(manifest) + "\n", encoding="utf-8"
        )
        return VALIDATOR.build_parser().parse_args(
            [
                str(self.trace),
                "--manifest",
                str(self.manifest),
                "--receiver",
                "1",
                "--transmitters",
                "2,3",
                "--slot",
                "5",
                "--time-start",
                "59.8",
                "--time-stop",
                "63",
            ]
        )

    def validate_rows(self, rows: list[dict[str, str]]) -> dict[str, object]:
        return VALIDATOR.validate(self.write_inputs(rows))

    def read_manifest(self) -> dict[str, object]:
        return json.loads(self.manifest.read_text(encoding="utf-8"))

    def write_manifest(self, manifest: object) -> None:
        self.manifest.write_text(json.dumps(manifest) + "\n", encoding="utf-8")

    @staticmethod
    def find(
        rows: list[dict[str, str]], event: str, *, node: int | None = None, peer: int | None = None
    ) -> dict[str, str]:
        return next(
            row
            for row in rows
            if row["event"] == event
            and (node is None or row["node"] == str(node))
            and (peer is None or row["peer"] == str(peer))
        )

    def assert_failed(self, report: dict[str, object], text: str) -> None:
        self.assertFalse(report["pass"])
        self.assertTrue(
            any(text in failure for failure in report["failures"]),
            report["failures"],
        )

    def retime_outcome(self, rows: list[dict[str, str]], node: int) -> None:
        tx = self.find(rows, "tx_start", node=node)
        outcome = self.find(rows, "rx_drop", peer=node)
        outcome["time_s"] = repr(
            float(tx["time_s"])
            + VALIDATOR.source_airtime_s(
                int(tx["rate_kbps"]),
                int(tx["size_bytes"]),
                VALIDATOR.parse_detail(tx["detail"])["preamble"],
            )
        )

    # Parser and source-arithmetic contracts.
    def test_parse_transmitters_accepts_unique_values(self) -> None:
        self.assertEqual(VALIDATOR.parse_transmitters("0x2,3"), (2, 3))

    def test_parse_transmitters_rejects_too_few_or_duplicates(self) -> None:
        for value in ("2", "2,2"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                VALIDATOR.parse_transmitters(value)

    def test_parse_detail_accepts_strict_fields(self) -> None:
        self.assertEqual(
            VALIDATOR.parse_detail("preamble=long;collisions=2"),
            {"preamble": "long", "collisions": "2"},
        )

    def test_parse_detail_rejects_malformed_or_duplicate_fields(self) -> None:
        for value in ("preamble", "preamble=long;preamble=short", "=long"):
            with self.subTest(value=value), self.assertRaises(
                VALIDATOR.ValidationInputError
            ):
                VALIDATOR.parse_detail(value)

    def test_source_airtime_uses_every_exact_nibble_duration(self) -> None:
        for rate, duration in VALIDATOR.RATE_NIBBLE_DURATION_S.items():
            with self.subTest(rate=rate):
                expected = (
                    (7888 + 48) / 4 * 0.000510
                    + (600 * 8 + 32) / 4 * duration
                )
                self.assertEqual(
                    VALIDATOR.source_airtime_s(rate, 600, "long"), expected
                )
        expected = (104 + 48) / 4 * 0.000510 + (8 + 32) / 4 * 0.000030
        self.assertEqual(VALIDATOR.source_airtime_s(128, 1, "short"), expected)

    def test_source_airtime_rejects_unsupported_inputs(self) -> None:
        for arguments in ((500, 600, "long"), (8, 0, "long"), (8, 1, "other")):
            with self.subTest(arguments=arguments), self.assertRaises(
                VALIDATOR.ValidationInputError
            ):
                VALIDATOR.source_airtime_s(*arguments)

    # Positive end-to-end contracts.
    def test_valid_ready_long_preamble_collision(self) -> None:
        self.assertTrue(self.validate_rows(self.base_rows())["pass"])

    def test_valid_wait_short_preamble_collision(self) -> None:
        self.assertTrue(
            self.validate_rows(
                self.base_rows(holdoff=True, rate_kbps=128, preamble="short")
            )["pass"]
        )

    def test_completion_positive_tolerance_boundary_is_inclusive(self) -> None:
        rows = self.base_rows()
        outcome = self.find(rows, "rx_drop", peer=2)
        outcome["time_s"] = repr(float(outcome["time_s"]) + 0.001)
        self.assertTrue(self.validate_rows(rows)["pass"])
        arguments = self.write_inputs(self.base_rows())
        arguments.completion_tolerance = 0.001001
        with self.assertRaisesRegex(
            VALIDATOR.ValidationInputError, "must not exceed 0.001 s"
        ):
            VALIDATOR.validate(arguments)

    def test_completion_negative_tolerance_boundary_is_inclusive(self) -> None:
        rows = self.base_rows()
        outcome = self.find(rows, "rx_drop", peer=2)
        outcome["time_s"] = repr(float(outcome["time_s"]) - 0.001)
        self.assertTrue(self.validate_rows(rows)["pass"])

    def test_unrelated_other_peer_outcome_is_ignored(self) -> None:
        rows = self.base_rows()
        rows.append(
            self.row(
                61.0,
                "rx_drop",
                1,
                peer=9,
                rate_kbps=8,
                size_bytes=600,
                success=0,
                detail="collisions=9",
            )
        )
        self.assertTrue(self.validate_rows(rows)["pass"])

    def test_report_binds_source_airtime_and_exact_rows(self) -> None:
        report = self.validate_rows(self.base_rows())
        self.assertEqual(report["bound_receiver_outcome_count"], 2)
        self.assertEqual(
            report["source_airtime_contract"]["rate_nibble_duration_s"],
            VALIDATOR.RATE_NIBBLE_DURATION_S,
        )
        self.assertEqual(report["receiver_evidence"]["2"]["bound_outcome_event"], "rx_drop")
        self.assertAlmostEqual(report["receiver_evidence"]["2"]["completion_error_s"], 0.0)

    # Transmit identity must be complete and source-supported.
    def test_rejects_500_kbps_transmit(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["rate_kbps"] = "500"
        self.assert_failed(self.validate_rows(rows), "not one of 8,16,32,64,128")

    def test_rejects_1000_kbps_transmit(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["rate_kbps"] = "1000"
        self.assert_failed(self.validate_rows(rows), "not one of 8,16,32,64,128")

    def test_rejects_missing_transmit_rate(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["rate_kbps"] = ""
        self.assert_failed(self.validate_rows(rows), "rate None")

    def test_rejects_zero_transmit_size(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["size_bytes"] = "0"
        self.assert_failed(self.validate_rows(rows), "size 0 is not positive")

    def test_rejects_negative_transmit_size(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["size_bytes"] = "-1"
        self.assert_failed(self.validate_rows(rows), "size -1 is not positive")

    def test_rejects_missing_transmit_size(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["size_bytes"] = ""
        self.assert_failed(self.validate_rows(rows), "size None is not positive")

    def test_rejects_missing_transmit_preamble(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["detail"] = ""
        self.assert_failed(self.validate_rows(rows), "preamble None")

    def test_rejects_unknown_transmit_preamble(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["detail"] = "preamble=unknown"
        self.assert_failed(self.validate_rows(rows), "not long or short")

    def test_rejects_duplicate_transmit_preamble_detail(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["detail"] = (
            "preamble=long;preamble=short"
        )
        self.assert_failed(self.validate_rows(rows), "duplicate detail key")

    def test_rejects_transmit_to_wrong_receiver(self) -> None:
        rows = self.base_rows()
        self.find(rows, "tx_start", node=2)["peer"] = "9"
        self.assert_failed(self.validate_rows(rows), "is not receiver 1")

    # The same outcome row must bind identity, completion, drop, and collision.
    def test_rejects_receiver_outcome_with_wrong_rate(self) -> None:
        rows = self.base_rows()
        self.find(rows, "rx_drop", peer=2)["rate_kbps"] = "16"
        self.assert_failed(self.validate_rows(rows), "0 exact identity/time-bound outcomes")

    def test_rejects_receiver_outcome_with_wrong_size(self) -> None:
        rows = self.base_rows()
        self.find(rows, "rx_drop", peer=2)["size_bytes"] = "601"
        self.assert_failed(self.validate_rows(rows), "0 exact identity/time-bound outcomes")

    def test_rejects_receiver_outcome_after_completion_window(self) -> None:
        rows = self.base_rows()
        outcome = self.find(rows, "rx_drop", peer=2)
        outcome["time_s"] = repr(float(outcome["time_s"]) + 0.001001)
        self.assert_failed(self.validate_rows(rows), "0 exact identity/time-bound outcomes")

    def test_rejects_receiver_outcome_before_completion_window(self) -> None:
        rows = self.base_rows()
        outcome = self.find(rows, "rx_drop", peer=2)
        outcome["time_s"] = repr(float(outcome["time_s"]) - 0.001001)
        self.assert_failed(self.validate_rows(rows), "0 exact identity/time-bound outcomes")

    def test_rejects_bound_accept_outcome(self) -> None:
        rows = self.base_rows()
        outcome = self.find(rows, "rx_drop", peer=2)
        outcome["event"] = "rx_accept"
        outcome["success"] = "1"
        self.assert_failed(self.validate_rows(rows), "is not a drop")

    def test_rejects_bound_drop_without_collision(self) -> None:
        rows = self.base_rows()
        self.find(rows, "rx_drop", peer=2)["detail"] = "collisions=0"
        self.assert_failed(self.validate_rows(rows), "no positive collision count")

    def test_rejects_malformed_bound_collision_count(self) -> None:
        rows = self.base_rows()
        self.find(rows, "rx_drop", peer=2)["detail"] = "collisions=one"
        self.assert_failed(self.validate_rows(rows), "malformed collision detail")

    def test_rejects_split_bound_accept_and_drop_rows(self) -> None:
        rows = self.base_rows()
        accepted = dict(self.find(rows, "rx_drop", peer=2))
        accepted["event"] = "rx_accept"
        accepted["success"] = "1"
        rows.append(accepted)
        self.assert_failed(self.validate_rows(rows), "2 exact identity/time-bound outcomes")

    def test_rejects_duplicate_bound_drop_rows(self) -> None:
        rows = self.base_rows()
        rows.append(dict(self.find(rows, "rx_drop", peer=2)))
        self.assert_failed(self.validate_rows(rows), "2 exact identity/time-bound outcomes")

    def test_unrelated_later_collision_cannot_satisfy_bound_drop(self) -> None:
        rows = self.base_rows()
        bound = self.find(rows, "rx_drop", peer=2)
        bound["detail"] = "collisions=0"
        later = dict(bound)
        later["time_s"] = repr(float(bound["time_s"]) + 0.5)
        later["detail"] = "collisions=4"
        rows.append(later)
        self.assert_failed(self.validate_rows(rows), "no positive collision count")

    def test_collision_accept_plus_collisionless_drop_cannot_split_proof(self) -> None:
        rows = self.base_rows()
        dropped = self.find(rows, "rx_drop", peer=2)
        dropped["detail"] = "collisions=0"
        accepted = dict(dropped)
        accepted["event"] = "rx_accept"
        accepted["success"] = "1"
        accepted["detail"] = "collisions=2"
        rows.append(accepted)
        self.assert_failed(self.validate_rows(rows), "2 exact identity/time-bound outcomes")

    # Every lifecycle field and source-derived timestamp is bound.
    def test_rejects_tampered_lifecycle_slot_and_counter_fields(self) -> None:
        cases = (
            (False, "reservation_prepare", "reservation_slot", "prepare slot"),
            (False, "reservation_prepare", "reservation_counter", "prepare counter"),
            (True, "reservation_holdoff", "reservation_slot", "holdoff slot"),
            (True, "reservation_holdoff", "reservation_counter", "holdoff counter"),
            (False, "reservation_tick", "reservation_slot", "tick slot"),
            (False, "reservation_tick", "reservation_counter", "tick counter"),
            (False, "reservation_advertise", "reservation_slot", "advertise slot"),
            (False, "reservation_advertise", "reservation_counter", "advertise counter"),
            (False, "tx_start", "reservation_slot", "tx reservation slot"),
            (False, "tx_start", "reservation_counter", "tx reservation counter"),
        )
        for holdoff, event, field, failure in cases:
            with self.subTest(event=event, field=field):
                rows = self.base_rows(holdoff=holdoff)
                self.find(rows, event, node=2)[field] = "99"
                self.assert_failed(self.validate_rows(rows), failure)

    def test_rejects_inconsistent_transmit_identity(self) -> None:
        cases = (
            ("src", "9", "source"),
            ("dst", "9", "destination"),
            ("sequence", "", "sequence"),
            ("sequence", "-1", "sequence"),
            ("sequence", "65536", "sequence"),
        )
        for field, value, failure in cases:
            with self.subTest(field=field, value=value):
                rows = self.base_rows()
                self.find(rows, "tx_start", node=2)[field] = value
                self.assert_failed(self.validate_rows(rows), failure)

    def test_rejects_inconsistent_receiver_source(self) -> None:
        rows = self.base_rows()
        self.find(rows, "rx_drop", peer=2)["src"] = "9"
        self.assert_failed(self.validate_rows(rows), "0 exact identity/time-bound outcomes")

    def test_rejects_tick_cadence_tamper(self) -> None:
        rows = self.base_rows()
        ticks = [
            row
            for row in rows
            if row["event"] == "reservation_tick" and row["node"] == "2"
        ]
        ticks[1]["time_s"] = repr(float(ticks[1]["time_s"]) + 0.001)
        self.assert_failed(self.validate_rows(rows), "tick cadence")

    def test_rejects_first_tick_epoch_tamper(self) -> None:
        rows = self.base_rows()
        for row in rows:
            if row["node"] == "2" and row["event"] in {
                "reservation_tick",
                "reservation_advertise",
                "tx_start",
            }:
                row["time_s"] = repr(float(row["time_s"]) + 0.001)
        self.retime_outcome(rows, 2)
        self.assert_failed(self.validate_rows(rows), "source epoch")

    def test_rejects_advertise_or_transmit_off_final_tick(self) -> None:
        for event in ("reservation_advertise", "tx_start"):
            with self.subTest(event=event):
                rows = self.base_rows()
                target = self.find(rows, event, node=2)
                target["time_s"] = repr(float(target["time_s"]) - 0.001)
                if event == "tx_start":
                    self.retime_outcome(rows, 2)
                self.assertFalse(self.validate_rows(rows)["pass"])

    def test_rejects_spliced_lifecycle_events(self) -> None:
        for event in ("reservation_holdoff", "reservation_prepare", "tx_start"):
            with self.subTest(event=event):
                rows = self.base_rows()
                if event == "reservation_holdoff":
                    injected = self.row(
                        60.05,
                        event,
                        2,
                        reservation_slot=5,
                        reservation_counter=5,
                    )
                elif event == "reservation_prepare":
                    injected = self.row(
                        60.05,
                        event,
                        2,
                        reservation_slot=5,
                        reservation_counter=5,
                        reason="controlled_ready",
                    )
                else:
                    injected = dict(self.find(rows, "tx_start", node=2))
                    injected["time_s"] = "60.05"
                rows.append(injected)
                self.assert_failed(self.validate_rows(rows), "no ordered controlled")

    def test_rejects_missing_duplicate_or_out_of_order_tick(self) -> None:
        for operation in ("missing", "duplicate", "out_of_order"):
            with self.subTest(operation=operation):
                rows = self.base_rows()
                ticks = [
                    row
                    for row in rows
                    if row["event"] == "reservation_tick" and row["node"] == "2"
                ]
                if operation == "missing":
                    rows.remove(ticks[2])
                elif operation == "duplicate":
                    rows.append(dict(ticks[2]))
                else:
                    ticks[2]["reservation_counter"] = "1"
                self.assertFalse(self.validate_rows(rows)["pass"])

    def test_rejects_relaxed_structural_tolerances(self) -> None:
        for field in (
            "simultaneous_tolerance",
            "holdoff_tolerance",
            "slot_tolerance",
        ):
            with self.subTest(field=field):
                arguments = self.write_inputs(self.base_rows())
                setattr(arguments, field, 1.0e-5)
                with self.assertRaisesRegex(
                    VALIDATOR.ValidationInputError, "must not exceed 0.000001 s"
                ):
                    VALIDATOR.validate(arguments)

    # The instrumentation manifest and co-located generated outputs are trusted.
    def test_rejects_nonobject_or_duplicate_key_manifest(self) -> None:
        arguments = self.write_inputs(self.base_rows())
        for text, failure in (
            ("[]\n", "root is not an object"),
            ('{"schema":"one","schema":"two"}\n', "duplicate JSON key"),
        ):
            with self.subTest(failure=failure):
                self.manifest.write_text(text, encoding="utf-8")
                with self.assertRaisesRegex(VALIDATOR.ValidationInputError, failure):
                    VALIDATOR.validate(arguments)

    def test_rejects_missing_wrong_or_extra_manifest_fields(self) -> None:
        for operation in ("missing", "wrong_trace_schema", "extra"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                if operation == "missing":
                    del manifest["trace_schema"]
                elif operation == "wrong_trace_schema":
                    manifest["trace_schema"] = "other"
                else:
                    manifest["unexpected"] = True
                self.write_manifest(manifest)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_generator_record_tamper(self) -> None:
        for operation in ("shape", "name", "digest"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                if operation == "shape":
                    manifest["generator"]["extra"] = "field"
                elif operation == "name":
                    manifest["generator"]["name"] = "other.py"
                else:
                    manifest["generator"]["sha256"] = "0" * 64
                self.write_manifest(manifest)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_source_set_or_record_shape_tamper(self) -> None:
        for operation in ("missing", "extra", "nonobject", "record_extra"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                if operation == "missing":
                    del manifest["sources"]["br_support.h"]
                elif operation == "extra":
                    manifest["sources"]["other.c"] = {}
                elif operation == "nonobject":
                    manifest["sources"] = []
                else:
                    manifest["sources"]["br_mac.pr.c"]["extra"] = True
                self.write_manifest(manifest)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_original_source_identity_tamper(self) -> None:
        for name in VALIDATOR.EXPECTED_ORIGINAL_SHA256:
            with self.subTest(name=name):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                manifest["sources"][name]["original_sha256"] = "0" * 64
                self.write_manifest(manifest)
                with self.assertRaisesRegex(
                    VALIDATOR.ValidationInputError, "original digest is not trusted"
                ):
                    VALIDATOR.validate(arguments)

    def test_rejects_malformed_or_uppercase_source_digest(self) -> None:
        for digest in ("short", "A" * 64):
            with self.subTest(digest=digest):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                manifest["sources"]["br_mac.pr.c"]["original_sha256"] = digest
                self.write_manifest(manifest)
                with self.assertRaisesRegex(
                    VALIDATOR.ValidationInputError, "lowercase SHA-256"
                ):
                    VALIDATOR.validate(arguments)

    def test_rejects_reference_or_patched_output_digest_tamper(self) -> None:
        for operation in ("reference_output", "patched_output"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                if operation == "reference_output":
                    manifest["sources"]["br_txdel.ps.c"]["output_sha256"] = (
                        "0" * 64
                    )
                else:
                    manifest["sources"]["br_mac.pr.c"]["output_sha256"] = (
                        "0" * 64
                    )
                self.write_manifest(manifest)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_missing_tampered_or_symlinked_generated_output(self) -> None:
        for operation in ("missing", "tampered", "symlink"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                output = self.root / "br_mac.pr.c"
                if operation == "missing":
                    output.unlink()
                elif operation == "tampered":
                    output.write_bytes(b"tampered\n")
                else:
                    output.unlink()
                    output.symlink_to(self.root / "br_app.pr.c")
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_header_template_or_generated_digest_tamper(self) -> None:
        for field in ("original_sha256", "output_sha256"):
            with self.subTest(field=field):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                manifest["sources"][VALIDATOR.HEADER_NAME][field] = "0" * 64
                self.write_manifest(manifest)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_activation_tamper_even_with_rehashed_header(self) -> None:
        arguments = self.write_inputs(self.base_rows())
        manifest = self.read_manifest()
        manifest["activation_time_s"] = 59.8
        header = VALIDATOR.render_expected_header(
            manifest["forced_reservation_slots"], 59.8
        )
        (self.root / VALIDATOR.HEADER_NAME).write_bytes(header)
        manifest["sources"][VALIDATOR.HEADER_NAME]["output_sha256"] = (
            VALIDATOR.hashlib.sha256(header).hexdigest()
        )
        self.write_manifest(manifest)
        with self.assertRaisesRegex(
            VALIDATOR.ValidationInputError, "activation_time_s is not 59.9"
        ):
            VALIDATOR.validate(arguments)

    def test_rejects_extra_or_malformed_forced_slot(self) -> None:
        for operation in ("extra", "bool", "receiver"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                manifest = self.read_manifest()
                if operation == "extra":
                    manifest["forced_reservation_slots"]["4"] = 5
                elif operation == "bool":
                    manifest["forced_reservation_slots"]["2"] = True
                else:
                    manifest["forced_reservation_slots"]["1"] = 5
                self.write_manifest(manifest)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_wrong_trace_schema_or_header(self) -> None:
        for operation in ("row_schema", "header"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                text = self.trace.read_text(encoding="utf-8")
                if operation == "row_schema":
                    text = text.replace(VALIDATOR.TRACE_SCHEMA, "other", 1)
                else:
                    text = text.replace("schema,event_index", "schema,extra,event_index", 1)
                self.trace.write_text(text, encoding="utf-8")
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_malformed_or_infinite_integer_without_traceback(self) -> None:
        for field, value in (("rate_kbps", "inf"), ("sequence", "1e309")):
            with self.subTest(field=field, value=value):
                rows = self.base_rows()
                self.find(rows, "tx_start", node=2)[field] = value
                arguments = self.write_inputs(rows)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    def test_rejects_nonfinite_number_field_without_traceback(self) -> None:
        for value, message in (
            ("not-a-number", "not numeric"),
            ("nan", "not finite"),
            ("inf", "not finite"),
            ("-inf", "not finite"),
            ("1e309", "not finite"),
        ):
            with self.subTest(value=value):
                rows = self.base_rows()
                arguments = self.write_inputs(rows)
                self.find(rows, "reservation_prepare", node=2)["time_s"] = value
                with self.trace.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(
                        stream,
                        fieldnames=VALIDATOR.OPNET_TRACE_COLUMNS,
                        lineterminator="\n",
                    )
                    writer.writeheader()
                    writer.writerows(rows)
                with self.assertRaisesRegex(
                    VALIDATOR.ValidationInputError, message
                ):
                    VALIDATOR.validate(arguments)

    def test_rejects_wide_or_short_csv_rows(self) -> None:
        for operation in ("wide", "short"):
            with self.subTest(operation=operation):
                arguments = self.write_inputs(self.base_rows())
                lines = self.trace.read_text(encoding="utf-8").splitlines()
                if operation == "wide":
                    lines[1] += ",unexpected"
                else:
                    lines[1] = ",".join(lines[1].split(",")[:-1])
                self.trace.write_text("\n".join(lines) + "\n", encoding="utf-8")
                with self.assertRaisesRegex(
                    VALIDATOR.ValidationInputError, "exact column width"
                ):
                    VALIDATOR.validate(arguments)

    def test_rejects_report_path_aliases_and_preserves_inputs(self) -> None:
        for target_name in ("trace", "manifest"):
            for alias in ("same", "symlink", "hardlink"):
                with self.subTest(target=target_name, alias=alias):
                    arguments = self.write_inputs(self.base_rows())
                    target = self.trace if target_name == "trace" else self.manifest
                    target_bytes = target.read_bytes()
                    if alias == "same":
                        report = target
                    else:
                        report = self.root / f"report-{target_name}-{alias}.json"
                        if alias == "symlink":
                            report.symlink_to(target)
                        else:
                            report.hardlink_to(target)
                    arguments.report = report
                    with self.assertRaisesRegex(
                        VALIDATOR.ValidationInputError,
                        f"report path aliases {target_name}",
                    ):
                        VALIDATOR.validate(arguments)
                    self.assertEqual(target.read_bytes(), target_bytes)

    def test_rejects_receiver_transmitter_overlap_or_out_of_range_node(self) -> None:
        for field, value in (("receiver", 2), ("receiver", 0x1000000)):
            with self.subTest(field=field, value=value):
                arguments = self.write_inputs(self.base_rows())
                setattr(arguments, field, value)
                with self.assertRaises(VALIDATOR.ValidationInputError):
                    VALIDATOR.validate(arguments)

    # Existing lifecycle invariants remain independently enforced.
    def test_rejects_inexact_source_holdoff_duration(self) -> None:
        rows = self.base_rows(holdoff=True)
        for row in rows:
            if row["event"] == "reservation_holdoff":
                row["time_s"] = repr(float(row["time_s"]) - 0.001)
        self.assert_failed(self.validate_rows(rows), "holdoff duration")

    def test_rejects_nonsimultaneous_transmissions(self) -> None:
        rows = self.base_rows()
        tx = self.find(rows, "tx_start", node=3)
        tx["time_s"] = repr(float(tx["time_s"]) + 0.01)
        self.retime_outcome(rows, 3)
        self.assert_failed(self.validate_rows(rows), "exceeds simultaneous tolerance")


if __name__ == "__main__":
    unittest.main()
