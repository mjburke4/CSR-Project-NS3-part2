#!/usr/bin/env python3
"""Focused tests for analyze-ns3-admission-ledger.py."""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


UTILS = Path(__file__).resolve().parents[1]
SCRIPT = UTILS / "analyze-ns3-admission-ledger.py"


def load_script():
    specification = importlib.util.spec_from_file_location(
        "analyze_ns3_admission_ledger", SCRIPT
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


analyzer = load_script()

TRACE_COLUMNS = (
    "schema",
    "event_index",
    "time_s",
    "event",
    "src",
    "dst",
    "sequence",
    "size_bytes",
    "reason",
    "node",
    "statistic",
    "value",
    "peer",
    "next_hop",
    "route_cost",
    "detail",
)


def row(index: int, time_s: float, event: str, **values: object) -> dict[str, str]:
    result = {column: "" for column in TRACE_COLUMNS}
    result.update(
        {
            "schema": analyzer.TRACE_SCHEMA,
            "event_index": str(index),
            "time_s": str(time_s),
            "event": event,
        }
    )
    result.update({key: str(value) for key, value in values.items()})
    return result


def packet_row(
    index: int,
    time_s: float,
    event: str,
    *,
    node: int,
    reason: str = "",
    peer: int | None = None,
    next_hop: int | None = None,
    detail: str = "",
) -> dict[str, str]:
    return row(
        index,
        time_s,
        event,
        src=0,
        dst=2,
        sequence=100,
        node=node,
        reason=reason,
        peer="" if peer is None else peer,
        next_hop="" if next_hop is None else next_hop,
        detail=detail,
    )


APP_DETAIL = (
    "flow_index=0;attempt_index=1;configured_destination=2;"
    "discovery_active=0;topology_known=1;gateway_cached=1;"
    "route_check_performed=1;route_available=1;nsdp_count=0;"
    "nsdp_limit=16;nwk_queue=0"
)
NWK_DETAIL = (
    "queue_before=1;queue_after=0;nsdp_count=1;nsdp_limit=16;"
    "pending=0;pending_limit=16;global_spad=17;outstanding=0;"
    "threshold=0;neighbor_spad=1;route_known=1"
)
HOP_DETAIL = (
    "hop_sequence=7;pending_before=0;pending_after=1;pending_limit=16;"
    "global_spad_before=17;global_spad_after=16;"
    "outstanding_before=0;outstanding_after=1;threshold=0;"
    "neighbor_spad_before=1;neighbor_spad_after=0;"
    "resend_queue_before=0;resend_queue_after=1;"
    "resend_tracked=1;resend_overflow=0"
)


def prefix_rows() -> list[dict[str, str]]:
    return [
        packet_row(
            0,
            0.0,
            "app_admission",
            node=0,
            peer=2,
            reason="admitted",
            detail=APP_DETAIL,
        ),
        packet_row(1, 0.0, "app_send", node=0, peer=2),
        packet_row(
            2,
            0.0,
            "nwk_admission",
            node=0,
            peer=1,
            next_hop=1,
            reason="admitted",
            detail=NWK_DETAIL,
        ),
        packet_row(
            3,
            0.0,
            "hop_admission",
            node=0,
            peer=1,
            next_hop=1,
            reason="admitted",
            detail=HOP_DETAIL,
        ),
    ]


def ack_rows() -> list[dict[str, str]]:
    return prefix_rows() + [
        packet_row(
            4,
            0.1,
            "hop_feedback",
            node=1,
            peer=0,
            reason="ack",
            detail=(
                "hop_sequence=7;first_reception=1;ackable=1;"
                "nsdp_count_before=0;nsdp_count_after=1;"
                "nsdp_state_valid=1;nsdp_limit=16"
            ),
        ),
        row(
            5,
            0.2,
            "nwk_nsdp_release",
            src=0,
            dst=2,
            node=0,
            reason="hop_feedback",
            detail="count_before=1;count_after=0;nsdp_limit=16;nwk_queue=0",
        ),
        packet_row(
            6,
            0.2,
            "hop_completion",
            node=0,
            peer=1,
            next_hop=1,
            reason="ack",
            detail=(
                "hop_sequence=7;resend_count=0;pending_before=1;pending_after=0;"
                "pending_limit=16;outstanding_before=1;outstanding_after=0;"
                "threshold_before=0;threshold_after=0;resend_queue_before=1;"
                "resend_queue_after=0;nsdp_released=1;capacity_released=1"
            ),
        ),
    ]


def no_ack_rows() -> list[dict[str, str]]:
    return prefix_rows() + [
        row(
            4,
            5.0,
            "nwk_nsdp_release",
            src=0,
            dst=2,
            node=0,
            reason="hop_feedback",
            detail="count_before=1;count_after=0;nsdp_limit=16;nwk_queue=0",
        ),
        packet_row(
            5,
            5.0,
            "hop_completion",
            node=0,
            peer=1,
            next_hop=1,
            reason="no_ack",
            detail=(
                "hop_sequence=7;resend_count=2;pending_before=1;pending_after=0;"
                "pending_limit=16;outstanding_before=1;outstanding_after=0;"
                "threshold_before=0;threshold_after=0;resend_queue_before=1;"
                "resend_queue_after=0;nsdp_released=1;capacity_released=1"
            ),
        ),
    ]


def dack_rows() -> list[dict[str, str]]:
    return prefix_rows() + [
        packet_row(
            4,
            0.1,
            "hop_feedback",
            node=1,
            peer=0,
            reason="dack",
            detail=(
                "hop_sequence=7;first_reception=1;ackable=1;"
                "nsdp_count_before=16;nsdp_count_after=17;"
                "nsdp_state_valid=1;nsdp_limit=16"
            ),
        ),
        row(
            5,
            0.2,
            "nwk_nsdp_release",
            src=0,
            dst=2,
            node=0,
            reason="hop_feedback",
            detail="count_before=1;count_after=0;nsdp_limit=16;nwk_queue=0",
        ),
        packet_row(
            6,
            0.2,
            "hop_completion",
            node=0,
            peer=1,
            next_hop=1,
            reason="dack",
            detail=(
                "hop_sequence=7;resend_count=0;pending_before=1;pending_after=1;"
                "pending_limit=16;outstanding_before=1;outstanding_after=1;"
                "threshold_before=0;threshold_after=0;resend_queue_before=1;"
                "resend_queue_after=0;dack_hold_seconds=20;"
                f"dack_scheduled_timer_offset_seconds={analyzer.OPNET_TIC_SECONDS};"
                "nsdp_released=1;capacity_released=0"
            ),
        ),
        packet_row(
            7,
            20.2 + analyzer.OPNET_TIC_SECONDS,
            "hop_capacity_release",
            node=0,
            peer=1,
            next_hop=1,
            reason="dack_expiry",
            detail=(
                "hop_sequence=7;resend_count=0;pending_before=1;pending_after=0;"
                "pending_limit=16;outstanding_before=1;outstanding_after=0;"
                "threshold_before=0;threshold_after=0;dack_hold_seconds=20;"
                f"dack_scheduled_timer_offset_seconds={analyzer.OPNET_TIC_SECONDS};"
                f"dack_effective_timer_offset_seconds={analyzer.OPNET_TIC_SECONDS};"
                "nsdp_released=0;capacity_released=1"
            ),
        ),
    ]


def write_trace(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_diagnostics(path: Path, *, admitted: int = 1) -> None:
    columns = (
        "schema",
        "flow_index",
        "attempts",
        "admitted",
        "blocked_discovery",
        "blocked_topology",
        "blocked_gateway_route",
        "blocked_destination",
        "blocked_nsdp",
        "first_admitted_s",
        "last_admitted_s",
    )
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerow(
            {
                "schema": analyzer.APP_DIAGNOSTICS_SCHEMA,
                "flow_index": "0",
                "attempts": str(admitted),
                "admitted": str(admitted),
                "blocked_discovery": "0",
                "blocked_topology": "0",
                "blocked_gateway_route": "0",
                "blocked_destination": "0",
                "blocked_nsdp": "0",
                "first_admitted_s": "0" if admitted else "",
                "last_admitted_s": "0" if admitted else "",
            }
        )


class AnalyzeAdmissionLedgerTests(unittest.TestCase):
    def test_valid_ack_lifecycle_and_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            diagnostics = root / "app.csv"
            write_trace(trace, ack_rows())
            write_diagnostics(diagnostics)

            legs, report = analyzer.analyze(trace, diagnostics)

            self.assertTrue(report["pass"])
            self.assertTrue(report["application_diagnostics"]["pass"])
            self.assertEqual(report["hop"]["completions_by_reason"], {"ack": 1})
            self.assertEqual(analyzer._leg_status(legs[0]), "complete")
            self.assertEqual(legs[0].nsdp_release_time_s, 0.2)
        with self.subTest(lifecycle="overlapping ACK legs"):
            self._check_overlapping_ack_legs_use_full_hop_identity()

    def test_valid_dack_lifecycle_releases_capacity_at_expiry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(trace, dack_rows())

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(report["hop"]["completions_by_reason"], {"dack": 1})
            self.assertEqual(report["hop"]["capacity_releases"], 1)
            self.assertEqual(legs[0].dack_hold_seconds, 20.0)
            self.assertAlmostEqual(
                legs[0].capacity_release_time_s,
                20.2 + analyzer.OPNET_TIC_SECONDS,
            )
        with self.subTest(lifecycle="overlapping DACK holds"):
            self._check_overlapping_dack_holds_release_by_full_hop_identity()

    def _check_overlapping_ack_legs_use_full_hop_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = prefix_rows() + [
                packet_row(
                    4,
                    0.05,
                    "nwk_admission",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="admitted",
                    detail=(
                        "queue_before=1;queue_after=0;nsdp_count=2;nsdp_limit=16;"
                        "pending=1;pending_limit=16;global_spad=16;outstanding=1;"
                        "threshold=1;neighbor_spad=1;route_known=1"
                    ),
                ),
                packet_row(
                    5,
                    0.05,
                    "hop_admission",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="admitted",
                    detail=(
                        "hop_sequence=8;pending_before=1;pending_after=2;"
                        "pending_limit=16;global_spad_before=16;"
                        "global_spad_after=15;outstanding_before=1;"
                        "outstanding_after=2;threshold=1;"
                        "neighbor_spad_before=1;neighbor_spad_after=0;"
                        "resend_queue_before=1;resend_queue_after=2;"
                        "resend_tracked=1;resend_overflow=0"
                    ),
                ),
                packet_row(
                    6,
                    0.1,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="ack",
                    detail=(
                        "hop_sequence=7;first_reception=1;ackable=1;"
                        "nsdp_count_before=0;nsdp_count_after=1;"
                        "nsdp_state_valid=1;nsdp_limit=16"
                    ),
                ),
                packet_row(
                    7,
                    0.11,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="ack",
                    detail=(
                        "hop_sequence=8;first_reception=1;ackable=1;"
                        "nsdp_count_before=1;nsdp_count_after=2;"
                        "nsdp_state_valid=1;nsdp_limit=16"
                    ),
                ),
                row(
                    8,
                    0.2,
                    "nwk_nsdp_release",
                    src=0,
                    dst=2,
                    node=0,
                    reason="hop_feedback",
                    detail=(
                        "count_before=2;count_after=1;nsdp_limit=16;nwk_queue=0"
                    ),
                ),
                packet_row(
                    9,
                    0.2,
                    "hop_completion",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="ack",
                    detail=(
                        "hop_sequence=7;resend_count=0;pending_before=2;"
                        "pending_after=1;pending_limit=16;outstanding_before=2;"
                        "outstanding_after=1;threshold_before=1;threshold_after=1;"
                        "resend_queue_before=2;resend_queue_after=1;"
                        "nsdp_released=1;capacity_released=1"
                    ),
                ),
                row(
                    10,
                    0.3,
                    "nwk_nsdp_release",
                    src=0,
                    dst=2,
                    node=0,
                    reason="hop_feedback",
                    detail=(
                        "count_before=1;count_after=0;nsdp_limit=16;nwk_queue=0"
                    ),
                ),
                packet_row(
                    11,
                    0.3,
                    "hop_completion",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="ack",
                    detail=(
                        "hop_sequence=8;resend_count=0;pending_before=1;"
                        "pending_after=0;pending_limit=16;outstanding_before=1;"
                        "outstanding_after=0;threshold_before=1;threshold_after=1;"
                        "resend_queue_before=1;resend_queue_after=0;"
                        "nsdp_released=1;capacity_released=1"
                    ),
                ),
            ]
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(report["hop"]["completions_by_reason"], {"ack": 2})
            self.assertEqual([leg.hop_sequence for leg in legs], [7, 8])
            self.assertEqual([leg.completion_time_s for leg in legs], [0.2, 0.3])
            self.assertTrue(all(not leg.issues for leg in legs))

    def _check_overlapping_dack_holds_release_by_full_hop_identity(self) -> None:
        tic = analyzer.OPNET_TIC_SECONDS
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = prefix_rows() + [
                packet_row(
                    4,
                    0.05,
                    "nwk_admission",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="admitted",
                    detail=(
                        "queue_before=1;queue_after=0;nsdp_count=2;nsdp_limit=16;"
                        "pending=1;pending_limit=16;global_spad=16;outstanding=1;"
                        "threshold=1;neighbor_spad=1;route_known=1"
                    ),
                ),
                packet_row(
                    5,
                    0.05,
                    "hop_admission",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="admitted",
                    detail=(
                        "hop_sequence=8;pending_before=1;pending_after=2;"
                        "pending_limit=16;global_spad_before=16;"
                        "global_spad_after=15;outstanding_before=1;"
                        "outstanding_after=2;threshold=1;"
                        "neighbor_spad_before=1;neighbor_spad_after=0;"
                        "resend_queue_before=1;resend_queue_after=2;"
                        "resend_tracked=1;resend_overflow=0"
                    ),
                ),
                packet_row(
                    6,
                    0.1,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="dack",
                    detail=(
                        "hop_sequence=7;first_reception=1;ackable=1;"
                        "nsdp_count_before=16;nsdp_count_after=17;"
                        "nsdp_state_valid=1;nsdp_limit=16"
                    ),
                ),
                packet_row(
                    7,
                    0.11,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="dack",
                    detail=(
                        "hop_sequence=8;first_reception=1;ackable=1;"
                        "nsdp_count_before=16;nsdp_count_after=17;"
                        "nsdp_state_valid=1;nsdp_limit=16"
                    ),
                ),
                row(
                    8,
                    0.2,
                    "nwk_nsdp_release",
                    src=0,
                    dst=2,
                    node=0,
                    reason="hop_feedback",
                    detail=(
                        "count_before=2;count_after=1;nsdp_limit=16;nwk_queue=0"
                    ),
                ),
                packet_row(
                    9,
                    0.2,
                    "hop_completion",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="dack",
                    detail=(
                        "hop_sequence=7;resend_count=2;pending_before=2;"
                        "pending_after=2;pending_limit=16;outstanding_before=2;"
                        "outstanding_after=2;threshold_before=1;threshold_after=1;"
                        "resend_queue_before=2;resend_queue_after=1;"
                        "dack_hold_seconds=40;"
                        f"dack_scheduled_timer_offset_seconds={tic};"
                        "nsdp_released=1;capacity_released=0"
                    ),
                ),
                row(
                    10,
                    0.3,
                    "nwk_nsdp_release",
                    src=0,
                    dst=2,
                    node=0,
                    reason="hop_feedback",
                    detail=(
                        "count_before=1;count_after=0;nsdp_limit=16;nwk_queue=0"
                    ),
                ),
                packet_row(
                    11,
                    0.3,
                    "hop_completion",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="dack",
                    detail=(
                        "hop_sequence=8;resend_count=0;pending_before=2;"
                        "pending_after=2;pending_limit=16;outstanding_before=2;"
                        "outstanding_after=2;threshold_before=1;threshold_after=1;"
                        "resend_queue_before=1;resend_queue_after=0;"
                        "dack_hold_seconds=20;"
                        f"dack_scheduled_timer_offset_seconds={tic};"
                        "nsdp_released=1;capacity_released=0"
                    ),
                ),
                packet_row(
                    12,
                    20.3 + tic,
                    "hop_capacity_release",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="dack_expiry",
                    detail=(
                        "hop_sequence=8;resend_count=0;pending_before=2;"
                        "pending_after=1;pending_limit=16;outstanding_before=2;"
                        "outstanding_after=1;threshold_before=1;threshold_after=1;"
                        "dack_hold_seconds=20;"
                        f"dack_scheduled_timer_offset_seconds={tic};"
                        f"dack_effective_timer_offset_seconds={tic};"
                        "nsdp_released=0;capacity_released=1"
                    ),
                ),
                packet_row(
                    13,
                    40.2 + tic,
                    "hop_capacity_release",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="dack_expiry",
                    detail=(
                        "hop_sequence=7;resend_count=2;pending_before=1;"
                        "pending_after=0;pending_limit=16;outstanding_before=1;"
                        "outstanding_after=0;threshold_before=1;threshold_after=1;"
                        "dack_hold_seconds=40;"
                        f"dack_scheduled_timer_offset_seconds={tic};"
                        f"dack_effective_timer_offset_seconds={tic};"
                        "nsdp_released=0;capacity_released=1"
                    ),
                ),
            ]
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(report["hop"]["completions_by_reason"], {"dack": 2})
            self.assertEqual(report["hop"]["capacity_releases"], 2)
            self.assertEqual([leg.hop_sequence for leg in legs], [7, 8])
            self.assertAlmostEqual(legs[0].capacity_release_time_s, 40.2 + tic)
            self.assertAlmostEqual(legs[1].capacity_release_time_s, 20.3 + tic)
            self.assertTrue(all(not leg.issues for leg in legs))

    def test_dack_expiry_at_nominal_hold_is_a_strict_finding(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = dack_rows()
            rows[7]["time_s"] = "20.2"
            rows[7]["detail"] = rows[7]["detail"].replace(
                f"dack_effective_timer_offset_seconds={analyzer.OPNET_TIC_SECONDS}",
                "dack_effective_timer_offset_seconds=0",
            )
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertFalse(report["pass"])
            self.assertIn(
                "dack_expiry_time_mismatch",
                {issue["code"] for issue in legs[0].issues},
            )

    def test_legacy_dack_trace_without_offsets_remains_analyzable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = dack_rows()
            rows[6]["detail"] = ";".join(
                token
                for token in rows[6]["detail"].split(";")
                if not token.startswith("dack_scheduled_timer_offset_seconds=")
            )
            rows[7]["time_s"] = "20.2"
            rows[7]["detail"] = ";".join(
                token
                for token in rows[7]["detail"].split(";")
                if not token.startswith(
                    (
                        "dack_scheduled_timer_offset_seconds=",
                        "dack_effective_timer_offset_seconds=",
                    )
                )
            )
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertFalse(report["pass"])
            self.assertEqual(
                legs[0].dack_scheduled_timer_offset_seconds,
                analyzer.OPNET_TIC_SECONDS,
            )
            self.assertAlmostEqual(legs[0].dack_effective_timer_offset_seconds, 0.0)
            self.assertIn(
                "dack_expiry_time_mismatch",
                {issue["code"] for issue in legs[0].issues},
            )

    def test_max_resend_dack_uses_40_second_hold_plus_tic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = dack_rows()
            rows[6]["detail"] = rows[6]["detail"].replace(
                "resend_count=0", "resend_count=2"
            ).replace("dack_hold_seconds=20", "dack_hold_seconds=40")
            rows[7]["detail"] = rows[7]["detail"].replace(
                "resend_count=0", "resend_count=2"
            ).replace("dack_hold_seconds=20", "dack_hold_seconds=40")
            rows[7]["time_s"] = str(40.2 + analyzer.OPNET_TIC_SECONDS)
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(legs[0].resend_count, 2)
            self.assertEqual(legs[0].dack_hold_seconds, 40.0)
            self.assertAlmostEqual(
                legs[0].capacity_release_time_s,
                40.2 + analyzer.OPNET_TIC_SECONDS,
            )

    def test_dack_trigger_validation_accepts_coalesced_release(self) -> None:
        tic = analyzer.OPNET_TIC_SECONDS
        release_time = 20.2 + tic

        def release_event(index: int, sequence: int) -> object:
            return analyzer.Event(
                index=index,
                time_s=release_time,
                name="hop_capacity_release",
                node=0,
                peer=1,
                next_hop=1,
                source=0,
                destination=2,
                packet=analyzer.PacketKey(0, 2, sequence),
                reason="dack_expiry",
                detail={},
                input_row=index + 2,
            )

        first = analyzer.Leg(
            packet=analyzer.PacketKey(0, 2, 100),
            leg_index=0,
            node=0,
            next_hop=1,
            first_event_index=0,
            completion_time_s=0.2,
            completion_reason="dack",
            capacity_release_time_s=release_time,
            dack_hold_seconds=20.0,
            dack_scheduled_timer_offset_seconds=tic,
            dack_effective_timer_offset_seconds=tic,
            capacity_release_event=release_event(7, 100),
        )
        second = analyzer.Leg(
            packet=analyzer.PacketKey(0, 2, 101),
            leg_index=0,
            node=0,
            next_hop=1,
            first_event_index=8,
            completion_time_s=0.2 + tic / 2.0,
            completion_reason="dack",
            capacity_release_time_s=release_time,
            dack_hold_seconds=20.0,
            dack_scheduled_timer_offset_seconds=tic,
            dack_effective_timer_offset_seconds=tic / 2.0,
            capacity_release_event=release_event(15, 101),
        )
        state = analyzer.AnalysisState(legs=[first, second])

        analyzer._validate_dack_timer_triggers(state)

        self.assertEqual(first.issues, [])
        self.assertEqual(second.issues, [])

    def test_dack_trigger_validation_rejects_other_nodes_timer(self) -> None:
        tic = analyzer.OPNET_TIC_SECONDS
        release_time = 20.2 + tic

        def release_event(index: int, node: int, sequence: int) -> object:
            return analyzer.Event(
                index=index,
                time_s=release_time,
                name="hop_capacity_release",
                node=node,
                peer=1,
                next_hop=1,
                source=0,
                destination=2,
                packet=analyzer.PacketKey(0, 2, sequence),
                reason="dack_expiry",
                detail={},
                input_row=index + 2,
            )

        valid = analyzer.Leg(
            packet=analyzer.PacketKey(0, 2, 100),
            leg_index=0,
            node=0,
            next_hop=1,
            first_event_index=0,
            completion_time_s=0.2,
            completion_reason="dack",
            capacity_release_time_s=release_time,
            dack_hold_seconds=20.0,
            dack_scheduled_timer_offset_seconds=tic,
            dack_effective_timer_offset_seconds=tic,
            capacity_release_event=release_event(7, 0, 100),
        )
        invalid = analyzer.Leg(
            packet=analyzer.PacketKey(0, 2, 101),
            leg_index=0,
            node=2,
            next_hop=1,
            first_event_index=8,
            completion_time_s=0.2 + tic / 2.0,
            completion_reason="dack",
            capacity_release_time_s=release_time,
            dack_hold_seconds=20.0,
            dack_scheduled_timer_offset_seconds=tic,
            dack_effective_timer_offset_seconds=tic / 2.0,
            capacity_release_event=release_event(15, 2, 101),
        )
        state = analyzer.AnalysisState(legs=[valid, invalid])

        analyzer._validate_dack_timer_triggers(state)

        self.assertEqual(valid.issues, [])
        self.assertIn(
            "dack_expiry_time_mismatch",
            {issue["code"] for issue in invalid.issues},
        )

    def test_tagless_duplicate_feedback_uses_hop_identity_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = ack_rows()
            duplicate = row(
                5,
                0.15,
                "hop_feedback",
                node=1,
                peer=0,
                reason="ack",
                detail=(
                    "hop_sequence=7;first_reception=0;ackable=1;"
                    "nsdp_count_before=0;nsdp_count_after=0;"
                    "nsdp_state_valid=0;nsdp_limit=16"
                ),
            )
            rows.insert(5, duplicate)
            rows[6]["event_index"] = "6"
            rows[7]["event_index"] = "7"
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(report["hop"]["uncorrelated_feedback"], 0)
            self.assertEqual(report["hop"]["feedback_by_reason"], {"ack": 2})
            self.assertEqual(legs[0].completion_reason, "ack")

    def test_uncorrelated_tagless_duplicate_is_nonfatal_metric(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = ack_rows()
            rows.append(
                row(
                    7,
                    0.3,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="ack",
                    detail=(
                        "hop_sequence=7;first_reception=0;ackable=1;"
                        "nsdp_count_before=0;nsdp_count_after=0;"
                        "nsdp_state_valid=0;nsdp_limit=16"
                    ),
                )
            )
            write_trace(trace, rows)

            _, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(report["hop"]["uncorrelated_feedback"], 1)
            self.assertEqual(report["integrity"]["global_issue_count"], 0)

    def test_late_packet_identified_feedback_is_nonfatal_metric(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            rows = no_ack_rows()
            rows.append(
                packet_row(
                    6,
                    10.2,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="ack",
                    detail=(
                        "hop_sequence=7;first_reception=1;ackable=1;"
                        "nsdp_count_before=0;nsdp_count_after=0;"
                        "nsdp_state_valid=1;nsdp_limit=16"
                    ),
                )
            )
            write_trace(trace, rows)

            _, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(report["hop"]["uncorrelated_feedback"], 1)
            self.assertEqual(report["hop"]["uncorrelated_tagged_feedback"], 1)
            self.assertEqual(report["hop"]["uncorrelated_tagless_feedback"], 0)
            self.assertEqual(report["hop"]["late_feedback_after_no_ack"], 1)
            self.assertEqual(report["integrity"]["global_issue_count"], 0)

            dack_trace = root / "late-after-dack.csv"
            dack = dack_rows()
            dack.insert(
                7,
                packet_row(
                    7,
                    1.0,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="ack",
                    detail=(
                        "hop_sequence=7;first_reception=0;ackable=1;"
                        "nsdp_count_before=0;nsdp_count_after=0;"
                        "nsdp_state_valid=0;nsdp_limit=16"
                    ),
                ),
            )
            dack[8]["event_index"] = "8"
            write_trace(dack_trace, dack)

            dack_legs, dack_report = analyzer.analyze(dack_trace)

            self.assertTrue(dack_report["pass"])
            self.assertEqual(dack_report["hop"]["uncorrelated_feedback"], 1)
            self.assertEqual(dack_legs[0].feedback_reason, "dack")
            self.assertEqual(dack_legs[0].feedback_time_s, 0.1)
            self.assertEqual(dack_legs[0].completion_reason, "dack")
            self.assertFalse(dack_legs[0].feedback_completion_mismatch)

            reuse_trace = root / "late-after-reused-identity.csv"
            reuse = no_ack_rows()
            templates = dack_rows()
            times = (6.0, 6.0, 6.1, 6.2, 6.2)
            for index, (template, time_s) in enumerate(
                zip(templates[2:7], times), start=6
            ):
                reused_row = dict(template)
                reused_row.update(event_index=str(index), time_s=str(time_s))
                reuse.append(reused_row)
            reuse.append(
                packet_row(
                    11,
                    7.0,
                    "hop_feedback",
                    node=1,
                    peer=0,
                    reason="ack",
                    detail=(
                        "hop_sequence=7;first_reception=1;ackable=1;"
                        "nsdp_count_before=0;nsdp_count_after=1;"
                        "nsdp_state_valid=1;nsdp_limit=16"
                    ),
                )
            )
            capacity = dict(templates[7])
            capacity.update(
                event_index="12",
                time_s=str(26.2 + analyzer.OPNET_TIC_SECONDS),
            )
            reuse.append(capacity)
            write_trace(reuse_trace, reuse)

            reuse_legs, reuse_report = analyzer.analyze(reuse_trace)

            self.assertFalse(reuse_report["pass"])
            self.assertEqual(reuse_report["hop"]["late_feedback_after_no_ack"], 0)
            self.assertIn(
                "uncorrelated_first_reception_feedback",
                {
                    issue["code"]
                    for issue in reuse_report["integrity"]["global_issues"]
                },
            )
            self.assertEqual(reuse_legs[1].feedback_reason, "dack")
            self.assertEqual(reuse_legs[1].completion_reason, "dack")

    def test_unknown_first_reception_feedback_is_strict_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = ack_rows()
            rows[4]["detail"] = rows[4]["detail"].replace(
                "hop_sequence=7", "hop_sequence=999"
            )
            write_trace(trace, rows)

            _, report = analyzer.analyze(trace)

            self.assertFalse(report["pass"])
            self.assertEqual(report["hop"]["uncorrelated_feedback"], 1)
            self.assertIn(
                "uncorrelated_first_reception_feedback",
                {
                    issue["code"]
                    for issue in report["integrity"]["global_issues"]
                },
            )

    def test_directed_hop_ledger_events_require_peer_and_next_hop(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cases = (
                ("admission", ack_rows(), 3),
                ("completion", ack_rows(), 6),
                ("capacity", dack_rows(), 7),
            )
            for name, rows, row_index in cases:
                with self.subTest(event=name):
                    trace = root / f"{name}.csv"
                    rows[row_index]["peer"] = ""
                    rows[row_index]["next_hop"] = ""
                    write_trace(trace, rows)
                    with self.assertRaisesRegex(
                        analyzer.AdmissionAnalysisError,
                        "requires directed-leg peer and next_hop",
                    ):
                        analyzer.analyze(trace)

            with self.subTest(event="duplicate active full identity"):
                trace = root / "duplicate-active-identity.csv"
                rows = prefix_rows()
                second_nwk = packet_row(
                    4,
                    0.05,
                    "nwk_admission",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="admitted",
                    detail=(
                        NWK_DETAIL.replace("pending=0;", "pending=1;")
                        .replace("global_spad=17", "global_spad=16")
                        .replace("outstanding=0;", "outstanding=1;")
                        .replace("threshold=0;", "threshold=1;")
                    ),
                )
                second_hop = packet_row(
                    5,
                    0.05,
                    "hop_admission",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="admitted",
                    detail=(
                        HOP_DETAIL.replace(
                            "pending_before=0;pending_after=1",
                            "pending_before=1;pending_after=2",
                        )
                        .replace(
                            "global_spad_before=17;global_spad_after=16",
                            "global_spad_before=16;global_spad_after=15",
                        )
                        .replace(
                            "outstanding_before=0;outstanding_after=1",
                            "outstanding_before=1;outstanding_after=2",
                        )
                        .replace("threshold=0;", "threshold=1;")
                        .replace(
                            "resend_queue_before=0;resend_queue_after=1",
                            "resend_queue_before=1;resend_queue_after=2",
                        )
                    ),
                )
                rows.extend((second_nwk, second_hop))
                write_trace(trace, rows)
                with self.assertRaisesRegex(
                    analyzer.AdmissionAnalysisError,
                    "active or held HOP identity is ambiguous",
                ):
                    analyzer.analyze(trace)

            with self.subTest(event="duplicate DACK-held full identity"):
                trace = root / "duplicate-held-identity.csv"
                rows = dack_rows()[:-1]
                held_nwk = dict(second_nwk)
                held_nwk.update(event_index="7", time_s="0.3")
                held_hop = dict(second_hop)
                held_hop.update(event_index="8", time_s="0.3")
                rows.extend((held_nwk, held_hop))
                write_trace(trace, rows)
                with self.assertRaisesRegex(
                    analyzer.AdmissionAnalysisError,
                    "active or held HOP identity is ambiguous",
                ):
                    analyzer.analyze(trace)

    def test_missing_window_feedback_is_nonfatal_metric(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = ack_rows()
            del rows[4]
            rows[4]["event_index"] = "4"
            rows[5]["event_index"] = "5"
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(report["hop"]["missing_per_leg_feedback"], 1)
            self.assertTrue(legs[0].feedback_missing)
            self.assertFalse(legs[0].issues)

    def test_window_feedback_reason_mismatch_is_nonfatal_metric(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = dack_rows()
            rows[4]["reason"] = "ack"
            rows[4]["detail"] = rows[4]["detail"].replace(
                "nsdp_count_before=16;nsdp_count_after=17",
                "nsdp_count_before=0;nsdp_count_after=1",
            )
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(
                report["hop"]["feedback_completion_reason_mismatches"], 1
            )
            self.assertTrue(legs[0].feedback_completion_mismatch)
            self.assertFalse(legs[0].issues)

    def test_dack_pre_limit_boundary_is_a_strict_finding(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = dack_rows()
            rows[4]["detail"] = rows[4]["detail"].replace(
                "nsdp_count_before=16;nsdp_count_after=17",
                "nsdp_count_before=15;nsdp_count_after=16",
            )
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertFalse(report["pass"])
            self.assertEqual(report["hop"]["dack_pre_limit_findings"], 1)
            self.assertEqual(report["hop"]["dack_boundary_count"], 1)
            self.assertIn(
                "bad_dack_selection",
                {issue["code"] for issue in legs[0].issues},
            )

    def test_nwk_holds_are_grouped_by_node_and_next_hop(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = prefix_rows()[:2]
            rows.append(
                packet_row(
                    2,
                    0.0,
                    "nwk_admission",
                    node=0,
                    peer=1,
                    next_hop=1,
                    reason="neighbor_flow_full",
                    detail=(
                        "queue_before=1;queue_after=1;nsdp_count=1;nsdp_limit=16;"
                        "pending=0;pending_limit=16;global_spad=17;outstanding=1;"
                        "threshold=0;neighbor_spad=0;route_known=1"
                    ),
                )
            )
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(analyzer._leg_status(legs[0]), "open_nwk")
            self.assertEqual(
                report["nwk"]["holds_by_node_next_hop"],
                [
                    {
                        "node": 0,
                        "next_hop": 1,
                        "attempts": 1,
                        "by_reason": {"neighbor_capacity": 1},
                    }
                ],
            )

    def test_nwk_route_and_scan_markers_are_strict(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)

            admitted_trace = root / "admitted.csv"
            admitted = ack_rows()
            admitted[2]["detail"] = admitted[2]["detail"].replace(
                "route_known=1", "route_known=0"
            )
            write_trace(admitted_trace, admitted)
            _, admitted_report = analyzer.analyze(admitted_trace)
            self.assertIn(
                "bad_route_known",
                {
                    issue["code"]
                    for issue in admitted_report["integrity"]["global_issues"]
                },
            )

            no_route_trace = root / "no-route.csv"
            no_route_rows = prefix_rows()[:2]
            no_route_rows.append(
                packet_row(
                    2,
                    0.0,
                    "nwk_admission",
                    node=0,
                    reason="no_route",
                    detail=(
                        "queue_before=1;queue_after=1;nsdp_count=1;nsdp_limit=16;"
                        "pending=0;pending_limit=16;global_spad=17;route_known=0"
                    ),
                )
            )
            write_trace(no_route_trace, no_route_rows)
            _, no_route_report = analyzer.analyze(no_route_trace)
            self.assertTrue(no_route_report["pass"])

            global_trace = root / "global.csv"
            global_rows = prefix_rows()[:2]
            global_rows.append(
                packet_row(
                    2,
                    0.0,
                    "nwk_admission",
                    node=0,
                    reason="global_hop_full",
                    detail=(
                        "queue_before=1;queue_after=1;nsdp_count=1;nsdp_limit=16;"
                        "pending=17;pending_limit=16;global_spad=0;scan_stopped=0"
                    ),
                )
            )
            write_trace(global_trace, global_rows)
            _, global_report = analyzer.analyze(global_trace)
            self.assertIn(
                "bad_scan_stop_marker",
                {
                    issue["code"]
                    for issue in global_report["integrity"]["global_issues"]
                },
            )

    def test_reports_signed_spad_formula_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = ack_rows()
            rows[2]["detail"] = NWK_DETAIL.replace("global_spad=17", "global_spad=16")
            write_trace(trace, rows)

            _, report = analyzer.analyze(trace)

            self.assertFalse(report["pass"])
            self.assertIn(
                "bad_global_spad",
                {issue["code"] for issue in report["integrity"]["global_issues"]},
            )

    def test_rejects_duplicate_detail_key(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = ack_rows()
            rows[0]["detail"] += ";flow_index=0"
            write_trace(trace, rows)

            with self.assertRaisesRegex(analyzer.AdmissionAnalysisError, "duplicate detail"):
                analyzer.analyze(trace)

    def test_flow_level_release_must_immediately_precede_completion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            rows = ack_rows()
            rows.insert(6, packet_row(6, 0.2, "nwk_delivery", node=2, peer=1))
            rows[7]["event_index"] = "7"
            write_trace(trace, rows)

            legs, report = analyzer.analyze(trace)

            self.assertFalse(report["pass"])
            self.assertIn(
                "nsdp_release_order_mismatch",
                {issue["code"] for issue in legs[0].issues},
            )

    def test_strict_cli_writes_failure_evidence_and_exits_one(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)

            rows = ack_rows()
            rows[3]["detail"] = HOP_DETAIL.replace(
                "global_spad_after=16", "global_spad_after=15"
            )
            cases = [("hop-state", rows, "bad_hop_global_spad")]

            rows = ack_rows()
            rows[3]["time_s"] = "0.01"
            cases.append(
                ("hop-time", rows, "hop_admission_time_mismatch")
            )

            rows = ack_rows()
            rows[3]["next_hop"] = "2"
            cases.append(("hop-next-hop", rows, "hop_next_hop_mismatch"))

            rows = ack_rows()
            rows[6]["next_hop"] = "2"
            cases.append(
                ("completion-next-hop", rows, "completion_next_hop_mismatch")
            )

            rows = ack_rows()
            rows[6]["detail"] = rows[6]["detail"].replace(
                "hop_sequence=7", "hop_sequence=8"
            )
            cases.append(
                ("completion-sequence", rows, "completion_sequence_mismatch")
            )

            rows = dack_rows()
            rows[7]["next_hop"] = "2"
            cases.append(
                ("capacity-next-hop", rows, "capacity_next_hop_mismatch")
            )

            rows = dack_rows()
            rows[7]["detail"] = rows[7]["detail"].replace(
                "hop_sequence=7", "hop_sequence=8"
            )
            cases.append(
                ("capacity-sequence", rows, "capacity_sequence_mismatch")
            )

            rows = dack_rows()
            duplicate_release = dict(rows[5])
            duplicate_release.update(event_index="7", time_s="0.3")
            duplicate_completion = dict(rows[6])
            duplicate_completion.update(event_index="8", time_s="0.3")
            rows[7]["event_index"] = "9"
            rows[7:7] = [duplicate_release, duplicate_completion]
            cases.append(
                ("duplicate-dack-completion", rows, "duplicate_hop_completion")
            )

            rows = prefix_rows()
            early_capacity = dict(dack_rows()[7])
            early_capacity.update(event_index="4", time_s="0.1")
            rows.append(early_capacity)
            cases.append(
                ("capacity-before-completion", rows, "capacity_release_without_dack")
            )

            for name, case_rows, expected_issue in cases:
                with self.subTest(case=name):
                    trace = root / f"{name}-trace.csv"
                    summary = root / f"{name}-summary.json"
                    legs_csv = root / f"{name}-legs.csv"
                    write_trace(trace, case_rows)

                    result = subprocess.run(
                        [
                            sys.executable,
                            str(SCRIPT),
                            str(trace),
                            "--summary-json",
                            str(summary),
                            "--legs-csv",
                            str(legs_csv),
                            "--strict",
                        ],
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        check=False,
                    )

                    self.assertEqual(result.returncode, 1)
                    self.assertTrue(summary.is_file())
                    self.assertTrue(legs_csv.is_file())
                    report = json.loads(summary.read_text(encoding="utf-8"))
                    self.assertFalse(report["pass"])
                    with legs_csv.open(
                        "r", encoding="utf-8", newline=""
                    ) as stream:
                        leg_rows = list(csv.DictReader(stream))
                    issue_codes = {
                        issue["code"]
                        for leg_row in leg_rows
                        for issue in json.loads(leg_row["issues_json"])
                    }
                    self.assertIn(expected_issue, issue_codes)


if __name__ == "__main__":
    unittest.main()
