#!/usr/bin/env python3
"""Synthetic fail-closed probes for the release DACK-lineage classifier."""

from __future__ import annotations

from argparse import Namespace
import csv
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


HARNESS = Path(__file__).with_name("certify_release.py").resolve()
SPEC = importlib.util.spec_from_file_location("certify_release_probe", HARNESS)
assert SPEC is not None and SPEC.loader is not None
CERTIFY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CERTIFY
SPEC.loader.exec_module(CERTIFY)


TRACE_COLUMNS = (
    "schema",
    "event_index",
    "time_s",
    "event",
    "src",
    "dst",
    "sequence",
    "size_bytes",
    "success",
    "reason",
    "node",
    "statistic",
    "value",
    "peer",
    "next_hop",
    "route_cost",
    "detail",
)


def application_aggregate_report() -> tuple[dict[str, object], dict[str, object]]:
    configuration: dict[str, object] = {
        "statistics": list(CERTIFY.CANONICAL_APPLICATION_AGGREGATE_STATISTICS)
    }
    series: list[dict[str, object]] = [
        {"statistic": statistic}
        for statistic in CERTIFY.CANONICAL_APPLICATION_AGGREGATE_STATISTICS
    ]
    packet_size = next(
        record
        for record in series
        if record["statistic"] == CERTIFY.APPLICATION_PACKET_SIZE_STATISTIC
    )
    packet_size.update(
        {
            "scenario": "blue_radio_campus-multihop",
            "status": "compared",
            "unit": "bits",
            "aggregation": "bucket_sample_mean",
            "opnet_points": 100,
            "ns3_points": 100,
            "matched_points": 100,
            "authoritative_numeric_points": 95,
            "numeric_points_compared": 95,
            "numeric_mismatches": 0,
            "maximum_absolute_delta": 0.0,
            "maximum_relative_delta": 0.0,
            "maximum_time_delta_s": 0.0,
            "missing_in_ns3": 0,
            "extra_in_ns3": 0,
            "missing_ns3_values": 0,
            "skipped_missing_values": 5,
            "pass": True,
        }
    )
    return configuration, {"series": series}


def protocol_aggregate_report() -> dict[str, object]:
    mismatch_counts = {
        "HOP.Resend Queue Size (packets)": 95,
        "MAC.ACK Queue Size (packets)": 93,
        "MAC.Tx Queue Size (packets)": 95,
        "MAC.Tx Queuing Delay (sec)": 95,
    }
    series = []
    for statistic in CERTIFY.PROTOCOL_AGGREGATE_STATISTICS:
        unit, aggregation = CERTIFY.PROTOCOL_AGGREGATE_SEMANTICS[statistic]
        mismatch_count = mismatch_counts[statistic]
        series.append(
            {
                "scenario": "blue_radio_campus-multihop",
                "statistic": statistic,
                "status": "compared",
                "unit": unit,
                "aggregation": aggregation,
                "opnet_points": 95,
                "ns3_points": 95,
                "matched_points": 95,
                "authoritative_numeric_points": 95,
                "numeric_points_compared": 95,
                "numeric_mismatches": mismatch_count,
                "missing_in_ns3": 0,
                "extra_in_ns3": 0,
                "missing_ns3_values": 0,
                "skipped_missing_values": 0,
                "pass": mismatch_count == 0,
            }
        )
    return {
        "schema": "csr-aggregate-differential-report-v1",
        "pass": False,
        "filters": {
            "scenarios": ["blue_radio_campus-multihop"],
            "statistics": list(CERTIFY.PROTOCOL_AGGREGATE_STATISTICS),
            "time_start_s": 360.0,
            "time_stop_s": 6000.0,
            "selected_point_count": 760,
        },
        "tolerances": {"absolute": 0.0, "relative": 0.0, "time_s": 0.0},
        "opnet_point_count": 380,
        "ns3_point_count": 380,
        "total_difference_count": sum(mismatch_counts.values()),
        "reported_difference_count": sum(mismatch_counts.values()),
        "differences_truncated": False,
        "total_skipped_count": 0,
        "reported_skipped_count": 0,
        "skipped_truncated": False,
        "counts": {
            "base_series": 4,
            "comparable_series": 4,
            "matched_points": 380,
            "numeric_points_compared": 380,
            "numeric_mismatches": sum(mismatch_counts.values()),
            "missing_in_ns3": 0,
            "extra_in_ns3": 0,
            "skipped_missing_values": 0,
            "skipped_noncomparable_points": 0,
            "unmatched_opnet_series": 0,
            "unmatched_ns3_series": 0,
            "noncomparable_series": 0,
            "missing_ns3_values": 0,
            "indeterminate_series": 0,
        },
        "series": series,
    }


class ClassifierProbe(unittest.TestCase):
    def build_case(
        self,
        terminal_reason: str = "neighbor_flow_full",
        *,
        terminal_attempt_feedback: bool = False,
        terminal_attempt_feedback_first_reception: str = "1",
        terminal_completion_reason: str | None = "no_ack",
        duplicate_terminal_admission: bool = False,
        duplicate_terminal_completion: bool = False,
        terminal_completion_before_admission: bool = False,
        reverse_terminal_completion: bool = False,
    ) -> tuple[object, ...]:
        temporary = tempfile.TemporaryDirectory(prefix="csr-classifier-probe-")
        root = Path(temporary.name).resolve()
        trace = root / "trace.csv"

        def trace_row(
            event_index: int,
            time_s: float,
            event: str,
            key: tuple[str, str, str],
            *,
            reason: str = "",
            node: str = "",
            peer: str = "",
            next_hop: str = "",
            success: str = "",
            detail: str = "",
        ) -> dict[str, str]:
            row = {name: "" for name in TRACE_COLUMNS}
            row.update(
                {
                    "schema": "csr-differential-trace-v1",
                    "event_index": str(event_index),
                    "time_s": repr(time_s),
                    "event": event,
                    "src": key[0],
                    "dst": key[1],
                    "sequence": key[2],
                    "reason": reason,
                    "node": node,
                    "peer": peer,
                    "next_hop": next_hop,
                    "success": success,
                    "detail": detail,
                }
            )
            return row

        delivered = ("1", "9", "10")
        terminal = ("2", "9", "20")

        def feedback_detail(
            hop_sequence: int, before: int, after: int
        ) -> str:
            return (
                f"hop_sequence={hop_sequence};first_reception=1;ackable=1;"
                "nsdp_state_valid=1;"
                f"nsdp_count_before={before};nsdp_count_after={after};"
                "nsdp_limit=16"
            )

        rows = [
            trace_row(1, 1.0, "app_send", delivered, node="1"),
            trace_row(2, 2.0, "hop_admission", delivered, reason="admitted", node="1", peer="2", next_hop="2", detail="hop_sequence=49"),
            trace_row(3, 3.0, "hop_feedback", delivered, reason="ack", node="2", peer="1", detail=feedback_detail(49, 0, 1)),
            trace_row(4, 4.0, "hop_admission", delivered, reason="admitted", node="2", peer="4", next_hop="4", detail="hop_sequence=50"),
            trace_row(5, 5.0, "nwk_enqueue", delivered, reason="relay", node="4", peer="2"),
            trace_row(6, 6.0, "hop_feedback", delivered, reason="dack", node="4", peer="2", detail=feedback_detail(50, 1, 2)),
            trace_row(7, 7.0, "nwk_enqueue", delivered, reason="relay", node="4", peer="2"),
            trace_row(8, 8.0, "hop_feedback", delivered, reason="dack", node="4", peer="2", detail=feedback_detail(50, 2, 3)),
            trace_row(9, 9.0, "hop_admission", delivered, reason="admitted", node="4", peer="9", next_hop="9", detail="hop_sequence=51"),
            trace_row(10, 10.0, "hop_feedback", delivered, reason="ack", node="9", peer="4", detail=feedback_detail(51, 0, 1)),
            trace_row(11, 11.0, "nwk_delivery", delivered, node="9"),
            trace_row(12, 12.0, "hop_admission", delivered, reason="admitted", node="4", peer="9", next_hop="9", detail="hop_sequence=52"),
            trace_row(13, 13.0, "hop_feedback", delivered, reason="ack", node="9", peer="4", detail=feedback_detail(52, 1, 2)),
            trace_row(14, 14.0, "nwk_delivery", delivered, node="9"),
            trace_row(15, 15.0, "app_send", terminal, node="2"),
            trace_row(16, 16.0, "hop_admission", terminal, reason="admitted", node="2", peer="4", next_hop="4", detail="hop_sequence=59"),
            trace_row(17, 17.0, "hop_feedback", terminal, reason="ack", node="4", peer="2", detail=feedback_detail(59, 0, 1)),
            trace_row(18, 18.0, "hop_admission", terminal, reason="admitted", node="4", peer="5", next_hop="5", detail="hop_sequence=60"),
            trace_row(19, 19.0, "nwk_enqueue", terminal, reason="relay", node="5", peer="4"),
            trace_row(20, 20.0, "hop_feedback", terminal, reason="dack", node="5", peer="4", detail=feedback_detail(60, 4, 5)),
            trace_row(21, 21.0, "nwk_enqueue", terminal, reason="relay", node="5", peer="4"),
            trace_row(22, 22.0, "hop_feedback", terminal, reason="dack", node="5", peer="4", detail=feedback_detail(60, 5, 6)),
        ]
        next_event_index = 23

        def append_terminal_completion(time_s: float) -> None:
            nonlocal next_event_index
            if terminal_completion_reason is None:
                return
            node, peer = (
                ("6", "5") if reverse_terminal_completion else ("5", "6")
            )
            rows.append(
                trace_row(
                    next_event_index,
                    time_s,
                    "hop_completion",
                    terminal,
                    reason=terminal_completion_reason,
                    node=node,
                    peer=peer,
                    next_hop=peer,
                    success=(
                        "0" if terminal_completion_reason == "no_ack" else "1"
                    ),
                    detail="hop_sequence=61;resend_count=2",
                )
            )
            next_event_index += 1

        if terminal_completion_before_admission:
            append_terminal_completion(22.5)
        terminal_admission_event_index = next_event_index
        rows.append(
            trace_row(
                terminal_admission_event_index,
                23.0,
                "hop_admission",
                terminal,
                reason="admitted",
                node="5",
                peer="6",
                next_hop="6",
                detail="hop_sequence=61",
            )
        )
        next_event_index += 1
        if duplicate_terminal_admission:
            rows.append(
                trace_row(
                    next_event_index,
                    23.5,
                    "hop_admission",
                    terminal,
                    reason="admitted",
                    node="5",
                    peer="6",
                    next_hop="6",
                    success="1",
                    detail="hop_sequence=61",
                )
            )
            next_event_index += 1
        if not terminal_completion_before_admission:
            append_terminal_completion(24.0)
        if duplicate_terminal_completion:
            append_terminal_completion(24.5)
        if terminal_attempt_feedback:
            rows.append(
                trace_row(
                    next_event_index,
                    25.0,
                    "hop_feedback",
                    terminal,
                    reason="ack",
                    node="6",
                    peer="5",
                    detail=(
                        "hop_sequence=61;first_reception="
                        f"{terminal_attempt_feedback_first_reception};ackable=1;"
                        "nsdp_state_valid=1;nsdp_count_before=0;"
                        "nsdp_count_after=1;nsdp_limit=16"
                    ),
                ),
            )
        rows.append(
            trace_row(
                40,
                5998.0,
                "nwk_admission",
                terminal,
                reason=terminal_reason,
                node="5",
                peer="9",
                next_hop="9",
                detail=(
                    "queue_before=4;queue_after=4;nsdp_count=3;nsdp_limit=16;"
                    "pending=1;pending_limit=16;global_spad=16;outstanding=1;"
                    "threshold=0;neighbor_spad=0;route_known=1"
                ),
            )
        )
        with trace.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        trace_digest = CERTIFY.sha256(trace)

        lineages = [
            {
                "src": "1",
                "dst": "9",
                "sequence": "10",
                "relay": "4",
                "ingress_peer": "2",
                "hop_sequence": "50",
                "qualifying_feedback_count": 2,
                "dack_feedback_count": 2,
                "proven_extra_delivery_budget": 1,
                "source_ordered_pairs": [
                    {"enqueue_event_index": 5, "feedback_event_index": 6},
                    {"enqueue_event_index": 7, "feedback_event_index": 8},
                ],
            },
            {
                "src": "2",
                "dst": "9",
                "sequence": "20",
                "relay": "5",
                "ingress_peer": "4",
                "hop_sequence": "60",
                "qualifying_feedback_count": 2,
                "dack_feedback_count": 2,
                "proven_extra_delivery_budget": 1,
                "source_ordered_pairs": [
                    {"enqueue_event_index": 19, "feedback_event_index": 20},
                    {"enqueue_event_index": 21, "feedback_event_index": 22},
                ],
            },
        ]
        provenance = root / "provenance.json"
        provenance.write_text(
            json.dumps(
                {
                    "schema": "csr-ns3-aggregate-provenance-v1",
                    "scenario": "blue_radio_campus-multihop",
                    "input": {
                        "path": str(trace.resolve()),
                        "sha256": trace_digest,
                        "size_bytes": trace.stat().st_size,
                        "event_row_count": len(rows),
                    },
                    "window": {
                        "bucket_width_s": 60.0,
                        "stop_time_s": 6000.0,
                        "stop_endpoint": "exclusive",
                    },
                    "source_exact_dack_retry_duplicates": {
                        "repeated_feedback_proof_count": 2,
                        "matched_duplicate_delivery_count": 1,
                        "unused_proof_count": 1,
                        "lineages": lineages,
                    }
                }
            ),
            encoding="utf-8",
        )

        packet_csv = root / "packet-paths.csv"
        packet_columns = (
            "src",
            "dst",
            "sequence",
            "status",
            "valid",
            "complete",
            "delivered",
            "issues_json",
        )
        with packet_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=packet_columns, lineterminator="\n")
            writer.writeheader()
            writer.writerow(
                {
                    "src": "1",
                    "dst": "9",
                    "sequence": "10",
                    "status": "invalid_incomplete_delivered",
                    "valid": "0",
                    "complete": "0",
                    "delivered": "1",
                    "issues_json": json.dumps(
                        [
                            {"code": "duplicate_delivery"},
                            {"code": "incomplete_or_out_of_order_lifecycle"},
                        ]
                    ),
                }
            )
            writer.writerow(
                {
                    "src": "2",
                    "dst": "9",
                    "sequence": "20",
                    "status": "invalid_incomplete_undelivered",
                    "valid": "0",
                    "complete": "0",
                    "delivered": "0",
                    "issues_json": json.dumps(
                        [
                            {"code": "missing_delivery"},
                            {"code": "incomplete_or_out_of_order_lifecycle"},
                        ]
                    ),
                }
            )

        packet_summary = root / "packet-summary.json"
        packet_summary.write_text(
            json.dumps(
                {
                    "schema": "csr-packet-path-analysis-v1",
                    "pass": False,
                    "input": {
                        "path": str(trace.resolve()),
                        "sha256": trace_digest,
                        "row_count": len(rows),
                    },
                    "configuration": {
                        "startup_time_s": 300.0,
                        "end_to_end_bucket_width_s": 60.0,
                        "end_to_end_stop_time_s": 6000.0,
                        "end_to_end_stop_time_semantics": "exclusive",
                        "decomposition_tolerance_s": 1.0e-9,
                    },
                    "outputs": {
                        "packet_csv": {
                            "path": str(packet_csv.resolve()),
                            "sha256": CERTIFY.sha256(packet_csv),
                            "row_count": 2,
                        }
                    },
                    "counts": {
                        "packet_keys": 2,
                        "invalid_packets": 2,
                        "delivered_packets": 1,
                        "valid_complete_delivered_packets": 0,
                        "incomplete_delivered_packets": 1,
                        "undelivered_packets": 1,
                        "incomplete_packets": 2,
                        "route_context_mismatch_packets": 0,
                        "route_context_mismatches": 0,
                    },
                    "issue_counts": {
                        "duplicate_delivery": 1,
                        "incomplete_or_out_of_order_lifecycle": 2,
                        "missing_delivery": 1,
                    },
                }
            ),
            encoding="utf-8",
        )

        admission_summary = root / "admission-summary.json"
        admission_legs = root / "admission-legs.csv"
        with admission_legs.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=CERTIFY.ADMISSION_LEG_COLUMNS,
                lineterminator="\n",
            )
            writer.writeheader()

        diagnostics = root / "app-admission-diagnostics.csv"
        with diagnostics.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=CERTIFY.APP_DIAGNOSTIC_COLUMNS,
                lineterminator="\n",
            )
            writer.writeheader()
            for flow_index in range(6):
                writer.writerow(
                    {
                        "schema": "csr-app-admission-diagnostics-v1",
                        "scenario": "blue_radio_campus-multihop",
                        "application_profile": "legacy-send-only-no-dscp",
                        "flow_index": str(flow_index),
                        "source": str(flow_index + 2),
                        "configured_destination": "1",
                        "destination_mode": "fixed",
                        "attempts": "0",
                        "admitted": "0",
                        "blocked_discovery": "0",
                        "blocked_topology": "0",
                        "blocked_gateway_route": "0",
                        "blocked_destination": "0",
                        "blocked_nsdp": "0",
                        "first_admitted_s": "",
                        "last_admitted_s": "",
                    }
                )
        event_counts: dict[str, int] = {}
        for row in rows:
            event_counts[row["event"]] = event_counts.get(row["event"], 0) + 1
        diagnostic_totals = {
            name: 0 for name in CERTIFY.APP_DIAGNOSTIC_COUNTERS
        }
        admission_summary.write_text(
            json.dumps(
                {
                    "schema": "csr-admission-ledger-analysis-v1",
                    "pass": False,
                    "input": {
                        "path": str(trace.resolve()),
                        "sha256": trace_digest,
                        "row_count": len(rows),
                        "event_counts": event_counts,
                    },
                    "outputs": {
                        "legs_csv": {
                            "path": str(admission_legs.resolve()),
                            "sha256": CERTIFY.sha256(admission_legs),
                            "row_count": 0,
                        }
                    },
                    "inventory": {"hop_legs": 0, "open_nwk_locations": 0},
                    "hop": {
                        "dack_pre_limit_findings": 0,
                        "dack_boundary_count": 0,
                    },
                    "application_diagnostics": {
                        "path": str(diagnostics.resolve()),
                        "sha256": CERTIFY.sha256(diagnostics),
                        "flow_rows": 6,
                        "totals": diagnostic_totals,
                        "mismatches": [],
                        "pass": True,
                    },
                    "integrity": {
                        "invalid_leg_count": 0,
                        "global_issue_count": 1,
                        "global_issues": [
                            {
                                "code": "duplicate_delivery",
                                "message": (
                                    "packet PacketKey(source=1, destination=9, "
                                    "sequence=10) has 2 deliveries"
                                ),
                            }
                        ],
                    },
                }
            ),
            encoding="utf-8",
        )
        certification = CERTIFY.Certification(
            Namespace(output_dir=root / "output", work_dir=root / "work")
        )
        return (
            temporary,
            certification,
            trace,
            provenance,
            admission_summary,
            packet_summary,
            packet_csv,
        )

    def classify(self, terminal_reason: str = "neighbor_flow_full") -> dict[str, object]:
        case = self.build_case(terminal_reason)
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        return certification.classify_analyzer_results(
            trace,
            provenance,
            admission,
            1,
            summary,
            packets,
            1,
        )

    def build_v2_case(
        self,
        *,
        terminal_attempt_feedback: bool = False,
        terminal_attempt_feedback_first_reception: str = "1",
        terminal_completion_reason: str | None = "no_ack",
        duplicate_terminal_admission: bool = False,
        duplicate_terminal_completion: bool = False,
        terminal_completion_before_admission: bool = False,
        reverse_terminal_completion: bool = False,
    ) -> tuple[object, ...]:
        case = self.build_case(
            terminal_attempt_feedback=terminal_attempt_feedback,
            terminal_attempt_feedback_first_reception=(
                terminal_attempt_feedback_first_reception
            ),
            terminal_completion_reason=terminal_completion_reason,
            duplicate_terminal_admission=duplicate_terminal_admission,
            duplicate_terminal_completion=duplicate_terminal_completion,
            terminal_completion_before_admission=(
                terminal_completion_before_admission
            ),
            reverse_terminal_completion=reverse_terminal_completion,
        )
        (
            temporary,
            certification,
            trace,
            provenance,
            admission,
            summary,
            packets,
        ) = case
        packet_columns = (
            "src",
            "dst",
            "sequence",
            "copy_index",
            "status",
            "valid",
            "complete",
            "delivered",
            "path_json",
            "issue_count",
            "issues_json",
            "hop_lineage_json",
        )

        def leg(
            key: tuple[str, str, str],
            from_node: int,
            to_node: int,
            hop_sequence: int,
        ) -> dict[str, int]:
            return {
                "src": int(key[0]),
                "dst": int(key[1]),
                "sequence": int(key[2]),
                "from": from_node,
                "to": to_node,
                "hop_sequence": hop_sequence,
            }

        delivered = ("1", "9", "10")
        terminal = ("2", "9", "20")
        rows: list[dict[str, str]] = []
        for copy_index, final_sequence in enumerate((51, 52)):
            rows.append(
                {
                    "src": delivered[0],
                    "dst": delivered[1],
                    "sequence": delivered[2],
                    "copy_index": str(copy_index),
                    "status": "valid_delivered",
                    "valid": "1",
                    "complete": "1",
                    "delivered": "1",
                    "path_json": json.dumps([1, 2, 4, 9]),
                    "issue_count": "0",
                    "issues_json": "[]",
                    "hop_lineage_json": json.dumps(
                        [
                            leg(delivered, 1, 2, 49),
                            leg(delivered, 2, 4, 50),
                            leg(delivered, 4, 9, final_sequence),
                        ]
                    ),
                }
            )
        terminal_issues = [
            {"code": "missing_delivery"},
            {"code": "undelivered_lifecycle"},
        ]
        for copy_index in range(2):
            terminal_lineage = [
                leg(terminal, 2, 4, 59),
                leg(terminal, 4, 5, 60),
            ]
            if copy_index == 0:
                terminal_lineage.append(leg(terminal, 5, 6, 61))
            rows.append(
                {
                    "src": terminal[0],
                    "dst": terminal[1],
                    "sequence": terminal[2],
                    "copy_index": str(copy_index),
                    "status": "incomplete_undelivered",
                    "valid": "1",
                    "complete": "0",
                    "delivered": "0",
                    "path_json": json.dumps([2, 4, 5]),
                    "issue_count": "2",
                    "issues_json": json.dumps(terminal_issues),
                    "hop_lineage_json": json.dumps(terminal_lineage),
                }
            )
        with packets.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=packet_columns, lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(rows)

        with trace.open("r", encoding="utf-8") as stream:
            trace_rows = sum(1 for _ in stream) - 1
        summary.write_text(
            json.dumps(
                {
                    "schema": "csr-packet-path-analysis-v2",
                    "pass": True,
                    "input": {
                        "path": str(trace.resolve()),
                        "sha256": CERTIFY.sha256(trace),
                        "row_count": trace_rows,
                    },
                    "configuration": {
                        "packet_key": ["src", "dst", "sequence"],
                        "directed_hop_identity": [
                            "src",
                            "dst",
                            "sequence",
                            "from",
                            "to",
                            "hop_sequence",
                        ],
                        "repeated_dack_branch_matching": (
                            CERTIFY.PACKET_PATH_V2_MATCHING_CONTRACT
                        ),
                        "undelivered_path_semantics": (
                            CERTIFY.PACKET_PATH_V2_UNDELIVERED_PATH_CONTRACT
                        ),
                        "queue_delay_statistic": (
                            CERTIFY.PACKET_PATH_V2_QUEUE_DELAY_STATISTIC
                        ),
                        "queue_delay_evidence_adjacency": (
                            CERTIFY.PACKET_PATH_V2_QUEUE_DELAY_ADJACENCY
                        ),
                        "queue_delay_match_abs_tolerance_s": 1.0e-9,
                        "startup_time_s": 300.0,
                        "end_to_end_bucket_width_s": 60.0,
                        "end_to_end_stop_time_s": 6000.0,
                        "end_to_end_stop_time_semantics": "exclusive",
                        "decomposition_tolerance_s": 1.0e-9,
                    },
                    "outputs": {
                        "packet_csv": {
                            "path": str(packets.resolve()),
                            "sha256": CERTIFY.sha256(packets),
                            "row_count": 4,
                        }
                    },
                    "counts": {
                        "packet_keys": 2,
                        "packet_lifecycles": 4,
                        "reconstructed_branch_packet_keys": 2,
                        "source_exact_dack_retry_forks": 2,
                        "ambiguous_repeated_dack_branch_packet_keys": 0,
                        "delivered_packets": 2,
                        "valid_complete_delivered_packets": 2,
                        "invalid_packets": 0,
                        "incomplete_packets": 2,
                        "incomplete_delivered_packets": 0,
                        "undelivered_packets": 2,
                        "route_context_mismatch_packets": 0,
                        "route_context_mismatches": 0,
                    },
                    "issue_counts": {
                        "missing_delivery": 2,
                        "undelivered_lifecycle": 2,
                    },
                }
            ),
            encoding="utf-8",
        )
        return case

    def classify_v2(self) -> dict[str, object]:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        return certification.classify_analyzer_results(
            trace,
            provenance,
            admission,
            1,
            summary,
            packets,
            0,
        )

    def test_accepts_exact_matched_and_terminal_unused_lineage_partition(self) -> None:
        result = self.classify()
        packet = result["packet_path_analyzer"]
        self.assertEqual(packet["strict_result"], "fail")
        self.assertEqual(packet["release_disposition"], "documented_nonblocking_scope_exclusion")
        self.assertFalse(packet["packet_path_integrity_claim"])
        self.assertNotIn("gate_pass", packet)
        self.assertEqual(packet["proved_extra_delivery_count"], 1)
        self.assertEqual(packet["unused_lineage_count"], 1)
        self.assertEqual(
            packet["terminal_undelivered_proofs"][0]["sequence"], "20"
        )

    def test_rejects_unused_lineage_without_valid_terminal_hold(self) -> None:
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "lacks a final-bucket relay capacity hold",
        ):
            self.classify("global_flow_full")

    def test_accepts_strict_v2_branch_lifecycle_ledger(self) -> None:
        result = self.classify_v2()
        packet = result["packet_path_analyzer"]
        self.assertEqual(packet["schema"], "csr-packet-path-analysis-v2")
        self.assertEqual(packet["strict_result"], "pass")
        self.assertEqual(packet["release_disposition"], "accepted_clean_gate")
        self.assertTrue(packet["packet_path_integrity_claim"])
        self.assertFalse(packet["opnet_packet_event_parity_claim"])
        self.assertEqual(packet["packet_lifecycle_count"], 4)
        self.assertEqual(packet["terminal_unreceived_hop_attempt_count"], 1)
        self.assertEqual(
            packet["terminal_unreceived_hop_attempts"],
            [
                {
                    "src": 2,
                    "dst": 9,
                    "sequence": 20,
                    "copy_index": 0,
                    "from": 5,
                    "to": 6,
                    "hop_sequence": 61,
                    "admission_event_index": 23,
                    "completion_event_index": 24,
                    "completion_success": 0,
                    "completion_reason": "no_ack",
                }
            ],
        )

    def test_hop_lineage_path_relation_is_fail_closed(self) -> None:
        exact = [
            {"from": 1, "to": 2},
            {"from": 2, "to": 4},
        ]
        terminal = exact + [{"from": 4, "to": 5}]
        self.assertEqual(
            CERTIFY.hop_lineage_path_relation([1, 2, 4], exact, True),
            "exact_observed_path",
        )
        self.assertEqual(
            CERTIFY.hop_lineage_path_relation([1, 2, 4], terminal, False),
            "terminal_admitted_unreceived",
        )
        self.assertIsNone(
            CERTIFY.hop_lineage_path_relation([1, 2, 4], terminal, True)
        )
        self.assertIsNone(
            CERTIFY.hop_lineage_path_relation(
                [1, 2, 4], terminal + [{"from": 5, "to": 6}], False
            )
        )
        self.assertIsNone(
            CERTIFY.hop_lineage_path_relation(
                [1, 2, 4], exact + [{"from": 9, "to": 5}], False
            )
        )
        self.assertIsNone(
            CERTIFY.hop_lineage_path_relation(
                [1, 2, 4], exact + [{"from": 4, "to": 1}], False
            )
        )
        self.assertIsNone(
            CERTIFY.hop_lineage_path_relation(
                [1, 2, 4],
                [{"from": 1, "to": 2}, {"from": 7, "to": 4}],
                False,
            )
        )

    def test_rejects_v2_terminal_attempt_with_receiver_feedback(self) -> None:
        for first_reception in ("1", "0"):
            with self.subTest(first_reception=first_reception):
                case = self.build_v2_case(
                    terminal_attempt_feedback=True,
                    terminal_attempt_feedback_first_reception=first_reception,
                )
                (
                    temporary,
                    certification,
                    trace,
                    provenance,
                    admission,
                    summary,
                    packets,
                ) = case
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(
                    CERTIFY.CertificationError,
                    "terminal HOP attempt has receiver feedback",
                ):
                    certification.classify_analyzer_results(
                        trace, provenance, admission, 1, summary, packets, 0
                    )

    def test_rejects_v2_terminal_attempt_without_no_ack_completion(self) -> None:
        for reason in (None, "ack"):
            with self.subTest(reason=reason):
                case = self.build_v2_case(terminal_completion_reason=reason)
                (
                    temporary,
                    certification,
                    trace,
                    provenance,
                    admission,
                    summary,
                    packets,
                ) = case
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(
                    CERTIFY.CertificationError,
                    "terminal HOP attempt lacks one exact post-admission no_ack completion",
                ):
                    certification.classify_analyzer_results(
                        trace, provenance, admission, 1, summary, packets, 0
                    )

    def test_rejects_v2_duplicate_terminal_hop_admission(self) -> None:
        case = self.build_v2_case(duplicate_terminal_admission=True)
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "trace repeats a directed-HOP admission identity",
        ):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 0
            )

    def test_rejects_v2_duplicate_terminal_hop_completion(self) -> None:
        case = self.build_v2_case(duplicate_terminal_completion=True)
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "terminal HOP attempt lacks one exact post-admission no_ack completion",
        ):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 0
            )

    def test_rejects_v2_pre_admission_terminal_hop_completion(self) -> None:
        case = self.build_v2_case(terminal_completion_before_admission=True)
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "terminal HOP attempt lacks one exact post-admission no_ack completion",
        ):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 0
            )

    def test_rejects_v2_reversed_terminal_hop_completion(self) -> None:
        case = self.build_v2_case(reverse_terminal_completion=True)
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "terminal HOP attempt lacks one exact post-admission no_ack completion",
        ):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 0
            )

    def test_rejects_reusing_one_terminal_hop_across_lifecycles(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with packets.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames or [])
            rows = list(reader)
        first_terminal_lineage = json.loads(rows[2]["hop_lineage_json"])
        second_terminal_lineage = json.loads(rows[3]["hop_lineage_json"])
        second_terminal_lineage.append(first_terminal_lineage[-1])
        rows[3]["hop_lineage_json"] = json.dumps(second_terminal_lineage)
        with packets.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=fieldnames, lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(rows)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["outputs"]["packet_csv"]["sha256"] = CERTIFY.sha256(packets)
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "reuses one terminal HOP attempt across lifecycles",
        ):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 0
            )

    def test_rejects_terminal_hop_reused_as_observed_path_edge(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with packets.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames or [])
            rows = list(reader)
        terminal_lineage = json.loads(rows[2]["hop_lineage_json"])
        rows[3]["path_json"] = json.dumps([2, 4, 5, 6])
        rows[3]["hop_lineage_json"] = json.dumps(terminal_lineage)
        with packets.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=fieldnames, lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(rows)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["outputs"]["packet_csv"]["sha256"] = CERTIFY.sha256(packets)
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "observed path edge lacks post-admission first-reception evidence",
        ):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 0
            )

    def test_rejects_v2_branch_count_drift(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["counts"]["source_exact_dack_retry_forks"] = 1
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "source_exact_dack_retry_forks",
        ):
            certification.classify_analyzer_results(
                trace,
                provenance,
                admission,
                1,
                summary,
                packets,
                0,
            )

    def test_rejects_v2_existence_only_matching_contract(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["configuration"]["repeated_dack_branch_matching"] = (
            "exact directed-HOP identity plus complete bipartite queue-copy "
            "matching; never FIFO"
        )
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "unique queue-delay matching contract drift",
        ):
            certification.classify_analyzer_results(
                trace,
                provenance,
                admission,
                1,
                summary,
                packets,
                0,
            )

    def test_rejects_v2_undelivered_path_contract_drift(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["configuration"]["undelivered_path_semantics"] = (
            "path includes an unreceived next hop"
        )
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "unique queue-delay matching contract drift",
        ):
            certification.classify_analyzer_results(
                trace,
                provenance,
                admission,
                1,
                summary,
                packets,
                0,
            )

    def test_rejects_v2_repeated_delivered_hop_lineage(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with packets.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            self.assertIsNotNone(reader.fieldnames)
            fieldnames = list(reader.fieldnames or [])
            rows = list(reader)
        rows[1]["hop_lineage_json"] = rows[0]["hop_lineage_json"]
        with packets.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["outputs"]["packet_csv"]["sha256"] = CERTIFY.sha256(packets)
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "repeats one delivered HOP lineage",
        ):
            certification.classify_analyzer_results(
                trace,
                provenance,
                admission,
                1,
                summary,
                packets,
                0,
            )

    def test_rejects_v2_issue_relocation_between_lifecycles(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with packets.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            self.assertIsNotNone(reader.fieldnames)
            fieldnames = list(reader.fieldnames or [])
            rows = list(reader)
        rows[0]["issue_count"] = rows[2]["issue_count"]
        rows[0]["issues_json"] = rows[2]["issues_json"]
        rows[2]["issue_count"] = "0"
        rows[2]["issues_json"] = "[]"
        with packets.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["outputs"]["packet_csv"]["sha256"] = CERTIFY.sha256(packets)
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(
            CERTIFY.CertificationError,
            "moves terminal-prefix issues to the wrong lifecycle",
        ):
            certification.classify_analyzer_results(
                trace,
                provenance,
                admission,
                1,
                summary,
                packets,
                0,
            )

    def test_rejects_v2_hop_lineage_absent_from_trace(self) -> None:
        case = self.build_v2_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        with packets.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames or [])
            rows = list(reader)
        lineage = json.loads(rows[0]["hop_lineage_json"])
        lineage[-1]["hop_sequence"] = 999
        rows[0]["hop_lineage_json"] = json.dumps(lineage)
        with packets.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        value = json.loads(summary.read_text(encoding="utf-8"))
        value["outputs"]["packet_csv"]["sha256"] = CERTIFY.sha256(packets)
        summary.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(CERTIFY.CertificationError, "absent from the trace"):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 0
            )

    def test_admission_integrity_is_exact_for_pass_and_exception(self) -> None:
        clean = {
            "pass": True,
            "integrity": {
                "invalid_leg_count": 0,
                "global_issue_count": 0,
                "global_issues": [],
            },
        }
        result = CERTIFY.Certification.classify_admission_integrity(clean, 0, {})
        self.assertEqual(result["classification"], "strict_clean_pass")
        clean["integrity"]["invalid_leg_count"] = 1
        with self.assertRaisesRegex(CERTIFY.CertificationError, "pass contradicts"):
            CERTIFY.Certification.classify_admission_integrity(clean, 0, {})

        duplicate = {
            "pass": False,
            "integrity": {
                "invalid_leg_count": 0,
                "global_issue_count": 2,
                "global_issues": [
                    {
                        "code": "duplicate_delivery",
                        "message": (
                            "packet PacketKey(source=1, destination=2, sequence=3) "
                            "has 2 deliveries"
                        ),
                    }
                ],
            },
        }
        with self.assertRaisesRegex(CERTIFY.CertificationError, "counts/issues"):
            CERTIFY.Certification.classify_admission_integrity(
                duplicate, 1, {("1", "2", "3"): 1}
            )

    def test_admission_leg_artifact_is_hash_bound(self) -> None:
        case = self.build_case()
        temporary, certification, trace, provenance, admission, summary, packets = case
        self.addCleanup(temporary.cleanup)
        (admission.parent / "admission-legs.csv").write_text(
            ",".join(CERTIFY.ADMISSION_LEG_COLUMNS) + "\n\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(CERTIFY.CertificationError, "leg artifact"):
            certification.classify_analyzer_results(
                trace, provenance, admission, 1, summary, packets, 1
            )

    def test_legacy_clean_pass_rows_and_counts_are_strict(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-v1-clean-probe-") as name:
            packet_csv = Path(name) / "packet-paths.csv"
            fields = (
                "src",
                "dst",
                "sequence",
                "status",
                "valid",
                "complete",
                "delivered",
                "issue_count",
                "issues_json",
            )
            row = {
                "src": "1",
                "dst": "2",
                "sequence": "3",
                "status": "valid_delivered",
                "valid": "1",
                "complete": "1",
                "delivered": "1",
                "issue_count": "0",
                "issues_json": "[]",
            }
            with packet_csv.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
                writer.writeheader()
                writer.writerow(row)
            key = ("1", "2", "3")
            observations = {
                "app_sends": CERTIFY.Counter({key: 1}),
                "deliveries": CERTIFY.Counter({key: 1}),
            }
            summary = {
                "pass": True,
                "counts": {
                    "packet_keys": 1,
                    "delivered_packets": 1,
                    "valid_complete_delivered_packets": 1,
                    "invalid_packets": 0,
                    "incomplete_packets": 0,
                    "incomplete_delivered_packets": 0,
                    "undelivered_packets": 0,
                    "route_context_mismatch_packets": 0,
                    "route_context_mismatches": 0,
                },
                "issue_counts": {},
            }
            result = CERTIFY.Certification.classify_packet_path_v1_clean(
                summary, packet_csv, 0, {}, observations
            )
            self.assertEqual(result["strict_result"], "pass")
            row["complete"] = "0"
            with packet_csv.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
                writer.writeheader()
                writer.writerow(row)
            with self.assertRaisesRegex(CERTIFY.CertificationError, "exact clean delivery"):
                CERTIFY.Certification.classify_packet_path_v1_clean(
                    summary, packet_csv, 0, {}, observations
                )

    def test_cmake_cache_parser_rejects_duplicate_and_preserves_exact_values(self) -> None:
        parsed = CERTIFY.parse_cmake_cache(
            "CMAKE_BUILD_TYPE:STRING=debugfoo\n"
            "NS3_ENABLED_MODULES:STRING=csr;internet\n"
        )
        self.assertEqual(parsed["CMAKE_BUILD_TYPE"], ("STRING", "debugfoo"))
        self.assertNotEqual(parsed["CMAKE_BUILD_TYPE"], ("STRING", "debug"))
        self.assertNotEqual(parsed["NS3_ENABLED_MODULES"], ("STRING", "csr"))
        with self.assertRaisesRegex(CERTIFY.CertificationError, "repeats assignment"):
            CERTIFY.parse_cmake_cache(
                "CMAKE_BUILD_TYPE:STRING=debug\n"
                "CMAKE_BUILD_TYPE:STRING=release\n"
            )

    def test_historical_security_provenance_binds_data_and_ack_projection(self) -> None:
        digest = CERTIFY.EXTERNAL_INPUTS["multihop_executable"][1]
        record = {
            "application_profile": "legacy-send-only-no-dscp",
            "mac_profile": "hist-2014-next-tslot-modulo-probe",
            "hop_security_profile": CERTIFY.HISTORICAL_HOP_SECURITY_PROFILE,
            "hop_security_profile_origin": "explicit",
            "hop_security_behavior": dict(
                CERTIFY.HISTORICAL_HOP_SECURITY_BEHAVIOR
            ),
            "ack_envelope_profile": "unspecified",
            "source_executable_sha256": digest,
        }
        CERTIFY.validate_historical_security_provenance(record, dict(record), digest)
        changed = dict(record)
        changed["hop_security_behavior"] = {
            "ordinary_data": "pairwise16",
            "ack_dack": "bare",
        }
        with self.assertRaisesRegex(CERTIFY.CertificationError, "DATA/ACK"):
            CERTIFY.validate_historical_security_provenance(
                changed, dict(record), digest
            )

    def test_effective_hop_profile_always_binds_source_executable_semantics(self) -> None:
        digest = CERTIFY.EXTERNAL_INPUTS["multihop_executable"][1]
        historical_alias = {
            "hop_security_profile": CERTIFY.HISTORICAL_HOP_SECURITY_PROFILE,
            "hop_security_profile_origin": "legacy_ack_alias",
            "source_executable_sha256": digest,
        }
        CERTIFY.validate_effective_hop_security_source_binding(historical_alias)
        historical_alias["source_executable_sha256"] = ""
        with self.assertRaisesRegex(CERTIFY.CertificationError, "lacks exact"):
            CERTIFY.validate_effective_hop_security_source_binding(historical_alias)

        production = {
            "hop_security_profile": "production-pairwise16",
            "hop_security_profile_origin": "explicit",
            "source_executable_sha256": "",
        }
        CERTIFY.validate_effective_hop_security_source_binding(production)
        production["source_executable_sha256"] = digest
        with self.assertRaisesRegex(CERTIFY.CertificationError, "claims historical"):
            CERTIFY.validate_effective_hop_security_source_binding(production)

    def test_preflight_git_environment_is_minimal_and_ignored_files_fail(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-git-clean-probe-") as name:
            root = Path(name)
            subprocess.run([CERTIFY.GIT, "init", "-q", str(root)], check=True)
            (root / ".gitignore").write_text("ignored.bin\n", encoding="utf-8")
            subprocess.run([CERTIFY.GIT, "-C", str(root), "add", ".gitignore"], check=True)
            subprocess.run(
                [
                    CERTIFY.GIT,
                    "-C",
                    str(root),
                    "-c",
                    "user.name=probe",
                    "-c",
                    "user.email=probe@example.invalid",
                    "commit",
                    "-qm",
                    "fixture",
                ],
                check=True,
            )
            certification = CERTIFY.Certification(
                Namespace(output_dir=root / "output", work_dir=root / "work")
            )
            self.assertNotIn("LD_PRELOAD", certification.git_env)
            self.assertNotIn("PYTHONPATH", certification.git_env)
            certification.git_clean(root)
            (root / "ignored.bin").write_text("not clean\n", encoding="utf-8")
            with self.assertRaisesRegex(CERTIFY.CertificationError, "ignored worktree"):
                certification.git_clean(root)

    def test_dynamic_unittest_count_requires_one_positive_ok_summary(self) -> None:
        self.assertEqual(
            CERTIFY.parse_unittest_count("Ran 237 tests in 1.0s\n\nOK\n"),
            237,
        )
        with self.assertRaises(CERTIFY.CertificationError):
            CERTIFY.parse_unittest_count("Ran 0 tests in 0.0s\n\nOK\n")
        with self.assertRaises(CERTIFY.CertificationError):
            CERTIFY.parse_unittest_count(
                "Ran 1 test in 0.0s\nRan 2 tests in 0.0s\nOK\n"
            )
        self.assertEqual(
            CERTIFY.parse_discovered_test_count("CSR_DISCOVERED_TESTS=237\n"),
            237,
        )
        with self.assertRaises(CERTIFY.CertificationError):
            CERTIFY.parse_discovered_test_count("CSR_DISCOVERED_TESTS=0\n")
        with self.assertRaises(CERTIFY.CertificationError):
            CERTIFY.parse_discovered_test_count(
                "CSR_DISCOVERED_TESTS=1\nCSR_DISCOVERED_TESTS=2\n"
            )

    def test_named_source_set_is_exact_and_hash_bound(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-source-set-probe-") as name:
            root = Path(name)
            first = root / "first.c"
            second = root / "second.c"
            first.write_text("first\n", encoding="utf-8")
            second.write_text("second\n", encoding="utf-8")
            expected = {
                first.name: CERTIFY.sha256(first),
                second.name: CERTIFY.sha256(second),
            }
            records = CERTIFY.validate_named_file_set(root, expected)
            self.assertEqual(set(records), set(expected))
            (root / "extra.c").write_text("extra\n", encoding="utf-8")
            with self.assertRaisesRegex(
                CERTIFY.CertificationError, "filename set drift"
            ):
                CERTIFY.validate_named_file_set(root, expected)
            (root / "extra.c").unlink()
            first.write_text("changed\n", encoding="utf-8")
            with self.assertRaisesRegex(
                CERTIFY.CertificationError, "source hash mismatch"
            ):
                CERTIFY.validate_named_file_set(root, expected)
            first.unlink()
            first.symlink_to(second.name)
            with self.assertRaisesRegex(
                CERTIFY.CertificationError, "not a regular file"
            ):
                CERTIFY.validate_named_file_set(root, expected)

    def test_supplied_source_directory_contract_is_exact_including_hidden_names(
        self,
    ) -> None:
        self.assertEqual(len(CERTIFY.SUPPLEMENTAL_SOURCE_INPUTS), 5)
        self.assertEqual(
            set(CERTIFY.SUPPLIED_SOURCE_DIRECTORY_INPUTS),
            {
                "CSR project examples.zip",
                "more missing files.zip",
                "routing.lib",
                "routes(1).c",
                "Modulation_tables.zip",
                "Project files for chatgpt(1).zip",
            },
        )
        self.assertEqual(
            CERTIFY.SUPPLIED_SOURCE_DIRECTORY_INPUTS[
                CERTIFY.PRIMARY_SOURCE_ARCHIVE_NAME
            ],
            CERTIFY.EXTERNAL_INPUTS["source_archive"][1],
        )
        with tempfile.TemporaryDirectory(prefix="csr-supplied-source-probe-") as name:
            root = Path(name)
            expected: dict[str, str] = {}
            for filename in CERTIFY.SUPPLIED_SOURCE_DIRECTORY_INPUTS:
                path = root / filename
                path.write_text(filename + "\n", encoding="utf-8")
                expected[filename] = CERTIFY.sha256(path)
            records = CERTIFY.validate_named_file_set(
                root,
                expected,
                description="supplied source artifact",
                include_hidden=True,
            )
            self.assertEqual(set(records), set(expected))
            (root / ".unexpected").write_text("drift\n", encoding="utf-8")
            with self.assertRaisesRegex(
                CERTIFY.CertificationError, "supplied source artifact filename set drift"
            ):
                CERTIFY.validate_named_file_set(
                    root,
                    expected,
                    description="supplied source artifact",
                    include_hidden=True,
                )

    def test_release_harness_and_classifier_bind_to_exact_rc_commit_blobs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-self-binding-probe-") as name:
            root = Path(name).resolve()
            release_dir = root / "utils" / "release"
            release_dir.mkdir(parents=True)
            harness = release_dir / "certify_release.py"
            classifier = release_dir / "test_certify_release_classifier.py"
            harness.write_text("# bound harness\n", encoding="utf-8")
            classifier.write_text("# bound classifier\n", encoding="utf-8")
            subprocess.run([CERTIFY.GIT, "init", "-q", str(root)], check=True)
            subprocess.run([CERTIFY.GIT, "-C", str(root), "add", "utils"], check=True)
            subprocess.run(
                [
                    CERTIFY.GIT,
                    "-C",
                    str(root),
                    "-c",
                    "user.name=probe",
                    "-c",
                    "user.email=probe@example.invalid",
                    "commit",
                    "-qm",
                    "fixture",
                ],
                check=True,
            )
            commit = subprocess.check_output(
                [CERTIFY.GIT, "-C", str(root), "rev-parse", "HEAD"],
                text=True,
            ).strip()
            certification = CERTIFY.Certification(
                Namespace(output_dir=root / "output", work_dir=root / "work")
            )
            bindings = certification.validate_release_harness_binding(
                root,
                commit,
                harness_path=harness,
                classifier_path=classifier,
            )
            self.assertEqual(
                set(bindings), set(CERTIFY.RELEASE_HARNESS_REPOSITORY_PATHS)
            )
            self.assertTrue(
                all(item["matches_rc_commit_blob"] for item in bindings.values())
            )

            classifier.write_text("# modified classifier\n", encoding="utf-8")
            with self.assertRaisesRegex(
                CERTIFY.CertificationError,
                "classifier_self_test content does not match the exact blob",
            ):
                certification.validate_release_harness_binding(
                    root,
                    commit,
                    harness_path=harness,
                    classifier_path=classifier,
                )

    def test_supplied_source_directory_argument_is_required(self) -> None:
        action = next(
            item
            for item in CERTIFY.build_parser()._actions
            if item.dest == "supplied_source_dir"
        )
        self.assertTrue(action.required)

    def test_scope_exemptions_are_hash_bound_subsets(self) -> None:
        self.assertEqual(len(CERTIFY.OPNET_SOURCE_INPUTS), 16)
        self.assertEqual(
            set(CERTIFY.OPNET_SCOPE_EXEMPTIONS),
            {"10-br_supervisor.pr.c", "12-br_battery.pr.c"},
        )
        self.assertLessEqual(
            set(CERTIFY.OPNET_SCOPE_EXEMPTIONS),
            set(CERTIFY.OPNET_SOURCE_INPUTS),
        )

    def test_application_packet_size_is_an_independent_exact_gate(self) -> None:
        configuration, report = application_aggregate_report()
        result = CERTIFY.validate_application_aggregate_coverage(
            configuration, report
        )
        self.assertEqual(result["numeric_points_compared"], 95)
        self.assertEqual(result["numeric_mismatches"], 0)
        result["numeric_mismatches"] = 1
        packet_size = next(
            record
            for record in report["series"]
            if record["statistic"] == CERTIFY.APPLICATION_PACKET_SIZE_STATISTIC
        )
        packet_size["numeric_mismatches"] = 1
        packet_size["pass"] = False
        with self.assertRaisesRegex(
            CERTIFY.CertificationError, "packet-size parity gate failed"
        ):
            CERTIFY.validate_application_aggregate_coverage(configuration, report)

    def test_application_aggregate_rejects_statistic_substitution(self) -> None:
        configuration, report = application_aggregate_report()
        configuration["statistics"][-1] = "substituted"
        with self.assertRaisesRegex(
            CERTIFY.CertificationError, "statistic set/order drift"
        ):
            CERTIFY.validate_application_aggregate_coverage(configuration, report)

    def test_protocol_aggregate_accepts_exact_steady_coverage_with_residual(self) -> None:
        report = protocol_aggregate_report()
        result = CERTIFY.validate_protocol_aggregate_coverage(report, 1)
        self.assertEqual(result["counts"]["matched_points"], 380)
        self.assertEqual(result["counts"]["numeric_mismatches"], 378)
        self.assertEqual(
            [record["statistic"] for record in result["series"]],
            list(CERTIFY.PROTOCOL_AGGREGATE_STATISTICS),
        )

    def test_protocol_aggregate_rejects_missing_hop_series(self) -> None:
        report = protocol_aggregate_report()
        report["series"] = report["series"][1:]
        with self.assertRaisesRegex(
            CERTIFY.CertificationError, "statistic set drift"
        ):
            CERTIFY.validate_protocol_aggregate_coverage(report, 1)

    def test_protocol_aggregate_rejects_ns3_value_gap(self) -> None:
        report = protocol_aggregate_report()
        report["counts"]["numeric_points_compared"] = 379
        report["counts"]["missing_ns3_values"] = 1
        with self.assertRaisesRegex(
            CERTIFY.CertificationError, "structural coverage drift"
        ):
            CERTIFY.validate_protocol_aggregate_coverage(report, 1)

    def test_protocol_aggregate_exit_must_match_numeric_result(self) -> None:
        report = protocol_aggregate_report()
        with self.assertRaisesRegex(
            CERTIFY.CertificationError, "schema/status/exit drift"
        ):
            CERTIFY.validate_protocol_aggregate_coverage(report, 0)

    def test_release_status_depends_on_both_aggregate_comparators(self) -> None:
        self.assertEqual(
            CERTIFY.release_status_from_aggregate_codes((0, 0)),
            "passed_with_scoped_exemptions",
        )
        self.assertEqual(
            CERTIFY.release_status_from_aggregate_codes((0, 1)),
            "passed_with_scoped_exemptions_and_observed_aggregate_residual",
        )
        self.assertEqual(
            CERTIFY.release_status_from_aggregate_codes((1, 0)),
            "passed_with_scoped_exemptions_and_observed_aggregate_residual",
        )
        with self.assertRaisesRegex(
            CERTIFY.CertificationError, "exit-code ledger is malformed"
        ):
            CERTIFY.release_status_from_aggregate_codes((0, 2))

    def test_checksum_snapshot_rejects_late_nested_artifact(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-seal-probe-") as temporary:
            output = Path(temporary)
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir()
            manifests.mkdir()
            (artifacts / "evidence.txt").write_text("evidence\n", encoding="utf-8")
            (manifests / "inputs.json").write_text("{}\n", encoding="utf-8")
            (manifests / ".lock-ns3_linux_build").write_text(
                "lock\n", encoding="utf-8"
            )
            (output / "release-manifest.json").write_text("{}\n", encoding="utf-8")
            files = CERTIFY.enumerate_sealed_evidence(
                output, (artifacts, manifests)
            )
            sealed_hashes = {path: CERTIFY.sha256(path) for path in files}
            CERTIFY.validate_sealed_evidence_snapshot(
                output, (artifacts, manifests), sealed_hashes
            )
            original_sha256 = CERTIFY.sha256
            injected = False

            def sha256_with_late_artifact(path: Path) -> str:
                nonlocal injected
                digest = original_sha256(path)
                if not injected:
                    (artifacts / "late.txt").write_text(
                        "late\n", encoding="utf-8"
                    )
                    injected = True
                return digest

            with mock.patch.object(
                CERTIFY, "sha256", side_effect=sha256_with_late_artifact
            ):
                with self.assertRaisesRegex(
                    CERTIFY.CertificationError, "file set changed"
                ):
                    CERTIFY.validate_sealed_evidence_snapshot(
                        output, (artifacts, manifests), sealed_hashes
                    )

    def test_completed_seal_rechecks_after_final_root_scan(self) -> None:
        for mutation in ("nested_file", "content_replacement"):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory(
                prefix="csr-completed-seal-race-probe-"
            ) as temporary:
                output = Path(temporary)
                artifacts = output / "artifacts"
                manifests = output / "manifests"
                artifacts.mkdir()
                manifests.mkdir()
                evidence = artifacts / "evidence.txt"
                evidence.write_text("evidence\n", encoding="utf-8")
                (manifests / ".lock-ns3_linux_build").write_text(
                    "lock\n", encoding="utf-8"
                )
                (output / "release-manifest.json").write_text(
                    "{}\n", encoding="utf-8"
                )
                sums = output / "SHA256SUMS"
                sidecar = output / "SHA256SUMS.sha256"
                sums.write_text("sealed\n", encoding="utf-8")
                sidecar.write_text("sidecar\n", encoding="utf-8")
                files = CERTIFY.enumerate_sealed_evidence(
                    output, (artifacts, manifests)
                )
                sealed_hashes = {
                    path: CERTIFY.sha256(path)
                    for path in [*files, sums, sidecar]
                }
                original_iterdir = Path.iterdir
                output_scan_count = 0

                def iterdir_with_late_mutation(path: Path):
                    nonlocal output_scan_count
                    entries = list(original_iterdir(path))
                    if path == output:
                        output_scan_count += 1
                        if output_scan_count == 2:
                            if mutation == "nested_file":
                                (artifacts / "late.txt").write_text(
                                    "late\n", encoding="utf-8"
                                )
                            else:
                                evidence.write_text(
                                    "replaced\n", encoding="utf-8"
                                )
                    return iter(entries)

                with mock.patch.object(
                    Path, "iterdir", new=iterdir_with_late_mutation
                ):
                    with self.assertRaisesRegex(
                        CERTIFY.CertificationError,
                        "frozen evidence (file set|content) changed",
                    ):
                        CERTIFY.validate_sealed_evidence_snapshot(
                            output,
                            (artifacts, manifests),
                            sealed_hashes,
                            additional_files=(sums, sidecar),
                        )

    def test_monitored_seal_rejects_mutation_after_final_hash(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="csr-monitored-seal-race-probe-"
        ) as temporary:
            output = Path(temporary)
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir()
            manifests.mkdir()
            evidence = artifacts / "evidence.txt"
            evidence.write_text("evidence\n", encoding="utf-8")
            (manifests / ".lock-ns3_linux_build").write_text(
                "lock\n", encoding="utf-8"
            )
            (output / "release-manifest.json").write_text(
                "{}\n", encoding="utf-8"
            )
            sums = output / "SHA256SUMS"
            sidecar = output / "SHA256SUMS.sha256"
            sums.write_text("sealed\n", encoding="utf-8")
            sidecar.write_text("sidecar\n", encoding="utf-8")
            files = CERTIFY.enumerate_sealed_evidence(
                output, (artifacts, manifests)
            )
            sealed_hashes = {
                path: CERTIFY.sha256(path)
                for path in [*files, sums, sidecar]
            }
            original_sha256 = CERTIFY.sha256
            hash_call_count = 0
            final_hash_call = 2 * len(sealed_hashes)

            def sha256_with_final_mutation(path: Path) -> str:
                nonlocal hash_call_count
                digest = original_sha256(path)
                hash_call_count += 1
                if hash_call_count == final_hash_call:
                    evidence.write_text("replaced\n", encoding="utf-8")
                return digest

            with mock.patch.object(
                CERTIFY, "sha256", side_effect=sha256_with_final_mutation
            ):
                with self.assertRaisesRegex(
                    CERTIFY.CertificationError,
                    "mutated during the final monitored sealing interval",
                ):
                    CERTIFY.validate_quiescent_sealed_evidence_snapshot(
                        output,
                        (artifacts, manifests),
                        sealed_hashes,
                        additional_files=(sums, sidecar),
                    )

    def test_monitored_seal_rejects_external_hardlink_mutation(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="csr-hardlink-seal-race-probe-"
        ) as temporary:
            root = Path(temporary)
            output = root / "output"
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir(parents=True)
            manifests.mkdir()
            evidence = artifacts / "evidence.txt"
            evidence.write_text("evidence\n", encoding="utf-8")
            (manifests / ".lock-ns3_linux_build").write_text(
                "lock\n", encoding="utf-8"
            )
            (output / "release-manifest.json").write_text(
                "{}\n", encoding="utf-8"
            )
            sums = output / "SHA256SUMS"
            sidecar = output / "SHA256SUMS.sha256"
            sums.write_text("sealed\n", encoding="utf-8")
            sidecar.write_text("sidecar\n", encoding="utf-8")
            files = CERTIFY.enumerate_sealed_evidence(
                output, (artifacts, manifests)
            )
            sealed_hashes = {
                path: CERTIFY.sha256(path)
                for path in [*files, sums, sidecar]
            }
            original_sha256 = CERTIFY.sha256
            hash_call_count = 0
            final_hash_call = 2 * len(sealed_hashes)
            outside_link = root / "outside-evidence-link"

            def sha256_with_hardlink_mutation(path: Path) -> str:
                nonlocal hash_call_count
                digest = original_sha256(path)
                hash_call_count += 1
                if hash_call_count == final_hash_call:
                    os.link(evidence, outside_link)
                    outside_link.write_text("replaced\n", encoding="utf-8")
                return digest

            with mock.patch.object(
                CERTIFY, "sha256", side_effect=sha256_with_hardlink_mutation
            ):
                with self.assertRaisesRegex(
                    CERTIFY.CertificationError,
                    "mutated during the final monitored sealing interval",
                ):
                    CERTIFY.validate_quiescent_sealed_evidence_snapshot(
                        output,
                        (artifacts, manifests),
                        sealed_hashes,
                        additional_files=(sums, sidecar),
                    )

    def test_seal_rejects_preexisting_hardlinked_evidence(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="csr-hardlink-seal-probe-"
        ) as temporary:
            root = Path(temporary)
            output = root / "output"
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir(parents=True)
            manifests.mkdir()
            evidence = artifacts / "evidence.txt"
            evidence.write_text("evidence\n", encoding="utf-8")
            os.link(evidence, root / "outside-evidence-link")
            (manifests / ".lock-ns3_linux_build").write_text(
                "lock\n", encoding="utf-8"
            )
            (output / "release-manifest.json").write_text(
                "{}\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(
                CERTIFY.CertificationError, "unsafe hard-link count"
            ):
                CERTIFY.enumerate_sealed_evidence(
                    output, (artifacts, manifests)
                )

    def test_seal_rejects_unexpected_hidden_artifact(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-hidden-seal-probe-") as temporary:
            output = Path(temporary)
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir()
            manifests.mkdir()
            (manifests / ".lock-ns3_linux_build").write_text(
                "lock\n", encoding="utf-8"
            )
            (artifacts / ".partial-transfer").write_text(
                "undeclared\n", encoding="utf-8"
            )
            (output / "release-manifest.json").write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(
                CERTIFY.CertificationError,
                "unexpected hidden entry",
            ):
                CERTIFY.enumerate_sealed_evidence(output, (artifacts, manifests))

    def test_seal_requires_exact_frozen_ns3_build_lock(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-missing-lock-probe-") as temporary:
            output = Path(temporary)
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir()
            manifests.mkdir()
            (output / "release-manifest.json").write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(
                CERTIFY.CertificationError,
                "required frozen ns-3 build lock is absent or invalid",
            ):
                CERTIFY.enumerate_sealed_evidence(output, (artifacts, manifests))

    def test_seal_accepts_only_exact_harness_lock_hidden_path(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-lock-seal-probe-") as temporary:
            output = Path(temporary)
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir()
            manifests.mkdir()
            lock = manifests / ".lock-ns3_linux_build"
            lock.write_text("lock\n", encoding="utf-8")
            (output / "release-manifest.json").write_text("{}\n", encoding="utf-8")
            files = CERTIFY.enumerate_sealed_evidence(
                output, (artifacts, manifests)
            )
            self.assertIn(lock, files)

    def test_seal_rejects_directory_named_like_harness_lock(self) -> None:
        with tempfile.TemporaryDirectory(prefix="csr-lock-dir-probe-") as temporary:
            output = Path(temporary)
            artifacts = output / "artifacts"
            manifests = output / "manifests"
            artifacts.mkdir()
            manifests.mkdir()
            (manifests / ".lock-ns3_linux_build").mkdir()
            (output / "release-manifest.json").write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(
                CERTIFY.CertificationError,
                "required frozen ns-3 build lock is absent or invalid",
            ):
                CERTIFY.enumerate_sealed_evidence(output, (artifacts, manifests))

    def test_ns3_build_lock_binding_is_byte_exact_and_fail_closed(self) -> None:
        for fault in ("source_changed", "frozen_missing", "frozen_directory"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory(
                prefix="csr-lock-binding-probe-"
            ) as temporary:
                root = Path(temporary)
                output = root / "output"
                manifests = output / "manifests"
                manifests.mkdir(parents=True)
                source = root / ".lock-ns3_linux_build"
                frozen = manifests / source.name
                source.write_text(
                    "ns3_runnable_programs = ['build/scratch/probe']\n",
                    encoding="utf-8",
                )
                frozen.write_bytes(source.read_bytes())
                record = {
                    "source_path": str(source.resolve()),
                    "frozen_artifact": "manifests/.lock-ns3_linux_build",
                    "sha256": CERTIFY.sha256(source),
                    "size_bytes": source.stat().st_size,
                }
                self.assertEqual(
                    CERTIFY.validate_ns3_lock_binding(record, output),
                    (source, frozen),
                )
                if fault == "source_changed":
                    source.write_text("changed\n", encoding="utf-8")
                elif fault == "frozen_missing":
                    frozen.unlink()
                else:
                    frozen.unlink()
                    frozen.mkdir()
                with self.assertRaisesRegex(
                    CERTIFY.CertificationError,
                    "ns-3 build lock or frozen binding changed",
                ):
                    CERTIFY.validate_ns3_lock_binding(record, output)


if __name__ == "__main__":
    unittest.main(verbosity=2)
