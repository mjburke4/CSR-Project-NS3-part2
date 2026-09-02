#!/usr/bin/env python3
"""Validate and summarize the source-ordered CSR admission-state ledger."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable, Iterator


TRACE_SCHEMA = "csr-differential-trace-v1"
REPORT_SCHEMA = "csr-admission-ledger-analysis-v1"
APP_DIAGNOSTICS_SCHEMA = "csr-app-admission-diagnostics-v1"
OPNET_TIC_SECONDS = 1.0 / 36.0e6
LEDGER_EVENTS = {
    "app_admission",
    "nwk_admission",
    "hop_admission",
    "hop_feedback",
    "nwk_nsdp_release",
    "hop_completion",
    "hop_capacity_release",
}
OBSERVED_EVENTS = LEDGER_EVENTS | {"app_send", "nwk_delivery"}
REQUIRED_COLUMNS = {
    "schema",
    "event_index",
    "time_s",
    "event",
    "node",
    "peer",
    "src",
    "dst",
    "sequence",
    "reason",
    "next_hop",
    "detail",
}
APP_REASONS = {
    "admitted": "admitted",
    "discovery_active": "blocked_discovery",
    "discovery": "blocked_discovery",
    "blocked_discovery": "blocked_discovery",
    "topology_unknown": "blocked_topology",
    "topology": "blocked_topology",
    "blocked_topology": "blocked_topology",
    "gateway_route_unknown": "blocked_gateway_route",
    "gateway_route": "blocked_gateway_route",
    "blocked_gateway_route": "blocked_gateway_route",
    "destination_unavailable": "blocked_destination",
    "destination": "blocked_destination",
    "blocked_destination": "blocked_destination",
    "nsdp_full": "blocked_nsdp",
    "nsdp": "blocked_nsdp",
    "blocked_nsdp": "blocked_nsdp",
}
NWK_REASONS = {
    "admitted": "admitted",
    "no_route": "no_route",
    "blocked_no_route": "no_route",
    "global_capacity": "global_capacity",
    "global_hop_capacity": "global_capacity",
    "global_hop_full": "global_capacity",
    "blocked_global_capacity": "global_capacity",
    "neighbor_capacity": "neighbor_capacity",
    "neighbor_flow_control": "neighbor_capacity",
    "neighbor_flow_full": "neighbor_capacity",
    "blocked_neighbor_capacity": "neighbor_capacity",
}
COMPLETION_REASONS = {
    "ack": "ack",
    "dack": "dack",
    "no_ack": "no_ack",
    "no-ack": "no_ack",
    "timeout": "no_ack",
}
LEG_CSV_COLUMNS = (
    "src",
    "dst",
    "sequence",
    "leg_index",
    "node",
    "next_hop",
    "hop_sequence",
    "nwk_admission_time_s",
    "hop_admission_time_s",
    "feedback_time_s",
    "feedback_reason",
    "feedback_missing",
    "feedback_completion_mismatch",
    "nsdp_release_time_s",
    "completion_time_s",
    "completion_reason",
    "capacity_release_time_s",
    "dack_hold_seconds",
    "dack_scheduled_timer_offset_seconds",
    "dack_effective_timer_offset_seconds",
    "resend_count",
    "resend_overflow",
    "no_route_holds",
    "global_capacity_holds",
    "neighbor_capacity_holds",
    "status",
    "valid",
    "issues_json",
)


class AdmissionAnalysisError(ValueError):
    """The trace or requested outputs cannot be interpreted safely."""


@dataclass(frozen=True, order=True)
class PacketKey:
    source: int
    destination: int
    sequence: int


@dataclass(frozen=True)
class Event:
    index: int
    time_s: float
    name: str
    node: int | None
    peer: int | None
    next_hop: int | None
    source: int | None
    destination: int | None
    packet: PacketKey | None
    reason: str
    detail: dict[str, str]
    input_row: int


@dataclass
class AppAttempt:
    flow_index: int
    attempt_index: int
    event: Event
    decision: str
    nsdp_count: int | None
    nsdp_limit: int | None


@dataclass
class Leg:
    packet: PacketKey
    leg_index: int
    node: int
    next_hop: int | None
    first_event_index: int
    nwk_admission_time_s: float | None = None
    hop_admission_time_s: float | None = None
    hop_sequence: int | None = None
    feedback_time_s: float | None = None
    feedback_reason: str = ""
    feedback_missing: bool = False
    feedback_completion_mismatch: bool = False
    nsdp_release_time_s: float | None = None
    completion_time_s: float | None = None
    completion_reason: str = ""
    capacity_release_time_s: float | None = None
    dack_hold_seconds: float | None = None
    dack_scheduled_timer_offset_seconds: float | None = None
    dack_effective_timer_offset_seconds: float | None = None
    capacity_release_event: Event | None = None
    resend_count: int | None = None
    resend_overflow: bool = False
    holds: Counter[str] = field(default_factory=Counter)
    issues: list[dict[str, object]] = field(default_factory=list)


@dataclass
class AnalysisState:
    event_counts: Counter[str] = field(default_factory=Counter)
    app_counts: Counter[str] = field(default_factory=Counter)
    nwk_counts: Counter[str] = field(default_factory=Counter)
    feedback_counts: Counter[str] = field(default_factory=Counter)
    feedback_dack_transitions: Counter[tuple[int, int]] = field(
        default_factory=Counter
    )
    feedback_dack_pre_limit_count: int = 0
    feedback_dack_boundary_count: int = 0
    uncorrelated_feedback_count: int = 0
    uncorrelated_tagged_feedback_count: int = 0
    uncorrelated_tagless_feedback_count: int = 0
    late_feedback_after_no_ack_count: int = 0
    missing_feedback_count: int = 0
    feedback_completion_mismatch_count: int = 0
    completion_counts: Counter[str] = field(default_factory=Counter)
    app_by_flow: dict[int, Counter[str]] = field(
        default_factory=lambda: defaultdict(Counter)
    )
    nwk_by_node: dict[int, Counter[str]] = field(
        default_factory=lambda: defaultdict(Counter)
    )
    nwk_holds_by_location: dict[tuple[int, int | None], Counter[str]] = field(
        default_factory=lambda: defaultdict(Counter)
    )
    last_attempt: dict[int, int] = field(default_factory=dict)
    first_admitted_s: dict[int, float] = field(default_factory=dict)
    last_admitted_s: dict[int, float] = field(default_factory=dict)
    flow_source: dict[int, int] = field(default_factory=dict)
    admitted_packets: dict[PacketKey, AppAttempt] = field(default_factory=dict)
    app_sends: Counter[PacketKey] = field(default_factory=Counter)
    deliveries: Counter[PacketKey] = field(default_factory=Counter)
    legs: list[Leg] = field(default_factory=list)
    leg_counts: Counter[PacketKey] = field(default_factory=Counter)
    pending_nwk_leg: dict[tuple[PacketKey, int], list[Leg]] = field(
        default_factory=lambda: defaultdict(list)
    )
    active_hop_leg: dict[tuple[int, int, int], list[Leg]] = field(
        default_factory=lambda: defaultdict(list)
    )
    active_packet_hop_leg: dict[
        tuple[PacketKey, int, int, int], list[Leg]
    ] = field(default_factory=lambda: defaultdict(list))
    active_packet_leg: dict[tuple[PacketKey, int], list[Leg]] = field(
        default_factory=lambda: defaultdict(list)
    )
    dack_hold_packet_hop_leg: dict[
        tuple[PacketKey, int, int, int], list[Leg]
    ] = field(default_factory=lambda: defaultdict(list))
    dack_hold_packet_leg: dict[tuple[PacketKey, int], list[Leg]] = field(
        default_factory=lambda: defaultdict(list)
    )
    known_hop_leg: dict[
        tuple[PacketKey, int, int, int], list[Leg]
    ] = field(default_factory=lambda: defaultdict(list))
    held: dict[tuple[PacketKey, int], Counter[str]] = field(
        default_factory=lambda: defaultdict(Counter)
    )
    pending_nsdp_release: Event | None = None
    global_issues: list[dict[str, object]] = field(default_factory=list)
    maxima: dict[str, int] = field(default_factory=dict)


@dataclass
class TraceStats:
    row_count: int = 0
    event_counts: Counter[str] = field(default_factory=Counter)
    ledger_rows: int = 0


def _paths_alias(first: Path, second: Path) -> bool:
    if first == second:
        return True
    try:
        return first.samefile(second)
    except OSError:
        return False


def _sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_paths(
    trace: Path,
    diagnostics: Path | None,
    summary: Path,
    legs: Path,
) -> None:
    paths = [("trace", trace), ("summary JSON", summary), ("legs CSV", legs)]
    if diagnostics is not None:
        paths.append(("application diagnostics", diagnostics))
    for index, (first_name, first) in enumerate(paths):
        for second_name, second in paths[index + 1 :]:
            if _paths_alias(first, second):
                raise AdmissionAnalysisError(
                    f"{first_name} must not alias {second_name}"
                )


def _integer(text: str | None, label: str, row: int) -> int:
    value = (text or "").strip()
    if not re.fullmatch(r"[0-9]+", value):
        raise AdmissionAnalysisError(
            f"row {row}: {label} must be a non-negative integer"
        )
    return int(value)


def _signed_integer(text: str | None, label: str, row: int) -> int:
    value = (text or "").strip()
    if not re.fullmatch(r"-?[0-9]+", value):
        raise AdmissionAnalysisError(f"row {row}: {label} must be an integer")
    return int(value)


def _optional_integer(text: str | None, label: str, row: int) -> int | None:
    value = (text or "").strip()
    return None if not value else _integer(value, label, row)


def _number(text: str | None, label: str, row: int) -> float:
    value = (text or "").strip()
    try:
        parsed = float(value)
    except ValueError as error:
        raise AdmissionAnalysisError(f"row {row}: {label} is not numeric") from error
    if not math.isfinite(parsed) or parsed < 0.0:
        raise AdmissionAnalysisError(
            f"row {row}: {label} must be finite and non-negative"
        )
    return parsed


def _boolean(text: str | None, label: str, row: int) -> bool:
    value = (text or "").strip().lower()
    if value in {"1", "true", "yes"}:
        return True
    if value in {"0", "false", "no"}:
        return False
    raise AdmissionAnalysisError(f"row {row}: {label} must be boolean 0 or 1")


def _detail(text: str | None, row: int) -> dict[str, str]:
    raw = (text or "").strip()
    if not raw:
        return {}
    parsed: dict[str, str] = {}
    for token in raw.split(";"):
        if not token or "=" not in token:
            raise AdmissionAnalysisError(
                f"row {row}: detail must contain semicolon-separated key=value pairs"
            )
        key, value = token.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key or not re.fullmatch(r"[a-z][a-z0-9_]*", key):
            raise AdmissionAnalysisError(f"row {row}: invalid detail key {key!r}")
        if key in parsed:
            raise AdmissionAnalysisError(
                f"row {row}: duplicate detail key {key!r}"
            )
        if not value:
            raise AdmissionAnalysisError(
                f"row {row}: detail value for {key!r} is empty"
            )
        parsed[key] = value
    return parsed


def _required(detail: dict[str, str], key: str, event: Event) -> str:
    if key not in detail:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} detail omits {key}"
        )
    return detail[key]


def _detail_int(detail: dict[str, str], key: str, event: Event) -> int:
    return _integer(_required(detail, key, event), key, event.input_row)


def _detail_signed_int(detail: dict[str, str], key: str, event: Event) -> int:
    return _signed_integer(_required(detail, key, event), key, event.input_row)


def _detail_optional_int(
    detail: dict[str, str], key: str, event: Event
) -> int | None:
    return (
        _integer(detail[key], key, event.input_row) if key in detail else None
    )


def _detail_bool(detail: dict[str, str], key: str, event: Event) -> bool:
    return _boolean(_required(detail, key, event), key, event.input_row)


def _packet_from_row(row: dict[str, str], input_row: int) -> PacketKey | None:
    values = [(row.get(name) or "").strip() for name in ("src", "dst", "sequence")]
    if not any(values):
        return None
    if not values[0] or not values[1]:
        raise AdmissionAnalysisError(
            f"row {input_row}: flow identity must provide both src and dst"
        )
    if not values[2]:
        return None
    return PacketKey(
        _integer(values[0], "src", input_row),
        _integer(values[1], "dst", input_row),
        _integer(values[2], "sequence", input_row),
    )


def iter_events(path: Path, stats: TraceStats) -> Iterator[Event]:
    try:
        stream = path.open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise AdmissionAnalysisError(str(error)) from error
    previous_index: int | None = None
    previous_time: float | None = None
    with stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise AdmissionAnalysisError(f"{path}: trace has no CSV header")
        duplicates = [
            key for key, count in Counter(reader.fieldnames).items() if count > 1
        ]
        if duplicates:
            raise AdmissionAnalysisError(
                f"{path}: duplicate CSV column(s): {', '.join(duplicates)}"
            )
        missing = REQUIRED_COLUMNS - set(reader.fieldnames)
        if missing:
            raise AdmissionAnalysisError(
                f"{path}: missing required column(s): {', '.join(sorted(missing))}"
            )
        for input_row, row in enumerate(reader, start=2):
            stats.row_count += 1
            if None in row or any(value is None for value in row.values()):
                raise AdmissionAnalysisError(f"row {input_row}: malformed CSV row")
            if (row.get("schema") or "").strip() != TRACE_SCHEMA:
                raise AdmissionAnalysisError(
                    f"row {input_row}: unsupported trace schema"
                )
            event_index = _integer(row.get("event_index"), "event_index", input_row)
            time_s = _number(row.get("time_s"), "time_s", input_row)
            if previous_index is not None and event_index <= previous_index:
                raise AdmissionAnalysisError(
                    f"row {input_row}: event_index is not strictly increasing"
                )
            if previous_time is not None and time_s < previous_time:
                raise AdmissionAnalysisError(
                    f"row {input_row}: time_s precedes the prior source event"
                )
            previous_index = event_index
            previous_time = time_s
            name = (row.get("event") or "").strip()
            if not name:
                raise AdmissionAnalysisError(f"row {input_row}: event is empty")
            stats.event_counts[name] += 1
            if name not in OBSERVED_EVENTS:
                continue
            event = Event(
                index=event_index,
                time_s=time_s,
                name=name,
                node=_optional_integer(row.get("node"), "node", input_row),
                peer=_optional_integer(row.get("peer"), "peer", input_row),
                next_hop=_optional_integer(row.get("next_hop"), "next_hop", input_row),
                source=_optional_integer(row.get("src"), "src", input_row),
                destination=_optional_integer(row.get("dst"), "dst", input_row),
                packet=_packet_from_row(row, input_row),
                reason=(row.get("reason") or "").strip().lower(),
                detail=_detail(row.get("detail"), input_row) if name in LEDGER_EVENTS else {},
                input_row=input_row,
            )
            if name in LEDGER_EVENTS:
                stats.ledger_rows += 1
            yield event
    if stats.row_count == 0:
        raise AdmissionAnalysisError(f"{path}: trace has no event rows")
    if stats.ledger_rows == 0:
        raise AdmissionAnalysisError(f"{path}: trace has no admission-ledger events")


def _issue(
    target: list[dict[str, object]],
    code: str,
    message: str,
    events: Iterable[Event] = (),
) -> None:
    target.append(
        {
            "code": code,
            "message": message,
            "event_indexes": [event.index for event in events],
        }
    )


def _record_maximum(state: AnalysisState, key: str, value: int) -> None:
    state.maxima[key] = max(state.maxima.get(key, 0), value)


def _require_packet(event: Event) -> PacketKey:
    if event.packet is None:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} requires packet identity"
        )
    return event.packet


def _require_node(event: Event) -> int:
    if event.node is None:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} requires node"
        )
    return event.node


def _require_directed_leg(event: Event) -> tuple[int, int]:
    if event.peer is None or event.next_hop is None:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} requires directed-leg peer "
            "and next_hop"
        )
    return event.peer, event.next_hop


def _normalize(reason: str, choices: dict[str, str], event: Event) -> str:
    if reason not in choices:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: unsupported {event.name} reason {reason!r}"
        )
    return choices[reason]


def _handle_app(event: Event, state: AnalysisState) -> None:
    detail = event.detail
    flow = _detail_int(detail, "flow_index", event)
    attempt = _detail_int(detail, "attempt_index", event)
    _detail_int(detail, "configured_destination", event)
    discovery = _detail_bool(detail, "discovery_active", event)
    topology = _detail_bool(detail, "topology_known", event)
    gateway_cached = _detail_bool(detail, "gateway_cached", event)
    route_check = _detail_bool(detail, "route_check_performed", event)
    route_available = _detail_bool(detail, "route_available", event)
    _detail_int(detail, "nwk_queue", event)
    decision = _normalize(event.reason, APP_REASONS, event)
    expected = state.last_attempt.get(flow, 0) + 1
    if attempt != expected:
        _issue(
            state.global_issues,
            "noncontiguous_attempt",
            f"flow {flow} attempt {attempt} follows {expected - 1}",
            [event],
        )
    state.last_attempt[flow] = attempt
    if event.node is None:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: app_admission requires node"
        )
    if flow in state.flow_source and state.flow_source[flow] != event.node:
        _issue(
            state.global_issues,
            "flow_source_changed",
            f"flow {flow} changed source node",
            [event],
        )
    state.flow_source[flow] = event.node

    nsdp_count = _detail_optional_int(detail, "nsdp_count", event)
    nsdp_limit = _detail_optional_int(detail, "nsdp_limit", event)
    if (nsdp_count is None) != (nsdp_limit is None):
        raise AdmissionAnalysisError(
            f"row {event.input_row}: NSDP count and limit must appear together"
        )
    if nsdp_limit == 0:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: application nsdp_limit must be positive"
        )
    if decision in {"admitted", "blocked_nsdp"} and nsdp_count is None:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {decision} requires NSDP state"
        )
    if decision == "blocked_discovery" and not discovery:
        _issue(state.global_issues, "bad_discovery_gate", "discovery block has inactive flag", [event])
    if decision != "blocked_discovery" and discovery:
        _issue(state.global_issues, "gate_priority_violation", "later gate evaluated while discovery is active", [event])
    if decision == "blocked_topology" and topology:
        _issue(state.global_issues, "bad_topology_gate", "topology block has known topology", [event])
    if decision not in {"blocked_discovery", "blocked_topology"} and not topology:
        _issue(state.global_issues, "gate_priority_violation", "later gate evaluated without topology", [event])
    if not route_check and route_available:
        _issue(
            state.global_issues,
            "route_result_without_check",
            "route_available is set although no route check was performed",
            [event],
        )
    if decision == "blocked_gateway_route":
        if not route_check:
            _issue(state.global_issues, "bad_route_gate", "gateway-route block skipped route check", [event])
        if route_available:
            _issue(state.global_issues, "bad_route_gate", "gateway-route block reports route available", [event])
    if decision in {"admitted", "blocked_nsdp"} and route_check:
        if not route_available:
            _issue(
                state.global_issues,
                "bad_route_gate",
                "later admission gate follows an unsuccessful route check",
                [event],
            )
        if not gateway_cached:
            _issue(
                state.global_issues,
                "bad_gateway_snapshot",
                "successful route check did not update the gateway cache snapshot",
                [event],
            )
    if decision in {"admitted", "blocked_nsdp"}:
        assert nsdp_count is not None and nsdp_limit is not None
        _record_maximum(state, "application_nsdp", nsdp_count)
        if decision == "admitted" and nsdp_count >= nsdp_limit:
            _issue(state.global_issues, "app_admitted_at_nsdp_limit", "application admission did not satisfy count < limit", [event])
        if decision == "blocked_nsdp" and nsdp_count < nsdp_limit:
            _issue(state.global_issues, "bad_nsdp_gate", "NSDP block reports capacity available", [event])

    attempt_record = AppAttempt(flow, attempt, event, decision, nsdp_count, nsdp_limit)
    state.app_counts[decision] += 1
    state.app_by_flow[flow][decision] += 1
    if decision == "admitted":
        state.first_admitted_s.setdefault(flow, event.time_s)
        state.last_admitted_s[flow] = event.time_s
        packet = _require_packet(event)
        if packet.source != event.node:
            _issue(state.global_issues, "app_source_mismatch", "packet source differs from application node", [event])
        if packet in state.admitted_packets:
            _issue(state.global_issues, "duplicate_admitted_packet", "packet belongs to multiple admitted attempts", [event])
        state.admitted_packets[packet] = attempt_record
    elif event.packet is not None:
        _issue(state.global_issues, "blocked_attempt_has_packet", "blocked attempt must not allocate packet identity", [event])


def _handle_nwk(event: Event, state: AnalysisState) -> None:
    packet = _require_packet(event)
    node = _require_node(event)
    decision = _normalize(event.reason, NWK_REASONS, event)
    detail = event.detail
    queue_before = _detail_int(detail, "queue_before", event)
    queue_after = _detail_int(detail, "queue_after", event)
    nsdp_count = _detail_int(detail, "nsdp_count", event)
    nsdp_limit = _detail_int(detail, "nsdp_limit", event)
    pending = _detail_int(detail, "pending", event)
    pending_limit = _detail_int(detail, "pending_limit", event)
    global_spad = _detail_signed_int(detail, "global_spad", event)
    route_known = (
        _boolean(detail["route_known"], "route_known", event.input_row)
        if "route_known" in detail
        else None
    )
    scan_stopped = (
        _boolean(detail["scan_stopped"], "scan_stopped", event.input_row)
        if "scan_stopped" in detail
        else None
    )
    _record_maximum(state, "nwk_queue", max(queue_before, queue_after))
    _record_maximum(state, "nwk_nsdp", nsdp_count)
    _record_maximum(state, "hop_pending", pending)
    if nsdp_limit == 0 or pending_limit == 0:
        raise AdmissionAnalysisError(f"row {event.input_row}: limits must be positive")
    expected_global_spad = pending_limit - pending + 1
    if global_spad != expected_global_spad:
        _issue(
            state.global_issues,
            "bad_global_spad",
            "global_spad does not match pending_limit-pending+1",
            [event],
        )

    if decision == "admitted":
        if route_known is not True:
            _issue(state.global_issues, "bad_route_known", "admitted NWK row must report route_known=1", [event])
        if event.next_hop is None:
            raise AdmissionAnalysisError(
                f"row {event.input_row}: admitted nwk_admission requires next_hop"
            )
        outstanding = _detail_int(detail, "outstanding", event)
        threshold = _detail_int(detail, "threshold", event)
        neighbor_spad = _detail_signed_int(detail, "neighbor_spad", event)
        _record_maximum(state, "hop_outstanding", outstanding)
        if queue_before < 1 or queue_after != queue_before - 1:
            _issue(state.global_issues, "bad_nwk_removal", "admission must remove exactly one NWK packet", [event])
        if pending > pending_limit or global_spad <= 0:
            _issue(state.global_issues, "nwk_admitted_without_global_capacity", "NWK admitted with exhausted global HOP capacity", [event])
        if neighbor_spad != threshold - outstanding + 1:
            _issue(state.global_issues, "bad_neighbor_spad", "neighbor_spad does not match threshold-outstanding+1", [event])
        if outstanding > threshold or neighbor_spad <= 0:
            _issue(state.global_issues, "nwk_admitted_without_neighbor_capacity", "NWK admitted with exhausted neighbor capacity", [event])
        leg_index = state.leg_counts[packet]
        state.leg_counts[packet] += 1
        leg = Leg(packet, leg_index, node, event.next_hop, event.index)
        leg.nwk_admission_time_s = event.time_s
        leg.holds.update(state.held.pop((packet, node), Counter()))
        state.legs.append(leg)
        state.pending_nwk_leg[(packet, node)].append(leg)
    else:
        if queue_after != queue_before:
            _issue(state.global_issues, "blocked_nwk_mutated_queue", "blocked NWK decision changed queue size", [event])
        state.held[(packet, node)][decision] += 1
        state.nwk_holds_by_location[(node, event.next_hop)][decision] += 1
        if decision == "global_capacity":
            if pending <= pending_limit or global_spad > 0:
                _issue(state.global_issues, "bad_global_capacity_gate", "global block reports available capacity", [event])
            if scan_stopped is not None:
                if not scan_stopped:
                    _issue(state.global_issues, "bad_scan_stop_marker", "top-level global-capacity row must report scan_stopped=1", [event])
            else:
                if route_known is not True or event.next_hop is None:
                    _issue(state.global_issues, "bad_global_context", "per-entry global-capacity row requires route_known=1 and next_hop", [event])
                outstanding = _detail_int(detail, "outstanding", event)
                threshold = _detail_int(detail, "threshold", event)
                neighbor_spad = _detail_signed_int(detail, "neighbor_spad", event)
                _record_maximum(state, "hop_outstanding", outstanding)
                if neighbor_spad != threshold - outstanding + 1:
                    _issue(state.global_issues, "bad_neighbor_spad", "per-entry global-capacity neighbor_spad formula mismatch", [event])
        elif decision == "no_route":
            if route_known is not False:
                _issue(state.global_issues, "bad_route_known", "no-route NWK row must report route_known=0", [event])
            if event.next_hop is not None:
                _issue(state.global_issues, "no_route_has_next_hop", "no-route block reports a next hop", [event])
        elif decision == "neighbor_capacity":
            if route_known is not True:
                _issue(state.global_issues, "bad_route_known", "neighbor-capacity NWK row must report route_known=1", [event])
            if event.next_hop is None:
                raise AdmissionAnalysisError(
                    f"row {event.input_row}: neighbor-capacity block requires next_hop"
                )
            outstanding = _detail_int(detail, "outstanding", event)
            threshold = _detail_int(detail, "threshold", event)
            neighbor_spad = _detail_signed_int(detail, "neighbor_spad", event)
            _record_maximum(state, "hop_outstanding", outstanding)
            if neighbor_spad != threshold - outstanding + 1:
                _issue(state.global_issues, "bad_neighbor_spad", "neighbor_spad formula mismatch", [event])
            if outstanding <= threshold or neighbor_spad > 0:
                _issue(state.global_issues, "bad_neighbor_capacity_gate", "neighbor block reports available capacity", [event])
    state.nwk_counts[decision] += 1
    state.nwk_by_node[node][decision] += 1


def _find_pending_nwk_leg(event: Event, state: AnalysisState) -> Leg:
    packet = _require_packet(event)
    node = _require_node(event)
    candidates = state.pending_nwk_leg.get((packet, node), [])
    exact = [
        leg
        for leg in candidates
        if leg.next_hop == event.next_hop
        and leg.nwk_admission_time_s == event.time_s
    ]
    if len(exact) == 1:
        return exact[0]
    if len(exact) > 1:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} has ambiguous matching NWK legs"
        )
    if not candidates:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} has no matching NWK leg"
        )

    resolutions: list[Leg] = []
    for partial in (
        [leg for leg in candidates if leg.next_hop == event.next_hop],
        [leg for leg in candidates if leg.nwk_admission_time_s == event.time_s],
    ):
        if len(partial) == 1 and not any(
            candidate is partial[0] for candidate in resolutions
        ):
            resolutions.append(partial[0])
    if len(resolutions) == 1:
        return resolutions[0]
    if len(resolutions) > 1 or len(candidates) != 1:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} has ambiguous matching NWK legs"
        )
    return candidates[0]


def _remove_pending_nwk_leg(leg: Leg, state: AnalysisState) -> None:
    key = (leg.packet, leg.node)
    candidates = state.pending_nwk_leg.get(key)
    if candidates is None:
        return
    state.pending_nwk_leg[key] = [
        candidate for candidate in candidates if candidate is not leg
    ]
    if not state.pending_nwk_leg[key]:
        del state.pending_nwk_leg[key]


def _resolve_hop_leg(
    event: Event,
    hop_sequence: int,
    exact: list[Leg],
    candidates: list[Leg],
    lifecycle: str,
) -> Leg:
    if len(exact) == 1:
        return exact[0]
    if len(exact) > 1:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} has ambiguous matching "
            f"{lifecycle} legs"
        )
    if not candidates:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} has no matching {lifecycle} leg"
        )
    resolutions: list[Leg] = []
    for partial in (
        [leg for leg in candidates if leg.hop_sequence == hop_sequence],
        [leg for leg in candidates if leg.next_hop == event.next_hop],
        [leg for leg in candidates if leg.next_hop == event.peer],
    ):
        if len(partial) == 1 and not any(
            candidate is partial[0] for candidate in resolutions
        ):
            resolutions.append(partial[0])
    if len(resolutions) == 1:
        return resolutions[0]
    if len(resolutions) > 1 or len(candidates) != 1:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} has ambiguous matching "
            f"{lifecycle} legs"
        )
    return candidates[0]


def _find_transition_hop_leg(
    event: Event, state: AnalysisState, hop_sequence: int
) -> Leg:
    packet = _require_packet(event)
    node = _require_node(event)
    if event.next_hop is None:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: {event.name} requires next_hop"
        )
    key = (packet, node, event.next_hop, hop_sequence)
    return _resolve_hop_leg(
        event,
        hop_sequence,
        (
            state.active_packet_hop_leg.get(key, [])
            + state.dack_hold_packet_hop_leg.get(key, [])
        ),
        (
            state.active_packet_leg.get((packet, node), [])
            + state.dack_hold_packet_leg.get((packet, node), [])
        ),
        "active or DACK-hold HOP",
    )


def _hop_index_key(leg: Leg) -> tuple[int, int, int] | None:
    if leg.next_hop is None or leg.hop_sequence is None:
        return None
    return (leg.node, leg.next_hop, leg.hop_sequence)


def _index_hop_leg(leg: Leg, state: AnalysisState) -> None:
    key = _hop_index_key(leg)
    if key is not None:
        packet_key = (leg.packet, key[0], key[1], key[2])
        if (
            state.active_packet_hop_leg.get(packet_key)
            or state.dack_hold_packet_hop_leg.get(packet_key)
        ):
            raise AdmissionAnalysisError(
                "active or held HOP identity is ambiguous for "
                f"packet={leg.packet}, node={leg.node}, "
                f"next_hop={leg.next_hop}, hop_sequence={leg.hop_sequence}"
            )
        state.active_hop_leg[key].append(leg)
        state.active_packet_hop_leg[packet_key].append(leg)
        state.active_packet_leg[(leg.packet, leg.node)].append(leg)
        state.known_hop_leg[packet_key].append(leg)


def _unindex_hop_leg(leg: Leg, state: AnalysisState) -> None:
    key = _hop_index_key(leg)
    if key is None:
        return
    candidates = state.active_hop_leg.get(key)
    if candidates is not None:
        state.active_hop_leg[key] = [
            candidate for candidate in candidates if candidate is not leg
        ]
        if not state.active_hop_leg[key]:
            del state.active_hop_leg[key]
    packet_key = (leg.packet, key[0], key[1], key[2])
    packet_candidates = state.active_packet_hop_leg.get(packet_key)
    if packet_candidates is not None:
        state.active_packet_hop_leg[packet_key] = [
            candidate for candidate in packet_candidates if candidate is not leg
        ]
        if not state.active_packet_hop_leg[packet_key]:
            del state.active_packet_hop_leg[packet_key]
    active_key = (leg.packet, leg.node)
    active_candidates = state.active_packet_leg.get(active_key)
    if active_candidates is not None:
        state.active_packet_leg[active_key] = [
            candidate for candidate in active_candidates if candidate is not leg
        ]
        if not state.active_packet_leg[active_key]:
            del state.active_packet_leg[active_key]


def _index_dack_hold_leg(leg: Leg, state: AnalysisState) -> None:
    key = _hop_index_key(leg)
    if key is None:
        return
    packet_key = (leg.packet, key[0], key[1], key[2])
    if state.dack_hold_packet_hop_leg.get(packet_key):
        raise AdmissionAnalysisError(
            "DACK-hold HOP identity is ambiguous for "
            f"packet={leg.packet}, node={leg.node}, "
            f"next_hop={leg.next_hop}, hop_sequence={leg.hop_sequence}"
        )
    state.dack_hold_packet_hop_leg[packet_key].append(leg)
    state.dack_hold_packet_leg[(leg.packet, leg.node)].append(leg)


def _unindex_dack_hold_leg(leg: Leg, state: AnalysisState) -> None:
    key = _hop_index_key(leg)
    if key is None:
        return
    packet_key = (leg.packet, key[0], key[1], key[2])
    packet_candidates = state.dack_hold_packet_hop_leg.get(packet_key)
    if packet_candidates is not None:
        state.dack_hold_packet_hop_leg[packet_key] = [
            candidate for candidate in packet_candidates if candidate is not leg
        ]
        if not state.dack_hold_packet_hop_leg[packet_key]:
            del state.dack_hold_packet_hop_leg[packet_key]
    active_key = (leg.packet, leg.node)
    active_candidates = state.dack_hold_packet_leg.get(active_key)
    if active_candidates is not None:
        state.dack_hold_packet_leg[active_key] = [
            candidate for candidate in active_candidates if candidate is not leg
        ]
        if not state.dack_hold_packet_leg[active_key]:
            del state.dack_hold_packet_leg[active_key]


def _handle_hop_admission(event: Event, state: AnalysisState) -> None:
    if event.reason != "admitted":
        raise AdmissionAnalysisError(
            f"row {event.input_row}: unsupported hop_admission reason {event.reason!r}"
        )
    peer, next_hop = _require_directed_leg(event)
    detail = event.detail
    hop_sequence = _detail_int(detail, "hop_sequence", event)
    leg = _find_pending_nwk_leg(event, state)
    if hop_sequence > 0xFFFF:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: hop_sequence exceeds 16 bits"
        )
    pending_before = _detail_int(detail, "pending_before", event)
    pending_after = _detail_int(detail, "pending_after", event)
    limit = _detail_int(detail, "pending_limit", event)
    global_before = _detail_signed_int(detail, "global_spad_before", event)
    global_after = _detail_signed_int(detail, "global_spad_after", event)
    outstanding_before = _detail_int(detail, "outstanding_before", event)
    outstanding_after = _detail_int(detail, "outstanding_after", event)
    threshold = _detail_int(detail, "threshold", event)
    neighbor_before = _detail_signed_int(detail, "neighbor_spad_before", event)
    neighbor_after = _detail_signed_int(detail, "neighbor_spad_after", event)
    resend_before = _detail_int(detail, "resend_queue_before", event)
    resend_after = _detail_int(detail, "resend_queue_after", event)
    tracked = _detail_bool(detail, "resend_tracked", event)
    overflow = _detail_bool(detail, "resend_overflow", event)
    if limit == 0:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: pending_limit must be positive"
        )
    if threshold > 16:
        _issue(leg.issues, "threshold_out_of_range", "neighbor threshold is outside 0..16", [event])
    if leg.hop_admission_time_s is not None:
        _issue(leg.issues, "duplicate_hop_admission", "leg has multiple HOP admissions", [event])
    if next_hop != leg.next_hop:
        _issue(leg.issues, "hop_next_hop_mismatch", "HOP admission next hop differs from NWK", [event])
    if peer != leg.next_hop:
        _issue(leg.issues, "hop_peer_mismatch", "HOP admission peer differs from NWK next hop", [event])
    if event.time_s != leg.nwk_admission_time_s:
        _issue(leg.issues, "hop_admission_time_mismatch", "NWK and HOP admissions are not source-synchronous", [event])
    if pending_after != pending_before + 1 or outstanding_after != outstanding_before + 1:
        _issue(leg.issues, "bad_hop_increment", "HOP admission must increment pending and outstanding", [event])
    if global_before != limit - pending_before + 1 or global_after != limit - pending_after + 1:
        _issue(leg.issues, "bad_hop_global_spad", "HOP global spare allowance mismatch", [event])
    if neighbor_before != threshold - outstanding_before + 1 or neighbor_after != threshold - outstanding_after + 1:
        _issue(leg.issues, "bad_hop_neighbor_spad", "HOP neighbor spare allowance mismatch", [event])
    if pending_before > limit or outstanding_before > threshold:
        _issue(leg.issues, "hop_admitted_without_capacity", "HOP admission began beyond its allowed boundary", [event])
    if tracked == overflow:
        _issue(leg.issues, "bad_resend_status", "resend_tracked must be the inverse of resend_overflow", [event])
    expected_resend_after = resend_before + (1 if tracked else 0)
    if resend_after != expected_resend_after:
        _issue(leg.issues, "bad_resend_queue_admission", "resend queue transition does not match tracking status", [event])
    leg.hop_admission_time_s = event.time_s
    leg.hop_sequence = hop_sequence
    leg.resend_overflow = overflow
    _remove_pending_nwk_leg(leg, state)
    _index_hop_leg(leg, state)
    _record_maximum(state, "hop_pending", pending_after)
    _record_maximum(state, "hop_outstanding", outstanding_after)
    _record_maximum(state, "hop_resend_queue", max(resend_before, resend_after))


def _note_uncorrelated_feedback(event: Event, state: AnalysisState) -> None:
    state.uncorrelated_feedback_count += 1
    if event.packet is None:
        state.uncorrelated_tagless_feedback_count += 1
    else:
        state.uncorrelated_tagged_feedback_count += 1


def _match_feedback_leg(
    event: Event,
    state: AnalysisState,
    hop_sequence: int,
    first_reception: bool,
) -> Leg | None:
    if event.peer is None or event.node is None:
        _note_uncorrelated_feedback(event, state)
        if first_reception:
            _issue(
                state.global_issues,
                "uncorrelated_first_reception_feedback",
                "first-reception hop_feedback omits sender or receiver identity",
                [event],
            )
        return None

    if event.packet is not None:
        candidates = state.active_packet_hop_leg.get(
            (event.packet, event.peer, event.node, hop_sequence), []
        )
        if len(candidates) == 1:
            return candidates[0]
        prior_legs = state.known_hop_leg.get(
            (event.packet, event.peer, event.node, hop_sequence), []
        )
        latest = max(
            prior_legs,
            key=lambda leg: leg.first_event_index,
            default=None,
        )
        if (
            latest is not None
            and latest.completion_reason == "no_ack"
            and latest.completion_time_s is not None
            and latest.completion_time_s <= event.time_s
        ):
            _note_uncorrelated_feedback(event, state)
            state.late_feedback_after_no_ack_count += 1
            return None
        _note_uncorrelated_feedback(event, state)
        if first_reception:
            _issue(
                state.global_issues,
                "uncorrelated_first_reception_feedback",
                "first-reception packet feedback has no matching active or timed-out HOP leg",
                [event],
            )
        return None

    candidates = state.active_hop_leg.get(
        (event.peer, event.node, hop_sequence), []
    )
    if len(candidates) != 1:
        _note_uncorrelated_feedback(event, state)
        if first_reception:
            _issue(
                state.global_issues,
                "uncorrelated_first_reception_feedback",
                "tagless first-reception feedback does not identify one active HOP leg",
                [event],
            )
        return None
    return candidates[0]


def _handle_feedback(event: Event, state: AnalysisState) -> None:
    reason = _normalize(
        event.reason,
        {
            "ack": "ack",
            "dack": "dack",
            "suppressed_no_route": "suppressed_no_route",
        },
        event,
    )
    first_reception = _detail_bool(event.detail, "first_reception", event)
    hop_sequence = _detail_int(event.detail, "hop_sequence", event)
    leg = _match_feedback_leg(
        event, state, hop_sequence, first_reception
    )
    issues = leg.issues if leg is not None else state.global_issues
    ackable = _detail_bool(event.detail, "ackable", event)
    count_before = _detail_int(event.detail, "nsdp_count_before", event)
    count_after = _detail_int(event.detail, "nsdp_count_after", event)
    state_valid = _detail_bool(event.detail, "nsdp_state_valid", event)
    limit = _detail_int(event.detail, "nsdp_limit", event)
    if leg is not None and hop_sequence != leg.hop_sequence:
        _issue(leg.issues, "feedback_sequence_mismatch", "feedback HOP sequence differs from admission", [event])
    if limit == 0:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: hop_feedback nsdp_limit must be positive"
        )
    if not state_valid and (count_before != 0 or count_after != 0):
        _issue(
            issues,
            "invalid_nsdp_snapshot",
            "invalid feedback NSDP state must use the neutral 0-to-0 snapshot",
            [event],
        )
    if state_valid:
        expected_after = count_before + (1 if first_reception else 0)
        if count_after not in {count_before, expected_after}:
            _issue(
                issues,
                "bad_feedback_nsdp_transition",
                "feedback NSDP state changed by more than one reception",
                [event],
            )
        if not first_reception and count_after != count_before:
            _issue(
                issues,
                "duplicate_changed_nsdp",
                "duplicate reception changed NSDP state",
                [event],
            )
    if reason == "suppressed_no_route":
        if not first_reception or count_after != count_before:
            _issue(
                issues,
                "bad_suppressed_feedback",
                "suppressed no-route feedback must precede first-reception delivery",
                [event],
            )
    else:
        if not ackable:
            _issue(
                issues,
                "feedback_not_ackable",
                "ACK/DACK feedback was selected for a non-ackable reception",
                [event],
            )
        if reason == "dack":
            if not first_reception or not state_valid or count_before < limit:
                _issue(
                    issues,
                    "bad_dack_selection",
                    "DACK requires valid pre-enqueue NSDP count at or above the limit",
                    [event],
                )
        elif first_reception and state_valid and count_before >= limit:
            _issue(
                issues,
                "bad_ack_selection",
                "first-reception ACK selected with pre-enqueue NSDP at or above the limit",
                [event],
            )

    # A suppressed first reception can be followed by an ACK-marked duplicate.
    # A retry after a lost DACK is reaccepted and freshly reassessed, and can
    # therefore also end in ACK.  The last emitted ACK/DACK is the only
    # feedback reason that can be compared with eventual sender completion.
    if reason == "dack":
        state.feedback_dack_transitions[(count_before, count_after)] += 1
        if count_before < limit:
            state.feedback_dack_pre_limit_count += 1
        if count_before + 1 == limit and count_after == limit:
            state.feedback_dack_boundary_count += 1
    if leg is not None and (
        reason in {"ack", "dack"} or leg.feedback_time_s is None
    ):
        leg.feedback_time_s = event.time_s
        leg.feedback_reason = reason
    state.feedback_counts[reason] += 1


def _handle_nsdp_release(event: Event, state: AnalysisState) -> None:
    _require_node(event)
    if event.source is None or event.destination is None:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: nwk_nsdp_release requires src and dst"
        )
    before = _detail_int(event.detail, "count_before", event)
    after = _detail_int(event.detail, "count_after", event)
    limit = _detail_int(event.detail, "nsdp_limit", event)
    _detail_int(event.detail, "nwk_queue", event)
    if event.reason != "hop_feedback":
        raise AdmissionAnalysisError(
            f"row {event.input_row}: unsupported nwk_nsdp_release reason "
            f"{event.reason!r}"
        )
    if before < 1 or after != before - 1:
        _issue(state.global_issues, "bad_nsdp_release", "NSDP release must decrement a positive count by one", [event])
    if limit == 0:
        raise AdmissionAnalysisError(f"row {event.input_row}: nsdp_limit must be positive")
    if state.pending_nsdp_release is not None:
        _issue(
            state.global_issues,
            "unpaired_nsdp_release",
            "a second flow-level NSDP release appeared before HOP completion",
            [state.pending_nsdp_release, event],
        )
    state.pending_nsdp_release = event
    _record_maximum(state, "nwk_nsdp", before)
    # Completion follows immediately in source order, but the separate event
    # intentionally remains the authoritative count transition.


def _handle_completion(event: Event, state: AnalysisState) -> None:
    peer, next_hop = _require_directed_leg(event)
    reason = _normalize(event.reason, COMPLETION_REASONS, event)
    detail = event.detail
    hop_sequence = _detail_int(detail, "hop_sequence", event)
    leg = _find_transition_hop_leg(event, state, hop_sequence)
    duplicate_completion = leg.completion_time_s is not None
    if duplicate_completion:
        _issue(
            leg.issues,
            "duplicate_hop_completion",
            "leg has multiple HOP completions",
            [event],
        )
    release = state.pending_nsdp_release
    if release is None:
        _issue(
            leg.issues,
            "missing_nsdp_release",
            "HOP completion has no immediately preceding flow-level NSDP release",
            [event],
        )
    else:
        if (
            release.node != event.node
            or release.source != leg.packet.source
            or release.destination != leg.packet.destination
        ):
            _issue(
                leg.issues,
                "nsdp_release_flow_mismatch",
                "preceding NSDP release does not match completion node and flow",
                [release, event],
            )
        if release.index + 1 != event.index:
            _issue(
                leg.issues,
                "nsdp_release_order_mismatch",
                "flow-level NSDP release is not immediately followed by completion",
                [release, event],
            )
        if not duplicate_completion:
            leg.nsdp_release_time_s = release.time_s
        state.pending_nsdp_release = None
    resend_count = _detail_int(detail, "resend_count", event)
    pending_before = _detail_int(detail, "pending_before", event)
    pending_after = _detail_int(detail, "pending_after", event)
    limit = _detail_int(detail, "pending_limit", event)
    outstanding_before = _detail_int(detail, "outstanding_before", event)
    outstanding_after = _detail_int(detail, "outstanding_after", event)
    threshold_before = _detail_int(detail, "threshold_before", event)
    threshold_after = _detail_int(detail, "threshold_after", event)
    resend_before = _detail_int(detail, "resend_queue_before", event)
    resend_after = _detail_int(detail, "resend_queue_after", event)
    nsdp_released = _detail_bool(detail, "nsdp_released", event)
    capacity_released = _detail_bool(detail, "capacity_released", event)
    if duplicate_completion:
        return
    if peer != leg.next_hop:
        _issue(leg.issues, "completion_peer_mismatch", "completion peer differs from the admitted next hop", [event])
    if next_hop != leg.next_hop:
        _issue(leg.issues, "completion_next_hop_mismatch", "completion next hop differs from admission", [event])
    if hop_sequence != leg.hop_sequence:
        _issue(leg.issues, "completion_sequence_mismatch", "completion HOP sequence differs from admission", [event])
    if resend_after != resend_before - 1 or resend_before < 1:
        _issue(leg.issues, "bad_resend_queue_completion", "completion must remove one resend entry", [event])
    if not nsdp_released:
        _issue(leg.issues, "bad_nsdp_release_flag", "HOP completion does not report its NSDP release", [event])
    elif leg.nsdp_release_time_s is not None and not math.isclose(
        event.time_s,
        leg.nsdp_release_time_s,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        _issue(leg.issues, "nsdp_release_time_mismatch", "NSDP and HOP completion were not source-synchronous", [event])
    if reason == "dack":
        hold = _number(
            _required(detail, "dack_hold_seconds", event),
            "dack_hold_seconds",
            event.input_row,
        )
        if hold not in {20.0, 40.0}:
            _issue(leg.issues, "unexpected_dack_hold", "DACK hold is not the current ns-3 configured 20/40-second interval", [event])
        # Traces produced before exact DACK timer instrumentation omit this
        # field.  Preserve their value as regression evidence by applying the
        # source-defined scheduled offset; the release-time post-pass below
        # will still reject a legacy bare-deadline release.
        scheduled_text = detail.get("dack_scheduled_timer_offset_seconds")
        scheduled_offset = (
            OPNET_TIC_SECONDS
            if scheduled_text is None
            else _number(
                scheduled_text,
                "dack_scheduled_timer_offset_seconds",
                event.input_row,
            )
        )
        if not math.isclose(
            scheduled_offset,
            OPNET_TIC_SECONDS,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            _issue(
                leg.issues,
                "unexpected_dack_scheduled_offset",
                "DACK timer is not scheduled one OPNET TIC after nominal expiry",
                [event],
            )
        leg.dack_hold_seconds = hold
        leg.dack_scheduled_timer_offset_seconds = scheduled_offset
        if capacity_released or pending_after != pending_before or outstanding_after != outstanding_before:
            _issue(leg.issues, "dack_released_capacity_early", "DACK must retain HOP capacity until expiry", [event])
        if threshold_after != threshold_before:
            _issue(leg.issues, "dack_changed_threshold", "DACK completion changed the neighbor threshold", [event])
    else:
        if not capacity_released or pending_after != pending_before - 1 or outstanding_after != outstanding_before - 1:
            _issue(leg.issues, "bad_capacity_release", "ACK/no-ACK must release pending and outstanding once", [event])
        if reason == "ack" and threshold_after not in {threshold_before, threshold_before + 1}:
            _issue(leg.issues, "bad_ack_threshold", "ACK threshold change is not zero or plus one", [event])
        if reason == "no_ack" and threshold_after != max(0, threshold_before - 1):
            _issue(leg.issues, "bad_timeout_threshold", "no-ACK threshold did not decrease once with saturation at zero", [event])
    if not 0 <= threshold_after <= 16:
        _issue(leg.issues, "threshold_out_of_range", "neighbor threshold is outside 0..16", [event])
    if reason in {"ack", "dack"} and leg.feedback_reason and leg.feedback_reason != reason:
        leg.feedback_completion_mismatch = True
        state.feedback_completion_mismatch_count += 1
    if reason in {"ack", "dack"} and not leg.feedback_reason:
        leg.feedback_missing = True
        state.missing_feedback_count += 1
    if pending_before < 1 or outstanding_before < 1 or limit == 0:
        _issue(leg.issues, "invalid_completion_state", "completion starts from invalid capacity state", [event])
    if leg.resend_overflow:
        _issue(leg.issues, "completion_after_resend_overflow", "untracked resend-overflow admission unexpectedly completed", [event])
    leg.completion_time_s = event.time_s
    leg.completion_reason = reason
    leg.resend_count = resend_count
    state.completion_counts[reason] += 1
    _record_maximum(state, "hop_pending", pending_before)
    _record_maximum(state, "hop_outstanding", outstanding_before)
    _unindex_hop_leg(leg, state)
    if reason == "dack":
        _index_dack_hold_leg(leg, state)


def _handle_capacity_release(event: Event, state: AnalysisState) -> None:
    if event.reason not in {"dack_expiry", "dack-expiry"}:
        raise AdmissionAnalysisError(
            f"row {event.input_row}: unsupported hop_capacity_release reason"
        )
    peer, next_hop = _require_directed_leg(event)
    detail = event.detail
    hop_sequence = _detail_int(detail, "hop_sequence", event)
    leg = _find_transition_hop_leg(event, state, hop_sequence)
    resend_count = _detail_int(detail, "resend_count", event)
    pending_before = _detail_int(detail, "pending_before", event)
    pending_after = _detail_int(detail, "pending_after", event)
    pending_limit = _detail_int(detail, "pending_limit", event)
    outstanding_before = _detail_int(detail, "outstanding_before", event)
    outstanding_after = _detail_int(detail, "outstanding_after", event)
    threshold_before = _detail_int(detail, "threshold_before", event)
    threshold_after = _detail_int(detail, "threshold_after", event)
    hold = _number(_required(detail, "dack_hold_seconds", event), "dack_hold_seconds", event.input_row)
    scheduled_text = detail.get("dack_scheduled_timer_offset_seconds")
    scheduled_offset = (
        leg.dack_scheduled_timer_offset_seconds
        if scheduled_text is None
        and leg.dack_scheduled_timer_offset_seconds is not None
        else OPNET_TIC_SECONDS
        if scheduled_text is None
        else _number(
            scheduled_text,
            "dack_scheduled_timer_offset_seconds",
            event.input_row,
        )
    )
    effective_text = detail.get("dack_effective_timer_offset_seconds")
    effective_offset = (
        event.time_s - leg.completion_time_s - hold
        if effective_text is None and leg.completion_time_s is not None
        else 0.0
        if effective_text is None
        else _number(
            effective_text,
            "dack_effective_timer_offset_seconds",
            event.input_row,
        )
    )
    nsdp_released = _detail_bool(detail, "nsdp_released", event)
    capacity_released = _detail_bool(detail, "capacity_released", event)
    if peer != leg.next_hop:
        _issue(leg.issues, "capacity_peer_mismatch", "DACK release peer differs from admission", [event])
    if next_hop != leg.next_hop:
        _issue(leg.issues, "capacity_next_hop_mismatch", "DACK release next hop differs from admission", [event])
    if hop_sequence != leg.hop_sequence:
        _issue(leg.issues, "capacity_sequence_mismatch", "DACK release sequence differs from admission", [event])
    if leg.completion_reason != "dack":
        _issue(leg.issues, "capacity_release_without_dack", "DACK expiry has no DACK completion", [event])
    if leg.resend_count is not None and resend_count != leg.resend_count:
        _issue(leg.issues, "capacity_resend_count_mismatch", "DACK expiry resend count differs from completion", [event])
    if pending_after != pending_before - 1 or outstanding_after != outstanding_before - 1:
        _issue(leg.issues, "bad_dack_capacity_release", "DACK expiry must release pending and outstanding once", [event])
    if threshold_after != threshold_before:
        _issue(leg.issues, "dack_expiry_changed_threshold", "DACK expiry changed threshold", [event])
    if pending_limit == 0 or pending_before < 1 or outstanding_before < 1:
        _issue(leg.issues, "invalid_dack_release_state", "DACK expiry starts from invalid capacity state", [event])
    if nsdp_released or not capacity_released:
        _issue(leg.issues, "bad_dack_release_flags", "DACK expiry must release capacity but not NSDP", [event])
    if hold not in {20.0, 40.0}:
        _issue(leg.issues, "unexpected_dack_hold", "DACK hold is not the current ns-3 observed 20/40-second interval", [event])
    # Each entry schedules a timer one TIC after nominal expiry, but every
    # timer scans the complete DACK list.  A nearby or stale timer may therefore
    # release another nominally expired entry with an effective offset in
    # [0, TIC].  A post-pass below also requires every release timestamp to
    # coincide with one of the scheduled per-entry timer timestamps.
    if not math.isclose(
        scheduled_offset,
        OPNET_TIC_SECONDS,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        _issue(
            leg.issues,
            "unexpected_dack_scheduled_offset",
            "DACK timer is not scheduled one OPNET TIC after nominal expiry",
            [event],
        )
    if effective_offset < -1.0e-9 or effective_offset > OPNET_TIC_SECONDS + 1.0e-9:
        _issue(
            leg.issues,
            "dack_effective_offset_out_of_range",
            "DACK effective timer offset is outside zero through one OPNET TIC",
            [event],
        )
    if leg.completion_time_s is not None and not math.isclose(
        event.time_s - leg.completion_time_s,
        hold + effective_offset,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        _issue(
            leg.issues,
            "dack_expiry_time_mismatch",
            "DACK release time does not equal its nominal hold plus effective timer offset",
            [event],
        )
    if leg.dack_hold_seconds is not None and not math.isclose(
        hold, leg.dack_hold_seconds, rel_tol=0.0, abs_tol=1.0e-9
    ):
        _issue(leg.issues, "dack_hold_mismatch", "DACK expiry hold differs from completion", [event])
    if (
        leg.dack_scheduled_timer_offset_seconds is not None
        and not math.isclose(
            scheduled_offset,
            leg.dack_scheduled_timer_offset_seconds,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
    ):
        _issue(
            leg.issues,
            "dack_scheduled_offset_mismatch",
            "DACK expiry scheduled offset differs from completion",
            [event],
        )
    if leg.capacity_release_time_s is not None:
        _issue(leg.issues, "duplicate_capacity_release", "leg released DACK capacity more than once", [event])
    leg.capacity_release_time_s = event.time_s
    leg.dack_hold_seconds = hold
    leg.dack_scheduled_timer_offset_seconds = scheduled_offset
    leg.dack_effective_timer_offset_seconds = effective_offset
    leg.capacity_release_event = event
    _unindex_hop_leg(leg, state)
    _unindex_dack_hold_leg(leg, state)


def _validate_dack_timer_triggers(state: AnalysisState) -> None:
    # CheckDack scans one CsrHopLayer's list.  A timer on another node cannot
    # release this node's entries even when both timestamps happen to match.
    timer_times_by_node: dict[int, list[float]] = defaultdict(list)
    for leg in state.legs:
        if (
            leg.completion_reason == "dack"
            and leg.completion_time_s is not None
            and leg.dack_hold_seconds is not None
            and leg.dack_scheduled_timer_offset_seconds is not None
        ):
            timer_times_by_node[leg.node].append(
                leg.completion_time_s
                + leg.dack_hold_seconds
                + leg.dack_scheduled_timer_offset_seconds
            )
    for leg in state.legs:
        if (
            leg.capacity_release_time_s is None
            or leg.capacity_release_event is None
            or any(
                math.isclose(
                    leg.capacity_release_time_s,
                    timer_time,
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
                for timer_time in timer_times_by_node.get(leg.node, [])
            )
        ):
            continue
        _issue(
            leg.issues,
            "dack_expiry_time_mismatch",
            "DACK capacity release did not coincide with any scheduled DACK timer",
            [leg.capacity_release_event],
        )


def _process(events: Iterable[Event]) -> AnalysisState:
    state = AnalysisState()
    previous_observed: Event | None = None
    for event in events:
        state.event_counts[event.name] += 1
        if event.name == "app_admission":
            _handle_app(event, state)
        elif event.name == "app_send":
            packet = _require_packet(event)
            state.app_sends[packet] += 1
            admitted = state.admitted_packets.get(packet)
            if (
                admitted is None
                or previous_observed is not admitted.event
                or previous_observed.index + 1 != event.index
            ):
                _issue(
                    state.global_issues,
                    "app_send_order_mismatch",
                    "app_send is not immediately preceded by its admitted attempt",
                    [event],
                )
        elif event.name == "nwk_admission":
            _handle_nwk(event, state)
        elif event.name == "hop_admission":
            _handle_hop_admission(event, state)
        elif event.name == "hop_feedback":
            _handle_feedback(event, state)
        elif event.name == "nwk_nsdp_release":
            _handle_nsdp_release(event, state)
        elif event.name == "hop_completion":
            _handle_completion(event, state)
        elif event.name == "hop_capacity_release":
            _handle_capacity_release(event, state)
        elif event.name == "nwk_delivery":
            state.deliveries[_require_packet(event)] += 1
        previous_observed = event

    for packet, attempt in state.admitted_packets.items():
        sends = state.app_sends.get(packet, 0)
        if sends != 1:
            _issue(
                state.global_issues,
                "app_send_count_mismatch",
                f"admitted packet {packet} has {sends} app_send rows",
                [attempt.event],
            )
    for packet, count in state.app_sends.items():
        if packet not in state.admitted_packets:
            _issue(state.global_issues, "send_without_admission", f"packet {packet} has no admitted attempt")
        if count != 1:
            _issue(state.global_issues, "duplicate_app_send", f"packet {packet} has {count} app_send rows")
    for packet, count in state.deliveries.items():
        if packet not in state.admitted_packets:
            _issue(state.global_issues, "delivery_without_admission", f"packet {packet} was delivered without app admission")
        if count != 1:
            _issue(state.global_issues, "duplicate_delivery", f"packet {packet} has {count} deliveries")
    for leg in state.legs:
        if leg.hop_admission_time_s is None:
            _issue(leg.issues, "missing_hop_admission", "successful NWK admission has no HOP admission")
        if leg.completion_reason == "dack" and leg.capacity_release_time_s is None:
            # A trace may end while a valid DACK hold is active.
            pass
    if state.pending_nsdp_release is not None:
        _issue(
            state.global_issues,
            "unpaired_nsdp_release",
            "trace ended before the flow-level NSDP release was paired with completion",
            [state.pending_nsdp_release],
        )
    _validate_dack_timer_triggers(state)
    return state


def _parse_diagnostics(path: Path) -> tuple[dict[int, dict[str, object]], dict[str, int]]:
    required = {
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
    }
    rows: dict[int, dict[str, object]] = {}
    totals: Counter[str] = Counter()
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise AdmissionAnalysisError("application diagnostics has no header")
        missing = required - set(reader.fieldnames)
        if missing:
            raise AdmissionAnalysisError(
                "application diagnostics missing columns: " + ", ".join(sorted(missing))
            )
        for input_row, row in enumerate(reader, start=2):
            if None in row or any(value is None for value in row.values()):
                raise AdmissionAnalysisError(
                    f"application diagnostics row {input_row} is malformed"
                )
            if row["schema"].strip() != APP_DIAGNOSTICS_SCHEMA:
                raise AdmissionAnalysisError(
                    f"application diagnostics row {input_row} has unsupported schema"
                )
            flow = _integer(row["flow_index"], "flow_index", input_row)
            if flow in rows:
                raise AdmissionAnalysisError("application diagnostics has duplicate flow_index")
            counters = {
                key: _integer(row[key], key, input_row)
                for key in (
                    "attempts",
                    "admitted",
                    "blocked_discovery",
                    "blocked_topology",
                    "blocked_gateway_route",
                    "blocked_destination",
                    "blocked_nsdp",
                )
            }
            if counters["attempts"] != sum(
                value for key, value in counters.items() if key != "attempts"
            ):
                raise AdmissionAnalysisError(
                    f"application diagnostics row {input_row} does not partition attempts"
                )
            first = (
                _number(row["first_admitted_s"], "first_admitted_s", input_row)
                if row["first_admitted_s"].strip()
                else None
            )
            last = (
                _number(row["last_admitted_s"], "last_admitted_s", input_row)
                if row["last_admitted_s"].strip()
                else None
            )
            rows[flow] = {**counters, "first_admitted_s": first, "last_admitted_s": last}
            totals.update(counters)
    return rows, dict(totals)


def _reconcile_diagnostics(
    path: Path | None, state: AnalysisState
) -> dict[str, object] | None:
    if path is None:
        return None
    rows, totals = _parse_diagnostics(path)
    mismatches: list[dict[str, object]] = []
    all_flows = sorted(set(rows) | set(state.app_by_flow))
    for flow in all_flows:
        trace_counts = state.app_by_flow.get(flow, Counter())
        trace_attempts = sum(trace_counts.values())
        diagnostic = rows.get(flow)
        expected = {
            "attempts": trace_attempts,
            "admitted": trace_counts["admitted"],
            "blocked_discovery": trace_counts["blocked_discovery"],
            "blocked_topology": trace_counts["blocked_topology"],
            "blocked_gateway_route": trace_counts["blocked_gateway_route"],
            "blocked_destination": trace_counts["blocked_destination"],
            "blocked_nsdp": trace_counts["blocked_nsdp"],
        }
        if diagnostic is None:
            mismatches.append({"flow_index": flow, "reason": "missing_diagnostic_row"})
            continue
        for key, value in expected.items():
            if diagnostic[key] != value:
                mismatches.append(
                    {
                        "flow_index": flow,
                        "field": key,
                        "trace": value,
                        "diagnostics": diagnostic[key],
                    }
                )
        expected_first = state.first_admitted_s.get(flow)
        expected_last = state.last_admitted_s.get(flow)
        for key, value in (
            ("first_admitted_s", expected_first),
            ("last_admitted_s", expected_last),
        ):
            observed = diagnostic[key]
            if (value is None) != (observed is None) or (
                value is not None
                and observed is not None
                and not math.isclose(value, observed, rel_tol=0.0, abs_tol=1.0e-9)
            ):
                mismatches.append(
                    {
                        "flow_index": flow,
                        "field": key,
                        "trace": value,
                        "diagnostics": observed,
                    }
                )
    return {
        "path": str(path.resolve()),
        "sha256": _sha256_path(path),
        "flow_rows": len(rows),
        "totals": totals,
        "mismatches": mismatches,
        "pass": not mismatches,
    }


def _leg_status(leg: Leg) -> str:
    if leg.hop_admission_time_s is None:
        return "open_nwk" if leg.holds else "missing_hop_admission"
    if leg.resend_overflow:
        return "resend_overflow_stranded"
    if leg.completion_time_s is None:
        return "open_resend"
    if leg.completion_reason == "dack" and leg.capacity_release_time_s is None:
        return "open_dack_hold"
    return "complete"


def _held_open_legs(state: AnalysisState) -> list[Leg]:
    result: list[Leg] = []
    for (packet, node), holds in state.held.items():
        leg_index = state.leg_counts[packet]
        state.leg_counts[packet] += 1
        first = state.admitted_packets.get(packet)
        result.append(
            Leg(
                packet=packet,
                leg_index=leg_index,
                node=node,
                next_hop=None,
                first_event_index=(first.event.index if first else 0),
                holds=holds,
            )
        )
    return result


def _undelivered_terminal_status(
    state: AnalysisState, legs: list[Leg]
) -> dict[str, int]:
    highest_leg: dict[PacketKey, Leg] = {}
    for leg in legs:
        prior = highest_leg.get(leg.packet)
        if prior is None or leg.leg_index > prior.leg_index:
            highest_leg[leg.packet] = leg

    counts: Counter[str] = Counter()
    for packet in state.admitted_packets:
        if state.deliveries.get(packet, 0) > 0:
            continue
        leg = highest_leg.get(packet)
        if leg is None:
            counts["no_nwk_observation"] += 1
        elif leg.hop_admission_time_s is None and leg.holds:
            counts["open_nwk"] += 1
        elif leg.completion_reason:
            counts[f"complete/{leg.completion_reason}"] += 1
        elif leg.resend_overflow:
            counts["resend_overflow_stranded"] += 1
        elif leg.hop_admission_time_s is not None:
            counts["open_resend"] += 1
        else:
            counts["missing_hop_admission"] += 1
    return dict(sorted(counts.items()))


def analyze(
    trace: Path, diagnostics: Path | None = None
) -> tuple[list[Leg], dict[str, object]]:
    trace_stats = TraceStats()
    state = _process(iter_events(trace, trace_stats))
    reconciliation = _reconcile_diagnostics(diagnostics, state)
    legs = state.legs + _held_open_legs(state)
    undelivered_by_terminal_status = _undelivered_terminal_status(state, legs)
    invalid_legs = sum(bool(leg.issues) for leg in legs)
    open_resend = sum(_leg_status(leg) == "open_resend" for leg in legs)
    open_dack = sum(_leg_status(leg) == "open_dack_hold" for leg in legs)
    resend_overflow = sum(
        _leg_status(leg) == "resend_overflow_stranded" for leg in legs
    )
    open_nwk = sum(
        leg.hop_admission_time_s is None and bool(leg.holds) for leg in legs
    )
    passed = (
        not state.global_issues
        and invalid_legs == 0
        and (reconciliation is None or bool(reconciliation["pass"]))
    )
    report: dict[str, object] = {
        "schema": REPORT_SCHEMA,
        "pass": passed,
        "evidence_boundary": (
            "ns-3 source-ordered isolation evidence; no corresponding OPNET "
            "event ledger is present in the recovered aggregate results"
        ),
        "input": {
            "path": str(trace.resolve()),
            "sha256": _sha256_path(trace),
            "row_count": trace_stats.row_count,
            "event_counts": dict(sorted(trace_stats.event_counts.items())),
        },
        "application": {
            "attempts": sum(state.app_counts.values()),
            "by_decision": dict(sorted(state.app_counts.items())),
            "flows": [
                {
                    "flow_index": flow,
                    "source": state.flow_source.get(flow),
                    "attempts": sum(counts.values()),
                    "by_decision": dict(sorted(counts.items())),
                }
                for flow, counts in sorted(state.app_by_flow.items())
            ],
        },
        "nwk": {
            "by_decision": dict(sorted(state.nwk_counts.items())),
            "nodes": [
                {"node": node, "by_decision": dict(sorted(counts.items()))}
                for node, counts in sorted(state.nwk_by_node.items())
            ],
            "holds_by_node_next_hop": [
                {
                    "node": node,
                    "next_hop": next_hop,
                    "attempts": sum(counts.values()),
                    "by_reason": dict(sorted(counts.items())),
                }
                for (node, next_hop), counts in sorted(
                    state.nwk_holds_by_location.items(),
                    key=lambda item: (
                        item[0][0],
                        -1 if item[0][1] is None else item[0][1],
                    ),
                )
            ],
        },
        "hop": {
            "admissions": state.event_counts["hop_admission"],
            "feedback_by_reason": dict(sorted(state.feedback_counts.items())),
            "completions_by_reason": dict(sorted(state.completion_counts.items())),
            "capacity_releases": state.event_counts["hop_capacity_release"],
            "uncorrelated_feedback": state.uncorrelated_feedback_count,
            "uncorrelated_tagged_feedback": (
                state.uncorrelated_tagged_feedback_count
            ),
            "uncorrelated_tagless_feedback": (
                state.uncorrelated_tagless_feedback_count
            ),
            "late_feedback_after_no_ack": (
                state.late_feedback_after_no_ack_count
            ),
            "missing_per_leg_feedback": state.missing_feedback_count,
            "feedback_completion_reason_mismatches": (
                state.feedback_completion_mismatch_count
            ),
            "dack_pre_limit_findings": state.feedback_dack_pre_limit_count,
            "dack_boundary_count": state.feedback_dack_boundary_count,
            "dack_nsdp_transitions": [
                {
                    "count_before": before,
                    "count_after": after,
                    "events": count,
                }
                for (before, after), count in sorted(
                    state.feedback_dack_transitions.items()
                )
            ],
            "open_resend_legs": open_resend,
            "open_dack_hold_legs": open_dack,
            "resend_overflow_stranded_legs": resend_overflow,
        },
        "inventory": {
            "admitted_packets": len(state.admitted_packets),
            "app_sends": sum(state.app_sends.values()),
            "delivered_packets": len(state.deliveries),
            "undelivered_packets": sum(
                undelivered_by_terminal_status.values()
            ),
            "undelivered_by_terminal_status": (
                undelivered_by_terminal_status
            ),
            "hop_legs": len(state.legs),
            "open_nwk_locations": open_nwk,
        },
        "maxima": dict(sorted(state.maxima.items())),
        "integrity": {
            "global_issue_count": len(state.global_issues),
            "invalid_leg_count": invalid_legs,
            "global_issues": state.global_issues,
        },
        "application_diagnostics": reconciliation,
    }
    return legs, report


def _csv_value(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return format(value, ".17g")
    return str(value)


def write_outputs(
    legs: list[Leg], report: dict[str, object], summary: Path, legs_csv: Path
) -> None:
    summary.parent.mkdir(parents=True, exist_ok=True)
    legs_csv.parent.mkdir(parents=True, exist_ok=True)
    with legs_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=LEG_CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for leg in sorted(
            legs,
            key=lambda item: (
                item.first_event_index,
                item.packet,
                item.node,
                item.leg_index,
            ),
        ):
            writer.writerow(
                {
                    "src": leg.packet.source,
                    "dst": leg.packet.destination,
                    "sequence": leg.packet.sequence,
                    "leg_index": leg.leg_index,
                    "node": leg.node,
                    "next_hop": _csv_value(leg.next_hop),
                    "hop_sequence": _csv_value(leg.hop_sequence),
                    "nwk_admission_time_s": _csv_value(leg.nwk_admission_time_s),
                    "hop_admission_time_s": _csv_value(leg.hop_admission_time_s),
                    "feedback_time_s": _csv_value(leg.feedback_time_s),
                    "feedback_reason": leg.feedback_reason,
                    "feedback_missing": "1" if leg.feedback_missing else "0",
                    "feedback_completion_mismatch": (
                        "1" if leg.feedback_completion_mismatch else "0"
                    ),
                    "nsdp_release_time_s": _csv_value(leg.nsdp_release_time_s),
                    "completion_time_s": _csv_value(leg.completion_time_s),
                    "completion_reason": leg.completion_reason,
                    "capacity_release_time_s": _csv_value(leg.capacity_release_time_s),
                    "dack_hold_seconds": _csv_value(leg.dack_hold_seconds),
                    "dack_scheduled_timer_offset_seconds": _csv_value(
                        leg.dack_scheduled_timer_offset_seconds
                    ),
                    "dack_effective_timer_offset_seconds": _csv_value(
                        leg.dack_effective_timer_offset_seconds
                    ),
                    "resend_count": _csv_value(leg.resend_count),
                    "resend_overflow": "1" if leg.resend_overflow else "0",
                    "no_route_holds": leg.holds["no_route"],
                    "global_capacity_holds": leg.holds["global_capacity"],
                    "neighbor_capacity_holds": leg.holds["neighbor_capacity"],
                    "status": _leg_status(leg),
                    "valid": "1" if not leg.issues else "0",
                    "issues_json": json.dumps(
                        leg.issues, separators=(",", ":"), sort_keys=True
                    ),
                }
            )
    report["outputs"] = {
        "legs_csv": {
            "path": str(legs_csv.resolve()),
            "sha256": _sha256_path(legs_csv),
            "row_count": len(legs),
        }
    }
    summary.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="csr-differential-trace-v1 CSV")
    parser.add_argument("--app-diagnostics", type=Path)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--legs-csv", type=Path, required=True)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="exit nonzero after writing evidence when structural/state checks fail",
    )
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    try:
        trace = arguments.trace.resolve()
        diagnostics = (
            arguments.app_diagnostics.resolve()
            if arguments.app_diagnostics is not None
            else None
        )
        summary = arguments.summary_json.resolve()
        legs_csv = arguments.legs_csv.resolve()
        _validate_paths(trace, diagnostics, summary, legs_csv)
        legs, report = analyze(trace, diagnostics)
        write_outputs(legs, report, summary, legs_csv)
    except (OSError, csv.Error, AdmissionAnalysisError) as error:
        raise SystemExit(f"error: {error}") from error
    counts = report["inventory"]
    print(
        f"{'PASS' if report['pass'] else 'FAIL'}: "
        f"attempts={report['application']['attempts']} "
        f"packets={counts['admitted_packets']} legs={counts['hop_legs']} "
        f"delivered={counts['delivered_packets']}"
    )
    if arguments.strict and not report["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
