#!/usr/bin/env python3
"""Reconstruct CSR NWK packet paths and residence times from an ns-3 trace."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable


TRACE_SCHEMA = "csr-differential-trace-v1"
REPORT_SCHEMA = "csr-packet-path-analysis-v1"
PACKET_EVENTS = {"app_send", "nwk_enqueue", "nwk_forward", "nwk_delivery"}
REQUIRED_COLUMNS = {
    "schema",
    "event_index",
    "time_s",
    "event",
    "node",
    "src",
    "dst",
    "sequence",
    "next_hop",
}
PACKET_CSV_COLUMNS = (
    "src",
    "dst",
    "sequence",
    "status",
    "valid",
    "complete",
    "delivered",
    "send_time_s",
    "local_enqueue_time_s",
    "delivery_time_s",
    "end_to_end_s",
    "app_to_local_enqueue_s",
    "path_json",
    "hop_count",
    "nwk_residences_json",
    "leg_transits_json",
    "total_nwk_residence_s",
    "total_leg_transit_s",
    "decomposition_s",
    "decomposition_error_s",
    "post_startup",
    "route_change_count",
    "route_context_mismatch_count",
    "issue_count",
    "issues_json",
)


class PathAnalysisError(ValueError):
    """The trace or requested analysis cannot be interpreted safely."""


@dataclass(frozen=True, order=True)
class PacketKey:
    """Observation-only identity carried unchanged through the CSR stack."""

    source: int
    destination: int
    sequence: int


@dataclass(frozen=True)
class TraceEvent:
    """One packet-lifecycle trace row in original source order."""

    event_index: int
    time_s: float
    event: str
    node: int
    key: PacketKey
    next_hop: int | None
    detail: str
    input_row: int


@dataclass(frozen=True)
class RouteChange:
    """One optional selected-route observation."""

    event_index: int
    time_s: float
    node: int
    destination: int
    selected: bool
    next_hop: int | None
    input_row: int


@dataclass
class LoadedTrace:
    """Validated trace rows needed by the path analysis."""

    packets: dict[PacketKey, list[TraceEvent]]
    route_changes: dict[tuple[int, int], list[RouteChange]]
    row_count: int
    event_counts: Counter[str]


@dataclass
class PacketAnalysis:
    """One reconstructed packet lifecycle."""

    key: PacketKey
    first_event_index: int
    valid: bool
    complete: bool
    delivered: bool
    send_time_s: float | None
    local_enqueue_time_s: float | None
    delivery_time_s: float | None
    end_to_end_s: float | None
    app_to_local_enqueue_s: float | None
    path: list[int]
    hop_count: int | None
    nwk_residences: list[dict[str, object]]
    leg_transits: list[dict[str, object]]
    total_nwk_residence_s: float | None
    total_leg_transit_s: float | None
    decomposition_s: float | None
    decomposition_error_s: float | None
    route_change_count: int
    route_context_mismatch_count: int
    issues: list[dict[str, object]]


def _paths_alias(first: Path, second: Path) -> bool:
    """Return whether two resolved paths identify the same filesystem entry."""

    if first == second:
        return True
    try:
        return first.samefile(second)
    except OSError:
        return False


def _validate_cli_paths(trace: Path, summary: Path, packet_csv: Path) -> None:
    """Protect the input and keep the generated artifacts distinct."""

    if _paths_alias(trace, summary):
        raise PathAnalysisError("summary JSON must not alias the input trace")
    if _paths_alias(trace, packet_csv):
        raise PathAnalysisError("packet CSV must not alias the input trace")
    if _paths_alias(summary, packet_csv):
        raise PathAnalysisError("summary JSON must not alias the packet CSV")


def _parse_nonnegative_integer(value: str | None, label: str, row: int) -> int:
    text = (value or "").strip()
    if not re.fullmatch(r"[0-9]+", text):
        raise PathAnalysisError(f"row {row}: {label} must be a non-negative integer")
    return int(text)


def _parse_time(value: str | None, row: int) -> float:
    text = (value or "").strip()
    try:
        parsed = float(text)
    except ValueError as error:
        raise PathAnalysisError(f"row {row}: time_s is not numeric: {text!r}") from error
    if not math.isfinite(parsed) or parsed < 0.0:
        raise PathAnalysisError(f"row {row}: time_s must be finite and non-negative")
    return parsed


def _parse_route_selected(value: str | None, next_hop: int | None, row: int) -> bool:
    text = (value or "").strip().lower()
    if text in {"1", "true", "yes", "selected", "available"}:
        return True
    if text in {"0", "false", "no", "removed", "unavailable"}:
        return False
    if not text:
        return next_hop is not None
    raise PathAnalysisError(f"row {row}: invalid route_change success value {text!r}")


def load_trace(path: Path) -> LoadedTrace:
    """Load a canonical trace and validate its global source order."""

    try:
        stream = path.open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise PathAnalysisError(str(error)) from error

    packets: dict[PacketKey, list[TraceEvent]] = defaultdict(list)
    route_changes: dict[tuple[int, int], list[RouteChange]] = defaultdict(list)
    event_counts: Counter[str] = Counter()
    previous_index: int | None = None
    previous_time: float | None = None
    row_count = 0

    with stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise PathAnalysisError(f"{path}: trace has no CSV header")
        duplicate_headers = [
            name for name, count in Counter(reader.fieldnames).items() if count > 1
        ]
        if duplicate_headers:
            raise PathAnalysisError(
                f"{path}: duplicate CSV column(s): {', '.join(duplicate_headers)}"
            )
        missing = REQUIRED_COLUMNS - set(reader.fieldnames)
        if missing:
            raise PathAnalysisError(
                f"{path}: missing required column(s): {', '.join(sorted(missing))}"
            )

        for input_row, row in enumerate(reader, start=2):
            row_count += 1
            if None in row:
                raise PathAnalysisError(f"row {input_row}: too many CSV fields")
            if any(value is None for value in row.values()):
                raise PathAnalysisError(f"row {input_row}: too few CSV fields")
            if (row.get("schema") or "").strip() != TRACE_SCHEMA:
                raise PathAnalysisError(
                    f"row {input_row}: expected schema {TRACE_SCHEMA!r}"
                )
            event_index = _parse_nonnegative_integer(
                row.get("event_index"), "event_index", input_row
            )
            time_s = _parse_time(row.get("time_s"), input_row)
            if previous_index is not None and event_index <= previous_index:
                raise PathAnalysisError(
                    f"row {input_row}: event_index {event_index} is not strictly "
                    f"greater than {previous_index}"
                )
            if previous_time is not None and time_s < previous_time:
                raise PathAnalysisError(
                    f"row {input_row}: time_s {time_s} precedes prior source time "
                    f"{previous_time}"
                )
            previous_index = event_index
            previous_time = time_s

            event = (row.get("event") or "").strip()
            if not event:
                raise PathAnalysisError(f"row {input_row}: event is empty")
            event_counts[event] += 1
            if event not in PACKET_EVENTS and event != "route_change":
                continue

            if event == "route_change":
                node = _parse_nonnegative_integer(row.get("node"), "node", input_row)
                destination = _parse_nonnegative_integer(
                    row.get("dst"), "dst", input_row
                )
                next_hop_text = (row.get("next_hop") or "").strip()
                next_hop = (
                    _parse_nonnegative_integer(next_hop_text, "next_hop", input_row)
                    if next_hop_text
                    else None
                )
                selected = _parse_route_selected(
                    row.get("success"), next_hop, input_row
                )
                if selected and next_hop is None:
                    raise PathAnalysisError(
                        f"row {input_row}: selected route_change has no next_hop"
                    )
                route_changes[(node, destination)].append(
                    RouteChange(
                        event_index=event_index,
                        time_s=time_s,
                        node=node,
                        destination=destination,
                        selected=selected,
                        next_hop=next_hop,
                        input_row=input_row,
                    )
                )
                continue

            source = _parse_nonnegative_integer(row.get("src"), "src", input_row)
            destination = _parse_nonnegative_integer(row.get("dst"), "dst", input_row)
            sequence = _parse_nonnegative_integer(
                row.get("sequence"), "sequence", input_row
            )
            node = _parse_nonnegative_integer(row.get("node"), "node", input_row)
            next_hop_text = (row.get("next_hop") or "").strip()
            next_hop = (
                _parse_nonnegative_integer(next_hop_text, "next_hop", input_row)
                if next_hop_text
                else None
            )
            if event == "nwk_forward" and next_hop is None:
                raise PathAnalysisError(
                    f"row {input_row}: nwk_forward requires next_hop"
                )
            key = PacketKey(source, destination, sequence)
            packets[key].append(
                TraceEvent(
                    event_index=event_index,
                    time_s=time_s,
                    event=event,
                    node=node,
                    key=key,
                    next_hop=next_hop,
                    detail=(row.get("detail") or "").strip(),
                    input_row=input_row,
                )
            )

    if row_count == 0:
        raise PathAnalysisError(f"{path}: trace has no data rows")
    if not packets:
        raise PathAnalysisError(f"{path}: trace has no packet lifecycle events")
    return LoadedTrace(dict(packets), dict(route_changes), row_count, event_counts)


def _issue(
    issues: list[dict[str, object]],
    code: str,
    message: str,
    events: Iterable[TraceEvent] = (),
) -> None:
    issues.append(
        {
            "code": code,
            "message": message,
            "event_indexes": [event.event_index for event in events],
        }
    )


def _matches_complete_pattern(events: list[TraceEvent]) -> bool:
    if len(events) < 4 or events[0].event != "app_send":
        return False
    if events[-1].event != "nwk_delivery":
        return False
    middle = events[1:-1]
    return len(middle) % 2 == 0 and all(
        event.event == ("nwk_enqueue" if index % 2 == 0 else "nwk_forward")
        for index, event in enumerate(middle)
    )


def _matches_undelivered_prefix(events: list[TraceEvent]) -> bool:
    """Return whether rows are a valid prefix truncated before delivery."""

    if not events or events[0].event != "app_send":
        return False
    if any(event.event == "nwk_delivery" for event in events):
        return False
    return all(
        event.event == ("nwk_enqueue" if index % 2 == 0 else "nwk_forward")
        for index, event in enumerate(events[1:])
    )


def _route_state_before(
    route_changes: dict[tuple[int, int], list[RouteChange]],
    node: int,
    destination: int,
    event_index: int,
) -> RouteChange | None:
    changes = route_changes.get((node, destination), [])
    indexes = [change.event_index for change in changes]
    position = bisect_left(indexes, event_index) - 1
    return changes[position] if position >= 0 else None


def analyze_packet(
    key: PacketKey,
    events: list[TraceEvent],
    route_changes: dict[tuple[int, int], list[RouteChange]],
    tolerance_s: float,
) -> PacketAnalysis:
    """Validate and reconstruct one exact packet lifecycle."""

    issues: list[dict[str, object]] = []
    sends = [event for event in events if event.event == "app_send"]
    deliveries = [event for event in events if event.event == "nwk_delivery"]
    enqueues = [event for event in events if event.event == "nwk_enqueue"]
    forwards = [event for event in events if event.event == "nwk_forward"]
    delivered = bool(deliveries)

    if not sends:
        _issue(issues, "missing_app_send", "packet has no app_send")
    elif len(sends) > 1:
        _issue(issues, "duplicate_app_send", "packet has multiple app_send rows", sends)
    if not deliveries:
        _issue(issues, "missing_delivery", "packet has no nwk_delivery")
    elif len(deliveries) > 1:
        _issue(
            issues,
            "duplicate_delivery",
            "packet has multiple nwk_delivery rows",
            deliveries,
        )

    local_enqueues = [event for event in enqueues if event.node == key.source]
    if not local_enqueues:
        _issue(issues, "missing_local_admission", "packet has no source NWK enqueue")
    elif len(local_enqueues) > 1:
        _issue(
            issues,
            "duplicate_local_admission",
            "packet entered its source NWK queue more than once",
            local_enqueues,
        )
    destination_enqueues = [
        event
        for event in enqueues
        if event.node == key.destination and key.destination != key.source
    ]
    if destination_enqueues:
        _issue(
            issues,
            "destination_queue_admission",
            "final destination must emit nwk_delivery instead of nwk_enqueue",
            destination_enqueues,
        )

    complete = _matches_complete_pattern(events)
    valid_undelivered_prefix = not delivered and _matches_undelivered_prefix(events)
    if valid_undelivered_prefix:
        _issue(
            issues,
            "undelivered_lifecycle",
            "valid packet prefix ends before nwk_delivery",
            events,
        )
    elif not complete:
        _issue(
            issues,
            "incomplete_or_out_of_order_lifecycle",
            "expected app_send,(nwk_enqueue,nwk_forward)+,nwk_delivery",
            events,
        )

    send_time_s = sends[0].time_s if len(sends) == 1 else None
    delivery_time_s = deliveries[0].time_s if len(deliveries) == 1 else None
    local_enqueue_time_s = (
        local_enqueues[0].time_s if len(local_enqueues) == 1 else None
    )
    end_to_end_s: float | None = None
    app_to_local_enqueue_s: float | None = None
    path: list[int] = []
    hop_count: int | None = None
    nwk_residences: list[dict[str, object]] = []
    leg_transits: list[dict[str, object]] = []
    total_nwk_residence_s: float | None = None
    total_leg_transit_s: float | None = None
    decomposition_s: float | None = None
    decomposition_error_s: float | None = None
    route_context_mismatch_count = 0

    if complete:
        send = events[0]
        delivery = events[-1]
        pairs = list(zip(events[1:-1:2], events[2:-1:2]))

        if send.node != key.source:
            _issue(
                issues,
                "send_node_mismatch",
                f"app_send node {send.node} does not equal source {key.source}",
                [send],
            )
        if delivery.node != key.destination:
            _issue(
                issues,
                "delivery_node_mismatch",
                f"delivery node {delivery.node} does not equal destination "
                f"{key.destination}",
                [delivery],
            )

        path = [pairs[0][0].node]
        for pair_index, (enqueue, forward) in enumerate(pairs):
            if enqueue.node != forward.node:
                _issue(
                    issues,
                    "enqueue_forward_node_mismatch",
                    f"enqueue node {enqueue.node} is followed by forward node "
                    f"{forward.node}",
                    [enqueue, forward],
                )
            residence = forward.time_s - enqueue.time_s
            if residence < 0.0:
                _issue(
                    issues,
                    "negative_nwk_residence",
                    f"NWK residence is {residence} seconds",
                    [enqueue, forward],
                )
            nwk_residences.append(
                {
                    "node": enqueue.node,
                    "enqueue_time_s": enqueue.time_s,
                    "forward_time_s": forward.time_s,
                    "residence_s": residence,
                }
            )

            next_event = events[2 * pair_index + 3]
            expected_next_node = next_event.node
            if forward.next_hop != expected_next_node:
                _issue(
                    issues,
                    "next_hop_mismatch",
                    f"forward next_hop {forward.next_hop} does not equal next "
                    f"observed node {expected_next_node}",
                    [forward, next_event],
                )
            transit = next_event.time_s - forward.time_s
            if transit < 0.0:
                _issue(
                    issues,
                    "negative_leg_transit",
                    f"leg transit is {transit} seconds",
                    [forward, next_event],
                )
            leg_transits.append(
                {
                    "from": forward.node,
                    "to": expected_next_node,
                    "forward_time_s": forward.time_s,
                    "arrival_time_s": next_event.time_s,
                    "arrival_event": next_event.event,
                    "transit_s": transit,
                }
            )
            path.append(expected_next_node)

            if forward.detail != "route=reverse":
                route_state = _route_state_before(
                    route_changes, forward.node, key.destination, forward.event_index
                )
                if route_state is not None and (
                    not route_state.selected
                    or route_state.next_hop != forward.next_hop
                ):
                    route_context_mismatch_count += 1
                    _issue(
                        issues,
                        "route_context_mismatch",
                        f"forward next_hop {forward.next_hop} disagrees with "
                        f"prior route state selected={route_state.selected} "
                        f"next_hop={route_state.next_hop}",
                        [forward],
                    )

        if path[0] != key.source:
            _issue(
                issues,
                "path_source_mismatch",
                f"path starts at {path[0]}, expected source {key.source}",
                [pairs[0][0]],
            )
        if path[-1] != key.destination:
            _issue(
                issues,
                "path_destination_mismatch",
                f"path ends at {path[-1]}, expected destination {key.destination}",
                [delivery],
            )
        repeated_nodes = sorted(node for node, count in Counter(path).items() if count > 1)
        if repeated_nodes:
            _issue(
                issues,
                "routing_loop",
                "path repeats node(s): " + ", ".join(map(str, repeated_nodes)),
                events,
            )

        hop_count = len(leg_transits)
        send_time_s = send.time_s
        local_enqueue_time_s = pairs[0][0].time_s
        delivery_time_s = delivery.time_s
        end_to_end_s = delivery.time_s - send.time_s
        app_to_local_enqueue_s = pairs[0][0].time_s - send.time_s
        if end_to_end_s < 0.0 or app_to_local_enqueue_s < 0.0:
            _issue(
                issues,
                "negative_source_timing",
                "application send occurs after enqueue or delivery",
                [send, pairs[0][0], delivery],
            )
        total_nwk_residence_s = sum(
            float(item["residence_s"]) for item in nwk_residences
        )
        total_leg_transit_s = sum(float(item["transit_s"]) for item in leg_transits)
        decomposition_s = (
            app_to_local_enqueue_s
            + total_nwk_residence_s
            + total_leg_transit_s
        )
        decomposition_error_s = abs(end_to_end_s - decomposition_s)
        if decomposition_error_s > tolerance_s:
            _issue(
                issues,
                "end_to_end_decomposition_mismatch",
                f"decomposition error {decomposition_error_s} exceeds tolerance "
                f"{tolerance_s}",
                events,
            )
    elif valid_undelivered_prefix:
        # A run can end with an otherwise legitimate packet held in NWK or in
        # flight.  Validate every portion that is observable even though no
        # end-to-end decomposition is possible without a delivery.
        send = events[0]
        send_time_s = send.time_s
        if send.node != key.source:
            _issue(
                issues,
                "send_node_mismatch",
                f"app_send node {send.node} does not equal source {key.source}",
                [send],
            )
        if len(events) > 1:
            first_enqueue = events[1]
            local_enqueue_time_s = first_enqueue.time_s
            app_to_local_enqueue_s = first_enqueue.time_s - send.time_s
            path = [first_enqueue.node]
            if first_enqueue.node != key.source:
                _issue(
                    issues,
                    "path_source_mismatch",
                    f"path starts at {first_enqueue.node}, expected source "
                    f"{key.source}",
                    [first_enqueue],
                )

        pairs = list(zip(events[1::2], events[2::2]))
        for pair_index, (enqueue, forward) in enumerate(pairs):
            if enqueue.node != forward.node:
                _issue(
                    issues,
                    "enqueue_forward_node_mismatch",
                    f"enqueue node {enqueue.node} is followed by forward node "
                    f"{forward.node}",
                    [enqueue, forward],
                )
            residence = forward.time_s - enqueue.time_s
            if residence < 0.0:
                _issue(
                    issues,
                    "negative_nwk_residence",
                    f"NWK residence is {residence} seconds",
                    [enqueue, forward],
                )
            nwk_residences.append(
                {
                    "node": enqueue.node,
                    "enqueue_time_s": enqueue.time_s,
                    "forward_time_s": forward.time_s,
                    "residence_s": residence,
                }
            )

            next_position = 2 * pair_index + 3
            if next_position < len(events):
                next_event = events[next_position]
                if forward.next_hop != next_event.node:
                    _issue(
                        issues,
                        "next_hop_mismatch",
                        f"forward next_hop {forward.next_hop} does not equal next "
                        f"observed node {next_event.node}",
                        [forward, next_event],
                    )
                transit = next_event.time_s - forward.time_s
                if transit < 0.0:
                    _issue(
                        issues,
                        "negative_leg_transit",
                        f"leg transit is {transit} seconds",
                        [forward, next_event],
                    )
                leg_transits.append(
                    {
                        "from": forward.node,
                        "to": next_event.node,
                        "forward_time_s": forward.time_s,
                        "arrival_time_s": next_event.time_s,
                        "arrival_event": next_event.event,
                        "transit_s": transit,
                    }
                )
                path.append(next_event.node)
            elif forward.next_hop in path:
                _issue(
                    issues,
                    "routing_loop",
                    f"terminal forward selects already visited node "
                    f"{forward.next_hop}",
                    [forward],
                )

            if forward.detail != "route=reverse":
                route_state = _route_state_before(
                    route_changes, forward.node, key.destination, forward.event_index
                )
                if route_state is not None and (
                    not route_state.selected
                    or route_state.next_hop != forward.next_hop
                ):
                    route_context_mismatch_count += 1
                    _issue(
                        issues,
                        "route_context_mismatch",
                        f"forward next_hop {forward.next_hop} disagrees with "
                        f"prior route state selected={route_state.selected} "
                        f"next_hop={route_state.next_hop}",
                        [forward],
                    )

        repeated_nodes = sorted(node for node, count in Counter(path).items() if count > 1)
        if repeated_nodes:
            _issue(
                issues,
                "routing_loop",
                "path repeats node(s): " + ", ".join(map(str, repeated_nodes)),
                events,
            )
        hop_count = len(leg_transits)
        if nwk_residences:
            total_nwk_residence_s = sum(
                float(item["residence_s"]) for item in nwk_residences
            )
        if leg_transits:
            total_leg_transit_s = sum(
                float(item["transit_s"]) for item in leg_transits
            )

    first_index = min(event.event_index for event in events)
    lifecycle_start = sends[0].event_index if sends else first_index
    lifecycle_stop = deliveries[-1].event_index if deliveries else max(
        event.event_index for event in events
    )
    observed_nodes = set(path) if path else {event.node for event in events}
    route_change_count = sum(
        1
        for (node, _), changes in route_changes.items()
        for change in changes
        if node in observed_nodes
        if change.destination == key.destination
        and lifecycle_start <= change.event_index <= lifecycle_stop
    )

    nonfatal_issue_codes = {"missing_delivery", "undelivered_lifecycle"}
    has_validation_error = any(
        str(issue["code"]) not in nonfatal_issue_codes for issue in issues
    )
    return PacketAnalysis(
        key=key,
        first_event_index=first_index,
        valid=not has_validation_error,
        complete=complete,
        delivered=delivered,
        send_time_s=send_time_s,
        local_enqueue_time_s=local_enqueue_time_s,
        delivery_time_s=delivery_time_s,
        end_to_end_s=end_to_end_s,
        app_to_local_enqueue_s=app_to_local_enqueue_s,
        path=path,
        hop_count=hop_count,
        nwk_residences=nwk_residences,
        leg_transits=leg_transits,
        total_nwk_residence_s=total_nwk_residence_s,
        total_leg_transit_s=total_leg_transit_s,
        decomposition_s=decomposition_s,
        decomposition_error_s=decomposition_error_s,
        route_change_count=route_change_count,
        route_context_mismatch_count=route_context_mismatch_count,
        issues=issues,
    )


def _numeric_summary(values: Iterable[float]) -> dict[str, float | int | None]:
    collected = list(values)
    if not collected:
        return {"count": 0, "sum": 0.0, "mean": None, "minimum": None, "maximum": None}
    total = sum(collected)
    return {
        "count": len(collected),
        "sum": total,
        "mean": total / len(collected),
        "minimum": min(collected),
        "maximum": max(collected),
    }


def _bucket_index(time_s: float, bucket_width_s: float) -> int:
    """Match aggregate-ns3-trace.py's stable half-open bucket mapping."""

    quotient = time_s / bucket_width_s
    nearest = round(quotient)
    if math.isclose(quotient, nearest, rel_tol=0.0, abs_tol=1.0e-12):
        quotient = float(nearest)
    return math.floor(quotient)


def _cohort_summary(packets: list[PacketAnalysis]) -> dict[str, object]:
    node_residences: dict[int, list[float]] = defaultdict(list)
    for packet in packets:
        for residence in packet.nwk_residences:
            node_residences[int(residence["node"])].append(
                float(residence["residence_s"])
            )
    return {
        "packet_count": len(packets),
        "end_to_end_s": _numeric_summary(
            packet.end_to_end_s
            for packet in packets
            if packet.end_to_end_s is not None
        ),
        "app_to_local_enqueue_s": _numeric_summary(
            packet.app_to_local_enqueue_s
            for packet in packets
            if packet.app_to_local_enqueue_s is not None
        ),
        "total_nwk_residence_s": _numeric_summary(
            packet.total_nwk_residence_s
            for packet in packets
            if packet.total_nwk_residence_s is not None
        ),
        "total_leg_transit_s": _numeric_summary(
            packet.total_leg_transit_s
            for packet in packets
            if packet.total_leg_transit_s is not None
        ),
        "hop_count": _numeric_summary(
            float(packet.hop_count)
            for packet in packets
            if packet.hop_count is not None
        ),
        "per_node_nwk_residence_s": [
            {"node": node, **_numeric_summary(values)}
            for node, values in sorted(node_residences.items())
        ],
    }


def _path_distribution(
    packets: list[PacketAnalysis], post_startup: set[PacketKey]
) -> list[dict[str, object]]:
    counts: Counter[tuple[int, ...]] = Counter(tuple(packet.path) for packet in packets)
    post_counts: Counter[tuple[int, ...]] = Counter(
        tuple(packet.path) for packet in packets if packet.key in post_startup
    )
    return [
        {
            "path": list(path),
            "hop_count": len(path) - 1,
            "count": count,
            "post_startup_count": post_counts[path],
        }
        for path, count in sorted(counts.items())
    ]


def _flow_distribution(
    packets: list[PacketAnalysis], post_startup: set[PacketKey]
) -> list[dict[str, object]]:
    flows: dict[tuple[int, int], list[PacketAnalysis]] = defaultdict(list)
    for packet in packets:
        flows[(packet.key.source, packet.key.destination)].append(packet)
    rows: list[dict[str, object]] = []
    for (source, destination), flow_packets in sorted(flows.items()):
        flow_post = [packet for packet in flow_packets if packet.key in post_startup]
        rows.append(
            {
                "src": source,
                "dst": destination,
                "packet_count": len(flow_packets),
                "post_startup_packet_count": len(flow_post),
                "end_to_end_s": _numeric_summary(
                    packet.end_to_end_s
                    for packet in flow_packets
                    if packet.end_to_end_s is not None
                ),
                "post_startup_end_to_end_s": _numeric_summary(
                    packet.end_to_end_s
                    for packet in flow_post
                    if packet.end_to_end_s is not None
                ),
                "paths": _path_distribution(flow_packets, post_startup),
            }
        )
    return rows


def _end_to_end_buckets(
    packets: list[PacketAnalysis],
    bucket_width_s: float,
    stop_time_s: float | None = None,
) -> list[dict[str, object]]:
    if stop_time_s is not None:
        bucket_count = round(stop_time_s / bucket_width_s)
    elif packets:
        bucket_count = (
            max(
                _bucket_index(packet.delivery_time_s, bucket_width_s)
                for packet in packets
                if packet.delivery_time_s is not None
            )
            + 1
        )
    else:
        return []
    values: list[list[float]] = [[] for _ in range(bucket_count)]
    for packet in packets:
        if packet.delivery_time_s is None or packet.end_to_end_s is None:
            continue
        index = _bucket_index(packet.delivery_time_s, bucket_width_s)
        if index < 0 or index >= bucket_count:
            raise PathAnalysisError(
                f"delivery at {packet.delivery_time_s} maps outside configured "
                f"E2E bucket window"
            )
        values[index].append(packet.end_to_end_s)
    return [
        {
            "bucket_index": index,
            "start_time_s": index * bucket_width_s,
            "end_time_s": (index + 1) * bucket_width_s,
            "packet_count": len(bucket),
            "mean_end_to_end_s": (sum(bucket) / len(bucket)) if bucket else None,
        }
        for index, bucket in enumerate(values)
    ]


def analyze_trace(
    trace_path: Path,
    startup_time_s: float = 300.0,
    bucket_width_s: float = 60.0,
    tolerance_s: float = 1.0e-9,
    stop_time_s: float | None = None,
) -> tuple[list[PacketAnalysis], dict[str, object]]:
    """Analyze a trace and return packet records plus a JSON-ready report."""

    if not math.isfinite(startup_time_s) or startup_time_s < 0.0:
        raise PathAnalysisError("startup_time_s must be finite and non-negative")
    if not math.isfinite(bucket_width_s) or bucket_width_s <= 0.0:
        raise PathAnalysisError("bucket_width_s must be finite and positive")
    if not math.isfinite(tolerance_s) or tolerance_s < 0.0:
        raise PathAnalysisError("tolerance_s must be finite and non-negative")
    if stop_time_s is not None:
        if not math.isfinite(stop_time_s) or stop_time_s <= 0.0:
            raise PathAnalysisError("stop_time_s must be finite and positive")
        bucket_quotient = stop_time_s / bucket_width_s
        if not math.isclose(
            bucket_quotient,
            round(bucket_quotient),
            rel_tol=1.0e-12,
            abs_tol=1.0e-12,
        ):
            raise PathAnalysisError(
                "stop_time_s must be an integer multiple of bucket_width_s"
            )

    loaded = load_trace(trace_path)
    packets = [
        analyze_packet(key, events, loaded.route_changes, tolerance_s)
        for key, events in loaded.packets.items()
    ]
    packets.sort(key=lambda packet: (packet.first_event_index, packet.key))
    valid_complete = [
        packet
        for packet in packets
        if packet.valid and packet.complete and packet.delivered
    ]
    post_startup_packets = [
        packet
        for packet in valid_complete
        if packet.delivery_time_s is not None and packet.delivery_time_s > startup_time_s
    ]
    bucket_packets: list[PacketAnalysis] = []
    excluded_at_stop = 0
    excluded_after_stop = 0
    if stop_time_s is None:
        bucket_packets = list(valid_complete)
    else:
        stop_bucket = round(stop_time_s / bucket_width_s)
        for packet in valid_complete:
            if packet.delivery_time_s is None:  # pragma: no cover - invariant.
                raise AssertionError("complete delivery has no delivery time")
            delivery_bucket = _bucket_index(
                packet.delivery_time_s, bucket_width_s
            )
            if delivery_bucket < stop_bucket:
                bucket_packets.append(packet)
            elif delivery_bucket == stop_bucket:
                # Boundary snapping uses the same quotient tolerance as the
                # bucket mapper, preventing a near-boundary packet from being
                # counted in-window and then silently discarded.
                quotient = packet.delivery_time_s / bucket_width_s
                if math.isclose(
                    quotient,
                    stop_bucket,
                    rel_tol=0.0,
                    abs_tol=1.0e-12,
                ):
                    excluded_at_stop += 1
                else:
                    excluded_after_stop += 1
            else:
                excluded_after_stop += 1
    post_keys = {packet.key for packet in post_startup_packets}
    issue_counts: Counter[str] = Counter(
        str(issue["code"]) for packet in packets for issue in packet.issues
    )
    invalid_packets = [packet for packet in packets if not packet.valid]
    incomplete_packets = [packet for packet in packets if not packet.complete]
    incomplete_delivered = [
        packet for packet in packets if packet.delivered and not packet.complete
    ]

    try:
        digest = hashlib.sha256(trace_path.read_bytes()).hexdigest()
    except OSError as error:
        raise PathAnalysisError(str(error)) from error

    report: dict[str, object] = {
        "schema": REPORT_SCHEMA,
        # A trace can legitimately end with packets queued or in flight.  Strict
        # acceptance therefore requires every delivered lifecycle (and every
        # observed prefix) to be internally valid, but does not turn an
        # otherwise valid undelivered prefix into a failure.
        "pass": not invalid_packets,
        "input": {
            "path": str(trace_path.resolve()),
            "sha256": digest,
            "row_count": loaded.row_count,
            "event_counts": dict(sorted(loaded.event_counts.items())),
        },
        "configuration": {
            "packet_key": ["src", "dst", "sequence"],
            "startup_time_s": startup_time_s,
            "post_startup_selection": "delivery_time_s > startup_time_s",
            "end_to_end_bucket_width_s": bucket_width_s,
            "end_to_end_stop_time_s": stop_time_s,
            "end_to_end_stop_time_semantics": "exclusive",
            "bucket_boundary_quotient_tolerance": 1.0e-12,
            "decomposition_tolerance_s": tolerance_s,
            "hop_resend_lifetime_is_additive": False,
        },
        "counts": {
            "packet_keys": len(packets),
            "delivered_packets": sum(packet.delivered for packet in packets),
            "valid_complete_delivered_packets": len(valid_complete),
            "post_startup_valid_complete_delivered_packets": len(
                post_startup_packets
            ),
            "invalid_packets": len(invalid_packets),
            "incomplete_packets": len(incomplete_packets),
            "incomplete_delivered_packets": len(incomplete_delivered),
            "undelivered_packets": sum(not packet.delivered for packet in packets),
            "route_change_events": sum(
                len(changes) for changes in loaded.route_changes.values()
            ),
            "route_context_mismatch_packets": sum(
                packet.route_context_mismatch_count > 0 for packet in packets
            ),
            "route_context_mismatches": sum(
                packet.route_context_mismatch_count for packet in packets
            ),
            "end_to_end_bucket_packets": len(bucket_packets),
            "end_to_end_bucket_excluded_at_stop_packets": excluded_at_stop,
            "end_to_end_bucket_excluded_after_stop_packets": excluded_after_stop,
        },
        "issue_counts": dict(sorted(issue_counts.items())),
        "issue_examples": [
            {
                "src": packet.key.source,
                "dst": packet.key.destination,
                "sequence": packet.key.sequence,
                "issues": packet.issues,
            }
            for packet in invalid_packets[:20]
        ],
        "incomplete_examples": [
            {
                "src": packet.key.source,
                "dst": packet.key.destination,
                "sequence": packet.key.sequence,
                "delivered": packet.delivered,
                "valid": packet.valid,
                "issues": packet.issues,
            }
            for packet in incomplete_packets[:20]
        ],
        "overall": _cohort_summary(valid_complete),
        "post_startup": _cohort_summary(post_startup_packets),
        "path_distribution": _path_distribution(valid_complete, post_keys),
        "flow_distribution": _flow_distribution(valid_complete, post_keys),
        "end_to_end_buckets": _end_to_end_buckets(
            bucket_packets, bucket_width_s, stop_time_s
        ),
    }
    return packets, report


def _packet_status(packet: PacketAnalysis) -> str:
    if packet.valid and packet.complete and packet.delivered:
        return "valid_delivered"
    if packet.delivered:
        if not packet.valid and not packet.complete:
            return "invalid_incomplete_delivered"
        if not packet.complete:
            return "incomplete_delivered"
        return "invalid_delivered"
    if packet.valid:
        return "incomplete_undelivered"
    return "invalid_incomplete_undelivered"


def _csv_number(value: float | int | None) -> str:
    return "" if value is None else repr(value)


def packet_rows(
    packets: Iterable[PacketAnalysis], startup_time_s: float
) -> list[dict[str, str]]:
    """Convert packet analyses to stable machine-readable CSV rows."""

    rows: list[dict[str, str]] = []
    for packet in packets:
        post_startup = (
            packet.delivery_time_s is not None
            and packet.delivery_time_s > startup_time_s
        )
        rows.append(
            {
                "src": str(packet.key.source),
                "dst": str(packet.key.destination),
                "sequence": str(packet.key.sequence),
                "status": _packet_status(packet),
                "valid": "1" if packet.valid else "0",
                "complete": "1" if packet.complete else "0",
                "delivered": "1" if packet.delivered else "0",
                "send_time_s": _csv_number(packet.send_time_s),
                "local_enqueue_time_s": _csv_number(
                    packet.local_enqueue_time_s
                ),
                "delivery_time_s": _csv_number(packet.delivery_time_s),
                "end_to_end_s": _csv_number(packet.end_to_end_s),
                "app_to_local_enqueue_s": _csv_number(
                    packet.app_to_local_enqueue_s
                ),
                "path_json": json.dumps(packet.path, separators=(",", ":")),
                "hop_count": _csv_number(packet.hop_count),
                "nwk_residences_json": json.dumps(
                    packet.nwk_residences, separators=(",", ":"), sort_keys=True
                ),
                "leg_transits_json": json.dumps(
                    packet.leg_transits, separators=(",", ":"), sort_keys=True
                ),
                "total_nwk_residence_s": _csv_number(
                    packet.total_nwk_residence_s
                ),
                "total_leg_transit_s": _csv_number(packet.total_leg_transit_s),
                "decomposition_s": _csv_number(packet.decomposition_s),
                "decomposition_error_s": _csv_number(
                    packet.decomposition_error_s
                ),
                "post_startup": "1" if post_startup else "0",
                "route_change_count": str(packet.route_change_count),
                "route_context_mismatch_count": str(
                    packet.route_context_mismatch_count
                ),
                "issue_count": str(len(packet.issues)),
                "issues_json": json.dumps(
                    packet.issues, separators=(",", ":"), sort_keys=True
                ),
            }
        )
    return rows


def write_outputs(
    packets: list[PacketAnalysis],
    report: dict[str, object],
    summary_path: Path,
    packet_csv_path: Path,
) -> None:
    """Write the summary and packet ledger after validating output paths."""

    resolved_summary = summary_path.resolve()
    resolved_packets = packet_csv_path.resolve()
    if _paths_alias(resolved_summary, resolved_packets):
        raise PathAnalysisError("summary JSON and packet CSV must be different paths")
    resolved_summary.parent.mkdir(parents=True, exist_ok=True)
    resolved_packets.parent.mkdir(parents=True, exist_ok=True)
    startup_time_s = float(report["configuration"]["startup_time_s"])  # type: ignore[index]
    with resolved_packets.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=PACKET_CSV_COLUMNS, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(packet_rows(packets, startup_time_s))
    report["outputs"] = {
        "packet_csv": {
            "path": str(resolved_packets),
            "sha256": hashlib.sha256(resolved_packets.read_bytes()).hexdigest(),
            "row_count": len(packets),
        }
    }
    resolved_summary.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="csr-differential-trace-v1 CSV")
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--packet-csv", type=Path, required=True)
    parser.add_argument(
        "--startup-time",
        type=float,
        default=300.0,
        help="post-startup deliveries are strictly later than this time (default: 300)",
    )
    parser.add_argument(
        "--bucket-width",
        type=float,
        default=60.0,
        help="end-to-end bucket width in seconds (default: 60)",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1.0e-9,
        help="maximum end-to-end decomposition error in seconds",
    )
    parser.add_argument(
        "--stop-time",
        type=float,
        help=(
            "exclusive E2E bucket stop time; must be an integer multiple of "
            "--bucket-width (packet ledger still includes later packets)"
        ),
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help=(
            "exit nonzero after writing evidence if a delivered chain or "
            "observed packet prefix is invalid; valid undelivered prefixes pass"
        ),
    )
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    try:
        trace_path = arguments.trace.resolve()
        summary_path = arguments.summary_json.resolve()
        packet_csv_path = arguments.packet_csv.resolve()
        _validate_cli_paths(trace_path, summary_path, packet_csv_path)
        packets, report = analyze_trace(
            trace_path,
            startup_time_s=arguments.startup_time,
            bucket_width_s=arguments.bucket_width,
            tolerance_s=arguments.tolerance,
            stop_time_s=arguments.stop_time,
        )
        write_outputs(packets, report, summary_path, packet_csv_path)
    except (OSError, csv.Error, PathAnalysisError) as error:
        raise SystemExit(f"error: {error}") from error

    counts = report["counts"]
    print(
        f"{'PASS' if report['pass'] else 'FAIL'}: "
        f"packets={counts['packet_keys']} "
        f"valid_delivered={counts['valid_complete_delivered_packets']} "
        f"invalid={counts['invalid_packets']} "
        f"incomplete={counts['incomplete_packets']}"
    )
    if arguments.strict and not report["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
