#!/usr/bin/env python3
"""Validate lifecycle and collision evidence in an instrumented OPNET trace."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import sys


def load_comparator():
    path = Path(__file__).with_name("compare-opnet-ns3-traces.py")
    specification = importlib.util.spec_from_file_location(
        "csr_trace_comparator_for_validation", path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load comparator from {path}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


COMPARATOR = load_comparator()


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def parse_transmitters(value: str) -> tuple[int, ...]:
    try:
        transmitters = tuple(int(item, 0) for item in value.split(",") if item)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if len(transmitters) < 2 or len(set(transmitters)) != len(transmitters):
        raise argparse.ArgumentTypeError("provide at least two unique transmitters")
    return transmitters


def number(event, field: str) -> float | None:
    value = event.values[field]
    return None if value == "" else float(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--receiver", required=True, type=int)
    parser.add_argument("--transmitters", required=True, type=parse_transmitters)
    parser.add_argument("--slot", required=True, type=int)
    parser.add_argument("--time-start", type=float)
    parser.add_argument("--time-stop", type=float)
    parser.add_argument("--simultaneous-tolerance", type=float, default=1.0e-6)
    parser.add_argument("--holdoff-tolerance", type=float, default=1.0e-6)
    parser.add_argument("--report", type=Path)
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    if not arguments.trace.is_file() or not arguments.manifest.is_file():
        raise SystemExit("error: trace and instrumentation manifest must exist")
    if arguments.slot < 1 or arguments.slot > 255:
        raise SystemExit("error: --slot must be in 1..255")
    for value, label in (
        (arguments.simultaneous_tolerance, "simultaneous tolerance"),
        (arguments.holdoff_tolerance, "holdoff tolerance"),
    ):
        if not math.isfinite(value) or value < 0.0:
            raise SystemExit(f"error: {label} must be finite and nonnegative")
    manifest = json.loads(arguments.manifest.read_text(encoding="utf-8"))
    if manifest.get("schema") != "csr-opnet-instrumentation-v1":
        raise SystemExit("error: manifest is not csr-opnet-instrumentation-v1")
    forced = manifest.get("forced_reservation_slots", {})
    activation_time = manifest.get("activation_time_s")
    if not isinstance(activation_time, (int, float)) or not math.isfinite(
        activation_time
    ):
        raise SystemExit("error: manifest activation_time_s is not finite")
    for node in arguments.transmitters:
        if forced.get(str(node)) != arguments.slot:
            raise SystemExit(
                f"error: manifest does not force node {node} to slot {arguments.slot}"
            )

    events = COMPARATOR.filter_trace(
        COMPARATOR.normalize_trace(arguments.trace),
        set(),
        arguments.time_start,
        arguments.time_stop,
    )
    failures: list[str] = []
    evidence: dict[str, object] = {}
    tx_times: list[float] = []
    expected_counters = list(range(arguments.slot - 1, -2, -1))

    for node in arguments.transmitters:
        node_text = str(node)
        node_events = [event for event in events if event.values["node"] == node_text]
        lifecycle = None
        for prepare_index, prepare in enumerate(node_events):
            if (
                prepare.values["event"] != "reservation_prepare"
                or number(prepare, "reservation_slot") != arguments.slot
                or prepare.values["reason"]
                not in {"controlled_ready", "controlled_wait"}
            ):
                continue
            wait_for_holdoff = prepare.values["reason"] == "controlled_wait"
            holdoff_index = None
            if wait_for_holdoff:
                holdoff_index = next(
                    (
                        index
                        for index in range(prepare_index + 1, len(node_events))
                        if node_events[index].values["event"]
                        == "reservation_holdoff"
                    ),
                    None,
                )
                if holdoff_index is None:
                    continue
            cursor = (
                prepare_index + 1
                if holdoff_index is None
                else holdoff_index + 1
            )
            tick_events = []
            valid = True
            for expected_counter in expected_counters:
                while (
                    cursor < len(node_events)
                    and node_events[cursor].values["event"]
                    != "reservation_tick"
                ):
                    cursor += 1
                if cursor >= len(node_events):
                    valid = False
                    break
                tick = node_events[cursor]
                if number(tick, "reservation_slot") != arguments.slot or number(
                    tick, "reservation_counter"
                ) != expected_counter:
                    valid = False
                    break
                tick_events.append(tick)
                cursor += 1
            if not valid:
                continue
            advertisement_index = next(
                (
                    index
                    for index in range(cursor, len(node_events))
                    if node_events[index].values["event"]
                    == "reservation_advertise"
                ),
                None,
            )
            if advertisement_index is None:
                continue
            advertisement = node_events[advertisement_index]
            if number(advertisement, "reservation_slot") != arguments.slot:
                continue
            transmission_index = next(
                (
                    index
                    for index in range(advertisement_index + 1, len(node_events))
                    if node_events[index].values["event"] == "tx_start"
                    and node_events[index].values["peer"]
                    in {"", str(arguments.receiver)}
                ),
                None,
            )
            if transmission_index is None:
                continue
            lifecycle = {
                "prepare": prepare,
                "holdoff": (
                    None if holdoff_index is None else node_events[holdoff_index]
                ),
                "holdoff_mode": "wait" if wait_for_holdoff else "ready",
                "ticks": tick_events,
                "advertisement": advertisement,
                "transmission": node_events[transmission_index],
            }
            break

        if lifecycle is None:
            failures.append(
                f"node {node}: no ordered controlled prepare/gate/countdown/"
                "advertise/transmit lifecycle"
            )
            evidence[str(node)] = {"lifecycle_found": False}
            continue
        prepare_time = float(lifecycle["prepare"].values["time_s"])
        holdoff = lifecycle["holdoff"]
        holdoff_time = (
            None if holdoff is None else float(holdoff.values["time_s"])
        )
        holdoff_duration = (
            None if holdoff_time is None else holdoff_time - prepare_time
        )
        if (
            holdoff_duration is not None
            and abs(holdoff_duration - 0.3) > arguments.holdoff_tolerance
        ):
            failures.append(
                f"node {node}: holdoff duration {holdoff_duration} is not 0.3 s"
            )
        if prepare_time < activation_time:
            failures.append(
                f"node {node}: controlled preparation precedes manifest activation"
            )
        transmission = lifecycle["transmission"]
        tx_time = float(transmission.values["time_s"])
        tx_times.append(tx_time)
        evidence[str(node)] = {
            "lifecycle_found": True,
            "prepare_row": lifecycle["prepare"].input_row,
            "prepare_time_s": prepare_time,
            "holdoff_mode": lifecycle["holdoff_mode"],
            "holdoff_row": None if holdoff is None else holdoff.input_row,
            "holdoff_time_s": holdoff_time,
            "holdoff_duration_s": holdoff_duration,
            "tick_rows": [event.input_row for event in lifecycle["ticks"]],
            "tick_counters": expected_counters,
            "advertise_row": lifecycle["advertisement"].input_row,
            "tx_row": transmission.input_row,
            "tx_time_s": tx_time,
        }

    if len(tx_times) == len(arguments.transmitters):
        spread = max(tx_times) - min(tx_times)
        if spread > arguments.simultaneous_tolerance:
            failures.append(
                f"transmit start spread {spread} exceeds simultaneous tolerance"
            )
    else:
        spread = None

    receiver_events = [
        event
        for event in events
        if event.values["node"] == str(arguments.receiver)
        and event.values["event"] in {"rx_accept", "rx_drop"}
        and event.values["peer"] in {str(node) for node in arguments.transmitters}
    ]
    collision_events = [
        event
        for event in receiver_events
        if re.search(r"(?:^|;)collisions=[1-9][0-9]*(?:;|$)", event.values["detail"])
    ]
    drops = [event for event in receiver_events if event.values["event"] == "rx_drop"]
    receiver_evidence: dict[str, object] = {}
    for node in arguments.transmitters:
        peer_events = [
            event for event in receiver_events if event.values["peer"] == str(node)
        ]
        peer_collisions = [event for event in collision_events if event in peer_events]
        peer_drops = [event for event in drops if event in peer_events]
        if not peer_events:
            failures.append(f"receiver has no outcome for transmitter {node}")
        if not peer_collisions:
            failures.append(
                f"receiver has no positive collision count for transmitter {node}"
            )
        if not peer_drops:
            failures.append(f"receiver has no drop for transmitter {node}")
        receiver_evidence[str(node)] = {
            "outcome_rows": [event.input_row for event in peer_events],
            "collision_rows": [event.input_row for event in peer_collisions],
            "drop_rows": [event.input_row for event in peer_drops],
        }

    report = {
        "schema": "csr-reservation-collision-validation-v1",
        "pass": not failures,
        "trace": str(arguments.trace.resolve()),
        "trace_sha256": digest(arguments.trace),
        "manifest": str(arguments.manifest.resolve()),
        "manifest_sha256": digest(arguments.manifest),
        "receiver": arguments.receiver,
        "transmitters": list(arguments.transmitters),
        "forced_slot": arguments.slot,
        "time_window": {"start": arguments.time_start, "stop": arguments.time_stop},
        "activation_time_s": activation_time,
        "tx_time_spread_s": spread,
        "receiver_outcome_count": len(receiver_events),
        "positive_collision_count": len(collision_events),
        "drop_count": len(drops),
        "transmitter_evidence": evidence,
        "receiver_evidence": receiver_evidence,
        "failures": failures,
    }
    if arguments.report is not None:
        arguments.report.parent.mkdir(parents=True, exist_ok=True)
        arguments.report.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(
        f"{'PASS' if report['pass'] else 'FAIL'}: "
        f"tx={len(tx_times)}/{len(arguments.transmitters)} "
        f"rx={len(receiver_events)} collisions={len(collision_events)} "
        f"drops={len(drops)}"
    )
    for failure in failures:
        print(f"- {failure}")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
