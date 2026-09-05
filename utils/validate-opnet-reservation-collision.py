#!/usr/bin/env python3
"""Validate lifecycle and collision evidence in an instrumented OPNET trace."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import sys


NIBBLE_BITS = 4.0
S0_DURATION_S = 0.000510
RATE_NIBBLE_DURATION_S = {
    8: S0_DURATION_S,
    16: 0.000254,
    32: 0.000126,
    64: 0.000062,
    128: 0.000030,
}
PREAMBLE_BITS = {"long": 7888, "short": 104}
PHY_HEADER_BITS = 48
FCS_BITS = 32
MAX_COMPLETION_TOLERANCE_S = 1.0e-3
SLOT_CYCLE_S = 0.013
CONTROL_EPOCH_S = 0.1
HOLDOFF_S = 0.3
MAX_STRUCTURAL_TOLERANCE_S = 1.0e-6
EXPECTED_ACTIVATION_TIME_S = 59.9
MAX_SEQUENCE = 65535
INSTRUMENTATION_SCHEMA = "csr-opnet-instrumentation-v1"
TRACE_SCHEMA = "csr-differential-trace-v1"
EXPECTED_ORIGINAL_SHA256 = {
    "br_app.pr.c": "0e3dfeefc90da99b78f1b6305ccffa765de4b35710414bd85aa9fc30d8bb6dde",
    "br_mac.pr.c": "d1bc9f7d57da4cc490a835e19b26eac47dc0088629ecd15abfaaa5a0734be27a",
    "br_ecc.ps.c": "fbc5d5fb63c08eb3a5b2afec0baacf7c59182cf07382ed5fd3a61bd61f53d83c",
    "br_support.h": "9855b95029a2e15190976ef5ae926d028a98ef91db5d7567597c5f01cc7f37ce",
    "br_txdel.ps.c": "6b9ae9213aea8546b97069df5fe5315e8a7c49203d7d27c3d6e559e452a49463",
}
EXPECTED_PATCHED_OUTPUT_SHA256 = {
    "br_app.pr.c": "1678d9f1c81d8692ab28728feb5ea2c7913e3578c350633a7e5991e6e302ceee",
    "br_mac.pr.c": "726dd449acf7b9670ea941bbaf6115569db6ddf547bc19437f1685c3c09712ae",
    "br_ecc.ps.c": "4d92630cffa8da10a14d02238eb9259dc8d5cb2dd6ffd68ad397ac37432df861",
}
EXPECTED_HEADER_TEMPLATE_SHA256 = (
    "60aac760949eb895304ef1457d8623253ddfb22245b0683fee1bea6b589692e6"
)
HEADER_NAME = "csr-opnet-trace.h"
GENERATOR_NAME = "instrument-opnet-reservation-trace.py"
SOURCE_RECORD_FIELDS = {"original_sha256", "output_sha256"}
MANIFEST_FIELDS = {
    "schema",
    "trace_schema",
    "generator",
    "forced_reservation_slots",
    "activation_time_s",
    "behavioral_control",
    "sources",
}
EXPECTED_BEHAVIORAL_CONTROL = (
    "A test hook loads the selected slot/counter at transmit preparation, "
    "controls future reservation draws, and aligns the first countdown "
    "tick to a common 100-ms epoch after any still-pending source "
    "holdoff. Idle prep_tx holdoff and Search-state holdoff reuse, "
    "countdown steps, transmit, receive, collision, BER, and ECC logic "
    "remain in the recovered sources."
)
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


class ValidationInputError(ValueError):
    """The trace, manifest, or requested validation contract is malformed."""


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
OPNET_TRACE_COLUMNS = COMPARATOR.TRACE_COLUMNS[:-2]
STRICT_INTEGER_FIELDS = (
    "event_index",
    "node",
    "peer",
    "src",
    "dst",
    "sequence",
    "rate_kbps",
    "size_bytes",
    "header_errors",
    "payload_errors",
    "total_errors",
    "next_hop",
    "security_count",
    "reservation_slot",
    "reservation_counter",
)


def digest(path: Path) -> str:
    value = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                value.update(chunk)
    except OSError as error:
        raise ValidationInputError(f"cannot hash {path}: {error}") from error
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
    if value == "":
        return None
    try:
        parsed = float(value)
    except (ValueError, OverflowError) as error:
        raise ValidationInputError(f"{field} is not numeric: {value!r}") from error
    if not math.isfinite(parsed):
        raise ValidationInputError(f"{field} is not finite: {value!r}")
    return parsed


def integer(event, field: str) -> int | None:
    value = event.values[field]
    if value == "":
        return None
    try:
        return int(value)
    except (ValueError, OverflowError) as error:
        raise ValidationInputError(
            f"{field} is not a finite integer: {value!r}"
        ) from error


def parse_detail(value: str) -> dict[str, str]:
    """Parse the instrumentation's strict semicolon-separated key/value data."""

    fields: dict[str, str] = {}
    if not value:
        return fields
    for component in value.split(";"):
        if not component or "=" not in component:
            raise ValidationInputError(f"malformed detail component {component!r}")
        key, item = component.split("=", 1)
        key = key.strip()
        item = item.strip()
        if not key:
            raise ValidationInputError("detail key is empty")
        if key in fields:
            raise ValidationInputError(f"duplicate detail key {key!r}")
        fields[key] = item
    return fields


def source_airtime_s(rate_kbps: int, size_bytes: int, preamble: str) -> float:
    """Reproduce ``br_txdel`` using the recovered nibble durations exactly."""

    if rate_kbps not in RATE_NIBBLE_DURATION_S:
        raise ValidationInputError(
            f"unsupported source-exact rate {rate_kbps} kbps"
        )
    if size_bytes <= 0:
        raise ValidationInputError("transmit size must be a positive byte count")
    if preamble not in PREAMBLE_BITS:
        raise ValidationInputError(
            f"preamble must be long or short, not {preamble!r}"
        )
    fixed_bits = PREAMBLE_BITS[preamble] + PHY_HEADER_BITS
    payload_and_fcs_bits = size_bytes * 8 + FCS_BITS
    return (
        fixed_bits / NIBBLE_BITS * S0_DURATION_S
        + payload_and_fcs_bits
        / NIBBLE_BITS
        * RATE_NIBBLE_DURATION_S[rate_kbps]
    )


def collision_count(event) -> int | None:
    """Return a strict nonnegative collision count from one outcome row."""

    try:
        raw = parse_detail(event.values["detail"]).get("collisions")
    except ValidationInputError:
        return None
    if raw is None or re.fullmatch(r"[0-9]+", raw) is None:
        return None
    return int(raw)


def paths_alias(first: Path, second: Path) -> bool:
    """Return whether two existing or prospective paths address one object."""

    try:
        if first.resolve(strict=False) == second.resolve(strict=False):
            return True
        return first.samefile(second)
    except FileNotFoundError:
        return False
    except (OSError, RuntimeError) as error:
        raise ValidationInputError(f"cannot resolve output identity: {error}") from error


def validate_report_path(report: Path | None, trace: Path, manifest: Path) -> None:
    """Refuse a report target that could overwrite either evidence input."""

    if report is None:
        return
    for label, evidence_path in (("trace", trace), ("manifest", manifest)):
        if paths_alias(report, evidence_path):
            raise ValidationInputError(f"report path aliases {label} input")


def require_sha256(value: object, label: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        raise ValidationInputError(f"{label} is not a lowercase SHA-256 digest")
    return value


def reject_duplicate_json_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValidationInputError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def validate_raw_trace(path: Path) -> None:
    """Require the exact header and row schema emitted by the OPNET writer."""

    try:
        stream = path.open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise ValidationInputError(f"cannot read trace: {error}") from error
    with stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != OPNET_TRACE_COLUMNS:
            raise ValidationInputError("trace does not have the exact OPNET trace header")
        for row_number, row in enumerate(reader, start=2):
            if None in row or any(row.get(name) is None for name in OPNET_TRACE_COLUMNS):
                raise ValidationInputError(
                    f"trace row {row_number} does not match the exact column width"
                )
            if row.get("schema") != TRACE_SCHEMA:
                raise ValidationInputError(
                    f"trace row {row_number} does not declare {TRACE_SCHEMA}"
                )
            try:
                time_s = float(row["time_s"])
            except (ValueError, OverflowError) as error:
                raise ValidationInputError(
                    f"time_s is not numeric: {row['time_s']!r}"
                ) from error
            if not math.isfinite(time_s):
                raise ValidationInputError(
                    f"time_s is not finite: {row['time_s']!r}"
                )


def render_expected_header(
    forced_slots: dict[str, int], activation_time: float
) -> bytes:
    """Recreate the generated header from the locally trusted template."""

    template_path = Path(__file__).with_name("opnet") / HEADER_NAME
    try:
        text = template_path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        raise ValidationInputError(f"cannot read header template: {error}") from error
    slot_marker = "        /* CSR_FORCE_SLOT_CASES */"
    activation_marker = "    /* CSR_CONTROL_START_SECONDS */"
    if text.count(slot_marker) != 1 or text.count(activation_marker) != 1:
        raise ValidationInputError("trusted header template markers are not unique")
    cases = "\n".join(
        f"        case {int(node)}: return {slot};"
        for node, slot in sorted(
            forced_slots.items(), key=lambda item: int(item[0])
        )
    )
    text = text.replace(slot_marker, cases, 1)
    text = text.replace(activation_marker, f"    {activation_time!r}", 1)
    return text.encode("utf-8")


def validate_manifest(
    path: Path, transmitters: tuple[int, ...], slot: int
) -> tuple[dict[str, object], float, dict[str, object]]:
    """Validate source identities and every locally reproducible artifact."""

    try:
        manifest = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=reject_duplicate_json_keys,
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValidationInputError(
            f"cannot read instrumentation manifest: {error}"
        ) from error
    if not isinstance(manifest, dict):
        raise ValidationInputError("instrumentation manifest root is not an object")
    if set(manifest) != MANIFEST_FIELDS:
        missing = sorted(MANIFEST_FIELDS - set(manifest))
        extra = sorted(set(manifest) - MANIFEST_FIELDS)
        raise ValidationInputError(
            f"instrumentation manifest fields mismatch; missing={missing}, extra={extra}"
        )
    if manifest["schema"] != INSTRUMENTATION_SCHEMA:
        raise ValidationInputError(
            f"manifest is not {INSTRUMENTATION_SCHEMA}"
        )
    if manifest["trace_schema"] != TRACE_SCHEMA:
        raise ValidationInputError(f"manifest trace_schema is not {TRACE_SCHEMA}")

    generator = manifest["generator"]
    if not isinstance(generator, dict) or set(generator) != {"name", "sha256"}:
        raise ValidationInputError("manifest generator record has the wrong shape")
    if generator["name"] != GENERATOR_NAME:
        raise ValidationInputError("manifest generator name is not trusted")
    generator_digest = require_sha256(
        generator["sha256"], "manifest generator sha256"
    )
    generator_path = Path(__file__).with_name(GENERATOR_NAME)
    if generator_digest != digest(generator_path):
        raise ValidationInputError("manifest generator digest does not match local tool")

    forced = manifest["forced_reservation_slots"]
    if not isinstance(forced, dict):
        raise ValidationInputError(
            "manifest forced_reservation_slots is not an object"
        )
    for node, forced_slot in forced.items():
        if (
            not isinstance(node, str)
            or not node.isdigit()
            or str(int(node)) != node
            or int(node) > 0xFFFFFF
            or type(forced_slot) is not int
            or forced_slot < 1
            or forced_slot > 255
        ):
            raise ValidationInputError("manifest has a malformed forced slot entry")
    expected_forced = {str(node): slot for node in transmitters}
    if forced != expected_forced:
        raise ValidationInputError(
            f"manifest forced slots {forced!r} do not equal {expected_forced!r}"
        )

    activation_time = manifest["activation_time_s"]
    if (
        type(activation_time) is not float
        or not math.isfinite(activation_time)
        or activation_time < 0.0
    ):
        raise ValidationInputError(
            "manifest activation_time_s is not a finite nonnegative float"
        )
    if activation_time != EXPECTED_ACTIVATION_TIME_S:
        raise ValidationInputError(
            f"manifest activation_time_s is not {EXPECTED_ACTIVATION_TIME_S}"
        )
    if manifest["behavioral_control"] != EXPECTED_BEHAVIORAL_CONTROL:
        raise ValidationInputError("manifest behavioral_control is not trusted")

    sources = manifest["sources"]
    expected_source_names = set(EXPECTED_ORIGINAL_SHA256) | {HEADER_NAME}
    if not isinstance(sources, dict) or set(sources) != expected_source_names:
        raise ValidationInputError(
            "manifest sources do not contain the exact required source set"
        )
    output_digests: dict[str, str] = {}
    for name, expected_original in EXPECTED_ORIGINAL_SHA256.items():
        record = sources[name]
        if not isinstance(record, dict) or set(record) != SOURCE_RECORD_FIELDS:
            raise ValidationInputError(f"manifest source {name} has the wrong shape")
        original = require_sha256(
            record["original_sha256"], f"manifest source {name} original_sha256"
        )
        if original != expected_original:
            raise ValidationInputError(
                f"manifest source {name} original digest is not trusted"
            )
        output = record["output_sha256"]
        if name in EXPECTED_PATCHED_OUTPUT_SHA256:
            output_digest = require_sha256(
                output, f"manifest source {name} output_sha256"
            )
            if output_digest != EXPECTED_PATCHED_OUTPUT_SHA256[name]:
                raise ValidationInputError(
                    f"manifest source {name} generated digest is not trusted"
                )
            output_digests[name] = output_digest
        elif output is not None:
            raise ValidationInputError(
                f"manifest reference source {name} unexpectedly names an output"
            )

    header_record = sources[HEADER_NAME]
    if not isinstance(header_record, dict) or set(header_record) != SOURCE_RECORD_FIELDS:
        raise ValidationInputError(
            f"manifest source {HEADER_NAME} has the wrong shape"
        )
    template_path = Path(__file__).with_name("opnet") / HEADER_NAME
    local_template_digest = digest(template_path)
    if local_template_digest != EXPECTED_HEADER_TEMPLATE_SHA256:
        raise ValidationInputError("local header template digest is not trusted")
    template_digest = require_sha256(
        header_record["original_sha256"],
        f"manifest source {HEADER_NAME} original_sha256",
    )
    if template_digest != EXPECTED_HEADER_TEMPLATE_SHA256:
        raise ValidationInputError("manifest header-template digest is not trusted")
    header_output_digest = require_sha256(
        header_record["output_sha256"],
        f"manifest source {HEADER_NAME} output_sha256",
    )
    if header_output_digest != hashlib.sha256(
        render_expected_header(forced, activation_time)
    ).hexdigest():
        raise ValidationInputError("manifest generated header digest is not reproducible")
    output_digests[HEADER_NAME] = header_output_digest

    for name, expected_digest in output_digests.items():
        output_path = path.parent / name
        if output_path.is_symlink() or not output_path.is_file():
            raise ValidationInputError(f"generated output {name} is missing")
        if digest(output_path) != expected_digest:
            raise ValidationInputError(
                f"generated output {name} does not match its trusted digest"
            )
    provenance = {
        "generator_sha256": generator_digest,
        "source_original_sha256": dict(EXPECTED_ORIGINAL_SHA256),
        "generated_output_sha256": output_digests,
    }
    return manifest, activation_time, provenance


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
    parser.add_argument("--slot-tolerance", type=float, default=1.0e-6)
    parser.add_argument(
        "--completion-tolerance",
        type=float,
        default=MAX_COMPLETION_TOLERANCE_S,
        help=(
            "maximum absolute difference between source-derived transmit "
            "completion and its receiver outcome; may be tightened but not "
            "increased above 0.001 s (default: 0.001 s)"
        ),
    )
    parser.add_argument("--report", type=Path)
    return parser


def validate(arguments: argparse.Namespace) -> dict[str, object]:
    """Validate one controlled collision trace and return its evidence report."""

    validate_report_path(arguments.report, arguments.trace, arguments.manifest)
    if not arguments.trace.is_file() or not arguments.manifest.is_file():
        raise ValidationInputError("trace and instrumentation manifest must exist")
    if (
        type(arguments.receiver) is not int
        or arguments.receiver < 0
        or arguments.receiver > 0xFFFFFF
    ):
        raise ValidationInputError("--receiver is outside the source node range")
    if arguments.receiver in arguments.transmitters:
        raise ValidationInputError("receiver must not also be a transmitter")
    if any(
        type(node) is not int or node < 0 or node > 0xFFFFFF
        for node in arguments.transmitters
    ):
        raise ValidationInputError("a transmitter is outside the source node range")
    if arguments.slot < 1 or arguments.slot > 255:
        raise ValidationInputError("--slot must be in 1..255")
    for value, label in (
        (arguments.simultaneous_tolerance, "simultaneous tolerance"),
        (arguments.holdoff_tolerance, "holdoff tolerance"),
        (arguments.slot_tolerance, "slot tolerance"),
        (arguments.completion_tolerance, "completion tolerance"),
    ):
        if not math.isfinite(value) or value < 0.0:
            raise ValidationInputError(f"{label} must be finite and nonnegative")
    for value, label in (
        (arguments.simultaneous_tolerance, "simultaneous tolerance"),
        (arguments.holdoff_tolerance, "holdoff tolerance"),
        (arguments.slot_tolerance, "slot tolerance"),
    ):
        if value > MAX_STRUCTURAL_TOLERANCE_S:
            raise ValidationInputError(f"{label} must not exceed 0.000001 s")
    if arguments.completion_tolerance > MAX_COMPLETION_TOLERANCE_S:
        raise ValidationInputError("completion tolerance must not exceed 0.001 s")
    for value, label in (
        (arguments.time_start, "time window start"),
        (arguments.time_stop, "time window stop"),
    ):
        if value is not None and not math.isfinite(value):
            raise ValidationInputError(f"{label} must be finite")
    if (
        arguments.time_start is not None
        and arguments.time_stop is not None
        and arguments.time_stop < arguments.time_start
    ):
        raise ValidationInputError("time window stop precedes its start")
    manifest, activation_time, manifest_provenance = validate_manifest(
        arguments.manifest, arguments.transmitters, arguments.slot
    )

    validate_raw_trace(arguments.trace)
    try:
        events = COMPARATOR.filter_trace(
            COMPARATOR.normalize_trace(arguments.trace),
            set(),
            arguments.time_start,
            arguments.time_stop,
        )
    except COMPARATOR.TraceError as error:
        raise ValidationInputError(str(error)) from error
    for event in events:
        number(event, "time_s")
        for field in STRICT_INTEGER_FIELDS:
            integer(event, field)
    failures: list[str] = []
    evidence: dict[str, object] = {}
    tx_times: list[float] = []
    transmissions: dict[int, dict[str, object]] = {}
    expected_counters = list(range(arguments.slot - 1, -2, -1))
    lifecycle_event_names = {
        "reservation_prepare",
        "reservation_holdoff",
        "reservation_tick",
        "reservation_advertise",
        "tx_start",
    }

    for node in arguments.transmitters:
        node_text = str(node)
        node_events = [
            event
            for event in events
            if event.values["node"] == node_text
            and event.values["event"] in lifecycle_event_names
        ]
        lifecycle = None
        controlled_prepare_indices = [
            index
            for index, event in enumerate(node_events)
            if event.values["event"] == "reservation_prepare"
            and event.values["reason"]
            in {"controlled_ready", "controlled_wait"}
        ]
        prepare_index = (
            controlled_prepare_indices[0]
            if len(controlled_prepare_indices) == 1
            else None
        )
        if prepare_index is not None:
            prepare = node_events[prepare_index]
            wait_for_holdoff = prepare.values["reason"] == "controlled_wait"
            holdoff_index = None
            if wait_for_holdoff:
                candidate = prepare_index + 1
                if (
                    candidate < len(node_events)
                    and node_events[candidate].values["event"]
                    == "reservation_holdoff"
                ):
                    holdoff_index = candidate
            cursor = prepare_index + 1
            valid = not wait_for_holdoff or holdoff_index is not None
            if holdoff_index is not None:
                cursor += 1
            tick_events = []
            for expected_counter in expected_counters:
                if (
                    not valid
                    or cursor >= len(node_events)
                    or node_events[cursor].values["event"]
                    != "reservation_tick"
                ):
                    valid = False
                    break
                tick = node_events[cursor]
                tick_events.append(tick)
                cursor += 1
            if (
                valid
                and cursor < len(node_events)
                and node_events[cursor].values["event"]
                == "reservation_advertise"
            ):
                advertisement = node_events[cursor]
                cursor += 1
            else:
                valid = False
                advertisement = None
            if (
                valid
                and cursor < len(node_events)
                and node_events[cursor].values["event"] == "tx_start"
            ):
                transmission = node_events[cursor]
            else:
                valid = False
                transmission = None
            if valid:
                lifecycle = {
                    "prepare": prepare,
                    "holdoff": (
                        None
                        if holdoff_index is None
                        else node_events[holdoff_index]
                    ),
                    "holdoff_mode": "wait" if wait_for_holdoff else "ready",
                    "ticks": tick_events,
                    "advertisement": advertisement,
                    "transmission": transmission,
                }

        if lifecycle is None:
            failures.append(
                f"node {node}: no ordered controlled prepare/gate/countdown/"
                "advertise/transmit lifecycle"
            )
            evidence[str(node)] = {"lifecycle_found": False}
            continue
        prepare_time = float(lifecycle["prepare"].values["time_s"])
        if number(lifecycle["prepare"], "reservation_slot") != arguments.slot:
            failures.append(f"node {node}: prepare slot is not {arguments.slot}")
        if number(lifecycle["prepare"], "reservation_counter") != arguments.slot:
            failures.append(f"node {node}: prepare counter is not {arguments.slot}")
        holdoff = lifecycle["holdoff"]
        holdoff_time = (
            None if holdoff is None else float(holdoff.values["time_s"])
        )
        holdoff_duration = (
            None if holdoff_time is None else holdoff_time - prepare_time
        )
        if (
            holdoff_duration is not None
            and abs(holdoff_duration - HOLDOFF_S) > arguments.holdoff_tolerance
        ):
            failures.append(
                f"node {node}: holdoff duration {holdoff_duration} is not 0.3 s"
            )
        if holdoff is not None:
            if number(holdoff, "reservation_slot") != arguments.slot:
                failures.append(f"node {node}: holdoff slot is not {arguments.slot}")
            if number(holdoff, "reservation_counter") != arguments.slot:
                failures.append(
                    f"node {node}: holdoff counter is not {arguments.slot}"
                )
        if prepare_time < activation_time:
            failures.append(
                f"node {node}: controlled preparation precedes manifest activation"
            )
        tick_times = [
            float(event.values["time_s"]) for event in lifecycle["ticks"]
        ]
        for tick, expected_counter in zip(
            lifecycle["ticks"], expected_counters
        ):
            if number(tick, "reservation_slot") != arguments.slot:
                failures.append(
                    f"node {node}: tick slot is not {arguments.slot}"
                )
            if number(tick, "reservation_counter") != expected_counter:
                failures.append(
                    f"node {node}: tick counter is not {expected_counter}"
                )
        expected_first_tick = math.ceil(
            (
                prepare_time
                + (HOLDOFF_S if lifecycle["holdoff_mode"] == "wait" else 0.0)
                + SLOT_CYCLE_S
            )
            / CONTROL_EPOCH_S
        ) * CONTROL_EPOCH_S
        if abs(tick_times[0] - expected_first_tick) > arguments.slot_tolerance:
            failures.append(
                f"node {node}: first controlled tick is not on source epoch"
            )
        for prior, current in zip(tick_times, tick_times[1:]):
            if abs((current - prior) - SLOT_CYCLE_S) > arguments.slot_tolerance:
                failures.append(f"node {node}: tick cadence is not 0.013 s")
        advertisement = lifecycle["advertisement"]
        if number(advertisement, "reservation_slot") != arguments.slot:
            failures.append(f"node {node}: advertise slot is not {arguments.slot}")
        if number(advertisement, "reservation_counter") != arguments.slot:
            failures.append(
                f"node {node}: advertise counter is not {arguments.slot}"
            )
        advertise_time = float(advertisement.values["time_s"])
        if abs(advertise_time - tick_times[-1]) > arguments.slot_tolerance:
            failures.append(
                f"node {node}: reservation advertisement is not at the final tick"
            )
        transmission = lifecycle["transmission"]
        tx_time = float(transmission.values["time_s"])
        if number(transmission, "reservation_slot") != arguments.slot:
            failures.append(f"node {node}: tx reservation slot is not {arguments.slot}")
        if number(transmission, "reservation_counter") != arguments.slot:
            failures.append(
                f"node {node}: tx reservation counter is not {arguments.slot}"
            )
        if abs(tx_time - tick_times[-1]) > arguments.slot_tolerance:
            failures.append(f"node {node}: tx_start is not at the final tick")
        if not math.isfinite(tx_time):
            failures.append(f"node {node}: transmit time is not finite")
        else:
            tx_times.append(tx_time)
        rate_kbps = integer(transmission, "rate_kbps")
        size_bytes = integer(transmission, "size_bytes")
        peer = integer(transmission, "peer")
        source = integer(transmission, "src")
        destination = integer(transmission, "dst")
        sequence = integer(transmission, "sequence")
        preamble = None
        try:
            preamble = parse_detail(transmission.values["detail"]).get("preamble")
        except ValidationInputError as error:
            failures.append(f"node {node}: tx_start {error}")
        if peer != arguments.receiver:
            failures.append(
                f"node {node}: tx_start peer {peer!r} is not receiver "
                f"{arguments.receiver}"
            )
        if source != node:
            failures.append(f"node {node}: tx_start source {source!r} is inconsistent")
        if destination != arguments.receiver or destination != peer:
            failures.append(
                f"node {node}: tx_start destination {destination!r} is inconsistent"
            )
        if sequence is None or sequence < 0 or sequence > MAX_SEQUENCE:
            failures.append(
                f"node {node}: tx_start sequence {sequence!r} is outside 0..65535"
            )
        if rate_kbps not in RATE_NIBBLE_DURATION_S:
            failures.append(
                f"node {node}: tx_start rate {rate_kbps!r} is not one of "
                "8,16,32,64,128 kbps"
            )
        if size_bytes is None or size_bytes <= 0:
            failures.append(
                f"node {node}: tx_start size {size_bytes!r} is not positive"
            )
        if preamble not in PREAMBLE_BITS:
            failures.append(
                f"node {node}: tx_start preamble {preamble!r} is not long or short"
            )
        duration_s = None
        completion_s = None
        if (
            math.isfinite(tx_time)
            and rate_kbps in RATE_NIBBLE_DURATION_S
            and size_bytes is not None
            and size_bytes > 0
            and preamble in PREAMBLE_BITS
        ):
            duration_s = source_airtime_s(rate_kbps, size_bytes, preamble)
            completion_s = tx_time + duration_s
            transmissions[node] = {
                "event": transmission,
                "rate_kbps": rate_kbps,
                "size_bytes": size_bytes,
                "preamble": preamble,
                "source": source,
                "destination": destination,
                "sequence": sequence,
                "duration_s": duration_s,
                "completion_s": completion_s,
            }
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
            "tick_times_s": tick_times,
            "expected_first_tick_time_s": expected_first_tick,
            "advertise_row": lifecycle["advertisement"].input_row,
            "advertise_time_s": advertise_time,
            "tx_row": transmission.input_row,
            "tx_time_s": tx_time,
            "tx_peer": peer,
            "tx_source": source,
            "tx_destination": destination,
            "tx_sequence": sequence,
            "tx_rate_kbps": rate_kbps,
            "tx_size_bytes": size_bytes,
            "tx_preamble": preamble,
            "source_airtime_s": duration_s,
            "expected_completion_time_s": completion_s,
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
        event for event in receiver_events if (collision_count(event) or 0) > 0
    ]
    drops = [event for event in receiver_events if event.values["event"] == "rx_drop"]
    receiver_evidence: dict[str, object] = {}
    for node in arguments.transmitters:
        peer_events = [
            event for event in receiver_events if event.values["peer"] == str(node)
        ]
        tx = transmissions.get(node)
        bound_events = []
        if tx is not None:
            bound_events = [
                event
                for event in peer_events
                if integer(event, "src") == node
                and integer(event, "rate_kbps") == tx["rate_kbps"]
                and integer(event, "size_bytes") == tx["size_bytes"]
                and abs(float(event.values["time_s"]) - tx["completion_s"])
                <= arguments.completion_tolerance
            ]
        outcome = bound_events[0] if len(bound_events) == 1 else None
        if tx is not None and len(bound_events) != 1:
            failures.append(
                f"receiver has {len(bound_events)} exact identity/time-bound outcomes "
                f"for transmitter {node}, expected one"
            )
        outcome_collision_count = None
        completion_error_s = None
        if outcome is not None:
            completion_error_s = (
                float(outcome.values["time_s"]) - tx["completion_s"]
            )
            outcome_collision_count = collision_count(outcome)
            if outcome.values["event"] != "rx_drop":
                failures.append(
                    f"receiver bound outcome for transmitter {node} is not a drop"
                )
            if outcome.values["success"] != "0":
                failures.append(
                    f"receiver bound drop for transmitter {node} lacks success=0"
                )
            if outcome_collision_count is None:
                failures.append(
                    f"receiver bound outcome for transmitter {node} has malformed "
                    "collision detail"
                )
            elif outcome_collision_count <= 0:
                failures.append(
                    f"receiver bound outcome for transmitter {node} has no positive "
                    "collision count"
                )
        receiver_evidence[str(node)] = {
            "outcome_rows": [event.input_row for event in peer_events],
            "bound_outcome_rows": [event.input_row for event in bound_events],
            "bound_outcome_row": None if outcome is None else outcome.input_row,
            "bound_outcome_event": (
                None if outcome is None else outcome.values["event"]
            ),
            "bound_outcome_source": (
                None if outcome is None else integer(outcome, "src")
            ),
            "bound_outcome_collision_count": outcome_collision_count,
            "completion_error_s": completion_error_s,
        }

    report = {
        "schema": "csr-reservation-collision-validation-v1",
        "pass": not failures,
        "trace": str(arguments.trace.resolve()),
        "trace_sha256": digest(arguments.trace),
        "manifest": str(arguments.manifest.resolve()),
        "manifest_sha256": digest(arguments.manifest),
        "manifest_provenance": manifest_provenance,
        "receiver": arguments.receiver,
        "transmitters": list(arguments.transmitters),
        "forced_slot": arguments.slot,
        "time_window": {"start": arguments.time_start, "stop": arguments.time_stop},
        "activation_time_s": activation_time,
        "tx_time_spread_s": spread,
        "source_airtime_contract": {
            "nibble_bits": NIBBLE_BITS,
            "preamble_bits": PREAMBLE_BITS,
            "phy_header_bits": PHY_HEADER_BITS,
            "fcs_bits": FCS_BITS,
            "rate_nibble_duration_s": RATE_NIBBLE_DURATION_S,
            "completion_tolerance_s": arguments.completion_tolerance,
            "holdoff_s": HOLDOFF_S,
            "slot_cycle_s": SLOT_CYCLE_S,
            "control_epoch_s": CONTROL_EPOCH_S,
            "structural_tolerances_s": {
                "holdoff": arguments.holdoff_tolerance,
                "slot": arguments.slot_tolerance,
                "simultaneous": arguments.simultaneous_tolerance,
            },
        },
        "receiver_outcome_count": len(receiver_events),
        "bound_receiver_outcome_count": sum(
            len(record["bound_outcome_rows"])
            for record in receiver_evidence.values()
        ),
        "positive_collision_count": len(collision_events),
        "drop_count": len(drops),
        "transmitter_evidence": evidence,
        "receiver_evidence": receiver_evidence,
        "failures": failures,
    }
    return report


def main() -> None:
    arguments = build_parser().parse_args()
    try:
        report = validate(arguments)
    except ValidationInputError as error:
        raise SystemExit(f"error: {error}") from error
    if arguments.report is not None:
        try:
            validate_report_path(
                arguments.report, arguments.trace, arguments.manifest
            )
        except ValidationInputError as error:
            raise SystemExit(f"error: {error}") from error
        arguments.report.parent.mkdir(parents=True, exist_ok=True)
        arguments.report.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(
        f"{'PASS' if report['pass'] else 'FAIL'}: "
        f"tx={sum(record.get('expected_completion_time_s') is not None for record in report['transmitter_evidence'].values())}/"
        f"{len(arguments.transmitters)} "
        f"rx={report['receiver_outcome_count']} "
        f"collisions={report['positive_collision_count']} "
        f"drops={report['drop_count']}"
    )
    for failure in report["failures"]:
        print(f"- {failure}")
    if report["failures"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
