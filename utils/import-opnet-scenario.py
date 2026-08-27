#!/usr/bin/env python3
"""Import a CSR OPNET Modeler scenario into the canonical harness CSV."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re
import struct
import sys
from typing import Callable, Iterable
import zipfile


MODEL_HEADER = b"MIL_3_Tfile_Hdr_"
ATTRIBUTE_SENTINEL = b"\x00\x01"
ATTRIBUTE_SUFFIX = b"\x00\x00\x00\x00\x00"
NODE_MODEL = "br_node_v1"
SCHEMA_VERSION = "csr-opnet-scenario-v1"

ATTRIBUTE_NAMES = (
    "name",
    "doc file",
    "icon name",
    "altitude",
    "Extended Attrs.",
    "model",
    "tooltip",
    "creation source",
    "creation timestamp",
    "creation data",
    "Model Details",
    "Object Documentation",
    "x position",
    "y position",
    "threshold",
    "label color",
    "Node ID",
    "Min Speed",
    "Max Speed",
    "Min Power",
    "Max Power",
    "Link Margin",
    "rx.ecc threshold",
    "rx.channel [0].min frequency",
    "tx.channel [0].min frequency",
    "Node Type",
    "app.Packet Interarrival Time",
    "app.Packet Size",
    "app.Start Time",
    "battery.Initial Energy",
)

SCENARIO_COLUMNS = (
    "record",
    "schema",
    "scenario",
    "source_sha256",
    "duration_s",
    "seed",
    "tmm",
    "coordinate_scale_m_per_unit",
    "reservation_control_start_s",
    "node_id",
    "forced_reservation_slot",
    "name",
    "node_type",
    "x_m",
    "y_m",
    "height_m",
    "min_speed_kbps",
    "max_speed_kbps",
    "min_power_dbm",
    "max_power_dbm",
    "link_margin_db",
    "ecc_threshold",
    "rx_frequency_hz",
    "tx_frequency_hz",
    "interarrival_s",
    "packet_bytes",
    "start_s",
    "flow_src",
    "flow_dst",
    "flow_start_s",
    "flow_interval_s",
    "flow_packet_bytes",
    "flow_dscp",
)


class ImportErrorDetail(ValueError):
    """A source file is valid input media but not a supported CSR scenario."""


@dataclass(frozen=True)
class Node:
    """Promoted values recovered from one br_node_v1 instance."""

    node_id: int
    name: str
    node_type: str
    x_m: float
    y_m: float
    height_m: float
    min_speed_kbps: int
    max_speed_kbps: int
    min_power_dbm: float
    max_power_dbm: float
    link_margin_db: float
    ecc_threshold: float
    rx_frequency_hz: float
    tx_frequency_hz: float
    interarrival_s: float | None
    packet_bytes: int | None
    start_s: float | None


@dataclass(frozen=True)
class Flow:
    """Explicit deterministic application traffic for differential runs."""

    source: int
    destination: int
    start_s: float
    interval_s: float
    packet_bytes: int
    dscp: int


@dataclass(frozen=True)
class ScenarioSource:
    """Resolved scenario members and byte reader."""

    scenario_name: str
    network_name: str
    environment_name: str | None
    read: Callable[[str], bytes]


def _attribute_signature(attribute_id: int) -> bytes:
    if not 0 < attribute_id < 256:
        raise ImportErrorDetail("OPNET promoted attribute ID exceeds one byte")
    # The four bytes before this suffix encode a Modeler type/limit reference
    # and are not constant across attributes. The repeated local attribute ID
    # and five-byte value prefix are stable in every supplied 17.1 tfile.
    return ATTRIBUTE_SENTINEL + bytes((attribute_id,)) + ATTRIBUTE_SUFFIX


def _read_string(data: bytes, offset: int) -> tuple[str, int]:
    if offset + 4 > len(data):
        raise ImportErrorDetail("truncated OPNET string length")
    (length,) = struct.unpack_from(">I", data, offset)
    end = offset + 4 + length
    if length > 4096 or end > len(data):
        raise ImportErrorDetail("invalid OPNET string length")
    try:
        return data[offset + 4 : end].decode("utf-8"), end
    except UnicodeDecodeError as error:
        raise ImportErrorDetail("non-text OPNET attribute value") from error


def _read_promoted_value(
    data: bytes, block_start: int, block_end: int, attribute_id: int, kind: str
) -> object | None:
    signature = _attribute_signature(attribute_id)
    position = data.find(signature, block_start, block_end)
    if position < 0:
        return None
    position += len(signature)
    if position >= block_end or data[position : position + 1] != b"J":
        return None
    position += 1
    if kind == "double":
        if position + 8 > block_end:
            raise ImportErrorDetail("truncated OPNET double attribute")
        return struct.unpack_from(">d", data, position)[0]
    if kind == "integer":
        if position + 4 > block_end:
            raise ImportErrorDetail("truncated OPNET integer attribute")
        return struct.unpack_from(">I", data, position)[0]
    if kind == "string":
        return _read_string(data, position)[0]
    if kind == "enum":
        # Promoted enum values store a selector followed by the display string.
        if position + 8 > block_end:
            raise ImportErrorDetail("truncated OPNET enum attribute")
        _, length = struct.unpack_from(">II", data, position)
        if length > 256 or position + 8 + length > block_end:
            raise ImportErrorDetail("invalid OPNET enum attribute")
        return data[position + 8 : position + 8 + length].decode("ascii")
    raise AssertionError(f"unknown promoted value kind {kind}")


def _schema_ids(data: bytes, first_node_offset: int) -> dict[str, int]:
    """Recover file-local attribute IDs from their ordered schema strings."""
    found: list[tuple[int, str]] = []
    for name in ATTRIBUTE_NAMES:
        encoded = name.encode("ascii")
        position = 512
        while True:
            position = data.find(encoded, position, first_node_offset)
            if position < 0:
                break
            # Modeler tfile schema names are uint32-length-prefixed. Checking
            # the prefix prevents a field such as "threshold" from matching
            # the suffix of "rx.ecc threshold" and shifting every later ID.
            if position >= 4 and data[position - 4 : position] == struct.pack(
                ">I", len(encoded)
            ):
                found.append((position, name))
                break
            position += 1
    found.sort()
    if not found or found[0][1] != "name" or "model" not in {x[1] for x in found}:
        raise ImportErrorDetail("could not recover the OPNET node attribute schema")
    return {name: index + 1 for index, (_, name) in enumerate(found)}


def _candidate_node_offsets(data: bytes, name_attribute_id: int) -> list[int]:
    signature = _attribute_signature(name_attribute_id)
    candidates: list[int] = []
    position = 0
    while True:
        position = data.find(signature, position)
        if position < 0:
            break
        value_offset = position + len(signature)
        try:
            if data[value_offset : value_offset + 1] == b"J":
                name, _ = _read_string(data, value_offset + 1)
                if name and name.isprintable():
                    candidates.append(position)
        except ImportErrorDetail:
            pass
        position += 1
    return candidates


def _constant_seconds(value: str | None) -> float | None:
    if value is None:
        return None
    match = re.fullmatch(
        r"\s*constant\s*\(\s*([0-9]+(?:\.[0-9]*)?(?:[eE][+-]?[0-9]+)?)\s*\)\s*",
        value,
    )
    if not match:
        raise ImportErrorDetail(
            f"unsupported Packet Interarrival Time distribution: {value!r}"
        )
    seconds = float(match.group(1))
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ImportErrorDetail("packet interarrival time must be positive")
    return seconds


def _frequency_hz(value: float) -> float:
    # The promoted validation models store 400 for 400 MHz. Modeler radio
    # pipeline attributes are expressed in Hz after promotion.
    return value * 1.0e6 if value < 1.0e6 else value


def parse_network_model(data: bytes, coordinate_scale: float) -> list[Node]:
    """Decode br_node_v1 instances from a Modeler 17.1 binary network model."""
    if not data.startswith(MODEL_HEADER):
        raise ImportErrorDetail("input is not an OPNET Modeler tfile")
    if not math.isfinite(coordinate_scale) or coordinate_scale <= 0.0:
        raise ImportErrorDetail("coordinate scale must be positive")

    first_name = data.find(b"node_")
    if first_name < 0:
        raise ImportErrorDetail("network model contains no recognizable node instances")
    ids = _schema_ids(data, first_name)
    candidates = _candidate_node_offsets(data, ids["name"])
    nodes: list[Node] = []

    for candidate_index, start in enumerate(candidates):
        end = (
            candidates[candidate_index + 1]
            if candidate_index + 1 < len(candidates)
            else len(data)
        )
        model_id = ids.get("model")
        if model_id is None:
            continue
        model = _read_promoted_value(data, start, min(end, start + 1800), model_id, "string")
        if model != NODE_MODEL:
            continue
        name = _read_promoted_value(data, start, end, ids["name"], "string")
        if not isinstance(name, str):
            raise ImportErrorDetail("node has no name")

        def required(field: str, kind: str) -> object:
            attribute_id = ids.get(field)
            if attribute_id is None:
                raise ImportErrorDetail(f"network schema omits required attribute {field!r}")
            value = _read_promoted_value(data, start, end, attribute_id, kind)
            if value is None:
                raise ImportErrorDetail(f"node {name!r} omits required attribute {field!r}")
            return value

        def optional(field: str, kind: str, default: object) -> object:
            attribute_id = ids.get(field)
            if attribute_id is None:
                return default
            value = _read_promoted_value(data, start, end, attribute_id, kind)
            return default if value is None else value

        node_type = str(optional("Node Type", "enum", "Ordinary")).strip().lower()
        if node_type == "relay":
            node_type = "routable"
        if node_type not in {"ordinary", "routable", "gateway"}:
            raise ImportErrorDetail(f"node {name!r} has unsupported type {node_type!r}")
        interarrival = optional("app.Packet Interarrival Time", "string", None)

        node = Node(
            node_id=int(required("Node ID", "integer")),
            name=name,
            node_type=node_type,
            x_m=float(required("x position", "double")) * coordinate_scale,
            y_m=float(required("y position", "double")) * coordinate_scale,
            height_m=float(required("altitude", "double")),
            min_speed_kbps=int(optional("Min Speed", "integer", 8)),
            max_speed_kbps=int(optional("Max Speed", "integer", 128)),
            min_power_dbm=float(required("Min Power", "double")),
            max_power_dbm=float(required("Max Power", "double")),
            link_margin_db=float(required("Link Margin", "double")),
            ecc_threshold=float(required("rx.ecc threshold", "double")),
            rx_frequency_hz=_frequency_hz(
                float(required("rx.channel [0].min frequency", "double"))
            ),
            tx_frequency_hz=_frequency_hz(
                float(required("tx.channel [0].min frequency", "double"))
            ),
            interarrival_s=_constant_seconds(
                None if interarrival is None else str(interarrival)
            ),
            packet_bytes=(
                None
                if "app.Packet Size" not in ids
                else int(optional("app.Packet Size", "integer", 0))
            ),
            start_s=(
                None
                if "app.Start Time" not in ids
                else float(optional("app.Start Time", "double", 0.0))
            ),
        )
        nodes.append(node)

    if not nodes:
        raise ImportErrorDetail(f"network model contains no {NODE_MODEL} instances")
    node_ids = [node.node_id for node in nodes]
    if len(node_ids) != len(set(node_ids)):
        raise ImportErrorDetail("network model contains duplicate Node ID values")
    if any(node_id < 0 or node_id > 0xFFFFFF for node_id in node_ids):
        raise ImportErrorDetail("network model contains a Node ID wider than 24 bits")
    return sorted(nodes, key=lambda node: node.node_id)


def parse_environment(data: bytes | None) -> dict[str, object]:
    """Parse the text DES execution file paired with a network model."""
    if data is None:
        return {"duration_s": 60.0, "seed": 128, "tmm": False}
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ImportErrorDetail("DES environment file is not text") from error
    values = dict(re.findall(r'^"([^"]+)"\s*:\s*"([^"]*)"\s*$', text, re.MULTILINE))

    def number(key: str, default: float) -> float:
        raw = values.get(key)
        if raw is None:
            return default
        value = float(raw.replace(",", ""))
        if not math.isfinite(value):
            raise ImportErrorDetail(f"DES value {key!r} is not finite")
        return value

    return {
        "duration_s": number("duration", 60.0),
        "seed": int(number("seed", 128.0)),
        "tmm": values.get("tmm_simulate", "false").strip().lower() == "true",
    }


def resolve_source(source: Path, scenario: str | None, environment: str | None) -> ScenarioSource:
    """Resolve raw files or archive members without extracting the archive."""
    if zipfile.is_zipfile(source):
        archive = zipfile.ZipFile(source)
        members = {Path(name).name: name for name in archive.namelist() if not name.endswith("/")}
        network_names = sorted(name for name in members if name.endswith(".nt.m"))
        if scenario is None:
            if len(network_names) != 1:
                raise ImportErrorDetail(
                    "archive contains multiple scenarios; pass --scenario with a *.nt.m basename"
                )
            network_name = network_names[0]
        else:
            network_name = Path(scenario).name
            if network_name not in members:
                raise ImportErrorDetail(f"scenario member not found: {network_name}")
        environment_name = Path(environment).name if environment else None
        if environment_name is None:
            stem = network_name[: -len(".nt.m")]
            candidate = f"{stem}-DES-1.ef"
            environment_name = candidate if candidate in members else None
        elif environment_name not in members:
            raise ImportErrorDetail(f"environment member not found: {environment_name}")
        return ScenarioSource(
            scenario_name=network_name[: -len(".nt.m")],
            network_name=network_name,
            environment_name=environment_name,
            read=lambda name: archive.read(members[name]),
        )

    if not source.is_file() or not source.name.endswith(".nt.m"):
        raise ImportErrorDetail("source must be a *.nt.m file or ZIP archive")
    environment_path = Path(environment) if environment else source.with_name(
        f"{source.name[:-len('.nt.m')]}-DES-1.ef"
    )
    environment_name = str(environment_path) if environment_path.is_file() else None

    def read_file(name: str) -> bytes:
        return Path(name).read_bytes()

    return ScenarioSource(
        scenario_name=source.name[: -len(".nt.m")],
        network_name=str(source),
        environment_name=environment_name,
        read=read_file,
    )


def parse_flow(specification: str) -> Flow:
    """Parse SRC:DST:START:INTERVAL:BYTES:DSCP."""
    parts = specification.split(":")
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            "flow must be SRC:DST:START_S:INTERVAL_S:BYTES:DSCP"
        )
    try:
        flow = Flow(
            source=int(parts[0], 0),
            destination=int(parts[1], 0),
            start_s=float(parts[2]),
            interval_s=float(parts[3]),
            packet_bytes=int(parts[4], 0),
            dscp=int(parts[5], 0),
        )
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if (
        flow.source < 0
        or flow.source > 0xFFFFFF
        or flow.destination < 0
        or flow.destination > 0xFFFFFF
        or flow.start_s < 0.0
        or flow.interval_s <= 0.0
        or flow.packet_bytes <= 0
        or flow.dscp < 0
        or flow.dscp > 7
    ):
        raise argparse.ArgumentTypeError("flow contains an out-of-range value")
    return flow


def parse_reservation_slot_override(specification: str) -> tuple[int, int]:
    """Parse NODE:SLOT for a controlled differential reservation run."""
    parts = specification.split(":")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("reservation override must be NODE:SLOT")
    try:
        node_id = int(parts[0], 0)
        slot = int(parts[1], 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if node_id < 0 or node_id > 0xFFFFFF or slot < 1 or slot > 255:
        raise argparse.ArgumentTypeError(
            "reservation override requires a 24-bit node and slot 1..255"
        )
    return node_id, slot


def infer_ring_flows(nodes: list[Node]) -> list[Flow]:
    """Create deterministic explicit flows from promoted app attributes."""
    if len(nodes) < 2:
        return []
    flows: list[Flow] = []
    for index, node in enumerate(nodes):
        if node.interarrival_s is None or node.packet_bytes is None or node.start_s is None:
            continue
        destination = nodes[(index + 1) % len(nodes)].node_id
        flows.append(
            Flow(
                source=node.node_id,
                destination=destination,
                start_s=node.start_s,
                interval_s=node.interarrival_s,
                packet_bytes=node.packet_bytes,
                dscp=5,
            )
        )
    return flows


def _blank_row() -> dict[str, object]:
    return {column: "" for column in SCENARIO_COLUMNS}


def write_scenario(
    output: Path,
    scenario_name: str,
    digest: str,
    run: dict[str, object],
    coordinate_scale: float,
    nodes: Iterable[Node],
    flows: Iterable[Flow],
    reservation_slot_overrides: dict[int, int],
) -> None:
    """Write the rectangular canonical scenario CSV."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SCENARIO_COLUMNS, lineterminator="\n")
        writer.writeheader()
        row = _blank_row()
        row.update(
            record="run",
            schema=SCHEMA_VERSION,
            scenario=scenario_name,
            source_sha256=digest,
            duration_s=repr(run["duration_s"]),
            seed=str(run["seed"]),
            tmm="1" if run["tmm"] else "0",
            coordinate_scale_m_per_unit=repr(coordinate_scale),
            reservation_control_start_s=(
                ""
                if "reservation_control_start_s" not in run
                else repr(run["reservation_control_start_s"])
            ),
        )
        writer.writerow(row)
        for node in nodes:
            row = _blank_row()
            row.update(
                record="node",
                schema=SCHEMA_VERSION,
                node_id=str(node.node_id),
                forced_reservation_slot=(
                    ""
                    if node.node_id not in reservation_slot_overrides
                    else str(reservation_slot_overrides[node.node_id])
                ),
                name=node.name,
                node_type=node.node_type,
                x_m=repr(node.x_m),
                y_m=repr(node.y_m),
                height_m=repr(node.height_m),
                min_speed_kbps=str(node.min_speed_kbps),
                max_speed_kbps=str(node.max_speed_kbps),
                min_power_dbm=repr(node.min_power_dbm),
                max_power_dbm=repr(node.max_power_dbm),
                link_margin_db=repr(node.link_margin_db),
                ecc_threshold=repr(node.ecc_threshold),
                rx_frequency_hz=repr(node.rx_frequency_hz),
                tx_frequency_hz=repr(node.tx_frequency_hz),
                interarrival_s="" if node.interarrival_s is None else repr(node.interarrival_s),
                packet_bytes="" if node.packet_bytes is None else str(node.packet_bytes),
                start_s="" if node.start_s is None else repr(node.start_s),
            )
            writer.writerow(row)
        for flow in flows:
            row = _blank_row()
            row.update(
                record="flow",
                schema=SCHEMA_VERSION,
                flow_src=str(flow.source),
                flow_dst=str(flow.destination),
                flow_start_s=repr(flow.start_s),
                flow_interval_s=repr(flow.interval_s),
                flow_packet_bytes=str(flow.packet_bytes),
                flow_dscp=str(flow.dscp),
            )
            writer.writerow(row)


def import_scenario(arguments: argparse.Namespace) -> tuple[list[Node], list[Flow]]:
    """Execute one import and return the decoded records for tests."""
    resolved = resolve_source(arguments.source, arguments.scenario, arguments.environment)
    network_data = resolved.read(resolved.network_name)
    environment_data = (
        None if resolved.environment_name is None else resolved.read(resolved.environment_name)
    )
    nodes = parse_network_model(network_data, arguments.coordinate_scale)
    flows = list(arguments.flow)
    if arguments.infer_ring_flows:
        if flows:
            raise ImportErrorDetail("--flow and --infer-ring-flows are mutually exclusive")
        flows = infer_ring_flows(nodes)
    known_ids = {node.node_id for node in nodes}
    for flow in flows:
        if flow.source not in known_ids or flow.destination not in known_ids:
            raise ImportErrorDetail(
                f"flow {flow.source}->{flow.destination} references an unknown node"
            )
    reservation_slot_overrides: dict[int, int] = {}
    for node_id, slot in arguments.force_reservation_slot:
        if node_id not in known_ids:
            raise ImportErrorDetail(
                f"reservation override references unknown node {node_id}"
            )
        if node_id in reservation_slot_overrides:
            raise ImportErrorDetail(
                f"reservation override repeats node {node_id}"
            )
        reservation_slot_overrides[node_id] = slot
    run = parse_environment(environment_data)
    if arguments.duration is not None:
        if not math.isfinite(arguments.duration) or arguments.duration <= 0.0:
            raise ImportErrorDetail("--duration must be finite and positive")
        run["duration_s"] = arguments.duration
    if arguments.seed is not None:
        if arguments.seed < 0 or arguments.seed > 0xFFFFFFFF:
            raise ImportErrorDetail("--seed must be in 0..4294967295")
        run["seed"] = arguments.seed
    if arguments.reservation_control_start is not None:
        control_start = arguments.reservation_control_start
        if not math.isfinite(control_start) or control_start < 0.0:
            raise ImportErrorDetail(
                "--reservation-control-start must be finite and nonnegative"
            )
        if not reservation_slot_overrides:
            raise ImportErrorDetail(
                "--reservation-control-start requires a reservation override"
            )
        if control_start > run["duration_s"]:
            raise ImportErrorDetail(
                "--reservation-control-start exceeds the run duration"
            )
        run["reservation_control_start_s"] = control_start
    digest = hashlib.sha256(network_data + b"\0" + (environment_data or b"")).hexdigest()
    write_scenario(
        arguments.output,
        resolved.scenario_name,
        digest,
        run,
        arguments.coordinate_scale,
        nodes,
        flows,
        reservation_slot_overrides,
    )
    if resolved.environment_name is None:
        print(
            "warning: no matching DES *.ef found; using duration=60 s, seed=128, TMM=false",
            file=sys.stderr,
        )
    return nodes, flows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="*.nt.m file or project ZIP archive")
    parser.add_argument("output", type=Path, help="canonical scenario CSV")
    parser.add_argument(
        "--scenario", help="*.nt.m member basename when SOURCE is a multi-scenario ZIP"
    )
    parser.add_argument(
        "--environment", help="paired DES *.ef path or archive member (auto-detected by default)"
    )
    parser.add_argument(
        "--coordinate-scale",
        type=float,
        default=1000.0,
        help="meters per Modeler fixed-subnet x/y unit (default: 1000)",
    )
    parser.add_argument(
        "--flow",
        action="append",
        default=[],
        type=parse_flow,
        metavar="SRC:DST:START:INTERVAL:BYTES:DSCP",
        help="append an explicit deterministic traffic flow",
    )
    parser.add_argument(
        "--force-reservation-slot",
        action="append",
        default=[],
        type=parse_reservation_slot_override,
        metavar="NODE:SLOT",
        help="control a node's reservation slot/counter for a differential run",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="override the paired DES duration in the canonical run row",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="override the paired DES random seed in the canonical run row",
    )
    parser.add_argument(
        "--reservation-control-start",
        type=float,
        help="simulation time at which forced reservation controls become active",
    )
    parser.add_argument(
        "--infer-ring-flows",
        action="store_true",
        help="make a deterministic ring from promoted app attributes",
    )
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    try:
        nodes, flows = import_scenario(arguments)
    except (ImportErrorDetail, OSError, zipfile.BadZipFile) as error:
        raise SystemExit(f"error: {error}") from error
    print(
        f"imported {len(nodes)} nodes and {len(flows)} explicit flows to {arguments.output}"
    )


if __name__ == "__main__":
    main()
