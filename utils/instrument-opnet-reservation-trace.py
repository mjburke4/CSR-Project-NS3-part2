#!/usr/bin/env python3
"""Instrument recovered OPNET sources for a controlled reservation collision."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


SCHEMA = "csr-opnet-instrumentation-v1"
TRACE_SCHEMA = "csr-differential-trace-v1"
PATCHED_SOURCES = {
    "br_app.pr.c": "0e3dfeefc90da99b78f1b6305ccffa765de4b35710414bd85aa9fc30d8bb6dde",
    "br_mac.pr.c": "d1bc9f7d57da4cc490a835e19b26eac47dc0088629ecd15abfaaa5a0734be27a",
    "br_ecc.ps.c": "fbc5d5fb63c08eb3a5b2afec0baacf7c59182cf07382ed5fd3a61bd61f53d83c",
}
AIRTIME_REFERENCE_SOURCES = {
    "br_support.h": "9855b95029a2e15190976ef5ae926d028a98ef91db5d7567597c5f01cc7f37ce",
    "br_txdel.ps.c": "6b9ae9213aea8546b97069df5fe5315e8a7c49203d7d27c3d6e559e452a49463",
}
EXPECTED_SOURCES = {**PATCHED_SOURCES, **AIRTIME_REFERENCE_SOURCES}
EXPECTED_PATCHED_OUTPUT_SHA256 = {
    "br_app.pr.c": "1678d9f1c81d8692ab28728feb5ea2c7913e3578c350633a7e5991e6e302ceee",
    "br_mac.pr.c": "726dd449acf7b9670ea941bbaf6115569db6ddf547bc19437f1685c3c09712ae",
    "br_ecc.ps.c": "4d92630cffa8da10a14d02238eb9259dc8d5cb2dd6ffd68ad397ac37432df861",
}
EXPECTED_HEADER_TEMPLATE_SHA256 = (
    "60aac760949eb895304ef1457d8623253ddfb22245b0683fee1bea6b589692e6"
)
GENERATED_OUTPUTS = tuple(PATCHED_SOURCES) + ("csr-opnet-trace.h",)
MANIFEST_NAME = "opnet-instrumentation-manifest.json"
GENERATOR_NAME = "instrument-opnet-reservation-trace.py"
BEHAVIORAL_CONTROL = (
    "A test hook loads the selected slot/counter at transmit preparation, "
    "controls future reservation draws, and aligns the first countdown "
    "tick to a common 100-ms epoch after any still-pending source "
    "holdoff. Idle prep_tx holdoff and Search-state holdoff reuse, "
    "countdown steps, transmit, receive, collision, BER, and ECC logic "
    "remain in the recovered sources."
)


class InstrumentationError(ValueError):
    """The supplied source does not match a safe instrumentation anchor."""


def digest_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def paths_overlap(first: Path, second: Path) -> bool:
    """Return whether resolved directory paths are equal or nested."""

    try:
        first = first.resolve(strict=False)
        second = second.resolve(strict=False)
    except (OSError, RuntimeError) as error:
        raise InstrumentationError(f"cannot resolve directory identity: {error}") from error
    return first == second or first in second.parents or second in first.parents


def paths_alias(first: Path, second: Path) -> bool:
    """Return whether two existing or prospective paths address one object."""

    try:
        if first.resolve(strict=False) == second.resolve(strict=False):
            return True
        return first.samefile(second)
    except FileNotFoundError:
        return False
    except (OSError, RuntimeError) as error:
        raise InstrumentationError(f"cannot resolve file identity: {error}") from error


def resolve_source(source_dir: Path, name: str) -> Path:
    """Resolve either an exported basename or the numbered upload copy."""
    exact = source_dir / name
    if exact.is_file():
        return exact
    matches = sorted(source_dir.glob(f"*-{name}"))
    if len(matches) != 1:
        rendered = ", ".join(str(path) for path in matches) or "none"
        raise InstrumentationError(
            f"{name}: expected one source file, found {len(matches)} ({rendered})"
        )
    return matches[0]


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise InstrumentationError(
            f"{label}: expected one source anchor, found {count}"
        )
    return text.replace(old, new, 1)


def replace_in_function(
    text: str,
    start: str,
    stop: str,
    old: str,
    new: str,
    label: str,
) -> str:
    start_at = text.find(start)
    stop_at = text.find(stop, start_at + len(start))
    if start_at < 0 or stop_at < 0:
        raise InstrumentationError(f"{label}: function boundary not found")
    body = text[start_at:stop_at]
    body = replace_once(body, old, new, label)
    return text[:start_at] + body + text[stop_at:]


def patch_app(text: str) -> str:
    text = replace_once(
        text,
        "#include <csr_api.h>\n",
        "#include <csr_api.h>\n"
        "#define CSR_OPNET_TRACE_APP 1\n"
        '#include "csr-opnet-trace.h"\n',
        "br_app trace include",
    )
    return replace_in_function(
        text,
        "static void ss_packet_generate(void)\n\t{",
        "static void proc_packet(void)",
        "\t/* Send the packet via the stream to the lower layer.\t*/\n"
        "\top_pk_send(pkptr, APP_TO_NWK_STRM);",
        "\t/* Send the packet via the stream to the lower layer.\t*/\n"
        "\tcsr_opnet_trace_app_send(op_sim_time(), my_node_id, nwk_dest, "
        "source_packet_size, dscp);\n"
        "\top_pk_send(pkptr, APP_TO_NWK_STRM);",
        "br_app app_send",
    )


def patch_mac(text: str) -> str:
    text = replace_once(
        text,
        "#include <csr_api.h>\n",
        "#include <csr_api.h>\n"
        "#define CSR_OPNET_TRACE_MAC 1\n"
        '#include "csr-opnet-trace.h"\n',
        "br_mac trace include",
    )
    text = replace_in_function(
        text,
        "static void br_mac_init(void)\n\t{",
        "static void start_search(void)",
        '\top_ima_obj_attr_get(my_attributes.node_objid, "Node ID", &my_node_id);',
        '\top_ima_obj_attr_get(my_attributes.node_objid, "Node ID", &my_node_id);\n'
        "\tcsr_opnet_trace_reset_once();",
        "br_mac trace reset",
    )
    text = replace_in_function(
        text,
        "static int get_slot(int nodes, int num_rslot)\n\t{",
        "static int renew_txslot(int islot)",
        "\tslot = ts_ptr->slot;",
        "\tslot = ts_ptr->slot;\n"
        "\tslot = csr_opnet_forced_slot(my_node_id, slot);",
        "br_mac forced draw",
    )
    text = replace_in_function(
        text,
        "static void prep_tx(void)\n\t{",
        "static void tsholdoffo(void)\n\t{",
        "\top_intrpt_schedule_remote(op_sim_time(), MAC_START_SEARCH_CODE, "
        "my_attributes.battery_objid);",
        "\tif (csr_opnet_has_forced_slot(my_node_id))\n"
        "\t\t{\n"
        "\t\ttxslot = csr_opnet_forced_slot(my_node_id, txslot);\n"
        "\t\ttxslot_counter = txslot;\n"
        "\t\top_ev_cancel_if_pending(tslot_Evh);\n"
        "\t\ttslot_Evh = op_intrpt_schedule_self(\n"
        "\t\t\tcsr_opnet_controlled_slot_epoch(op_sim_time(), "
        "TS_HOLDOFF_TIME, TSLOT_CYCLE), TSLOT_CODE);\n"
        "\t\t}\n"
        "\tcsr_opnet_trace_reservation(op_sim_time(), \"reservation_prepare\", "
        "my_node_id, txslot, txslot_counter,\n"
        "\t\tcsr_opnet_has_forced_slot(my_node_id) ? \"controlled_wait\" :\n"
        "\t\t((txslot_counter == txslot) ? \"new\" : \"reuse\"));\n"
        "\top_intrpt_schedule_remote(op_sim_time(), MAC_START_SEARCH_CODE, "
        "my_attributes.battery_objid);",
        "br_mac reservation prepare",
    )
    text = replace_in_function(
        text,
        "static void tsholdoffo(void)",
        "static void send_pk(void)",
        "\tTS_HOLDOFF_OVER = OPC_TRUE;",
        "\tTS_HOLDOFF_OVER = OPC_TRUE;\n"
        "\tcsr_opnet_trace_reservation(op_sim_time(), \"reservation_holdoff\", "
        "my_node_id, txslot, txslot_counter, OPC_NIL);",
        "br_mac reservation holdoff",
    )
    text = replace_in_function(
        text,
        "static void tslot_tasks(void)\n\t{",
        "static int get_slot(int nodes, int num_rslot)",
        "\t\t\t\ttxslot_counter--;",
        "\t\t\t\ttxslot_counter--;\n"
        "\t\t\t\tcsr_opnet_trace_reservation(op_sim_time(), "
        "\"reservation_tick\", my_node_id, txslot, txslot_counter, OPC_NIL);",
        "br_mac reservation tick",
    )
    text = replace_in_function(
        text,
        "static void tslot_tasks(void)\n\t{",
        "static int get_slot(int nodes, int num_rslot)",
        "\t\t\t\tPREP_TX = OPC_TRUE;\n"
        "\t\t\t\tif (txslot_counter < 0)",
        "\t\t\t\tPREP_TX = OPC_TRUE;\n"
        "\t\t\t\tif (csr_opnet_has_forced_slot(my_node_id))\n"
        "\t\t\t\t\t{\n"
        "\t\t\t\t\ttxslot = csr_opnet_forced_slot(my_node_id, txslot);\n"
        "\t\t\t\t\ttxslot_counter = txslot;\n"
        "\t\t\t\t\top_ev_cancel_if_pending(tslot_Evh);\n"
        "\t\t\t\t\ttslot_Evh = op_intrpt_schedule_self(\n"
        "\t\t\t\t\t\tcsr_opnet_controlled_slot_epoch(op_sim_time(),\n"
        "\t\t\t\t\t\t\tTS_HOLDOFF_OVER ? 0.0 : TS_HOLDOFF_TIME,\n"
        "\t\t\t\t\t\t\tTSLOT_CYCLE), TSLOT_CODE);\n"
        "\t\t\t\t\tcsr_opnet_trace_reservation(op_sim_time(),\n"
        "\t\t\t\t\t\t\"reservation_prepare\", my_node_id, txslot,\n"
        "\t\t\t\t\t\ttxslot_counter, TS_HOLDOFF_OVER ?\n"
        "\t\t\t\t\t\t\t\"controlled_ready\" : \"controlled_wait\");\n"
        "\t\t\t\t\t}\n"
        "\t\t\t\tif (txslot_counter < 0)",
        "br_mac Search reservation prepare",
    )
    text = replace_in_function(
        text,
        "static void send_pk(void)",
        "static void post_tx(void)",
        "\tint\t\t\t\t\tdest, dest_type, dest_type_c, ackable, resendable;\n"
        "\tint\t\t\t\t\tseq_num, sgm_num, byte_count, pk_count;",
        "\tint\t\t\t\t\tdest = BROADCAST_ADDRESS, dest_type, dest_type_c, "
        "ackable, resendable;\n"
        "\tint\t\t\t\t\tseq_num = 0, sgm_num, byte_count, pk_count;",
        "br_mac trace variable initialization",
    )
    text = replace_in_function(
        text,
        "static void send_pk(void)",
        "static void post_tx(void)",
        "\t\tRESERVATION_SENT = OPC_TRUE;",
        "\t\tRESERVATION_SENT = OPC_TRUE;\n"
        "\t\tcsr_opnet_trace_reservation(op_sim_time(), "
        "\"reservation_advertise\", my_node_id, txslot_counter, "
        "txslot_counter, OPC_NIL);",
        "br_mac reservation advertise",
    )
    return replace_in_function(
        text,
        "static void send_pk(void)",
        "static void post_tx(void)",
        "\t// Send packet to transmitter\n\top_pk_send(pkptr, MAC_TO_TX_STRM);",
        "\t// Send packet to transmitter\n"
        "\tcsr_opnet_trace_tx_start(op_sim_time(), my_node_id, dest, seq_num, "
        "payload_speed, (int)f1/8,\n"
        "\t\tstrcmp(pk_format, \"br_OTA_LongPream\") == 0 ? \"long\" :\n"
        "\t\t(strcmp(pk_format, \"br_OTA_ShortPream\") == 0 ? \"short\" : "
        "\"unknown\"), txslot_counter);\n"
        "\top_pk_send(pkptr, MAC_TO_TX_STRM);",
        "br_mac tx_start",
    )


def patch_ecc(text: str) -> str:
    text = replace_once(
        text,
        '#include "include/br_support.h"\n',
        '#include "include/br_support.h"\n'
        "#define CSR_OPNET_TRACE_ECC 1\n"
        '#include "csr-opnet-trace.h"\n',
        "br_ecc trace include",
    )
    text = replace_in_function(
        text,
        "void\nbr_ecc_mt",
        "/* Place flag indicating accept/reject in transmission data block. */",
        "#endif\n\t\tFOUT",
        "#endif\n"
        "\t\tcsr_opnet_trace_ecc(op_sim_time(), pkptr, node_id, OPC_FALSE, "
        "\"prior_stage\", rxch_state_ptr);\n"
        "\t\tFOUT",
        "br_ecc prior-stage outcome",
    )
    return replace_once(
        text,
        "\t\top_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, accept);",
        "\t\top_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, accept);\n"
        "\t\tcsr_opnet_trace_ecc(op_sim_time(), pkptr, node_id, accept,\n"
        "\t\t\taccept ? \"accepted\" : "
        "(rxch_state_ptr->signal_lock ? \"half_duplex\" :\n"
        "\t\t\t(op_td_is_set(pkptr, OPC_TDA_RA_ND_FAIL) ? "
        "\"node_disabled\" : \"ecc\")),\n"
        "\t\t\trxch_state_ptr);",
        "br_ecc final outcome",
    )


def parse_slot(specification: str) -> tuple[int, int]:
    parts = specification.split(":")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("forced slot must be NODE:SLOT")
    try:
        node = int(parts[0], 0)
        slot = int(parts[1], 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if node < 0 or node > 0xFFFFFF or slot < 1 or slot > 255:
        raise argparse.ArgumentTypeError("forced slot is outside NODE:SLOT ranges")
    return node, slot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_dir", type=Path, help="directory with recovered sources")
    parser.add_argument("output_dir", type=Path, help="instrumented output directory")
    parser.add_argument(
        "--force-reservation-slot",
        action="append",
        default=[],
        required=True,
        type=parse_slot,
        metavar="NODE:SLOT",
    )
    parser.add_argument(
        "--activation-time",
        type=float,
        default=0.0,
        help="simulation time at which reservation controls become active",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace instrumented outputs already present in OUTPUT_DIR",
    )
    return parser


def instrument(arguments: argparse.Namespace) -> dict[str, object]:
    """Build one fail-closed instrumentation bundle and return its manifest."""

    slots = dict(arguments.force_reservation_slot)
    if len(slots) != len(arguments.force_reservation_slot):
        raise InstrumentationError("a forced reservation node was repeated")
    if not math.isfinite(arguments.activation_time) or arguments.activation_time < 0.0:
        raise InstrumentationError(
            "--activation-time must be finite and nonnegative"
        )
    if not arguments.source_dir.is_dir():
        raise InstrumentationError("source directory does not exist")
    if arguments.output_dir.exists() and not arguments.output_dir.is_dir():
        raise InstrumentationError("output path exists and is not a directory")
    if paths_overlap(arguments.source_dir, arguments.output_dir):
        raise InstrumentationError(
            "source and output directories must be disjoint and non-overlapping"
        )

    output_names = GENERATED_OUTPUTS + (MANIFEST_NAME,)
    existing = [
        name
        for name in output_names
        if (arguments.output_dir / name).exists()
        or (arguments.output_dir / name).is_symlink()
    ]
    if existing and not arguments.overwrite:
        raise InstrumentationError(
            "output files already exist; pass --overwrite: " + ", ".join(existing)
        )

    source_bytes: dict[str, bytes] = {}
    source_paths: dict[str, Path] = {}
    for name, expected in EXPECTED_SOURCES.items():
        path = resolve_source(arguments.source_dir, name)
        try:
            data = path.read_bytes()
        except OSError as error:
            raise InstrumentationError(f"cannot read {path}: {error}") from error
        actual = digest_bytes(data)
        if actual != expected:
            raise InstrumentationError(
                f"{name} SHA-256 {actual} does not match recovered source {expected}"
            )
        source_bytes[name] = data
        source_paths[name] = path

    header_path = Path(__file__).with_name("opnet") / "csr-opnet-trace.h"
    protected_inputs = {**source_paths, "csr-opnet-trace.h template": header_path}
    output_paths = {
        name: arguments.output_dir / name for name in output_names
    }
    for name in output_names:
        output_path = output_paths[name]
        if output_path.is_symlink():
            raise InstrumentationError(
                f"output {name} is a symbolic link; refusing --overwrite"
            )
        if output_path.exists() and not output_path.is_file():
            raise InstrumentationError(f"output {name} is not a regular file")
        for source_name, source_path in protected_inputs.items():
            if paths_alias(output_path, source_path):
                raise InstrumentationError(
                    f"output {name} aliases recovered source {source_name}"
                )
    for index, first_name in enumerate(output_names):
        for second_name in output_names[index + 1 :]:
            if paths_alias(output_paths[first_name], output_paths[second_name]):
                raise InstrumentationError(
                    f"outputs {first_name} and {second_name} alias one another"
                )

    try:
        outputs = {
            "br_app.pr.c": patch_app(
                source_bytes["br_app.pr.c"].decode("utf-8").replace("\r\n", "\n")
            ),
            "br_mac.pr.c": patch_mac(
                source_bytes["br_mac.pr.c"].decode("utf-8").replace("\r\n", "\n")
            ),
            "br_ecc.ps.c": patch_ecc(
                source_bytes["br_ecc.ps.c"].decode("utf-8").replace("\r\n", "\n")
            ),
        }
    except (UnicodeDecodeError, InstrumentationError) as error:
        raise InstrumentationError(str(error)) from error

    try:
        header_bytes = header_path.read_bytes()
        header = header_bytes.decode("utf-8")
    except (OSError, UnicodeDecodeError) as error:
        raise InstrumentationError(f"cannot read header template: {error}") from error
    header_digest = digest_bytes(header_bytes)
    if header_digest != EXPECTED_HEADER_TEMPLATE_SHA256:
        raise InstrumentationError(
            f"header template SHA-256 {header_digest} is not trusted"
        )
    cases = "\n".join(
        f"        case {node}: return {slot};" for node, slot in sorted(slots.items())
    )
    header = replace_once(
        header,
        "        /* CSR_FORCE_SLOT_CASES */",
        cases,
        "forced slot header marker",
    )
    header = replace_once(
        header,
        "    /* CSR_CONTROL_START_SECONDS */",
        f"    {arguments.activation_time!r}",
        "reservation control activation marker",
    )
    outputs["csr-opnet-trace.h"] = header

    encoded_outputs = {
        name: text.encode("utf-8") for name, text in outputs.items()
    }
    for name, expected_digest in EXPECTED_PATCHED_OUTPUT_SHA256.items():
        actual_digest = digest_bytes(encoded_outputs[name])
        if actual_digest != expected_digest:
            raise InstrumentationError(
                f"generated {name} SHA-256 {actual_digest} is not trusted"
            )
    manifest_sources: dict[str, object] = {
        name: {
            "output_sha256": (
                digest_bytes(encoded_outputs[name])
                if name in encoded_outputs
                else None
            ),
            "original_sha256": expected,
        }
        for name, expected in EXPECTED_SOURCES.items()
    }
    manifest_sources["csr-opnet-trace.h"] = {
        "output_sha256": digest_bytes(encoded_outputs["csr-opnet-trace.h"]),
        "original_sha256": header_digest,
    }
    generator_path = Path(__file__).resolve()
    for name, text in outputs.items():
        if name not in GENERATED_OUTPUTS:
            raise InstrumentationError(f"unexpected generated output {name}")
    try:
        generator_digest = digest_bytes(generator_path.read_bytes())
    except OSError as error:
        raise InstrumentationError(f"cannot hash generator: {error}") from error
    manifest = {
        "schema": SCHEMA,
        "trace_schema": TRACE_SCHEMA,
        "generator": {
            "name": generator_path.name,
            "sha256": generator_digest,
        },
        "forced_reservation_slots": {str(node): slot for node, slot in sorted(slots.items())},
        "activation_time_s": arguments.activation_time,
        "behavioral_control": BEHAVIORAL_CONTROL,
        "sources": manifest_sources,
    }
    manifest_bytes = (
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")

    try:
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        for name, encoded in encoded_outputs.items():
            (arguments.output_dir / name).write_bytes(encoded)
        manifest_path = arguments.output_dir / MANIFEST_NAME
        manifest_path.write_bytes(manifest_bytes)
    except OSError as error:
        raise InstrumentationError(f"cannot write instrumentation bundle: {error}") from error
    return manifest


def main() -> None:
    arguments = build_parser().parse_args()
    try:
        instrument(arguments)
    except InstrumentationError as error:
        raise SystemExit(f"error: {error}") from error
    manifest_path = arguments.output_dir / MANIFEST_NAME
    print(f"instrumented {len(PATCHED_SOURCES)} OPNET sources in {arguments.output_dir}")
    print(f"provenance manifest: {manifest_path}")


if __name__ == "__main__":
    main()
