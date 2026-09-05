#!/usr/bin/env python3
"""Fail-closed tests for controlled OPNET source instrumentation."""

from __future__ import annotations

import argparse
import importlib.util
import json
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


INSTRUMENTER = load_script(
    "csr_opnet_reservation_collision_instrumenter",
    "instrument-opnet-reservation-trace.py",
)


APP_SOURCE = (
    "#include <csr_api.h>\n"
    "static void ss_packet_generate(void)\n\t{\n"
    "\t/* Send the packet via the stream to the lower layer.\t*/\n"
    "\top_pk_send(pkptr, APP_TO_NWK_STRM);\n\t}\n"
    "static void proc_packet(void)\n\t{\n\t}\n"
)

MAC_SEND_ANCHOR = "\t// Send packet to transmitter\n\top_pk_send(pkptr, MAC_TO_TX_STRM);"
MAC_SOURCE = (
    "#include <csr_api.h>\n"
    "static void br_mac_init(void)\n\t{\n"
    '\top_ima_obj_attr_get(my_attributes.node_objid, "Node ID", &my_node_id);\n'
    "\t}\n"
    "static void start_search(void)\n\t{\n\t}\n"
    "static void prep_tx(void)\n\t{\n"
    "\top_intrpt_schedule_remote(op_sim_time(), MAC_START_SEARCH_CODE, "
    "my_attributes.battery_objid);\n\t}\n"
    "static void tsholdoffo(void)\n\t{\n"
    "\tTS_HOLDOFF_OVER = OPC_TRUE;\n\t}\n"
    "static void send_pk(void)\n\t{\n"
    "\tint\t\t\t\t\tdest, dest_type, dest_type_c, ackable, resendable;\n"
    "\tint\t\t\t\t\tseq_num, sgm_num, byte_count, pk_count;\n"
    "\t\tRESERVATION_SENT = OPC_TRUE;\n"
    f"{MAC_SEND_ANCHOR}\n\t}}\n"
    "static void post_tx(void)\n\t{\n\t}\n"
    "static void tslot_tasks(void)\n\t{\n"
    "\t\t\t\ttxslot_counter--;\n"
    "\t\t\t\tPREP_TX = OPC_TRUE;\n"
    "\t\t\t\tif (txslot_counter < 0)\n\t\t\t\t\t{}\n"
    "\t}\n"
    "static int get_slot(int nodes, int num_rslot)\n\t{\n"
    "\tslot = ts_ptr->slot;\n\t}\n"
    "static int renew_txslot(int islot)\n\t{\n\t}\n"
)

ECC_SOURCE = (
    '#include "include/br_support.h"\n'
    "void\nbr_ecc_mt (Packet *pkptr)\n\t{\n"
    "#endif\n\t\tFOUT\n"
    "\t\top_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, accept);\n"
    "\t}\n"
    "/* Place flag indicating accept/reject in transmission data block. */\n"
)


class ReservationCollisionInstrumenterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.output = self.root / "output"
        self.source.mkdir()
        self.source_data = {
            "br_app.pr.c": APP_SOURCE.encode("utf-8"),
            "br_mac.pr.c": MAC_SOURCE.encode("utf-8"),
            "br_ecc.ps.c": ECC_SOURCE.encode("utf-8"),
            "br_support.h": b"#define S0_DURATION 0.00051\n",
            "br_txdel.ps.c": b"/* tx delay reference */\n",
        }
        for name, data in self.source_data.items():
            (self.source / name).write_bytes(data)
        expected_sources = {
            name: INSTRUMENTER.digest_bytes(data)
            for name, data in self.source_data.items()
        }
        patched_outputs = {
            "br_app.pr.c": INSTRUMENTER.patch_app(APP_SOURCE),
            "br_mac.pr.c": INSTRUMENTER.patch_mac(MAC_SOURCE),
            "br_ecc.ps.c": INSTRUMENTER.patch_ecc(ECC_SOURCE),
        }
        expected_outputs = {
            name: INSTRUMENTER.digest_bytes(text.encode("utf-8"))
            for name, text in patched_outputs.items()
        }
        self.source_patch = mock.patch.dict(
            INSTRUMENTER.EXPECTED_SOURCES, expected_sources, clear=True
        )
        self.output_patch = mock.patch.dict(
            INSTRUMENTER.EXPECTED_PATCHED_OUTPUT_SHA256,
            expected_outputs,
            clear=True,
        )
        self.source_patch.start()
        self.output_patch.start()

    def tearDown(self) -> None:
        self.output_patch.stop()
        self.source_patch.stop()
        self.temporary.cleanup()

    def arguments(
        self,
        *,
        source: Path | None = None,
        output: Path | None = None,
        overwrite: bool = False,
    ) -> argparse.Namespace:
        return argparse.Namespace(
            source_dir=self.source if source is None else source,
            output_dir=self.output if output is None else output,
            force_reservation_slot=[(2, 5), (3, 5)],
            activation_time=59.9,
            overwrite=overwrite,
        )

    def source_snapshot(self) -> dict[str, bytes]:
        return {
            name: (self.source / name).read_bytes() for name in self.source_data
        }

    def test_emits_preamble_hook_and_recomputable_manifest_digests(self) -> None:
        manifest = INSTRUMENTER.instrument(self.arguments())
        mac = (self.output / "br_mac.pr.c").read_text(encoding="utf-8")
        expected_hook = (
            "csr_opnet_trace_tx_start(op_sim_time(), my_node_id, dest, seq_num, "
            "payload_speed, (int)f1/8,\n"
            '\t\tstrcmp(pk_format, "br_OTA_LongPream") == 0 ? "long" :\n'
            '\t\t(strcmp(pk_format, "br_OTA_ShortPream") == 0 ? "short" : '
            '"unknown"), txslot_counter);\n'
            "\top_pk_send(pkptr, MAC_TO_TX_STRM);"
        )
        self.assertEqual(mac.count("csr_opnet_trace_tx_start("), 1)
        self.assertIn(expected_hook, mac)
        self.assertEqual(manifest["trace_schema"], INSTRUMENTER.TRACE_SCHEMA)
        self.assertEqual(set(manifest["sources"]), set(INSTRUMENTER.EXPECTED_SOURCES) | {"csr-opnet-trace.h"})
        for name in INSTRUMENTER.GENERATED_OUTPUTS:
            self.assertEqual(
                manifest["sources"][name]["output_sha256"],
                INSTRUMENTER.digest_bytes((self.output / name).read_bytes()),
            )
        for name, expected in INSTRUMENTER.EXPECTED_SOURCES.items():
            self.assertEqual(manifest["sources"][name]["original_sha256"], expected)
        for name in INSTRUMENTER.AIRTIME_REFERENCE_SOURCES:
            self.assertIsNone(manifest["sources"][name]["output_sha256"])
        generator = ROOT / "utils" / INSTRUMENTER.GENERATOR_NAME
        self.assertEqual(
            manifest["generator"]["sha256"],
            INSTRUMENTER.digest_bytes(generator.read_bytes()),
        )

    def test_preamble_send_anchor_is_unique_and_function_scoped(self) -> None:
        for source in (
            MAC_SOURCE.replace(MAC_SEND_ANCHOR, "/* missing send */"),
            MAC_SOURCE.replace(MAC_SEND_ANCHOR, MAC_SEND_ANCHOR + "\n" + MAC_SEND_ANCHOR),
            MAC_SOURCE.replace(MAC_SEND_ANCHOR, "/* moved */") + MAC_SEND_ANCHOR,
        ):
            with self.subTest(count=source.count(MAC_SEND_ANCHOR)):
                with self.assertRaises(INSTRUMENTER.InstrumentationError):
                    INSTRUMENTER.patch_mac(source)

    def test_same_nested_or_symlinked_output_directory_is_rejected(self) -> None:
        original = self.source_snapshot()
        nested = self.source / "nested-output"
        alias = self.root / "source-alias"
        alias.symlink_to(self.source, target_is_directory=True)
        for output in (self.source, nested, alias):
            with self.subTest(output=output):
                with self.assertRaisesRegex(
                    INSTRUMENTER.InstrumentationError, "non-overlapping"
                ):
                    INSTRUMENTER.instrument(
                        self.arguments(output=output, overwrite=True)
                    )
                self.assertEqual(self.source_snapshot(), original)
        self.assertFalse(nested.exists())

    def test_output_alias_to_source_or_header_is_rejected_and_preserved(self) -> None:
        original = self.source_snapshot()
        header = ROOT / "utils" / "opnet" / "csr-opnet-trace.h"
        for alias_kind, candidate_name, target in (
            ("source_hardlink", "br_app.pr.c", self.source / "br_mac.pr.c"),
            ("source_symlink", "br_app.pr.c", self.source / "br_mac.pr.c"),
            ("header_hardlink", INSTRUMENTER.MANIFEST_NAME, header),
        ):
            with self.subTest(alias_kind=alias_kind):
                output = self.root / f"output-{alias_kind}"
                output.mkdir()
                candidate = output / candidate_name
                if alias_kind == "source_symlink":
                    candidate.symlink_to(target)
                else:
                    candidate.hardlink_to(target)
                with self.assertRaises(INSTRUMENTER.InstrumentationError):
                    INSTRUMENTER.instrument(
                        self.arguments(output=output, overwrite=True)
                    )
                self.assertEqual(self.source_snapshot(), original)

    def test_managed_output_aliases_are_rejected_before_write(self) -> None:
        self.output.mkdir()
        first = self.output / "br_app.pr.c"
        second = self.output / "br_mac.pr.c"
        first.write_bytes(b"shared sentinel\n")
        second.hardlink_to(first)
        with self.assertRaisesRegex(INSTRUMENTER.InstrumentationError, "one another"):
            INSTRUMENTER.instrument(self.arguments(overwrite=True))
        self.assertEqual(first.read_bytes(), b"shared sentinel\n")
        self.assertEqual(second.read_bytes(), b"shared sentinel\n")

    def test_lone_manifest_counts_as_existing_output_and_is_preserved(self) -> None:
        self.output.mkdir()
        manifest_path = self.output / INSTRUMENTER.MANIFEST_NAME
        manifest_path.write_bytes(b"manifest sentinel\n")
        with self.assertRaisesRegex(
            INSTRUMENTER.InstrumentationError, INSTRUMENTER.MANIFEST_NAME
        ):
            INSTRUMENTER.instrument(self.arguments())
        self.assertEqual(manifest_path.read_bytes(), b"manifest sentinel\n")
        self.assertEqual([path.name for path in self.output.iterdir()], [INSTRUMENTER.MANIFEST_NAME])

    def test_dangling_symlink_counts_as_existing_output(self) -> None:
        self.output.mkdir()
        link = self.output / "br_app.pr.c"
        link.symlink_to(self.root / "missing-target")
        with self.assertRaisesRegex(INSTRUMENTER.InstrumentationError, "already exist"):
            INSTRUMENTER.instrument(self.arguments())
        self.assertTrue(link.is_symlink())

    def test_digest_or_anchor_rejection_preserves_all_existing_outputs(self) -> None:
        mac_path = self.source / "br_mac.pr.c"
        original = self.source_snapshot()
        mac_path.write_bytes(b"wrong source bytes\n")
        with self.assertRaisesRegex(INSTRUMENTER.InstrumentationError, "SHA-256"):
            INSTRUMENTER.instrument(self.arguments())
        self.assertFalse(self.output.exists())
        mac_path.write_bytes(original["br_mac.pr.c"])

        self.output.mkdir()
        sentinels = {}
        for name in INSTRUMENTER.GENERATED_OUTPUTS + (INSTRUMENTER.MANIFEST_NAME,):
            data = f"sentinel {name}\n".encode("utf-8")
            (self.output / name).write_bytes(data)
            sentinels[name] = data
        broken = MAC_SOURCE.replace(MAC_SEND_ANCHOR, "/* missing send */")
        mac_path.write_text(broken, encoding="utf-8")
        expected = dict(INSTRUMENTER.EXPECTED_SOURCES)
        expected["br_mac.pr.c"] = INSTRUMENTER.digest_bytes(broken.encode("utf-8"))
        with mock.patch.dict(INSTRUMENTER.EXPECTED_SOURCES, expected, clear=True):
            with self.assertRaises(INSTRUMENTER.InstrumentationError):
                INSTRUMENTER.instrument(self.arguments(overwrite=True))
        for name, data in sentinels.items():
            self.assertEqual((self.output / name).read_bytes(), data)
        mac_path.write_bytes(original["br_mac.pr.c"])

    def test_safe_overwrite_preserves_unrelated_output(self) -> None:
        INSTRUMENTER.instrument(self.arguments())
        unrelated = self.output / "keep.txt"
        unrelated.write_bytes(b"keep\n")
        manifest = INSTRUMENTER.instrument(self.arguments(overwrite=True))
        self.assertEqual(unrelated.read_bytes(), b"keep\n")
        self.assertEqual(
            json.loads((self.output / INSTRUMENTER.MANIFEST_NAME).read_text()),
            manifest,
        )


if __name__ == "__main__":
    unittest.main()
