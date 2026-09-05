#!/usr/bin/env python3
"""Build, exercise, and content-address one CSR/ns-3 release candidate.

The harness reads clean Git objects, stages immutable source archives, builds
all 38 CSR programs from that one CSR tree, executes the release evidence
workflow, and produces a closed evidence set with SHA256SUMS.  A canonical
aggregate comparison may either pass or retain a numeric residual; structural
misalignment, missing data, and unaccounted packet identities always fail.
"""

from __future__ import annotations

import argparse
import ast
from collections import Counter, defaultdict
import ctypes
import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import time
from typing import Any, Iterable, Sequence


sys.dont_write_bytecode = True


NS3_COMMIT = "6b5cd24ea80713ce16d88575869aedd6f432bdae"
GIT = str(Path("/usr/bin/git").resolve())
EXPECTED_SCENARIO_SHA256 = (
    "90b143d93c13c6c2761bc5f2875ccc3fff98f85af6f2550370e435df2aaabcfc"
)
EXPECTED_LATENCY_AGGREGATE_SHA256 = (
    "afa390ed606fac3075e0a82eae04648bc6f9946cdcb0639aa7b5187c42692bf8"
)

EXTERNAL_INPUTS: dict[str, tuple[str, str]] = {
    "source_archive": (
        "",
        "5ae5a14ba36918e19d274a9f2385eb00dd9b63de518b72b728fc373a29be5b73",
    ),
    "multihop_network": (
        "multihop/blue_radio_campus-multihop.nt.m",
        "7052743dc082f3ba34ee2bd8ac70068adf8091efaf4ff8cb2116faff088abcd7",
    ),
    "multihop_environment": (
        "multihop/blue_radio_campus-multihop-DES-1.ef",
        "a5ce1d570adbb9dd0e2720506bcbf645abf4d53d563e3ccb0fbeac2f6d7feff9",
    ),
    "multihop_vectors": (
        "multihop/blue_radio_campus-multihop-DES-1.ov",
        "b643bee5c1d0260581a0135c0add5c87441bd98f70f66388fac0bf95096c39ee",
    ),
    "multihop_probes": (
        "multihop/blue_radio_campus-multihop.pb.m",
        "2871b54d0c531198f8ff77b1aa2a3cc81da6f05814c9ca00e64ed6a544a6470f",
    ),
    "multihop_executable": (
        "multihop/blue_radio_campus-multihop.dev32.i1.nt.so",
        "adb97c54f7566439f1404e972d3d777a3bca613e2a965bf12f03353fb009d9af",
    ),
    "latency_network": (
        "latency/CSR_Validation-2_Nodes_Latency.nt.m",
        "0e9791d395e33a9064a16a65ca83afcb7b25079a5e4a3285c802faa20a355785",
    ),
    "latency_environment": (
        "latency/CSR_Validation-2_Nodes_Latency-DES-1.ef",
        "8747d8f89d320f8914f87178de7e63009c1bf46418bfd2311d8f0c5576119307",
    ),
    "latency_vectors": (
        "latency/CSR_Validation-2_Nodes_Latency-DES-1.ov",
        "c9ebc87410b68780abac187a5164939cd95ffe5ba4a5a7e393012117d4235493",
    ),
    "latency_probes": (
        "latency/CSR_Validation-2_Nodes_Latency.pb.m",
        "a43c81806e6216491cad831482d509481239934b5f3d8980df4b588fb6d9ff94",
    ),
}

PRIMARY_SOURCE_ARCHIVE_NAME = "CSR project examples.zip"
SUPPLEMENTAL_SOURCE_INPUTS: dict[str, str] = {
    "more missing files.zip": (
        "84c2d49170d507ff2cc69f251e818db0b3fd9a89f44074a9b639ab1ab22f7827"
    ),
    "routing.lib": (
        "406aea05652fded887bb836bf9a6383c61a2a8caff26e7e5452d13adc5efe0d7"
    ),
    "routes(1).c": (
        "88f3a99a5ba74071ae3ebe3900c9f0033841ce6dcdee01e7d93b2b408842a17d"
    ),
    "Modulation_tables.zip": (
        "33a48ba724ea759a4725b0a321a655988c898dc9e0b29fe856d44cf92c0ccbb5"
    ),
    "Project files for chatgpt(1).zip": (
        "82e0c3d3fbb95ccfcd42bfca067c347681cc5f818e95f42018de746a3cc03d5a"
    ),
}
SUPPLIED_SOURCE_DIRECTORY_INPUTS: dict[str, str] = {
    PRIMARY_SOURCE_ARCHIVE_NAME: EXTERNAL_INPUTS["source_archive"][1],
    **SUPPLEMENTAL_SOURCE_INPUTS,
}
SUPPLEMENTAL_SCOPE_DISPOSITION = (
    "hash_bound_provenance_reference_not_wholesale_parity_scope"
)
RELEASE_HARNESS_REPOSITORY_PATHS: dict[str, str] = {
    "certification_harness": "utils/release/certify_release.py",
    "classifier_self_test": "utils/release/test_certify_release_classifier.py",
}
UNBOUND_HISTORICAL_SOURCE_REFERENCES = (
    "arlSecurity.c",
    "arlSecurity.h",
    "packetTypes.c",
    "packetTypes.h",
    "msectime.c",
    "msectime.h",
    "arlAuthTag.c",
    "arlContext.c",
    "arlEncrypt.c",
    "arlKey.c",
)

EXPECTED_ALL_VALUE_LENGTHS = [10357, 10364, 11017, 8, 0, 12]
SPECIAL_TARGETS = {"csr-mac-demo-split", "csr-opnet-scenario-runner"}
SUPPORTED_PACKET_PATH_SCHEMAS = {
    "csr-packet-path-analysis-v1",
    "csr-packet-path-analysis-v2",
}
PACKET_PATH_V2_MATCHING_CONTRACT = (
    "exact directed-HOP identity plus queue-delay-constrained unique bipartite "
    "queue-copy matching; never FIFO"
)
PACKET_PATH_V2_UNDELIVERED_PATH_CONTRACT = (
    "path contains only observed NWK nodes; hop_lineage may contain one "
    "terminal admitted but unreceived HOP edge"
)
PACKET_PATH_V2_QUEUE_DELAY_STATISTIC = "NWK.Network Queuing Delay (sec)"
PACKET_PATH_V2_QUEUE_DELAY_ADJACENCY = (
    "immediately preceding CSV row, consecutive event index, same node and timestamp"
)
HISTORICAL_HOP_SECURITY_PROFILE = "hist-adb97c54-bare"
HIDDEN_HISTORICAL_HOP_SECURITY_PROFILE = "hist-dd3f38e8-bare"
HIDDEN_HISTORICAL_EXECUTABLE_SHA256 = (
    "dd3f38e8d33700b61f9e360a737ba34e56cb75b2570eb2960a02de381ed0fff0"
)
HISTORICAL_HOP_SECURITY_BEHAVIOR = {
    "ordinary_data": "bare",
    "ack_dack": "bare",
}
HISTORICAL_HOP_SECURITY_BINDINGS = {
    HISTORICAL_HOP_SECURITY_PROFILE: {
        "application_profile": "legacy-send-only-no-dscp",
        "mac_profile": "hist-2014-next-tslot-modulo-probe",
        "source_executable_sha256": EXTERNAL_INPUTS["multihop_executable"][1],
    },
    HIDDEN_HISTORICAL_HOP_SECURITY_PROFILE: {
        "application_profile": "legacy-send-to-from-no-dscp",
        "mac_profile": "hist-2015-fine-one-based-table-no-avoid",
        "source_executable_sha256": HIDDEN_HISTORICAL_EXECUTABLE_SHA256,
    },
}
CANONICAL_APPLICATION_AGGREGATE_STATISTICS = (
    "Generator.Traffic Sent (packets/sec)",
    "Generator.Traffic Sent (bits/sec)",
    "Generator.Packet Size (bits)",
    "Sink.Traffic Received (packets/sec)",
    "Sink.Traffic Received (packets)",
    "Sink.Traffic Received (bits/sec)",
    "Sink.Traffic Received (bits)",
    "Sink.End-to-End Delay (seconds)",
)
APPLICATION_PACKET_SIZE_STATISTIC = "Generator.Packet Size (bits)"
PROTOCOL_AGGREGATE_STATISTICS = (
    "HOP.Resend Queue Size (packets)",
    "MAC.ACK Queue Size (packets)",
    "MAC.Tx Queue Size (packets)",
    "MAC.Tx Queuing Delay (sec)",
)
PROTOCOL_AGGREGATE_SEMANTICS = {
    "HOP.Resend Queue Size (packets)": ("packets", "bucket_sample_mean"),
    "MAC.ACK Queue Size (packets)": ("packets", "bucket_sample_mean"),
    "MAC.Tx Queue Size (packets)": ("packets", "bucket_sample_mean"),
    "MAC.Tx Queuing Delay (sec)": ("s", "bucket_sample_mean"),
}
PROTOCOL_AGGREGATE_START_S = 360.0
PROTOCOL_AGGREGATE_STOP_S = 6000.0
PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES = 95
ADMISSION_LEG_COLUMNS = (
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
APP_DIAGNOSTIC_COLUMNS = (
    "schema",
    "scenario",
    "application_profile",
    "flow_index",
    "source",
    "configured_destination",
    "destination_mode",
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
APP_DIAGNOSTIC_COUNTERS = (
    "attempts",
    "admitted",
    "blocked_discovery",
    "blocked_topology",
    "blocked_gateway_route",
    "blocked_destination",
    "blocked_nsdp",
)

# This is the complete visible direct-source set supplied for the parity audit.
# The hashes are immutable input identities, not generated expectations.
OPNET_SOURCE_INPUTS: dict[str, str] = {
    "01-br_snr.ps.c": "a866ce49698d5e9d779b38193a078e1f8a1ef3abad2df8fe5e18e02503bd7924",
    "02-br_inoise.ps.c": "5538fcc949ecb8049d27a2e6a342727499f0fbe325b8fe2059d3e1fe996d6af4",
    "03-br_ber.ps.c": "3f98be10ce70c79e5aa47f724e97ec4fc947e5ca1f820050c3532680f1241c7f",
    "04-br_support.h": "9855b95029a2e15190976ef5ae926d028a98ef91db5d7567597c5f01cc7f37ce",
    "05-br_rxgroup.ps.c": "3c02e9a01d810e0923e2087eba674dd17063b5816cdc53e69aedbc3162e396ee",
    "06-br_power.ps.c": "dc6eac68ff971e31b492bec4f39683367464da31c242fbb7e4e6b8220b8925ae",
    "07-br_txdel.ps.c": "6b9ae9213aea8546b97069df5fe5315e8a7c49203d7d27c3d6e559e452a49463",
    "08-br_ecc.ps.c": "fbc5d5fb63c08eb3a5b2afec0baacf7c59182cf07382ed5fd3a61bd61f53d83c",
    "09-br_app.pr.c": "0e3dfeefc90da99b78f1b6305ccffa765de4b35710414bd85aa9fc30d8bb6dde",
    "10-br_supervisor.pr.c": "edddca67eedee42822bafd84d315161a5aece69715c7acc64c2c6d79dcf93b5c",
    "11-br_closure.ps.c": "422745b67ad2aa9883164148322ab85866d4e89f5abb02020d4141f7d21be7a5",
    "12-br_battery.pr.c": "5d03afcac7be4fbaf399f6b28c246b1fae83f689db8dd5baecad407c70fc17c0",
    "13-br_nwk.pr.c": "9095b90f860de84c466edc14fb73f8efa2e2f583234a107a3dd8bc59f18aa9d8",
    "14-br_mac.pr.c": "d1bc9f7d57da4cc490a835e19b26eac47dc0088629ecd15abfaaa5a0734be27a",
    "15-br_hop.pr.c": "27fdd8a3c71bce367abb37d2ac8656f1f2440c8e5221f91a983bda42330d4b62",
    "16-br_error_all_stats.ps.c": "38a41f338021d8828ffd8271ee30c075a8c84db5ce6afc26d64e3966638c5508",
}

OPNET_SCOPE_EXEMPTIONS: dict[str, dict[str, str]] = {
    "10-br_supervisor.pr.c": {
        "component": "supervisory_layer",
        "disposition": "explicitly_exempt_from_this_release",
        "reason": "Project-authorized phased exemption; no supervisory-layer parity claim is made.",
    },
    "12-br_battery.pr.c": {
        "component": "battery_and_energy",
        "disposition": "explicitly_exempt_from_this_release",
        "reason": "Project-authorized phased exemption; no battery or energy parity claim is made.",
    },
}
PacketKey = tuple[str, str, str]
LineageIdentity = tuple[str, str, str, str, str, str]
ALLOWED_SEALED_HIDDEN_PATHS = {"manifests/.lock-ns3_linux_build"}


class CertificationError(RuntimeError):
    """A release gate failed closed."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise CertificationError(message)


def hop_lineage_path_relation(
    path: list[int], hop_lineage: list[dict[str, Any]], delivered: bool
) -> str | None:
    """Classify an exact observed path or one terminal unreceived HOP attempt."""

    observed_edges = len(path) - 1
    if len(hop_lineage) < observed_edges:
        return None
    prefix_matches = all(
        hop_lineage[index]["from"] == path[index]
        and hop_lineage[index]["to"] == path[index + 1]
        for index in range(observed_edges)
    )
    if not prefix_matches:
        return None
    if len(hop_lineage) == observed_edges:
        return "exact_observed_path"
    if delivered or len(hop_lineage) != observed_edges + 1:
        return None
    terminal = hop_lineage[-1]
    if terminal["from"] != path[-1] or terminal["to"] in path:
        return None
    return "terminal_admitted_unreceived"


def json_load(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CertificationError(f"cannot read JSON {path}: {error}") from error
    require(isinstance(value, dict), f"JSON root is not an object: {path}")
    return value


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def relative(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def command_text(arguments: Sequence[os.PathLike[str] | str]) -> str:
    return shlex.join(str(value) for value in arguments)


def semantic_aggregate_rows(path: Path) -> list[dict[str, str]]:
    """Load aggregate rows while excluding provenance-only trace identities."""

    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        row.pop("source_file", None)
        row.pop("source_file_sha256", None)
    return rows


def stable_json_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, ensure_ascii=True, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def is_within(path: Path, directory: Path) -> bool:
    try:
        path.resolve().relative_to(directory.resolve())
        return True
    except ValueError:
        return False


def validate_ns3_lock_binding(
    record: Any, output: Path
) -> tuple[Path, Path]:
    """Verify the parsed ns-3 build lock and its frozen byte-exact copy."""

    require(
        isinstance(record, dict)
        and set(record)
        == {"source_path", "frozen_artifact", "sha256", "size_bytes"},
        "ns-3 build-lock binding record is malformed",
    )
    source_path = record.get("source_path")
    frozen_artifact = record.get("frozen_artifact")
    digest = record.get("sha256")
    size_bytes = record.get("size_bytes")
    require(
        isinstance(source_path, str)
        and Path(source_path).is_absolute()
        and isinstance(frozen_artifact, str)
        and frozen_artifact == "manifests/.lock-ns3_linux_build"
        and isinstance(digest, str)
        and re.fullmatch(r"[0-9a-f]{64}", digest) is not None
        and isinstance(size_bytes, int)
        and not isinstance(size_bytes, bool)
        and size_bytes >= 0,
        "ns-3 build-lock binding fields are malformed",
    )
    source = Path(source_path)
    frozen = output / frozen_artifact
    require(
        source.is_file()
        and not source.is_symlink()
        and source.stat().st_nlink == 1
        and frozen.is_file()
        and not frozen.is_symlink()
        and frozen.stat().st_nlink == 1
        and not is_within(source, output)
        and is_within(frozen, output)
        and source.resolve() != frozen.resolve()
        and not os.path.samefile(source, frozen)
        and source.stat().st_size == size_bytes
        and frozen.stat().st_size == size_bytes
        and sha256(source) == digest
        and sha256(frozen) == digest,
        "ns-3 build lock or frozen binding changed",
    )
    return source, frozen


def enumerate_sealed_evidence(
    output: Path, roots: Sequence[Path]
) -> list[Path]:
    """Return the complete regular-file set covered by the checksum list."""

    required_lock = output / "manifests" / ".lock-ns3_linux_build"
    require(
        required_lock.is_file() and not required_lock.is_symlink(),
        f"required frozen ns-3 build lock is absent or invalid: {required_lock}",
    )
    included: list[Path] = []
    for root in roots:
        require(
            is_within(root, output)
            and root.is_dir()
            and not root.is_symlink(),
            f"frozen evidence root is absent, outside output, or symlinked: {root}",
        )
        for path in root.rglob("*"):
            require(
                not path.is_symlink()
                and (path.is_dir() or path.is_file()),
                f"frozen evidence contains a symlink or non-regular entry: {path}",
            )
            relative_path = path.relative_to(output)
            if any(part.startswith(".") for part in relative_path.parts):
                require(
                    relative_path.as_posix() in ALLOWED_SEALED_HIDDEN_PATHS
                    and path.is_file(),
                    f"frozen evidence contains an unexpected hidden entry: {path}",
                )
            if path.is_file():
                require(
                    path.stat().st_nlink == 1,
                    f"frozen evidence file has an unsafe hard-link count: {path}",
                )
                included.append(path)
    release_manifest = output / "release-manifest.json"
    require(
        release_manifest.is_file()
        and not release_manifest.is_symlink()
        and release_manifest.stat().st_nlink == 1,
        "frozen release manifest is absent, linked, or invalid",
    )
    included.append(release_manifest)
    return sorted(set(included), key=lambda path: relative(path, output))


def validate_sealed_evidence_snapshot(
    output: Path,
    roots: Sequence[Path],
    sealed_hashes: dict[Path, str],
    *,
    additional_files: Sequence[Path] = (),
) -> None:
    """Fail if the post-enumeration file set or any sealed content changed."""

    def enumerate_snapshot() -> list[Path]:
        expected_root_entries = set(roots) | {
            output / "release-manifest.json",
            *additional_files,
        }
        require(
            set(output.iterdir()) == expected_root_entries
            and all(not path.is_symlink() for path in expected_root_entries),
            "frozen output root changed during checksum sealing",
        )
        current = enumerate_sealed_evidence(output, roots)
        for path in additional_files:
            require(
                is_within(path, output)
                and path.parent == output
                and path.is_file()
                and not path.is_symlink()
                and path.stat().st_nlink == 1,
                f"final checksum artifact is absent, invalid, or misplaced: {path}",
            )
            current.append(path)
        return sorted(set(current), key=lambda path: relative(path, output))

    def validate_snapshot() -> None:
        current = enumerate_snapshot()
        require(
            set(current) == set(sealed_hashes),
            "frozen evidence file set changed during final checksum sealing",
        )
        require(
            all(sha256(path) == sealed_hashes[path] for path in current),
            "frozen evidence content changed during final checksum sealing",
        )

    validate_snapshot()
    validate_snapshot()


class EvidenceMutationMonitor:
    """Reject Linux inotify-reported changes during final seal validation."""

    _BASE_MASK = (
        0x00000002  # IN_MODIFY
        | 0x00000004  # IN_ATTRIB
        | 0x00000008  # IN_CLOSE_WRITE
        | 0x00000400  # IN_DELETE_SELF
        | 0x00000800  # IN_MOVE_SELF
        | 0x00002000  # IN_UNMOUNT
        | 0x00004000  # IN_Q_OVERFLOW
        | 0x00008000  # IN_IGNORED
        | 0x02000000  # IN_DONT_FOLLOW
    )
    _DIRECTORY_MASK = (
        _BASE_MASK
        | 0x00000040  # IN_MOVED_FROM
        | 0x00000080  # IN_MOVED_TO
        | 0x00000100  # IN_CREATE
        | 0x00000200  # IN_DELETE
        | 0x01000000  # IN_ONLYDIR
    )
    _FILE_MASK = _BASE_MASK

    def __init__(self, root: Path) -> None:
        self.root = root
        self.fd = -1
        self._libc: Any = None

    def __enter__(self) -> EvidenceMutationMonitor:
        require(
            platform.system() == "Linux",
            "final evidence mutation monitor requires Linux inotify",
        )
        self._libc = ctypes.CDLL(None, use_errno=True)
        init = self._libc.inotify_init1
        init.argtypes = [ctypes.c_int]
        init.restype = ctypes.c_int
        add_watch = self._libc.inotify_add_watch
        add_watch.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_uint32]
        add_watch.restype = ctypes.c_int
        self.fd = init(os.O_NONBLOCK | os.O_CLOEXEC)
        require(
            self.fd >= 0,
            f"cannot initialize final evidence mutation monitor: errno={ctypes.get_errno()}",
        )
        try:
            pending = [self.root]
            watched: set[Path] = set()
            while pending:
                directory = pending.pop()
                require(
                    directory.is_dir() and not directory.is_symlink(),
                    f"cannot monitor invalid evidence directory: {directory}",
                )
                resolved = directory.resolve()
                if resolved in watched:
                    continue
                descriptor = add_watch(
                    self.fd, os.fsencode(directory), self._DIRECTORY_MASK
                )
                require(
                    descriptor >= 0,
                    f"cannot watch evidence directory {directory}: errno={ctypes.get_errno()}",
                )
                watched.add(resolved)
                with os.scandir(directory) as entries:
                    for entry in entries:
                        require(
                            not entry.is_symlink(),
                            f"cannot monitor symlinked evidence entry: {entry.path}",
                        )
                        if entry.is_dir(follow_symlinks=False):
                            pending.append(Path(entry.path))
                        else:
                            require(
                                entry.is_file(follow_symlinks=False),
                                f"cannot monitor non-regular evidence entry: {entry.path}",
                            )
                            descriptor = add_watch(
                                self.fd,
                                os.fsencode(entry.path),
                                self._FILE_MASK,
                            )
                            require(
                                descriptor >= 0,
                                f"cannot watch evidence file {entry.path}: errno={ctypes.get_errno()}",
                            )
            self.require_quiet()
            return self
        except Exception:
            os.close(self.fd)
            self.fd = -1
            raise

    def require_quiet(self) -> None:
        require(self.fd >= 0, "final evidence mutation monitor is not active")
        try:
            changed = os.read(self.fd, 1024 * 1024)
        except BlockingIOError:
            changed = b""
        require(
            not changed,
            "frozen evidence mutated during the final monitored sealing interval",
        )

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        try:
            if exc_type is None:
                self.require_quiet()
        finally:
            if self.fd >= 0:
                os.close(self.fd)
                self.fd = -1


def validate_quiescent_sealed_evidence_snapshot(
    output: Path,
    roots: Sequence[Path],
    sealed_hashes: dict[Path, str],
    *,
    additional_files: Sequence[Path] = (),
) -> None:
    """Validate a checksum snapshot with inotify mutation defense-in-depth."""

    with EvidenceMutationMonitor(output):
        validate_sealed_evidence_snapshot(
            output,
            roots,
            sealed_hashes,
            additional_files=additional_files,
        )


def safe_remote(value: str) -> dict[str, str]:
    redacted = re.sub(r"(?<=://)[^/@\s]+@", "[redacted]@", value)
    return {
        "display": redacted,
        "sha256": hashlib.sha256(value.encode("utf-8")).hexdigest(),
    }


def validate_named_file_set(
    directory: Path,
    expected: dict[str, str],
    *,
    description: str = "direct OPNET source",
    include_hidden: bool = False,
) -> dict[str, dict[str, Any]]:
    """Validate an exact regular-file set and its immutable hashes."""

    require(
        directory.is_dir() and not directory.is_symlink(),
        f"required input directory is absent or symlinked: {directory}",
    )
    members = {
        path.name: path
        for path in directory.iterdir()
        if include_hidden or not path.name.startswith(".")
    }
    require(
        set(members) == set(expected),
        f"{description} filename set drift: "
        f"missing={sorted(set(expected) - set(members))}, "
        f"extra={sorted(set(members) - set(expected))}",
    )
    records: dict[str, dict[str, Any]] = {}
    for name, expected_digest in sorted(expected.items()):
        path = members[name]
        require(
            path.is_file() and not path.is_symlink(),
            f"{description} is not a regular file or is symlinked: {path}",
        )
        actual = sha256(path)
        require(
            actual == expected_digest,
            f"{description} hash mismatch for {name}: {actual}",
        )
        records[name] = {
            "path": path.absolute(),
            "sha256": actual,
            "size_bytes": path.stat().st_size,
        }
    return records


def parse_unittest_count(output: str) -> int:
    """Return the one unittest summary count, rejecting absent/ambiguous runs."""

    matches = re.findall(r"^Ran\s+(\d+)\s+tests?\s+in\s+", output, re.MULTILINE)
    require(len(matches) == 1, "test log does not contain one unittest summary")
    count = int(matches[0])
    require(count > 0, "test suite discovered zero tests")
    require(
        re.search(r"^OK(?:\s+\([^\n]+\))?$", output, re.MULTILINE) is not None,
        "test suite lacks final OK",
    )
    return count


def parse_discovered_test_count(output: str) -> int:
    """Parse the isolated discovery probe's fail-closed count marker."""

    matches = re.findall(r"^CSR_DISCOVERED_TESTS=(\d+)$", output, re.MULTILINE)
    require(len(matches) == 1, "test discovery log lacks one count marker")
    count = int(matches[0])
    require(count > 0, "test discovery returned zero tests")
    return count


def parse_cmake_cache(text: str) -> dict[str, tuple[str, str]]:
    """Parse unique typed CMake cache assignments without substring matching."""

    records: dict[str, tuple[str, str]] = {}
    for row_number, line in enumerate(text.splitlines(), start=1):
        if not line or line.startswith("//") or line.startswith("#"):
            continue
        match = re.fullmatch(r"([^:=]+):([^=]+)=(.*)", line)
        if match is None:
            continue
        name, value_type, value = match.groups()
        require(
            name not in records,
            f"CMake cache repeats assignment {name!r} at row {row_number}",
        )
        records[name] = (value_type, value)
    return records


def validate_historical_security_provenance(
    canonical_identity: dict[str, Any],
    runner_configuration: dict[str, Any],
    archived_executable_digest: str,
) -> None:
    """Require the archived executable's joint DATA and ACK/DACK projection."""

    matches = [
        (profile, binding)
        for profile, binding in HISTORICAL_HOP_SECURITY_BINDINGS.items()
        if binding["source_executable_sha256"] == archived_executable_digest
    ]
    require(
        len(matches) == 1,
        "archived executable digest is not uniquely bound historical evidence",
    )
    profile, binding = matches[0]
    for label, record in (
        ("canonical_scenario", canonical_identity),
        ("runner", runner_configuration),
    ):
        validate_effective_hop_security_source_binding(record)
        require(
            isinstance(record, dict)
            and record.get("application_profile")
            == binding["application_profile"]
            and record.get("mac_profile")
            == binding["mac_profile"]
            and record.get("hop_security_profile")
            == profile
            and record.get("hop_security_profile_origin") == "explicit"
            and record.get("hop_security_behavior")
            == HISTORICAL_HOP_SECURITY_BEHAVIOR
            and record.get("ack_envelope_profile") == "unspecified"
            and record.get("source_executable_sha256")
            == archived_executable_digest,
            f"aggregate {label} DATA/ACK security provenance drift",
        )


def validate_effective_hop_security_source_binding(record: dict[str, Any]) -> None:
    """Bind every effective HOP profile/origin to allowed executable evidence."""

    require(isinstance(record, dict), "HOP security provenance record is malformed")
    profile = record.get("hop_security_profile")
    origin = record.get("hop_security_profile_origin")
    source_digest = record.get("source_executable_sha256")
    require(
        origin in {"explicit", "legacy_ack_alias", "missing"},
        "HOP security profile origin is unsupported",
    )
    if profile in HISTORICAL_HOP_SECURITY_BINDINGS:
        expected_digest = HISTORICAL_HOP_SECURITY_BINDINGS[profile][
            "source_executable_sha256"
        ]
        require(
            origin in {"explicit", "legacy_ack_alias"}
            and source_digest == expected_digest,
            "effective historical HOP profile lacks exact executable binding",
        )
        return
    require(
        profile == "production-pairwise16" and source_digest in {"", None},
        "effective production HOP profile claims historical executable evidence",
    )


def _comparison_series_by_statistic(
    report: dict[str, Any], expected: Sequence[str], label: str
) -> dict[str, dict[str, Any]]:
    """Return one exact report series per required statistic."""

    series = report.get("series")
    require(isinstance(series, list), f"{label} comparison series ledger is absent")
    records: dict[str, dict[str, Any]] = {}
    for record in series:
        require(isinstance(record, dict), f"{label} comparison series record is malformed")
        statistic = record.get("statistic")
        require(
            isinstance(statistic, str) and statistic not in records,
            f"{label} comparison repeats or malforms statistic {statistic!r}",
        )
        records[statistic] = record
    require(
        set(records) == set(expected) and len(records) == len(expected),
        f"{label} comparison statistic set drift: {sorted(records)}",
    )
    return records


def validate_application_aggregate_coverage(
    configuration: dict[str, Any], report: dict[str, Any]
) -> dict[str, Any]:
    """Bind the canonical application series and hard-gate packet-size parity."""

    require(
        configuration.get("statistics")
        == list(CANONICAL_APPLICATION_AGGREGATE_STATISTICS),
        "canonical application aggregate statistic set/order drift",
    )
    records = _comparison_series_by_statistic(
        report,
        CANONICAL_APPLICATION_AGGREGATE_STATISTICS,
        "application aggregate",
    )
    packet_size = records[APPLICATION_PACKET_SIZE_STATISTIC]
    expected = {
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
    require(
        all(packet_size.get(name) == value for name, value in expected.items()),
        f"application packet-size parity gate failed: {packet_size}",
    )
    return dict(packet_size)


def validate_protocol_aggregate_coverage(
    report: dict[str, Any], comparison_exit_code: int
) -> dict[str, Any]:
    """Require exact steady-window coverage for the OPNET-backed queue series."""

    require(
        report.get("schema") == "csr-aggregate-differential-report-v1"
        and isinstance(report.get("pass"), bool)
        and comparison_exit_code in {0, 1}
        and report["pass"] == (comparison_exit_code == 0),
        "protocol aggregate comparison schema/status/exit drift",
    )
    filters = report.get("filters") or {}
    tolerances = report.get("tolerances") or {}
    require(
        filters.get("scenarios") == ["blue_radio_campus-multihop"]
        and filters.get("statistics") == list(PROTOCOL_AGGREGATE_STATISTICS)
        and filters.get("time_start_s") == PROTOCOL_AGGREGATE_START_S
        and filters.get("time_stop_s") == PROTOCOL_AGGREGATE_STOP_S
        and tolerances == {"absolute": 0.0, "relative": 0.0, "time_s": 0.0},
        "protocol aggregate filters or zero-tolerance contract drift",
    )
    counts = report.get("counts") or {}
    expected_counts = {
        "base_series": 4,
        "comparable_series": 4,
        "matched_points": 380,
        "numeric_points_compared": 380,
        "missing_in_ns3": 0,
        "extra_in_ns3": 0,
        "skipped_missing_values": 0,
        "skipped_noncomparable_points": 0,
        "unmatched_opnet_series": 0,
        "unmatched_ns3_series": 0,
        "noncomparable_series": 0,
        "missing_ns3_values": 0,
        "indeterminate_series": 0,
    }
    require(
        all(counts.get(name) == value for name, value in expected_counts.items()),
        f"protocol aggregate structural coverage drift: {counts}",
    )
    numeric_mismatches = counts.get("numeric_mismatches")
    require(
        isinstance(numeric_mismatches, int)
        and not isinstance(numeric_mismatches, bool)
        and 0 <= numeric_mismatches <= counts["numeric_points_compared"]
        and (numeric_mismatches == 0) == (comparison_exit_code == 0),
        "protocol aggregate numeric mismatch count disagrees with exit status",
    )
    require(
        report.get("opnet_point_count") == 380
        and report.get("ns3_point_count") == 380
        and filters.get("selected_point_count") == 760
        and report.get("total_difference_count") == numeric_mismatches
        and report.get("reported_difference_count") == numeric_mismatches
        and report.get("differences_truncated") is False
        and report.get("total_skipped_count") == 0
        and report.get("reported_skipped_count") == 0
        and report.get("skipped_truncated") is False,
        "protocol aggregate report accounting drift",
    )
    records = _comparison_series_by_statistic(
        report, PROTOCOL_AGGREGATE_STATISTICS, "protocol aggregate"
    )
    for statistic, record in records.items():
        unit, aggregation = PROTOCOL_AGGREGATE_SEMANTICS[statistic]
        expected = {
            "scenario": "blue_radio_campus-multihop",
            "status": "compared",
            "unit": unit,
            "aggregation": aggregation,
            "opnet_points": PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES,
            "ns3_points": PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES,
            "matched_points": PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES,
            "authoritative_numeric_points": PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES,
            "numeric_points_compared": PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES,
            "missing_in_ns3": 0,
            "extra_in_ns3": 0,
            "missing_ns3_values": 0,
            "skipped_missing_values": 0,
        }
        require(
            all(record.get(name) == value for name, value in expected.items()),
            f"protocol aggregate series coverage drift for {statistic}: {record}",
        )
        series_mismatches = record.get("numeric_mismatches")
        require(
            isinstance(series_mismatches, int)
            and not isinstance(series_mismatches, bool)
            and 0 <= series_mismatches <= PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES
            and record.get("pass") == (series_mismatches == 0),
            f"protocol aggregate series result drift for {statistic}: {record}",
        )
    require(
        sum(record["numeric_mismatches"] for record in records.values())
        == numeric_mismatches,
        "protocol aggregate per-series mismatch total drift",
    )
    return {
        "counts": dict(counts),
        "series": [dict(records[name]) for name in PROTOCOL_AGGREGATE_STATISTICS],
    }


def release_status_from_aggregate_codes(exit_codes: Sequence[int]) -> str:
    """Classify the checkpoint from every residual-permitting aggregate gate."""

    codes = tuple(exit_codes)
    require(
        codes and all(code in {0, 1} for code in codes),
        f"aggregate comparator exit-code ledger is malformed: {codes}",
    )
    return (
        "passed_with_scoped_exemptions"
        if all(code == 0 for code in codes)
        else "passed_with_scoped_exemptions_and_observed_aggregate_residual"
    )


def classify_duplicate_lineage_summary(duplicate: Any) -> str:
    """Accept strict zero-duplicate evidence and the bounded legacy ledger."""

    require(isinstance(duplicate, dict), "malformed DACK duplicate-proof summary")
    proof_count = duplicate.get("repeated_feedback_proof_count")
    matched_count = duplicate.get("matched_duplicate_delivery_count")
    unused_count = duplicate.get("unused_proof_count")
    lineages = duplicate.get("lineages")
    require(
        isinstance(proof_count, int)
        and not isinstance(proof_count, bool)
        and proof_count >= 0
        and isinstance(matched_count, int)
        and not isinstance(matched_count, bool)
        and matched_count >= 0
        and isinstance(unused_count, int)
        and not isinstance(unused_count, bool)
        and unused_count >= 0
        and proof_count == matched_count + unused_count
        and isinstance(lineages, list),
        "malformed DACK duplicate-proof summary",
    )
    if proof_count == 0:
        require(
            matched_count == 0 and unused_count == 0 and lineages == [],
            "zero-duplicate proof summary contains historical lineage records",
        )
        return "strict_zero_duplicate_lineage"
    require(lineages, "historical duplicate-proof summary omits lineage records")
    return "historical_source_proved_duplicate_compatibility"


class Certification:
    def __init__(self, arguments: argparse.Namespace):
        self.args = arguments
        self.output = arguments.output_dir.resolve()
        self.artifacts = self.output / "artifacts"
        self.manifests = self.output / "manifests"
        self.work = (
            arguments.work_dir.resolve()
            if arguments.work_dir is not None
            else Path(str(self.output) + ".work").resolve()
        )
        self.commands: list[dict[str, Any]] = []
        self.release: dict[str, Any] = {
            "schema": "csr-release-certification-v2",
            "status": "initializing",
            "started_utc": utc_now(),
            "completed_utc": None,
            "failure": None,
            "repositories": {},
            "environment": {},
            "configuration": {},
            "inputs": {},
            "build": {},
            "executables": {},
            "checks": {},
            "scope": {
                "objective": "NS-3 behavioral parity with the supplied OPNET project",
                "opnet_direct_source_file_count": len(OPNET_SOURCE_INPUTS),
                "in_scope_direct_source_file_count": (
                    len(OPNET_SOURCE_INPUTS) - len(OPNET_SCOPE_EXEMPTIONS)
                ),
                "supplemental_source_artifact_count": len(
                    SUPPLEMENTAL_SOURCE_INPUTS
                ),
                "exemptions": [
                    {
                        "source_file": name,
                        "source_sha256": OPNET_SOURCE_INPUTS[name],
                        **record,
                    }
                    for name, record in sorted(OPNET_SCOPE_EXEMPTIONS.items())
                ],
                "claim_boundary": (
                    "Battery/energy and supervisory-layer behavior are excluded. "
                    "Their supplied sources and the archived adb97 executable that "
                    "contains them remain hash-bound evidence inputs, but this "
                    "release makes no implementation or parity claim for them. The "
                    "legacy zero-DSCP scenario leaves the supervisor hook inert; "
                    "fixed ns-3 energy=100 is not a battery-depletion model. "
                    "The canonical routing configuration is __ARL_ROUTING__. "
                    "Alternative BBN routing remains an unimplemented parity "
                    "gap/non-claim, not a project-authorized exemption. "
                    "Five additional supplied source artifacts are hash-bound for "
                    "provenance, but binding an archive does not place every archive "
                    "member within the release parity scope."
                ),
            },
            "commands": self.commands,
            "evidence_boundaries": [
                "The recovered PB/OV data are historical bucket aggregates, not an OPNET packet-event trace.",
                "The synthetic six-event differential validates harness plumbing only.",
                "All-values pairs are per-statistic writes without packet identifiers and are not cross-correlated.",
                "Packet-path schema v2 is accepted only on a strict zero-invalid lifecycle pass with queue-delay-constrained unique branch matching, zero ambiguous branch keys, and exact repeated-DACK accounting. Legacy schema v1 remains supported only through its narrower trace-bound diagnostic classification.",
                "Battery/energy behavior is an explicit scoped exemption; no parity claim is made.",
                "The supervisory layer is an explicit scoped exemption; no parity claim is made.",
                "The canonical routing configuration is __ARL_ROUTING__; alternative BBN routing is an unimplemented parity gap/non-claim, not an exemption.",
                "The hash-bound adb97 OPNET executable contains battery and supervisor code; excluded means present-but-out-of-scope, never absent.",
                "The zero-DSCP canonical application leaves the supervisor hook inert, and ns-3 fixed energy=100 must not be generalized to battery depletion or metrics.",
                "The five supplemental user source artifacts are hash-bound provenance references; archive membership alone does not establish implementation coverage or parity scope.",
                "The ns-3 wrapper, top-level Python utilities, and nested Python differential stages use the pinned interpreter with -B -I under the minimal no-bytecode environment.",
            ],
            "workspace": {
                "path": str(self.work),
                "part_of_frozen_evidence": False,
                "note": "Ephemeral source/build staging is outside the deliverable; immutable source archives, binaries, loaded libraries, configuration, logs, and results are copied into the hashed evidence set.",
            },
        }
        self.runtime_env: dict[str, str] = {}
        self.rc_stage = self.work / "rc-source"
        self.ns3_stage = self.work / "ns3-source"
        self.build_dir = self.work / "ns3-build"
        self.tool_bin = self.work / "pinned-tools-bin"
        self.binary_paths: dict[str, Path] = {}
        self.binary_hashes_before: dict[str, str] = {}
        self.library_hashes_before: dict[str, str] = {}
        self.source_bindings: list[dict[str, str]] = []
        self.python = Path(sys.executable).resolve()
        self.tool_hashes_before: dict[str, str] = {
            GIT: sha256(Path(GIT)),
            str(self.python): sha256(self.python),
        }
        # Repository admission happens before the general build environment is
        # prepared. Give Git a minimal deterministic environment so loader,
        # Python, shell-startup, Git, and toolchain variables from the host
        # cannot inject unrecorded behavior into clean/status/archive checks.
        self.git_env = {
            "PATH": "/usr/bin:/bin",
            "LC_ALL": "C",
            "LANG": "C",
            "TZ": "UTC",
            "HOME": "/nonexistent",
            "XDG_CONFIG_HOME": "/nonexistent",
            "GIT_CONFIG_NOSYSTEM": "1",
            "GIT_CONFIG_GLOBAL": "/dev/null",
            "GIT_CONFIG_SYSTEM": "/dev/null",
            "GIT_TERMINAL_PROMPT": "0",
            "GIT_OPTIONAL_LOCKS": "0",
        }
        self.tool_bindings: list[dict[str, str]] = []
        self.external_originals: dict[str, Path] = {}
        self.external_staged: dict[str, Path] = {}
        self.external_before: dict[str, dict[str, Any]] = {}
        self.rc_content_snapshot: list[dict[str, Any]] = []
        self.ns3_content_snapshot: list[dict[str, Any]] = []
        # Keep the lexical entry path so preflight can reject a symlinked or
        # out-of-tree launcher before comparing it with the committed blob.
        self.harness_original = Path(__file__).absolute()
        self.harness_sha256_before = sha256(self.harness_original)
        self.classifier_probe_original = self.harness_original.with_name(
            "test_certify_release_classifier.py"
        )
        require(
            self.classifier_probe_original.is_file(),
            "classifier self-test is absent beside certification harness",
        )
        self.classifier_probe_sha256_before = sha256(
            self.classifier_probe_original
        )
        self.harness_rc_binding_before: dict[str, Any] = {}

    def checkpoint(self) -> None:
        atomic_json(self.output / "release-manifest.json", self.release)

    def stage(self, name: str) -> None:
        print(f"[{utc_now()}] {name}", flush=True)
        self.release["status"] = "running"
        self.release["current_stage"] = name
        self.checkpoint()

    def run_command(
        self,
        arguments: Sequence[os.PathLike[str] | str],
        *,
        cwd: Path,
        log: Path,
        allowed: Iterable[int] = (0,),
        timeout: int | None = None,
        env: dict[str, str] | None = None,
    ) -> int:
        argv = [str(value) for value in arguments]
        executable = Path(argv[0]).resolve()
        expected_executable_digest = self.tool_hashes_before.get(str(executable))
        if expected_executable_digest is None:
            for target, path in self.binary_paths.items():
                if executable == path.resolve():
                    expected_executable_digest = self.binary_hashes_before[target]
                    break
        require(
            expected_executable_digest is not None,
            f"attempted to run an executable without a recorded hash: {executable}",
        )
        require(
            executable.is_file()
            and sha256(executable) == expected_executable_digest,
            f"bound executable changed before use: {executable}",
        )
        bound_script: Path | None = None
        bound_script_digest: str | None = None
        if executable == self.python and len(argv) > 3:
            candidate = Path(argv[3])
            if candidate.is_file():
                bound_script = candidate.absolute()
                require(
                    not candidate.is_symlink(),
                    f"Python entry script is symlinked: {candidate}",
                )
                bound_script_digest = sha256(bound_script)
        log.parent.mkdir(parents=True, exist_ok=True)
        record: dict[str, Any] = {
            "argv": argv,
            "display": command_text(argv),
            "cwd": str(cwd.resolve()),
            "log": relative(log, self.output),
            "started_utc": utc_now(),
            "timeout_seconds": timeout,
            "exit_code": None,
            "duration_seconds": None,
            "resolved_executable": str(executable),
            "resolved_executable_sha256": expected_executable_digest,
            "python_entry_script": (
                {
                    "path": str(bound_script),
                    "sha256": bound_script_digest,
                }
                if bound_script is not None
                else None
            ),
        }
        self.commands.append(record)
        self.checkpoint()
        started = time.monotonic()
        process: subprocess.Popen[bytes] | None = None
        try:
            with log.open("wb") as stream:
                process = subprocess.Popen(
                    argv,
                    cwd=cwd,
                    env=env,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            code = 124
            record["timeout"] = True
            record["timeout_detail"] = str(error)
            require(process is not None, "timed out before child process creation")
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
            # The group leader can exit on SIGTERM while a descendant remains.
            # Always follow the grace period with a group-wide SIGKILL; a
            # missing group is the normal clean case.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=10)
            cleanup_deadline = time.monotonic() + 10
            while True:
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    break
                require(
                    time.monotonic() < cleanup_deadline,
                    f"timed-out process group {process.pid} did not terminate",
                )
                time.sleep(0.05)
        record["exit_code"] = code
        record["completed_utc"] = utc_now()
        record["duration_seconds"] = round(time.monotonic() - started, 6)
        self.checkpoint()
        require(
            executable.is_file()
            and sha256(executable) == expected_executable_digest,
            f"bound executable changed during use: {executable}",
        )
        if bound_script is not None:
            require(
                bound_script.is_file()
                and not bound_script.is_symlink()
                and sha256(bound_script) == bound_script_digest,
                f"Python entry script changed during use: {bound_script}",
            )
        permitted = tuple(allowed)
        if code not in permitted:
            raise CertificationError(
                f"command exited {code}, expected {permitted}: {command_text(argv)}; "
                f"see {log}"
            )
        return code

    def capture(
        self, arguments: Sequence[os.PathLike[str] | str], cwd: Path
    ) -> str:
        require(
            sha256(Path(GIT)) == self.tool_hashes_before[GIT],
            "Git executable changed before repository query",
        )
        completed = subprocess.run(
            [str(value) for value in arguments],
            cwd=cwd,
            env=self.git_env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if completed.returncode != 0:
            raise CertificationError(
                f"command failed: {command_text(arguments)}\n{completed.stderr}"
            )
        require(
            sha256(Path(GIT)) == self.tool_hashes_before[GIT],
            "Git executable changed during repository query",
        )
        return completed.stdout.strip()

    def git_clean(self, repository: Path) -> str:
        status = self.capture(
            [GIT, "status", "--porcelain=v1", "--untracked-files=all"],
            repository,
        )
        require(not status, f"repository is not completely clean: {repository}\n{status}")
        ignored = self.capture(
            [GIT, "ls-files", "--others", "--ignored", "--exclude-standard", "-z"],
            repository,
        )
        require(
            not ignored,
            f"repository contains ignored worktree files: {repository}\n"
            + ignored.replace("\0", "\n"),
        )
        return status

    def repository_identity(
        self,
        repository: Path,
        expected_commit: str,
        *,
        require_origin_main: bool,
    ) -> dict[str, Any]:
        require(repository.is_dir(), f"repository directory is absent: {repository}")
        require(
            re.fullmatch(r"[0-9a-f]{40}", expected_commit) is not None,
            "--rc-commit/expected commit must be a full 40-character Git object ID",
        )
        self.git_clean(repository)
        head = self.capture([GIT, "rev-parse", "HEAD"], repository)
        resolved = self.capture(
            [GIT, "rev-parse", f"{expected_commit}^{{commit}}"], repository
        )
        require(head == expected_commit == resolved, f"HEAD is not {expected_commit}")
        origin_main = None
        if require_origin_main:
            origin_main = self.capture([GIT, "rev-parse", "origin/main"], repository)
            require(
                origin_main == expected_commit,
                f"origin/main {origin_main} does not identify tested commit {expected_commit}",
            )
        tree = self.capture([GIT, "rev-parse", f"{expected_commit}^{{tree}}"], repository)
        timestamp = self.capture(
            [GIT, "show", "-s", "--format=%ct", expected_commit], repository
        )
        remote = self.capture([GIT, "remote", "get-url", "origin"], repository)
        branch = self.capture(
            [GIT, "symbolic-ref", "--quiet", "--short", "HEAD"], repository
        ) if subprocess.run(
            [GIT, "symbolic-ref", "--quiet", "HEAD"],
            cwd=repository,
            env=self.git_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode == 0 else None
        return {
            "path": str(repository.resolve()),
            "head": head,
            "tree": tree,
            "origin": safe_remote(remote),
            "origin_main": origin_main,
            "branch": branch,
            "commit_timestamp": int(timestamp),
            "clean_porcelain": True,
        }

    def validate_release_harness_binding(
        self,
        repository: Path,
        commit: str,
        *,
        harness_path: Path | None = None,
        classifier_path: Path | None = None,
    ) -> dict[str, dict[str, Any]]:
        """Bind the executing harness and its probe to their exact RC blobs."""

        repository = repository.resolve()
        harness = (
            self.harness_original if harness_path is None else harness_path.absolute()
        )
        classifier = (
            self.classifier_probe_original
            if classifier_path is None
            else classifier_path.absolute()
        )
        require(
            harness.parent == classifier.parent,
            "classifier self-test is not beside the executing certification harness",
        )
        candidates = {
            "certification_harness": harness,
            "classifier_self_test": classifier,
        }
        bindings: dict[str, dict[str, Any]] = {}
        for role, lexical_path in candidates.items():
            require(
                lexical_path.is_file() and not lexical_path.is_symlink(),
                f"{role} is absent, non-regular, or symlinked: {lexical_path}",
            )
            resolved_path = lexical_path.resolve()
            repository_path = RELEASE_HARNESS_REPOSITORY_PATHS[role]
            expected_path = (repository / repository_path).resolve()
            require(
                is_within(resolved_path, repository)
                and resolved_path == expected_path,
                f"{role} does not resolve to {repository_path} inside --rc-repo",
            )
            committed_blob = self.capture(
                [GIT, "rev-parse", f"{commit}:{repository_path}"], repository
            )
            working_blob = self.capture(
                [GIT, "hash-object", "--no-filters", "--", resolved_path],
                repository,
            )
            require(
                re.fullmatch(r"[0-9a-f]{40,64}", committed_blob) is not None
                and working_blob == committed_blob,
                f"{role} content does not match the exact blob at --rc-commit",
            )
            bindings[role] = {
                "resolved_path": str(resolved_path),
                "repository_path": repository_path,
                "sha256": sha256(resolved_path),
                "git_blob_oid": committed_blob,
                "matches_rc_commit_blob": True,
            }
        return bindings

    @staticmethod
    def content_tree(root: Path) -> list[dict[str, Any]]:
        records: list[dict[str, Any]] = []
        for path in sorted(root.rglob("*"), key=lambda item: item.as_posix()):
            name = path.relative_to(root).as_posix()
            if path.is_symlink():
                records.append({"path": name, "type": "symlink", "target": os.readlink(path)})
            elif path.is_file():
                records.append(
                    {
                        "path": name,
                        "type": "file",
                        "sha256": sha256(path),
                        "size_bytes": path.stat().st_size,
                        "executable": bool(path.stat().st_mode & 0o111),
                    }
                )
        return records

    @staticmethod
    def make_read_only(root: Path) -> None:
        paths = sorted(root.rglob("*"), key=lambda item: len(item.parts), reverse=True)
        for path in paths:
            if path.is_symlink():
                continue
            if path.is_file():
                mode = 0o555 if path.stat().st_mode & 0o111 else 0o444
                path.chmod(mode)
            elif path.is_dir():
                path.chmod(0o555)
        root.chmod(0o555)

    def archive_repository(
        self,
        repository: Path,
        commit: str,
        archive: Path,
        destination: Path,
        label: str,
    ) -> None:
        archive.parent.mkdir(parents=True, exist_ok=True)
        destination.mkdir(parents=True, exist_ok=False)
        self.run_command(
            [GIT, "archive", "--format=tar", f"--output={archive}", commit],
            cwd=repository,
            log=self.artifacts / "logs" / f"archive-{label}.log",
            timeout=300,
            env=self.git_env,
        )
        with tarfile.open(archive, mode="r:") as stream:
            stream.extractall(destination, filter="data")

    def prepare_inputs(self) -> None:
        self.stage("preflight repositories and immutable inputs")
        require(
            sha256(Path(GIT)) == self.tool_hashes_before[GIT],
            "Git executable changed before repository preflight",
        )
        rc_repo = self.args.rc_repo.resolve()
        ns3_repo = self.args.ns3_repo.resolve()
        rc_identity = self.repository_identity(
            rc_repo, self.args.rc_commit, require_origin_main=True
        )
        ns3_identity = self.repository_identity(
            ns3_repo, NS3_COMMIT, require_origin_main=False
        )
        self.release["repositories"] = {"rc": rc_identity, "ns3": ns3_identity}
        self.harness_rc_binding_before = self.validate_release_harness_binding(
            rc_repo, self.args.rc_commit
        )
        self.release["certification_harness"]["rc_commit_binding"] = (
            self.harness_rc_binding_before
        )

        rc_ls_tree = self.capture(
            [GIT, "ls-tree", "-r", "--full-tree", self.args.rc_commit], rc_repo
        )
        (self.manifests / "rc-git-ls-tree.txt").parent.mkdir(parents=True, exist_ok=True)
        (self.manifests / "rc-git-ls-tree.txt").write_text(
            rc_ls_tree + "\n", encoding="utf-8"
        )

        self.archive_repository(
            rc_repo,
            self.args.rc_commit,
            self.artifacts / "source" / "rc-source.tar",
            self.rc_stage,
            "rc",
        )
        self.archive_repository(
            ns3_repo,
            NS3_COMMIT,
            self.artifacts / "source" / "ns3-source.tar",
            self.ns3_stage,
            "ns3",
        )
        self.release["repositories"]["rc"]["source_archive_sha256"] = sha256(
            self.artifacts / "source" / "rc-source.tar"
        )
        self.release["repositories"]["ns3"]["source_archive_sha256"] = sha256(
            self.artifacts / "source" / "ns3-source.tar"
        )
        require(
            sha256(Path(GIT)) == self.tool_hashes_before[GIT],
            "Git executable changed during repository preflight/archive",
        )

        # Preserve the lexical input paths until after lstat-style symlink
        # admission.  Calling resolve() first would silently turn a supplied
        # symlink into its regular-file target and defeat the fail-closed check.
        source_archive = self.args.source_archive.absolute()
        evidence_root = self.args.evidence_root.absolute()
        opnet_source_dir = self.args.opnet_source_dir.absolute()
        supplied_source_dir = self.args.supplied_source_dir.absolute()
        require(
            evidence_root.is_dir() and not evidence_root.is_symlink(),
            f"evidence root is absent or symlinked: {evidence_root}",
        )
        originals: dict[str, Path] = {"source_archive": source_archive}
        for label, (relative_name, _expected) in EXTERNAL_INPUTS.items():
            if label == "source_archive":
                continue
            originals[label] = evidence_root / relative_name

        supplied_sources = validate_named_file_set(
            supplied_source_dir,
            SUPPLIED_SOURCE_DIRECTORY_INPUTS,
            description="supplied source artifact",
            include_hidden=True,
        )
        require(
            supplied_sources[PRIMARY_SOURCE_ARCHIVE_NAME]["sha256"]
            == EXTERNAL_INPUTS["source_archive"][1],
            "supplied-source directory archive is not the canonical --source-archive identity",
        )

        input_dir = self.artifacts / "inputs"
        input_dir.mkdir(parents=True, exist_ok=True)
        for label, original in originals.items():
            expected = EXTERNAL_INPUTS[label][1]
            require(
                original.is_file() and not original.is_symlink(),
                f"required input is absent or not a regular non-symlink file: {original}",
            )
            actual = sha256(original)
            require(actual == expected, f"input hash mismatch for {label}: {actual}")
            # Preserve evidence basenames.  Both the aggregate workflow and
            # the OV extractor derive/corroborate scenario identity from the
            # Modeler filename; label-based renaming would invalidate that
            # fail-closed check.
            staged = input_dir / label / original.name
            staged.parent.mkdir(parents=True, exist_ok=False)
            shutil.copy2(original, staged)
            require(sha256(staged) == actual, f"staged copy mismatch for {label}")
            self.external_originals[label] = original
            self.external_staged[label] = staged
            self.external_before[label] = {
                "input_kind": "archived_or_scenario_evidence",
                "original_path": str(original),
                "expected_sha256": expected,
                "before_sha256": actual,
                "size_bytes": original.stat().st_size,
                "staged_path": relative(staged, self.output),
                "staged_sha256": sha256(staged),
            }

        require(
            self.external_before["source_archive"]["before_sha256"]
            == supplied_sources[PRIMARY_SOURCE_ARCHIVE_NAME]["sha256"],
            "--source-archive and supplied-source directory archive identities differ",
        )

        supplemental_input_dir = input_dir / "supplemental-user-sources"
        supplemental_input_dir.mkdir(parents=True, exist_ok=False)
        supplemental_records: list[dict[str, Any]] = []
        for name in sorted(SUPPLEMENTAL_SOURCE_INPUTS):
            source_record = supplied_sources[name]
            original = source_record["path"]
            staged = supplemental_input_dir / name
            shutil.copy2(original, staged)
            require(
                sha256(staged) == source_record["sha256"],
                f"staged supplemental source changed: {name}",
            )
            staged.chmod(0o444)
            label = f"supplemental_source/{name}"
            record = {
                "input_kind": "supplemental_user_source_artifact",
                "original_path": str(original),
                "expected_sha256": SUPPLEMENTAL_SOURCE_INPUTS[name],
                "before_sha256": source_record["sha256"],
                "size_bytes": source_record["size_bytes"],
                "staged_path": relative(staged, self.output),
                "staged_sha256": sha256(staged),
                "scope_disposition": SUPPLEMENTAL_SCOPE_DISPOSITION,
                "archive_members_wholesale_in_parity_scope": False,
            }
            self.external_originals[label] = original
            self.external_staged[label] = staged
            self.external_before[label] = record
            supplemental_records.append(
                {
                    "source_artifact": name,
                    "sha256": source_record["sha256"],
                    "size_bytes": source_record["size_bytes"],
                    "staged_path": relative(staged, self.output),
                    "scope_disposition": SUPPLEMENTAL_SCOPE_DISPOSITION,
                    "archive_members_wholesale_in_parity_scope": False,
                }
            )
        supplemental_input_dir.chmod(0o555)
        require(
            len(supplemental_records) == len(SUPPLEMENTAL_SOURCE_INPUTS),
            "supplemental source provenance ledger is incomplete",
        )

        direct_sources = validate_named_file_set(opnet_source_dir, OPNET_SOURCE_INPUTS)
        source_input_dir = input_dir / "opnet-direct-sources"
        source_input_dir.mkdir(parents=True, exist_ok=False)
        scope_records: list[dict[str, Any]] = []
        for name, source_record in direct_sources.items():
            label = f"opnet_direct_source/{name}"
            original = source_record["path"]
            staged = source_input_dir / name
            shutil.copy2(original, staged)
            require(
                sha256(staged) == source_record["sha256"],
                f"staged direct OPNET source changed: {name}",
            )
            scope = OPNET_SCOPE_EXEMPTIONS.get(name)
            disposition = (
                scope["disposition"] if scope is not None else "in_scope_reference"
            )
            record = {
                "input_kind": "direct_opnet_source",
                "original_path": str(original),
                "expected_sha256": OPNET_SOURCE_INPUTS[name],
                "before_sha256": source_record["sha256"],
                "size_bytes": source_record["size_bytes"],
                "staged_path": relative(staged, self.output),
                "staged_sha256": sha256(staged),
                "scope_disposition": disposition,
            }
            self.external_originals[label] = original
            self.external_staged[label] = staged
            self.external_before[label] = record
            scope_records.append(
                {
                    "source_file": name,
                    "sha256": source_record["sha256"],
                    "size_bytes": source_record["size_bytes"],
                    "staged_path": relative(staged, self.output),
                    "scope_disposition": disposition,
                    **(
                        {"exempt_component": scope["component"]}
                        if scope is not None
                        else {}
                    ),
                }
            )
        require(
            len(scope_records) == 16
            and sum(
                record["scope_disposition"]
                == "explicitly_exempt_from_this_release"
                for record in scope_records
            )
            == 2,
            "direct OPNET source scope ledger is incomplete",
        )
        self.release["scope"]["opnet_direct_sources"] = scope_records
        source_scope_manifest = self.manifests / "opnet-direct-source-scope.json"
        atomic_json(
            source_scope_manifest,
            {
                "schema": "csr-opnet-direct-source-scope-v1",
                "objective": self.release["scope"]["objective"],
                "claim_boundary": self.release["scope"]["claim_boundary"],
                "source_file_count": len(scope_records),
                "in_scope_source_file_count": len(scope_records)
                - len(OPNET_SCOPE_EXEMPTIONS),
                "exempt_source_file_count": len(OPNET_SCOPE_EXEMPTIONS),
                "sources": scope_records,
                "exemptions": self.release["scope"]["exemptions"],
            },
        )
        self.release["scope"]["manifest"] = relative(
            source_scope_manifest, self.output
        )
        supplemental_manifest = (
            self.manifests / "supplemental-source-provenance.json"
        )
        atomic_json(
            supplemental_manifest,
            {
                "schema": "csr-supplemental-source-provenance-v1",
                "objective": self.release["scope"]["objective"],
                "claim_boundary": (
                    "These five artifacts are immutable provenance references. "
                    "Artifact or archive binding does not assert that every archive "
                    "member is implemented, tested, or within parity scope. Direct "
                    "source scope and the battery/supervisor exemptions remain "
                    "defined by the separate direct-source scope ledger. This "
                    "ledger also does not claim complete historical source "
                    "provenance."
                ),
                "required_directory_file_count": len(
                    SUPPLIED_SOURCE_DIRECTORY_INPUTS
                ),
                "supplemental_artifact_count": len(supplemental_records),
                "canonical_source_archive_cross_binding": {
                    "directory_name": PRIMARY_SOURCE_ARCHIVE_NAME,
                    "directory_path": str(
                        supplied_sources[PRIMARY_SOURCE_ARCHIVE_NAME]["path"]
                    ),
                    "directory_sha256": supplied_sources[
                        PRIMARY_SOURCE_ARCHIVE_NAME
                    ]["sha256"],
                    "cli_path": str(source_archive),
                    "cli_sha256": self.external_before["source_archive"][
                        "before_sha256"
                    ],
                    "identical_content_identity": True,
                },
                "artifacts": supplemental_records,
                "unbound_historical_source_references": [
                    {
                        "source_file": name,
                        "status": "not_separately_supplied_as_an_individual_hash_bound_source",
                    }
                    for name in UNBOUND_HISTORICAL_SOURCE_REFERENCES
                ],
                "direct_source_scope_manifest": relative(
                    source_scope_manifest, self.output
                ),
                "exemptions": self.release["scope"]["exemptions"],
            },
        )
        self.release["scope"]["supplemental_source_inputs"] = (
            supplemental_records
        )
        self.release["scope"]["supplemental_provenance_manifest"] = relative(
            supplemental_manifest, self.output
        )
        atomic_json(self.manifests / "input-hashes.before.json", self.external_before)

        self.make_read_only(self.rc_stage)
        self.rc_content_snapshot = self.content_tree(self.rc_stage)
        self.ns3_content_snapshot = self.content_tree(self.ns3_stage)
        atomic_json(self.manifests / "rc-staged-content.json", self.rc_content_snapshot)
        atomic_json(self.manifests / "ns3-staged-git-content.json", self.ns3_content_snapshot)
        self.release["inputs"] = self.external_before

    def prepare_environment(self) -> None:
        self.stage("record and sanitize build environment")
        cmake = (self.args.cmake_bin_dir / "cmake").resolve()
        require(cmake.is_file() and os.access(cmake, os.X_OK), f"cmake is unavailable: {cmake}")
        compiler_c = Path("/usr/bin/gcc").resolve()
        compiler_cxx = Path("/usr/bin/g++").resolve()
        make = Path("/usr/bin/make").resolve()
        git = Path("/usr/bin/git").resolve()
        ldd = Path("/usr/bin/ldd").resolve()
        require(compiler_c.is_file() and compiler_cxx.is_file(), "pinned /usr/bin gcc/g++ unavailable")
        require(make.is_file(), "pinned /usr/bin/make unavailable")
        require(git.is_file() and ldd.is_file(), "pinned /usr/bin git/ldd unavailable")
        require(
            self.python.is_file() and os.access(self.python, os.X_OK),
            f"resolved Python interpreter is unavailable: {self.python}",
        )

        self.tool_bin.mkdir(parents=True, exist_ok=False)
        pinned_commands = {
            "cmake": cmake,
            "gcc": compiler_c,
            "g++": compiler_cxx,
            "make": make,
            "python": self.python,
            "python3": self.python,
        }
        self.tool_bindings = []
        for name, target in sorted(pinned_commands.items()):
            link = self.tool_bin / name
            link.symlink_to(target)
            self.tool_bindings.append(
                {
                    "command": name,
                    "link": str(link),
                    "resolved_path": str(target),
                    "sha256": sha256(target),
                }
            )

        empty_home = self.work / "empty-home"
        temporary = self.work / "tmp"
        empty_home.mkdir()
        temporary.mkdir()
        base_env = {
            "PATH": os.pathsep.join((str(self.tool_bin), "/usr/bin", "/bin")),
            "HOME": str(empty_home),
            "XDG_CONFIG_HOME": str(empty_home),
            "TMPDIR": str(temporary),
            "LC_ALL": "C",
            "LANG": "C",
            "TZ": "UTC",
            "PYTHONDONTWRITEBYTECODE": "1",
            "PYTHONNOUSERSITE": "1",
            "PYTHONHASHSEED": "0",
            "SOURCE_DATE_EPOCH": str(
                self.release["repositories"]["rc"]["commit_timestamp"]
            ),
            "CC": str(compiler_c),
            "CXX": str(compiler_cxx),
        }
        self.runtime_env = base_env
        self.validate_tool_bindings()

        tools: dict[str, dict[str, Any]] = {}
        for label, path, version_args in (
            ("cmake", cmake, [str(cmake), "--version"]),
            ("gcc", compiler_c, [compiler_c, "--version"]),
            ("g++", compiler_cxx, [compiler_cxx, "--version"]),
            ("make", make, [make, "--version"]),
            ("git", git, [git, "--version"]),
            ("ldd", ldd, [ldd, "--version"]),
            ("python", self.python, [self.python, "--version"]),
        ):
            tool_digest = sha256(path)
            prior_digest = self.tool_hashes_before.get(str(path))
            require(
                prior_digest is None or prior_digest == tool_digest,
                f"tool executable changed before environment capture: {path}",
            )
            output = subprocess.run(
                version_args,
                env=base_env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
            ).stdout.strip()
            require(
                sha256(path) == tool_digest,
                f"tool executable changed during version capture: {path}",
            )
            tools[label] = {
                "path": str(path),
                "sha256": tool_digest,
                "version": output,
            }
            self.tool_hashes_before.setdefault(str(path), tool_digest)
        environment = {
            "captured_utc": utc_now(),
            "platform": platform.platform(),
            "uname": list(platform.uname()),
            "tools": tools,
            "effective": {
                name: base_env[name]
                for name in (
                    "PATH",
                    "HOME",
                    "XDG_CONFIG_HOME",
                    "TMPDIR",
                    "LC_ALL",
                    "LANG",
                    "TZ",
                    "PYTHONDONTWRITEBYTECODE",
                    "PYTHONNOUSERSITE",
                    "PYTHONHASHSEED",
                    "SOURCE_DATE_EPOCH",
                    "CC",
                    "CXX",
                )
            },
            "host_environment_inheritance": "none",
            "discarded_host_environment_variable_names": sorted(os.environ),
            "preflight_git_environment": dict(sorted(self.git_env.items())),
            "pinned_path_bindings": self.tool_bindings,
            "executable_run_environment_overrides": {
                "LD_LIBRARY_PATH": str((self.build_dir / "lib").resolve())
            },
        }
        os_release = Path("/etc/os-release")
        if os_release.is_file():
            environment["os_release_sha256"] = sha256(os_release)
            environment["os_release"] = os_release.read_text(encoding="utf-8")
        self.release["environment"] = environment
        atomic_json(self.manifests / "environment.json", environment)

    def validate_tool_bindings(self) -> None:
        """Recheck the exact command-search surface used for build/tool runs."""

        require(self.tool_bindings, "pinned tool binding ledger is empty")
        expected_links = {Path(record["link"]) for record in self.tool_bindings}
        require(
            self.tool_bin.is_dir() and set(self.tool_bin.iterdir()) == expected_links,
            "pinned tool command set changed",
        )
        for record in self.tool_bindings:
            link = Path(record["link"])
            target = Path(record["resolved_path"])
            require(
                link.is_symlink()
                and link.resolve() == target
                and target.is_file()
                and sha256(target) == record["sha256"]
                and shutil.which(record["command"], path=self.runtime_env["PATH"])
                == str(link),
                f"pinned tool binding changed: {record['command']}",
            )

    def assemble_and_build(self) -> None:
        self.stage("assemble one-source-tree ns-3 candidate and build 38 targets")
        sources = sorted(self.rc_stage.glob("csr-*.cc"))
        require(
            all(source.is_file() and not source.is_symlink() for source in sources),
            "CSR source list contains a symlink or non-file",
        )
        targets = [source.stem for source in sources]
        smoke = sorted(target for target in targets if target.endswith("-smoke"))
        require(len(targets) == 38, f"expected 38 top-level CSR programs, found {len(targets)}")
        require(len(smoke) == 36, f"expected 36 smoke programs, found {len(smoke)}")
        require(set(targets) - set(smoke) == SPECIAL_TARGETS, "unexpected non-smoke targets")
        required_tools = (
            "utils/import-opnet-scenario.py",
            "utils/run-opnet-aggregate-differential.py",
            "utils/run-opnet-differential.py",
            "utils/extract-opnet-ov.py",
            "utils/aggregate-ns3-trace.py",
            "utils/compare-opnet-ns3-aggregates.py",
            "utils/analyze-ns3-admission-ledger.py",
            "utils/analyze-ns3-packet-paths.py",
        )
        for name in required_tools:
            tool = self.rc_stage / name
            require(
                tool.is_file() and not tool.is_symlink(),
                f"RC lacks regular non-symlink utility {name}",
            )
        ns3_wrapper = self.ns3_stage / "ns3"
        require(
            ns3_wrapper.is_file() and not ns3_wrapper.is_symlink(),
            "ns-3 wrapper is absent or symlinked",
        )

        contrib_link = self.ns3_stage / "contrib" / "csr"
        require(not contrib_link.exists() and not contrib_link.is_symlink(), "ns-3 staging already has contrib/csr")
        contrib_link.symlink_to(self.rc_stage, target_is_directory=True)
        scratch = self.ns3_stage / "scratch"
        scratch.mkdir(parents=True, exist_ok=True)
        source_bindings: list[dict[str, str]] = []
        for source in sources:
            link = scratch / source.name
            require(not link.exists() and not link.is_symlink(), f"scratch collision: {link}")
            link.symlink_to(source)
            source_bindings.append(
                {
                    "target": source.stem,
                    "scratch_link": str(link),
                    "source": str(source),
                    "source_sha256": sha256(source),
                }
            )
        self.release["configuration"] = {
            "ns3_commit": NS3_COMMIT,
            "build_profile": "debug",
            "asserts": True,
            "logs": True,
            "examples": False,
            "ns3_tests": False,
            "build_version_embedding": False,
            "monolithic_library": False,
            "python_bindings": False,
            "static_library": False,
            "warnings_as_errors": False,
            "enabled_modules": ["csr"],
            "jobs": self.args.jobs,
            "csr_program_count": 38,
            "smoke_program_count": 36,
            "source_bindings": source_bindings,
            "ns3_wrapper_invocation": {
                "path": str((self.ns3_stage / "ns3").resolve()),
                "sha256": sha256(self.ns3_stage / "ns3"),
                "interpreter": str(self.python),
                "interpreter_sha256": self.tool_hashes_before[str(self.python)],
                "interpreter_flags": ["-B", "-I"],
            },
        }
        self.source_bindings = source_bindings
        atomic_json(self.manifests / "source-bindings.json", source_bindings)
        self.validate_source_bindings()

        configure = [
            str(self.python),
            "-B",
            "-I",
            str(self.ns3_stage / "ns3"),
            "configure",
            "-d",
            "debug",
            "-G",
            "Unix Makefiles",
            "--disable-examples",
            "--disable-tests",
            "--disable-build-version",
            "--disable-monolib",
            "--disable-python-bindings",
            "--disable-static",
            "--disable-werror",
            "--enable-asserts",
            "--enable-logs",
            "--enable-modules=csr",
            f"--out={self.build_dir}",
        ]
        self.run_command(
            configure,
            cwd=self.ns3_stage,
            log=self.artifacts / "logs" / "ns3-configure.log",
            timeout=900,
            env=self.runtime_env,
        )
        self.validate_tool_bindings()
        self.validate_source_bindings()
        cache = self.ns3_stage / "cmake-cache" / "CMakeCache.txt"
        require(cache.is_file(), "ns-3 configure did not produce CMakeCache.txt")
        cache_text = cache.read_text(encoding="utf-8", errors="replace")
        cache_records = parse_cmake_cache(cache_text)
        expected_cache_records = {
            "CMAKE_BUILD_TYPE": ("STRING", "debug"),
            "NS3_ASSERT": ("BOOL", "ON"),
            "NS3_LOG": ("BOOL", "ON"),
            "NS3_ENABLED_MODULES": ("STRING", "csr"),
            "NS3_ENABLE_BUILD_VERSION": ("BOOL", "OFF"),
            "NS3_MONOLIB": ("BOOL", "OFF"),
            "NS3_PYTHON_BINDINGS": ("BOOL", "OFF"),
            "NS3_STATIC": ("BOOL", "OFF"),
            "NS3_WARNINGS_AS_ERRORS": ("BOOL", "OFF"),
            "CMAKE_GENERATOR": ("INTERNAL", "Unix Makefiles"),
        }
        for name, expected in expected_cache_records.items():
            require(
                cache_records.get(name) == expected,
                f"CMake cache {name} is {cache_records.get(name)!r}, expected {expected!r}",
            )
        for cache_name, tool_name in (
            ("CMAKE_CXX_COMPILER", "g++"),
        ):
            record = cache_records.get(cache_name)
            expected_path = Path(
                self.release["environment"]["tools"][tool_name]["path"]
            ).resolve()
            require(
                isinstance(record, tuple)
                and record[0] in {"FILEPATH", "STRING"}
                and Path(record[1]).resolve() == expected_path
                and sha256(Path(record[1]).resolve())
                == self.tool_hashes_before[str(expected_path)],
                f"CMake cache compiler identity drift for {cache_name}: {record!r}",
            )
        cmake_record = cache_records.get("CMAKE_COMMAND")
        expected_cmake = Path(
            self.release["environment"]["tools"]["cmake"]["path"]
        ).resolve()
        require(
            isinstance(cmake_record, tuple)
            and cmake_record[0] == "INTERNAL"
            and Path(cmake_record[1]).resolve() == expected_cmake
            and sha256(Path(cmake_record[1]).resolve())
            == self.tool_hashes_before[str(expected_cmake)],
            f"CMake cache command identity drift: {cmake_record!r}",
        )
        make_record = cache_records.get("CMAKE_MAKE_PROGRAM")
        expected_make = Path(
            self.release["environment"]["tools"]["make"]["path"]
        ).resolve()
        require(
            isinstance(make_record, tuple)
            and make_record[0] == "FILEPATH"
            and Path(make_record[1]).resolve() == expected_make
            and sha256(Path(make_record[1]).resolve())
            == self.tool_hashes_before[str(expected_make)],
            f"CMake cache build-program identity drift: {make_record!r}",
        )

        build = [
            str(self.python),
            "-B",
            "-I",
            str(self.ns3_stage / "ns3"),
            "build",
            "-j",
            str(self.args.jobs),
            *targets,
        ]
        self.run_command(
            build,
            cwd=self.ns3_stage,
            log=self.artifacts / "logs" / "ns3-build.log",
            timeout=self.args.build_timeout,
            env=self.runtime_env,
        )
        self.validate_tool_bindings()
        self.validate_source_bindings()

        locks = list(self.ns3_stage.glob(".lock-ns3_*_build"))
        require(len(locks) == 1, f"expected one ns-3 build lock, found {len(locks)}")
        lock = locks[0]
        require(
            lock.is_file() and not lock.is_symlink(),
            f"ns-3 build lock is not a regular file: {lock}",
        )
        lock_digest = sha256(lock)
        lock_size = lock.stat().st_size
        runnable = self.parse_runnable_programs(lock)
        require(
            lock.stat().st_size == lock_size and sha256(lock) == lock_digest,
            "ns-3 build lock changed while it was parsed",
        )
        for target in targets:
            matches = [Path(path) for path in runnable if Path(path).name.endswith(f"-{target}-debug")]
            require(len(matches) == 1, f"cannot resolve exactly one binary for {target}: {matches}")
            binary = matches[0].resolve()
            require(binary.is_file() and os.access(binary, os.X_OK), f"binary not executable: {binary}")
            require(
                is_within(binary, self.build_dir),
                f"resolved binary is outside this certification build: {binary}",
            )
            self.binary_paths[target] = binary
            self.binary_hashes_before[target] = sha256(binary)
        require(
            len(set(self.binary_paths.values())) == len(targets),
            "multiple targets resolved to the same executable",
        )

        binary_dir = self.artifacts / "binaries"
        binary_dir.mkdir(parents=True, exist_ok=True)
        executable_records: dict[str, dict[str, Any]] = {}
        for target, binary in sorted(self.binary_paths.items()):
            frozen = binary_dir / target
            shutil.copy2(binary, frozen)
            require(sha256(frozen) == self.binary_hashes_before[target], "binary copy changed")
            executable_records[target] = {
                "build_path": str(binary),
                "sha256": self.binary_hashes_before[target],
                "size_bytes": binary.stat().st_size,
                "frozen_artifact": relative(frozen, self.output),
                "source_sha256": sha256(self.rc_stage / f"{target}.cc"),
            }
        frozen_lock = self.manifests / lock.name
        shutil.copy2(lock, frozen_lock)
        require(
            lock.is_file()
            and not lock.is_symlink()
            and lock.stat().st_size == lock_size
            and sha256(lock) == lock_digest
            and frozen_lock.is_file()
            and not frozen_lock.is_symlink()
            and frozen_lock.stat().st_size == lock_size
            and sha256(frozen_lock) == lock_digest,
            "ns-3 build lock changed during frozen-copy capture",
        )
        self.release["executables"] = executable_records
        self.release["build"] = {
            "cmake_cache": relative(self.manifests / "CMakeCache.txt", self.output),
            "ns3_lock": {
                "source_path": str(lock.resolve()),
                "frozen_artifact": relative(frozen_lock, self.output),
                "sha256": lock_digest,
                "size_bytes": lock_size,
            },
            "binary_count": len(executable_records),
        }
        shutil.copy2(cache, self.manifests / "CMakeCache.txt")
        validate_ns3_lock_binding(self.release["build"]["ns3_lock"], self.output)
        self.capture_linkage()

    def validate_source_bindings(self) -> None:
        """Recheck every ns-3/CSR symlink and source hash as one exact set."""

        require(self.source_bindings, "CSR source binding ledger is empty")
        contrib_link = self.ns3_stage / "contrib" / "csr"
        require(
            contrib_link.is_symlink() and contrib_link.resolve() == self.rc_stage,
            "contrib/csr no longer resolves to the immutable staged RC tree",
        )
        expected_links: set[Path] = set()
        for record in self.source_bindings:
            link = Path(record["scratch_link"])
            source = Path(record["source"])
            expected_links.add(link)
            require(
                link.is_symlink()
                and link.resolve() == source.resolve()
                and source.is_file()
                and sha256(source) == record["source_sha256"],
                f"CSR scratch source binding changed: {record['target']}",
            )
        actual_links = set((self.ns3_stage / "scratch").glob("csr-*.cc"))
        require(
            actual_links == expected_links,
            "ns-3 scratch CSR source-link set changed",
        )

    @staticmethod
    def parse_runnable_programs(lock: Path) -> list[str]:
        tree = ast.parse(lock.read_text(encoding="utf-8"), filename=str(lock))
        for node in tree.body:
            if not isinstance(node, ast.Assign):
                continue
            if any(
                isinstance(target, ast.Name) and target.id == "ns3_runnable_programs"
                for target in node.targets
            ):
                value = ast.literal_eval(node.value)
                require(
                    isinstance(value, list) and all(isinstance(item, str) for item in value),
                    "malformed ns3_runnable_programs",
                )
                return value
        raise CertificationError("ns-3 lock lacks ns3_runnable_programs")

    def run_env(self) -> dict[str, str]:
        environment = dict(self.runtime_env)
        environment["LD_LIBRARY_PATH"] = str((self.build_dir / "lib").resolve())
        return environment

    @staticmethod
    def ldd_paths(log: Path) -> list[Path]:
        text = log.read_text(encoding="utf-8", errors="replace")
        require("not found" not in text, f"unresolved dynamic dependency in {log}")
        paths: list[Path] = []
        for line in text.splitlines():
            matched = re.search(r"=>\s+(/\S+)\s+\(", line)
            if matched is None:
                matched = re.match(r"\s*(/\S+)\s+\(", line)
            if matched is not None:
                paths.append(Path(matched.group(1)).resolve())
        return paths

    def capture_linkage(self) -> None:
        self.stage("capture per-binary ldd and loaded shared-library identities")
        ldd_dir = self.artifacts / "ldd"
        all_libraries: set[Path] = set()
        per_binary: dict[str, list[str]] = {}
        for target, binary in sorted(self.binary_paths.items()):
            log = ldd_dir / f"{target}.txt"
            self.run_command(
                [str(Path("/usr/bin/ldd").resolve()), str(binary)],
                cwd=self.ns3_stage,
                log=log,
                timeout=60,
                env=self.run_env(),
            )
            libraries = self.ldd_paths(log)
            require(libraries, f"ldd resolved no file-backed libraries for {target}")
            require(all(path.is_file() for path in libraries), f"ldd path absent for {target}")
            ns3_libraries = [path for path in libraries if path.name.startswith("libns3")]
            require(ns3_libraries, f"{target} has no dynamically loaded ns-3 libraries")
            for path in ns3_libraries:
                require(
                    is_within(path, self.build_dir / "lib"),
                    f"{target} loads an ns-3 library outside this build: {path}",
                )
                require(
                    path.name.endswith("-debug.so"),
                    f"{target} loads a non-debug ns-3 library: {path}",
                )
            per_binary[target] = [str(path) for path in libraries]
            all_libraries.update(libraries)
        runner_libraries = per_binary["csr-opnet-scenario-runner"]
        require(
            any(re.search(r"libns3.*-csr-debug\.so$", Path(path).name) for path in runner_libraries),
            "runner does not load the debug CSR shared library",
        )
        frozen_dir = self.artifacts / "shared-libraries"
        frozen_dir.mkdir(parents=True, exist_ok=True)
        library_records: list[dict[str, Any]] = []
        for path in sorted(all_libraries, key=lambda item: str(item)):
            digest = sha256(path)
            self.library_hashes_before[str(path)] = digest
            frozen = frozen_dir / f"{digest}-{path.name}"
            if not frozen.exists():
                shutil.copy2(path, frozen)
            require(sha256(frozen) == digest, f"shared-library copy mismatch: {path}")
            library_records.append(
                {
                    "runtime_path": str(path),
                    "sha256": digest,
                    "size_bytes": path.stat().st_size,
                    "frozen_artifact": relative(frozen, self.output),
                    "used_by": sorted(
                        target for target, paths in per_binary.items() if str(path) in paths
                    ),
                }
            )
        linkage = {
            "runner_ldd": relative(
                self.artifacts / "ldd" / "csr-opnet-scenario-runner.txt", self.output
            ),
            "per_binary": per_binary,
            "shared_libraries": library_records,
        }
        atomic_json(self.manifests / "dynamic-linkage.json", linkage)
        self.release["build"]["dynamic_linkage"] = relative(
            self.manifests / "dynamic-linkage.json", self.output
        )

    def import_scenario(self) -> Path:
        self.stage("import canonical 6,000-second scenario")
        scenario = self.artifacts / "scenario" / "imported-campus.csv"
        scenario.parent.mkdir(parents=True, exist_ok=True)
        self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                str(self.rc_stage / "utils" / "import-opnet-scenario.py"),
                str(self.external_staged["source_archive"]),
                str(scenario),
                "--scenario",
                "blue_radio_campus-multihop.nt.m",
                "--infer-gateway-flows",
                "--application-profile",
                "legacy-send-only-no-dscp",
                "--mac-profile",
                "hist-2014-next-tslot-modulo-probe",
                "--hop-security-profile",
                HISTORICAL_HOP_SECURITY_PROFILE,
            ],
            cwd=self.artifacts / "scenario",
            log=self.artifacts / "logs" / "scenario-import.log",
            timeout=300,
            env=self.runtime_env,
        )
        require(sha256(scenario) == EXPECTED_SCENARIO_SHA256, "canonical scenario hash drift")
        with scenario.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        runs = [row for row in rows if row.get("record") == "run"]
        require(len(runs) == 1, "canonical scenario does not contain one run row")
        run = runs[0]
        require(run.get("scenario") == "blue_radio_campus-multihop", "scenario identity drift")
        require(float(run.get("duration_s", "nan")) == 6000.0, "duration is not 6000 seconds")
        require(int(run.get("seed", "0")) == 128, "scenario seed is not 128")
        archived_executable_digest = sha256(
            self.external_staged["multihop_executable"]
        )
        require(
            run.get("application_profile") == "legacy-send-only-no-dscp"
            and run.get("mac_profile") == "hist-2014-next-tslot-modulo-probe"
            and run.get("hop_security_profile")
            == HISTORICAL_HOP_SECURITY_PROFILE
            and run.get("ack_envelope_profile") == ""
            and run.get("source_executable_sha256")
            == archived_executable_digest
            == EXTERNAL_INPUTS["multihop_executable"][1],
            "canonical executable-backed compatibility profile drift",
        )
        require(sum(row.get("record") == "node" for row in rows) == 7, "node count drift")
        require(sum(row.get("record") == "flow" for row in rows) == 6, "flow count drift")
        self.release["checks"]["canonical_scenario"] = {
            "path": relative(scenario, self.output),
            "sha256": sha256(scenario),
            "nodes": 7,
            "flows": 6,
            "duration_s": 6000,
            "seed": 128,
            "application_profile": run["application_profile"],
            "mac_profile": run["mac_profile"],
            "hop_security_profile": run["hop_security_profile"],
            "hop_security_behavior": HISTORICAL_HOP_SECURITY_BEHAVIOR,
            "deprecated_ack_envelope_profile": run["ack_envelope_profile"],
            "bound_opnet_executable_sha256": archived_executable_digest,
        }
        return scenario

    def run_baseline(self, scenario: Path) -> Path:
        self.stage("run canonical fresh 6,000-second aggregate baseline")
        output = self.artifacts / "baseline"
        require(not output.exists(), "baseline output pre-exists")
        code = self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                str(self.rc_stage / "utils" / "run-opnet-aggregate-differential.py"),
                "--scenario",
                str(scenario),
                "--runner",
                str(self.binary_paths["csr-opnet-scenario-runner"]),
                "--opnet-ov",
                str(self.external_staged["multihop_vectors"]),
                "--probe-definition",
                str(self.external_staged["multihop_probes"]),
                "--expected-opnet-ov-sha256",
                EXTERNAL_INPUTS["multihop_vectors"][1],
                "--expected-probe-definition-sha256",
                EXTERNAL_INPUTS["multihop_probes"][1],
                "--output-dir",
                str(output),
            ],
            cwd=self.artifacts,
            log=self.artifacts / "logs" / "aggregate-workflow.log",
            allowed=(0, 1),
            timeout=self.args.workflow_timeout,
            env=self.run_env(),
        )
        manifest = json_load(output / "aggregate-run-manifest.json")
        report = json_load(output / "aggregate-report.json")
        require(manifest.get("schema") == "csr-opnet-aggregate-differential-run-v1", "bad aggregate manifest schema")
        require(manifest.get("evidence_scope") == "historical_bucket_aggregates_not_event_parity", "aggregate evidence scope drift")
        require(manifest.get("profile") == "canonical", "aggregate run is not canonical")
        require(manifest.get("noncertifying_overrides") == [], "aggregate run has overrides")
        configuration = manifest.get("configuration") or {}
        identities = configuration.get("identities") or {}
        canonical_identity = identities.get("canonical_scenario") or {}
        runner_configuration = configuration.get("runner") or {}
        archived_executable_digest = EXTERNAL_INPUTS["multihop_executable"][1]
        require(
            sha256(self.external_staged["multihop_executable"])
            == archived_executable_digest,
            "archived executable evidence changed before profile validation",
        )
        validate_historical_security_provenance(
            canonical_identity,
            runner_configuration,
            archived_executable_digest,
        )
        require(
            canonical_identity.get("scenario")
            == "blue_radio_campus-multihop"
            and canonical_identity.get("duration_s") == 6000.0
            and canonical_identity.get("seed") == 128,
            "aggregate canonical scenario identity/window drift",
        )
        require(
            report.get("schema") == "csr-aggregate-differential-report-v1"
            and isinstance(report.get("pass"), bool),
            "aggregate comparison report schema/status drift",
        )
        failure = manifest.get("failure") or {}
        if code == 0:
            require(
                manifest.get("status") == "selected_comparison_passed"
                and report.get("pass") is True
                and not failure,
                "aggregate pass status/exit/report disagree",
            )
        else:
            require(
                manifest.get("status") == "selected_comparison_failed"
                and report.get("pass") is False
                and failure.get("stage") == "compare"
                and failure.get("exit_code") == 1,
                "aggregate residual status/exit/report disagree",
            )
        workflow_inputs = manifest.get("inputs") or {}
        expected_workflow_hashes = {
            "scenario": sha256(scenario),
            "runner": self.binary_hashes_before["csr-opnet-scenario-runner"],
            "opnet_ov": EXTERNAL_INPUTS["multihop_vectors"][1],
            "probe_definition": EXTERNAL_INPUTS["multihop_probes"][1],
        }
        for name, expected in expected_workflow_hashes.items():
            require(
                (workflow_inputs.get(name) or {}).get("sha256") == expected,
                f"aggregate workflow input identity drift: {name}",
            )
        stages = manifest.get("stages") or {}
        expected_codes = {
            "extract_opnet": 0,
            "run_ns3": 0,
            "aggregate_ns3": 0,
            "compare": code,
        }
        for name, expected in expected_codes.items():
            require((stages.get(name) or {}).get("exit_code") == expected, f"aggregate stage {name} exit drift")
        for name, record in (manifest.get("artifacts") or {}).items():
            require(isinstance(record, dict), f"aggregate workflow omitted artifact {name}")
            artifact = output / str(record.get("path", ""))
            require(
                artifact.is_file()
                and sha256(artifact) == record.get("sha256")
                and artifact.stat().st_size == record.get("size_bytes"),
                f"aggregate workflow artifact identity drift: {name}",
            )
        counts = report.get("counts") or {}
        structural_counts = {
            "base_series": 8,
            "comparable_series": 8,
            "matched_points": 800,
            "numeric_points_compared": 780,
            "missing_in_ns3": 0,
            "extra_in_ns3": 0,
            "skipped_missing_values": 20,
            "skipped_noncomparable_points": 0,
            "unmatched_opnet_series": 0,
            "unmatched_ns3_series": 0,
            "noncomparable_series": 0,
            "missing_ns3_values": 0,
            "indeterminate_series": 0,
        }
        for name, expected in structural_counts.items():
            require(counts.get(name) == expected, f"aggregate count {name} is {counts.get(name)}, expected {expected}")
        numeric_mismatches = counts.get("numeric_mismatches")
        require(
            isinstance(numeric_mismatches, int)
            and not isinstance(numeric_mismatches, bool)
            and 0 <= numeric_mismatches <= counts["numeric_points_compared"]
            and (numeric_mismatches == 0) == (code == 0),
            "aggregate numeric mismatch count is malformed or disagrees with exit status",
        )
        packet_size_gate = validate_application_aggregate_coverage(
            configuration, report
        )
        exact = (stages.get("aggregate_ns3") or {}).get("exact_sequence_validation") or {}
        require(exact.get("unmatched_delivery_count") == 0, "baseline has unmatched deliveries")
        require(exact.get("matched_size_mismatch_count") == 0, "baseline has matched size mismatches")
        methods = exact.get("matched_by_method") or {}
        require(
            isinstance(methods, dict)
            and set(methods) <= {
                "exact_sequence",
                "source_exact_dack_retry_duplicate",
            }
            and methods
            and all(
                isinstance(value, int)
                and not isinstance(value, bool)
                and value >= 0
                for value in methods.values()
            ),
            f"baseline exact-match method ledger is malformed: {methods}",
        )
        matched_count = exact.get("matched_count")
        duplicate_delivery_count = exact.get(
            "source_exact_dack_retry_duplicate_delivery_count"
        )
        duplicate_proof_count = exact.get(
            "source_exact_dack_retry_duplicate_proof_count"
        )
        require(
            isinstance(matched_count, int)
            and not isinstance(matched_count, bool)
            and matched_count > 0
            and sum(methods.values()) == matched_count
            and exact.get("matched_size_compared_count") == matched_count
            and isinstance(duplicate_delivery_count, int)
            and not isinstance(duplicate_delivery_count, bool)
            and duplicate_delivery_count >= 0
            and methods.get("source_exact_dack_retry_duplicate", 0)
            == duplicate_delivery_count
            and isinstance(duplicate_proof_count, int)
            and not isinstance(duplicate_proof_count, bool)
            and duplicate_proof_count >= duplicate_delivery_count,
            "baseline exact-sequence/DACK lineage totals are inconsistent",
        )
        admission = (stages.get("run_ns3") or {}).get("application_admission_totals") or {}
        admission_fields = {
            "admitted",
            "attempts",
            "blocked_destination",
            "blocked_discovery",
            "blocked_gateway_route",
            "blocked_nsdp",
            "blocked_topology",
        }
        require(
            isinstance(admission, dict)
            and set(admission) == admission_fields
            and all(
                isinstance(admission[name], int)
                and not isinstance(admission[name], bool)
                and admission[name] >= 0
                for name in admission_fields
            )
            and admission["attempts"] > 0
            and admission["admitted"]
            + sum(
                admission[name]
                for name in admission_fields
                if name.startswith("blocked_")
            )
            == admission["attempts"],
            f"application admissions do not form an exact nonnegative partition: {admission}",
        )
        self.release["checks"]["aggregate_baseline"] = {
            "classification": (
                "exact_numeric_parity"
                if code == 0
                else "observed_exact_tolerance_numeric_residual"
            ),
            "workflow_exit_code": code,
            "manifest": relative(output / "aggregate-run-manifest.json", self.output),
            "report": relative(output / "aggregate-report.json", self.output),
            "counts": counts,
            "application_packet_size_gate": packet_size_gate,
            "exact_sequence_validation": exact,
            "application_admission_totals": admission,
        }
        return output

    def run_protocol_aggregate_comparison(self, baseline: Path) -> None:
        """Gate the source-comparable steady HOP/MAC aggregate surface."""

        self.stage("compare steady HOP and MAC protocol aggregates")
        output = self.artifacts / "protocol-aggregate"
        output.mkdir(parents=True, exist_ok=False)
        staged_comparator = (
            self.rc_stage / "utils" / "compare-opnet-ns3-aggregates.py"
        )
        require(
            staged_comparator.is_file() and not staged_comparator.is_symlink(),
            "aggregate comparator is absent or symlinked",
        )
        comparator_digest = sha256(staged_comparator)
        frozen_comparator = self.artifacts / "tools" / staged_comparator.name
        require(
            not frozen_comparator.exists() and not frozen_comparator.is_symlink(),
            "frozen aggregate comparator path already exists",
        )
        shutil.copy2(staged_comparator, frozen_comparator)
        require(
            frozen_comparator.is_file()
            and not frozen_comparator.is_symlink()
            and sha256(frozen_comparator) == comparator_digest,
            "failed to freeze aggregate comparator identity",
        )

        opnet_aggregates = baseline / "opnet-aggregates.csv"
        ns3_aggregates = baseline / "ns3-aggregates.csv"
        aggregate_inputs = (opnet_aggregates, ns3_aggregates)
        require(
            all(path.is_file() and not path.is_symlink() for path in aggregate_inputs),
            "protocol aggregate comparison input is absent or symlinked",
        )
        input_digests = {path: sha256(path) for path in aggregate_inputs}
        report_path = output / "protocol-aggregate-report.json"
        normalized_path = output / "protocol-aggregate-normalized.csv"
        log_path = output / "protocol-aggregate-compare.log"
        command = [
            str(self.python),
            "-B",
            "-I",
            str(frozen_comparator),
            str(opnet_aggregates),
            str(ns3_aggregates),
            "--scenario",
            "blue_radio_campus-multihop",
            "--time-start",
            str(int(PROTOCOL_AGGREGATE_START_S)),
            "--time-stop",
            str(int(PROTOCOL_AGGREGATE_STOP_S)),
            "--abs-tolerance",
            "0",
            "--rel-tolerance",
            "0",
            "--time-tolerance",
            "0",
            "--max-differences",
            "400",
            "--report",
            str(report_path),
            "--normalized",
            str(normalized_path),
        ]
        for statistic in PROTOCOL_AGGREGATE_STATISTICS:
            command.extend(("--statistic", statistic))
        code = self.run_command(
            command,
            cwd=output,
            log=log_path,
            allowed=(0, 1),
            timeout=300,
            env=self.runtime_env,
        )
        require(
            all(sha256(path) == digest for path, digest in input_digests.items()),
            "aggregate input changed during protocol comparison",
        )
        require(
            report_path.is_file()
            and normalized_path.is_file()
            and log_path.is_file()
            and all(
                not path.is_symlink()
                for path in (report_path, normalized_path, log_path)
            ),
            "protocol aggregate comparator omitted or symlinked an artifact",
        )
        report = json_load(report_path)
        coverage = validate_protocol_aggregate_coverage(report, code)
        expected_inputs = [
            (opnet_aggregates.resolve(), input_digests[opnet_aggregates]),
            (ns3_aggregates.resolve(), input_digests[ns3_aggregates]),
        ]
        report_inputs = report.get("inputs")
        require(
            isinstance(report_inputs, list)
            and len(report_inputs) == len(expected_inputs)
            and all(
                isinstance(record, dict)
                and Path(str(record.get("path", ""))).resolve() == expected_path
                and record.get("sha256") == expected_digest
                for record, (expected_path, expected_digest) in zip(
                    report_inputs, expected_inputs
                )
            ),
            "protocol aggregate report input binding drift",
        )
        artifacts = {}
        for name, path in (
            ("report", report_path),
            ("normalized", normalized_path),
            ("log", log_path),
        ):
            artifacts[name] = {
                "path": relative(path, self.output),
                "sha256": sha256(path),
                "size_bytes": path.stat().st_size,
            }
        self.release["checks"]["protocol_aggregate"] = {
            "evidence_scope": "steady_state_queue_surface_not_full_duration_protocol_parity",
            "classification": (
                "steady_state_queue_surface_exact_numeric_parity"
                if code == 0
                else "steady_state_queue_surface_observed_exact_tolerance_numeric_residual"
            ),
            "comparison_exit_code": code,
            "window": {
                "first_bucket_end_s": PROTOCOL_AGGREGATE_START_S,
                "last_bucket_end_s": PROTOCOL_AGGREGATE_STOP_S,
                "bucket_endpoints_per_series": (
                    PROTOCOL_AGGREGATE_ENDPOINTS_PER_SERIES
                ),
                "excluded_startup_bucket_ends_s": [60, 120, 180, 240, 300],
            },
            "statistics": list(PROTOCOL_AGGREGATE_STATISTICS),
            "coverage": coverage,
            "inputs": [
                {
                    "path": relative(path, self.output),
                    "sha256": input_digests[path],
                    "size_bytes": path.stat().st_size,
                }
                for path in aggregate_inputs
            ],
            "comparator": {
                "staged_path": str(staged_comparator),
                "frozen_artifact": relative(frozen_comparator, self.output),
                "sha256": comparator_digest,
            },
            "artifacts": artifacts,
        }

    def run_program_workflows(self) -> None:
        self.stage("run 36 smoke workflows and demo in isolated directories")
        workflow_records: dict[str, Any] = {}
        smoke_targets = sorted(
            target for target in self.binary_paths if target.endswith("-smoke")
        )
        for target in smoke_targets:
            root = self.artifacts / "program-workflows" / target
            cwd = root / "cwd"
            cwd.mkdir(parents=True, exist_ok=False)
            log = root / "program.log"
            self.run_command(
                [str(self.binary_paths[target])],
                cwd=cwd,
                log=log,
                timeout=self.args.smoke_timeout,
                env=self.run_env(),
            )
            text = log.read_text(encoding="utf-8", errors="replace")
            pass_lines = [line for line in text.splitlines() if line.startswith("PASS:")]
            require(len(pass_lines) == 1, f"{target} did not emit exactly one PASS line")
            require(not any(line.startswith("FAIL:") for line in text.splitlines()), f"{target} emitted FAIL")
            workflow_records[target] = {
                "exit_code": 0,
                "classification": "smoke_pass",
                "pass_line": pass_lines[0],
                "log": relative(log, self.output),
                "log_sha256": sha256(log),
            }

        demo = "csr-mac-demo-split"
        root = self.artifacts / "program-workflows" / demo
        cwd = root / "cwd"
        (cwd / "results").mkdir(parents=True, exist_ok=False)
        log = root / "program.log"
        self.run_command(
            [str(self.binary_paths[demo])],
            cwd=cwd,
            log=log,
            timeout=self.args.demo_timeout,
            env=self.run_env(),
        )
        result_records: dict[str, Any] = {}
        for name in ("rx_metrics.csv", "nsdp_metrics.csv"):
            path = cwd / "results" / name
            require(path.is_file() and path.stat().st_size > 0, f"demo did not write {name}")
            line_count = sum(1 for _ in path.open("r", encoding="utf-8", errors="replace"))
            require(line_count > 1, f"demo {name} has no data rows")
            result_records[name] = {
                "path": relative(path, self.output),
                "sha256": sha256(path),
                "size_bytes": path.stat().st_size,
                "line_count": line_count,
            }
        workflow_records[demo] = {
            "exit_code": 0,
            "classification": "demo_outputs_valid",
            "log": relative(log, self.output),
            "log_sha256": sha256(log),
            "results": result_records,
        }
        self.release["checks"]["program_workflows"] = workflow_records

    def run_synthetic_differential(self) -> None:
        self.stage("run synthetic six-event differential and runner workflow")
        root = self.artifacts / "differential" / "synthetic"
        fixture = self.rc_stage / "utils" / "testdata" / "csr-scenario-fixture.csv"
        direct_root = self.artifacts / "program-workflows" / "csr-opnet-scenario-runner"
        direct_cwd = direct_root / "cwd"
        direct_cwd.mkdir(parents=True, exist_ok=False)
        direct_trace = direct_root / "ns3-trace.csv"
        direct_log = direct_root / "program.log"
        self.run_command(
            [
                str(self.binary_paths["csr-opnet-scenario-runner"]),
                f"--scenario={fixture}",
                f"--trace={direct_trace}",
                "--quietModelLogs=1",
            ],
            cwd=direct_cwd,
            log=direct_log,
            timeout=600,
            env=self.run_env(),
        )
        with direct_trace.open("r", encoding="utf-8", newline="") as stream:
            require(
                sum(1 for _ in csv.DictReader(stream)) == 6,
                "direct runner fixture trace is not exactly six rows",
            )
        self.release["checks"]["program_workflows"]["csr-opnet-scenario-runner"] = {
            "exit_code": 0,
            "classification": "direct_six_event_fixture_runner_pass",
            "log": relative(direct_log, self.output),
            "log_sha256": sha256(direct_log),
            "trace": relative(direct_trace, self.output),
            "trace_sha256": sha256(direct_trace),
        }
        require(
            len(self.release["checks"]["program_workflows"]) == 38,
            "program workflow ledger does not contain all 38 executables",
        )

        output = root / "run"
        root.mkdir(parents=True, exist_ok=False)
        self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                str(self.rc_stage / "utils" / "run-opnet-differential.py"),
                "--scenario",
                str(fixture),
                "--runner",
                str(self.binary_paths["csr-opnet-scenario-runner"]),
                "--opnet-trace",
                str(self.rc_stage / "utils" / "testdata" / "opnet-trace-fixture.csv"),
                "--output-dir",
                str(output),
            ],
            cwd=root,
            log=root / "workflow.log",
            timeout=600,
            env=self.run_env(),
        )
        report = json_load(output / "differential-report.json")
        manifest = json_load(output / "run-manifest.json")
        expected = {
            "matched_events": 6,
            "missing_in_ns3": 0,
            "extra_in_ns3": 0,
            "replaced_events": 0,
            "field_mismatches": 0,
            "coverage_gaps": 0,
        }
        require(report.get("pass") is True, "synthetic differential did not pass")
        require(
            report.get("schema") == "csr-differential-report-v1",
            "synthetic differential report schema drift",
        )
        for name, value in expected.items():
            require((report.get("counts") or {}).get(name) == value, f"synthetic differential {name} drift")
        require(manifest.get("ns3_exit_code") == 0 and manifest.get("comparison_exit_code") == 0, "synthetic workflow stage failed")
        trace = output / "ns3-trace.csv"
        with trace.open("r", encoding="utf-8", newline="") as stream:
            require(sum(1 for _ in csv.DictReader(stream)) == 6, "runner fixture trace is not six rows")
        self.release["checks"]["synthetic_differential"] = {
            "classification": "plumbing_fixture_only",
            "report": relative(output / "differential-report.json", self.output),
            "manifest": relative(output / "run-manifest.json", self.output),
            "counts": expected,
        }

    def run_python_tests(self) -> None:
        self.stage("discover and run the complete hash-bound Python suite")
        test_sources = sorted(
            (self.rc_stage / "utils" / "tests").rglob("test_*.py"),
            key=lambda path: path.relative_to(self.rc_stage).as_posix(),
        )
        require(test_sources, "RC Python test source set is empty")
        source_records = [
            {
                "path": path.relative_to(self.rc_stage).as_posix(),
                "sha256": sha256(path),
                "size_bytes": path.stat().st_size,
            }
            for path in test_sources
        ]
        discovery_log = self.artifacts / "python-tests" / "discovery.log"
        discovery_probe = (
            "import unittest\n"
            "loader = unittest.TestLoader()\n"
            "suite = loader.discover('utils/tests', pattern='test_*.py')\n"
            "if loader.errors:\n"
            "    raise RuntimeError(loader.errors)\n"
            "print('CSR_DISCOVERED_TESTS=' + str(suite.countTestCases()))\n"
        )
        self.run_command(
            [str(self.python), "-B", "-I", "-c", discovery_probe],
            cwd=self.rc_stage,
            log=discovery_log,
            timeout=self.args.python_timeout,
            env=self.runtime_env,
        )
        discovered_count = parse_discovered_test_count(
            discovery_log.read_text(encoding="utf-8", errors="replace")
        )
        log = self.artifacts / "python-tests" / "unittest.log"
        self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                "-m",
                "unittest",
                "discover",
                "-s",
                "utils/tests",
                "-p",
                "test_*.py",
                "-v",
            ],
            cwd=self.rc_stage,
            log=log,
            timeout=self.args.python_timeout,
            env=self.runtime_env,
        )
        text = log.read_text(encoding="utf-8", errors="replace")
        executed_count = parse_unittest_count(text)
        require(
            executed_count == discovered_count,
            f"Python execution reported {executed_count} tests after discovery found {discovered_count}",
        )
        self.release["checks"]["python_tests"] = {
            "count": executed_count,
            "discovered_count": discovered_count,
            "status": "OK",
            "test_source_count": len(source_records),
            "test_sources": source_records,
            "discovery_log": relative(discovery_log, self.output),
            "discovery_log_sha256": sha256(discovery_log),
            "log": relative(log, self.output),
            "log_sha256": sha256(log),
        }

    def run_classifier_self_tests(self) -> None:
        self.stage("run external DACK-lineage classifier self-tests")
        frozen_probe = self.artifacts / "tools" / self.classifier_probe_original.name
        log = self.artifacts / "logs" / "classifier-self-tests.log"
        self.run_command(
            [str(self.python), "-B", "-I", str(frozen_probe)],
            cwd=self.work,
            log=log,
            timeout=60,
            env=self.runtime_env,
        )
        output = log.read_text(encoding="utf-8", errors="replace")
        count = parse_unittest_count(output)
        self.release["checks"]["classifier_self_tests"] = {
            "count": count,
            "status": "OK",
            "log": relative(log, self.output),
            "log_sha256": sha256(log),
        }

    def run_all_values(self) -> None:
        self.stage("extract and structurally validate all 31,758 All-values events")
        root = self.artifacts / "all-values"
        root.mkdir(parents=True, exist_ok=False)
        output_json = root / "latency-all-values.json"
        output_csv = root / "latency-aggregates.csv"
        self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                str(self.rc_stage / "utils" / "extract-opnet-ov.py"),
                str(self.external_staged["latency_vectors"]),
                "--probe-definition",
                str(self.external_staged["latency_probes"]),
                "--json",
                str(output_json),
                "--csv",
                str(output_csv),
                "--include-all-values",
                "--strict",
            ],
            cwd=root,
            log=root / "extract.log",
            timeout=600,
            env=self.runtime_env,
        )
        manifest = json_load(output_json)
        require(manifest.get("schema") == "csr-opnet-ov-extract-v1", "all-values schema drift")
        require(manifest.get("status") == "complete", "all-values extraction is not complete")
        completeness = manifest.get("vector_completeness") or {}
        expected = {
            "vector_count": 18,
            "complete_vector_count": 18,
            "partial_vector_count": 0,
            "decoded_bucket_count": 1800,
            "validated_excluded_nonaggregate_vector_count": 6,
            "validated_nonaggregate_event_count": 31758,
            "emitted_nonaggregate_event_count": 31758,
        }
        for name, value in expected.items():
            require(completeness.get(name) == value, f"all-values {name} drift")
        records = manifest.get("excluded_nonaggregate_vectors") or []
        require(len(records) == 6, "all-values record count drift")
        lengths = [len(record.get("events") or []) for record in records]
        require(lengths == EXPECTED_ALL_VALUE_LENGTHS, f"all-values lengths drift: {lengths}")
        for record, expected_length in zip(records, EXPECTED_ALL_VALUE_LENGTHS):
            data = record.get("data_record") or {}
            require(data.get("marker") == "0x81", "all-values marker drift")
            require(data.get("event_value_count") == expected_length, "all-values metadata count drift")
            require(data.get("numeric_contents_emitted") is True, "all-values numeric contents not emitted")
            events = record.get("events") or []
            prior_time = -1.0
            for index, event in enumerate(events):
                require(event.get("record_index") == index, "all-values record index discontinuity")
                current = event.get("time_s")
                value = event.get("value")
                require(
                    isinstance(current, (int, float))
                    and math.isfinite(float(current))
                    and current >= prior_time,
                    "all-values time axis is not finite and nondecreasing",
                )
                require(
                    isinstance(value, (int, float)) and math.isfinite(float(value)),
                    "all-values event value is not finite numeric data",
                )
                prior_time = float(current)
        require(sha256(output_csv) == EXPECTED_LATENCY_AGGREGATE_SHA256, "latency aggregate CSV changed when All-values were enabled")
        with output_csv.open("r", encoding="utf-8", newline="") as stream:
            require(sum(1 for _ in csv.DictReader(stream)) == 1800, "latency aggregate row count drift")
        self.release["checks"]["all_values"] = {
            "classification": "validated_per_statistic_writes_not_packet_events",
            "event_lengths": lengths,
            "event_total": sum(lengths),
            "aggregate_rows": 1800,
            "aggregate_sha256": sha256(output_csv),
            "manifest": relative(output_json, self.output),
        }

    def run_enriched_diagnostics(self, scenario: Path, baseline: Path) -> None:
        self.stage("run separate admission-ledger-enriched 6,000-second trajectory")
        root = self.artifacts / "admission-ledger"
        root.mkdir(parents=True, exist_ok=False)
        trace = root / "ns3-admission-trace.csv"
        diagnostics = root / "app-admission-diagnostics.csv"
        self.run_command(
            [
                str(self.binary_paths["csr-opnet-scenario-runner"]),
                f"--scenario={scenario}",
                f"--trace={trace}",
                f"--appDiagnostics={diagnostics}",
                "--stop=6000",
                "--flowLimit=0",
                "--dutyCycling=1",
                "--opnetAlignedDutyCycle=1",
                "--gatewayDiscovery=1",
                "--opnetAppGating=1",
                "--aggregateTraceOnly=0",
                "--admissionTrace=1",
                "--quietModelLogs=1",
            ],
            cwd=root,
            log=root / "runner.log",
            timeout=self.args.workflow_timeout,
            env=self.run_env(),
        )
        require(trace.is_file() and diagnostics.is_file(), "enriched runner outputs are absent")

        aggregates = root / "ns3-aggregates.csv"
        provenance = root / "ns3-aggregates.provenance.json"
        self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                str(self.rc_stage / "utils" / "aggregate-ns3-trace.py"),
                str(trace),
                str(aggregates),
                "--scenario",
                "blue_radio_campus-multihop",
                "--bucket-width",
                "60",
                "--stop-time",
                "6000",
                "--legacy-trace-size-exclusion-bits",
                "0",
                "--require-zero-size-mismatches",
                "--provenance",
                str(provenance),
            ],
            cwd=root,
            log=root / "aggregate.log",
            timeout=1800,
            env=self.runtime_env,
        )
        exact_sequence_validation = self.validate_enriched_provenance(
            provenance, trace, aggregates
        )
        atomic_json(
            root / "exact-sequence-validation.json", exact_sequence_validation
        )
        enriched_rows = semantic_aggregate_rows(aggregates)
        compact_rows = semantic_aggregate_rows(baseline / "ns3-aggregates.csv")
        require(
            enriched_rows == compact_rows,
            "enriched and compact traces derive different semantic aggregate rows",
        )
        semantic_aggregate_sha256 = stable_json_sha256(enriched_rows)
        require(
            sha256(diagnostics) == sha256(baseline / "app-admission-diagnostics.csv"),
            "enriched and compact runs have different application diagnostics",
        )

        admission_summary = root / "admission-summary.json"
        admission_legs = root / "admission-legs.csv"
        admission_code = self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                str(self.rc_stage / "utils" / "analyze-ns3-admission-ledger.py"),
                str(trace),
                "--app-diagnostics",
                str(diagnostics),
                "--summary-json",
                str(admission_summary),
                "--legs-csv",
                str(admission_legs),
                "--strict",
            ],
            cwd=root,
            log=root / "admission-analysis.log",
            allowed=(0, 1),
            timeout=1800,
            env=self.runtime_env,
        )

        path_summary = root / "packet-path-summary.json"
        packet_csv = root / "packet-paths.csv"
        path_code = self.run_command(
            [
                str(self.python),
                "-B",
                "-I",
                str(self.rc_stage / "utils" / "analyze-ns3-packet-paths.py"),
                str(trace),
                "--summary-json",
                str(path_summary),
                "--packet-csv",
                str(packet_csv),
                "--startup-time",
                "300",
                "--bucket-width",
                "60",
                "--stop-time",
                "6000",
                "--tolerance",
                "1e-9",
                "--strict",
            ],
            cwd=root,
            log=root / "packet-path-analysis.log",
            allowed=(0, 1),
            timeout=1800,
            env=self.runtime_env,
        )

        classifications = self.classify_analyzer_results(
            trace,
            provenance,
            admission_summary,
            admission_code,
            path_summary,
            packet_csv,
            path_code,
        )
        atomic_json(root / "analyzer-classification.json", classifications)
        self.release["checks"]["admission_ledger"] = {
            "trace": relative(trace, self.output),
            "trace_sha256": sha256(trace),
            "diagnostics_sha256": sha256(diagnostics),
            "enriched_aggregate_file_sha256": sha256(aggregates),
            "compact_aggregate_file_sha256": sha256(baseline / "ns3-aggregates.csv"),
            "shared_semantic_aggregate_sha256": semantic_aggregate_sha256,
            "exact_sequence_validation": exact_sequence_validation,
            "classifications": classifications,
            "classification_manifest": relative(root / "analyzer-classification.json", self.output),
        }

    def validate_enriched_provenance(
        self, provenance: Path, trace: Path, aggregates: Path
    ) -> dict[str, Any]:
        """Reuse the immutable workflow's full lineage/provenance validator."""

        workflow = self.rc_stage / "utils" / "run-opnet-aggregate-differential.py"
        specification = importlib.util.spec_from_file_location(
            "csr_release_aggregate_workflow", workflow
        )
        require(
            specification is not None and specification.loader is not None,
            "cannot load aggregate workflow provenance validator",
        )
        module = importlib.util.module_from_spec(specification)
        sys.modules[specification.name] = module
        try:
            specification.loader.exec_module(module)
            result = module._load_and_validate_ns3_provenance(
                provenance,
                trace,
                aggregates,
                "blue_radio_campus-multihop",
            )
        except SystemExit as error:
            raise CertificationError(
                "enriched aggregate provenance validator attempted to terminate "
                f"the certification process (code={error.code!r})"
            ) from error
        except Exception as error:
            raise CertificationError(
                f"enriched aggregate provenance validation failed: {error}"
            ) from error
        finally:
            sys.modules.pop(specification.name, None)
        require(isinstance(result, dict), "provenance validator returned non-object")
        require(result.get("unmatched_delivery_count") == 0, "enriched trace has unmatched deliveries")
        require(result.get("matched_size_mismatch_count") == 0, "enriched trace has size mismatches")
        return result

    @staticmethod
    def parse_trace_detail(value: str, row_number: int) -> dict[str, str]:
        fields: dict[str, str] = {}
        for item in value.split(";"):
            if not item:
                continue
            require("=" in item, f"trace row {row_number} has malformed detail")
            name, field_value = item.split("=", 1)
            require(name and name not in fields, f"trace row {row_number} has duplicate detail key")
            fields[name] = field_value
        return fields

    @classmethod
    def trace_lineage_observations(
        cls,
        trace: Path,
        lineage_keys: set[PacketKey],
        proof_expectations: dict[int, dict[str, Any]],
    ) -> dict[str, Any]:
        """Bind provenance proof indexes and terminal states to the trace once."""

        required_columns = {
            "schema",
            "event_index",
            "time_s",
            "event",
            "src",
            "dst",
            "sequence",
            "reason",
            "node",
            "peer",
            "next_hop",
            "success",
            "detail",
        }
        deliveries: Counter[PacketKey] = Counter()
        app_sends: Counter[PacketKey] = Counter()
        latest: dict[PacketKey, dict[str, Any]] = {}
        seen_proof_indexes: set[int] = set()
        dack_feedback_counts: Counter[LineageIdentity] = Counter()
        hop_admission_counts: Counter[LineageIdentity] = Counter()
        hop_admission_event_indexes: defaultdict[
            LineageIdentity, list[int]
        ] = defaultdict(list)
        hop_first_reception_counts: Counter[LineageIdentity] = Counter()
        hop_first_reception_event_indexes: defaultdict[
            LineageIdentity, list[int]
        ] = defaultdict(list)
        hop_feedback_counts: Counter[LineageIdentity] = Counter()
        hop_completion_outcomes: defaultdict[
            LineageIdentity, list[tuple[int, str, str]]
        ] = defaultdict(list)
        feedback_reasons: defaultdict[LineageIdentity, list[str]] = defaultdict(list)
        event_counts: Counter[str] = Counter()
        previous_index = -1
        previous_time = -1.0
        row_count = 0

        with trace.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            require(reader.fieldnames is not None, "trace has no CSV header")
            require(
                len(reader.fieldnames) == len(set(reader.fieldnames)),
                "trace has duplicate CSV columns",
            )
            require(
                required_columns <= set(reader.fieldnames),
                "trace lacks columns required for DACK terminal proof",
            )
            for row_number, row in enumerate(reader, start=2):
                row_count += 1
                require(None not in row and all(value is not None for value in row.values()), f"trace row {row_number} has the wrong field count")
                require(row["schema"].strip() == "csr-differential-trace-v1", f"trace row {row_number} has wrong schema")
                index_text = row["event_index"].strip()
                require(index_text.isdigit(), f"trace row {row_number} has invalid event_index")
                event_index = int(index_text)
                try:
                    time_s = float(row["time_s"].strip())
                except ValueError as error:
                    raise CertificationError(f"trace row {row_number} has invalid time_s") from error
                require(math.isfinite(time_s) and time_s >= 0.0, f"trace row {row_number} has non-finite/negative time_s")
                require(event_index > previous_index, f"trace row {row_number} event indexes are not strictly increasing")
                require(time_s >= previous_time, f"trace row {row_number} time moves backwards")
                previous_index = event_index
                previous_time = time_s

                key: PacketKey = tuple(
                    row[name].strip() for name in ("src", "dst", "sequence")
                )  # type: ignore[assignment]
                event = row["event"].strip()
                require(event, f"trace row {row_number} has an empty event")
                event_counts[event] += 1
                if event == "nwk_delivery":
                    require(all(key), f"delivery row {row_number} lacks packet identity")
                    deliveries[key] += 1
                if event == "app_send":
                    require(all(key), f"app_send row {row_number} lacks packet identity")
                    app_sends[key] += 1
                if event == "hop_admission" and key in lineage_keys:
                    detail = cls.parse_trace_detail(
                        row["detail"].strip(), row_number
                    )
                    node = row["node"].strip()
                    peer = row["peer"].strip()
                    hop_sequence = detail.get("hop_sequence", "")
                    require(
                        row["reason"].strip() == "admitted"
                        and node.isdigit()
                        and peer.isdigit()
                        and row["next_hop"].strip() == peer
                        and hop_sequence.isdigit(),
                        f"trace row {row_number} has malformed tagged HOP admission evidence",
                    )
                    identity: LineageIdentity = (
                        key[0],
                        key[1],
                        key[2],
                        node,
                        peer,
                        hop_sequence,
                    )
                    hop_admission_counts[identity] += 1
                    hop_admission_event_indexes[identity].append(event_index)
                if event == "hop_feedback" and key in lineage_keys:
                    detail = cls.parse_trace_detail(
                        row["detail"].strip(), row_number
                    )
                    first_reception = detail.get("first_reception")
                    require(
                        first_reception in {"0", "1"},
                        f"trace row {row_number} has malformed first-reception evidence",
                    )
                    node = row["node"].strip()
                    peer = row["peer"].strip()
                    hop_sequence = detail.get("hop_sequence", "")
                    require(
                        row["reason"].strip() in {"ack", "dack"}
                        and node.isdigit()
                        and peer.isdigit()
                        and hop_sequence.isdigit(),
                        f"trace row {row_number} has malformed tagged HOP reception evidence",
                    )
                    directed_identity: LineageIdentity = (
                        key[0],
                        key[1],
                        key[2],
                        peer,
                        node,
                        hop_sequence,
                    )
                    hop_feedback_counts[directed_identity] += 1
                    if first_reception == "1":
                        hop_first_reception_counts[directed_identity] += 1
                        hop_first_reception_event_indexes[
                            directed_identity
                        ].append(event_index)
                if event == "hop_completion" and key in lineage_keys:
                    detail = cls.parse_trace_detail(
                        row["detail"].strip(), row_number
                    )
                    node = row["node"].strip()
                    peer = row["peer"].strip()
                    hop_sequence = detail.get("hop_sequence", "")
                    success = row["success"].strip()
                    reason = row["reason"].strip()
                    require(
                        node.isdigit()
                        and peer.isdigit()
                        and row["next_hop"].strip() == peer
                        and hop_sequence.isdigit()
                        and success in {"0", "1"}
                        and bool(reason),
                        f"trace row {row_number} has malformed tagged HOP completion evidence",
                    )
                    completion_identity: LineageIdentity = (
                        key[0],
                        key[1],
                        key[2],
                        node,
                        peer,
                        hop_sequence,
                    )
                    hop_completion_outcomes[completion_identity].append(
                        (event_index, success, reason)
                    )
                if key in lineage_keys:
                    latest[key] = {
                        "event_index": event_index,
                        "time_s": time_s,
                        "event": event,
                        "reason": row["reason"].strip(),
                        "node": row["node"].strip(),
                        "peer": row["peer"].strip(),
                        "next_hop": row["next_hop"].strip(),
                        "detail": row["detail"].strip(),
                        "row_number": row_number,
                    }

                expectation = proof_expectations.get(event_index)
                if expectation is None:
                    continue
                require(event_index not in seen_proof_indexes, "provenance proof index appears twice in trace")
                seen_proof_indexes.add(event_index)
                identity: LineageIdentity = expectation["identity"]
                expected_key: PacketKey = identity[:3]
                relay, ingress_peer, hop_sequence = identity[3:]
                require(key == expected_key, f"DACK proof index {event_index} has wrong packet identity")
                require(row["node"].strip() == relay, f"DACK proof index {event_index} has wrong relay")
                require(row["peer"].strip() == ingress_peer, f"DACK proof index {event_index} has wrong ingress peer")
                if expectation["kind"] == "enqueue":
                    require(event == "nwk_enqueue" and row["reason"].strip() == "relay", f"DACK proof index {event_index} is not a relay enqueue")
                else:
                    reason = row["reason"].strip()
                    require(event == "hop_feedback" and reason in {"ack", "dack"}, f"DACK proof index {event_index} is not qualifying feedback")
                    detail = cls.parse_trace_detail(row["detail"].strip(), row_number)
                    require(
                        detail.get("hop_sequence") == hop_sequence
                        and detail.get("first_reception") == "1"
                        and detail.get("ackable") == "1"
                        and detail.get("nsdp_state_valid") == "1",
                        f"DACK proof index {event_index} lacks full first-reception evidence",
                    )
                    require(
                        all(
                            (detail.get(name) or "").isdigit()
                            for name in (
                                "nsdp_count_before",
                                "nsdp_count_after",
                                "nsdp_limit",
                            )
                        )
                        and int(detail["nsdp_count_after"])
                        == int(detail["nsdp_count_before"]) + 1
                        and int(detail["nsdp_limit"]) > 0,
                        f"DACK proof index {event_index} lacks a valid NSDP increment",
                    )
                    feedback_reasons[identity].append(reason)
                    if reason == "dack":
                        dack_feedback_counts[identity] += 1

        require(previous_index >= 0, "trace contains no event rows")
        require(
            seen_proof_indexes == set(proof_expectations),
            "one or more DACK provenance proof indexes are absent from the trace",
        )
        return {
            "deliveries": deliveries,
            "app_sends": app_sends,
            "latest": latest,
            "dack_feedback_counts": dack_feedback_counts,
            "hop_admission_counts": hop_admission_counts,
            "hop_admission_event_indexes": hop_admission_event_indexes,
            "hop_first_reception_counts": hop_first_reception_counts,
            "hop_first_reception_event_indexes": (
                hop_first_reception_event_indexes
            ),
            "hop_feedback_counts": hop_feedback_counts,
            "hop_completion_outcomes": hop_completion_outcomes,
            "feedback_reasons": feedback_reasons,
            "event_counts": event_counts,
            "row_count": row_count,
        }

    @classmethod
    def classify_packet_path_v2(
        cls,
        path_summary: dict[str, Any],
        packet_csv: Path,
        path_code: int,
        budgets: dict[PacketKey, int],
        lineage_records: dict[PacketKey, dict[str, Any]],
        observations: dict[str, Any],
    ) -> dict[str, Any]:
        """Validate the schema-v2 lifecycle ledger as a strict clean gate."""

        required_columns = {
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
        }
        lifecycle_ids: set[tuple[PacketKey, int]] = set()
        rows_by_key: defaultdict[PacketKey, list[dict[str, Any]]] = defaultdict(list)
        csv_deliveries: Counter[PacketKey] = Counter()
        csv_issue_counts: Counter[str] = Counter()
        terminal_unreceived_attempts: list[dict[str, Any]] = []
        consumed_terminal_unreceived_identities: set[LineageIdentity] = set()
        observed_hop_admissions: Counter[LineageIdentity] = observations[
            "hop_admission_counts"
        ]
        observed_hop_admission_indexes: defaultdict[
            LineageIdentity, list[int]
        ] = observations["hop_admission_event_indexes"]
        observed_hop_first_receptions: Counter[LineageIdentity] = observations[
            "hop_first_reception_counts"
        ]
        observed_hop_first_reception_indexes: defaultdict[
            LineageIdentity, list[int]
        ] = observations["hop_first_reception_event_indexes"]
        observed_hop_feedback: Counter[LineageIdentity] = observations[
            "hop_feedback_counts"
        ]
        observed_hop_completions: defaultdict[
            LineageIdentity, list[tuple[int, str, str]]
        ] = observations["hop_completion_outcomes"]
        require(
            all(count == 1 for count in observed_hop_admissions.values()),
            "trace repeats a directed-HOP admission identity",
        )
        with packet_csv.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            require(reader.fieldnames is not None, "packet-path CSV has no header")
            require(
                len(reader.fieldnames) == len(set(reader.fieldnames)),
                "packet-path CSV has duplicate columns",
            )
            require(
                required_columns <= set(reader.fieldnames),
                "packet-path schema-v2 CSV lacks lifecycle proof columns",
            )
            for row_number, row in enumerate(reader, start=2):
                key: PacketKey = tuple(
                    (row.get(name) or "").strip()
                    for name in ("src", "dst", "sequence")
                )  # type: ignore[assignment]
                copy_text = (row.get("copy_index") or "").strip()
                require(
                    all(value.isdigit() for value in key) and copy_text.isdigit(),
                    f"packet-path row {row_number} has malformed lifecycle identity",
                )
                copy_index = int(copy_text)
                lifecycle_id = (key, copy_index)
                require(
                    lifecycle_id not in lifecycle_ids,
                    f"packet-path row {row_number} repeats lifecycle identity",
                )
                lifecycle_ids.add(lifecycle_id)

                flags = {
                    name: (row.get(name) or "").strip()
                    for name in ("valid", "complete", "delivered")
                }
                require(
                    all(value in {"0", "1"} for value in flags.values()),
                    f"packet-path row {row_number} has malformed lifecycle flags",
                )
                valid = flags["valid"] == "1"
                complete = flags["complete"] == "1"
                delivered = flags["delivered"] == "1"
                expected_status = (
                    "valid_delivered" if delivered else "incomplete_undelivered"
                )
                require(
                    valid
                    and complete == delivered
                    and row.get("status") == expected_status,
                    f"packet-path row {row_number} is not a valid delivered lifecycle or valid terminal prefix",
                )

                try:
                    issues = json.loads(row.get("issues_json") or "[]")
                    path = json.loads(row.get("path_json") or "[]")
                    hop_lineage = json.loads(row.get("hop_lineage_json") or "[]")
                except json.JSONDecodeError as error:
                    raise CertificationError(
                        f"packet-path row {row_number} has invalid JSON proof data"
                    ) from error
                require(
                    isinstance(issues, list)
                    and all(
                        isinstance(issue, dict)
                        and isinstance(issue.get("code"), str)
                        and issue["code"]
                        for issue in issues
                    )
                    and (row.get("issue_count") or "").isdigit()
                    and int(row["issue_count"]) == len(issues),
                    f"packet-path row {row_number} has malformed issue evidence",
                )
                issue_codes = [issue["code"] for issue in issues]
                require(
                    len(issue_codes) == len(set(issue_codes)),
                    f"packet-path row {row_number} repeats an issue code",
                )
                require(
                    (delivered and not issue_codes)
                    or (
                        not delivered
                        and set(issue_codes)
                        == {"missing_delivery", "undelivered_lifecycle"}
                    ),
                    f"packet-path row {row_number} moves terminal-prefix issues to the wrong lifecycle",
                )
                csv_issue_counts.update(issue_codes)
                require(
                    isinstance(path, list)
                    and path
                    and all(
                        isinstance(node, int)
                        and not isinstance(node, bool)
                        and node >= 0
                        for node in path
                    )
                    and path[0] == int(key[0])
                    and (not delivered or path[-1] == int(key[1])),
                    f"packet-path row {row_number} has malformed path identity",
                )
                require(
                    isinstance(hop_lineage, list),
                    f"packet-path row {row_number} has malformed HOP lineage",
                )
                parsed_legs: list[tuple[str, str, str, str, str, str]] = []
                for leg in hop_lineage:
                    names = {"src", "dst", "sequence", "from", "to", "hop_sequence"}
                    require(
                        isinstance(leg, dict)
                        and set(leg) == names
                        and all(
                            isinstance(leg[name], int)
                            and not isinstance(leg[name], bool)
                            and leg[name] >= 0
                            for name in names
                        )
                        and leg["src"] == int(key[0])
                        and leg["dst"] == int(key[1])
                        and leg["sequence"] == int(key[2]),
                        f"packet-path row {row_number} has malformed directed-HOP identity",
                    )
                    parsed_legs.append(
                        tuple(
                            str(leg[name])
                            for name in (
                                "src",
                                "dst",
                                "sequence",
                                "from",
                                "to",
                                "hop_sequence",
                            )
                        )  # type: ignore[arg-type]
                    )
                require(
                    len(parsed_legs) == len(set(parsed_legs)),
                    f"packet-path row {row_number} repeats a directed-HOP identity",
                )
                require(
                    all(observed_hop_admissions.get(leg, 0) == 1 for leg in parsed_legs),
                    f"packet-path row {row_number} contains HOP lineage absent from the trace",
                )
                if hop_lineage:
                    relation = hop_lineage_path_relation(
                        path, hop_lineage, delivered
                    )
                    require(
                        relation is not None,
                        f"packet-path row {row_number} HOP lineage does not reproduce its path",
                    )
                    for observed_identity in parsed_legs[: len(path) - 1]:
                        admission_indexes = observed_hop_admission_indexes.get(
                            observed_identity, []
                        )
                        reception_indexes = (
                            observed_hop_first_reception_indexes.get(
                                observed_identity, []
                            )
                        )
                        require(
                            len(admission_indexes) == 1
                            and bool(reception_indexes)
                            and all(
                                index > admission_indexes[0]
                                for index in reception_indexes
                            ),
                            f"packet-path row {row_number} observed path edge lacks post-admission first-reception evidence",
                        )
                    if relation == "terminal_admitted_unreceived":
                        terminal_identity = parsed_legs[-1]
                        terminal_admission_indexes = (
                            observed_hop_admission_indexes.get(
                                terminal_identity, []
                            )
                        )
                        terminal_completions = observed_hop_completions.get(
                            terminal_identity, []
                        )
                        require(
                            observed_hop_feedback.get(terminal_identity, 0)
                            == 0
                            and observed_hop_first_receptions.get(
                                terminal_identity, 0
                            )
                            == 0,
                            f"packet-path row {row_number} terminal HOP attempt has receiver feedback",
                        )
                        require(
                            terminal_identity
                            not in consumed_terminal_unreceived_identities,
                            f"packet-path row {row_number} reuses one terminal HOP attempt across lifecycles",
                        )
                        consumed_terminal_unreceived_identities.add(
                            terminal_identity
                        )
                        require(
                            len(terminal_admission_indexes) == 1
                            and len(terminal_completions) == 1
                            and terminal_completions[0][1:]
                            == ("0", "no_ack")
                            and terminal_completions[0][0]
                            > terminal_admission_indexes[0],
                            f"packet-path row {row_number} terminal HOP attempt lacks one exact post-admission no_ack completion",
                        )
                        terminal = hop_lineage[-1]
                        terminal_unreceived_attempts.append(
                            {
                                "src": int(key[0]),
                                "dst": int(key[1]),
                                "sequence": int(key[2]),
                                "copy_index": copy_index,
                                "from": terminal["from"],
                                "to": terminal["to"],
                                "hop_sequence": terminal["hop_sequence"],
                                "admission_event_index": (
                                    terminal_admission_indexes[0]
                                ),
                                "completion_event_index": (
                                    terminal_completions[0][0]
                                ),
                                "completion_success": 0,
                                "completion_reason": "no_ack",
                            }
                        )
                if key in budgets:
                    lineage = lineage_records[key]
                    expected_leg: LineageIdentity = tuple(
                        lineage[name]
                        for name in (
                            "src",
                            "dst",
                            "sequence",
                            "ingress_peer",
                            "relay",
                            "hop_sequence",
                        )
                    )  # type: ignore[assignment]
                    require(
                        expected_leg in parsed_legs,
                        f"packet-path row {row_number} omits its provenance-bound repeated-DACK leg",
                    )
                else:
                    require(
                        not parsed_legs,
                        f"packet-path row {row_number} invents branch lineage without a DACK proof budget",
                    )

                if delivered:
                    csv_deliveries[key] += 1
                rows_by_key[key].append(
                    {
                        "copy_index": copy_index,
                        "delivered": delivered,
                        "complete": complete,
                        "hop_lineage": tuple(parsed_legs),
                    }
                )

        app_sends: Counter[PacketKey] = observations["app_sends"]
        deliveries: Counter[PacketKey] = observations["deliveries"]
        require(
            app_sends
            and all(count == 1 for count in app_sends.values())
            and set(rows_by_key) == set(app_sends),
            "packet-path schema-v2 keys do not exactly match one-send application identities",
        )
        for key, rows in rows_by_key.items():
            expected_lifecycles = 1 + budgets.get(key, 0)
            require(
                len(rows) == expected_lifecycles
                and sorted(row["copy_index"] for row in rows)
                == list(range(expected_lifecycles))
                and csv_deliveries.get(key, 0) == deliveries.get(key, 0),
                f"packet-path schema-v2 copy/delivery accounting drift for {key}",
            )
            delivered_lineages = [
                row["hop_lineage"] for row in rows if row["delivered"]
            ]
            require(
                len(delivered_lineages) <= 1
                or len(set(delivered_lineages)) == len(delivered_lineages),
                f"packet-path schema-v2 repeats one delivered HOP lineage for {key}",
            )
            if key in budgets:
                reported_legs = {
                    leg
                    for row in rows
                    for leg in row["hop_lineage"]
                }
                trace_legs = {
                    identity
                    for identity in observed_hop_admissions
                    if identity[:3] == key
                }
                require(
                    reported_legs == trace_legs,
                    f"packet-path schema-v2 does not exactly cover trace HOP identities for {key}",
                )

        counts = path_summary.get("counts") or {}
        delivered_count = sum(deliveries.values())
        lifecycle_count = len(lifecycle_ids)
        undelivered_count = lifecycle_count - delivered_count
        exact_counts = {
            "packet_keys": len(app_sends),
            "packet_lifecycles": lifecycle_count,
            "reconstructed_branch_packet_keys": len(budgets),
            "source_exact_dack_retry_forks": sum(budgets.values()),
            "ambiguous_repeated_dack_branch_packet_keys": 0,
            "delivered_packets": delivered_count,
            "valid_complete_delivered_packets": delivered_count,
            "invalid_packets": 0,
            "incomplete_packets": undelivered_count,
            "incomplete_delivered_packets": 0,
            "undelivered_packets": undelivered_count,
            "route_context_mismatch_packets": 0,
            "route_context_mismatches": 0,
        }
        require(isinstance(counts, dict), "packet-path schema-v2 counts are absent")
        for name, expected in exact_counts.items():
            actual = counts.get(name)
            require(
                isinstance(actual, int)
                and not isinstance(actual, bool)
                and actual == expected,
                f"packet-path schema-v2 count {name} is {actual}, expected {expected}",
            )
        expected_issue_counts = {
            "missing_delivery": undelivered_count,
            "undelivered_lifecycle": undelivered_count,
        }
        expected_issue_counts = {
            name: count for name, count in expected_issue_counts.items() if count
        }
        require(
            dict(sorted(csv_issue_counts.items())) == expected_issue_counts
            and path_summary.get("issue_counts") == expected_issue_counts,
            "packet-path schema-v2 issue ledger exceeds valid terminal-prefix diagnostics",
        )
        require(
            path_summary.get("pass") is True and path_code == 0,
            "packet-path schema-v2 is not a strict clean pass",
        )
        return {
            "classification": "strict_repeated_dack_branch_pass",
            "schema": "csr-packet-path-analysis-v2",
            "strict_result": "pass",
            "strict_exit_code": path_code,
            "release_disposition": "accepted_clean_gate",
            "packet_path_integrity_claim": True,
            "opnet_packet_event_parity_claim": False,
            "packet_key_count": len(app_sends),
            "packet_lifecycle_count": lifecycle_count,
            "reconstructed_branch_packet_keys": len(budgets),
            "source_exact_dack_retry_forks": sum(budgets.values()),
            "terminal_unreceived_hop_attempt_count": len(
                terminal_unreceived_attempts
            ),
            "terminal_unreceived_hop_attempts": terminal_unreceived_attempts,
            "note": (
                "Every observed ns-3 lifecycle or terminal prefix is strict-valid. "
                "Repeated-DACK copies are bound to exact directed-HOP identities; "
                "any terminal unreceived HOP is admission-, reception-, and "
                "no_ack-completion-bound; "
                "the recovered OPNET evidence remains aggregate-only."
            ),
        }

    @staticmethod
    def classify_packet_path_v1_clean(
        path_summary: dict[str, Any],
        packet_csv: Path,
        path_code: int,
        extras: dict[PacketKey, int],
        observations: dict[str, Any],
    ) -> dict[str, Any]:
        """Fail closed on every row/count of a legacy schema-v1 clean pass."""

        require(
            path_summary.get("pass") is True and path_code == 0 and not extras,
            "packet-path v1 clean pass contradicts status or trace deliveries",
        )
        required = {
            "src",
            "dst",
            "sequence",
            "status",
            "valid",
            "complete",
            "delivered",
            "issue_count",
            "issues_json",
        }
        rows: set[PacketKey] = set()
        with packet_csv.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            require(reader.fieldnames is not None, "packet-path v1 CSV has no header")
            require(
                len(reader.fieldnames) == len(set(reader.fieldnames))
                and required <= set(reader.fieldnames),
                "packet-path v1 CSV lacks clean-pass proof columns",
            )
            for row_number, row in enumerate(reader, start=2):
                require(
                    None not in row and all(value is not None for value in row.values()),
                    f"packet-path v1 row {row_number} has the wrong field count",
                )
                key: PacketKey = tuple(
                    row[name].strip() for name in ("src", "dst", "sequence")
                )  # type: ignore[assignment]
                require(
                    all(value.isdigit() for value in key) and key not in rows,
                    f"packet-path v1 row {row_number} has malformed/repeated identity",
                )
                rows.add(key)
                try:
                    issues = json.loads(row["issues_json"])
                except json.JSONDecodeError as error:
                    raise CertificationError(
                        f"packet-path v1 row {row_number} has invalid issues JSON"
                    ) from error
                require(
                    row["status"] == "valid_delivered"
                    and row["valid"] == "1"
                    and row["complete"] == "1"
                    and row["delivered"] == "1"
                    and row["issue_count"] == "0"
                    and issues == [],
                    f"packet-path v1 row {row_number} is not an exact clean delivery",
                )

        app_sends: Counter[PacketKey] = observations["app_sends"]
        deliveries: Counter[PacketKey] = observations["deliveries"]
        require(
            rows
            and set(app_sends) == rows == set(deliveries)
            and all(app_sends[key] == deliveries[key] == 1 for key in rows),
            "packet-path v1 rows do not exactly match one-send/one-delivery trace identities",
        )
        counts = path_summary.get("counts")
        require(isinstance(counts, dict), "packet-path v1 clean counts are absent")
        expected = {
            "packet_keys": len(rows),
            "delivered_packets": len(rows),
            "valid_complete_delivered_packets": len(rows),
            "invalid_packets": 0,
            "incomplete_packets": 0,
            "incomplete_delivered_packets": 0,
            "undelivered_packets": 0,
            "route_context_mismatch_packets": 0,
            "route_context_mismatches": 0,
        }
        require(
            all(counts.get(name) == value for name, value in expected.items())
            and path_summary.get("issue_counts") == {},
            "packet-path v1 clean summary disagrees with its trace-bound CSV",
        )
        return {
            "classification": "strict_clean_pass",
            "schema": "csr-packet-path-analysis-v1",
            "strict_result": "pass",
            "strict_exit_code": path_code,
            "release_disposition": "accepted_clean_gate",
            "packet_path_integrity_claim": True,
            "opnet_packet_event_parity_claim": False,
            "invalid_packet_count": 0,
        }

    @staticmethod
    def classify_admission_integrity(
        admission: dict[str, Any],
        admission_code: int,
        extras: dict[PacketKey, int],
    ) -> dict[str, Any]:
        """Validate admission strict status against its explicit integrity ledger."""

        integrity = admission.get("integrity")
        require(isinstance(integrity, dict), "admission integrity ledger is absent")
        invalid_leg_count = integrity.get("invalid_leg_count")
        global_issue_count = integrity.get("global_issue_count")
        issues = integrity.get("global_issues")
        require(
            isinstance(invalid_leg_count, int)
            and not isinstance(invalid_leg_count, bool)
            and invalid_leg_count >= 0
            and isinstance(global_issue_count, int)
            and not isinstance(global_issue_count, bool)
            and global_issue_count >= 0
            and isinstance(issues, list)
            and global_issue_count == len(issues),
            "admission integrity counts/issues are malformed or inconsistent",
        )
        if admission.get("pass") is True:
            require(
                admission_code == 0
                and not extras
                and invalid_leg_count == 0
                and global_issue_count == 0
                and not issues,
                "admission pass contradicts its exit status, deliveries, or integrity ledger",
            )
            return {
                "gate_pass": True,
                "classification": "strict_clean_pass",
                "strict_result": "pass",
                "strict_exit_code": admission_code,
                "release_disposition": "accepted_clean_gate",
            }

        require(admission.get("pass") is False, "admission pass field is not boolean")
        require(admission_code == 1, "admission failure/exit inconsistency")
        require(invalid_leg_count == 0, "admission has invalid HOP legs")
        require(
            issues
            and all(
                isinstance(issue, dict)
                and issue.get("code") == "duplicate_delivery"
                for issue in issues
            ),
            "admission has non-duplicate global issues",
        )
        issue_keys: set[PacketKey] = set()
        pattern = re.compile(
            r"PacketKey\(source=(\d+), destination=(\d+), sequence=(\d+)\)"
        )
        for issue in issues:
            match = pattern.search(str(issue.get("message", "")))
            require(match is not None, "cannot bind admission duplicate issue to packet key")
            issue_keys.add(match.groups())
        require(
            len(issues) == len(extras) and issue_keys == set(extras),
            "admission duplicate issues/keys do not exactly match delivered proven lineages",
        )
        return {
            "gate_pass": True,
            "classification": "source_proved_dack_duplicate_counter_limitation",
            "strict_result": "fail",
            "strict_exit_code": admission_code,
            "release_disposition": "accepted_narrow_source_proved_exception",
            "duplicate_packet_keys": len(issue_keys),
            "note": "Only the packet-wide duplicate-delivery assertion is waived; any invalid leg or other global issue remains release-blocking.",
        }

    @staticmethod
    def validate_admission_legs(path: Path) -> dict[str, int]:
        """Validate and count the exact analyzer leg-ledger artifact."""

        row_count = 0
        invalid_count = 0
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            require(
                reader.fieldnames == list(ADMISSION_LEG_COLUMNS),
                "admission leg CSV header/schema drift",
            )
            for row_number, row in enumerate(reader, start=2):
                row_count += 1
                require(
                    None not in row and all(value is not None for value in row.values()),
                    f"admission leg row {row_number} has the wrong field count",
                )
                require(
                    all(
                        row[name].strip().isdigit()
                        for name in ("src", "dst", "sequence", "leg_index", "node")
                    )
                    and row["valid"] in {"0", "1"}
                    and bool(row["status"].strip()),
                    f"admission leg row {row_number} has malformed identity/status",
                )
                try:
                    issues = json.loads(row["issues_json"])
                except json.JSONDecodeError as error:
                    raise CertificationError(
                        f"admission leg row {row_number} has invalid issues JSON"
                    ) from error
                require(
                    isinstance(issues, list)
                    and all(
                        isinstance(issue, dict)
                        and isinstance(issue.get("code"), str)
                        and issue["code"]
                        for issue in issues
                    )
                    and ((row["valid"] == "1") == (not issues)),
                    f"admission leg row {row_number} has inconsistent validity/issues",
                )
                invalid_count += row["valid"] == "0"
        return {"row_count": row_count, "invalid_leg_count": invalid_count}

    @staticmethod
    def validate_application_diagnostics(path: Path) -> dict[str, Any]:
        """Validate the canonical application-admission diagnostic partition."""

        totals = {name: 0 for name in APP_DIAGNOSTIC_COUNTERS}
        flows: set[int] = set()
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            require(
                reader.fieldnames == list(APP_DIAGNOSTIC_COLUMNS),
                "application diagnostics CSV header/schema drift",
            )
            for row_number, row in enumerate(reader, start=2):
                require(
                    None not in row and all(value is not None for value in row.values()),
                    f"application diagnostics row {row_number} has the wrong field count",
                )
                flow_text = row["flow_index"].strip()
                require(
                    row["schema"] == "csr-app-admission-diagnostics-v1"
                    and row["scenario"] == "blue_radio_campus-multihop"
                    and row["application_profile"] == "legacy-send-only-no-dscp"
                    and flow_text.isdigit()
                    and int(flow_text) not in flows
                    and row["source"].strip().isdigit()
                    and row["configured_destination"].strip().isdigit()
                    and row["destination_mode"] == "fixed",
                    f"application diagnostics row {row_number} has identity/profile drift",
                )
                flows.add(int(flow_text))
                counters: dict[str, int] = {}
                for name in APP_DIAGNOSTIC_COUNTERS:
                    value = row[name].strip()
                    require(
                        value.isdigit(),
                        f"application diagnostics row {row_number} has invalid {name}",
                    )
                    counters[name] = int(value)
                    totals[name] += int(value)
                require(
                    counters["attempts"]
                    == sum(
                        counters[name]
                        for name in APP_DIAGNOSTIC_COUNTERS
                        if name != "attempts"
                    ),
                    f"application diagnostics row {row_number} does not partition attempts",
                )
                times: list[float | None] = []
                for name in ("first_admitted_s", "last_admitted_s"):
                    value = row[name].strip()
                    try:
                        parsed = float(value) if value else None
                    except ValueError as error:
                        raise CertificationError(
                            f"application diagnostics row {row_number} has invalid {name}"
                        ) from error
                    require(
                        parsed is None or (math.isfinite(parsed) and parsed >= 0.0),
                        f"application diagnostics row {row_number} has non-finite {name}",
                    )
                    times.append(parsed)
                first, last = times
                require(
                    (
                        counters["admitted"] == 0
                        and first is None
                        and last is None
                    )
                    or (
                        counters["admitted"] > 0
                        and first is not None
                        and last is not None
                        and first <= last < 6000.0
                    ),
                    f"application diagnostics row {row_number} has inconsistent admission times",
                )
        require(
            flows == set(range(6)),
            "application diagnostics flow set is not the canonical six flows",
        )
        return {
            "row_count": len(flows),
            "flow_indexes": sorted(flows),
            "totals": totals,
        }

    def classify_analyzer_results(
        self,
        trace: Path,
        provenance_path: Path,
        admission_summary_path: Path,
        admission_code: int,
        path_summary_path: Path,
        packet_csv: Path,
        path_code: int,
    ) -> dict[str, Any]:
        admission_legs_path = admission_summary_path.parent / "admission-legs.csv"
        diagnostics_path = (
            admission_summary_path.parent / "app-admission-diagnostics.csv"
        )
        evidence_paths = {
            "trace": trace,
            "aggregate_provenance": provenance_path,
            "packet_path_summary": path_summary_path,
            "packet_paths_csv": packet_csv,
            "admission_summary": admission_summary_path,
            "admission_legs": admission_legs_path,
            "application_diagnostics": diagnostics_path,
        }
        require(
            all(
                path.is_file() and not path.is_symlink()
                for path in evidence_paths.values()
            ),
            "classified evidence must be present as regular non-symlink files",
        )
        evidence_digests = {
            name: sha256(path) for name, path in evidence_paths.items()
        }
        trace_digest = evidence_digests["trace"]
        provenance_digest = evidence_digests["aggregate_provenance"]
        provenance = json_load(provenance_path)
        duplicate = provenance.get("source_exact_dack_retry_duplicates") or {}
        duplicate_classification = classify_duplicate_lineage_summary(duplicate)
        proof_count = duplicate.get("repeated_feedback_proof_count")
        matched_duplicate_count = duplicate.get("matched_duplicate_delivery_count")
        unused_proof_count = duplicate.get("unused_proof_count")
        lineages = duplicate.get("lineages")
        budgets: dict[PacketKey, int] = {}
        lineage_records: dict[PacketKey, dict[str, Any]] = {}
        lineage_identities: set[LineageIdentity] = set()
        proof_expectations: dict[int, dict[str, Any]] = {}
        for lineage in lineages:
            require(isinstance(lineage, dict), "malformed DACK lineage")
            identity: LineageIdentity = tuple(
                lineage.get(name) for name in (
                    "src",
                    "dst",
                    "sequence",
                    "relay",
                    "ingress_peer",
                    "hop_sequence",
                )
            )  # type: ignore[assignment]
            require(
                all(isinstance(value, str) and value.isdigit() for value in identity)
                and identity not in lineage_identities,
                "malformed or repeated DACK lineage identity",
            )
            lineage_identities.add(identity)
            key: PacketKey = identity[:3]
            require(
                key not in lineage_records,
                "packet key has multiple DACK lineages and cannot be classified one-to-one",
            )
            qualifying = lineage.get("qualifying_feedback_count")
            dack_count = lineage.get("dack_feedback_count")
            budget = lineage.get("proven_extra_delivery_budget")
            pairs = lineage.get("source_ordered_pairs")
            require(
                isinstance(qualifying, int)
                and not isinstance(qualifying, bool)
                and qualifying >= 2
                and isinstance(dack_count, int)
                and not isinstance(dack_count, bool)
                and dack_count in {qualifying - 1, qualifying}
                and isinstance(budget, int)
                and not isinstance(budget, bool)
                and budget == qualifying - 1
                and isinstance(pairs, list)
                and len(pairs) == qualifying,
                "malformed DACK lineage proof/budget",
            )
            previous_feedback_index = -1
            for pair in pairs:
                require(isinstance(pair, dict), "malformed DACK lineage pair")
                enqueue_index = pair.get("enqueue_event_index")
                feedback_index = pair.get("feedback_event_index")
                require(
                    isinstance(enqueue_index, int)
                    and not isinstance(enqueue_index, bool)
                    and enqueue_index >= 0
                    and isinstance(feedback_index, int)
                    and not isinstance(feedback_index, bool)
                    and feedback_index > enqueue_index > previous_feedback_index
                    and enqueue_index not in proof_expectations
                    and feedback_index not in proof_expectations,
                    "malformed or reused DACK lineage event index",
                )
                proof_expectations[enqueue_index] = {
                    "kind": "enqueue",
                    "identity": identity,
                }
                proof_expectations[feedback_index] = {
                    "kind": "feedback",
                    "identity": identity,
                }
                previous_feedback_index = feedback_index
            budgets[key] = budget
            lineage_records[key] = lineage
        require(
            sum(budgets.values()) == proof_count,
            "DACK lineage budgets do not equal provenance proof count",
        )

        observations = self.trace_lineage_observations(
            trace, set(budgets), proof_expectations
        )
        require(
            sha256(trace) == trace_digest,
            "trace changed while source-exact lineage observations were parsed",
        )
        provenance_input = provenance.get("input") or {}
        provenance_window = provenance.get("window") or {}
        require(
            provenance.get("schema") == "csr-ns3-aggregate-provenance-v1"
            and provenance.get("scenario") == "blue_radio_campus-multihop"
            and provenance_input.get("path") == str(trace.resolve())
            and provenance_input.get("sha256") == trace_digest
            and provenance_input.get("size_bytes") == trace.stat().st_size
            and provenance_input.get("event_row_count") == observations["row_count"]
            and provenance_window.get("bucket_width_s") == 60.0
            and provenance_window.get("stop_time_s") == 6000.0
            and provenance_window.get("stop_endpoint") == "exclusive",
            "aggregate provenance is not exactly bound to the canonical enriched trace/window",
        )
        for key, lineage in lineage_records.items():
            identity = tuple(
                lineage[name]
                for name in (
                    "src",
                    "dst",
                    "sequence",
                    "relay",
                    "ingress_peer",
                    "hop_sequence",
                )
            )
            require(
                observations["dack_feedback_counts"].get(identity, 0)
                == lineage["dack_feedback_count"],
                f"DACK feedback count does not bind to trace for {key}",
            )
            feedback_reasons = observations["feedback_reasons"].get(identity, [])
            require(
                len(feedback_reasons) == lineage["qualifying_feedback_count"]
                and all(reason == "dack" for reason in feedback_reasons[:-1])
                and feedback_reasons[-1] in {"ack", "dack"},
                f"DACK feedback ordering does not prove every retry budget for {key}",
            )
        deliveries: Counter[PacketKey] = observations["deliveries"]
        extras = {
            key: count - 1 for key, count in deliveries.items() if count > 1
        }
        require(
            sum(extras.values()) == matched_duplicate_count,
            "actual duplicate deliveries do not equal aggregate provenance",
        )
        for key, extra in extras.items():
            require(
                key in budgets and budgets[key] == extra,
                f"duplicate delivery does not exactly consume one full DACK lineage budget: {key}",
            )
        remaining_budgets = {
            key: budget - extras.get(key, 0)
            for key, budget in budgets.items()
            if budget - extras.get(key, 0) > 0
        }
        require(
            set(extras).isdisjoint(remaining_budgets)
            and set(extras) | set(remaining_budgets) == set(budgets)
            and sum(remaining_budgets.values()) == unused_proof_count
            and len(remaining_budgets) == unused_proof_count
            and all(value == 1 for value in remaining_budgets.values()),
            "matched and unused DACK lineage budgets do not form an exact one-key partition",
        )

        path_summary_digest = evidence_digests["packet_path_summary"]
        path_summary = json_load(path_summary_path)
        path_schema = path_summary.get("schema")
        require(
            path_schema in SUPPORTED_PACKET_PATH_SCHEMAS,
            f"unsupported packet-path schema: {path_schema!r}",
        )
        path_input = path_summary.get("input") or {}
        require(
            path_input.get("path") == str(trace.resolve())
            and path_input.get("sha256") == trace_digest
            and path_input.get("row_count") == observations["row_count"],
            "packet-path input identity/count drift",
        )
        path_configuration = path_summary.get("configuration") or {}
        require(
            path_configuration.get("startup_time_s") == 300.0
            and path_configuration.get("end_to_end_bucket_width_s") == 60.0
            and path_configuration.get("end_to_end_stop_time_s") == 6000.0
            and path_configuration.get("end_to_end_stop_time_semantics")
            == "exclusive"
            and path_configuration.get("decomposition_tolerance_s") == 1.0e-9,
            "packet-path analyzer configuration drift",
        )
        if path_schema == "csr-packet-path-analysis-v2":
            require(
                path_configuration.get("packet_key")
                == ["src", "dst", "sequence"]
                and path_configuration.get("directed_hop_identity")
                == ["src", "dst", "sequence", "from", "to", "hop_sequence"]
                and path_configuration.get("repeated_dack_branch_matching")
                == PACKET_PATH_V2_MATCHING_CONTRACT
                and path_configuration.get("undelivered_path_semantics")
                == PACKET_PATH_V2_UNDELIVERED_PATH_CONTRACT
                and path_configuration.get("queue_delay_statistic")
                == PACKET_PATH_V2_QUEUE_DELAY_STATISTIC
                and path_configuration.get("queue_delay_evidence_adjacency")
                == PACKET_PATH_V2_QUEUE_DELAY_ADJACENCY
                and path_configuration.get("queue_delay_match_abs_tolerance_s")
                == 1.0e-9
                and path_summary.get("pass") is True,
                "packet-path schema-v2 unique queue-delay matching contract drift or strict failure",
            )
        packet_output = (path_summary.get("outputs") or {}).get("packet_csv") or {}
        packet_csv_digest = evidence_digests["packet_paths_csv"]
        require(
            packet_output.get("path") == str(packet_csv.resolve())
            and packet_output.get("sha256") == packet_csv_digest,
            "packet-path CSV is not hash-bound by its summary",
        )
        invalid_rows: dict[PacketKey, dict[str, Any]] = {}
        packet_keys: set[PacketKey] = set()
        lifecycle_ids: set[tuple[PacketKey, int]] = set()
        with packet_csv.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            require(reader.fieldnames is not None, "packet-path CSV has no header")
            require(
                {
                    "src",
                    "dst",
                    "sequence",
                    "status",
                    "valid",
                    "complete",
                    "delivered",
                    "issues_json",
                }
                <= set(reader.fieldnames),
                "packet-path CSV lacks classification columns",
            )
            for row_number, row in enumerate(reader, start=2):
                key: PacketKey = tuple(
                    (row.get(name) or "").strip()
                    for name in ("src", "dst", "sequence")
                )  # type: ignore[assignment]
                require(all(value.isdigit() for value in key), f"packet-path row {row_number} has malformed key")
                if path_schema == "csr-packet-path-analysis-v2":
                    copy_text = (row.get("copy_index") or "").strip()
                    require(copy_text.isdigit(), f"packet-path row {row_number} has malformed copy index")
                    lifecycle_id = (key, int(copy_text))
                    require(
                        lifecycle_id not in lifecycle_ids,
                        f"packet-path row {row_number} repeats lifecycle identity",
                    )
                    lifecycle_ids.add(lifecycle_id)
                else:
                    require(
                        key not in packet_keys,
                        f"packet-path row {row_number} repeats packet key",
                    )
                packet_keys.add(key)
                require(row.get("valid") in {"0", "1"}, f"packet-path row {row_number} has invalid validity flag")
                if row.get("valid") == "1":
                    continue
                try:
                    issues = json.loads(row.get("issues_json") or "[]")
                except json.JSONDecodeError as error:
                    raise CertificationError(
                        f"packet-path row {row_number} has invalid issues JSON"
                    ) from error
                require(
                    isinstance(issues, list)
                    and all(
                        isinstance(issue, dict)
                        and isinstance(issue.get("code"), str)
                        for issue in issues
                    ),
                    f"packet-path row {row_number} has malformed issues",
                )
                codes = sorted(issue["code"] for issue in issues)
                require(len(codes) == len(set(codes)), f"packet-path row {row_number} repeats an issue code")
                invalid_rows[key] = {
                    "codes": codes,
                    "status": row.get("status"),
                    "complete": row.get("complete"),
                    "delivered": row.get("delivered"),
                }
        require(
            sha256(packet_csv) == packet_csv_digest,
            "packet-path CSV changed while it was being classified",
        )
        path_counts = path_summary.get("counts") or {}
        require(
            isinstance(path_counts, dict)
            and path_counts.get("packet_keys") == len(packet_keys)
            and path_counts.get("invalid_packets") == len(invalid_rows),
            "packet-path CSV/summary packet counts disagree",
        )
        require(
            packet_output.get("row_count")
            == (
                len(lifecycle_ids)
                if path_schema == "csr-packet-path-analysis-v2"
                else len(packet_keys)
            ),
            "packet-path output row count drift",
        )
        if path_summary.get("pass") is True:
            require(path_code == 0 and not invalid_rows, "packet-path pass/exit inconsistency")
            if path_schema == "csr-packet-path-analysis-v2":
                path_classification = self.classify_packet_path_v2(
                    path_summary,
                    packet_csv,
                    path_code,
                    budgets,
                    lineage_records,
                    observations,
                )
            else:
                path_classification = self.classify_packet_path_v1_clean(
                    path_summary,
                    packet_csv,
                    path_code,
                    extras,
                    observations,
                )
        else:
            require(
                path_schema == "csr-packet-path-analysis-v1",
                "packet-path schema-v2 strict failure is release-blocking",
            )
            require(
                path_summary.get("pass") is False
                and path_code == 1
                and invalid_rows,
                "packet-path failure/exit inconsistency",
            )
            require(
                set(invalid_rows) == set(budgets),
                "packet-path invalid keys do not exactly equal full repeated-DACK lineage keys",
            )
            delivered_issue_codes = {
                "duplicate_delivery",
                "incomplete_or_out_of_order_lifecycle",
            }
            terminal_issue_codes = {
                "missing_delivery",
                "incomplete_or_out_of_order_lifecycle",
            }
            delivered_invalid_keys: set[PacketKey] = set()
            terminal_invalid_keys: set[PacketKey] = set()
            for key, row in invalid_rows.items():
                if row["delivered"] == "1":
                    require(
                        row["complete"] == "0"
                        and row["status"] == "invalid_incomplete_delivered"
                        and set(row["codes"]) == delivered_issue_codes,
                        f"delivered DACK-lineage invalidity has unexpected form for {key}: {row}",
                    )
                    delivered_invalid_keys.add(key)
                elif row["delivered"] == "0":
                    require(
                        row["complete"] == "0"
                        and row["status"] == "invalid_incomplete_undelivered"
                        and set(row["codes"]) == terminal_issue_codes,
                        f"terminal DACK-lineage invalidity has unexpected form for {key}: {row}",
                    )
                    terminal_invalid_keys.add(key)
                else:
                    raise CertificationError(
                        f"packet-path invalid row has malformed delivery flag for {key}"
                    )
            require(
                delivered_invalid_keys == set(extras),
                "delivered invalid packet keys do not exactly equal consumed DACK proof keys",
            )
            require(
                terminal_invalid_keys == set(remaining_budgets),
                "terminal-undelivered invalid keys do not exactly equal unused DACK proof keys",
            )

            terminal_proofs: list[dict[str, Any]] = []
            for key in sorted(terminal_invalid_keys):
                require(
                    observations["app_sends"].get(key, 0) == 1
                    and deliveries.get(key, 0) == 0,
                    f"unused DACK lineage is not a single-send zero-delivery terminal packet: {key}",
                )
                terminal = observations["latest"].get(key)
                require(isinstance(terminal, dict), f"unused DACK lineage has no trace terminal observation: {key}")
                lineage = lineage_records[key]
                pairs = lineage["source_ordered_pairs"]
                require(
                    terminal.get("event") == "nwk_admission"
                    and terminal.get("reason") == "neighbor_flow_full"
                    and terminal.get("node") == lineage["relay"]
                    and isinstance(terminal.get("peer"), str)
                    and terminal["peer"].isdigit()
                    and terminal.get("peer") == terminal.get("next_hop")
                    and isinstance(terminal.get("event_index"), int)
                    and terminal["event_index"] > pairs[-1]["feedback_event_index"]
                    and isinstance(terminal.get("time_s"), float)
                    and 5940.0 <= terminal["time_s"] < 6000.0,
                    f"unused DACK lineage lacks a final-bucket relay capacity hold: {key}",
                )
                detail = self.parse_trace_detail(
                    terminal["detail"], terminal["row_number"]
                )
                required_detail = {
                    "queue_before",
                    "queue_after",
                    "nsdp_count",
                    "nsdp_limit",
                    "pending",
                    "pending_limit",
                    "global_spad",
                    "outstanding",
                    "threshold",
                    "neighbor_spad",
                    "route_known",
                }
                require(
                    required_detail <= set(detail)
                    and all(
                        re.fullmatch(r"-?[0-9]+", detail[name]) is not None
                        for name in required_detail - {"route_known"}
                    )
                    and detail["route_known"] == "1",
                    f"terminal DACK-lineage hold has malformed capacity detail: {key}",
                )
                numeric = {
                    name: int(detail[name])
                    for name in required_detail - {"route_known"}
                }
                require(
                    numeric["queue_before"] == numeric["queue_after"]
                    and numeric["queue_before"] >= 0
                    and numeric["nsdp_count"] >= 0
                    and numeric["nsdp_limit"] > 0
                    and numeric["pending_limit"] > 0
                    and 0 <= numeric["pending"] <= numeric["pending_limit"]
                    and numeric["global_spad"]
                    == numeric["pending_limit"] - numeric["pending"] + 1
                    and numeric["neighbor_spad"]
                    == numeric["threshold"] - numeric["outstanding"] + 1
                    and numeric["outstanding"] >= 0
                    and numeric["threshold"] >= 0
                    and numeric["outstanding"] > numeric["threshold"]
                    and numeric["neighbor_spad"] <= 0,
                    f"terminal DACK-lineage hold violates neighbor-capacity semantics: {key}",
                )
                terminal_proofs.append(
                    {
                        "src": key[0],
                        "dst": key[1],
                        "sequence": key[2],
                        "event": terminal["event"],
                        "event_index": terminal["event_index"],
                        "time_s": terminal["time_s"],
                        "reason": terminal["reason"],
                        "node": terminal["node"],
                        "next_hop": terminal["next_hop"],
                        "seconds_before_exclusive_stop": 6000.0
                        - terminal["time_s"],
                    }
                )

            issue_counts = path_summary.get("issue_counts") or {}
            require(
                set(issue_counts)
                <= {
                    "duplicate_delivery",
                    "incomplete_or_out_of_order_lifecycle",
                    "missing_delivery",
                    "undelivered_lifecycle",
                },
                f"unexpected packet-path issue classes: {issue_counts}",
            )
            require(
                issue_counts.get("duplicate_delivery", 0) == len(extras)
                and issue_counts.get("incomplete_or_out_of_order_lifecycle", 0)
                == len(invalid_rows)
                and issue_counts.get("missing_delivery", 0)
                - issue_counts.get("undelivered_lifecycle", 0)
                == len(terminal_invalid_keys)
                and path_counts.get("incomplete_delivered_packets") == len(extras)
                and path_counts.get("undelivered_packets")
                == issue_counts.get("missing_delivery", 0)
                and path_counts.get("incomplete_packets")
                == issue_counts.get("missing_delivery", 0) + len(extras)
                and path_counts.get("packet_keys")
                == path_counts.get("delivered_packets")
                + path_counts.get("undelivered_packets")
                and path_counts.get("delivered_packets")
                == path_counts.get("valid_complete_delivered_packets")
                + len(extras)
                and path_counts.get("route_context_mismatch_packets") == 0
                and path_counts.get("route_context_mismatches") == 0,
                "packet-path failure counts exceed or disagree with the DACK lineage boundary",
            )
            path_classification = {
                "classification": "known_source_proved_dack_duplicate_lineage_tool_boundary",
                "strict_result": "fail",
                "strict_exit_code": path_code,
                "release_disposition": "documented_nonblocking_scope_exclusion",
                "packet_path_integrity_claim": False,
                "invalid_packet_count": len(invalid_rows),
                "proved_extra_delivery_count": sum(extras.values()),
                "unused_lineage_count": len(terminal_invalid_keys),
                "terminal_undelivered_proofs": terminal_proofs,
                "note": "The strict analyzer failure is not relabeled as a pass. Every invalid key is one fully trace-bound repeated-DACK lineage: delivered extras exactly consume the matched proof budget, while each unused proof is a zero-delivery lineage with a valid final-bucket relay capacity-hold observation. Duplicate/branched lifecycles still bypass detailed suffix reconstruction, so this result makes no packet-path integrity or OPNET event-parity claim.",
            }

        admission_summary_digest = evidence_digests["admission_summary"]
        admission = json_load(admission_summary_path)
        require(admission.get("schema") == "csr-admission-ledger-analysis-v1", "admission schema drift")
        admission_input = admission.get("input") or {}
        require(
            admission_input.get("path") == str(trace.resolve())
            and admission_input.get("sha256") == trace_digest
            and admission_input.get("row_count") == observations["row_count"]
            and admission_input.get("event_counts")
            == dict(sorted(observations["event_counts"].items())),
            "admission input identity/count/event ledger drift",
        )
        legs_digest = evidence_digests["admission_legs"]
        leg_validation = self.validate_admission_legs(admission_legs_path)
        legs_output = (admission.get("outputs") or {}).get("legs_csv") or {}
        integrity = admission.get("integrity") or {}
        inventory = admission.get("inventory") or {}
        analyzed_hop_legs = inventory.get("hop_legs")
        open_nwk_locations = inventory.get("open_nwk_locations")
        require(
            isinstance(analyzed_hop_legs, int)
            and not isinstance(analyzed_hop_legs, bool)
            and analyzed_hop_legs >= 0
            and isinstance(open_nwk_locations, int)
            and not isinstance(open_nwk_locations, bool)
            and open_nwk_locations >= 0
            and legs_output.get("path") == str(admission_legs_path.resolve())
            and legs_output.get("sha256") == legs_digest
            and legs_output.get("row_count") == leg_validation["row_count"]
            and analyzed_hop_legs + open_nwk_locations
            == leg_validation["row_count"]
            and integrity.get("invalid_leg_count")
            == leg_validation["invalid_leg_count"],
            "admission leg artifact/summary ledger drift",
        )
        hop = admission.get("hop") or {}
        require(hop.get("dack_pre_limit_findings") == 0, "admission ledger has pre-limit DACK")
        require(hop.get("dack_boundary_count") == 0, "admission ledger has 15-to-16 DACK boundary")
        reconciliation = admission.get("application_diagnostics")
        diagnostics_digest = evidence_digests["application_diagnostics"]
        diagnostics_validation = self.validate_application_diagnostics(
            diagnostics_path
        )
        require(
            isinstance(reconciliation, dict)
            and reconciliation.get("path") == str(diagnostics_path.resolve())
            and reconciliation.get("sha256") == diagnostics_digest
            and reconciliation.get("flow_rows")
            == diagnostics_validation["row_count"]
            and reconciliation.get("totals") == diagnostics_validation["totals"]
            and reconciliation.get("mismatches") == []
            and reconciliation.get("pass") is True,
            "admission diagnostics reconciliation/artifact binding failed",
        )
        admission_classification = self.classify_admission_integrity(
            admission, admission_code, extras
        )
        admission_classification["bound_output_evidence"] = {
            "leg_rows": leg_validation["row_count"],
            "invalid_leg_rows": leg_validation["invalid_leg_count"],
            "diagnostic_flow_rows": diagnostics_validation["row_count"],
            "diagnostic_totals": diagnostics_validation["totals"],
        }
        require(
            all(
                sha256(evidence_paths[name]) == digest
                for name, digest in evidence_digests.items()
            ),
            "classification evidence changed during DACK-lineage analysis",
        )
        return {
            "classified_evidence_sha256": evidence_digests,
            "source_exact_duplicate_provenance": {
                "classification": duplicate_classification,
                "matched_duplicate_delivery_count": matched_duplicate_count,
                "repeated_feedback_proof_count": proof_count,
                "unused_proof_count": unused_proof_count,
                "actual_duplicate_packet_keys": len(extras),
                "actual_extra_deliveries": sum(extras.values()),
                "full_lineage_packet_keys": [
                    {
                        "src": key[0],
                        "dst": key[1],
                        "sequence": key[2],
                        "budget": budgets[key],
                        "disposition": (
                            "matched_duplicate_delivery"
                            if key in extras
                            else "terminal_undelivered_unused_proof"
                        ),
                    }
                    for key in sorted(budgets)
                ],
            },
            "admission_analyzer": admission_classification,
            "packet_path_analyzer": path_classification,
        }

    def final_integrity(self) -> None:
        self.stage("recheck TOCTOU, repositories, staged sources, binaries, and libraries")
        final_direct_sources = validate_named_file_set(
            self.args.opnet_source_dir.absolute(), OPNET_SOURCE_INPUTS
        )
        require(
            all(
                final_direct_sources[name]["sha256"] == digest
                for name, digest in OPNET_SOURCE_INPUTS.items()
            ),
            "direct OPNET source set changed during certification",
        )
        final_supplied_sources = validate_named_file_set(
            self.args.supplied_source_dir.absolute(),
            SUPPLIED_SOURCE_DIRECTORY_INPUTS,
            description="supplied source artifact",
            include_hidden=True,
        )
        require(
            all(
                final_supplied_sources[name]["sha256"] == digest
                for name, digest in SUPPLIED_SOURCE_DIRECTORY_INPUTS.items()
            ),
            "supplied source artifact set changed during certification",
        )
        after: dict[str, dict[str, Any]] = {}
        for label, original in self.external_originals.items():
            require(
                original.is_file() and not original.is_symlink(),
                f"input became absent, non-regular, or symlinked: {label}",
            )
            actual = sha256(original)
            before = self.external_before[label]
            require(actual == before["before_sha256"], f"input changed during run: {label}")
            staged = self.external_staged[label]
            require(
                staged.is_file()
                and not staged.is_symlink()
                and sha256(staged) == actual,
                f"staged input changed or became symlinked during run: {label}",
            )
            after[label] = {
                "original_path": str(original),
                "before_sha256": before["before_sha256"],
                "after_sha256": actual,
                "staged_sha256": sha256(staged),
                "unchanged": True,
            }
        atomic_json(self.manifests / "input-hashes.after.json", after)
        require(self.content_tree(self.rc_stage) == self.rc_content_snapshot, "immutable RC source tree changed")
        self.validate_tool_bindings()
        self.validate_source_bindings()
        current_ns3 = self.content_tree(self.ns3_stage)
        current_by_path = {record["path"]: record for record in current_ns3}
        for before in self.ns3_content_snapshot:
            require(current_by_path.get(before["path"]) == before, f"ns-3 Git source changed: {before['path']}")
        self.repository_identity(self.args.rc_repo.resolve(), self.args.rc_commit, require_origin_main=True)
        require(
            self.validate_release_harness_binding(
                self.args.rc_repo.resolve(), self.args.rc_commit
            )
            == self.harness_rc_binding_before,
            "release harness RC blob binding changed during certification",
        )
        self.repository_identity(self.args.ns3_repo.resolve(), NS3_COMMIT, require_origin_main=False)
        validate_ns3_lock_binding(
            (self.release.get("build") or {}).get("ns3_lock"), self.output
        )
        for target, binary in self.binary_paths.items():
            require(sha256(binary) == self.binary_hashes_before[target], f"binary changed after execution: {target}")
            frozen = self.output / self.release["executables"][target]["frozen_artifact"]
            require(
                sha256(frozen) == self.binary_hashes_before[target],
                f"frozen executable changed after copy: {target}",
            )
        for name, digest in self.library_hashes_before.items():
            require(sha256(Path(name)) == digest, f"loaded library changed after execution: {name}")
        for name, digest in self.tool_hashes_before.items():
            require(
                sha256(Path(name)) == digest,
                f"recorded tool executable changed after use: {name}",
            )
        linkage = json_load(self.manifests / "dynamic-linkage.json")
        library_records = linkage.get("shared_libraries")
        require(isinstance(library_records, list), "dynamic linkage library ledger is malformed")
        require(
            len(library_records) == len(self.library_hashes_before)
            and {
                str(record.get("runtime_path"))
                for record in library_records
                if isinstance(record, dict)
            }
            == set(self.library_hashes_before),
            "dynamic linkage library ledger changed after capture",
        )
        for record in library_records:
            require(isinstance(record, dict), "dynamic linkage library record is malformed")
            runtime_path = str(record.get("runtime_path"))
            require(
                record.get("sha256") == self.library_hashes_before[runtime_path],
                f"dynamic linkage digest changed after capture: {runtime_path}",
            )
            frozen = self.output / str(record.get("frozen_artifact", ""))
            require(
                frozen.is_file() and sha256(frozen) == record.get("sha256"),
                f"frozen shared-library copy changed: {frozen}",
            )
        for repository_name in ("rc", "ns3"):
            source_archive = self.artifacts / "source" / f"{repository_name}-source.tar"
            require(
                sha256(source_archive)
                == self.release["repositories"][repository_name]["source_archive_sha256"],
                f"frozen {repository_name} source archive changed",
            )
        require(
            sha256(self.harness_original) == self.harness_sha256_before,
            "certification harness changed during execution",
        )
        frozen_harness = self.output / self.release["certification_harness"]["frozen_artifact"]
        require(
            sha256(frozen_harness) == self.harness_sha256_before,
            "frozen certification harness changed after copy",
        )
        classifier_probe_record = self.release["certification_harness"][
            "classifier_probe"
        ]
        frozen_classifier_probe = self.output / classifier_probe_record[
            "frozen_artifact"
        ]
        require(
            sha256(self.classifier_probe_original)
            == self.classifier_probe_sha256_before
            and sha256(frozen_classifier_probe)
            == self.classifier_probe_sha256_before
            == classifier_probe_record["sha256"],
            "classifier self-test changed after capture",
        )
        classified_hashes = self.release["checks"]["admission_ledger"][
            "classifications"
        ]["classified_evidence_sha256"]
        classified_paths = {
            "trace": self.artifacts / "admission-ledger" / "ns3-admission-trace.csv",
            "aggregate_provenance": self.artifacts
            / "admission-ledger"
            / "ns3-aggregates.provenance.json",
            "packet_path_summary": self.artifacts
            / "admission-ledger"
            / "packet-path-summary.json",
            "packet_paths_csv": self.artifacts
            / "admission-ledger"
            / "packet-paths.csv",
            "admission_summary": self.artifacts
            / "admission-ledger"
            / "admission-summary.json",
            "admission_legs": self.artifacts
            / "admission-ledger"
            / "admission-legs.csv",
            "application_diagnostics": self.artifacts
            / "admission-ledger"
            / "app-admission-diagnostics.csv",
        }
        require(
            set(classified_hashes) == set(classified_paths)
            and all(
                sha256(classified_paths[name]) == digest
                for name, digest in classified_hashes.items()
            ),
            "classified enriched evidence changed before release sealing",
        )
        protocol = self.release["checks"]["protocol_aggregate"]
        comparator = protocol["comparator"]
        staged_comparator = Path(comparator["staged_path"])
        frozen_comparator = self.output / comparator["frozen_artifact"]
        require(
            staged_comparator.is_file()
            and not staged_comparator.is_symlink()
            and frozen_comparator.is_file()
            and not frozen_comparator.is_symlink()
            and sha256(staged_comparator)
            == sha256(frozen_comparator)
            == comparator["sha256"],
            "protocol aggregate comparator changed after use",
        )
        for label, record in protocol["artifacts"].items():
            path = self.output / record["path"]
            require(
                path.is_file()
                and not path.is_symlink()
                and sha256(path) == record["sha256"]
                and path.stat().st_size == record["size_bytes"],
                f"protocol aggregate artifact changed after use: {label}",
            )
        for record in protocol["inputs"]:
            path = self.output / record["path"]
            require(
                path.is_file()
                and not path.is_symlink()
                and sha256(path) == record["sha256"]
                and path.stat().st_size == record["size_bytes"],
                f"protocol aggregate input changed after use: {path}",
            )
        self.release["checks"]["toctou_and_immutability"] = {
            "external_inputs_unchanged": True,
            "direct_opnet_source_set_complete_and_unchanged": True,
            "supplied_source_artifact_set_complete_and_unchanged": True,
            "staged_inputs_unchanged": True,
            "rc_repository_clean_and_unchanged": True,
            "ns3_repository_clean_and_unchanged": True,
            "immutable_rc_tree_unchanged": True,
            "ns3_csr_source_bindings_unchanged": True,
            "ns3_git_source_files_unchanged": True,
            "ns3_build_lock_and_frozen_copy_unchanged": True,
            "executed_binaries_unchanged": True,
            "frozen_binary_copies_unchanged": True,
            "loaded_shared_libraries_unchanged": True,
            "recorded_tool_executables_unchanged": True,
            "pinned_tool_command_bindings_unchanged": True,
            "frozen_shared_library_copies_unchanged": True,
            "frozen_source_archives_unchanged": True,
            "certification_harness_unchanged": True,
            "certification_harness_and_classifier_match_rc_commit_blobs": True,
            "frozen_certification_harness_unchanged": True,
            "classifier_self_test_and_frozen_copy_unchanged": True,
            "classified_enriched_evidence_unchanged": True,
            "protocol_aggregate_tool_inputs_and_outputs_unchanged": True,
            "after_manifest": relative(self.manifests / "input-hashes.after.json", self.output),
        }

    def finalize(self) -> None:
        self.release.pop("current_stage", None)
        aggregate_codes = (
            self.release["checks"]["aggregate_baseline"]["workflow_exit_code"],
            self.release["checks"]["protocol_aggregate"]["comparison_exit_code"],
        )
        self.release["status"] = release_status_from_aggregate_codes(
            aggregate_codes
        )
        self.release["completed_utc"] = utc_now()
        self.release["failure"] = None
        self.checkpoint()

        expected_root_entries = {
            self.artifacts,
            self.manifests,
            self.output / "release-manifest.json",
        }
        require(
            set(self.output.iterdir()) == expected_root_entries
            and all(not path.is_symlink() for path in expected_root_entries),
            "frozen output root contains an unexpected or symlinked entry",
        )
        seal_roots = (self.artifacts, self.manifests)
        unique = enumerate_sealed_evidence(self.output, seal_roots)
        sums = self.output / "SHA256SUMS"
        sealed_hashes: dict[Path, str] = {}
        with sums.open("w", encoding="utf-8", newline="\n") as stream:
            for path in unique:
                digest = sha256(path)
                sealed_hashes[path] = digest
                stream.write(f"{digest}  {relative(path, self.output)}\n")
        require(
            all(sha256(path) == digest for path, digest in sealed_hashes.items()),
            "frozen evidence changed while SHA256SUMS was written",
        )
        sums_digest = sha256(sums)
        sums_sidecar = self.output / "SHA256SUMS.sha256"
        sidecar_text = f"{sums_digest}  SHA256SUMS\n"
        sums_sidecar.write_text(sidecar_text, encoding="utf-8")
        sums_sidecar_digest = sha256(sums_sidecar)
        require(
            sums_sidecar.is_file()
            and not sums_sidecar.is_symlink()
            and sums_sidecar.read_text(encoding="utf-8") == sidecar_text
            and sha256(sums) == sums_digest,
            "frozen evidence changed during final checksum sealing",
        )
        completed_seal_hashes = dict(sealed_hashes)
        completed_seal_hashes[sums] = sums_digest
        completed_seal_hashes[sums_sidecar] = sums_sidecar_digest
        require(
            set(self.output.iterdir())
            == expected_root_entries
            | {self.output / "SHA256SUMS", self.output / "SHA256SUMS.sha256"},
            "frozen output root changed during checksum sealing",
        )
        validate_quiescent_sealed_evidence_snapshot(
            self.output,
            seal_roots,
            completed_seal_hashes,
            additional_files=(sums, sums_sidecar),
        )
        print(f"release evidence checkpoint sealed: {self.output}", flush=True)
        print(f"SHA256SUMS sha256: {sums_digest}", flush=True)

    def execute(self) -> None:
        require(not self.output.exists(), f"output directory must not exist: {self.output}")
        for protected in (
            self.args.rc_repo.resolve(),
            self.args.ns3_repo.resolve(),
            self.args.evidence_root.resolve(),
            self.args.opnet_source_dir.resolve(),
            self.args.supplied_source_dir.resolve(),
        ):
            require(
                not is_within(self.output, protected),
                f"output directory must not be inside protected input tree {protected}",
            )
            require(
                not is_within(self.work, protected),
                f"work directory must not be inside protected input tree {protected}",
            )
        require(not self.work.exists(), f"work directory must not exist: {self.work}")
        require(
            not is_within(self.work, self.output)
            and not is_within(self.output, self.work),
            "work and frozen output directories must be disjoint",
        )
        self.output.mkdir(parents=True)
        self.artifacts.mkdir()
        self.manifests.mkdir()
        self.work.mkdir(parents=True)
        frozen_harness = self.artifacts / "tools" / "certify_release.py"
        frozen_harness.parent.mkdir(parents=True)
        shutil.copy2(self.harness_original, frozen_harness)
        require(
            sha256(frozen_harness) == self.harness_sha256_before,
            "failed to freeze certification harness identity",
        )
        frozen_classifier_probe = frozen_harness.with_name(
            self.classifier_probe_original.name
        )
        shutil.copy2(self.classifier_probe_original, frozen_classifier_probe)
        require(
            sha256(frozen_classifier_probe)
            == self.classifier_probe_sha256_before,
            "failed to freeze classifier self-test identity",
        )
        self.release["certification_harness"] = {
            "original_path": str(self.harness_original),
            "sha256": self.harness_sha256_before,
            "frozen_artifact": relative(frozen_harness, self.output),
            "classifier_probe": {
                "original_path": str(self.classifier_probe_original),
                "sha256": self.classifier_probe_sha256_before,
                "frozen_artifact": relative(frozen_classifier_probe, self.output),
            },
        }
        self.checkpoint()
        try:
            self.prepare_inputs()
            self.prepare_environment()
            self.run_classifier_self_tests()
            self.assemble_and_build()
            scenario = self.import_scenario()
            baseline = self.run_baseline(scenario)
            self.run_protocol_aggregate_comparison(baseline)
            self.run_program_workflows()
            self.run_synthetic_differential()
            self.run_python_tests()
            self.run_all_values()
            self.run_enriched_diagnostics(scenario, baseline)
            self.final_integrity()
            self.finalize()
        except (Exception, SystemExit) as error:
            self.release["status"] = "failed"
            self.release["completed_utc"] = utc_now()
            self.release["failure"] = {
                "type": type(error).__name__,
                "message": str(error),
                "stage": self.release.get("current_stage"),
            }
            self.checkpoint()
            if isinstance(error, SystemExit):
                raise CertificationError(
                    "certification component attempted to terminate the harness "
                    f"(code={error.code!r})"
                ) from error
            raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rc-repo", required=True, type=Path, help="clean CSR Git checkout at the tested commit")
    parser.add_argument("--rc-commit", required=True, help="full 40-digit tested CSR commit, equal to HEAD and origin/main")
    parser.add_argument("--ns3-repo", required=True, type=Path, help=f"clean ns-3 Git checkout at {NS3_COMMIT}")
    parser.add_argument("--source-archive", required=True, type=Path, help="CSR project examples source ZIP")
    parser.add_argument("--evidence-root", required=True, type=Path, help="directory containing multihop/ and latency/ evidence")
    parser.add_argument(
        "--opnet-source-dir",
        required=True,
        type=Path,
        help="directory containing the exact 16 directly supplied OPNET source files",
    )
    parser.add_argument(
        "--supplied-source-dir",
        required=True,
        type=Path,
        help=(
            "directory containing the exact six supplied source artifacts: the "
            "canonical source ZIP and five supplemental provenance inputs"
        ),
    )
    parser.add_argument("--cmake-bin-dir", required=True, type=Path, help="directory containing the pinned/selected cmake executable")
    parser.add_argument("--output-dir", required=True, type=Path, help="new, nonexistent certification output directory")
    parser.add_argument("--work-dir", type=Path, help="new disjoint staging directory (default: OUTPUT_DIR.work)")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--build-timeout", type=int, default=3600)
    parser.add_argument("--workflow-timeout", type=int, default=7200)
    parser.add_argument("--smoke-timeout", type=int, default=300)
    parser.add_argument("--demo-timeout", type=int, default=900)
    parser.add_argument("--python-timeout", type=int, default=1800)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    for name in (
        "jobs",
        "build_timeout",
        "workflow_timeout",
        "smoke_timeout",
        "demo_timeout",
        "python_timeout",
    ):
        if getattr(arguments, name) <= 0:
            raise SystemExit(f"error: --{name.replace('_', '-')} must be positive")
    try:
        Certification(arguments).execute()
    except CertificationError as error:
        print(f"CERTIFICATION FAILED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
