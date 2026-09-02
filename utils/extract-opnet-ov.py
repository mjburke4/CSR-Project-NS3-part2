#!/usr/bin/env python3
"""Extract historical OPNET Modeler 17.1 aggregate vectors without mutation.

The recovered CSR ``.ov`` files use the Modeler 17.1 ``MIL_3`` binary layout.
This utility decodes only the structures observed in those files and validates
every linked offset and vector extent before publishing values.  It emits the
canonical ``csr-aggregate-series-v1`` CSV consumed by the aggregate
differential harness and/or a richer JSON manifest.

An optional paired ``.pb.m`` probe-definition file supplies an independent
label and aggregation cross-check.  Files with zero-length vector records,
such as the two recovered fragments in the archive, are marked partial and do
not produce invented samples.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
import hashlib
import io
import json
import math
from pathlib import Path
import re
import struct
import sys
from typing import Any


MODEL_HEADER_SIZE = 512
MODEL_HEADER_MAGIC = b"MIL_3_Tfile_Hdr_"
AGGREGATE_SCHEMA = "csr-aggregate-series-v1"
MANIFEST_SCHEMA = "csr-opnet-ov-extract-v1"
MISSING_SENTINEL = 2.0e100
ALL_VALUES_END_SENTINEL = 4.0e100
RECOVERED_COMPLETE_BUCKET_COUNT = 100
RECOVERED_ALL_VALUES_MAX_ENCODED_COUNT = 11_019
RECOVERED_ALL_VALUES_MAX_EVENT_COUNT = 31_758
RECOVERED_MAX_DESCRIPTION_COUNT = 19
RECOVERED_MAX_DESCRIPTION_VARIANT_COUNT = 2
RECOVERED_MAX_METADATA_UNIT_COUNT = 113
RECOVERED_MAX_PB_FILE_SIZE = 15_123
RECOVERED_MAX_PB_PROBE_COUNT = 27
RECOVERED_MAX_PB_TAGGED_STRING_COUNT = 489
RECOVERED_MAX_PB_TAGGED_STRING_LENGTH = 32
RECOVERED_VECTOR_RECORD_COOKIE = bytes.fromhex("d4b249ad2594c37d")
RECOVERED_VECTOR_RESERVED = b"\0" * 8
RECOVERED_COMPLETE_VECTOR_TRAILER = bytes.fromhex("54d249ad2594c37d")
CSV_COLUMNS = (
    "schema",
    "scenario",
    "statistic",
    "time_s",
    "value",
    "source",
    "unit",
    "aggregation",
    "raw_value",
    "value_status",
    "source_file",
    "source_file_sha256",
)


class OvFormatError(ValueError):
    """The input is not a safely decodable recovered OPNET vector file."""


class Cursor:
    """Bounds-checked big-endian reader for MIL_3 metadata."""

    def __init__(self, data: bytes, offset: int = 0) -> None:
        self.data = data
        self.offset = offset

    def require(self, size: int, label: str) -> None:
        if size < 0 or self.offset + size > len(self.data):
            raise OvFormatError(
                f"truncated {label} at offset 0x{self.offset:x}: "
                f"need {size} bytes, file has {len(self.data)}"
            )

    def bytes(self, size: int, label: str) -> bytes:
        self.require(size, label)
        value = self.data[self.offset : self.offset + size]
        self.offset += size
        return value

    def u32(self, label: str) -> int:
        return struct.unpack(">I", self.bytes(4, label))[0]

    def f64(self, label: str) -> float:
        return struct.unpack(">d", self.bytes(8, label))[0]

    def string(self, label: str) -> str:
        length = self.u32(f"{label} length")
        if length > len(self.data) - self.offset:
            raise OvFormatError(
                f"invalid {label} length {length} at offset "
                f"0x{self.offset - 4:x}"
            )
        raw = self.bytes(length, label)
        try:
            return raw.decode("utf-8")
        except UnicodeDecodeError as error:
            raise OvFormatError(
                f"{label} at offset 0x{self.offset - length:x} is not UTF-8"
            ) from error


@dataclass
class Probe:
    probe_id: str
    scope: str
    name: str
    aggregation_description: str
    aggregation: str

    @property
    def statistic(self) -> str:
        return f"{self.scope}.{self.name}" if self.scope else self.name

    def as_dict(self) -> dict[str, str]:
        return {
            "probe_id": self.probe_id,
            "scope": self.scope,
            "name": self.name,
            "statistic": self.statistic,
            "aggregation_description": self.aggregation_description,
            "aggregation": self.aggregation,
        }


@dataclass
class ProbeDefinition:
    path: Path
    sha256: str
    header: str
    scenario: str
    declared_probe_count: int
    probes: list[Probe]
    warnings: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "source_file": self.path.name,
            "input_path": str(self.path.resolve()),
            "source_file_sha256": self.sha256,
            "header": self.header,
            "scenario": self.scenario,
            "declared_probe_count": self.declared_probe_count,
            "decoded_probe_count": len(self.probes),
            "probes": [probe.as_dict() for probe in self.probes],
            "warnings": self.warnings,
        }


@dataclass
class Bucket:
    index: int
    time_s: float
    raw_value: float
    value: float | None
    value_status: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "time_s": self.time_s,
            "value": self.value,
            "raw_value": self.raw_value,
            "value_status": self.value_status,
        }


@dataclass
class Vector:
    scope: str
    name: str
    aggregation_description: str
    aggregation: str
    unit: str
    description_id: int
    description: str
    data_offset: int
    expected_value_count: int | None
    metadata_details_flags: int
    status: str = "partial"
    bucket_start_s: float | None = None
    bucket_width_s: float | None = None
    buckets: list[Bucket] = field(default_factory=list)
    data_record: dict[str, Any] = field(default_factory=dict)
    probe_match: dict[str, Any] | None = None
    warnings: list[str] = field(default_factory=list)

    @property
    def statistic(self) -> str:
        return f"{self.scope}.{self.name}" if self.scope else self.name

    def as_dict(self) -> dict[str, Any]:
        first_time = self.buckets[0].time_s if self.buckets else None
        last_time = self.buckets[-1].time_s if self.buckets else None
        return {
            "scope": self.scope,
            "name": self.name,
            "statistic": self.statistic,
            "unit": self.unit,
            "aggregation": self.aggregation,
            "aggregation_description": self.aggregation_description,
            "description_id": self.description_id,
            "description": self.description,
            "data_offset": self.data_offset,
            "metadata_details_flags": f"0x{self.metadata_details_flags:x}",
            "status": self.status,
            "expected_value_count": self.expected_value_count,
            "decoded_value_count": len(self.buckets),
            "missing_value_count": sum(
                bucket.value_status == "missing" for bucket in self.buckets
            ),
            "time_axis": {
                "encoded_start_s": self.bucket_start_s,
                "bucket_width_s": self.bucket_width_s,
                "timestamp_semantics": "bucket_end",
                "first_timestamp_s": first_time,
                "last_timestamp_s": last_time,
            },
            "data_record": self.data_record,
            "probe_definition_match": self.probe_match,
            "buckets": [bucket.as_dict() for bucket in self.buckets],
            "warnings": self.warnings,
        }


@dataclass
class ExcludedNonAggregateVector:
    """A validated per-object ``All values`` vector not emitted as buckets.

    The documented recovered latency result stores six ``All values`` records
    below 0x22 hierarchy containers and uses a distinct 0x81 two-array
    encoding.  Other archived 0x224 records use bucket aggregation and 0x82,
    so classification depends on the aggregation string and record marker,
    not on 0x224 alone.  Record boundaries, sentinels, and time axes are always
    validated.  An explicit opt-in may retain the validated interior
    value/time pairs in the rich JSON manifest, but they are never published
    as aggregate samples.
    """

    hierarchy: tuple[str, ...]
    scope: str
    name: str
    description_id: int
    description: str
    data_offset: int
    metadata_details_flags: int
    data_record: dict[str, Any] = field(default_factory=dict)
    probe_match: dict[str, Any] | None = None
    events: list[dict[str, int | float]] = field(default_factory=list)

    @property
    def statistic(self) -> str:
        return f"{self.scope}.{self.name}" if self.scope else self.name

    def as_dict(self) -> dict[str, Any]:
        result = {
            "hierarchy": list(self.hierarchy),
            "scope": self.scope,
            "name": self.name,
            "statistic": self.statistic,
            "unit": _unit_from_name(self.name),
            "aggregation_description": "All values",
            "aggregation": "",
            "description_id": self.description_id,
            "description": self.description,
            "data_offset": self.data_offset,
            "metadata_details_flags": f"0x{self.metadata_details_flags:x}",
            "status": "validated_excluded_nonaggregate",
            "data_record": self.data_record,
            "probe_definition_match": self.probe_match,
            "exclusion_reason": (
                "per-object All values series uses the recovered 0x81 "
                "two-array encoding; it is excluded from canonical aggregate CSV"
            ),
        }
        if self.data_record.get("numeric_contents_emitted"):
            result["events"] = self.events
        return result


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _header_text(data: bytes, path: Path) -> str:
    if len(data) < MODEL_HEADER_SIZE:
        raise OvFormatError(
            f"{path}: file is {len(data)} bytes; MIL_3 header requires 512"
        )
    header = data[:MODEL_HEADER_SIZE]
    if not header.startswith(MODEL_HEADER_MAGIC):
        raise OvFormatError(f"{path}: missing MIL_3_Tfile_Hdr_ signature")
    return header.split(b"\0", 1)[0].decode("ascii", errors="replace").rstrip()


def _scenario_from_path(path: Path) -> str:
    stem = path.name[:-3] if path.name.lower().endswith(".ov") else path.stem
    match = re.fullmatch(r"(.+)-DES-\d+", stem)
    return match.group(1) if match else stem


def _normalized_aggregation(description: str) -> str:
    normalized = description.strip().lower().replace("_", "/")
    if "sample mean" in normalized:
        return "bucket_sample_mean"
    if "sum/time" in normalized:
        return "bucket_sum_per_second"
    if re.search(r"(?:^|[/ :])sum(?:$|[ :])", normalized):
        return "bucket_sum"
    return ""


def _unit_from_name(name: str) -> str:
    lowered = name.lower()
    if "(packets/sec)" in lowered:
        return "packets/s"
    if "(bits/sec)" in lowered:
        return "bits/s"
    if "(packets)" in lowered:
        return "packets"
    if "(bits)" in lowered:
        return "bits"
    if any(token in lowered for token in ("(seconds)", "(secs)", "(sec)")):
        return "s"
    if "(joule)" in lowered:
        return "J"
    if "ratio" in lowered:
        return "1"
    return ""


def _parse_ov_aggregation_description(description: str) -> tuple[str, int]:
    """Parse only the exact bucket grammar observed in recovered OV files."""
    match = re.fullmatch(
        r"Bucket: (sample mean|sum/time|sum): total of ([0-9]+) values",
        description,
    )
    if match is None:
        raise OvFormatError(
            f"unsupported aggregate description {description!r}; expected "
            "exact recovered grammar 'Bucket: {sample mean|sum/time|sum}: "
            "total of N values'"
        )
    aggregation = {
        "sample mean": "bucket_sample_mean",
        "sum/time": "bucket_sum_per_second",
        "sum": "bucket_sum",
    }[match.group(1)]
    digits = match.group(2)
    if len(digits) > 8:
        raise OvFormatError(
            "implausible aggregate expected-value count with "
            f"{len(digits)} decimal digits"
        )
    try:
        count = int(digits)
    except ValueError as error:
        raise OvFormatError(
            f"invalid aggregate expected-value count {digits!r}"
        ) from error
    if count != RECOVERED_COMPLETE_BUCKET_COUNT:
        raise OvFormatError(
            f"unsupported aggregate expected-value count {count}; every "
            "recovered complete 17.1 vector declares exactly "
            f"{RECOVERED_COMPLETE_BUCKET_COUNT} buckets"
        )
    return aggregation, count


def parse_probe_definition(path: Path) -> ProbeDefinition:
    """Decode labels and aggregate selectors from a paired MIL_3 ``.pb.m``."""
    file_size = path.stat().st_size
    if file_size > RECOVERED_MAX_PB_FILE_SIZE:
        raise OvFormatError(
            f"{path}: probe-definition size {file_size} exceeds the "
            f"source-observed maximum {RECOVERED_MAX_PB_FILE_SIZE}"
        )
    data = path.read_bytes()
    header = _header_text(data, path)
    header_parts = header.split()
    release_token = header_parts[1] if len(header_parts) > 1 else ""
    if release_token != "171A":
        raise OvFormatError(
            f"{path}: unsupported Modeler release token {release_token!r}; "
            "this decoder is limited to recovered 17.1 files"
        )
    cursor = Cursor(data, MODEL_HEADER_SIZE)
    scenario = cursor.string("probe-definition scenario")
    declared_count = cursor.u32("declared probe count")
    if declared_count > RECOVERED_MAX_PB_PROBE_COUNT:
        raise OvFormatError(
            f"{path}: declared probe count {declared_count} exceeds the "
            f"source-observed maximum {RECOVERED_MAX_PB_PROBE_COUNT}"
        )

    # Probe attributes are MIL_3 tagged values.  In the supplied files bit
    # 0x40 varies with serialization context, while the low six bits retain
    # string type 0x0a.  Scan only bounded, printable tagged strings and then
    # group them by their explicit pb<number> object identifier.
    tagged_strings: list[tuple[int, str]] = []
    for offset in range(cursor.offset, max(cursor.offset, len(data) - 7)):
        tag, length = struct.unpack_from(">II", data, offset)
        if (
            tag & 0x3F != 0x0A
            or length > RECOVERED_MAX_PB_TAGGED_STRING_LENGTH
            or offset + 8 + length > len(data)
        ):
            continue
        raw = data[offset + 8 : offset + 8 + length]
        if all(byte in b"\t\r\n" or 32 <= byte <= 126 for byte in raw):
            if len(tagged_strings) >= RECOVERED_MAX_PB_TAGGED_STRING_COUNT:
                raise OvFormatError(
                    f"{path}: printable tagged-string count exceeds the "
                    "source-observed maximum "
                    f"{RECOVERED_MAX_PB_TAGGED_STRING_COUNT}"
                )
            tagged_strings.append((offset, raw.decode("ascii")))

    starts = [
        index
        for index, (_, value) in enumerate(tagged_strings)
        if re.fullmatch(r"pb\d+", value)
    ]
    probes: list[Probe] = []
    probe_ids: set[str] = set()
    warnings: list[str] = []
    for position, start in enumerate(starts):
        end = starts[position + 1] if position + 1 < len(starts) else len(tagged_strings)
        values = [value for _, value in tagged_strings[start:end]]
        probe_id = values[0]
        if probe_id in probe_ids:
            raise OvFormatError(
                f"{path}: duplicate probe/vector identifier {probe_id!r}"
            )
        probe_ids.add(probe_id)
        try:
            linear_index = values.index("linear")
        except ValueError:
            warnings.append(f"{probe_id}: missing linear probe selector")
            continue
        aggregation_description = next(
            (
                value.strip()
                for value in values
                if "bucket/default total/" in value.lower()
            ),
            "",
        )
        mean_index = next(
            (
                index
                for index in range(linear_index + 1, len(values))
                if values[index].strip() == "sample mean"
            ),
            len(values),
        )
        identity_parts = [
            value.strip()
            for value in values[linear_index + 1 : mean_index]
            if value.strip()
        ]
        if len(identity_parts) >= 2:
            scope, name = identity_parts[0], identity_parts[1]
        elif len(identity_parts) == 1:
            scope, name = "", identity_parts[0]
        else:
            warnings.append(f"{probe_id}: missing statistic identity")
            continue
        probes.append(
            Probe(
                probe_id=probe_id,
                scope=scope,
                name=name,
                aggregation_description=aggregation_description,
                aggregation=_normalized_aggregation(aggregation_description),
            )
        )
    if len(probes) != declared_count:
        warnings.append(
            f"declared {declared_count} probes but decoded {len(probes)} "
            "probe identities"
        )
    return ProbeDefinition(
        path=path,
        sha256=_sha256(data),
        header=header,
        scenario=scenario,
        declared_probe_count=declared_count,
        probes=probes,
        warnings=warnings,
    )


def _match_probe_identity(
    scope: str,
    name: str,
    aggregation: str,
    definition: ProbeDefinition,
) -> dict[str, Any]:
    candidates = [
        probe
        for probe in definition.probes
        if probe.name == name and probe.aggregation == aggregation
    ]
    exact = [probe for probe in candidates if probe.scope == scope]
    if len(exact) == 1:
        return {"status": "exact", **exact[0].as_dict()}
    unscoped = [probe for probe in candidates if not probe.scope]
    if len(unscoped) == 1:
        return {
            "status": "name_and_aggregation_only",
            "note": "paired probe definition stores an empty scope",
            **unscoped[0].as_dict(),
        }
    if not candidates:
        return {"status": "unmatched"}
    return {
        "status": "ambiguous",
        "candidates": [probe.as_dict() for probe in candidates],
    }


def _match_probe(vector: Vector, definition: ProbeDefinition) -> dict[str, Any]:
    return _match_probe_identity(
        vector.scope,
        vector.name,
        vector.aggregation,
        definition,
    )


def _description_text(
    descriptions: dict[int, list[str]], description_id: int
) -> str:
    return next(
        (value for value in descriptions.get(description_id, []) if value),
        "",
    )


def _increment_metadata_count(
    *,
    path: Path,
    offset: int,
    metadata_counts: dict[str, int],
    field: str,
    declared_limit: int,
) -> None:
    """Increment one metadata record while enforcing both trusted limits."""
    metadata_counts[field] += 1
    decoded_units = (
        metadata_counts["hierarchy_containers"]
        + metadata_counts["statistic_groups"]
        + 2 * metadata_counts["statistics"]
    )
    effective_limit = min(declared_limit, RECOVERED_MAX_METADATA_UNIT_COUNT)
    if decoded_units > effective_limit:
        raise OvFormatError(
            f"{path}: decoded metadata units {decoded_units} exceed limit "
            f"{effective_limit} while parsing offset 0x{offset:x}"
        )


def _parse_statistic_group(
    *,
    data: bytes,
    path: Path,
    cursor: Cursor,
    group_offset: int,
    hierarchy: tuple[str, ...],
    aggregate: bool,
    descriptions: dict[int, list[str]],
    vectors: list[Vector],
    excluded_vectors: list[ExcludedNonAggregateVector],
    vector_description_ids: set[int],
    vector_identities: set[tuple[str, str, str]],
    excluded_identities: set[tuple[tuple[str, ...], str, str]],
    metadata_counts: dict[str, int],
    metadata_unit_limit: int,
) -> None:
    """Decode one exact recovered 0x23 statistic group."""
    _increment_metadata_count(
        path=path,
        offset=group_offset,
        metadata_counts=metadata_counts,
        field="statistic_groups",
        declared_limit=metadata_unit_limit,
    )
    scope = cursor.string("statistic scope")
    if not scope.strip():
        raise OvFormatError(
            f"{path}: empty statistic scope at offset 0x{group_offset:x}"
        )
    group_fields = [
        cursor.u32(f"statistic group field {index}") for index in range(5)
    ]
    if group_fields[:4] != [0xFFFFFFFF, 0xFFFFFFFF, 0, 0]:
        raise OvFormatError(
            f"{path}: unsupported statistic group fields at "
            f"offset 0x{group_offset:x}"
        )
    next_group_offset = group_fields[-1]
    if next_group_offset <= cursor.offset or next_group_offset > len(data):
        raise OvFormatError(
            f"{path}: invalid next group offset 0x{next_group_offset:x}"
        )

    expected_statistic_flags = 0x124 if aggregate else 0x224
    allowed_details = (0x145, 0x165) if aggregate else (0x245, 0x265)
    statistic_count = 0
    while True:
        statistic_offset = cursor.offset
        statistic_flags = cursor.u32("statistic flags")
        if statistic_flags == 0:
            break
        if statistic_flags != expected_statistic_flags:
            raise OvFormatError(
                f"{path}: unsupported statistic flags 0x{statistic_flags:x} "
                f"at offset 0x{statistic_offset:x}; expected recovered "
                f"0x{expected_statistic_flags:x} layout"
            )
        statistic_count += 1
        _increment_metadata_count(
            path=path,
            offset=statistic_offset,
            metadata_counts=metadata_counts,
            field="statistics",
            declared_limit=metadata_unit_limit,
        )
        name = cursor.string("statistic name")
        if not name.strip():
            raise OvFormatError(
                f"{path}: empty statistic name at offset "
                f"0x{statistic_offset:x}"
            )
        statistic_fields = [
            cursor.u32(f"statistic field {index}") for index in range(5)
        ]
        if statistic_fields[:4] != [0xFFFFFFFF, 0xFFFFFFFF, 0, 0]:
            raise OvFormatError(
                f"{path}: unsupported statistic fields at "
                f"offset 0x{statistic_offset:x}"
            )
        next_statistic_offset = statistic_fields[-1]
        details_flags = cursor.u32("statistic details flags")
        if details_flags not in allowed_details:
            supported = ", ".join(f"0x{value:x}" for value in allowed_details)
            raise OvFormatError(
                f"{path}: unsupported statistic details 0x{details_flags:x}; "
                f"recovered layout uses {supported}"
            )
        # Do not infer completeness from this field.  The recovered corpus
        # uses 0x145 for complete bucket vectors and 0x165 for both complete
        # and zero-count bucket vectors.  This latency file's nested
        # All-values records use 0x245/0x265, but other nested 0x224 records
        # in the archive use bucket aggregation and are not classified by
        # details flags alone.
        _filter = cursor.string("statistic filter")
        _filter_terminator = cursor.u32("statistic filter terminator")
        aggregation_description = cursor.string("aggregation description")
        description_id = cursor.u32("statistic description identifier")
        _reserved = cursor.u32("statistic reserved field")
        data_offset = cursor.u32("statistic data offset")
        _data_reserved = cursor.u32("statistic data reserved field")
        if _filter != "none":
            raise OvFormatError(
                f"{path}: unsupported statistic filter {_filter!r} at "
                f"offset 0x{statistic_offset:x}; recovered records use 'none'"
            )
        if (
            _filter_terminator != 0xFFFFFFFF
            or _reserved != 0
            or _data_reserved != 0
        ):
            raise OvFormatError(
                f"{path}: unsupported statistic terminator/reserved fields "
                f"at offset 0x{statistic_offset:x}"
            )
        if next_statistic_offset != cursor.offset:
            raise OvFormatError(
                f"{path}: statistic link 0x{next_statistic_offset:x} "
                f"does not equal decoded end 0x{cursor.offset:x}"
            )
        if description_id not in descriptions:
            raise OvFormatError(
                f"{path}: statistic {scope}.{name} references missing "
                f"description identifier {description_id}"
            )
        description = _description_text(descriptions, description_id)

        if aggregate:
            if description_id in vector_description_ids:
                raise OvFormatError(
                    f"{path}: duplicate vector description identifier "
                    f"{description_id}"
            )
            vector_description_ids.add(description_id)
            try:
                aggregation, expected_value_count = (
                    _parse_ov_aggregation_description(
                        aggregation_description
                    )
                )
            except OvFormatError as error:
                raise OvFormatError(
                    f"{path}: {error} for {scope}.{name}"
                ) from error
            vector_identity = (scope, name, aggregation)
            if vector_identity in vector_identities:
                raise OvFormatError(
                    f"{path}: duplicate canonical vector identity "
                    f"{scope}.{name} ({aggregation or 'unknown aggregation'})"
                )
            vector_identities.add(vector_identity)
            vectors.append(
                Vector(
                    scope=scope,
                    name=name,
                    aggregation_description=aggregation_description,
                    aggregation=aggregation,
                    unit=_unit_from_name(name),
                    description_id=description_id,
                    description=description,
                    data_offset=data_offset,
                    expected_value_count=expected_value_count,
                    metadata_details_flags=details_flags,
                )
            )
        else:
            if aggregation_description != "All values":
                raise OvFormatError(
                    f"{path}: nested statistic {scope}.{name} uses unsupported "
                    f"aggregation {aggregation_description!r}; recovered "
                    "latency All-values exclusion requires the exact "
                    "'All values' aggregation"
                )
            excluded_identity = (hierarchy, scope, name)
            if excluded_identity in excluded_identities:
                raise OvFormatError(
                    f"{path}: duplicate nested vector identity "
                    f"{'.'.join((*hierarchy, scope, name))}"
                )
            excluded_identities.add(excluded_identity)
            excluded_vectors.append(
                ExcludedNonAggregateVector(
                    hierarchy=hierarchy,
                    scope=scope,
                    name=name,
                    description_id=description_id,
                    description=description,
                    data_offset=data_offset,
                    metadata_details_flags=details_flags,
                )
            )

    if statistic_count == 0:
        raise OvFormatError(
            f"{path}: empty statistic group at offset 0x{group_offset:x}"
        )
    if cursor.offset != next_group_offset:
        raise OvFormatError(
            f"{path}: group link 0x{next_group_offset:x} does not equal "
            f"decoded end 0x{cursor.offset:x}"
        )


def _parse_nested_container(
    *,
    data: bytes,
    path: Path,
    cursor: Cursor,
    container_offset: int,
    hierarchy: tuple[str, ...],
    depth: int,
    descriptions: dict[int, list[str]],
    vectors: list[Vector],
    excluded_vectors: list[ExcludedNonAggregateVector],
    vector_description_ids: set[int],
    vector_identities: set[tuple[str, str, str]],
    excluded_identities: set[tuple[tuple[str, ...], str, str]],
    metadata_counts: dict[str, int],
    metadata_unit_limit: int,
) -> None:
    """Decode the exact 0x22 network -> node hierarchy in the latency OV."""
    if depth > 1:
        raise OvFormatError(
            f"{path}: unsupported nested hierarchy depth at "
            f"offset 0x{container_offset:x}"
        )
    _increment_metadata_count(
        path=path,
        offset=container_offset,
        metadata_counts=metadata_counts,
        field="hierarchy_containers",
        declared_limit=metadata_unit_limit,
    )
    name = cursor.string("nested hierarchy name")
    if not name:
        raise OvFormatError(
            f"{path}: empty nested hierarchy name at offset 0x{container_offset:x}"
        )
    container_fields = [
        cursor.u32(f"nested hierarchy field {index}") for index in range(5)
    ]
    if container_fields[:4] != [0xFFFFFFFF, 0xFFFFFFFF, 0, 0]:
        raise OvFormatError(
            f"{path}: unsupported nested hierarchy fields at "
            f"offset 0x{container_offset:x}"
        )
    next_container_offset = container_fields[-1]
    if next_container_offset <= cursor.offset or next_container_offset > len(data):
        raise OvFormatError(
            f"{path}: invalid nested hierarchy link "
            f"0x{next_container_offset:x}"
        )

    nested_hierarchy = (*hierarchy, name)
    child_count = 0
    while True:
        child_offset = cursor.offset
        child_flags = cursor.u32("nested hierarchy child flags")
        if child_flags == 0:
            break
        child_count += 1
        if depth == 0:
            if child_flags != 0x22:
                raise OvFormatError(
                    f"{path}: unsupported nested hierarchy child flags "
                    f"0x{child_flags:x} at offset 0x{child_offset:x}; "
                    "recovered network containers contain 0x22 node containers"
                )
            _parse_nested_container(
                data=data,
                path=path,
                cursor=cursor,
                container_offset=child_offset,
                hierarchy=nested_hierarchy,
                depth=1,
                descriptions=descriptions,
                vectors=vectors,
                excluded_vectors=excluded_vectors,
                vector_description_ids=vector_description_ids,
                vector_identities=vector_identities,
                excluded_identities=excluded_identities,
                metadata_counts=metadata_counts,
                metadata_unit_limit=metadata_unit_limit,
            )
        else:
            if child_flags != 0x23:
                raise OvFormatError(
                    f"{path}: unsupported node child flags 0x{child_flags:x} "
                    f"at offset 0x{child_offset:x}; recovered node containers "
                    "contain 0x23 statistic groups"
                )
            _parse_statistic_group(
                data=data,
                path=path,
                cursor=cursor,
                group_offset=child_offset,
                hierarchy=nested_hierarchy,
                aggregate=False,
                descriptions=descriptions,
                vectors=vectors,
                excluded_vectors=excluded_vectors,
                vector_description_ids=vector_description_ids,
                vector_identities=vector_identities,
                excluded_identities=excluded_identities,
                metadata_counts=metadata_counts,
                metadata_unit_limit=metadata_unit_limit,
            )
    if child_count == 0:
        raise OvFormatError(
            f"{path}: empty nested hierarchy container at "
            f"offset 0x{container_offset:x}"
        )
    if cursor.offset != next_container_offset:
        raise OvFormatError(
            f"{path}: nested hierarchy link 0x{next_container_offset:x} "
            f"does not equal decoded end 0x{cursor.offset:x}"
        )


def _validate_excluded_nonaggregate_block(
    data: bytes,
    path: Path,
    vector: ExcludedNonAggregateVector,
    block_limit: int,
    duration_s: float | None,
) -> None:
    """Validate, but do not publish, a latency-file 0x81 All-values block."""
    offset = vector.data_offset
    if offset < 0 or offset + 25 > len(data) or offset >= block_limit:
        raise OvFormatError(
            f"{path}: nested vector offset 0x{offset:x} is outside its block"
        )
    cursor = Cursor(data, offset)
    marker = cursor.bytes(1, "nested vector record marker")[0]
    encoding_tag = cursor.bytes(4, "nested vector encoding tag").hex()
    record_cookie = cursor.bytes(8, "nested vector record cookie")
    reserved = cursor.bytes(8, "nested vector reserved field")
    encoded_count = cursor.u32("nested vector encoded count")
    if marker != 0x81:
        raise OvFormatError(
            f"{path}: unsupported nested vector marker 0x{marker:02x} at "
            f"offset 0x{offset:x}; recovered All-values records use 0x81"
        )
    if encoding_tag != "00000100":
        raise OvFormatError(
            f"{path}: unsupported nested vector encoding tag {encoding_tag}"
        )
    if record_cookie != RECOVERED_VECTOR_RECORD_COOKIE:
        raise OvFormatError(
            f"{path}: unexpected nested vector record cookie "
            f"{record_cookie.hex()} at offset 0x{offset + 5:x}"
        )
    if reserved != RECOVERED_VECTOR_RESERVED:
        raise OvFormatError(
            f"{path}: nonzero nested vector reserved field {reserved.hex()} "
            f"at offset 0x{offset + 13:x}"
        )
    if encoded_count < 2:
        raise OvFormatError(
            f"{path}: implausible nested vector encoded count {encoded_count}; "
            "recovered All-values records include two boundary entries"
        )
    available_payload_bytes = block_limit - cursor.offset
    max_count_from_extent = available_payload_bytes // 16
    if encoded_count > max_count_from_extent:
        raise OvFormatError(
            f"{path}: nested vector count {encoded_count} exceeds the "
            f"block-derived maximum {max_count_from_extent}"
        )
    if encoded_count > RECOVERED_ALL_VALUES_MAX_ENCODED_COUNT:
        raise OvFormatError(
            f"{path}: nested vector encoded count {encoded_count} exceeds "
            "the largest source-observed latency record "
            f"({RECOVERED_ALL_VALUES_MAX_ENCODED_COUNT})"
        )
    expected_end = cursor.offset + encoded_count * 16
    if expected_end != block_limit:
        raise OvFormatError(
            f"{path}: nested vector extent mismatch: encoded block ends at "
            f"0x{expected_end:x}, next record/file boundary is "
            f"0x{block_limit:x}"
        )
    values_offset = cursor.offset
    times_offset = values_offset + encoded_count * 8
    first_value = struct.unpack_from(">d", data, values_offset)[0]
    last_value = struct.unpack_from(
        ">d", data, values_offset + (encoded_count - 1) * 8
    )[0]
    if (
        first_value != MISSING_SENTINEL
        or last_value != ALL_VALUES_END_SENTINEL
    ):
        raise OvFormatError(
            f"{path}: nested All-values boundary sentinels are "
            f"{first_value:.17g}, {last_value:.17g}; expected "
            f"{MISSING_SENTINEL:.17g}, {ALL_VALUES_END_SENTINEL:.17g}"
        )
    for index in range(encoded_count):
        value = struct.unpack_from(">d", data, values_offset + index * 8)[0]
        if not math.isfinite(value):
            raise OvFormatError(
                f"{path}: nested All-values value {index} is not finite"
            )
        if (
            0 < index < encoded_count - 1
            and value in (MISSING_SENTINEL, ALL_VALUES_END_SENTINEL)
        ):
            raise OvFormatError(
                f"{path}: nested All-values interior value {index} reuses "
                f"boundary sentinel {value:.17g}"
            )
    if duration_s is None:
        raise OvFormatError(
            f"{path}: cannot validate nested All-values record without "
            "Execution.simulated duration (sec)"
        )
    first_time = struct.unpack_from(">d", data, times_offset)[0]
    if first_time != 0.0:
        raise OvFormatError(
            f"{path}: nested All-values time axis starts at {first_time:.17g}; "
            "recovered records start at 0"
        )
    previous_time: float | None = None
    last_time = first_time
    for index in range(encoded_count):
        time = struct.unpack_from(">d", data, times_offset + index * 8)[0]
        if not math.isfinite(time):
            raise OvFormatError(
                f"{path}: nested All-values time {index} is not finite"
            )
        if time < 0.0 or time > duration_s:
            raise OvFormatError(
                f"{path}: nested All-values time {time:.17g} lies outside "
                f"[0, {duration_s:.17g}]"
            )
        if previous_time is not None and time < previous_time:
            raise OvFormatError(
                f"{path}: nested All-values time axis is not nondecreasing"
            )
        previous_time = time
        last_time = time
    if not math.isclose(last_time, duration_s, rel_tol=1e-12, abs_tol=1e-9):
        raise OvFormatError(
            f"{path}: nested All-values time axis ends at {last_time:.17g}; "
            f"execution duration is {duration_s:.17g}"
        )
    vector.data_record = {
        "marker": f"0x{marker:02x}",
        "encoding_tag": encoding_tag,
        "record_cookie": record_cookie.hex(),
        "reserved": reserved.hex(),
        "encoded_count": encoded_count,
        "event_value_count": encoded_count - 2,
        "block_size": block_limit - offset,
        "layout": "value_array_then_time_array_with_boundary_entries",
        "event_semantics": "per_statistic_write_record_not_packet_trace",
        "numeric_contents_emitted": False,
    }


def _emit_excluded_nonaggregate_events(
    data: bytes,
    path: Path,
    vectors: list[ExcludedNonAggregateVector],
) -> int:
    """Retain already-validated 0x81 interior pairs under a corpus bound."""
    event_count = sum(
        int(vector.data_record["event_value_count"]) for vector in vectors
    )
    if event_count > RECOVERED_ALL_VALUES_MAX_EVENT_COUNT:
        raise OvFormatError(
            f"{path}: nested All-values event count {event_count} exceeds "
            "the source-observed total "
            f"({RECOVERED_ALL_VALUES_MAX_EVENT_COUNT})"
        )

    # The validation pass has already checked every extent, boundary sentinel,
    # numeric value, and timestamp.  Allocate only after the aggregate corpus
    # bound passes, omit the two structural boundary entries, and retain equal
    # timestamps and encoded order exactly as recorded.
    for vector in vectors:
        encoded_count = int(vector.data_record["encoded_count"])
        values_offset = vector.data_offset + 25
        times_offset = values_offset + encoded_count * 8
        vector.events = [
            {
                "record_index": index - 1,
                "time_s": struct.unpack_from(
                    ">d", data, times_offset + index * 8
                )[0],
                "value": struct.unpack_from(
                    ">d", data, values_offset + index * 8
                )[0],
            }
            for index in range(1, encoded_count - 1)
        ]
        vector.data_record["numeric_contents_emitted"] = True
    return event_count


def _parse_vector_block(
    data: bytes,
    vector: Vector,
    block_limit: int,
    duration_s: float | None,
) -> None:
    offset = vector.data_offset
    if offset < 0 or offset + 25 > len(data) or offset >= block_limit:
        vector.warnings.append(
            f"data record offset 0x{offset:x} is outside the available file"
        )
        return
    cursor = Cursor(data, offset)
    marker = cursor.bytes(1, "vector record marker")[0]
    encoding_tag = cursor.bytes(4, "vector encoding tag").hex()
    record_cookie = cursor.bytes(8, "vector record cookie")
    reserved = cursor.bytes(8, "vector reserved field")
    encoded_count = cursor.u32("vector encoded count")
    vector.data_record = {
        "marker": f"0x{marker:02x}",
        "encoding_tag": encoding_tag,
        "record_cookie": record_cookie.hex(),
        "reserved": reserved.hex(),
        "encoded_count": encoded_count,
        "block_size": block_limit - offset,
    }
    if marker != 0x82:
        vector.warnings.append(f"unexpected vector marker 0x{marker:02x}")
        return
    if encoding_tag != "00000100":
        vector.warnings.append(f"unsupported vector encoding tag {encoding_tag}")
        return
    if record_cookie != RECOVERED_VECTOR_RECORD_COOKIE:
        raise OvFormatError(
            f"unexpected vector record cookie {record_cookie.hex()} at "
            f"offset 0x{offset + 5:x}; recovered complete records use "
            f"{RECOVERED_VECTOR_RECORD_COOKIE.hex()}"
        )
    if reserved != RECOVERED_VECTOR_RESERVED:
        raise OvFormatError(
            f"nonzero vector reserved field {reserved.hex()} at "
            f"offset 0x{offset + 13:x}"
        )
    if encoded_count == 0:
        if vector.metadata_details_flags != 0x165:
            raise OvFormatError(
                "zero-count vector record uses details flags "
                f"0x{vector.metadata_details_flags:x}; recovered zero-count "
                "diagnostics use 0x165"
            )
        vector.warnings.append(
            f"vector record contains no time axis or samples (encoded count "
            f"{encoded_count})"
        )
        return
    if encoded_count == 1:
        raise OvFormatError(
            "unsupported vector record with a time axis but no samples "
            "(encoded count 1)"
        )
    expected_encoded_count = RECOVERED_COMPLETE_BUCKET_COUNT + 1
    if encoded_count != expected_encoded_count:
        raise OvFormatError(
            f"aggregate encoded count {encoded_count} does not equal the "
            f"source-observed complete count {expected_encoded_count}"
        )
    if duration_s is None:
        raise OvFormatError(
            "cannot validate a complete vector without "
            "Execution.simulated duration (sec)"
        )

    double_count = encoded_count + 1
    expected_end = cursor.offset + double_count * 8 + 8
    if expected_end != block_limit:
        vector.warnings.append(
            f"vector extent mismatch: encoded block ends at 0x{expected_end:x}, "
            f"next record/file boundary is 0x{block_limit:x}"
        )
        return
    values = [cursor.f64("vector numeric value") for _ in range(double_count)]
    trailer = cursor.bytes(8, "vector record trailer")
    vector.data_record["trailer"] = trailer.hex()
    # Every complete record in the recovered 17.1 corpus uses this exact
    # trailer.  The zero-count fragments have a different incomplete suffix,
    # so no broader cookie-to-trailer transformation is inferred here.
    if trailer != RECOVERED_COMPLETE_VECTOR_TRAILER:
        raise OvFormatError(
            f"unexpected complete vector trailer {trailer.hex()} at "
            f"offset 0x{cursor.offset - 8:x}; expected "
            f"{RECOVERED_COMPLETE_VECTOR_TRAILER.hex()}"
        )
    start_s, width_s, *raw_values = values
    if not math.isfinite(start_s) or not math.isfinite(width_s) or width_s <= 0.0:
        vector.warnings.append(
            f"invalid compressed time axis start={start_s!r}, width={width_s!r}"
        )
        return
    if start_s != 0.0:
        raise OvFormatError(
            f"unsupported compressed time axis start {start_s:.17g}; "
            "every recovered complete 17.1 vector starts at 0"
        )
    expected_last = start_s + RECOVERED_COMPLETE_BUCKET_COUNT * width_s
    if not math.isfinite(expected_last):
        raise OvFormatError(
            "reconstructed aggregate bucket end is not finite"
        )
    if not math.isclose(expected_last, duration_s, rel_tol=1e-12, abs_tol=1e-9):
        vector.warnings.append(
            f"last bucket end {expected_last:.17g} does not match "
            f"simulated duration {duration_s:.17g}"
        )
        return
    vector.bucket_start_s = start_s
    vector.bucket_width_s = width_s
    for index, raw_value in enumerate(raw_values, start=1):
        if not math.isfinite(raw_value):
            vector.warnings.append(
                f"bucket {index} contains non-finite value {raw_value!r}"
            )
            vector.buckets.clear()
            return
        if raw_value == ALL_VALUES_END_SENTINEL:
            raise OvFormatError(
                f"aggregate bucket {index} reuses structural end sentinel "
                f"{ALL_VALUES_END_SENTINEL:.17g}"
            )
        missing = raw_value == MISSING_SENTINEL
        time_s = start_s + index * width_s
        if not math.isfinite(time_s):
            raise OvFormatError(
                f"reconstructed aggregate bucket {index} time is not finite"
            )
        vector.buckets.append(
            Bucket(
                index=index,
                time_s=time_s,
                raw_value=raw_value,
                value=None if missing else raw_value,
                value_status="missing" if missing else "observed",
            )
        )

    if (
        vector.expected_value_count is not None
        and len(vector.buckets) != vector.expected_value_count
    ):
        vector.warnings.append(
            f"decoded {len(vector.buckets)} buckets, metadata declares "
            f"{vector.expected_value_count}"
        )
        return
    vector.status = "complete"


def extract_ov(
    path: Path,
    *,
    scenario: str | None = None,
    probe_definition: Path | None = None,
    include_all_values: bool = False,
) -> dict[str, Any]:
    """Decode one supplied-layout ``.ov`` into a canonical JSON-ready object.

    ``include_all_values`` retains validated 0x81 event pairs in the rich
    object only.  It never converts them into canonical aggregate buckets.
    """
    data = path.read_bytes()
    header = _header_text(data, path)
    header_parts = header.split()
    release_token = header_parts[1] if len(header_parts) > 1 else ""
    if release_token != "171A":
        raise OvFormatError(
            f"{path}: unsupported Modeler release token {release_token!r}; "
            "this decoder is limited to recovered 17.1 files"
        )
    digest = _sha256(data)
    cursor = Cursor(data, MODEL_HEADER_SIZE)
    if cursor.u32("post-header reserved field") != 0:
        raise OvFormatError(f"{path}: unsupported nonzero post-header field")
    execution_date = cursor.string("execution date")
    execution_time = cursor.string("execution time")
    scalar_count = cursor.u32("execution scalar count")
    if scalar_count > 1024:
        raise OvFormatError(f"{path}: implausible scalar count {scalar_count}")
    scalar_records: list[dict[str, Any]] = []
    scalars: dict[str, float] = {}
    for index in range(scalar_count):
        name = cursor.string(f"execution scalar {index} name")
        value_count = cursor.u32(f"execution scalar {name} value count")
        value = cursor.f64(f"execution scalar {name} value")
        terminator = cursor.u32(f"execution scalar {name} terminator")
        if value_count != 1 or terminator != 0xFFFFFFFF:
            raise OvFormatError(
                f"{path}: unsupported scalar record for {name!r}: "
                f"count={value_count}, terminator=0x{terminator:08x}"
            )
        if not math.isfinite(value):
            raise OvFormatError(f"{path}: scalar {name!r} is not finite")
        if name in scalars:
            raise OvFormatError(
                f"{path}: duplicate execution scalar identifier {name!r}"
            )
        scalars[name] = value
        scalar_records.append({"name": name, "value": value})

    if cursor.u32("description-section reserved field") != 0:
        raise OvFormatError(f"{path}: unsupported description-section field")
    description_count = cursor.u32("description count")
    if description_count > RECOVERED_MAX_DESCRIPTION_COUNT:
        raise OvFormatError(
            f"{path}: description count {description_count} exceeds the "
            f"source-observed maximum {RECOVERED_MAX_DESCRIPTION_COUNT}"
        )
    descriptions: dict[int, list[str]] = {}
    for index in range(description_count):
        identifier = cursor.u32(f"description {index} identifier")
        if identifier in descriptions:
            raise OvFormatError(
                f"{path}: duplicate description identifier {identifier}"
            )
        variant_count = cursor.u32(f"description {identifier} variant count")
        if variant_count > RECOVERED_MAX_DESCRIPTION_VARIANT_COUNT:
            raise OvFormatError(
                f"{path}: description {identifier} variant count "
                f"{variant_count} exceeds the source-observed maximum "
                f"{RECOVERED_MAX_DESCRIPTION_VARIANT_COUNT}"
            )
        descriptions[identifier] = [
            cursor.string(f"description {identifier} variant {variant}")
            for variant in range(variant_count)
        ]

    declared_metadata_unit_count = cursor.u32("declared vector metadata units")
    if (
        declared_metadata_unit_count == 0
        or declared_metadata_unit_count > RECOVERED_MAX_METADATA_UNIT_COUNT
    ):
        raise OvFormatError(
            f"{path}: implausible declared vector metadata unit count "
            f"{declared_metadata_unit_count}; source-observed range is "
            f"1..{RECOVERED_MAX_METADATA_UNIT_COUNT}"
        )
    vectors: list[Vector] = []
    excluded_vectors: list[ExcludedNonAggregateVector] = []
    vector_description_ids: set[int] = set()
    vector_identities: set[tuple[str, str, str]] = set()
    excluded_identities: set[tuple[tuple[str, ...], str, str]] = set()
    metadata_counts = {
        "hierarchy_containers": 0,
        "statistic_groups": 0,
        "statistics": 0,
    }
    saw_nested_hierarchy = False
    while True:
        item_offset = cursor.offset
        item_flags = cursor.u32("vector metadata item flags")
        if item_flags == 0:
            break
        if item_flags == 0x23:
            if saw_nested_hierarchy:
                raise OvFormatError(
                    f"{path}: top-level statistic group follows nested "
                    f"hierarchy at offset 0x{item_offset:x}"
                )
            _parse_statistic_group(
                data=data,
                path=path,
                cursor=cursor,
                group_offset=item_offset,
                hierarchy=(),
                aggregate=True,
                descriptions=descriptions,
                vectors=vectors,
                excluded_vectors=excluded_vectors,
                vector_description_ids=vector_description_ids,
                vector_identities=vector_identities,
                excluded_identities=excluded_identities,
                metadata_counts=metadata_counts,
                metadata_unit_limit=declared_metadata_unit_count,
            )
            continue
        if item_flags == 0x22:
            if saw_nested_hierarchy:
                raise OvFormatError(
                    f"{path}: multiple top-level nested hierarchy containers"
                )
            saw_nested_hierarchy = True
            _parse_nested_container(
                data=data,
                path=path,
                cursor=cursor,
                container_offset=item_offset,
                hierarchy=(),
                depth=0,
                descriptions=descriptions,
                vectors=vectors,
                excluded_vectors=excluded_vectors,
                vector_description_ids=vector_description_ids,
                vector_identities=vector_identities,
                excluded_identities=excluded_identities,
                metadata_counts=metadata_counts,
                metadata_unit_limit=declared_metadata_unit_count,
            )
            continue
        raise OvFormatError(
            f"{path}: unsupported vector metadata item flags "
            f"0x{item_flags:x} at offset 0x{item_offset:x}"
        )

    decoded_metadata_unit_count = (
        metadata_counts["hierarchy_containers"]
        + metadata_counts["statistic_groups"]
        + 2 * metadata_counts["statistics"]
    )
    if decoded_metadata_unit_count != declared_metadata_unit_count:
        raise OvFormatError(
            f"{path}: declared vector metadata unit count "
            f"{declared_metadata_unit_count} does not equal decoded count "
            f"{decoded_metadata_unit_count} "
            "(hierarchy containers + statistic groups + 2*statistics)"
        )
    if saw_nested_hierarchy and not excluded_vectors:
        raise OvFormatError(
            f"{path}: nested hierarchy contains no supported All-values records"
        )

    metadata_end = cursor.offset
    all_records: list[Vector | ExcludedNonAggregateVector] = [
        *vectors,
        *excluded_vectors,
    ]
    offsets = [vector.data_offset for vector in all_records]
    if offsets != sorted(offsets) or len(set(offsets)) != len(offsets):
        raise OvFormatError(
            f"{path}: vector data offsets are not unique and ordered"
        )
    if all_records and offsets[0] != metadata_end:
        raise OvFormatError(
            f"{path}: metadata ends at 0x{metadata_end:x}, first vector starts "
            f"at 0x{offsets[0]:x}"
        )
    duration_s = scalars.get("Execution.simulated duration (sec)")
    if duration_s is not None and duration_s <= 0.0:
        raise OvFormatError(
            f"{path}: Execution.simulated duration (sec) must be positive; "
            f"decoded {duration_s:.17g}"
        )
    for index, vector in enumerate(all_records):
        block_limit = (
            offsets[index + 1] if index + 1 < len(offsets) else len(data)
        )
        if isinstance(vector, Vector):
            _parse_vector_block(data, vector, block_limit, duration_s)
        else:
            _validate_excluded_nonaggregate_block(
                data,
                path,
                vector,
                block_limit,
                duration_s,
            )

    validated_nonaggregate_event_count = sum(
        int(vector.data_record["event_value_count"])
        for vector in excluded_vectors
    )
    emitted_nonaggregate_event_count = 0
    if include_all_values:
        emitted_nonaggregate_event_count = _emit_excluded_nonaggregate_events(
            data,
            path,
            excluded_vectors,
        )

    parsed_probe: ProbeDefinition | None = None
    probe_warning: str | None = None
    resolved_scenario = scenario or _scenario_from_path(path)
    if probe_definition is not None:
        parsed_probe = parse_probe_definition(probe_definition)
        if parsed_probe.scenario != resolved_scenario:
            probe_warning = (
                f"probe-definition scenario {parsed_probe.scenario!r} does not "
                f"match vector scenario {resolved_scenario!r}; labels were not "
                "cross-matched"
            )
            for vector in vectors:
                vector.probe_match = {"status": "scenario_mismatch"}
            for vector in excluded_vectors:
                vector.probe_match = {"status": "scenario_mismatch"}
        else:
            for vector in vectors:
                vector.probe_match = _match_probe(vector, parsed_probe)
            for vector in excluded_vectors:
                vector.probe_match = _match_probe_identity(
                    vector.scope,
                    vector.name,
                    "",
                    parsed_probe,
                )

    complete_vectors = sum(vector.status == "complete" for vector in vectors)
    status = (
        "complete"
        if vectors
        and complete_vectors == len(vectors)
        and duration_s is not None
        else "partial"
    )
    warnings: list[str] = []
    if not vectors:
        warnings.append("no vector metadata records were present")
    if duration_s is None:
        warnings.append("Execution.simulated duration (sec) is absent")
    if complete_vectors != len(vectors):
        warnings.append(
            f"{len(vectors) - complete_vectors} of {len(vectors)} vectors are partial"
        )
    if probe_warning:
        warnings.append(probe_warning)

    return {
        "schema": MANIFEST_SCHEMA,
        "status": status,
        "scenario": resolved_scenario,
        "source": "opnet",
        "provenance": {
            "source_file": path.name,
            "input_path": str(path.resolve()),
            "source_file_sha256": digest,
            "file_size_bytes": len(data),
        },
        "format": {
            "header": header,
            "release_token": release_token,
            "modeler_release": "17.1" if release_token == "171A" else None,
            "declared_metadata_unit_count": declared_metadata_unit_count,
            "decoded_metadata_unit_count": decoded_metadata_unit_count,
            "metadata_unit_formula": (
                "hierarchy_containers + statistic_groups + 2*statistics"
            ),
            "metadata_counts": metadata_counts,
            "metadata_end_offset": metadata_end,
        },
        "execution": {
            "date": execution_date,
            "time": execution_time,
            "duration_s": duration_s,
            "seed": scalars.get("Execution.seed"),
            "scalars": scalar_records,
        },
        "vector_completeness": {
            "status": status,
            "vector_count": len(vectors),
            "complete_vector_count": complete_vectors,
            "partial_vector_count": len(vectors) - complete_vectors,
            "validated_excluded_nonaggregate_vector_count": len(
                excluded_vectors
            ),
            "validated_nonaggregate_event_count": (
                validated_nonaggregate_event_count
            ),
            "emitted_nonaggregate_event_count": (
                emitted_nonaggregate_event_count
            ),
            "decoded_bucket_count": sum(len(vector.buckets) for vector in vectors),
            "missing_bucket_count": sum(
                bucket.value_status == "missing"
                for vector in vectors
                for bucket in vector.buckets
            ),
        },
        "descriptions": [
            {"id": identifier, "variants": variants}
            for identifier, variants in sorted(descriptions.items())
        ],
        "vectors": [vector.as_dict() for vector in vectors],
        "excluded_nonaggregate_vectors": [
            vector.as_dict() for vector in excluded_vectors
        ],
        "probe_definition": parsed_probe.as_dict() if parsed_probe else None,
        "decoder": {
            "layout_confidence": "high_for_supplied_modeler_17_1_files",
            "semantic_confidence": {
                "execution_scalars": "high",
                "statistic_identity": "high",
                "aggregation": "high",
                "numeric_values": "high",
                "all_values_value_time_pairing": "high",
                "bucket_width": "high",
                "bucket_end_timestamps": "medium",
                "missing_value_sentinel": "medium",
            },
            "limitations": [
                "This is a conservative decoder for the recovered MIL_3 17.1 "
                "layout, not a general parser for every OPNET release.",
                "Bucket-end timestamps are reconstructed from the encoded "
                "start/increment pair and checked against simulated duration.",
                "The repeated 2e+100 value in empty sample-mean buckets is "
                "treated as Modeler's missing-value sentinel and preserved as "
                "raw_value.",
                "Unknown tags, broken linked offsets, non-finite values, and "
                "truncated metadata are rejected instead of guessed.",
                "The documented latency result's nested All-values 0x81 "
                "two-array records are structurally and temporally validated. "
                "Their interior event pairs are retained in rich JSON only "
                "when explicitly requested and remain excluded from canonical "
                "aggregate output.",
                "All-values pairs are per-statistic write records, not packet "
                "traces. Duplicate timestamps and record order are preserved, "
                "but records from different statistics cannot be joined as "
                "packets without an independent identifier.",
                "Nested 0x224 records are not assumed to be All-values: this "
                "classification also requires the exact aggregation string "
                "and 0x81 record marker observed in the latency result.",
                "Statistic details flag 0x165 is not a completeness marker: "
                "the recovered corpus contains both a complete 0x165 battery "
                "vector and zero-count 0x165 fragment records.",
            ],
        },
        "warnings": warnings,
    }


def aggregate_rows(manifest: dict[str, Any]) -> list[dict[str, str]]:
    """Flatten a complete extraction, including canonical missing rows.

    A partial extraction may retain decoded buckets in its JSON diagnostics
    (for example, when a later vector or duration check fails).  Those samples
    are never emitted as canonical comparison input.
    """
    if manifest.get("status") != "complete":
        return []
    rows: list[dict[str, str]] = []
    provenance = manifest["provenance"]
    for vector in manifest["vectors"]:
        for bucket in vector["buckets"]:
            value = bucket["value"]
            rows.append(
                {
                    "schema": AGGREGATE_SCHEMA,
                    "scenario": manifest["scenario"],
                    "statistic": vector["statistic"],
                    "time_s": _format_number(bucket["time_s"]),
                    "value": "" if value is None else _format_number(value),
                    "source": "opnet",
                    "unit": vector["unit"],
                    "aggregation": vector["aggregation"],
                    "raw_value": _format_number(bucket["raw_value"]),
                    "value_status": bucket["value_status"],
                    "source_file": provenance["source_file"],
                    "source_file_sha256": provenance["source_file_sha256"],
                }
            )
    return rows


def _format_number(value: float) -> str:
    return format(value, ".17g")


def write_csv(manifest: dict[str, Any], stream: io.TextIOBase) -> None:
    """Write canonical aggregate rows to a text stream."""
    writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(aggregate_rows(manifest))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="historical Modeler .ov file")
    parser.add_argument(
        "--probe-definition",
        type=Path,
        metavar="FILE.pb.m",
        help="optional paired probe-definition file used to cross-check labels",
    )
    parser.add_argument(
        "--scenario",
        help="canonical scenario name (default: strip -DES-N from input name)",
    )
    parser.add_argument("--json", type=Path, help="write rich JSON manifest")
    parser.add_argument("--csv", type=Path, help="write canonical aggregate CSV")
    parser.add_argument(
        "--include-all-values",
        action="store_true",
        help=(
            "retain validated nested 0x81 All-values event pairs in rich "
            "JSON output; canonical aggregate CSV is unchanged"
        ),
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return status 3 after writing outputs when the input is partial",
    )
    return parser


def _path_identity(path: Path, label: str) -> tuple[Path, tuple[int, int] | None]:
    """Resolve one CLI path without opening it and retain any inode identity."""
    try:
        resolved = path.resolve(strict=False)
        metadata = path.stat()
    except FileNotFoundError:
        metadata = None
    except (OSError, RuntimeError) as error:
        raise OvFormatError(f"cannot resolve {label} {path}: {error}") from error
    inode = None if metadata is None else (metadata.st_dev, metadata.st_ino)
    return resolved, inode


def _identities_alias(
    left: tuple[Path, tuple[int, int] | None],
    right: tuple[Path, tuple[int, int] | None],
) -> bool:
    return left[0] == right[0] or (
        left[1] is not None and left[1] == right[1]
    )


def _validate_output_collisions(arguments: argparse.Namespace) -> None:
    """Reject output aliases before extraction opens or writes any file."""
    inputs: list[tuple[str, Path]] = [("input", arguments.input)]
    if arguments.probe_definition is not None:
        inputs.append(("probe definition", arguments.probe_definition))
    outputs: list[tuple[str, Path]] = []
    if arguments.json is not None:
        outputs.append(("JSON output", arguments.json))
    if arguments.csv is not None:
        outputs.append(("CSV output", arguments.csv))

    identities = {
        (kind, str(path)): _path_identity(path, kind)
        for kind, path in (*inputs, *outputs)
    }
    for output_kind, output_path in outputs:
        output_identity = identities[(output_kind, str(output_path))]
        for input_kind, input_path in inputs:
            input_identity = identities[(input_kind, str(input_path))]
            if _identities_alias(output_identity, input_identity):
                raise OvFormatError(
                    f"{output_kind} {output_path} aliases {input_kind} "
                    f"{input_path}; refusing to overwrite an input"
                )

    if len(outputs) == 2:
        first_kind, first_path = outputs[0]
        second_kind, second_path = outputs[1]
        if _identities_alias(
            identities[(first_kind, str(first_path))],
            identities[(second_kind, str(second_path))],
        ):
            raise OvFormatError(
                f"{first_kind} {first_path} aliases {second_kind} "
                f"{second_path}; outputs must be distinct"
            )


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        _validate_output_collisions(arguments)
        if (
            arguments.include_all_values
            and arguments.csv is not None
            and arguments.json is None
        ):
            raise OvFormatError(
                "--include-all-values requires --json when --csv is the "
                "selected output"
            )
        manifest = extract_ov(
            arguments.input,
            scenario=arguments.scenario,
            probe_definition=arguments.probe_definition,
            include_all_values=arguments.include_all_values,
        )
        if arguments.json:
            arguments.json.write_text(
                json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False)
                + "\n",
                encoding="utf-8",
            )
        if arguments.csv:
            with arguments.csv.open("w", encoding="utf-8", newline="") as stream:
                write_csv(manifest, stream)
        if not arguments.json and not arguments.csv:
            json.dump(manifest, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
            sys.stdout.write("\n")
    except (OSError, OvFormatError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    if arguments.strict and manifest["status"] != "complete":
        print(
            f"error: {arguments.input}: extraction status is "
            f"{manifest['status']}",
            file=sys.stderr,
        )
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
