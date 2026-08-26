#!/usr/bin/env python3
"""Convert the supplied OPNET CSR modulation tables to exact C++ data."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import re
import struct
from typing import Callable
import zipfile


HEADER_BYTES = 512
METADATA = struct.Struct(">ddddI")
TRAILER = struct.Struct(">I")
RATES = (8, 16, 32, 64, 128)
JSR_BUCKETS = (-12, -9, -6, -3, 0)
HALF_CHIP_COUNTS = {8: 510, 16: 254, 32: 126, 64: 62, 128: 30}
COLLISION_NAME = re.compile(
    r"csr_(8|16|32|64|128)kbps_(-12|-9|-6|-3|0)dBJSR_"
    r"([0-9]+\.[05])ChipOff\.md\.m"
)


class Table:
    """Decoded numeric portion of an OPNET modulation table."""

    def __init__(self, name: str, data: bytes) -> None:
        if len(data) < HEADER_BYTES + METADATA.size + TRAILER.size:
            raise ValueError(f"{name}: file is too short")
        self.name = name
        (
            self.x_start,
            self.x_step,
            self.display_y_min,
            self.display_y_max,
            self.sample_count,
        ) = METADATA.unpack_from(data, HEADER_BYTES)
        expected = HEADER_BYTES + METADATA.size + 8 * self.sample_count + 4
        if len(data) != expected:
            raise ValueError(
                f"{name}: expected {expected} bytes, found {len(data)}"
            )
        self.values = struct.unpack_from(
            f">{self.sample_count}d", data, HEADER_BYTES + METADATA.size
        )
        (trailer,) = TRAILER.unpack_from(data, expected - TRAILER.size)
        if trailer != 0:
            raise ValueError(f"{name}: nonzero table trailer {trailer}")
        if not all(math.isfinite(value) for value in self.values):
            raise ValueError(f"{name}: non-finite BER sample")

    def nonzero_prefix(self) -> tuple[float, ...]:
        """Return samples through the last nonzero value."""
        count = len(self.values)
        while count > 0 and self.values[count - 1] == 0.0:
            count -= 1
        if any(value == 0.0 for value in self.values[:count]):
            raise ValueError(f"{self.name}: zero occurs before a nonzero sample")
        return self.values[:count]


def input_reader(source: Path) -> tuple[list[str], Callable[[str], bytes]]:
    """Return names and a byte reader for a directory or ZIP archive."""
    if source.is_dir():
        paths = {
            path.name: path
            for path in source.rglob("*.md.m")
            if path.is_file()
        }
        return sorted(paths), lambda name: paths[name].read_bytes()

    if zipfile.is_zipfile(source):
        archive = zipfile.ZipFile(source)
        members: dict[str, str] = {}
        for member in archive.namelist():
            name = Path(member).name
            if name.endswith(".md.m"):
                if name in members:
                    raise ValueError(f"duplicate table basename: {name}")
                members[name] = member
        return sorted(members), lambda name: archive.read(members[name])

    raise ValueError(f"input is neither a directory nor a ZIP archive: {source}")


def verify_metadata(
    table: Table, x_start: float, x_step: float, sample_count: int
) -> None:
    """Require the exact independent-variable grid used by the model."""
    if (
        table.x_start != x_start
        or table.x_step != x_step
        or table.sample_count != sample_count
    ):
        raise ValueError(
            f"{table.name}: unexpected grid "
            f"({table.x_start}, {table.x_step}, {table.sample_count})"
        )


def write_integer_array(
    output, declaration: str, values: list[int], per_line: int = 12
) -> None:
    """Write a compact C++ integer array."""
    output.write(f"constexpr {declaration} = {{\n")
    for index in range(0, len(values), per_line):
        chunk = values[index : index + per_line]
        output.write("  " + ", ".join(str(value) for value in chunk) + ",\n")
    output.write("};\n\n")


def write_double_array(output, declaration: str, values: list[float]) -> None:
    """Write IEEE-754 values as exact hexadecimal C++ literals."""
    output.write(f"constexpr {declaration} = {{\n")
    for index in range(0, len(values), 4):
        chunk = values[index : index + 4]
        output.write("  " + ", ".join(value.hex() for value in chunk) + ",\n")
    output.write("};\n\n")


def generate(source: Path, destination: Path) -> None:
    """Parse, validate, and emit the complete CSR table bundle."""
    names, read = input_reader(source)
    available = set(names)
    required = {"csr.md.m", "dqpsk.md.m"}
    missing = required - available
    if missing:
        raise ValueError("missing modulation tables: " + ", ".join(sorted(missing)))

    digest = hashlib.sha256()

    def load(name: str) -> Table:
        if name not in available:
            raise ValueError(f"missing table: {name}")
        data = read(name)
        digest.update(name.encode("ascii"))
        digest.update(b"\0")
        digest.update(data)
        return Table(name, data)

    standard = load("csr.md.m")
    verify_metadata(standard, -20.0, 0.1, 401)
    standard_values = list(standard.nonzero_prefix())

    dqpsk = load("dqpsk.md.m")
    verify_metadata(dqpsk, -20.0, 0.25, 161)
    dqpsk_values = list(dqpsk.nonzero_prefix())

    collision_values: list[float] = []
    collision_offsets = [0]
    expected_names = required.copy()
    for rate in RATES:
        for jsr in JSR_BUCKETS:
            for half_chip in range(HALF_CHIP_COUNTS[rate]):
                chip_offset = half_chip / 2.0
                name = (
                    f"csr_{rate}kbps_{jsr}dBJSR_"
                    f"{chip_offset:.1f}ChipOff.md.m"
                )
                expected_names.add(name)
                table = load(name)
                verify_metadata(table, -6.0206, 1.0, 31)
                collision_values.extend(table.nonzero_prefix())
                collision_offsets.append(len(collision_values))

    unexpected = available - expected_names
    if unexpected:
        raise ValueError(
            "unexpected modulation tables: "
            + ", ".join(sorted(unexpected)[:10])
        )
    collision_file_count = sum(
        1 for name in expected_names if COLLISION_NAME.fullmatch(name)
    )
    if collision_file_count != 4910:
        raise ValueError(f"expected 4910 collision tables, found {collision_file_count}")

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="ascii", newline="\n") as output:
        output.write(
            "/* Generated by utils/import-opnet-modulation-tables.py. */\n"
        )
        output.write(f"/* Parsed input SHA-256: {digest.hexdigest()} */\n\n")
        output.write(f"constexpr double CSR_STANDARD_X_START = {standard.x_start.hex()};\n")
        output.write(f"constexpr double CSR_STANDARD_X_STEP = {standard.x_step.hex()};\n")
        output.write(
            f"constexpr std::size_t CSR_STANDARD_SAMPLE_COUNT = "
            f"{standard.sample_count};\n"
        )
        output.write(
            f"constexpr std::size_t CSR_STANDARD_STORED_COUNT = "
            f"{len(standard_values)};\n"
        )
        write_double_array(output, "double CSR_STANDARD_VALUES[]", standard_values)

        output.write(f"constexpr double CSR_DQPSK_X_START = {dqpsk.x_start.hex()};\n")
        output.write(f"constexpr double CSR_DQPSK_X_STEP = {dqpsk.x_step.hex()};\n")
        output.write(
            f"constexpr std::size_t CSR_DQPSK_SAMPLE_COUNT = "
            f"{dqpsk.sample_count};\n"
        )
        output.write(
            f"constexpr std::size_t CSR_DQPSK_STORED_COUNT = "
            f"{len(dqpsk_values)};\n"
        )
        write_double_array(output, "double CSR_DQPSK_VALUES[]", dqpsk_values)

        output.write(
            f"constexpr double CSR_COLLISION_X_START = {(-6.0206).hex()};\n"
        )
        output.write(
            f"constexpr double CSR_COLLISION_X_STEP = {(1.0).hex()};\n"
        )
        output.write("constexpr std::size_t CSR_COLLISION_SAMPLE_COUNT = 31;\n")
        output.write("constexpr std::size_t CSR_COLLISION_CURVE_COUNT = 4910;\n\n")
        write_integer_array(
            output,
            "std::uint32_t CSR_COLLISION_VALUE_OFFSETS[]",
            collision_offsets,
        )
        write_double_array(
            output, "double CSR_COLLISION_VALUES[]", collision_values
        )
        output.write(
            "static_assert (sizeof (CSR_COLLISION_VALUE_OFFSETS) /\n"
            "               sizeof (CSR_COLLISION_VALUE_OFFSETS[0]) ==\n"
            "               CSR_COLLISION_CURVE_COUNT + 1);\n"
        )


def main() -> None:
    """Parse command-line arguments and generate the include file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "source", type=Path, help="modulation-table directory or ZIP archive"
    )
    parser.add_argument("destination", type=Path, help="generated .inc path")
    arguments = parser.parse_args()
    generate(arguments.source, arguments.destination)


if __name__ == "__main__":
    main()
