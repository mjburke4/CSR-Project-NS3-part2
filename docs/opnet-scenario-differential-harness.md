# OPNET scenario importer and differential harness

Updated: 2026-08-27

## Purpose

This increment turns the recovered OPNET project media into repeatable ns-3
comparison runs. It has three independent parts:

1. `utils/import-opnet-scenario.py` decodes a CSR `*.nt.m` network model and
   its paired DES `*.ef` file into a versioned rectangular CSV.
2. `csr-opnet-scenario-runner.cc` creates the CSR ns-3 stack from that CSV and
   records ordered protocol, MAC, and PHY events.
3. `utils/compare-opnet-ns3-traces.py` normalizes an OPNET CSV export and the
   ns-3 trace, aligns their event identities, and reports ordering or field
   differences. `utils/run-opnet-differential.py` runs steps 2 and 3 together
   and records a reproducibility manifest.

The deterministic reservation/collision reference additionally uses
`utils/instrument-opnet-reservation-trace.py` and
`utils/validate-opnet-reservation-collision.py`. Its exact runbook is in
`docs/opnet-reservation-collision-reference.md`.

The importer reads ZIP members directly; the project archive does not need to
be unpacked first.

## Supported source media

The supplied CSR network models use the binary Modeler 17.1 tfile format,
identified by `MIL_3_Tfile_Hdr_`. The importer recovers the file-local promoted
attribute IDs from the length-prefixed schema rather than assuming that every
scenario uses the same numeric IDs. This matters because the one-node model
omits `Node Type`, shifting the later application fields.

All eight supplied CSR validation scenarios import successfully. The following
values are recovered when present:

| Source | Imported values |
| --- | --- |
| `*.nt.m` node instance | 24-bit Node ID, name, node type, x/y position, altitude, minimum/maximum rate, minimum/maximum power, link margin, ECC threshold, RX/TX frequency, application interval/size/start |
| Paired `*-DES-1.ef` | simulation duration, random seed, and TMM enable flag |
| Command line | explicit deterministic source, destination, start, interval, packet size, and DSCP |

Modeler fixed-subnet coordinates are converted with a configurable scale. The
default is 1,000 metres per displayed coordinate unit, matching the supplied
validation layouts. Promoted frequency values below 1 MHz are interpreted as
MHz because the recovered scenarios store `400` for 400 MHz.

The supplied node model sets a 1-MHz transmitter and receiver bandwidth but
does not promote it per node. The runner therefore applies that source-backed
node-model default. TMM terrain runs are rejected explicitly rather than
silently falling back to the non-terrain propagation model.

The supplied `*.ov` files are Modeler output-vector databases containing
aggregate statistics. They are useful as aggregate drop/count checkpoints,
but they do not expose the packet-level ordering needed by the differential
event comparator. An OPNET event export in CSV form is still required for an
event-by-event certification run.

## Canonical scenario schema

Every row uses schema `csr-opnet-scenario-v1` and one of three record types:

- `run`: scenario digest, duration, seed, TMM flag, coordinate scale, and an
  optional reservation-control activation time.
- `node`: one imported CSR node, all calibrated radio/link attributes, and an
  optional controlled reservation slot.
- `flow`: one explicit deterministic application flow.

The file is deliberately rectangular so both Python and the C++ runner reject
missing or shifted fields. The source digest covers the exact `*.nt.m` and
paired `*.ef` bytes.

Import a scenario directly from the multi-scenario archive:

```bash
python3 utils/import-opnet-scenario.py \
  "Project files for chatgpt(2).zip" \
  imported-scenario.csv \
  --scenario CSR_Validation-2_Nodes.nt.m \
  --flow 1:2:300:0.02:600:5
```

For repeatable differential runs, explicit `--flow` records are preferred.
`--infer-ring-flows` is available as a deterministic smoke-test surrogate when
only promoted application attributes exist. It is not source-exact because the
OPNET application obtains destinations from runtime route-table state.

## Canonical event trace

The runner writes `csr-differential-trace-v1` rows with a monotonic event index
and simulation timestamp. Its current observation points are:

- scenario node/link construction and application sends;
- MAC Idle/Search/Track/Tx state transitions;
- MAC reservation preparation, holdoff completion, per-slot countdown, and
  next-reservation advertisement;
- OTA transmission starts;
- PHY/MAC receive acceptance and rejection, including closure, receiver state,
  path loss, received power, noise, SNR, JSR, and interval error counts;
- NWK delivery and route changes; and
- packet identity, rate, modeled size, sequence, and security count wherever
  that information exists.

The trace writer contains observation only. Opening or closing it does not
change queueing, random draws, packet delivery, or simulator event scheduling.
Controlled reservation fields in a scenario are separate, explicit test
inputs and are disabled in ordinary imported scenarios.

OPNET CSV exports may use canonical names or common aliases such as `Time`,
`Action`, `Node ID`, `Tx Node`, `Pkt Type`, `Seq`, `SNR`, and `Accepted`. Event
aliases such as `TX`, `Receive`, `Drop`, and `Delivery` are normalized before
alignment. Rows must remain in nondecreasing timestamp order.

## Comparison behavior

Events are aligned in order using their mutually available identity fields.
The default identity is event, receiver node, peer, packet type, source,
destination, and sequence; optional identity fields are omitted when one input
does not provide them. Matching rows compare every field present in both
traces.

Default absolute tolerances are:

| Field | Tolerance |
| --- | ---: |
| Time | 1 microsecond |
| Path loss | 0.001 dB |
| Received power | 0.001 dBm |
| Noise | 0.001 dBm |
| SNR | 0.001 dB |
| JSR | 0.001 dB |

Missing optional values are counted as coverage gaps but do not turn an
otherwise matching comparison into a failure. Missing, extra, replaced, or
field-mismatched events do fail and make the comparator exit with status 1.
Tolerances can be overridden per field, and known unavailable fields can be
ignored explicitly; both choices are captured in the report/manifest.
Repeatable `--event` arguments and inclusive `--time-start`/`--time-stop`
values isolate a certification window. Repeatable `--select EVENT:NODE`
arguments further isolate exact observation points. `--coincident-tolerance`
canonicalizes the arbitrary execution order of events that occur at
effectively the same simulation time while preserving order between distinct
event times.

## Running the harness

Build the runner in the ns-3 tree:

```bash
./ns3 build csr-opnet-scenario-runner
```

Then execute a comparison:

```bash
python3 utils/run-opnet-differential.py \
  --scenario imported-scenario.csv \
  --runner ../ns-3-dev/build/scratch/ns3-dev-csr-opnet-scenario-runner-default \
  --opnet-trace opnet-events.csv \
  --opnet-manifest opnet-instrumentation-manifest.json \
  --output-dir differential-run
```

The output directory contains:

- the raw ns-3 event trace and console log;
- normalized OPNET and ns-3 traces;
- `differential-report.json`, with counts and bounded detailed differences;
- `run-manifest.json`, with input hashes, executable path, commands, and exit
  statuses.

Focused repository fixtures exercise binary promoted-attribute decoding,
shifted schemas, DES parsing, alias normalization, timestamp tolerance,
missing-event reporting, and the complete runner/comparator workflow.

## Remaining certification boundary

This harness makes differences reproducible; it does not manufacture an
OPNET reference trace. The source-checked instrumentation, deterministic
reservation/collision scenario, validator, and ns-3 side are complete, but
exact certification still needs the CSV exported by a licensed Modeler run.
Terrain
closure also remains external because the supplied TMM database and callable
terrain model were not recovered. Aggregate `*.ov` statistics can be compared
separately once their vector identities are mapped, but they cannot establish
same-time control ordering on their own.
