# OPNET scenario importer and differential harness

Updated: 2026-08-27

## Purpose

This increment turns the recovered OPNET project media into repeatable ns-3
comparison runs. The packet/event path has three independent parts:

1. `utils/import-opnet-scenario.py` decodes a CSR `*.nt.m` network model and
   its paired DES `*.ef` file into a versioned rectangular CSV.
2. `csr-opnet-scenario-runner.cc` creates the CSR ns-3 stack from that CSV and
   records ordered protocol, MAC, and PHY events.
3. `utils/compare-opnet-ns3-traces.py` normalizes an OPNET CSV export and the
   ns-3 trace, aligns their event identities, and reports ordering or field
   differences. `utils/run-opnet-differential.py` runs steps 2 and 3 together
   and records a reproducibility manifest.

A separate historical-aggregate path decodes recovered `*.ov` vector
databases with `utils/extract-opnet-ov.py`, derives equivalent bucket series
from the ns-3 trace with `utils/aggregate-ns3-trace.py`, and compares them with
`utils/compare-opnet-ns3-aggregates.py`. The fail-closed
`utils/run-opnet-aggregate-differential.py` workflow connects those steps and
records all inputs, commands, hashes, and exit statuses. Aggregate vectors and
chronological event traces are different evidence classes; one is not used as
a surrogate for the other.

The deterministic reservation/collision reference additionally uses
`utils/instrument-opnet-reservation-trace.py` and
`utils/validate-opnet-reservation-collision.py`. Its exact runbook is in
`docs/opnet-reservation-collision-reference.md`.

The importer reads ZIP members directly; the project archive does not need to
be unpacked first.

## Supported source media

The supplied CSR network models use the binary Modeler 17.1 tfile format,
identified by `MIL_3_Tfile_Hdr_`. The importer recovers the complete contiguous
length-prefixed promoted-attribute schema and its file-local IDs rather than
assuming that every scenario uses the same numeric IDs. This handles both the
shifted one-node schema and the additional fields present in older/mobile
campus nodes without moving the node ID or application attributes.

All 12 recovered campus `*.nt.m` scenarios and their paired DES environments
import successfully. The following
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

The recovered historical results include 10 complete Modeler `*.ov` databases:
109 vectors and 10,900 time buckets in total. The conservative extractor
validates vector offsets and extents, cross-checks identities and aggregation
types against the paired `*.pb.m` definitions, and preserves Modeler's
`2e+100` no-sample sentinel as a missing value. Two additional partial
fragments contain zero-length vector records and are correctly rejected by
strict extraction instead of being padded or guessed.

An `*.ov` database contains bucketed aggregates, not a chronological protocol
event stream. It can support historical traffic, delay, size, and drop
comparisons, but it cannot expose packet-level MAC reservation ordering or
same-time collision decisions. An instrumented OPNET event export in CSV form
is still required for event-by-event certification.

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

For repeatable differential runs, explicit `--flow` records remain available.
For the recovered campus models, `--infer-gateway-flows` implements the
source-backed `SEND_ONLY_TO_GATEWAY=1` destination pattern: it requires exactly
one gateway, creates a flow from every non-gateway node to that gateway, and
leaves the gateway itself silent. Every sender must provide promoted start,
interval, and packet-size attributes; a missing value is an error rather than a
silently omitted flow. For example:

```bash
python3 utils/import-opnet-scenario.py \
  historical-campus.zip imported-campus.csv \
  --scenario blue_radio_campus-2_nodes.nt.m \
  --infer-gateway-flows \
  --gateway-flow-dscp 5
```

The DSCP is a deterministic modeling choice, not a recovered per-packet fact.
`br_app` uses process-level `DSCP` and `DSCP_pct` inputs to choose between that
DSCP and zero probabilistically. The importer defaults the deterministic choice
to 5 for these comparisons, exposes it through `--gateway-flow-dscp`, and
records it in every canonical flow row.

`--infer-ring-flows` remains a deterministic smoke-test surrogate. It is not
source-exact for these runs because the OPNET application obtains destinations
from runtime route-table state.

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
shifted/mobile schemas, DES parsing, gateway-flow inference, alias
normalization, timestamp tolerance, missing-event reporting, and both complete
runner/comparator workflows.

## Running the historical aggregate workflow

The aggregate workflow accepts the same canonical scenario CSV and runner,
plus a recovered OPNET vector database, its mandatory probe definition, and
the expected SHA-256 identity of both historical inputs:

```bash
python3 utils/run-opnet-aggregate-differential.py \
  --scenario imported-campus.csv \
  --runner ../ns-3-dev/build/scratch/ns3-dev-csr-opnet-scenario-runner-default \
  --opnet-ov blue_radio_campus-2_nodes-DES-1.ov \
  --probe-definition blue_radio_campus-2_nodes.pb.m \
  --expected-opnet-ov-sha256 dbbbd61759299e9a41aaed9d53acb983df653670843d5ae62f039b772dd989e5 \
  --expected-probe-definition-sha256 8ed62d4c3bc1a11eb5639394c6ccea5b5d415c8afad138b3e493b413ee0a359e \
  --output-dir aggregate-differential
```

The workflow verifies the mandatory PB/OV expected hashes before touching its
output directory, extracts in strict mode, requires matching scenario name,
seed, full duration, and complete consistent OPNET time axes, runs ns-3 with a
compact exactly correlated aggregate trace, derives source-equivalent bucket
series, and compares only exact statistic/unit/aggregation identities. A
successful manifest says `selected_comparison_passed` within the historical
aggregate evidence scope; it does not claim event-order or whole-model parity.
The recovered application packet-size statistic is
`configured_bytes * 8 - 64` bits. The runner reproduces that rule at packet
creation: a configured 600-byte flow becomes a 592-byte total network packet,
and a configured 200-byte flow becomes 192 bytes. The aggregator consumes
those actual trace sizes with no default post-processing adjustment. A legacy
trace adjustment is explicit and recorded in provenance.

The first full-duration exact-tolerance comparisons are calibration evidence,
not parity passes:

| Scenario | Compared points | Exact packet size | Mean OPNET vs ns-3 | Exact-tolerance outcome |
| --- | ---: | --- | --- | --- |
| `blue_radio_campus-2_nodes` | 800 | All 100 buckets | Sent 16.4417 vs 12.9128 packet/s; received 16.4373 vs 12.9126 packet/s; delay 1.45125 vs 1.05710 s | 700 numeric mismatches; no missing or extra points |
| `blue_radio_campus-hidden_nodes_symmetrical` | 800 | All 100 buckets | Sent 12.1707 vs 14.7748 packet/s; received 11.4370 vs 14.5755 packet/s; delay 9.17976 vs 3.29167 s | 700 numeric mismatches; no missing or extra points |
| `blue_radio_campus-multihop` | 800 | All 95 measured buckets; 5 OPNET no-sample buckets preserved | Sent 2.0120 vs 2.39233 packet/s; received 1.90167 vs 2.27833 packet/s; delay 112.748 vs 47.0161 s | 665 numeric mismatches; no missing or extra points; 20 missing values skipped |

These failures were not hidden by widening tolerances. The exact packet-size
matches validate vector decoding and the source-level modeled size, while the
traffic-rate and delay differences identify real model-calibration work.

## Remaining certification boundary

These harnesses make differences reproducible; they do not manufacture missing
evidence. The recovered `*.ov` files now provide authoritative historical
bucket aggregates, and the measured comparisons above establish both exact
size agreement and unresolved traffic/delay differences. They cannot establish
same-time control ordering. The source-checked instrumentation, deterministic
reservation/collision scenario, validator, and ns-3 side are complete, but
event-order certification still needs the CSV exported by a licensed Modeler
run. Terrain closure also remains external because the supplied TMM database
and callable terrain model were not recovered.
