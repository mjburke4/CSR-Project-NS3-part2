# OPNET aggregate comparison

Updated: 2026-09-05

## Purpose and evidence boundary

`utils/compare-opnet-ns3-aggregates.py` compares historical OPNET output-vector
series with equivalent ns-3 aggregate series. It is deliberately separate from
the packet/event comparator: bucketed output vectors can validate traffic,
drop, delay, and queue statistics, but they cannot recover chronological MAC
reservation or collision events that were not recorded by Modeler.

The selected recovered evidence set contains 11 complete Modeler 17.1 `*.ov`
files. The read-only extractor decodes 127 vectors and 12,700 buckets across
those runs and cross-checks their labels and aggregation identities against
paired `*.pb.m` definitions. Two additional partial fragments contain zero-length
vector records; `--strict` writes their diagnostic manifests and exits with
status 3 without inventing samples.

The aggregate toolchain is:

| Tool | Role |
| --- | --- |
| `utils/extract-opnet-ov.py` | Validate and decode Modeler 17.1 `MIL_3` vector records to canonical CSV/JSON |
| `utils/aggregate-ns3-trace.py` | Correlate application sends/deliveries and derive OPNET-equivalent ns-3 buckets |
| `utils/compare-opnet-ns3-aggregates.py` | Align exact series/time identities and report coverage or value differences |
| `utils/run-opnet-aggregate-differential.py` | Run the strict end-to-end workflow and record reproducibility provenance |

Every workflow run also retains `app-admission-diagnostics.csv`. Its per-flow
counters must exactly partition generator attempts into admitted packets and
one source-ordered gate reason before the workflow proceeds.

The workflow revalidates immutable input/tool identity and the output
directory's device/inode at every cross-stage boundary. Managed artifacts must
appear only in their owning stage and must be single-link regular files that do
not alias protected inputs. Parent-written logs and manifests use guarded
atomic replacement; injected symlinks, hardlinks, special files, unexpected
future outputs, and path-resolution errors stop the run. The ns-3 aggregation
provenance must bind both the aggregate SHA-256 and the canonical
`output.path`. These checks protect the evidence chain between subprocesses;
they do not claim kernel-level exclusion of a concurrent path swap inside a
child executable's own file-open interval.

## Canonical CSV contract

Every row describes one sample from one aggregate time series. The six core
columns, in canonical order, are:

| Column | Meaning |
| --- | --- |
| `schema` | `csr-aggregate-series-v1` |
| `scenario` | Exact stable scenario identity |
| `statistic` | Exact scope-qualified statistic identity |
| `time_s` | Finite, nonnegative simulation time in seconds |
| `value` | Finite numeric sample, or empty for a recorded no-sample bucket |
| `source` | `opnet` or `ns3` |

The standardized optional columns are `unit`, `aggregation`, `raw_value`,
`value_status`, `source_file`, and `source_file_sha256`. Extra columns are
tolerated. The recovered OPNET extractor uses these aggregation identities:

- `bucket_sum_per_second`
- `bucket_sum`
- `bucket_sample_mean`

Units and aggregation semantics are evidence, not display metadata. Series are
comparable only when `(scenario, statistic, unit, aggregation)` matches
exactly. A statistic expressed in seconds is therefore never numerically
compared with a statistic expressed in milliseconds, nor is a bucket sum
compared with a sample mean. Unknown values remain empty rather than being
guessed.

Scenario node IDs and concrete flow endpoints are limited to
`0x000000..0xFFFFFE`; `0xFFFFFF` is the CSR broadcast sentinel and is never a
concrete topology identity. Explicit flow start and interval values must be
finite, with nonnegative start and positive interval.

### Nested latency-result layout

`CSR_Validation-2_Nodes_Latency-DES-1.ov` adds a source-observed nested
metadata shape. The u32 after the description table is a declared metadata
unit count, not a free format tag. The decoder requires it to equal
`hierarchy_containers + statistic_groups + 2*statistics`; the recovered value
is exactly 62 (`3 + 11 + 2*24`). Linked 0x22 hierarchy containers hold
`Campus Network`, `node_2`, and `node_1`; their 0x23 statistic groups and every
statistic link must end exactly at the decoded zero terminator.

The file contains 18 canonical bucket aggregates with 100 12-second buckets
and six per-node `All values` records. Only the aggregates are emitted to the
canonical CSV. By default the richer JSON reports only validated metadata for
the nonaggregate records. `--include-all-values` additionally retains their
interior `{record_index, time_s, value}` pairs in JSON; it never adds them to
the aggregate CSV. Equal timestamps and encoded order are preserved, while the
structural first/last boundary entries are not reported as events. Each record
must independently have the exact
`All values` metadata, marker 0x81, encoding/cookie/reserved fields, two-array
extent, boundary sentinels, and a finite nondecreasing time axis from 0 to
1,200 seconds. The paired probe definition provides 18 exact aggregate
identities and three wildcard node identities that expand to the six records.
Other archived 0x224 records use bucket aggregation and marker 0x82, so 0x224
alone is never treated as evidence of an `All values` record.

Formula-based metadata counts intentionally replace the earlier empirical
count allowlist. Acceptance is still shape-gated: only the observed linked
0x22/0x23 hierarchy, 0x124/0x224 statistic records, exact reserved fields,
known details variants, ordered unique data offsets, and validated 0x81/0x82
record forms pass. Unknown nesting depth, mixed ordering, aliases, malformed
terminators, and unsupported aggregation/record combinations fail closed.
Every supported complete 0x82 record in the recovered corpus declares exactly
100 buckets, encodes count 101, starts at time zero, and has a positive
execution duration; those are enforced before allocating bucket state. The
latency result's largest 0x81 record encodes 11,019 entries, which is also the
strict per-record upper bound for this bounded decoder. Opt-in retention is
additionally bounded before allocation by the recovered six-record total of
31,758 interior events.

These pairs are per-statistic Modeler write records, not packet traces. They
contain no packet or tree identifier. Duplicate timestamps are legitimate, and
an index is meaningful only within one statistic's encoded record. A
source-derived state transition may be bounded by one series' write time, but
row-by-row correlation between MAC, HOP, and NWK series is unsafe without an
independent identity or uniquely proven boundary.

The paired-probe scan is likewise source-bounded before retaining labels:
15,123 bytes, 27 declared probes, 489 printable tagged strings, and 32 bytes
per tagged string are the recovered corpus maxima. Metadata counts are checked
incrementally against both their file declaration and the 113-unit corpus
maximum. Reconstructed timestamps must remain finite, scope/name identities
must be nonempty, and the 4e100 All-values end sentinel is rejected inside an
aggregate sample payload.

Modeler encodes an empty sample-mean bucket as the numeric sentinel `2e+100`.
The extractor converts it to an empty `value`, preserves `raw_value=2e+100`,
and emits `value_status=missing`. The comparator aligns that bucket but reports
it under `skipped_missing_values`; it never treats the sentinel as a measured
value or a numeric mismatch.

The recovered application size has one source-specific rule. For a configured
packet size of `N` bytes, `br_app` creates a total `br_Network` packet of
`N * 8 - 64` bits (4,736 bits for 600 bytes and 1,536 bits for 200 bytes). The
runner applies that exclusion when it creates the modeled packet, so airtime,
queuing, and the emitted trace all use `N - 8` bytes. The aggregator's default
adjustment is therefore zero. `--legacy-trace-size-exclusion-bits` (and its
deprecated `--opnet-size-exclusion-bits` alias) exists only for older traces
that recorded the configured size; any adjustment is explicit in provenance.

## Alignment and tolerances

Recovered Modeler buckets use `[previous_bucket_end, bucket_end)` endpoints.
The ns-3 aggregator therefore assigns an event exactly on an interior boundary
to the following bucket, includes `t=0` in the first bucket, and excludes an
event exactly at the configured stop time. Stop-boundary and after-stop events
are reported separately in provenance rather than clamped into the last
bucket.

Input order is irrelevant. Within each comparable series, OPNET and ns-3
points are sorted by time and aligned one-to-one. Alignment first maximizes the
number of points within `--time-tolerance`, then minimizes their total timestamp
error. This prevents a nearby sample from consuming a better exact match.

Matched values pass when:

```text
absolute_delta <= max(abs_tolerance,
                      rel_tolerance * max(abs(opnet), abs(ns3)))
```

All tolerances default to zero. The JSON report separates:

- unmatched OPNET-only and ns-3-only series;
- noncomparable series whose unit or aggregation semantics differ;
- missing and extra timestamps inside a comparable series;
- aligned numeric mismatches;
- OPNET reference buckets with no measured value, which are skipped; and
- missing ns-3 values where OPNET did observe a value, which fail as
  `missing_ns3_value`.

Unmatched/noncomparable series, point coverage differences, and numeric
mismatches fail the comparison. An OPNET no-sample bucket is reported but does
not fail an otherwise anchored comparison; an all-missing OPNET series is
indeterminate and fails rather than establishing parity without a reference
value. Exact `--scenario`, `--statistic`, `--unit`, and `--aggregation` filters
can restrict a run to the subset that both simulators actually measure with the
same semantics; closed `--time-start` and `--time-stop` filters select
bucket-end windows. Filtered rows are counted in the report.

## Extraction and full workflow

Decode and independently cross-check one historical result:

```bash
python3 utils/extract-opnet-ov.py \
  blue_radio_campus-2_nodes-DES-1.ov \
  --probe-definition blue_radio_campus-2_nodes.pb.m \
  --scenario blue_radio_campus-2_nodes \
  --csv opnet-aggregates.csv \
  --json opnet-extraction.json \
  --strict
```

Run the complete scenario/extract/aggregate/compare path:

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

The paired probe definition and both expected digests are mandatory. The
workflow checks them before accepting or modifying the output directory and
records expected and actual identities in the manifest. It then fails closed
if extraction is partial, vector time axes are inconsistent, scenario name,
seed, or full duration differs, exact sequence correlation is lost, the runner
fails, or the comparator finds a selected difference.

"Exact sequence correlation" includes one source-exact exception: a duplicate
delivery may reuse the original send timestamp only when compact `hop_feedback`
rows prove that the same relay and ingress peer accepted the same incoming HOP
sequence and application tag after an earlier DACK. The retry's later feedback
may be ACK or DACK as congestion changes; each feedback consumes a prior,
unconsumed matching relay enqueue (allowing intervening statistic samples),
and ACK closes that lineage. Every unproved duplicate remains an unmatched
delivery and stops the workflow.

A current run with zero repeated-feedback proofs is the strict form: all three
duplicate counts and the lineage list must be empty. The historical
source-proved duplicate ledger remains accepted for reproducibility only when
every count partitions exactly and every extra delivery consumes one complete,
trace-bound lineage budget. A zero-count summary carrying historical lineage
records, or a nonzero summary without them, fails closed.

Manifest success is `selected_comparison_passed`, never a claim of whole-model
OPNET parity. `evidence_scope` is
`historical_bucket_aggregates_not_event_parity`. Exact tolerances, the full core
statistic set, the full time window, normal OPNET behavior flags, and no legacy
size adjustment form the `canonical` profile; any supported override is listed
and marks the run `diagnostic`. The default statistic set excludes ECC drops
because that vector is absent from some historical results. Direct inspection
of `br_mac` and `br_ecc` proves the event identity: only a packet that remains
selected and exceeds the inclusive ECC threshold increments the statistic.
Signals rejected during synchronization/acquisition are `prior_stage`, while
a locked transmitter is `half_duplex`; neither contributes. The ECC series
therefore remains an explicit diagnostic selection where Modeler recorded it.

## Direct comparator usage

The comparator accepts one merged CSV or multiple source-specific CSVs:

```bash
python3 utils/compare-opnet-ns3-aggregates.py \
  opnet-aggregates.csv ns3-aggregates.csv \
  --time-tolerance 1e-6 \
  --abs-tolerance 1e-9 \
  --rel-tolerance 0.01 \
  --report aggregate-report.json \
  --normalized aggregate-normalized.csv
```

To compare only one source-equivalent statistic:

```bash
python3 utils/compare-opnet-ns3-aggregates.py \
  opnet-aggregates.csv ns3-aggregates.csv \
  --statistic "Sink.Traffic Received (packets/sec)" \
  --aggregation bucket_sum_per_second \
  --report traffic-received-report.json
```

The report records input paths and SHA-256 digests, exact filters and
tolerances, bounded detailed differences, skipped buckets, and a per-series
summary. Exit status is zero only when every selected comparable measurement
passes and no selected series or point is unmatched.

## Recovered-case provenance

The selected historical comparisons use these exact source files. A future
rerun should reject a hash mismatch rather than silently treating a similarly
named scenario as the same evidence.

| Scenario | Source | SHA-256 |
| --- | --- | --- |
| `blue_radio_campus-2_nodes` | `*.nt.m` | `7337801b9f8886e3bc42903433c3fb7237fa7eb3aff48b377046640e30477e43` |
|  | `*-DES-1.ef` | `67b1667da06408ce190347a6ada98cf9989e75e986b753dd21671823c7ff3fa5` |
|  | `*-DES-1.ov` | `dbbbd61759299e9a41aaed9d53acb983df653670843d5ae62f039b772dd989e5` |
|  | `*.pb.m` | `8ed62d4c3bc1a11eb5639394c6ccea5b5d415c8afad138b3e493b413ee0a359e` |
|  | `*.dev32.i1.nt.so` | `d9ebf7626e641ee68ccc9b58b1bb4b28fc0906111762e4456a0e8c7a0bd8b055` |
| `blue_radio_campus-hidden_nodes_symmetrical` | `*.nt.m` | `4ac306b6d984d2d8b65557261d11a0a4321082f30e75dd2ce1affa9486093cbd` |
|  | `*-DES-1.ef` | `3f4a302486fbdc8e5fe56e0df5da1de8ea2c6216ae9324b3e33ecbb33a3f703c` |
|  | `*-DES-1.ov` | `f656cf217236392a845fbfa5bcc1458df8fbe0b62bc04af54435013a6f2e537d` |
|  | `*.pb.m` | `3a22de6cfa554916f6f9649bd67308f956a04f86bc6b79a3cd1dacd059bd100c` |
|  | `*.dev32.i1.nt.so` | `dd3f38e8d33700b61f9e360a737ba34e56cb75b2570eb2960a02de381ed0fff0` |
| `blue_radio_campus-multihop` | `*.nt.m` | `7052743dc082f3ba34ee2bd8ac70068adf8091efaf4ff8cb2116faff088abcd7` |
|  | `*-DES-1.ef` | `a5ce1d570adbb9dd0e2720506bcbf645abf4d53d563e3ccb0fbeac2f6d7feff9` |
|  | `*-DES-1.ov` | `b643bee5c1d0260581a0135c0add5c87441bd98f70f66388fac0bf95096c39ee` |
|  | `*.pb.m` | `2871b54d0c531198f8ff77b1aa2a3cc81da6f05814c9ca00e64ed6a544a6470f` |
|  | `*.dev32.i1.nt.so` | `adb97c54f7566439f1404e972d3d777a3bca613e2a965bf12f03353fb009d9af` |
| `CSR_Validation-2_Nodes_Latency` | `*-DES-1.ov` | `c9ebc87410b68780abac187a5164939cd95ffe5ba4a5a7e393012117d4235493` |
|  | `*.pb.m` | `a43c81806e6216491cad831482d509481239934b5f3d8980df4b588fb6d9ff94` |

## Retained pre-reconstruction exact-tolerance results

The four completed full-duration core-statistic comparisons align all 800
selected points in each case, with no missing or extra timestamps. They do not
pass numeric parity:

| Scenario | Statistic | OPNET mean | ns-3 mean | Result |
| --- | --- | ---: | ---: | --- |
| `blue_radio_campus-2_nodes` | Traffic sent | 16.4417167 packet/s | 16.3596500 packet/s | 0.4991% error vs OPNET mean |
|  | Traffic received | 16.4372667 packet/s | 16.3473667 packet/s | 0.5469% error vs OPNET mean |
|  | End-to-end delay | 1.4512490 s | 1.4273729 s | 1.6452% error vs OPNET mean |
|  | Application packet size | 4,736 bits | 4,736 bits | Exact in all 100 buckets |
| `blue_radio_campus-hidden_nodes_symmetrical` | Traffic sent | 12.1707333 packet/s | 12.7283333 packet/s | 4.5815% error vs OPNET mean |
|  | Traffic received | 11.4369500 packet/s | 12.0342167 packet/s | 5.2223% error vs OPNET mean |
|  | End-to-end delay | 9.1797597 s | 8.4022281 s | 8.4701% error vs OPNET mean |
|  | Application packet size | 4,736 bits | 4,736 bits | Exact in all 100 buckets |
| `blue_radio_campus-multihop` | Traffic sent | 2.0120000 packet/s | 2.4566667 packet/s | 22.1007% error vs OPNET mean |
|  | Traffic received | 1.9016667 packet/s | 2.2900000 packet/s | 20.4207% error vs OPNET mean |
|  | End-to-end delay | 112.7480 s | 91.4031 s | 18.9315% error vs OPNET mean |
|  | Application packet size | 1,536 bits | 1,536 bits | Exact in 95 measured buckets; 5 no-sample buckets |
| `CSR_Validation-2_Nodes_Latency` | Traffic sent | 8.6366667 packet/s | 8.6708333 packet/s | 0.3956% high vs OPNET mean |
|  | Traffic received | 8.6283333 packet/s | 8.6625000 packet/s | 0.3960% high vs OPNET mean |
|  | End-to-end delay | 1.0508268 s | 1.0694113 s | 1.7686% high vs OPNET mean |
|  | Application packet size | 4,736 bits | 4,736 bits | Exact in all 75 measured buckets |

The 1,200-second latency run is a canonical workflow execution through the
comparison stage, but it does not pass zero-tolerance numeric parity. It
aligns all 800 selected bucket identities; 100 OPNET no-sample buckets are
skipped, 700 numeric points are compared, and 517 differ. Packet-size values
match in all 75 measured buckets. The runner records 45,000 application
attempts, 10,405 admissions, 34,595 source-exact NSDP blocks, 10,395 matched
deliveries, no unmatched delivery, and no packet-size mismatch. Thus the close
headline means are aggregate evidence, not an exact bucket-level parity claim.

These retained runs use the hash-bound historical application and MAC
profiles but predate the source-backed 217-byte bare-DATA correction. They are
not release-candidate evidence until regenerated with explicit
`hop_security_profile=hist-adb97c54-bare` and the full executable digest. They
otherwise use FIFO DSCP 0, source-ordered generator gates, and the common 0.988-second wake
phase. The two-node and hidden-node headline means are now close without a
tolerance adjustment. Multihop remains materially high in traffic and low in
delay, so the four recovered HOP/MAC queue vectors were compared next rather
than changing slot policy again.

The hidden-node reconstruction is independently bound to
`legacy-send-to-from-no-dscp`,
`hist-2015-fine-one-based-table-no-avoid`, and
`hist-dd3f38e8-bare`, with executable SHA-256
`dd3f38e8d33700b61f9e360a737ba34e56cb75b2570eb2960a02de381ed0fff0`.
That bare profile produces 617-byte OTA DATA from the configured 600-byte
application packet and 41-byte ACK/DACK. Multihop remains bound to the adb97
tuple and 217-/41-byte wire forms. The binding follows executable evidence;
the tools do not infer either tuple from a scenario name or topology.

### Reconstructed worktree results (2026-09-05)

The full-duration hidden-node and multihop workflows were rerun from this
reconstruction with their exact hash-bound tuples and no noncertifying
overrides. Both workflows completed extraction, ns-3 execution, and strict
aggregation successfully. Their only nonzero stage status is the expected
zero-tolerance aggregate comparator result:

| Scenario | Statistic | OPNET mean | ns-3 mean | ns-3 residual |
| --- | --- | ---: | ---: | ---: |
| `blue_radio_campus-hidden_nodes_symmetrical` | Traffic sent | 12.1707333 packet/s | 13.3092167 packet/s | +9.3543% |
|  | Traffic received | 11.4369500 packet/s | 12.6972833 packet/s | +11.0198% |
|  | End-to-end delay | 9.1797597 s | 8.1754843 s | -10.9401% |
|  | Application packet size | 4,736 bits | 4,736 bits | Exact in all 100 buckets |
| `blue_radio_campus-multihop` | Traffic sent | 2.0120000 packet/s | 2.0561667 packet/s | +2.1952% |
|  | Traffic received | 1.9016667 packet/s | 1.9596667 packet/s | +3.0500% |
|  | End-to-end delay | 112.7480075 s | 95.6824538 s | -15.1360% |
|  | Application packet size | 1,536 bits | 1,536 bits | Exact in 95 measured buckets; 5 no-sample buckets |

Hidden-node aligns all 800 bucket identities and compares all 800 numeric
values; 700 differ at exact tolerance. All 761,837 deliveries match exact
application sequences and packet sizes, with 36,716 sends lacking an in-window
delivery and no unmatched delivery. Multihop also aligns all 800 identities,
compares 780 numeric values, preserves the 20 authoritative no-sample values,
and reports 625 exact-tolerance differences. It validates 11,757 exact-sequence
deliveries plus one fully source-proved repeated-DACK delivery, with 580 sends
lacking an in-window delivery and no unmatched or size-mismatched delivery.

These results support the bounded general-parity claim: scenario identity,
time axes, packet sizes, and delivery lineage are exact, while the three
headline aggregate means remain within a largest absolute residual of 15.136%.
They do not establish exact bucket values, chronological event/RNG identity,
or PHY/ECC numeric calibration.

### Multihop queue-residence diagnostic

The recovered probes are global `bucket/default total/sample mean` vectors.
Each OPNET process writes its local post-event queue value into one pooled
global sample stream; the result is neither a time-weighted occupancy nor an
average of per-node means. The ns-3 observers reproduce the source write
sites and ordering without inventing initialization or final-zero samples:

- HOP resend size after successful admission, ACK removal, and timeout
  removal, but not after DACK removal or obsolete-key cleanup;
- MAC ACK size after successful admission, unchanged on a full-queue
  rejection, and after resend-limit removal;
- MAC Tx size after successful admission and each actual transmission
  selection, but not after ACK/DACK cancellation; and
- MAC Tx queuing delay at actual transmission selection, measured from the
  timestamp recorded immediately before successful admission.

The retained pre-fix 6,000-second multihop workflow aligned all 400
queue-vector bucket
positions. It compared 394 numeric pairs, found 394 exact-value mismatches,
skipped the four OPNET no-sample values at 300 seconds, and reported two
additional ns-3 no-sample values at 180 seconds for HOP resend and MAC ACK.
There were no missing, extra, noncomparable, or time-misaligned series.
As intended for this queue-only selection, the manifest identifies the run as
the `diagnostic` profile with `statistics_not_complete_core_set`; the workflow
status is `selected_comparison_failed` because the measured residual is real.

To avoid treating startup-only empty buckets as steady behavior, these are the
arithmetic means of the observed 60-second bucket sample-means strictly after
300 seconds:

| Statistic | OPNET | ns-3 | ns-3 residual |
| --- | ---: | ---: | ---: |
| HOP resend queue size | 10.301027 packets | 11.959792 packets | +16.1029% |
| MAC ACK queue size | 0.676701 packets | 0.591629 packets | -12.5715% |
| MAC Tx queue size | 9.537101 packets | 10.840019 packets | +13.6616% |
| MAC Tx queuing delay | 7.208239 s | 7.846244 s | +8.8510% |

This rules out a simple "ns-3 MAC serves packets too quickly" explanation for
the 18.9% low end-to-end delay: ns-3's MAC Tx queuing delay is higher, not
lower. The higher HOP and Tx queue samples also confirm excess admitted or
retransmitted load. The next diagnostic boundary is therefore above and
across the MAC: delivered hop count/path, route convergence, and NWK/HOP
admission and residence time.

### Packet-path and NWK-residence diagnostic

That boundary is now instrumented without changing modeled packet bytes,
queue order, route selection, random draws, or event scheduling. The NWK
queue-size and delay samples reproduce the supplied `br_nwk.pr.c` write sites:
post-insert size for local and relay DATA, then post-remove size and per-hop
residence immediately before HOP admission. The compact trace also retains
correlated `nwk_enqueue`, `nwk_forward`, `nwk_delivery`, and `route_change`
rows. Forward rows record the actual next hop and the selected route cost/path;
stored partial static paths are normalized by the same rule used for ARL
serialization.

The retained pre-fix 6,000-second multihop rerun produced 395,412 trace rows
and 14,740 packet keys. The packet-path analyzer accepted all 13,740 delivered
chains,
reported no incomplete delivery, loop, duplicate admission, next-hop mismatch,
route-context mismatch, negative interval, or decomposition error, and retained
1,000 valid end-of-run prefixes instead of misclassifying them as failures.
Of those prefixes, 508 end in an NWK queue and 492 have already been handed to
HOP without a subsequent relay enqueue or delivery. Joining the path and
admission ledgers now classifies those 492 terminal-forward prefixes as 438
completed `no_ack`, 33 open resend, 16 completed ACK, and 5 completed DACK
legs. Thus most of the formerly generic post-NWK population is historical
final no-ACK loss rather than traffic still in flight at the stop. All 95
nonempty end-to-end-delay buckets match `aggregate-ns3-trace.py` exactly, with
maximum absolute difference zero.

The pre-fix delivered paths are stable and source-specific:

| Source | Delivered path | Hops | Packets | Packet-weighted mean E2E |
| ---: | --- | ---: | ---: | ---: |
| 2 | `2>4>5>1` | 3 | 245 | 441.115101 s |
| 3 | `3>1` | 1 | 7,508 | 11.766925 s |
| 4 | `4>5>1` | 2 | 475 | 168.650200 s |
| 5 | `5>1` | 1 | 4,902 | 18.427855 s |
| 7 | `7>8>2>4>5>1` | 5 | 483 | 1,648.481810 s |
| 8 | `8>2>4>5>1` | 4 | 127 | 743.472364 s |

Across delivered packets, the packet-weighted 91.521046-second mean decomposes
exactly into 78.460522 seconds of NWK residence (85.73%) and 13.060524 seconds
from NWK-to-HOP handoff through the next NWK enqueue or final delivery
(14.27%). This packet-weighted value is intentionally distinct from the
91.403103-second arithmetic mean of the 95 aggregate bucket means quoted
above. The new steady-window bucket means are 22.694762 packets for NWK queue
size and 88.032154 seconds for NWK queuing delay. They are source-grounded
ns-3 diagnostics, not numeric OPNET parity results: the recovered multihop
PB/OV selection contains no NWK vectors or packet-path events.

In the retained pre-fix path run, only 1,330 delivered packets (9.68%) use
more than one hop, yet they contribute
85.79% of cumulative delivered delay. Source 7 alone contributes 63.32% while
representing 3.52% of deliveries. Mean per-visit NWK residence rises from
0.073 seconds at node 3 and 5.361 seconds at node 5 to 153.224 seconds at node
4, 287.907 seconds at node 2, and 919.847 seconds at node 8. Route discovery
does explain source 7's late first admission in that older ns-3 trajectory:
its five-hop gateway route is selected at 386.106 seconds, and the route set
stops changing by 389.955 seconds. Those timestamps are historical ns-3
diagnostics, not OPNET event truth; the recovered PB/OV selection contains no
route events.

The retained route-convergence checkpoint removes that startup admission lag. At
88.548912442 seconds, node 8 selects the direct route to destination 7 with
next hop 7, cost 7,450, path `7`, and reason `candidate update`. At
88.848492442 seconds, node 7 admits neighbor 8 and selects the direct route to
destination 8 with next hop 8, cost 7,450, path `8`, and reason
`neighbor admitted`. The already cached transit candidates become selectable
only when a fresh routing UPDATE from neighbor 8 is processed at the same
timestamp; the resulting gateway route to destination 1 has next hop 8, cost
37,250, and path `8>2>4>5>1`. The final global route change occurs at
92.514939163 seconds (node 3 to destination 7 via next hop 5, cost 37,250,
path `5>4>2>8>7`), so all 44 route changes complete before application traffic
starts at 300 seconds. These exact times are likewise ns-3 diagnostic evidence,
not OPNET route-event observations.

### Application/NWK/HOP admission-state diagnostic

The source-ordered ledger is now implemented as an opt-in observation surface.
It records every application gate decision, every NWK-to-HOP admission or
hold, the HOP admission snapshot, receiver ACK/DACK feedback, the exact NSDP
release, sender completion, and delayed HOP-capacity release. The snapshots
include route availability, NWK queue size, NSDP count/limit, global pending
DATA and allowance, per-neighbor outstanding/threshold/allowance, resend
state, and the correlated `(src,dst,sequence)` identity wherever the modeled
packet carries it. Enabling the ledger does not change modeled bytes, queue
order, route selection, random draws, or event scheduling.

That retained route-convergence 6,000-second trace has 3,271,308 events and
SHA-256
`64bf81c8b2155cd10e3688b468219c6d7d018dee73097c3cb0a45456339231f2`.
Its 1,710,000 application decisions reconcile exactly to the compact
diagnostics: 14,551 admissions and 1,695,449 `blocked_nsdp` decisions, with
zero discovery, empty-topology, gateway-route, or dynamic-destination blocks.
At the NWK boundary it records 18,713 admissions, 1,073,828
`neighbor_capacity` holds, 4,057 `global_capacity` holds, and zero `no_route`
holds. The neighbor holds identify the congested directed legs:

| Directed leg | `neighbor_capacity` holds |
| --- | ---: |
| `2>4` | 433,191 |
| `4>5` | 295,107 |
| `8>2` | 228,055 |
| `7>8` | 59,749 |
| `5>1` | 56,640 |
| `3>1` | 1,086 |

These hold totals count repeated queue-scan decisions, not unique packets or
time-weighted occupancy. HOP records 15,321 ACK completions, 2,076 DACK
completions, and 1,281 `no_ack` completions, with 2,070 delayed capacity
releases. Receiver feedback contains 16,200 ACK and 2,067 DACK decisions.
Strict admission analysis passes with zero invalid legs, zero global issues,
and zero pre-limit DACK findings. Strict path analysis also passes with 13,817
delivered chains, 734 valid incomplete prefixes, zero invalid packets, and
zero route-context mismatches. The incomplete inventory is 201 open in NWK,
471 final `no_ack`, 34 open resend, 24 completed ACK, 3 completed DACK, and 1
packet with no NWK observation.

That retained rerun's aggregate trace has SHA-256
`64bf81c8b2155cd10e3688b468219c6d7d018dee73097c3cb0a45456339231f2`,
and its generated ns-3 aggregate CSV has SHA-256
`059b326efa52f1526823d6dfac124b9c1c7e9ec8e642544a31e8f44b2037cfa3`.
The selected eight-series historical comparison still aligns all 800 points,
reports 665 exact-tolerance numeric mismatches, and preserves 20 OPNET
no-sample values; application packet size is exact in every measured bucket.
Its ns-3 means are 2.42516666667 packets/s sent, 2.30283333333 packets/s
received, and 74.979382019 seconds end-to-end delay, versus OPNET's 2.012,
1.9016667, and 112.748007455 respectively. Correct route convergence and
pre-traffic admission therefore do not by themselves establish aggregate
parity.

The 20-/40-second values above are the nominal custody intervals. Recovered
`br_hop.pr.c` schedules one DACK-list scan per entry at the nominal hold plus
one `TIC`; each scan releases all entries whose nominal deadline has passed.
ns-3 now preserves both the timer and list-scan ordering. An isolated entry
releases at `hold + TIC`, while entries less than one `TIC` apart can coalesce
with an effective offset in `[0,TIC]`. One source `TIC` is represented as
28 ns at ns-3's nanosecond resolution. Strict ledger analysis validates that
range and requires every release to coincide with a scheduled DACK timer, so a
bare nominal-only implementation does not pass. The subsequent HOP-origin NWK
wake is nominally one more `TIC` later and can coalesce with an already-pending
HOP wake. Its event handle remains distinct from NWK's local queue-check
handle, matching the two OPNET processes.

At the earlier resend/generic-wake checkpoint, compared with its
DACK-ordering predecessor, that correction left packet counts and the eight
core aggregate series unchanged while adding the source-required NWK scans.
This is historical checkpoint evidence; the retained route-convergence rerun
supersedes its admission, hold, packet-outcome, and aggregate counts above.

The retained pre-fix trace has SHA-256
`cf9392db5d10d47f5a7cc6459b00f828556cb72d5988e80e33a55cd0764e79ef`.
It contains 138 DACK decisions whose relay NSDP changes from 15 before enqueue
to 16 afterward. The recovered `br_hop.pr.c` and the campus archive's embedded
`br_hop.pr.m` both obtain the NSDP entry before sending to the NWK process and
then test that pre-event count. ns-3 now preserves that ordering: pre-count 15
ACKs and pre-count 16 DACKs.  ACK-marked duplicates do not enqueue again;
DACK-marked retries remain eligible for a fresh NWK enqueue and ACK/DACK
decision because the legacy duplicate test consults only the ACK register.

A subsequent commit-pinned 6,000-second trace at commit `05b6064` (SHA-256
`0582f1928c93cae9eeebf0a06c573535944e2ed293e9ae53be5e0aeeefd7a3fd`)
contained 9,400 sends and 8,680 delivery rows for 8,678 unique identities. The
two repeated identities were `8->1/10446` at 2136.682911086 and
2158.769911086 seconds, and `7->1/33041` at 4264.899911086 and
4266.953911086 seconds; both also had repeated relay enqueues and forwards.
That run stopped at aggregate validation because the older compact trace did
not retain HOP feedback, so those two cases cannot be retrospectively promoted
to source-exact duplicates. It remains noncertifying regression evidence. New
compact traces retain only the needed feedback rows and still fail closed when
the complete lineage is absent.

The retained instrumented rerun produced trace SHA-256
`263e954e3ba1054787c5a32e7386f619d38ee789276c68d0900585ef3b0211f8`.
Strict aggregation passed with 8,678 exact-sequence matches plus two
`source_exact_dack_retry_duplicate` matches, zero unmatched delivery rows, and
zero size mismatches across all 8,680 deliveries. Both proofs contain two
source-ordered relay-enqueue/feedback pairs, two DACKs, the same ingress HOP
sequence, and continuous source-valid NSDP increments at or above the DACK
limit (`16->17->18` and `32->33->34`). The workflow then stopped only at the
expected zero-tolerance aggregate comparison: 665 of 780 numeric points
differed, with 20 missing delay-bucket values skipped. Across the 100 buckets,
ns-3 averaged 1.566667 sent packets/s and 1.446667 received packets/s versus
OPNET's 2.012000 and 1.901667; across the 95 observed delay buckets, ns-3
averaged 192.361188 seconds versus OPNET's 112.748007 seconds. This is a
commit-pinned historical-aggregate checkpoint, not packet-level OPNET evidence.
It is also diagnostic rather than build-certified: this manifest hashes the
runner executable but not its dynamically loaded ns-3/CSR shared libraries or
the already-running workflow wrapper. A later harness-hardening commit should
record those behavior-bearing binaries before claiming a reproducible build
identity.

These counts are ns-3 source-ordered isolation evidence combined with a direct
comparison to recovered OPNET source. They are not an OPNET event-trace
comparison: the recovered PB/OV results contain aggregate vectors and no
corresponding admission ledger.

The hash-bound core-comparison traces retain complete correlation and
size-integrity provenance. These three traces predate the additive NWK/path
events and remain the inputs to the published core aggregate comparator
reports:

| Scenario | Trace events | Exact deliveries | Unmatched sends | Unmatched deliveries | Size mismatches |
| --- | ---: | ---: | ---: | ---: | ---: |
| `blue_radio_campus-2_nodes` | 1,974,900 | 980,842 | 737 | 0 | 0 |
| `blue_radio_campus-hidden_nodes_symmetrical` | 2,234,152 | 722,053 | 41,647 | 0 | 0 |
| `blue_radio_campus-multihop` | 191,591 | 13,740 | 1,000 | 0 | 0 |

The retained pre-fix generator diagnostics account for every scheduled
attempt before packet creation:

| Scenario | Attempts | Admitted | Discovery | Empty topology | Gateway route | Dynamic destination | NSDP full |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `blue_radio_campus-2_nodes` | 59,700,000 | 981,579 | 0 | 0 | 0 | 0 | 58,718,421 |
| `blue_radio_campus-hidden_nodes_symmetrical` | 149,250,000 | 763,700 | 0 | 0 | 0 | 0 | 148,486,300 |
| `blue_radio_campus-multihop` | 1,710,000 | 14,740 | 1,500 | 4,117 | 189 | 0 | 1,689,454 |

The retained route-convergence multihop row is 1,710,000 attempts, 14,551
admissions, zero discovery blocks, zero empty-topology blocks, zero
gateway-route blocks, zero dynamic-destination blocks, and 1,695,449 NSDP
blocks. It is intentionally kept separate rather than rewriting the provenance
of the earlier comparator manifest.

The compact trace SHA-256 values are, respectively,
`94091ec88155ab9c97b3a667b52f96cf516c618e5405e0533de5092a8bae494b`,
`a7b2bbc98ce4acd4c3d72145e6c7043bf143c7bfd4a8785ca0994a45afdbaf56`,
and `57721b50d4dcad31d7da36d79ad5f7bd24147ea54d255490c0dd3ac321d90070`.
The mandatory-hash canonical workflow was also executed end to end for the
multihop case; its scoped status was `selected_comparison_failed`, with all
four stages completing as designed and the comparator returning status 1.

The separate pre-fix path-enriched multihop diagnostic contains 395,412 events,
13,740 exact deliveries, 1,000 unmatched end-of-run sends, no unmatched
deliveries, and no size mismatches. Its SHA-256 is
`002097d468a54d94416923b8c75d56fe93217e2c69cb318a1d7c0b2593114e76`.
It was run through the scenario runner, strict packet-path analyzer, and ns-3
aggregate builder. It has not replaced the older mandatory-hash comparator
manifest or its 665-mismatch core report.

At exact tolerance, the two-node report contains 696 numeric mismatches, the
hidden-node report contains 700, and the multihop report contains 665 and
skips 20 aligned missing values; the sentinel-derived gaps are not failures by
themselves. Exact packet size in all three cases validates the vector decoder
and source-level modeled size, but it does not validate traffic generation,
reservation/ACK timing, routing, delay, or collision behavior.

The hidden-node run also shows a substantive ECC-behavior mismatch. Modeler's
`ECC.Traffic Dropped (packets/sec)` mean is 10.84105, while ns-3's
source-equivalent `rx_drop` `reason=ecc` mean is 0.6195167. Direct source
inspection rules out synchronization/acquisition and half-duplex drops: the
former enter `br_ecc` with `PK_ACCEPT=false`, and the latter encounters
`signal_lock=true`; neither increments the OPNET statistic. The large gap is
therefore a PHY/MAC behavior result to investigate, not an event-taxonomy
ambiguity or a tolerable numeric offset. Tolerances were not widened to turn
the measured difference into a pass.
