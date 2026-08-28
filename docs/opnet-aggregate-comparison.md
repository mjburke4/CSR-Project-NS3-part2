# OPNET aggregate comparison

Updated: 2026-08-28

## Purpose and evidence boundary

`utils/compare-opnet-ns3-aggregates.py` compares historical OPNET output-vector
series with equivalent ns-3 aggregate series. It is deliberately separate from
the packet/event comparator: bucketed output vectors can validate traffic,
drop, delay, and queue statistics, but they cannot recover chronological MAC
reservation or collision events that were not recorded by Modeler.

The recovered archive contains 10 complete Modeler 17.1 `*.ov` files. The
read-only extractor decodes 109 vectors and 10,900 buckets across those runs
and cross-checks their labels and aggregation identities against paired
`*.pb.m` definitions. Two additional partial fragments contain zero-length
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

Manifest success is `selected_comparison_passed`, never a claim of whole-model
OPNET parity. `evidence_scope` is
`historical_bucket_aggregates_not_event_parity`. Exact tolerances, the full core
statistic set, the full time window, normal OPNET behavior flags, and no legacy
size adjustment form the `canonical` profile; any supported override is listed
and marks the run `diagnostic`. The default statistic set intentionally
excludes an unproven mapping of OPNET ECC drops to ns-3 `reason=ecc`: earlier
ns-3 receive stages can also account for collision loss, so those counters
must be reconciled before they are called equivalent.

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

## Measured exact-tolerance results

The three completed full-duration core-statistic comparisons align all 800
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

These runs use the hash-bound historical application and MAC profiles, FIFO
DSCP 0, source-ordered generator gates, and the common 0.988-second wake
phase. The two-node and hidden-node headline means are now close without a
tolerance adjustment. Multihop remains materially high in traffic and low in
delay, so the four recovered HOP/MAC queue vectors were compared next rather
than changing slot policy again.

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
does explain source 7's late first admission: its five-hop gateway route is
selected at 386.106 seconds. The route set stops changing by 389.955 seconds,
however, and every delivered source uses one loop-free path thereafter.

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

The exact-order canonical 6,000-second multihop ledger has 2,875,403 events
and SHA-256
`4f85672a78044db4176a0866dc2e225e4f4652e95b5aa636a1b1f7234518a8ef`.
Its 1,710,000 application decisions reconcile exactly to the compact
diagnostics: 14,566 admissions and 1,695,434 source-ordered blocks. At the NWK
boundary it records 18,331 admissions, 686,581 `neighbor_capacity` holds,
3,251 `global_capacity` holds, and zero `no_route` holds. The neighbor holds
identify the congested directed legs:

| Directed leg | `neighbor_capacity` holds |
| --- | ---: |
| `2>4` | 335,294 |
| `4>5` | 168,733 |
| `8>2` | 122,552 |
| `5>1` | 31,065 |
| `7>8` | 28,130 |
| `3>1` | 807 |

These hold totals count repeated queue-scan decisions, not unique packets or
time-weighted occupancy. Their directed-leg distribution identifies the gate
that repeatedly prevents progress.

HOP records 14,892 ACK completions, 2,162 DACK completions, and 1,244
`no_ack` completions; 2,156 delayed DACK-capacity holds expire. Receiver
feedback contains 15,747 ACK and 2,161 DACK decisions. Strict admission
analysis passes with zero invalid legs, zero global issues, zero pre-limit
DACKs, and zero 15-to-16 DACK decisions. Strict path analysis also passes with
13,854 delivered chains, 712 valid incomplete prefixes, and zero invalid
packets. The incomplete inventory is 220 open in NWK, 443 final `no_ack`, 32
open resend, 12 completed ACK, and 5 completed DACK packets.

The corrected aggregate CSV has SHA-256
`35b0b694322186ff217d1612877f257cd11321e7bed11ee3e9ceb3a810936b8b`.
The selected eight-series historical comparison still aligns all 800 points,
reports 665 exact-tolerance numeric mismatches, and preserves 20 OPNET
no-sample values. Its corrected ns-3 means are 2.42767 packets/s sent, 2.30900
packets/s received, and 79.9450 seconds end-to-end delay, versus OPNET's
2.01200, 1.90167, and 112.748 respectively. Thus clearing the source-order
violation does not by itself establish aggregate parity.

The 20-/40-second values above are the nominal custody intervals. Recovered
`br_hop.pr.c` schedules one DACK-list scan per entry at the nominal hold plus
one `TIC`; each scan releases all entries whose nominal deadline has passed.
ns-3 now preserves both the timer and list-scan ordering. An isolated entry
releases at `hold + TIC`, while entries less than one `TIC` apart can coalesce
with an effective offset in `[0,TIC]`. One source `TIC` is represented as
28 ns at ns-3's nanosecond resolution. Strict ledger analysis validates that
range and requires every release to coincide with a scheduled DACK timer, so a
bare nominal-only implementation does not pass. The subsequent NWK check is
nominally one more `TIC` later but can coalesce with an already-pending check,
matching OPNET's pending-event guard.

Compared with the preceding pre-enqueue-only checkpoint, the corrected timer
leaves all packet counts and all eight core aggregate series unchanged. It
changes 95 MAC and 95 NWK queue-delay bucket means by less than 5.6 ns, which
is the expected queue-residence redistribution from the added event quantum.

The retained pre-fix trace has SHA-256
`cf9392db5d10d47f5a7cc6459b00f828556cb72d5988e80e33a55cd0764e79ef`.
It contains 138 DACK decisions whose relay NSDP changes from 15 before enqueue
to 16 afterward. The recovered `br_hop.pr.c` and the campus archive's embedded
`br_hop.pr.m` both obtain the NSDP entry before sending to the NWK process and
then test that pre-event count. ns-3 now preserves that ordering: pre-count 15
ACKs, pre-count 16 DACKs, and duplicates do not enqueue again.

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

The post-fix multihop row is 1,710,000 attempts, 14,566 admissions, 2,000
discovery blocks, 4,112 empty-topology blocks, 462 gateway-route blocks, zero
dynamic-destination blocks, and 1,688,860 NSDP blocks. It is intentionally
kept separate rather than rewriting the provenance of the earlier comparator
manifest.

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

The hidden-node run also confirms why ECC drop accounting is excluded from the
default set. Modeler's `ECC.Traffic Dropped (packets/sec)` mean is 10.84105,
while the narrow ns-3 `rx_drop` `reason=ecc` mean is 0.6195167.
The ns-3 trace separately records 487,751 `prior_stage` and 223,477 `state`
drops. That is an unresolved event-identity mapping, not a tolerable numeric
offset; prior-stage collision rejections must be mapped from source semantics
before the counters can be called equivalent. Tolerances were not widened to
turn any of these measured differences into a pass.
