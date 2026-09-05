# OPNET scenario importer and differential harness

Updated: 2026-09-02

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

The recovered historical results include 11 complete Modeler `*.ov` databases:
127 aggregate vectors and 12,700 time buckets in total. The conservative
extractor validates vector offsets and extents, cross-checks identities and
aggregation types against the paired `*.pb.m` definitions, and preserves
Modeler's `2e+100` no-sample sentinel as a missing value. Two additional
partial fragments contain zero-length vector records and are correctly
rejected by strict extraction instead of being padded or guessed.

The nested latency database additionally contains six per-statistic
`All values` records with 31,758 interior time/value writes. Opt-in JSON can
retain those writes, but they contain no packet or tree identifier. Bucketed
aggregates and `All values` statistic writes can support historical traffic,
delay, size, and drop comparisons; neither is a chronological packet-event
stream that can expose packet-level MAC reservation ordering or same-time
collision decisions. An instrumented OPNET event export in CSV form is still
required for event-by-event certification.

## Canonical scenario schema

Every row uses schema `csr-opnet-scenario-v1` and one of three record types:

- `run`: scenario digest, duration, seed, TMM flag, coordinate scale,
  executable-backed application, MAC, and atomic HOP-security profiles, the
  optional source-executable SHA-256, and an optional reservation-control
  activation time.
- `node`: one imported CSR node, all calibrated radio/link attributes, and an
  optional controlled reservation slot.
- `flow`: one application generator, including fixed or live
  route/neighbor destination selection.

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
Gateway inference requires exactly one gateway and every generator must provide
promoted start, interval, and packet-size attributes. A missing value is an
error rather than a silently omitted flow. Compile-time application behavior
is not stored in `*.nt.m`, so `--application-profile` records which executable
era is being reconstructed:

- `current-send-only` leaves the gateway silent and retains the explicit
  `--gateway-flow-dscp` modeling choice;
- `legacy-send-only-no-dscp` leaves the gateway silent and forces DSCP 0; and
- `legacy-send-to-from-no-dscp` adds one gateway generator which chooses
  uniformly from its live route list, with a neighbor-list fallback, and
  forces DSCP 0.

The selected 2014/15 aggregate executables are content-addressed evidence:

| Scenario | Application profile | Executable SHA-256 |
| --- | --- | --- |
| `blue_radio_campus-2_nodes` | `legacy-send-to-from-no-dscp` | `d9ebf7626e641ee68ccc9b58b1bb4b28fc0906111762e4456a0e8c7a0bd8b055` |
| `blue_radio_campus-hidden_nodes_symmetrical` | `legacy-send-to-from-no-dscp` | `dd3f38e8d33700b61f9e360a737ba34e56cb75b2570eb2960a02de381ed0fff0` |
| `blue_radio_campus-multihop` | `legacy-send-only-no-dscp` | `adb97c54f7566439f1404e972d3d777a3bca613e2a965bf12f03353fb009d9af` |

Their bounded packet-generator disassembly contains no DSCP setter or DSCP
probability draw. For example:

```bash
python3 utils/import-opnet-scenario.py \
  historical-campus.zip imported-campus.csv \
  --scenario blue_radio_campus-2_nodes.nt.m \
  --infer-gateway-flows \
  --application-profile legacy-send-to-from-no-dscp
```

Only the newer current profile treats DSCP as a deterministic modeling choice.
The supplied newer `br_app` uses process-level `DSCP` and `DSCP_pct` inputs to
choose between that DSCP and zero probabilistically; the importer retains the
previous fixed default of 5 for backward compatibility and records it in every
flow row. None of the recovered scenario inputs supplies `DSCP_pct`, and no
authoritative OPNET-to-ns-3 RNG-stream mapping is available, so the current
runner does not claim percentage-mixture parity. It never substitutes the
fixed-DSCP assumption into either historical no-DSCP profile.

Project-directory labels such as `prior_to_dscp_slot_range_change` and
`started_dscp` are useful chronology hints, but neither label occurs in the
hash-bound archive used here. They therefore do not select behavior. The
hidden-node and multihop imports are classified as pre-DSCP because their exact
archived executables contain no application DSCP assignment/probability draw,
while the separately supplied newer `br_app.pr.c` does. This distinction keeps
DSCP from being used to explain a residual produced by either historical run.

### Executable-bound MAC profiles

The run row's `mac_profile` records the exact `get_slot()` family selected by
`--mac-profile`. It is bound to compiled-code evidence, not inferred from the
scenario name, node count, or topology: the same `*.nt.m` can be run with a
different compiled executable. The five accepted profile names are:

| MAC profile | Evidence binding | Slot-selection behavior |
| --- | --- | --- |
| `current-fine-free-slot` | Supplied newer `br_mac.pr.c`; no recovered executable SHA binding | Fine active-node range; draw a one-based free-slot ordinal, mark neighbor `rtslot_counter` values reserved, and walk the reservation table. With no reservations the support is `1..R`; reservations can shift the physical slot above `R`. |
| `hist-2014-coarse-inclusive-no-avoid` | `08c3647956a9dd31c03c9a2c7bac2cf00b62216968e08ee539b91bc327bafbf5` (`CSR_Validation-1_Node` and `2_Nodes_Latency`) | Coarse range from process-local `active_nodes`; draw uniformly in `0..R` inclusive, without reservation avoidance or probing. |
| `hist-2014-zero-based-rebuild-list` | `d9ebf7626e641ee68ccc9b58b1bb4b28fc0906111762e4456a0e8c7a0bd8b055` (`blue_radio_campus-2_nodes`) | Coarse range; rebuild an ordered `0..R-1` list and choose one list index uniformly. |
| `hist-2015-fine-one-based-table-no-avoid` | `dd3f38e8d33700b61f9e360a737ba34e56cb75b2570eb2960a02de381ed0fff0` (`blue_radio_campus-hidden_nodes_symmetrical`) | Fine range; index an ordered 255-entry table with a uniform integer in `1..R`, without reservation avoidance. |
| `hist-2014-next-tslot-modulo-probe` | `adb97c54f7566439f1404e972d3d777a3bca613e2a965bf12f03353fb009d9af` (`blue_radio_campus-multihop`) | Coarse range; draw in `0..R`, compare directly with each neighbor's `next_tslot`, and probe after a collision. |

For the three coarse profiles, the disassembled signed comparisons give
`R=31` for `N<=4`, `R=63` for `5<=N<=8`, `R=127` for `9<=N<=12`, and
`R=255` for `N>=13`; the unreachable-for-normal-input negative-node edge also
falls in the first branch. The fine mapping is exact:

| Active nodes `N` | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 or other |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Range `R` | 15 | 18 | 21 | 25 | 31 | 37 | 44 | 52 | 63 | 75 | 89 | 106 | 127 | 151 | 180 | 214 | 255 |

The four historical algorithms preserve their executable-specific endpoints
and defects:

- The operational coarse-inclusive executable has ELF Build ID
  `cbc38b846deb800c1df8d99c0f5628a68c3b24ee`. Its DWARF binds
  `get_slot@0x1e39f` to `/home/klee/op_models/blue_radio/br_mac.pr.c` lines
  1330--1350. All five production call sites pass process-state
  `active_nodes`, refreshed from the NWK interface immediately before the
  call; they do not substitute `max_reported_active_nodes`. The function
  returns `floor(uniform(R+1))`, giving exact support `0..R`, and reads no
  neighbor, reservation, or probing state. The focused ns-3 regression uses
  seed 1, automatic stream 0, and runs 5,
  47, and 899 to exercise occupied returns 0, 31, and 255 respectively. These
  are deterministic ns-3 branch/endpoint vectors, not a claim that Modeler
  and ns-3 share an RNG sequence. This profile is classified as source-exact
  behavior for the SHA-bound executable; it remains an explicit selection and
  is not the importer or runner default. Other archived operational builds
  implement different `get_slot()` families, so the date and scenario era
  alone are not sufficient evidence to select it.
- The zero-based executable creates exactly `R` entries with slot fields
  `0..R-1`, draws `floor(uniform(R))`, and consults no reservation state. Slot
  zero is selectable and `R` is not, despite the executable also recording
  `max_tslot=R+1`.
- The fine-table executable creates 255 entries with indexes and slot fields
  `0..254`, draws `j=floor(uniform(R))+1`, and returns table entry `j`. For
  `R<=254` its exact support is `1..R`; slot zero is excluded. Neighbor
  reservation flags and counters exist but `get_slot()` reads neither. Its
  five call sites all pass the process-state `rn_range` as `num_rslot`.
  `rn_range` is initialized to zero, has no later executable assignment, and
  is not present in the selected `*.nt.m` or `*.ef`, so the archived run uses
  the default fine mapping. A debugger-injected positive value after the first
  transmission would replace `R`, but that is not part of the archived run.
  When a draw selects index 255 (or a larger injected override reaches it),
  the historical code dereferences outside the table; this profile aborts
  explicitly instead of clamping, wrapping, or inventing a slot.
- The modulo-probe executable initially draws `floor(uniform(R+1))`, giving
  `0..R` inclusive, and returns immediately if no neighbor has that
  `next_tslot`. On a collision it advances with `(candidate+1)%R`, so the
  subsequent search is restricted to `0..R-1`: `R` can be returned only as a
  free initial draw, a collision at `R` advances to 1, and a collision at
  `R-1` wraps to zero. It uses direct neighbor comparisons, not a reservation
  array. If every slot in `0..R-1` is occupied, the historical loop never
  terminates even when `R` is free; this profile detects a complete modulo
  cycle and aborts explicitly instead of hanging.
- The same `adb97c54...` executable binds the range input to direct-link
  population. Its DWARF state lists `active_nodes` at `br_mac.pr.c:185`
  (offset 228) and `txslot` at line 186 (offset 232), but no
  `max_reported_active_nodes`; `get_slot@0x1f27e` has the single formal
  `nodes` at line 1513. All five call sites reload `nwk_IFptr->int_1` and pass
  that local value (`back2search@0x1d264/0x1d3a9`,
  `tslot_tasks@0x1f08a`, `prep_tx@0x1f6e2`, and
  `post_tx@0x2226f`). In the same executable,
  `proc_hello@0x2ff21..0x300ec` looks up or adds only the received packet
  source, raises `active_nodes` from that neighbor-list size plus one, and
  publishes it before route-payload processing begins at `0x300ee`
  (`br_nwk.pr.c:925-956`). Thus advertised transit path nodes remain routing
  records but cannot enlarge this MAC slot range. The supplied later source
  corroborates the direct-source/list-size sequence at `br_nwk.pr.c:936-965`
  and acquisition of the shared HOP `neighbor_lptr` at line 2071. That later
  `br_mac.pr.c` is a separate provenance boundary: its state does contain
  `max_reported_active_nodes`, and its call sites explicitly pass
  `max(local, reported)` (for example lines 1147-1150 and 2810-2814). Profiles
  representing the supplied later source retain that behavior. The hash-bound
  `hist-2014-next-tslot-modulo-probe` run uses the adb97 local-only input; the
  coarse-inclusive profile's separate local-only rule remains bound to its own
  recovered executable evidence above.

`csr-mac-slot-parity-smoke.cc` pins this provenance boundary with the campus
node-5 fixture: direct peers 1/3/4 plus self select `N=4`, `R=31`; transit-only
route records 2/7/8 remain stored but do not turn it into `N=7`, `R=63`. A
fourth directly heard peer deliberately crosses the coarse threshold to
`N=5`, `R=63`. It also verifies that unrelated executable profiles retain
their previous max-reported-population input. At the 13-ms slot period, the
incorrect `0..63` uniform draw has a 409.5-ms mean countdown, versus 201.5 ms
for the source-exact `0..31` draw: an avoidable 208-ms mean delay before each
new transmit opportunity.

For example, a hash-matched two-node import records both executable-era
choices:

```bash
python3 utils/import-opnet-scenario.py \
  historical-campus.zip imported-campus.csv \
  --scenario blue_radio_campus-2_nodes.nt.m \
  --infer-gateway-flows \
  --application-profile legacy-send-to-from-no-dscp \
  --mac-profile hist-2014-zero-based-rebuild-list
```

### Executable-bound HOP-security profiles

The run row's `hop_security_profile` is one atomic ordinary-DATA and ACK/DACK
selector. It prevents either half of the HOP wire contract from being silently
mixed into an archived-executable aggregate comparison. The importer writes
the selection and, for each archived profile, the full
`source_executable_sha256`; the aggregate workflow records both plus an
explicit DATA/ACK behavior projection.

| HOP-security profile | Evidence binding | Exact behavior |
| --- | --- | --- |
| `production-pairwise16` | Supplied production `packetTypes.c` maps AppData/AppDataNoAck and common `AckMsg` to Pairwise16 | A 192-byte network packet is 222 bytes on the OPNET wire; exact ACK is 30 bytes and cumulative ACK/DACK is 46 bytes. Authentication precedes state changes. |
| `hist-dd3f38e8-bare` | Archived campus hidden-node executable SHA-256 `dd3f38e8d33700b61f9e360a737ba34e56cb75b2570eb2960a02de381ed0fff0` | Ordinary DATA and cumulative ACK/DACK are bare. A configured 600-byte application packet becomes a 592-byte network packet and a 617-byte OTA DATA frame; ACK/DACK is 41 bytes. |
| `hist-adb97c54-bare` | Archived campus-multihop executable SHA-256 `adb97c54f7566439f1404e972d3d777a3bca613e2a965bf12f03353fb009d9af` | `br_Network -> br_Hop -> br_Mac` is direct and bare, producing 217 bytes for a 192-byte network packet. All five compiled ACK callers select both cumulative registers, producing bare 41-byte ACK/DACK. The 25-byte exact form is receive compatibility, not an observed archived transmit path. |

Select the historical boundary explicitly when importing that executable's
aggregate scenario:

```bash
python3 utils/import-opnet-scenario.py \
  historical-campus.zip imported-multihop.csv \
  --scenario blue_radio_campus-multihop.nt.m \
  --infer-gateway-flows \
  --application-profile legacy-send-only-no-dscp \
  --mac-profile hist-2014-next-tslot-modulo-probe \
  --hop-security-profile hist-adb97c54-bare
```

The hidden-node executable uses a distinct provenance name even though the
runner projects the same bare wire behavior:

```bash
python3 utils/import-opnet-scenario.py \
  historical-campus.zip imported-hidden.csv \
  --scenario blue_radio_campus-hidden_nodes_symmetrical.nt.m \
  --infer-gateway-flows \
  --application-profile legacy-send-to-from-no-dscp \
  --mac-profile hist-2015-fine-one-based-table-no-avoid \
  --hop-security-profile hist-dd3f38e8-bare
```

Omitting `--hop-security-profile` selects `production-pairwise16`. The exact
archived application/MAC/HOP tuple and its executable digest must be selected
together: the dd3 hidden-node tuple and adb multihop tuple cannot be crossed.
This is an executable provenance constraint, not a scenario-name, node-count,
or topology heuristic. Older CSVs may use `ack_envelope_profile` as a deprecated
input alias, but alias-only or missing-profile runs are diagnostic and cannot
become a final certificate. An alias-resolved historical run still requires
the full matching digest. Conflicting fields, any effective historical
selection without its digest, and any effective production selection claiming
one fail closed.

`--infer-ring-flows` remains a deterministic smoke-test surrogate. It is not
source-exact for these runs because the OPNET application obtains destinations
from runtime route-table state.

## Canonical event trace

The runner writes `csr-differential-trace-v1` rows with a monotonic event index
and simulation timestamp. Its current observation points are:

- scenario node/link construction and application sends;
- opt-in application, NWK, HOP, feedback, completion, and delayed-capacity
  admission events;
- MAC Idle/Search/Track/Tx state transitions;
- MAC reservation preparation, holdoff completion, per-slot countdown, and
  next-reservation advertisement;
- OTA transmission starts;
- PHY/MAC receive acceptance and rejection, including closure, receiver state,
  path loss, received power, noise, SNR, JSR, and interval error counts;
- NWK enqueue, actual next-hop forwarding, delivery, and selected-route
  changes, including route cost, hop count, and normalized path;
- packet identity, rate, modeled size, sequence, and security count wherever
  that information exists; and
- source-ordered discrete NWK queue-size/delay, HOP resend-size, MAC ACK-size,
  MAC Tx-size, and MAC Tx queuing-delay samples, encoded as
  `statistic_sample` rows with `node`, `statistic`, and `value` fields.

The trace writer contains observation only. Opening or closing it does not
change queueing, random draws, packet delivery, or simulator event scheduling.
Controlled reservation fields in a scenario are separate, explicit test
inputs and are disabled in ordinary imported scenarios.

`statistic_sample` is an additive v1 row type using `node`, `statistic`, and
`value`. Recognized samples dynamically activate the six queue series in the
aggregate builder; legacy traces without those columns retain the original
nine-statistic output. Strict aggregation requires finite nonnegative values
and integral queue-size samples. Aggregate-only v1 traces preserve the
original 12 columns in their original order and append `peer`, `next_hop`,
`route_cost`, and `detail`; repository readers are name-based. The compact
writer now retains `nwk_enqueue`, `nwk_forward`, and `route_change` in addition
to its earlier aggregate events. With `--admissionTrace=1`, it also retains the
seven admission-ledger event types documented below. Readers that require an
exact header width or an unchanged event set must account for those additions.

### Packet-path and NWK-residence ledger

`analyze-ns3-packet-paths.py` consumes either the full or compact v1 trace
header and emits a v2 path report. A strict branch-safe run must also enable
`--admissionTrace=1`: aggregate-only output without that surface has no tagged
`hop_admission` rows. Such a compact trace remains sufficient for ordinary
one-copy paths, but the analyzer rejects it as ineligible if repeated relay or
delivery observations require exact HOP-identity branch splitting. It never
infers that identity from FIFO order. The analyzer correlates ordinary
`(src,dst,sequence)` lifecycles and splits only source-proved repeated-DACK
copies. For a delivered copy it requires:

```text
app_send,(nwk_enqueue,nwk_forward)+,nwk_delivery
```

It rejects duplicate source admission, destination enqueue, loops, node or
next-hop discontinuities, negative intervals, malformed delivered chains, and
end-to-end decomposition errors. A simulation may legitimately stop with a
packet queued or handed to HOP, so a structurally valid undelivered prefix is
reported but does not fail strict mode.

```bash
python3 utils/analyze-ns3-packet-paths.py ns3-trace.csv \
  --summary-json packet-path-summary.json \
  --packet-csv packet-ledger.csv \
  --startup-time 300 \
  --bucket-width 60 \
  --stop-time 6000 \
  --strict
```

The JSON report contains overall and post-startup cohorts, flow/path
distributions, per-node NWK residence, route-context checks, exact 60-second
end-to-end buckets, and input/output hashes. The packet CSV contains the
complete path, each per-node residence, each forward-to-arrival leg, and the
exact end-to-end decomposition. Its `copy_index` distinguishes byte-identical
application copies, and reconstructed repeated-DACK rows use
`hop_lineage_json` to record every exact
`(src,dst,sequence,from,to,hop_sequence)` edge. HOP resend lifetime is
deliberately not added as another component because it overlaps MAC queuing,
transmission, and ACK service.

On the retained pre-fix canonical 6,000-second multihop run, strict analysis
accepted all 13,740 delivered lifecycles with zero invalid packets and
preserved 1,000
valid end-of-run prefixes. Its 95 nonempty end-to-end buckets were exactly
identical to the aggregate builder. This validates the ns-3 ledger and
isolates model behavior; it does not manufacture an OPNET packet path, because
the recovered PB/OV result contains bucket aggregates only.

### Application/NWK/HOP admission-state ledger

The admission ledger is disabled by default and enabled explicitly with
`--admissionTrace=1`. When enabled, the compact writer retains seven additional
source-ordered events. One narrow exception applies in ordinary aggregate-only
runs: `hop_feedback` is retained without enabling the other six ledger events,
because it is the minimum observation needed to distinguish a source-exact
lost-DACK retry fork from an arbitrary duplicate delivery.

| Event | Observation boundary |
| --- | --- |
| `app_admission` | Every generator attempt and its admitted or blocked application gate |
| `nwk_admission` | Every successful NWK-to-HOP transfer or hold due to route/global/per-neighbor state |
| `hop_admission` | Pending, outstanding, threshold, and resend state before and after HOP admission |
| `hop_feedback` | Receiver ACK/DACK choice, first-reception state, and NSDP count before/after relay enqueue |
| `nwk_nsdp_release` | Exact NSDP decrement and release cause |
| `hop_completion` | Sender ACK, DACK, or final no-ACK completion and capacity state |
| `hop_capacity_release` | Deferred 20-/40-second DACK hold expiry |

The events carry `(src,dst,sequence)` and directed-leg identity wherever the
modeled packet carries the observation tag. Detail fields preserve the live
queue, NSDP, global pending/allowance, per-neighbor
outstanding/threshold/allowance, resend, completion, and DACK-hold snapshots.
They add no modeled bytes and do not alter route lookup, queue order, random
draws, or event scheduling.

The aggregate analyzer accepts an additional delivery for one application
identity only when the trace contains a prior first-reception DACK and a later
first-reception ACK or DACK with the same relay, ingress peer, incoming HOP
sequence, NWK source/destination, and observation tag. Each feedback consumes
one prior, unconsumed matching relay `nwk_enqueue`; unrelated statistic samples
may occur between those rows, while conflicting same-identity enqueue or
feedback rows invalidate the pairing. Both feedback rows show the source-exact
NSDP-16 increment. An ACK closes that lineage. The original application send
timestamp is reused for every such proven copy, matching legacy APP delay
sampling. Repeated ACKs, changed lineage fields, malformed NSDP transitions,
and duplicates without feedback remain strict failures.

The admission-ledger analyzer binds concurrent same-packet/node legs by full
packet and directed-HOP identity, so completions and deferred DACK releases do
not overwrite another active leg. It still reports packet-identity
`duplicate_delivery`. The packet-path analyzer now expands a repeated-DACK
fork only after each admission and feedback joins on exact
`(src,dst,sequence,from,to,hop_sequence)` identity, each extra reception passes
the source-exact pre/post-NSDP proof, every relay enqueue and delivery is
covered, and every enqueue-to-forward edge agrees within 1 ns with the direct
`NWK.Network Queuing Delay (sec)` sample on the immediately preceding CSV row.
That sample must also have the consecutive event index and the same node and
timestamp as the forward. The resulting queue-copy graph must have exactly one
complete matching: the analyzer removes each selected edge and rematches to reject any
alternate saturation. This is not FIFO, and reversing candidate enumeration is
pinned to the same output for the observed case in which HOP sequence 990 is
acknowledged before 989. Missing, displaced, wrong-node, or inconsistent delay
evidence, non-unique matching, ordinary repeated ACKs, changed HOP identity,
malformed NSDP transitions, orphan suffix rows, and incomplete graph coverage
remain strict failures.

Replaying the frozen release trace with SHA-256
`1f18699e67cb3d57906c04f29cb305e6fd7aa10913aa00f74fb97aef05e98863`
now passes strict packet-path analysis. It retains 9,158 application packet
keys, expands the eight proved DACK forks into 9,166 lifecycle copies, closes
all 8,733 deliveries, and classifies 433 end-of-run copies as valid incomplete
prefixes, with zero invalid copies or route-context mismatches. All 100
end-to-end buckets are exactly equal to the aggregate builder, including the
seven extra delivered copies; the eighth fork remains queued at the stop.

Run the imported scenario with the opt-in surface:

```bash
../ns-3-dev/build/scratch/ns3-dev-csr-opnet-scenario-runner-default \
  --scenario=imported-campus.csv \
  --trace=ns3-admission-trace.csv \
  --appDiagnostics=app-admission-diagnostics.csv \
  --stop=6000 \
  --aggregateTraceOnly=1 \
  --admissionTrace=1 \
  --quietModelLogs=1
```

The runner defaults to the source-declared `normal(-11, 0.25)`
synchronization threshold, where 0.25 is variance in dB squared. The optional
`--stochasticSyncThreshold=0` switch is a deterministic-mean compatibility
mode for replaying pre-variance checkpoints; it is not the source-exact
default. Devices receive consecutive two-stream blocks in imported node order,
keeping the duty-phase and synchronization sequences reproducible under the
scenario seed and ns-3 run number.

Then validate and summarize the ledger:

```bash
python3 utils/analyze-ns3-admission-ledger.py ns3-admission-trace.csv \
  --app-diagnostics app-admission-diagnostics.csv \
  --summary-json admission-summary.json \
  --legs-csv admission-legs.csv \
  --strict
```

Strict mode is a source-semantics assertion, not merely a CSV parser. The
pre-enqueue ACK/DACK correction now passes this assertion. The retained
pre-fix trace still exits nonzero, including on its 138 early-DACK boundary
decisions; its former bare-deadline DACK releases are also reported. It remains
useful before/after evidence even though those older rows predate the explicit
scheduled/effective-offset trace fields.

The analyzer reconciles every `app_admission` decision with the compact
per-flow diagnostics, validates NWK and HOP state transitions, correlates
feedback, NSDP release, completion, and DACK expiry for each directed packet
leg, and writes both a bounded JSON summary and a packet-leg CSV. Structurally
valid open NWK, resend, and DACK-hold states are inventoried rather than
silently converted to failures.

The exact-order canonical 6,000-second multihop trace contains 3,271,308
events and has SHA-256
`64bf81c8b2155cd10e3688b468219c6d7d018dee73097c3cb0a45456339231f2`.
The paired application diagnostics have SHA-256
`9844d77e33f0f7a8a2e222d5ee8c32816b1d2acd9fb1f3abcdea621a554eb76c`,
and the strict admission summary has SHA-256
`4e495ab06071b6b7f94e864cf664f43eaf3fab815cf964482eaec6f306d6069a`.
The application ledger exactly partitions 1,710,000 attempts into 14,551
admissions and 1,695,449 NSDP blocks. The application diagnostics record zero
destination, discovery, empty-topology, or gateway-route blocks, including no
discovery, topology, or gateway-route blocks after traffic starts. NWK
decisions are:

| NWK decision | Count |
| --- | ---: |
| Admitted | 18,713 |
| `neighbor_capacity` | 1,073,828 |
| `global_capacity` | 4,057 |
| `no_route` | 0 |

The per-neighbor holds are 433,191 on `2>4`, 295,107 on `4>5`, 228,055 on
`8>2`, 59,749 on `7>8`, 56,640 on `5>1`, and 1,086 on `3>1`. Those hold counts
are repeated queue-scan decisions rather than unique packets or time-weighted
occupancy. There are zero exact-same-time duplicate NWK hold scans in this
trace after removing the route-, admission-, and discovery-triggered NWK-local
wakes. HOP also records 18,713 admissions. Completion counts are 15,321 ACK,
2,076 DACK, and 1,281 final `no_ack`; 2,070 delayed DACK-capacity holds expire.

Receiver feedback contains 16,200 ACK and 2,067 DACK decisions. Every DACK
uses a pre-enqueue NSDP count of at least 16: both the analyzer's pre-limit
finding count and its 15-to-16 DACK boundary count are zero. The strict
packet-path result also passes with 13,817 valid deliveries, 734 valid
incomplete prefixes, zero invalid packets, and zero route-context mismatches.
The joined inventory classifies those prefixes as 201 open in NWK, 471
completed by final `no_ack`, 34 open resends, 24 completed ACKs, 3 completed
DACKs, and 1 packet with no NWK observation.

The convergence trace records node 7 admitting direct neighbor 8 at
`88.848492442 s` with one-hop cost 7,450, followed at the same timestamp by
transit-candidate updates for destinations 2, 4, 5, and 1. The resulting
gateway route uses next hop 8, normalized path `8>2>4>5>1`, and cost 37,250.
This timestamp is an exact ns-3 convergence diagnostic only; without a
corresponding chronological OPNET event export it is not convergence-time
parity evidence.

The pre-fix trace contained 3,228,471 events with SHA-256
`cf9392db5d10d47f5a7cc6459b00f828556cb72d5988e80e33a55cd0764e79ef`.
Its 138 pre-15/post-16 DACK findings establish the behavioral delta; they are
not relabeled as passing evidence.

The recorded `dack_hold_seconds` values remain the source's nominal 20-/40-
second custody intervals. Each entry schedules a list-wide expiry scan one
OPNET `TIC` later. Since `TIC = 1 / 36 MHz`, ns-3's nanosecond resolution
represents the scheduled offset as 28 ns. A scan releases every nominally
expired entry, so nearby expiries can coalesce with an effective offset in
`[0,TIC]`. Trace details distinguish
`dack_scheduled_timer_offset_seconds` from
`dack_effective_timer_offset_seconds`; strict ledger analysis requires the
release to remain in range and to coincide with an actual scheduled DACK
timer. Focused regressions cover isolated 20- and 40-second holds, reject the
former bare-deadline behavior, exercise a coalesced two-entry scan, and prove
integrated isolated NWK readmission at `hold + 2*TIC`. As in OPNET, an already
pending HOP-origin queue wake can coalesce the final one-`TIC` wake. The HOP
and NWK processes retain separate pending-event handles, so an NWK-local queue
check does not suppress or postpone that remote wake.

### Resend expiry and HOP-origin queue wakes

The resend path now preserves the recovered `br_hop` timer boundaries and
same-time scan order. Every matching MAC sent indication records the actual
transmission time and installs an independent list-wide resend scan. Its own
normal-retry scan runs at that sent time plus two seconds and one OPNET `TIC`.
After the second retry, its own final no-ACK scan runs from that retry's actual
sent time plus four seconds and one `TIC`. An earlier list-wide timer may still
process an entry once its nominal predicate is due. Each scan first completes its
entire final-deletion pass, including capacity and tagged NSDP releases, and
only then completes a second pass that queues eligible retransmissions. Thus a
final expiry and another packet's retry at the same timestamp cannot reverse
the source's release-before-requeue ordering.

The source retains only the most recently installed resend-timer handle but
does not cancel older timer events. Consequently, any timer callback scans the
whole resend list and may process other entries whose nominal deadline has
already passed. A MAC sent indication with no matching resend entry also arms
the source's fallback global scan when that retained handle is no longer
pending, including the resend-queue-overflow case. Final deletion always
requests an NWK queue wake, even when the expired frame carries no NSDP or
routing metadata.

HOP owns one remote NWK-wake event distinct from NWK's local queue-check event.
NWK-owned queue wakes now come only from application enqueue and relay
enqueue; route changes, route admission, and discovery transitions do not arm
an NWK-local wake. HOP-origin generic wakes retain their source ordering: they
run one `TIC` after the first request, coalesce only with another pending
HOP-origin request, and are not postponed by later requests. Every readable,
locally addressed ACK or DACK requests this wake after feedback processing,
including unknown, stale, duplicate, zero-bit cumulative, and other no-op
feedback. A readable ACK/DACK for another HOP destination does not. Final
no-ACK expiry and DACK-capacity expiry use the same HOP-owned wake path, so the
former wakes NWK one `TIC` after deletion and the latter one `TIC` after its
already delayed capacity release.

The focused ns-3 regressions all pass and make those boundaries executable
evidence:

- `csr-hop-mac-sent-time-smoke` rejects retry at the bare two-second boundary,
  requires both retries at their actual sent time plus `2 s + TIC`, rejects
  final deletion at the bare four-second boundary, and requires deletion at
  `4 s + TIC` followed by the NWK wake one more `TIC` later. It also exercises
  the unmatched-sent fallback and proves that a same-scan final deletion
  completes before another entry is requeued.
- `csr-ack-window-smoke` proves that Pairwise16-authenticated ordinary,
  unknown, stale, DACK, and zero-bit cumulative feedback use the delayed
  generic wake; requests within one `TIC` coalesce without postponement, while
  wrong-destination feedback produces no wake. It also proves that the
  source-disabled single-DACK path leaves a known resend entry and its pending
  MAC copy untouched.
- `csr-hop-dack-hold-smoke` keeps the independent 20-/40-second DACK-release
  timing checks and additionally requires feedback wakes at receipt plus one
  `TIC`, expiry wakes at nominal hold plus two `TIC`s, and coalescing for
  nearby feedback and expiry requests.
- `csr-mac-hop-queue-limit-smoke` confirms that an ACK for the untracked 513th
  overflow frame still produces its delayed generic wake without repairing
  the source-equivalent pending or per-neighbor count mismatch.

Recovered `br_hop.pr.c` obtains the NSDP entry, sends the relay packet to the
separate NWK process, and tests the still-pre-enqueue count: 15 selects ACK
and 16 selects DACK. ns-3 now makes that decision before its synchronous NWK
delivery callback. Explicit smoke fixtures cover pre-15/post-16 ACK,
pre-16/post-17 DACK, ACK-marked duplicate plain-ACK/no-second-enqueue
behavior, DACK-marked retry re-enqueue/reassessment, and first-reception
no-route ACK suppression.

That retry can source-exactly create a second downstream delivery: legacy NWK
has no DATA duplicate table, assigns each relay copy a fresh onward HOP
sequence, and delivers every destination copy to APP. A three-hop fixture now
proves two byte-identical ingress receptions, two DACK decisions, two relay
custody entries, distinct onward HOP sequences, and two destination APP
callbacks. The aggregate analyzer accepts the extra delivery only with the
complete enqueue/feedback proof described above. Admission-ledger analysis now
binds distinct overlapping HOP legs but still reports the packet-identity
duplicate delivery. Packet-path analysis binds each proved copy to its exact
directed-HOP suffix and treats the eight recovered forks as two valid lifecycle
copies apiece. It does not generalize that exception to a packet-wide duplicate
budget: a duplicate without complete HOP/enqueue/feedback coverage remains
fatal.
Reaccepting a Pairwise16-authenticated retry from the DACK bitmap is inferred
cross-source integration; the downstream lost-DACK delivery fork itself is
source-exact.

The ledger is ns-3 isolation evidence interpreted against recovered OPNET
source. It is not an OPNET event comparison. No authoritative OPNET
application/NWK/HOP event trace exists in the recovered PB/OV results.

OPNET CSV exports may use canonical names or common aliases such as `Time`,
`Action`, `Node ID`, `Tx Node`, `Pkt Type`, `Seq`, `SNR`, and `Accepted`. Event
aliases such as `TX`, `Receive`, `Drop`, and `Delivery` are normalized before
alignment. Rows must remain in nondecreasing timestamp order.

## Comparison behavior

Events are aligned in order using their mutually available identity fields.
The default identity is event, receiver node, peer, packet type, source,
destination, sequence, and statistic name; optional identity fields are
omitted when one input does not provide them. Matching rows compare every
field present in both traces.

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

This generic event workflow is deliberately diagnostic, not an OPNET-origin
certificate. `--opnet-manifest` is optional so legacy and synthetic traces can
still exercise the comparator. When supplied, it must be an exact
`csr-opnet-instrumentation-v1` manifest: the workflow validates its schema,
generator and source digests, generated-file locations and hashes, activation
contract, and trace schema. That establishes a self-consistent source bundle,
but the exported CSV has no cryptographic run identifier tying it to that
bundle, so the manifest records `trace_origin_bound=false` and
`certifying=false`. When omitted, the manifest classifies the trace as
`not_supplied_legacy_or_synthetic_trace`. Event-order certification belongs to
the reservation/collision validator, whose licensed-run evidence contract
binds the trace and execution manifest explicitly.

The output directory contains:

- the raw ns-3 event trace and console log;
- normalized OPNET and ns-3 traces;
- `differential-report.json`, with counts and bounded detailed differences;
- `run-manifest.json`, with input hashes, executable path, commands, and exit
  statuses.

The workflow treats all inputs and tools as immutable across stages. It also
checks the output-directory identity and the exact set of managed artifacts
before and after each subprocess and before every parent write. Symlinks,
hardlinks, special files, aliases to protected inputs, and artifacts created
ahead of their owning stage fail closed. This prevents a child runner or
comparator from redirecting a later write through a managed-output path. A
hostile process that can exchange a regular path concurrently inside the C++
runner's own file-open interval remains outside this user-space evidence
boundary.

Focused repository fixtures exercise binary promoted-attribute decoding,
shifted/mobile schemas, DES parsing, gateway-flow inference, alias
normalization, timestamp tolerance, missing-event reporting, and both complete
runner/comparator workflows.

`utils/testdata/opnet-trace-fixture.csv` is a synthetic alias-schema mirror
used only to test that end-to-end runner/comparator plumbing.  Its time-zero
gateway `route_change` row mirrors the source-owned self-capability event that
the live runner now emits during scenario construction.  It is not an exported
OPNET packet/event trace and does not provide differential parity evidence.

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
The ns-3 aggregation provenance must name the canonical aggregate output path
as well as its SHA-256. The same immutable-input and staged managed-output
checks described above surround the extractor, runner, aggregator, and
comparator; path-resolution errors fail closed.
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
| `blue_radio_campus-2_nodes` | 800 | All 100 buckets | Sent 16.4417 vs 16.3597 packet/s; received 16.4373 vs 16.3474 packet/s; delay 1.45125 vs 1.42737 s | 696 numeric mismatches; no missing or extra points |
| `blue_radio_campus-hidden_nodes_symmetrical` | 800 | All 100 buckets | Sent 12.1707 vs 12.7283 packet/s; received 11.4370 vs 12.0342 packet/s; delay 9.17976 vs 8.40223 s | 700 numeric mismatches; no missing or extra points |
| `blue_radio_campus-multihop` | 800 | All 95 measured buckets; 5 OPNET no-sample buckets preserved | Sent 2.0120 vs 2.42517 packet/s; received 1.90167 vs 2.30283 packet/s; delay 112.748 vs 74.9794 s | 665 numeric mismatches; no missing or extra points; 20 missing values skipped |

These failures were not hidden by widening tolerances. The exact packet-size
matches validate vector decoding and the source-level modeled size, while the
remaining bucket-by-bucket traffic and delay differences identify real
model-calibration work. The two-node and hidden-node mean values are now close
under their executable-bound application and MAC profiles. Multihop's
remaining rate and delay residual was therefore taken through the recovered
queue/service diagnostic surface.

The post-fix multihop queue run aligned all 400 bucket positions for HOP
resend size, MAC ACK size, MAC Tx size, and MAC Tx queuing delay. Of those
positions, 394
contained numeric values on both sides and all 394 differed at exact
tolerance; four OPNET no-sample values were skipped and two additional ns-3
values were missing. For bucket ends strictly after 300 seconds, ns-3 is
19.13% high in HOP resend size, 14.23% low in MAC ACK size, 16.87% high in MAC
Tx size, and 12.39% high in MAC Tx queuing delay. Because MAC queuing delay is
higher while end-to-end delay is lower, the path ledger remains the stronger
diagnostic boundary. It finds one stable loop-free route per source, zero
route-context mismatches, 13,817 valid deliveries, and 734 valid incomplete
prefixes. Its packet-weighted 74.7152 seconds decomposes exactly into 61.7828
seconds of NWK residence and 12.9324 seconds of post-NWK leg service/transit.

The completed admission ledger records 1,073,828 per-neighbor holds and 4,057
global holds, led by 433,191 on `2>4`, 295,107 on `4>5`, and 228,055 on
`8>2`, with no no-route holds. All 2,067 receiver DACK decisions use a
pre-enqueue NSDP count of at least 16. Strict admission analysis now passes
with no invalid legs or global issues; the retained pre-fix trace remains the
evidence for the corrected 138 early-DACK decisions.

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
