# OPNET parity audit

Updated: 2026-09-01

## Scope and confidence limits

Battery/energy and supervisory-layer behavior are excluded because they were
experimental and were not part of the deployed system. BBN routing is also
excluded; the target is the `__ARL_ROUTING__` configuration selected by the
supplied `csr_api.h`. This audit compares the supplied OPNET process, pipeline,
ARL API, and packet-model files with the ns-3 model.

`br_nwk.pr.c` delegates core routing decisions to an external routing library
through functions such as `routesCreate`, `routesRcvMsg`, `routesProcess`, and
`routesValidRx`. The later-supplied `routes(1).c` now provides the corresponding
ARL implementation, including automatic changed-route fanout, routing requests,
and full INFO/UPDATE/FLUSH snapshots. Together with `csr_api_routes.h`,
`csr_api_linkchar.h`, and the original `br_*.pk.m` definitions, this supports a
source-backed routing audit. The later-supplied `arlSecurity.c`,
`arlSecurity.h`, `packetTypes.h`, `msectime.*`, and split
`arlAuthTag.c`/`arlContext.c`/`arlEncrypt.c`/`arlKey.c` sources establish the
KeyRequest/KeyUpdate security records, exact enabled cryptographic transforms,
millisecond timer units, the ACK-driven group-key lifecycle, all three
pairwise and all three group security modes, and the source's key-evolution
rules. The later-supplied node/network models, DES configurations, modulation
tables, and output-vector files additionally establish radio defaults,
scenario overrides, the BER-table inventory, and aggregate receiver-drop
baselines. Exact end-to-end certification still requires authoritative
OPNET/ns-3 event-trace comparison.

## Current status

| Area | Estimated parity | Evidence and remaining uncertainty |
| --- | ---: | --- |
| Visible NWK wrapper behavior | 95-98% | Queue order/timing, blocked-entry scanning, DATA reliability, NSDP, reverse routes, source-owned self capability, source-exact same-cycle grouped changed-route fanout, local-only link cost, gateway-driven SNMP discovery coordination, relay-holdoff/clear state, active-node accounting, and fixed packet/control envelopes are covered. |
| Full NWK routing behavior | 90-94% | ARL byte-stream exchange, self-route capability advertisement, same-pass INFO/changed-route propagation with NWK-owned residual-destination retry, 24-bit node identifiers, exact supplied `br_Routes`/`br_SNMP` fixed fields, and no-static-route multi-hop convergence now have source-backed coverage. |
| ARL neighbor admission | 88-94% | Inactive/active state is now separate from freshness; two-sided key completion, deferred NeighborCheck proof, DATA gating, RoutingUpdate handling, security-count reset, and 5-second retry foundations are source-backed. Loss/restart edge cases still need broader trace coverage. |
| HOP | 93-96% | ACK/DACK windows, source-exact Pairwise16 ACK/DACK wrapping, exact DACK expiration and generic-wake ordering, actual-MAC-sent resend/final-expiration timing, flow control, queue limits, grouped routing ACKs, retry DSCP, custody, overhearing, link-control export, reliable KeyUpdate completion, authenticated Discover/routing/DATA/neighborcast paths, one-hop SNMP dispatch ordering, exact fixed HOP/ACK envelope models, and OPNET DATA-versus-control timeout effects are covered. |
| Full hop security | 89-94% | Pairwise16, Pairwise32, Pairwise32Encrypt, KeyRequest, KeyUpdate, GroupEstablish, Group16, and Group32Encrypt now have source-faithful KDF, HMAC, AES, key-rotation, replay, and golden-record coverage. Ordinary DATA and packet-type-0 ACK/DACK use Pairwise16, an explicit encrypted DATA path uses Pairwise32Encrypt, and AppNeighborcast uses live Group32Encrypt with authentication before state mutation or delivery. Remaining uncertainty is exact security wrapping for other control packets, distinct network-security modes, differential traces, and operational key-store hardware. |
| MAC transmit/control | 90-95% | Slot selection, holdoff, ACK priority, queue limits, exact OPNET segment-size accounting and airtime, concatenation, rate/power aggregation, preamble selection, freshness, Tx occupancy, and RX-induced countdown freezing are covered. |
| Full MAC including receive contention | 89-94% | Idle/Search/Track/Tx state, 6.63-ms acquisition, overlapping-signal selection, rate-aware interference, table/ECC-based collision outcomes, half duplex, slot freezing, and long-preamble sleep/wake behavior are reproduced. Remaining uncertainty is concentrated in stochastic synchronization and differential traces. |
| PHY | 84-91% | Airtime, receiver/channel qualification, source-exact non-TMM three-path power, band overlap, gains, bandwidth-scaled noise, mixed-/same-rate interference, exact modulation curves, interval errors, ECC, closure ordering, final accept/drop, and promoted scenario radio/link attributes are live. Remaining uncertainty is threshold variance, opaque off-grid interpolation, external LOS/TMM closure, and authoritative trace calibration. |

Overall estimates:

- NWK/HOP/MAC protocol-control behavior excluding PHY: 92-96% complete.
- Whole in-scope radio behavior including PHY, excluding battery and BBN:
  86-91%
  complete.

These ranges reflect confidence in observable behavior, not code-volume
completion.

## Completed or close

- Strict NWK positive-DSCP head insertion and best-effort tail insertion.
- Source-owned NWK queue wake semantics: local and relay enqueue paths request
  the NWK process's own queue check at `+TIC`, while HOP feedback, DACK expiry,
  and final resend expiry request the separate coalescing HOP wake. Route
  receipt, neighbor activation, and discovery completion do not themselves
  wake a packet already held in NWK, and DATA never starts implicit discovery.
- OPNET-global ACK/resend behavior for NWK DATA.
- HELLO role metadata preservation and monotonic active-node population,
  separated from routing capability. A direct route begins at capability zero;
  only the reporter's source-owned routing UPDATE establishes capability 1/2.
- OPNET ARL local-only link-cost calculation: RoutingInfo is retained as
  transaction data but does not create non-legacy bidirectional negotiation.
- NWK/HOP admission, NSDP ACK/DACK selection, and global pending-DATA state.
- Exact DACK custody ordering: nominal 20-/40-second holds, one per-entry timer
  at `hold + TIC`, list-wide expiry scans, and source-equivalent pending-event
  coalescing for the subsequent HOP-owned NWK queue wake.
- Source-backed ARL neighbor admission: unauthenticated Discover triggers a
  no-ACK KeyRequest; valid KeyUpdate triggers a reciprocal reliable KeyUpdate;
  only its ACK marks the outbound group key sent; and a NeighborCheck proof
  activates the relationship after both directions complete.
- Exact security-record layouts: seven-byte Pairwise32 KeyRequest and 51-byte
  KeyUpdate, including 12-bit packed key/sequence fields and the two-byte
  security-count envelope.
- Enabled KeyRequest/KeyUpdate crypto: source-faithful A5/5A protection-key
  derivation, four-/eight-byte truncated HMAC-SHA1 tags, pairwise wrapping-key
  derivation, AES-256-CBC group-key wrap/unwrap, AES-256-CTR primitive, and
  injectable synthetic mission/pairwise/group keys.
- Authentication precedes KeyUpdate HOP replay bookkeeping, ACK generation,
  group-key installation, and ARL admission. Tampered or wrong-key traffic
  cannot poison the receive window or earn an ACK.
- Source-faithful GroupEstablish protection for Discover broadcasts: AES-CTR
  payload encryption, a two-byte mission-key tag half, and a two-byte
  group-key tag half. Mission-authenticated records awaiting a group key are
  held newest-per-source and replayed after KeyUpdate completion.
- Source-faithful Group16 protection for routing broadcasts and reliable
  routing control, with authentication before HOP duplicate handling,
  delivery, or ACK.
- Source-faithful Pairwise16 protection for ordinary acknowledged and
  unacknowledged DATA, plus Pairwise32Encrypt for explicit encrypted one-hop
  DATA. HMAC binds the source, destination, packet type, length, and protected
  record; AES-CTR uses the legacy source/destination/count/key/sequence IV.
- Source-exact Pairwise16 policy for the common ACK/DACK packet type 0. The
  ns-3 logical ACK body authenticates before cancellation, custody mutation,
  or the delayed NWK wake. Pairwise16 exact control ACKs model 30 bytes;
  cumulative DATA ACK/DACK feedback models 46 bytes because both
  source-defined 64-bit receive registers are present. Exact production
  protected-body byte mapping remains unverified without a legacy golden
  ACK/DACK security record.
- Live Group32Encrypt AppNeighborcast with a broadcast, non-ACKable envelope,
  authentication and replay checks before decrypt/delivery, and no HOP ACK.
- Shared 12-bit group sequence/key-ID evolution, same-epoch one-way key
  derivation, one previous received key, fresh keys at 16-key boundaries, and
  KeyUpdate redistribution when a local fresh-key epoch begins.
- Target `SLIDINGWINDOW` behavior with 128 combined key-ID/sequence entries,
  out-of-order acceptance, authenticated duplicate classification, full
  16-bit security counts, and modulo restart-count validation.
- Inactive-neighbor DATA rejection with CHECK_MESSAGE initiation, continued
  RoutingUpdate processing from inactive neighbors, and relationship reset on
  remote security-count change.
- Source-backed inactive-transit admission ordering: RoutingUpdates received
  from an inactive reporter are retained but cannot be selected. Admitting the
  reporter recomputes only its direct-neighbor destination; cached transit
  candidates remain deferred until a later source-owned recomputation revisits
  their destinations. Normal and looped post-admission RoutingUpdates are
  covered triggers, including selection of a cached alternate reporter. If a
  reporter becomes inactive before or after admission, its learned transit
  candidates are deleted and cannot resurrect merely because that neighbor is
  admitted again.
- Legacy ARL INFO/UPDATE/FLUSH byte streams, six-byte section prefixes,
  out-of-order atomic reassembly, and independent reliable HOP sections.
- Source-owned self-route capability state: nonzero local capability is encoded
  as the exact zero-hop, zero-cost 11-byte UPDATE; capability zero is omitted
  from snapshots and a nonzero-to-zero change is sent as DELETE. On receipt,
  the advertisement controls routing capability while direct reachability and
  HELLO role metadata remain independent.
- Source-exact grouped same-pass routing propagation: a processing pass freezes
  each changed destination once in newest-created-first order and optionally
  prepends the changed 17-byte INFO record to the same mixed UPDATE/DELETE
  stream. INFO-only passes are sent without FLUSH. Same-pass snapshots run
  first and omit the frozen destinations; INFO and destination dirty state are
  consumed even with no recipients. Power and margin fields preserve the
  wrapper's signed `int16` truncation toward zero. Active neighbors are
  grouped newest-created-first in sets of ten; every group reuses the same
  section bytes and starting sequence. The next logical stream advances by the
  number of groups, and retained-owner LIFO order determines the NWK-to-HOP
  section/group handoff.
- End-to-end 24-bit node IDs with `0xFFFFFF` broadcast, exact big-endian node
  serialization in NWK/HOP/HELLO/SNMP headers, and golden 24/16/32-bit ARL
  record tests. HOP sequence and ARL hop-count fields remain 16-bit.
- Deterministic bidirectional three-node convergence from empty route tables
  without static routes, using the source-exact grouped changed-route stream.
- Source-backed SNMP relay controls: command values 5/6, one-hop non-ACK
  delivery, stable equal-priority ordering, no HOP-neighbor freshness update,
  and the legacy stored-but-inert `relay_holdoff` flag.
- Exact fixed-bit serializers for all eleven supplied `br_*.pk.m` formats,
  including inherited-payload position, zero-bit metadata exclusion, and the
  104-/7,888-bit OTA preamble fields.
- Source-backed complete envelope nesting and modeled sizes for DATA, ACK,
  routed SNMP, HELLO, and ARL route control. MAC concatenation, PHY packet
  length, and payload airtime use those modeled sizes while compatibility
  metadata remains available to the protocol implementation.
- Relay custody, reverse routes, invalidation, alternate paths, and no-route
  ACK behavior.
- 16-bit HOP sequences, cumulative ACK/DACK windows, duplicate suppression,
  and a 512-entry resend limit. Each matching MAC sent indication starts an
  uncancelled, list-wide resend scan at the actual sent instant plus the
  nominal two-second retry or four-second final-ACK wait and one 36-MHz `TIC`.
- Source-order resend scans delete every due final timeout before making a
  second pass for due retries. A MAC sent indication missing its resend entry
  installs the source's fallback global scan when the retained timer handle is
  no longer pending.
- Multi-destination reliable routing-control ACK bookkeeping.
- Final DATA timeout releases flow-control/NSDP custody without creating an
  OPNET-incompatible routing-link failure and requests the HOP-owned generic
  NWK queue wake one `TIC` later.
- Every readable ACK/DACK addressed to the local node requests that same
  generic wake, including unknown, stale, duplicate, and zero-bit/no-op
  feedback. HOP-origin wakes coalesce behind the first pending HOP event, while
  the NWK process retains a distinct local queue-check handle.
- The hard-disabled legacy single-DACK path remains a custody and MAC no-op;
  DACK state transitions come from the cumulative/windowed feedback path.
- Grouped routing-control timeout preserves the original ordered transaction
  metadata without separately penalizing an already-ACKed primary target.
- MAC OPNET slot selection/holdoff, ACK queue semantics, 512-entry DATA limit,
  concatenation, retransmission DSCP, and head blocking.
- Aggregate rate/power and preamble selection across all destinations,
  preamble freshness, wireless overhearing, and live HOP link-control results.
- OPNET Idle/Search/Track/Tx transitions, deterministic -11-dB synchronization
  qualification, 6.63-ms acquisition, mature-preamble selection, and Track
  lock that rejects later acquisition attempts.
- Explicit overlapping-signal lifetime, equal-power collision, strongest-signal
  capture at the source-backed 10.5-dB JSR boundary, post-lock interference,
  and half-duplex receive loss while transmitting.
- Search-without-SYNC-only local and neighbor slot-counter advancement, with
  countdown freeze during preamble acquisition, Track, Tx, and Idle.
- Source-derived OTA duration: long/short preamble plus SOF/speed/length at S0,
  followed by exact envelope payload and FCS at the selected data rate.
- Periodic Idle/Search duty-cycle behavior in which a long preamble can bridge
  sleep to the next wake window while an expired short preamble is missed.
- Exact non-TMM `br_power` minimum-gain selection across free-space,
  flat-earth, and ad-hoc WLAN equations, including fractional passband power,
  configurable antenna gains/heights, and zero-overlap rejection.
- Source-backed receiver noise and `br_snr` in linear watts, with the
  -106.975-dBm 1-MHz reference scaling with receiver bandwidth.
- `br_inoise` rate split: different-rate received powers accumulate as noise,
  while same-rate overlaps preserve strongest JSR and signed time offset for
  the modulation-table path.
- Distance-derived propagation delay and expanded receive diagnostics for
  power, noise, SNR, band overlap, path model, JSR, and accept/drop.
- Exact standard CSR curve and all 4,910 supplied JSR/half-chip collision
  curves, with a reproducible binary-table importer and exact sample storage.
- Source-exact four-bit payload intervals, processing gain, and table selection
  for 8-128 kbit/s, plus recovered 500-kbit/s DPSK and 1000-kbit/s DQPSK
  equations in the opt-in high-rate extension profile.
- Old-state interval closure before interference changes, source inverse-CDF
  binomial sampling, preamble exclusion, separate 48-bit header accounting,
  and payload/FCS error allocation.
- Full-packet ECC with the recovered 0.1 threshold, inclusive truncated error
  limit, prior-rejection preservation, and ECC-specific drop counting.
- Closure before propagation/channel/power/MAC effects, with never-occluded,
  geometric Earth-LOS, and injectable terrain/reference modes.

## Steps 1-3: admission and enabled hop-security crypto

The ARL admission step and enabled KeyRequest/KeyUpdate crypto step are
complete. The mapping is:

| Legacy source behavior | ns-3 implementation | Verification |
| --- | --- | --- |
| `routesRcvUnAuthedMsg(Discover)` resets 5000-ms delays and sends KeyRequest | `EvaluateNeighborAdmission()` and `SendKeyRequest()` | Admission smoke test observes KeyRequest and blocks pre-admission DATA. |
| KeyRequest is unicast Pairwise32 with `NoAck`/`NoResend` and neighbor immediate tag | `CsrHopLayer::SendKeyRequest()` and MAC type cancellation | Seven-byte golden vector plus lifecycle test. |
| KeyUpdate is reliable and carries KeyUpdate security | `CsrHopLayer::SendKeyUpdate()` through the existing ACK/resend queue | 51-byte golden vector and ACK-completion assertions. |
| MNK + pairwise material + A5/5A pad PBKDF2 derives AES/HMAC protection keys | `CsrLegacyCrypto::DeriveProtectionKeys()` | Deterministic legacy record vectors plus standard PBKDF2/HMAC vectors. |
| KeyUpdate HMAC covers the plaintext group key; its IV is tag + five-byte PRN + source | `ComputeKeyUpdateAuthTag()` and `BuildKeyUpdateIv()` | Exact independently generated 51-byte record and tamper tests. |
| Pairwise wrapping key uses MNK + 64-byte pairwise material and source/count/group-ID/PRN salt | `DerivePairwiseWrappingKey()` | Exact AES-256-CBC ciphertext and recovered group-key assertions. |
| Bad authentication is rejected before HOP ACK/replay/admission | KeyUpdate receive ordering in `CsrHopLayer::ReceiveFromMac()` | A bad frame followed by the valid frame with the same HOP sequence produces zero then one ACK. |
| Valid KeyUpdate is ACKed and causes a reciprocal update | HOP receive path and `NoteKeyUpdateReceived()` | Both peers must report sent and received keys. |
| Outbound key is sent only after its ACK | `MarkKeyUpdateAcked()` and `NoteKeyUpdateCompletion()` | State test proves enqueue alone is insufficient and records ACK time. |
| Both key directions plus CHECK_DISCOVERY/CHECK_OVERHEARD/CHECK_MESSAGE admit a neighbor | `TryMakeNeighborActive()` and the admission check state | Route is unusable before admission and selectable afterward. |
| Security-count change replaces the relationship | HOP anti-replay state and `NoteSecurityCountChange()` | State test accepts a restarted sequence only with a changed count and clears the old sent-key state. |
| Discover uses GroupEstablish AES-CTR plus split mission/group HMAC | `SendProtectedDiscovery()` and `ReceiveGroupMessage(GroupEstablish)` | Independent golden record, separate lower-/upper-tag tamper tests, and live encrypted discovery. |
| A Discover authenticated only by its mission tag waits for the missing key | `HandleProtectedHello()` and `ReplayPendingGroupEstablish()` | Newest-per-source queue replacement and KeyUpdate-before-payload callback order. |
| RoutingUpdate uses Group16 over its plaintext record | `SendAuthenticatedRoutingHello()` and protected `SendRoutingControl()` | Golden record plus valid/tampered/replayed reliable-control tests. |
| Group32Encrypt authenticates encrypted payload | `ProtectGroupMessage()` / `ReceiveGroupMessage()` | Golden ciphertext/tag, wrong-key rejection, and decrypt round trip. |
| AppData/AppDataNoAck use Pairwise16 | `SendData()` and `ReceivePairwiseMessage(Pairwise16)` | Independent golden record, tamper/replay checks, auth-before-ACK/delivery, and live over-air round trip. |
| Pairwise32 and Pairwise32Encrypt use four-byte HMAC; the encrypted mode applies AES-CTR before HMAC | `ProtectPairwiseMessage()` / `ReceivePairwiseMessage()` | Independent records for all three pairwise modes plus wrong-key/type and ciphertext-tamper tests. |
| AppNeighborcast uses non-ACKed Group32Encrypt broadcast | `SendNeighborcast()` / `HandleProtectedNeighborcast()` | Live one-hop encrypted delivery, exact-once replay behavior, and malformed-envelope rejection. |
| Group sequence rollover evolves or replaces the key | `NextGroupKeySequence()` / `DeriveNextGroupKeyMaterial()` | ID 1-to-2 derived-key vector, previous-key receive, and ID 15-to-16 fresh-key redistribution state. |

The KeyUpdate wire record deliberately reserves six data-PRN bytes because
that is what `hopSecLayer.c` places on the packet. `arlSecurity.h` defines a
five-byte pseudorandom value, so enabled crypto uses only the first five bytes;
the ns-3 sender makes the otherwise uninitialized sixth byte deterministic by
setting it to zero.

The portable provider uses synthetic injected key material instead of
`KEYS_IN_FLASH` and key-manager hardware. This preserves packet and state
behavior without embedding operational keys or radio-specific storage in the
simulator.

## Step 4: relay-holdoff signaling and control ordering

The supplied wrapper resolves the relay-holdoff ambiguity as a control-state
feature, not a working transit gate. `br_support.h` assigns commands 5 and 6,
and `br_nwk.pr.c` stores or clears `BrT_Neighbor_Entry::relay_holdoff`. No
supplied queue, forwarding, or flow-control path ever reads that field.
Adding a transit block would therefore diverge from the executable OPNET
wrapper.

| Legacy source behavior | ns-3 implementation | Verification |
| --- | --- | --- |
| `SNMP_RELAY_HOLDOFF == 5` and `SNMP_RELAY_CLEAR == 6` | Explicit `CsrSnmpCommand` values and `SendRelayHoldoff()` / `SendRelayClear()` | Serialized command-byte assertions. |
| `br_SNMP` bypasses ACK/resend and DATA flow control | Existing `CsrHopLayer::SendSnmp()` path with sequence zero, DSCP zero, and no ACK | No HOP pending-DATA, resend, or MAC ACK-queue entry. |
| Equal-DSCP packets retain insertion order and may be concatenated | Existing stable MAC insertion and aggregate segment order | HOLD followed by CLEAR in one aggregate leaves the flag clear. |
| HOP forwards SNMP before `update_neighbor()` and returns | SNMP dispatch remains before `UpdateNeighborHeard()` | Control reception leaves the HOP last-heard timestamp unchanged. |
| NWK looks up the pre-existing source neighbor and writes the flag synchronously | Known-neighbor state in `ReceiveSnmpFromHop()` | Unknown controls do not create a neighbor; ordered controls update counters and final state. |
| `check_nwk_queue()` never reads `relay_holdoff` | The flag is exposed for observation but deliberately omitted from queue admission | Three-node transit DATA is delivered while HOLD is stored. |

The exact command IDs and behavior are now source-backed. The compact SNMP
compatibility envelope remains as state storage, but its wire-size annotation
now maps it to the exact `br_Mac -> br_Hop -> br_SNMP` 31-byte envelope.

## Step 5: exact packet and control envelopes

All eleven supplied packet models now have direct, big-endian serializers and
deserializers. Their declared fixed sizes, inherited-payload positions, and
zero-bit metadata rules are covered by independent golden tests. Runtime
packets retain semantic compatibility headers but carry a copied packet tag
with the exact OPNET envelope tree and size; MAC packing, PER packet bits, and
payload transmission duration consume the modeled size rather than
`Packet::GetSize()`.

The complete mapping, including the source conflict between the eight-bit
`br_Hop.pk.m` sequence and the 16-bit HOP implementations, is documented in
`docs/opnet-packet-envelope-parity.md`.

## Step 6: MAC receive/contention state machine

The supplied `br_mac.pr.c`, `br_txdel.ps.c`, and radio-pipeline sources now
drive an explicit per-receiver signal model instead of independent successful
delivery events.

| Legacy source behavior | ns-3 implementation | Verification |
| --- | --- | --- |
| `Idle_st`, `Search_st`, `Track_st`, and `Tx_st` are distinct operational states | `CsrMacCore::State` plus device-owned receive transitions and MAC-owned Tx transitions | State test observes Search before acquisition, Track after 6.63 ms, and Search after completion. |
| `SYNC2TRACK_MIN` is 0.00663 s and synchronization requires the source threshold | Scheduled acquisition over sync-eligible preambles with the deterministic -11-dB mean threshold | Acquisition cannot occur at 5 ms and has occurred by 10 ms. |
| Search starts with the earliest signal; a later mature and stronger preamble may replace it | Per-receiver `RxSignal` registry ordered by start time and arrival order | Simultaneous strong/weak scenario delivers only the strongest mature candidate. |
| Track rejects new synchronization attempts while the tracked packet remains exposed to interference | Fixed tracked-signal identity and post-lock jammer processing | A later, stronger signal cannot capture and corrupts the tracked packet. |
| Comparable overlaps collide; sufficiently weak interference falls in the source's -12-dB JSR bucket | Selected overlaps proceed through exact JSR/offset BER and full-packet ECC; only explicit test hooks retain the old hard gate | Equal-power overlap is ECC-dropped while a weak selected jammer remains decodable. |
| `tslot_tasks()` changes counters only in Search with no `SYNC_PRES` | Tick-by-tick local Tx countdown and neighbor reservation decay gates | A tracked reception freezes and later resumes a pending transmission. |
| The 13-ms clock and 300-ms holdoff are independent; every Tx loads and advertises its next persistent reservation | One shared local/neighbor slot clock, explicit PREP/holdoff state, Idle's globally aligned RTS event, and retained empty-queue counter | During-Tx, post-Tx, Track-return, Idle-RTS, expiry, zero-counter, and HOP-retry cases consume or redraw the source-ordered slot. |
| Tx is half duplex and returns to Search after OTA completion | `NotifyPhyTxStart()` / `FinishTx()` plus state-gated receive delivery | A receiver cannot decode an overlapping frame while transmitting. |
| Long preamble supports duty-cycled discovery; short preamble must arrive during an awake window | Periodic wake phase, Idle/Search transitions, and preamble-survival check | Long preamble bridges a 500-ms sleep interval; short preamble records one miss. |
| Preamble/header fields transmit at S0 and payload/FCS at the selected rate | Source-derived OTA duration using exact OPNET envelope bytes | Existing sent-time, queue, integration, and slot tests use the new airtime checkpoints. |

The recovered discovery schedule is independent of radio completion.
`routesDiscoveryInitLocal()` uses a 5,000-ms interval with repeat count three:
the node broadcasts at relative `+0`, `+5`, and `+10` seconds, then the fourth
broadcaster callback ends discovery at `+15` seconds. That callback immediately
invokes the wrapper's discovery-complete path and cancels the nominal
30-second stop, which remains only a fallback. OPNET does not wait for pending
admission, local MAC/queue quiet, or an extra response interval, and the NWK
wrapper does not synthesize per-neighbor Verify traffic before completion.
Focused timing checks pin every boundary at nanosecond resolution; a separate
ordering case proves that the newest-created logical destination is the first
serialized SNMP discovery handoff and that a capable local node includes a
usable reverse-only destination in the walk. That reverse-only case also
observes the actual frame: the outer HOP destination is the selected reverse
neighbor while the inner SNMP destination remains the logical endpoint.

## Step 7: deterministic PHY front end

The supplied `br_rxgroup`, `br_power`, `br_inoise`, and `br_snr` stages now
drive live reception. The default profile maps the node-model 30-MHz base
frequency, 1-MHz bandwidth, one-meter height, and source -106.975-dBm noise
reference. Promoted validation-scenario values such as 400 MHz and per-node
altitude remain configurable.

| Legacy source behavior | ns-3 implementation | Verification |
| --- | --- | --- |
| Receiver group excludes the transmitting node and channel match requires overlap | Peer self-exclusion plus explicit Tx/Rx passband intersection | Runtime mismatch never enters Track or payload delivery. |
| `br_power` selects the smallest of three linear path gains | `CsrPhyModel::ComputeFrontEnd()` exact free-space, flat-earth, and ad-hoc WLAN equations | Independent 30.5-MHz and 2.4-GHz model-selection vectors. |
| In-band power is proportional to overlap/Tx bandwidth before antenna/path gain | Linear-watt band, antenna, and propagation pipeline | Full/half/zero-overlap golden vectors and +6/+3-dB gain vector. |
| Different payload speeds add received watts to `NOISE_ACCUM` | Per-signal active and peak interference power | Live 128/64-rate overlap produces exact -99.205734-dBm noise and 19.205734-dB SNR. |
| Same payload speeds use JSR and start-time offset instead of additive noise | Strongest same-rate metadata retained per tracked signal | Live -6-dB jammer at +20 ms preserves background noise and records both values exactly. |
| `br_snr` divides received watts by background plus accumulated noise | Source-backed linear ratio at every interference boundary | Golden baseline and interfered SNR vectors. |

The complete equations, recovered project attributes, runtime diagnostics, and
explicit BER boundary are documented in `docs/opnet-phy-front-end.md`.

## Step 8: modulation BER, interval errors, ECC, and closure

The supplied binary modulation tables and final receive stages now drive live
accept/drop behavior. The generated runtime data contains the standard CSR and
DQPSK curves plus all 4,910 spread-rate collision curves without requiring
OPNET at simulation time. Recovered modem scripts establish 500 kbit/s as
DPSK and 1000 kbit/s as DQPSK, both at 500 ksymbols/s. The DQPSK samples match
the recovered analytical function to table precision; the supplied C pipeline
still does not select either high-rate mode directly.

| Source or extension behavior | ns-3 implementation | Verification |
| --- | --- | --- |
| `op_tbl_mod_ber()` selects `csr` or a spread-rate JSR/half-chip curve | Exact embedded samples, source JSR thresholds, circular signed-offset quantization, endpoint clamp, and linear interpolation | Independent standard and all-five-spread-rate collision goldens plus boundary tests. |
| Recovered high-rate modem uses DPSK at 500 kbit/s and DQPSK at 1000 kbit/s | Opt-in `EXTENDED_DQPSK` compatibility profile, analytical `DPSK_PB`, exact DQPSK samples, NWK-to-MAC propagation, compact wire codes, and live OTA/ECC path | DPSK/DQPSK goldens plus live 500/1000 delivery, duration, low-SNR drop, and same-rate overlap. |
| Header rate is S0; 8-128-kbit/s payload uses source `4 / payload_interval` timing | Exact non-rounded source rates and processing-gain helpers shared by airtime and BER; `modem_ex.m` independently establishes the same Eb/N0 conversion for the two high rates | Five source rates plus two extension rates and gains match independent numeric vectors. |
| `NUM_COLLS > 0` always selects a collision preamble; only same-speed payload selects a collision curve | Per-interval cumulative collision and shared receiver same-speed/JSR/offset state | Standard, same-rate, and different-rate selection vectors. |
| `br_error_all_stats` closes the old BER interval before a change | Live per-signal intervals are sampled before interference state is modified | Split, truncation, boundary, cap, and short-jammer tests. |
| One inverse-CDF binomial draw per nontrivial header/payload region | Log-gamma source sampler, including `p > 0.5` inversion | Independent uniform-to-error golden outcomes. |
| Preamble excluded; 48-bit header separated; payload includes FCS | Explicit timing boundaries and source integer truncation | Preamble/FCS and combined header/payload assertions. |
| `br_ecc` accepts at `errors <= int(0.1 * protected_bits)` | Full-packet inclusive ECC, prior reject/lock/failure preservation, and ECC-drop counter | At-limit/over-limit and side-effect matrix. |
| Closure runs before channel, power, interference, synchronization, or MAC | Sender-side visibility gate with never, Earth-LOS, and delegated modes | Failed closure produces no receive decision or delivery effect. |
| A selected collision is not intrinsically invalid | Exact BER/error/ECC outcome replaces the production hard-collision gate | Live weak collision succeeds and equal-power collision is rejected by ECC. |

The complete table inventory, importer, equations, ordering, and explicit
external-closure boundary are documented in `docs/opnet-phy-ber-ecc.md`.

## Step 9: scenario import and differential traces

The binary Modeler scenario boundary is now explicit and executable. The
versioned importer recovers the complete contiguous promoted-attribute schema
and file-local IDs, so it handles normal, shifted one-node, and older/mobile
campus records without hard-coded field numbers. All 12 recovered campus
`*.nt.m` models and their paired DES environments import successfully.
`--infer-gateway-flows` now requires an explicit executable-backed application
profile. The selected 2014/15 two-node and hidden-symmetrical executables use
send-to-and-from-gateway behavior; the selected multihop executable uses
send-only-to-gateway. All three historical packet generators lack DSCP code,
so their profiles force FIFO DSCP 0. The newer supplied source remains a
separate send-only profile with an explicit fixed-DSCP approximation.

The canonical run row also carries an executable-backed `mac_profile`.
Selection is hash-bound and explicit; it is never inferred from a topology,
scenario name, or node count because a network model can be paired with a
different compiled executable. All five supported profiles are recorded here:

| MAC profile | Source or executable binding | Exact selection family |
| --- | --- | --- |
| `current-fine-free-slot` | Supplied newer `br_mac.pr.c`; no recovered executable SHA binding | Fine range, one-based free-slot ordinal, with neighbor `rtslot_counter` reservations. |
| `hist-2014-coarse-inclusive-no-avoid` | `08c3647956a9dd31c03c9a2c7bac2cf00b62216968e08ee539b91bc327bafbf5` (`CSR_Validation-1_Node` and `2_Nodes_Latency`) | Coarse range from local `active_nodes`, uniform support `0..R` inclusive, no reservation avoidance or probing. |
| `hist-2014-zero-based-rebuild-list` | `d9ebf7626e641ee68ccc9b58b1bb4b28fc0906111762e4456a0e8c7a0bd8b055` | Coarse range, uniform ordered-list support `0..R-1`, no reservation avoidance. |
| `hist-2015-fine-one-based-table-no-avoid` | `dd3f38e8d33700b61f9e360a737ba34e56cb75b2570eb2960a02de381ed0fff0` | Fine range, ordered-table support `1..R` for `R<=254`, no reservation avoidance. |
| `hist-2014-next-tslot-modulo-probe` | `adb97c54f7566439f1404e972d3d777a3bca613e2a965bf12f03353fb009d9af` | Coarse range, initial support `0..R`, then direct neighbor-`next_tslot` modulo probing. |

The coarse mapping is `N<=4 -> R=31`, `5..8 -> 63`, `9..12 -> 127`, and
`N>=13 -> 255`. The exact fine mapping for `N=0..16` is respectively
`15,18,21,25,31,37,44,52,63,75,89,106,127,151,180,214,255`, with other
values selecting 255. The zero-based executable rebuilds entries `0..R-1`
and draws `floor(uniform(R))`; its separate `max_tslot=R+1` assignment does
not extend that support. The new operational executable
(`CSR_Validation-2_Nodes_Latency`, Build ID
`cbc38b846deb800c1df8d99c0f5628a68c3b24ee`) instead draws
`floor(uniform(R+1))`; its five production call sites pass local
`active_nodes` directly and the function reads no reservation state. This is
source-exact for that SHA-bound executable and is intentionally not a default.
Other archived operational builds conflict, so this profile must not be
selected from the era label alone. The exact 2014 source text is unavailable;
this classification rests on the authoritative executable disassembly and its
DWARF source mapping. The fine-table executable builds indexes
`0..254`
and dereferences `floor(uniform(R))+1`. All five of its call sites pass
`rn_range`; the archived binary initializes it to zero, never assigns it
again, and the selected network/environment files do not configure it, so the
default mapping applies. Its dormant positive-after-first-transmission
override would replace `R`. A selected index 255 is the historical
out-of-bounds defect and now fails explicitly instead of being clamped or
wrapped.

The modulo-probe executable draws `floor(uniform(R+1))`, compares the candidate
directly with every neighbor's `next_tslot`, and on collision advances with
`(candidate+1)%R`. Thus `R` is returnable only as a free initial draw; after a
collision the search covers `0..R-1`, with `R -> 1` and `R-1 -> 0`. If all of
that modulo domain is occupied, the historical code loops forever even if
`R` is free. The corresponding profile detects the completed cycle and fails
explicitly. Full disassembly semantics, endpoints, and the import command are
in `docs/opnet-scenario-differential-harness.md`.

The ns-3 runner creates the corresponding node roles, coordinates, link
distances, rate/power limits, link margin, ECC threshold, frequency, duration,
and seed. Its parity mode starts every receiver in Idle, performs no random
wake-phase draw, and issues the first common WAKE at exactly 0.988 seconds.
It emits a canonical ordered trace across scenario construction,
application, MAC state, transmission, PHY accept/drop, NWK delivery, and route
changes. The comparator normalizes common OPNET CSV column/event aliases,
aligns event identities without discarding ordering, applies explicit numeric
tolerances, and reports missing, extra, replaced, or field-mismatched events.
Every run records normalized inputs, a JSON report, logs, hashes, and exact
commands in a manifest.

The application runner schedules the next constant generator event before
all gates, caches the discovered gateway, supports the historical gateway's
live destination draw, and writes one compact admission row per flow by
default. Those rows partition attempts into discovery, empty topology,
unknown gateway route, missing dynamic destination, NSDP-full, and admitted
counts. An explicit `--admissionTrace=1` diagnostic mode additionally records
every source-ordered attempt and its live application/NWK/HOP state; ordinary
aggregate runs retain the compact surface.

Historical result databases are now available and executable evidence. The
read-only extractor decoded 10 complete Modeler `*.ov` files containing 109
vectors and 10,900 buckets, with exact paired-`*.pb.m` identity checks. Two
partial fragments with zero-length vectors are rejected in strict mode rather
than treated as results. The aggregate workflow runs an imported scenario,
derives source-equivalent ns-3 buckets, compares exact statistic/unit/
aggregation identities, and records all commands, hashes, and statuses.

Full core-statistic two-node, symmetrical hidden-node, and multihop
comparisons align all 800 selected points in each case without missing or
extra timestamps. The recovered application packet size matches exactly with
the source-backed
`configured_bytes - 8` modeled network packet. With the hash-bound application
and MAC profiles, the two-node OPNET/ns-3 means are 16.4417/16.3597 sent,
16.4373/16.3474 received, and 1.45125/1.42737 seconds delay. Hidden-node means
are 12.1707/12.7283 sent, 11.4370/12.0342 received, and 9.17976/8.40223 seconds
delay. The retained pre-fix multihop comparison remains the outlier at
2.0120/2.45667 sent,
1.90167/2.29000 received, and 112.748/91.4031 seconds delay. Exact-tolerance
reports still contain 696, 700, and 665 numeric mismatches respectively, plus
20 correctly skipped multihop no-sample values; close means are calibration
evidence, not claimed bucket parity.

The retained pre-fix source-exact multihop queue diagnostic also aligns all
400 positions for
HOP resend size, MAC ACK size, MAC Tx size, and MAC Tx queuing delay. It
compares 394 numeric pairs and preserves six no-sample cases. For bucket ends
strictly after 300 seconds, ns-3 is 16.10% high in HOP resend size, 12.57% low
in ACK size, 13.66% high in Tx size, and 8.85% high in Tx queuing delay. MAC
queuing delay is therefore higher while end-to-end delay is lower, so the
route/path and NWK-residence ledger was added before changing service timing.

That pre-fix path ledger validates 13,740 complete deliveries and 1,000
structurally valid incomplete prefixes at the exclusive 6,000-second stop,
with no invalid chains, route-context mismatches, loops, or end-to-end
decomposition errors.
Every source uses one stable delivered path. The packet-weighted 91.5210-second
mean decomposes exactly into 78.4605 seconds of NWK residence and 13.0605
seconds of post-NWK leg service/transit. Packets taking more than one hop are
only 9.68% of deliveries but contribute 85.79% of cumulative delay. These are
ns-3 isolation measurements, not OPNET numeric parity: the recovered `*.ov`
results do not contain NWK queue vectors or packet-path event order.

The opt-in application/NWK/HOP ledger now closes that source-order boundary,
the recovered DACK-expiration and generic-wake boundaries, and the
route-convergence/admission boundary. The current canonical 6,000-second
campus trace has SHA-256
`64bf81c8b2155cd10e3688b468219c6d7d018dee73097c3cb0a45456339231f2`.
It records 14,551 application admissions, 18,713 NWK/HOP admissions,
1,073,828 per-neighbor holds, and 4,057 global-capacity holds. Strict
packet-path analysis closes 13,817 deliveries and classifies 734 unfinished
prefixes as structurally valid, with zero invalid packets.

In that seeded campus diagnostic, node 7's direct-neighbor route and the fresh
transit updates become selectable at `88.848492442` seconds. Its selected
gateway route is `8>2>4>5>1` with cost 37,250, and convergence completes before
application traffic starts at 300 seconds. The exact timestamp is a diagnostic
for this ns-3 run, not a claimed cross-simulator timing contract.

Each DACK entry now schedules its list-wide expiry scan at the nominal 20- or
40-second deadline plus one OPNET `TIC`. At ns-3's nanosecond resolution, the
source value `1 / 36 MHz` is 28 ns. A scan releases every entry already past
its nominal deadline, so entries less than one `TIC` apart can coalesce: the
effective per-entry offset is in `[0,TIC]`, and every release must coincide
with an actual scheduled DACK timer. At the preceding generic-wake checkpoint,
all 2,156 releases were isolated and occurred at `hold + TIC`. Focused
event-order tests cover the normal and maximum-resend 40-second production
paths, prove capacity remains held immediately after the nominal deadline,
exercise a two-entry coalesced scan, and prove isolated NWK readmission at
`hold + 2*TIC`. A previously
pending HOP-origin wake coalesces that final request exactly as OPNET's
`op_ev_pending()` does, without postponing the first event. The NWK process's
local queue-check event has a separate handle, so it cannot consume or
coalesce the HOP request.

Resend expiry now uses the same source boundary. Every matching MAC sent
indication records the actual transmission instant and leaves its independent
list-wide timer scheduled for `sent + 2 seconds + TIC`, or
`sent + 4 seconds + TIC` after the final retry. `check_resend` first completes
all due final deletions and their queue-wake requests, then performs a second
full pass for due retransmissions. A sent indication whose resend entry is
already absent still schedules the recovered fallback scan when the retained
resend-timer handle is not pending; scheduling a newer timer does not cancel
older scans.

The generic feedback boundary is also explicit. After processing any readable
ACK or DACK addressed to the local node, HOP requests its own NWK queue wake at
`feedback + TIC`, even when the feedback is unknown, stale, duplicate,
zero-bit, or otherwise changes no custody state. Wrong-destination feedback
does not wake the queue. Final resend timeout and DACK expiry use the same
HOP-owned scheduler; simultaneous HOP requests coalesce without postponement,
but never coalesce against the NWK-local queue-check handle. Focused tests
cover the nominal resend/final-expiry boundaries, delete-before-retry order,
missing-entry fallback, unknown/stale/no-op feedback, wrong-destination
rejection, the disabled known single-DACK path, isolated wake latency, and both
feedback and expiry coalescing.

The preceding generic-wake checkpoint's aggregate trace has SHA-256
`8498d2bec10292df73ce60d1bd7823408dcfab614475559b27f8bd03bf264ed9`,
and its generated ns-3 aggregate CSV has SHA-256
`ec20882a5419834d6920303c38f0528096aee445539d4d5d80def6f2ccf64012`.
The eight-series aggregate comparison still aligns all 800 points,
contains 665 exact-tolerance numeric mismatches, and preserves 20 OPNET
no-sample values. Corrected ns-3 means are 2.42767 packets/s sent, 2.30900
packets/s received, and 79.9450 seconds end-to-end delay. Neither corrected
ordering erases the larger aggregate calibration gap.

Relative to the DACK-ordering checkpoint, application decisions, NWK/HOP
admissions, HOP completions, deliveries, and all eight core aggregate series
are unchanged. The generic wakes add source-required NWK rescans, so repeated
hold observations rise from 686,581 to 1,204,924 per-neighbor rows and from
3,251 to 4,152 global-capacity rows. Those extra observations do not change a
packet trajectory or close the remaining aggregate gap.

The retained pre-fix trace has SHA-256
`cf9392db5d10d47f5a7cc6459b00f828556cb72d5988e80e33a55cd0764e79ef`
and contains the 138 pre-15/post-16 DACK decisions that motivated this change.
Recovered `br_hop.pr.c` obtains the NSDP entry before sending to the separate
NWK process and tests that pre-event count. ns-3 now decides before its
synchronous NWK callback: 15 ACKs and 16 DACKs. ACK-marked duplicates ACK
without a second enqueue; DACK-marked retries are re-enqueued and reassessed,
and first no-route receptions remain ACK-suppressed.

The campus blue-radio archive supplies no newer source revision: all 15 C
files are byte-identical to the previously recovered copies. Its serialized
`br_hop.pr.m` independently repeats the same pre-enqueue NSDP ordering. It
also confirms that the archived 500/1000-kbit/s OPNET path is only partially
wired: the node UI and HOP link control expose both rates, while BER,
transmit-delay, error, and concatenation code implement only 8--128 kbit/s and
fall through or reject high-rate packets. It therefore does not supersede the
opt-in recovered-modem extension.

The hidden-node ECC-drop probe also exposes a deliberately unresolved identity
mapping: Modeler records 10.84105
dropped packets/s on average, while ns-3's narrow `reason=ecc` event
classification records 0.6195167 packets/s. It is excluded from the default
equivalence set until prior-stage rejection accounting is reconciled. The
`*.ov` files contain bucket aggregates rather than packet-level ordering, so
an instrumented OPNET CSV event export remains necessary for reservation/
collision event certification. Exact formats, commands, measured results, and
this evidence boundary are documented in
`docs/opnet-scenario-differential-harness.md` and
`docs/opnet-aggregate-comparison.md`.

The first reference case is now fully specified and locally executable. It
uses the recovered three-node validation geometry, two 600-byte sends at 60 s,
controlled slot/counter 5, the source holdoff gate and 13-ms countdown, and
an equal-time collision at gateway 1. SHA-checked instrumentation records the
OPNET app send, reservation lifecycle, Tx start, collision count, PHY values,
and ECC outcome. The ns-3 side passes the lifecycle/collision validator with
both transmissions at 60.165 s and two gateway collision drops. A same-input
harness plumbing check passes 22 selected events; it is not a substitute for
the licensed Modeler export. The complete one-run handoff is documented in
`docs/opnet-reservation-collision-reference.md`.

## Highest-priority remaining work

1. When licensed Modeler access is available, execute the prepared
   reservation/collision case, retain its CSV and provenance manifest,
   validate it, and run the event-order differential comparison.
2. Reproduce synchronization-threshold variance for calibrated runs.
3. Close exact security-wrapper gaps for remaining HOP control packets and the
   distinct network-security modes once authoritative mapping or trace
   evidence is available.

## Deferred PHY/calibration work

- Stochastic synchronization threshold variance around the implemented
  -11-dB mean.
- Authoritative off-grid `op_tbl_mod_ber()` behavior if an OPNET probe becomes
  available; current lookups use endpoint clamp and arithmetic interpolation.
- Exact external Earth-LOS/TMM injection; imported TMM scenarios are rejected
  until that implementation is supplied.
- Authoritative same-rate high-rate collision curves, if they exist outside
  the supplied archive; the extended 500/1000-kbit/s profile treats the
  recorded jammer as payload noise and retains the rate-specific DPSK/DQPSK
  BER law.
- DSP firmware validation and calibration against the physical modem are
  post-parity work; they are not required to reproduce the supplied OPNET
  model and recovered MATLAB theory path.

## Missing high-value scenarios

1. Route loss and recovery while mixed-DSCP traffic is held in NWK.
2. Sequence wraparound and cumulative windows older than 64 packets.
3. Sustained lossy overload across all queue limits with counter invariants.
4. Three-or-more-signal mixed-rate intervals and multiple competing same-rate
   JSR records under the recovered receiver-global last-collision state.
5. An authoritative OPNET CSV event reference using the now-identical imported
   topology, explicit traffic, and seed. Historical aggregate results are
   available but cannot substitute for chronological events.
6. Lost KeyRequest, lost KeyUpdate ACK, and pairwise sequence/key-ID rollover
   during admission. Focused tests now cover authenticated security-count
   restart and wrap.

## Current regression baseline

All 38 CSR executable targets build on this audit revision. The 36 focused
parity smoke tests, `csr-mac-demo-split`, and the imported-scenario runner
workflow pass (38/38). All 143 focused Python tests pass; the suite additionally
covers the admission-state and packet-path analyzers as well as the
scenario importer, event comparator/instrumenter, conservative `*.ov`
extractor, ns-3 bucket aggregator, aggregate comparator, and end-to-end
aggregate workflow. The importer decodes all 12 recovered campus network
models and paired DES environments, while strict vector extraction accepts all
10 complete historical results and rejects both partial fragments. The event
end-to-end fixture produces six ordered events on each side with zero missing,
extra, replaced, field-mismatch, or coverage-gap results. The packet-envelope
test covers all eleven fixed formats, golden bytes, inherited payload order,
zero-bit metadata, every live control-envelope inference branch, and aggregate
size tags. The relay-holdoff test covers command bytes, one-hop no-ACK
handling, FIFO
HOLD/CLEAR ordering, known-neighbor state, no freshness update, and transit
delivery while held. The pairwise crypto test covers standard primitive
vectors, exact 7-/51-byte enabled-security vectors, tamper and wrong-key
rejection, authentication-before-ACK, group-key recovery,
128-entry replay behavior, and 16-bit security-count wrap. The group-security
test adds exact GroupEstablish/Group16/Group32Encrypt records, deferred replay,
both authentication halves, key evolution, previous-key handling, and fresh
epoch redistribution. The remaining-security test adds independent
Pairwise16/Pairwise32/Pairwise32Encrypt records, AES-CTR ciphertext, wrong-type
and wrong-key rejection, pairwise auth-before-ACK/delivery, source-exact
Pairwise16 ACK/DACK policy and live DACK capture, live encrypted
AppNeighborcast, malformed-envelope rejection, and all three live send paths.
The receive-contention test covers Search/Track timing, simultaneous collision,
strongest-preamble capture, post-lock interference, RX-induced slot freezing,
long- versus short-preamble duty-cycle behavior, and half-duplex loss.
The reservation-lifecycle test adds Idle queueing until its globally aligned
RTS TSLOT, the shared local/neighbor phase, `prep_tx` holdoff restart,
first-contact reservation ordering, persistent advertisement/reuse,
expiry/redraw ordering,
SYNC/Track activation, real HOP retry reuse, and delayed-packing state gates.
The MAC-sent-time test now pins retry and final-expiry scans one `TIC` beyond
their nominal actual-send boundaries, verifies the final queue wake one
additional `TIC` later, and covers the missing-entry fallback plus
delete-all-before-retry ordering. The ACK-window and DACK-hold tests cover
HOP-owned generic wakes for valid and no-op feedback, wrong-destination
suppression, HOP-only coalescing, and DACK capacity-release ordering.
The queue-observation test adds exact source write-site and ordering checks for
HOP enqueue/ACK removal, MAC data admission/selection delay, ACK duplicate and
cumulative-update suppression, unchanged full-queue rejection, and
one-plus-four resend-limit removal. It now also exercises the real compact
aggregate writer, direct DATA held until route availability, a forced two-hop
path, and exact NWK queue-size and residence-delay samples at each forwarding
node. It also proves the relay feedback boundary: pre-15/post-16 selects ACK,
pre-16/post-17 selects DACK, an ACK-marked duplicate ACKs without another
enqueue, a DACK-marked retry is re-enqueued and reassessed, and a first
no-route reception remains ACK-suppressed. Legacy NWK performs no duplicate
suppression after that retry and can deliver both relay copies under fresh
onward HOP sequences. The strict packet-path analyzer currently treats that
source-exact lost-DACK quirk as a duplicate-delivery failure; a three-hop
lineage fixture and narrowly scoped analyzer classification remain open. For
all other paths, the analyzer validates delivered and in-flight event
grammars, route context, next-hop continuity, loops, and the end-to-end
residence/transit decomposition.
The PHY-front-end test adds independent propagation, overlap, gain, noise, and
SNR vectors plus live channel-mismatch, different-rate additive-noise, and
same-rate JSR/time-offset paths. The BER/ECC test adds exact modulation-table
vectors, interpolation/quantization boundaries, source rates and processing
gain, interval truncation, binomial outcomes, ECC ordering, closure modes, and
live selected-collision acceptance and rejection.
The live high-rate test adds default-profile preservation, NWK-to-MAC extended
profile propagation, 500-kbit/s scenario caps, exact 500/1000 OTA durations,
production DPSK/DQPSK BER and ECC outcomes, same-rate jammer-as-noise ordering,
signed stronger/shorter-jammer interval handling, and safe one-frame queue
progress when no high-rate concat limit is available.
The admission test still covers pre-admission DATA rejection, reciprocal key
exchange, ACK-driven outbound-key completion, NeighborCheck activation, and
exactly-once post-admission DATA delivery. It now also proves that an inactive
reporter's cached transit route remains deferred when the reporter is admitted
and becomes selectable when a fresh ARL byte-stream RoutingUpdate triggers
destination recomputation. A second cached candidate proves that an accepted
looped byte-stream UPDATE performs the same recomputation without selecting its
invalid tombstone. The same test forces that alternate reporter inactive,
re-admits it, and requires a fresh UPDATE before its transit route returns. The
test also fails the primary reporter before its first admission and proves its
cached transit route was deleted. The discovery-ordering
test pins the gateway's `10/15/20/25`-second broadcast/completion boundaries and
newest-destination-first SNMP handoff, including exact reverse-only HOP and
final-destination addressing. The queue-order tests prove that route
receipt, neighbor activation, and discovery completion do not create an
unsourced NWK wake. The autonomous convergence test still covers a lossless
0-1-2 line with no static routes, complete ARL snapshot ACK/reassembly state,
bidirectional two-hop learning, and exactly-once DATA delivery. Warnings are
limited to the pre-existing unused callback/CSV helper warnings.
