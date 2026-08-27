# OPNET parity audit

Updated: 2026-08-26

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
| Visible NWK wrapper behavior | 94-97% | Queue order/timing, blocked-entry scanning, DATA reliability, NSDP, reverse routes, capability handling, local-only link cost, gateway-driven SNMP discovery coordination, relay-holdoff/clear state, active-node accounting, automatic changed-route propagation, and fixed packet/control envelopes are covered. |
| Full NWK routing behavior | 86-91% | ARL byte-stream exchange, 24-bit node identifiers, exact supplied `br_Routes`/`br_SNMP` fixed fields, and no-static-route multi-hop convergence now have source-backed coverage. Remaining uncertainty is concentrated in wrapper timing and untested edge cases rather than an unavailable routing implementation. |
| ARL neighbor admission | 88-94% | Inactive/active state is now separate from freshness; two-sided key completion, deferred NeighborCheck proof, DATA gating, RoutingUpdate handling, security-count reset, and 5-second retry foundations are source-backed. Loss/restart edge cases still need broader trace coverage. |
| HOP | 93-96% | ACK/DACK windows, resend timing, flow control, queue limits, grouped routing ACKs, retry DSCP, custody, overhearing, link-control export, reliable KeyUpdate completion, authenticated Discover/routing/DATA/neighborcast paths, one-hop SNMP dispatch ordering, exact fixed HOP/ACK envelope models, and OPNET DATA-versus-control timeout effects are covered. |
| Full hop security | 89-94% | Pairwise16, Pairwise32, Pairwise32Encrypt, KeyRequest, KeyUpdate, GroupEstablish, Group16, and Group32Encrypt now have source-faithful KDF, HMAC, AES, key-rotation, replay, and golden-record coverage. Ordinary DATA uses Pairwise16, an explicit encrypted DATA path uses Pairwise32Encrypt, and AppNeighborcast uses live Group32Encrypt with authentication before ACK/delivery. Remaining uncertainty is exact security wrapping for other control packets, distinct network-security modes, differential traces, and operational key-store hardware. |
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
- One-`TIC` NWK queue release, blocked-entry scanning, and no implicit
  discovery from the DATA queue.
- OPNET-global ACK/resend behavior for NWK DATA.
- HELLO capability preservation and monotonic active-node population.
- OPNET ARL local-only link-cost calculation: RoutingInfo is retained as
  transaction data but does not create non-legacy bidirectional negotiation.
- NWK/HOP admission, NSDP ACK/DACK selection, and global pending-DATA state.
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
- Legacy ARL INFO/UPDATE/FLUSH byte streams, six-byte section prefixes,
  out-of-order atomic reassembly, and independent reliable HOP sections.
- End-to-end 24-bit node IDs with `0xFFFFFF` broadcast, exact big-endian node
  serialization in NWK/HOP/HELLO/SNMP headers, and golden 24/16/32-bit ARL
  record tests. HOP sequence and ARL hop-count fields remain 16-bit.
- Automatic ARL changed-route propagation and deterministic bidirectional
  three-node convergence from empty route tables without static routes.
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
  retransmission timing from actual MAC send time, and 512-entry resend limit.
- Multi-destination reliable routing-control ACK bookkeeping.
- Final DATA timeout releases flow-control/NSDP custody without creating an
  OPNET-incompatible routing-link failure.
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
  for 8-128 kbit/s, plus exact recovered DQPSK samples in an explicitly
  owner-confirmed 500/1000-kbit/s extension profile.
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
| Tx is half duplex and returns to Search after OTA completion | `NotifyPhyTxStart()` / `FinishTx()` plus state-gated receive delivery | A receiver cannot decode an overlapping frame while transmitting. |
| Long preamble supports duty-cycled discovery; short preamble must arrive during an awake window | Periodic wake phase, Idle/Search transitions, and preamble-survival check | Long preamble bridges a 500-ms sleep interval; short preamble records one miss. |
| Preamble/header fields transmit at S0 and payload/FCS at the selected rate | Source-derived OTA duration using exact OPNET envelope bytes | Existing sent-time, queue, integration, and slot tests use the new airtime checkpoints. |

Exact airtime also exposed a control-order race: the fourth discovery timer
could stop discovery while the third long-preamble broadcast was still on air.
Discovery completion now waits for local MAC control to clear and observes one
quiet response interval, while the independently scheduled stop event remains
the hard bound. The autonomous no-static-route convergence scenario covers
this ordering.

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
OPNET at simulation time. The DQPSK samples are exact recovered data; their
mapping to the actual radio's owner-confirmed 500/1000-kbit/s modes is an
extension because the supplied C pipeline does not reference `dqpsk`.

| Source or extension behavior | ns-3 implementation | Verification |
| --- | --- | --- |
| `op_tbl_mod_ber()` selects `csr` or a spread-rate JSR/half-chip curve | Exact embedded samples, source JSR thresholds, circular signed-offset quantization, endpoint clamp, and linear interpolation | Independent standard and all-five-spread-rate collision goldens plus boundary tests. |
| Owner-confirmed 500/1000-kbit/s hardware modes use the recovered `dqpsk` table | Opt-in `EXTENDED_DQPSK` profile, exact DQPSK samples, NWK-to-MAC propagation, compact wire codes, and live OTA/ECC path | DQPSK goldens plus live 500/1000 delivery, duration, low-SNR drop, and same-rate overlap. |
| Header rate is S0; 8-128-kbit/s payload uses source `4 / payload_interval` timing | Exact non-rounded source rates and processing-gain helpers shared by airtime and BER; 500/1000 four-bit accounting intervals are derived from the owner-confirmed bit rates, and reuse of the processing-gain rule is explicit extension behavior | Five source rates plus two extension rates and gains match independent numeric vectors. |
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
versioned importer recovers file-local promoted attribute IDs, so it handles
both the normal CSR schema and the shifted one-node schema without hard-coded
field numbers. All eight supplied validation `*.nt.m` models and their paired
DES environments import successfully.

The ns-3 runner creates the corresponding node roles, coordinates, link
distances, rate/power limits, link margin, ECC threshold, frequency, duration,
and seed. It emits a canonical ordered trace across scenario construction,
application, MAC state, transmission, PHY accept/drop, NWK delivery, and route
changes. The comparator normalizes common OPNET CSV column/event aliases,
aligns event identities without discarding ordering, applies explicit numeric
tolerances, and reports missing, extra, replaced, or field-mismatched events.
Every run records normalized inputs, a JSON report, logs, hashes, and exact
commands in a manifest.

The supplied `.ov` files are aggregate output-vector databases rather than
packet-level event logs. They remain valid count/drop checkpoints, but an OPNET
CSV event export is required to populate the reference side of a full
differential certification. Exact formats, commands, assumptions, and this
boundary are documented in `docs/opnet-scenario-differential-harness.md`.

## Highest-priority remaining work

1. Export at least one authoritative OPNET packet/event CSV and run it through
   the differential harness, using recovered `.ov` totals as aggregate checks.
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
- Authoritative same-rate DQPSK collision curves, if they exist outside the
  supplied archive; the extended 500/1000-kbit/s profile treats the recorded
  jammer as payload noise and retains the DQPSK curve.

## Missing high-value scenarios

1. Duplicate DATA that re-ACKs without a second NWK enqueue or NSDP increment.
2. Route loss and recovery while mixed-DSCP traffic is held in NWK.
3. Sequence wraparound and cumulative windows older than 64 packets.
4. Sustained lossy overload across all queue limits with counter invariants.
5. Three-or-more-signal mixed-rate intervals and multiple competing same-rate
   JSR records under the recovered receiver-global last-collision state.
6. An authoritative OPNET CSV reference run using the now-identical imported
   topology, explicit traffic, and seed.
7. Lost KeyRequest, lost KeyUpdate ACK, and pairwise sequence/key-ID rollover
   during admission. Focused tests now cover authenticated security-count
   restart and wrap.

## Current regression baseline

All 30 CSR executable targets build on this audit revision. The 28 focused
parity smoke tests, `csr-mac-demo-split`, and the imported-scenario runner
workflow pass (30/30), along with all five focused Python importer/comparator
tests. The importer additionally decodes all eight supplied validation network
models and paired DES environments. The end-to-end fixture produces five
ordered events on each side with zero missing, extra, replaced, field-mismatch,
or coverage-gap results. The packet-envelope
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
and wrong-key rejection, pairwise auth-before-ACK/delivery, live encrypted
AppNeighborcast, malformed-envelope rejection, and all three live send paths.
The receive-contention test covers Search/Track timing, simultaneous collision,
strongest-preamble capture, post-lock interference, RX-induced slot freezing,
long- versus short-preamble duty-cycle behavior, and half-duplex loss.
The PHY-front-end test adds independent propagation, overlap, gain, noise, and
SNR vectors plus live channel-mismatch, different-rate additive-noise, and
same-rate JSR/time-offset paths. The BER/ECC test adds exact modulation-table
vectors, interpolation/quantization boundaries, source rates and processing
gain, interval truncation, binomial outcomes, ECC ordering, closure modes, and
live selected-collision acceptance and rejection.
The live high-rate test adds default-profile preservation, NWK-to-MAC extended
profile propagation, 500-kbit/s scenario caps, exact 500/1000 OTA durations,
production DQPSK BER and ECC outcomes, same-rate jammer-as-noise ordering, and
safe one-frame queue progress when no high-rate concat limit is available.
The admission test still covers pre-admission DATA
rejection, reciprocal key exchange,
ACK-driven outbound-key completion, NeighborCheck activation, route selection,
and exactly-once post-admission DATA delivery. The autonomous convergence test
still covers a lossless 0-1-2 line with no static routes, complete ARL snapshot
ACK/reassembly state, bidirectional two-hop learning, and exactly-once DATA
delivery. Warnings are limited to the pre-existing unused callback/CSV helper
warnings.
