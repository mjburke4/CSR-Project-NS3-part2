# OPNET parity audit

Updated: 2026-08-24

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
millisecond timer units, the ACK-driven group-key lifecycle, all three group
security modes, and the source's key-evolution rules. Exact end-to-end
certification still requires authoritative OPNET/ns-3 trace comparison.

## Current status

| Area | Estimated parity | Evidence and remaining uncertainty |
| --- | ---: | --- |
| Visible NWK wrapper behavior | 94-97% | Queue order/timing, blocked-entry scanning, DATA reliability, NSDP, reverse routes, capability handling, local-only link cost, gateway-driven SNMP discovery coordination, relay-holdoff/clear state, active-node accounting, automatic changed-route propagation, and fixed packet/control envelopes are covered. |
| Full NWK routing behavior | 86-91% | ARL byte-stream exchange, 24-bit node identifiers, exact supplied `br_Routes`/`br_SNMP` fixed fields, and no-static-route multi-hop convergence now have source-backed coverage. Remaining uncertainty is concentrated in wrapper timing and untested edge cases rather than an unavailable routing implementation. |
| ARL neighbor admission | 88-94% | Inactive/active state is now separate from freshness; two-sided key completion, deferred NeighborCheck proof, DATA gating, RoutingUpdate handling, security-count reset, and 5-second retry foundations are source-backed. Loss/restart edge cases still need broader trace coverage. |
| HOP | 91-95% | ACK/DACK windows, resend timing, flow control, queue limits, grouped routing ACKs, retry DSCP, custody, overhearing, link-control export, reliable KeyUpdate completion, authenticated Discover/routing control, one-hop SNMP dispatch ordering, exact fixed HOP/ACK envelope models, and OPNET DATA-versus-control timeout effects are covered. |
| Full hop security | 78-85% | KeyRequest, KeyUpdate, GroupEstablish, Group16, and the Group32Encrypt provider now use source-faithful KDF, HMAC, AES, key-rotation, replay, deferred-decode, and authentication-before-ACK/admission behavior. Remaining gaps are live AppNeighborcast Group32 integration, Pairwise16/Pairwise32Encrypt data paths, exact outer packet wrappers, and operational key-store hardware. |
| MAC transmit/control | 85-92% | Slot selection, holdoff, ACK priority, queue limits, exact OPNET segment-size accounting, concatenation, rate/power aggregation, preamble selection, and freshness are covered. |
| Full MAC including receive contention | 55-65% | OPNET Idle/Search/Track receive state, acquisition, overlapping signals, capture/collision, and RX-induced slot freezing are not yet reproduced. |
| PHY | 15-30% | The current log-distance/PER model does not reproduce the supplied OPNET radio pipeline. |

Overall estimates:

- NWK/HOP/MAC protocol-control behavior excluding PHY: 88-93% complete.
- Whole in-scope radio behavior including PHY, excluding battery and BBN:
  64-72%
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
  delivery, or ACK. Group32Encrypt is available through the same portable
  provider with independent golden-vector coverage.
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

## Highest-priority remaining non-PHY work

1. Finish the live AppNeighborcast Group32 path and remaining Pairwise16 /
   Pairwise32Encrypt modes.
2. Audit extra NWK queue wakeups and route/security edge cases with differential
   traces from identical OPNET/ns-3 topologies, seeds, and loss schedules.
3. Implement the OPNET MAC receive/contention state machine.
4. Then calibrate the supplied OPNET PHY pipeline against differential traces.

## Deferred MAC/PHY work

- OPNET Idle/Search/Track/Tx receive-state transitions and DSP acquisition.
- Simultaneous-signal interference, hidden terminals, capture, and collision.
- Receiver-group qualification and synchronization detection.
- Header versus payload BER, ECC, accumulated interference, and closure/error
  statistics from `br_snr`, `br_inoise`, `br_ber`, `br_rxgroup`, `br_power`,
  `br_txdel`, `br_ecc`, `br_closure`, and `br_error_all_stats`.
- Exact OPNET field/chip-derived airtime, preamble duration, propagation delay,
  and the 500/1000 kbps cases.

## Missing high-value scenarios

1. Duplicate DATA that re-ACKs without a second NWK enqueue or NSDP increment.
2. Route loss and recovery while mixed-DSCP traffic is held in NWK.
3. Sequence wraparound and cumulative windows older than 64 packets.
4. Sustained lossy overload across all queue limits with counter invariants.
5. Hidden-terminal and sleep/wake boundary cases after MAC/PHY work begins.
6. Differential traces using identical OPNET/ns-3 topology, traffic, and seed.
7. Lost KeyRequest, lost KeyUpdate ACK, and pairwise sequence/key-ID rollover
   during admission. Focused tests now cover authenticated security-count
   restart and wrap.

## Current regression baseline

The complete ns-3 build, all 23 focused parity smoke tests, and
`csr-mac-demo-split` pass on this audit revision (24/24). The packet-envelope
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
epoch redistribution. The admission test still covers pre-admission DATA
rejection, reciprocal key exchange,
ACK-driven outbound-key completion, NeighborCheck activation, route selection,
and exactly-once post-admission DATA delivery. The autonomous convergence test
still covers a lossless 0-1-2 line with no static routes, complete ARL snapshot
ACK/reassembly state, bidirectional two-hop learning, and exactly-once DATA
delivery. Warnings are limited to the pre-existing unused callback/CSV helper
warnings.
