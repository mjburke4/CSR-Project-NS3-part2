# OPNET parity audit

Updated: 2026-08-21

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
`arlSecurity.h`, `packetTypes.h`, and `msectime.*` additionally establish the
KeyRequest/KeyUpdate security records, millisecond timer units, and the
ACK-driven group-key lifecycle. Exact end-to-end certification still requires
the production cryptographic dependencies and authoritative OPNET/ns-3 trace
comparison.

## Current status

| Area | Estimated parity | Evidence and remaining uncertainty |
| --- | ---: | --- |
| Visible NWK wrapper behavior | 92-95% | Queue order/timing, blocked-entry scanning, DATA reliability, NSDP, reverse routes, capability handling, local-only link cost, gateway-driven SNMP discovery coordination, active-node accounting, and automatic changed-route propagation are covered. |
| Full NWK routing behavior | 84-90% | ARL byte-stream exchange, 24-bit node identifiers, and no-static-route multi-hop convergence now have source-backed coverage. Remaining uncertainty is concentrated in wrapper timing, non-address packet-model fields, and untested edge cases rather than an unavailable routing implementation. |
| ARL neighbor admission | 88-94% | Inactive/active state is now separate from freshness; two-sided key completion, deferred NeighborCheck proof, DATA gating, RoutingUpdate handling, security-count reset, and 5-second retry foundations are source-backed. Loss/restart edge cases still need broader trace coverage. |
| HOP | 88-92% | ACK/DACK windows, resend timing, flow control, queue limits, grouped routing ACKs, retry DSCP, custody, overhearing, link-control export, reliable KeyUpdate completion, and OPNET DATA-versus-control timeout effects are covered. |
| Production hop security | 35-45% | Exact 7-byte KeyRequest and 51-byte KeyUpdate records, pairwise counters, anti-replay state, key history, and security-count reset exist. AES/HMAC transforms, key derivation/provisioning, and authenticated acceptance are intentionally not implemented yet. |
| MAC transmit/control | 80-90% | Slot selection, holdoff, ACK priority, queue limits, concatenation, rate/power aggregation, preamble selection, and freshness are covered. |
| Full MAC including receive contention | 55-65% | OPNET Idle/Search/Track receive state, acquisition, overlapping signals, capture/collision, and RX-induced slot freezing are not yet reproduced. |
| PHY | 15-30% | The current log-distance/PER model does not reproduce the supplied OPNET radio pipeline. |

Overall estimates:

- NWK/HOP/MAC protocol-control behavior excluding PHY: 82-89% complete.
- Whole in-scope radio behavior including PHY, excluding battery and BBN:
  60-69%
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

## Step 1: ARL admission and hop-security foundation

The first parity step is complete at the behavioral and wire-foundation level.
The mapping is:

| Legacy source behavior | ns-3 implementation | Verification |
| --- | --- | --- |
| `routesRcvUnAuthedMsg(Discover)` resets 5000-ms delays and sends KeyRequest | `EvaluateNeighborAdmission()` and `SendKeyRequest()` | Admission smoke test observes KeyRequest and blocks pre-admission DATA. |
| KeyRequest is unicast Pairwise32 with `NoAck`/`NoResend` and neighbor immediate tag | `CsrHopLayer::SendKeyRequest()` and MAC type cancellation | Seven-byte golden vector plus lifecycle test. |
| KeyUpdate is reliable and carries KeyUpdate security | `CsrHopLayer::SendKeyUpdate()` through the existing ACK/resend queue | 51-byte golden vector and ACK-completion assertions. |
| Valid KeyUpdate is ACKed and causes a reciprocal update | HOP receive path and `NoteKeyUpdateReceived()` | Both peers must report sent and received keys. |
| Outbound key is sent only after its ACK | `MarkKeyUpdateAcked()` and `NoteKeyUpdateCompletion()` | State test proves enqueue alone is insufficient and records ACK time. |
| Both key directions plus CHECK_DISCOVERY/CHECK_OVERHEARD/CHECK_MESSAGE admit a neighbor | `TryMakeNeighborActive()` and the admission check state | Route is unusable before admission and selectable afterward. |
| Security-count change replaces the relationship | HOP anti-replay state and `NoteSecurityCountChange()` | State test accepts a restarted sequence only with a changed count and clears the old sent-key state. |

The KeyUpdate wire record deliberately reserves six data-PRN bytes because
that is what `hopSecLayer.c` places on the packet. `arlSecurity.h` defines a
five-byte pseudorandom value, so the sixth byte is treated as a preserved wire
byte. This discrepancy must remain visible when real crypto is connected.

This is not production authentication yet. The legacy source contains a
compile-time non-`SECURITY` fallback that zero-fills authentication fields, and
the ns-3 foundation currently models that behavior. The subsequently supplied
target `parameters.h` defines `SECURITY`, `SLIDINGWINDOW`, and `KEYS_IN_FLASH`,
so final target-build parity requires the enabled AES/HMAC and provisioned-key
path. The record sizes and lifecycle boundaries are ready for that provider
without changing routing admission.

## Highest-priority remaining non-PHY work

1. Implement and vector-test actual key provisioning, derivation, AES, HMAC,
   KeyRequest authentication, and KeyUpdate wrap/verify behavior. The supplied
   security bundle provides HMAC, key-lookup, parameters, sliding-window code,
   and the AES interface, but still lacks the AES and SHA-160 implementations,
   `arlSecurityInternal.c`, and a portable master/pairwise-key provisioning
   path. Its `arlSecurityInternal.h` also lacks the security-count parameters
   used by the supplied `arlSecurity.c`, indicating a source-version mismatch
   that should be resolved before treating it as an executable oracle.
2. Extend that crypto provider through the remaining `GroupEstablish`,
   `Group16`, and `Group32Encrypt` packet modes without changing the new
   admission state machine.
3. Resolve relay-holdoff signaling and control ordering against its legacy
   wrapper.
4. Finish the non-address packet-model audit, especially aggregate nesting and
   fields represented by ns-3 compatibility envelopes rather than direct
   `br_*.pk.m` equivalents.
5. Audit extra NWK queue wakeups and route/security edge cases with differential
   traces from identical OPNET/ns-3 topologies, seeds, and loss schedules.

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
7. Lost KeyRequest, lost KeyUpdate ACK, security-count restart, and pairwise
   sequence/key-ID rollover during admission.

## Current regression baseline

The complete ns-3 build, all 19 focused parity smoke tests, and
`csr-mac-demo-split` pass on this audit revision. The new admission test covers
pre-admission DATA rejection, reciprocal key exchange, ACK-driven outbound-key
completion, NeighborCheck activation, route selection, and exactly-once
post-admission DATA delivery. The wire test covers exact 7/51-byte security
records, shared pairwise sequencing, replay rejection, and security-count
reset. The autonomous convergence test still covers a lossless 0-1-2 line with
no static routes, complete ARL snapshot ACK/reassembly state, bidirectional
two-hop learning, and exactly-once DATA delivery. Warnings are limited to the
pre-existing unused callback/CSV helper warnings.
