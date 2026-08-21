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
source-backed routing audit. Exact end-to-end certification still requires
matching the remaining OPNET wrappers and comparing authoritative OPNET/ns-3
traces.

## Current status

| Area | Estimated parity | Evidence and remaining uncertainty |
| --- | ---: | --- |
| Visible NWK wrapper behavior | 92-95% | Queue order/timing, blocked-entry scanning, DATA reliability, NSDP, reverse routes, capability handling, local-only link cost, gateway-driven SNMP discovery coordination, active-node accounting, and automatic changed-route propagation are covered. |
| Full NWK routing behavior | 84-90% | ARL byte-stream exchange, 24-bit node identifiers, and no-static-route multi-hop convergence now have source-backed coverage. Remaining uncertainty is concentrated in wrapper timing, non-address packet-model fields, and untested edge cases rather than an unavailable routing implementation. |
| HOP | 85-90% | ACK/DACK windows, resend timing, flow control, queue limits, grouped routing ACKs, retry DSCP, custody, overhearing, link-control export, and OPNET DATA-versus-control timeout effects are covered. |
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

## Highest-priority remaining non-PHY work

1. Finish the non-address packet-model audit, especially aggregate nesting and
   fields represented by the ns-3 compatibility envelopes rather than direct
   `br_*.pk.m` equivalents.
2. Resolve the remaining ARL wrappers, especially `KeyRequest` and
   relay-holdoff signaling/control ordering.
3. Audit extra NWK queue wakeups and edge-case route transitions against
   differential traces from identical OPNET/ns-3 topologies and seeds.

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

## Current regression baseline

The complete ns-3 build, all 18 focused parity smoke tests, and
`csr-mac-demo-split` pass on this audit revision. The autonomous convergence
test covers a lossless 0-1-2 line with no static routes, complete ARL snapshot
ACK/reassembly state, bidirectional two-hop learning, and exactly-once DATA
delivery. Warnings are limited to the pre-existing unused callback/CSV helper
warnings.
