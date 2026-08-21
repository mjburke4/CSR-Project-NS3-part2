# OPNET parity audit

Updated: 2026-08-21

## Scope and confidence limits

Battery/energy behavior is excluded because it was experimental and was not
part of the deployed system. BBN routing is also excluded; the target is the
`__ARL_ROUTING__` configuration selected by the supplied `csr_api.h`. This
audit compares the supplied OPNET process, pipeline, ARL API, and packet-model
files with the ns-3 model.

`br_nwk.pr.c` delegates core routing decisions to an external routing library
through functions such as `routesCreate`, `routesRcvMsg`, `routesProcess`, and
`routesValidRx`. The supplied archive now provides the declarations and data
contracts in `csr_api_routes.h`, `csr_api_linkchar.h`, and the original
`br_*.pk.m` packet-model definitions. It does not contain the implementation
that defines the `routes*` functions. The visible NWK wrapper and packet
contracts can therefore be matched more closely, but exact routing-algorithm
parity still cannot be certified without that implementation or authoritative
OPNET traces.

## Current status

| Area | Estimated parity | Evidence and remaining uncertainty |
| --- | ---: | --- |
| Visible NWK wrapper behavior | 88-93% | Queue order/timing, blocked-entry scanning, DATA reliability, NSDP, reverse routes, capability handling, local-only link cost, discovery/control wrappers, and active-node accounting are covered. External routing-library semantics remain uncertain. |
| Full NWK routing behavior | 70-80% | Substantial route discovery/update/failover behavior and the ARL API/packet contracts are covered, but the unavailable `routes*` implementation prevents exact comparison. |
| HOP | 85-90% | ACK/DACK windows, resend timing, flow control, queue limits, grouped routing ACKs, retry DSCP, custody, overhearing, link-control export, and OPNET DATA-versus-control timeout effects are covered. |
| MAC transmit/control | 80-90% | Slot selection, holdoff, ACK priority, queue limits, concatenation, rate/power aggregation, preamble selection, and freshness are covered. |
| Full MAC including receive contention | 55-65% | OPNET Idle/Search/Track receive state, acquisition, overlapping signals, capture/collision, and RX-induced slot freezing are not yet reproduced. |
| PHY | 15-30% | The current log-distance/PER model does not reproduce the supplied OPNET radio pipeline. |

Overall estimates:

- NWK/HOP/MAC protocol-control behavior excluding PHY: 78-86% complete.
- Whole in-scope radio behavior including PHY, excluding battery and BBN:
  58-67%
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

1. Validate autonomous multi-hop convergence from empty route tables. Current
   deterministic integration coverage begins with installed routes.
2. Audit ns-3 serialization against the newly supplied `br_*.pk.m` field
   definitions, especially routing-operation widths, target/broadcast values,
   and aggregate nesting.
3. Resolve remaining ARL routing wrappers: `KeyRequest`, exact SNMP discovery and
   relay-holdoff signaling, discovery cooldown/restart timing, and address or
   broadcast-width differences.
4. Audit extra NWK queue wakeups against observable ARL wrapper behavior and
   differential traces; the underlying `routes*` implementation is still
   unavailable.

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

1. Natural three- or four-node convergence with no static routes.
2. Duplicate DATA that re-ACKs without a second NWK enqueue or NSDP increment.
3. Route loss and recovery while mixed-DSCP traffic is held in NWK.
4. Sequence wraparound and cumulative windows older than 64 packets.
5. Sustained lossy overload across all queue limits with counter invariants.
6. Hidden-terminal and sleep/wake boundary cases after MAC/PHY work begins.
7. Differential traces using identical OPNET/ns-3 topology, traffic, and seed.

## Current regression baseline

The complete ns-3 build, all 15 focused parity smoke tests, and
`csr-mac-demo-split` pass on this audit revision. The strict NWK smoke test now
also covers asymmetric remote RoutingInfo during initial link calculation and
failure-driven cost recomputation. Warnings are limited to the pre-existing
unused callback/CSV helper warnings.
