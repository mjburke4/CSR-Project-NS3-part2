# OPNET parity audit

Updated: 2026-08-20

## Scope and confidence limits

Battery/energy behavior is excluded because it was experimental and was not
part of the deployed system. This audit compares the supplied OPNET process
and pipeline files with the ns-3 model.

`br_nwk.pr.c` delegates core routing decisions to an external routing library
through functions such as `routesCreate`, `routesRcvMsg`, `routesProcess`, and
`routesValidRx`. Those sources and the original packet-format definitions are
not available. The visible NWK wrapper can be matched closely, but exact
routing-algorithm and wire-format parity cannot be certified without those
sources or authoritative OPNET traces.

## Current status

| Area | Estimated parity | Evidence and remaining uncertainty |
| --- | ---: | --- |
| Visible NWK wrapper behavior | 85-90% | Queue order/timing, blocked-entry scanning, DATA reliability, NSDP, reverse routes, capability handling, discovery/control wrappers, and active-node accounting are covered. External routing-library semantics remain uncertain. |
| Full NWK routing behavior | 65-75% | Substantial route discovery/update/failover behavior exists, but the unavailable routing library prevents exact comparison. |
| HOP | 80-90% | ACK/DACK windows, resend timing, flow control, queue limits, grouped routing ACKs, retry DSCP, custody, overhearing, and link-control export are covered. Edge failure bookkeeping still needs work. |
| MAC transmit/control | 80-90% | Slot selection, holdoff, ACK priority, queue limits, concatenation, rate/power aggregation, preamble selection, and freshness are covered. |
| Full MAC including receive contention | 55-65% | OPNET Idle/Search/Track receive state, acquisition, overlapping signals, capture/collision, and RX-induced slot freezing are not yet reproduced. |
| PHY | 15-30% | The current log-distance/PER model does not reproduce the supplied OPNET radio pipeline. |

Overall estimates:

- NWK/HOP/MAC protocol-control behavior excluding PHY: 75-85% complete.
- Whole in-scope radio behavior including PHY, excluding battery: 55-65%
  complete.

These ranges reflect confidence in observable behavior, not code-volume
completion.

## Completed or close

- Strict NWK positive-DSCP head insertion and best-effort tail insertion.
- One-`TIC` NWK queue release, blocked-entry scanning, and no implicit
  discovery from the DATA queue.
- OPNET-global ACK/resend behavior for NWK DATA.
- HELLO capability preservation and monotonic active-node population.
- NWK/HOP admission, NSDP ACK/DACK selection, and global pending-DATA state.
- Relay custody, reverse routes, invalidation, alternate paths, and no-route
  ACK behavior.
- 16-bit HOP sequences, cumulative ACK/DACK windows, duplicate suppression,
  retransmission timing from actual MAC send time, and 512-entry resend limit.
- Multi-destination reliable routing-control ACK bookkeeping.
- MAC OPNET slot selection/holdoff, ACK queue semantics, 512-entry DATA limit,
  concatenation, retransmission DSCP, and head blocking.
- Aggregate rate/power and preamble selection across all destinations,
  preamble freshness, wireless overhearing, and live HOP link-control results.

## Highest-priority remaining non-PHY work

1. Correct grouped routing-control partial timeout handling. If the primary
   destination ACKs but a secondary times out, the primary must not receive the
   link-failure penalty.
2. Match OPNET routing-control versus DATA timeout effects. A final DATA
   timeout should not automatically become a routing-link failure when no
   routing `Packet_Tx_Info` exists.
3. Remove or gate negotiated bidirectional link-cost behavior. The supplied
   OPNET wrapper computes one local `link_calc` result and copies it to the
   remote cost; the ns-3 negotiation is an enhancement rather than parity.
4. Validate autonomous multi-hop convergence from empty route tables. Current
   deterministic integration coverage begins with installed routes.
5. Resolve remaining routing wrappers: `KeyRequest`, exact SNMP discovery and
   relay-holdoff signaling, discovery cooldown/restart timing, and address or
   broadcast-width differences.
6. Audit extra NWK queue wakeups against the unavailable routing library.

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
2. Grouped routing control where the primary ACKs and a secondary times out.
3. Duplicate DATA that re-ACKs without a second NWK enqueue or NSDP increment.
4. Route loss and recovery while mixed-DSCP traffic is held in NWK.
5. Sequence wraparound and cumulative windows older than 64 packets.
6. Sustained lossy overload across all queue limits with counter invariants.
7. Hidden-terminal and sleep/wake boundary cases after MAC/PHY work begins.
8. Differential traces using identical OPNET/ns-3 topology, traffic, and seed.

## Current regression baseline

The complete ns-3 build, all 15 focused parity smoke tests, and
`csr-mac-demo-split` pass on this audit revision. Warnings are limited to the
pre-existing unused callback/CSV helper warnings.
