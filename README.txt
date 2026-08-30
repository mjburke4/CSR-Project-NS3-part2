CSR ns-3 Module

Structure:
- model/           Header files for the CSR module (placed in src/csr/model/)
- CMakeLists.txt   ns-3 module build configuration (placed in src/csr/)
- csr-mac-demo-split.cc   Demo simulation (placed in scratch/)

Setup:
1. Copy model/ and CMakeLists.txt to ns-3-dev/src/csr/
2. Copy csr-mac-demo-split.cc to ns-3-dev/scratch/
3. Configure: ./ns3 configure --disable-examples
4. Build: ./ns3 build csr-mac-demo-split
5. Run: ./ns3 run csr-mac-demo-split

Cumulative ACK/DACK smoke test:
1. Copy csr-ack-window-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-ack-window-smoke
3. Run: ./ns3 run csr-ack-window-smoke

NWK/HOP integration parity test:
1. Copy csr-nwk-hop-integration-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-hop-integration-smoke
3. Run: ./ns3 run csr-nwk-hop-integration-smoke
4. Expected: PASS: NWK/HOP integration parity test

Strict OPNET NWK legacy behavior test:
1. Copy csr-nwk-legacy-strict-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-legacy-strict-smoke
3. Run: ./ns3 run csr-nwk-legacy-strict-smoke
4. Expected: PASS: strict OPNET NWK legacy behavior test

MAC/HOP sent-time parity test:
1. Copy csr-hop-mac-sent-time-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-mac-sent-time-smoke
3. Run: ./ns3 run csr-hop-mac-sent-time-smoke
4. Expected: PASS: MAC/HOP sent-time parity test

Exact DACK timer parity test:
1. Copy csr-hop-dack-hold-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-dack-hold-smoke
3. Run: ./ns3 run csr-hop-dack-hold-smoke
4. Expected: PASS: exact 20/40-second DACK timer parity test

OPNET MAC slot scheduling parity test:
1. Copy csr-mac-slot-parity-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-mac-slot-parity-smoke
3. Run: ./ns3 run csr-mac-slot-parity-smoke
4. Expected: PASS: OPNET MAC slot scheduling parity test

OPNET MAC ACK queue parity test:
1. Copy csr-mac-ack-queue-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-mac-ack-queue-smoke
3. Run: ./ns3 run csr-mac-ack-queue-smoke
4. Expected: PASS: OPNET MAC ACK queue parity test

OPNET MAC packet concatenation parity test:
1. Copy csr-mac-concat-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-mac-concat-smoke
3. Run: ./ns3 run csr-mac-concat-smoke
4. Expected: PASS: OPNET MAC packet concatenation parity test

OPNET HOP retransmission DSCP parity test:
1. Copy csr-hop-retry-dscp-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-retry-dscp-smoke
3. Run: ./ns3 run csr-hop-retry-dscp-smoke
4. Expected: PASS: OPNET HOP retransmission DSCP parity test

OPNET MAC/HOP queue-limit parity test:
1. Copy csr-mac-hop-queue-limit-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-mac-hop-queue-limit-smoke
3. Run: ./ns3 run csr-mac-hop-queue-limit-smoke
4. Expected: PASS: OPNET MAC/HOP queue-limit parity test

OPNET multidestination routing ACK parity test:
1. Copy csr-hop-multidest-routing-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-multidest-routing-smoke
3. Run: ./ns3 run csr-hop-multidest-routing-smoke
4. Expected: PASS: OPNET multidestination routing ACK parity test

OPNET preamble freshness and aggregate selection parity test:
1. Copy csr-mac-preamble-selection-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-mac-preamble-selection-smoke
3. Run: ./ns3 run csr-mac-preamble-selection-smoke
4. Expected: PASS: OPNET preamble freshness and aggregate selection parity test

OPNET wireless overhearing parity test:
1. Copy csr-wireless-overhearing-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-wireless-overhearing-smoke
3. Run: ./ns3 run csr-wireless-overhearing-smoke
4. Expected: PASS: OPNET wireless overhearing parity test

OPNET HOP/MAC live link-control parity test:
1. Copy csr-hop-mac-link-control-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-mac-link-control-smoke
3. Run: ./ns3 run csr-hop-mac-link-control-smoke
4. Expected: PASS: OPNET HOP/MAC live link-control parity test

OPNET no-route relay custody/ACK parity test:
1. Copy csr-hop-no-route-relay-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-no-route-relay-smoke
3. Run: ./ns3 run csr-hop-no-route-relay-smoke
4. Expected: PASS: no-route relay custody/ACK parity test

ARL routing byte-stream and reassembly parity test:
1. Copy csr-nwk-arl-routing-stream-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-arl-routing-stream-smoke
3. Run: ./ns3 run csr-nwk-arl-routing-stream-smoke
4. Expected: PASS: ARL routing byte-stream parity test

Source-owned self-route capability parity test:
1. Copy csr-nwk-self-capability-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-self-capability-smoke
3. Run: ./ns3 run csr-nwk-self-capability-smoke
4. Expected: PASS: source-owned self-route capability parity test

Gateway-driven discovery/SNMP convergence test:
1. Copy csr-nwk-autonomous-convergence-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-autonomous-convergence-smoke
3. Run: ./ns3 run csr-nwk-autonomous-convergence-smoke
4. Expected: PASS: gateway-driven discovery/SNMP convergence test

This scenario starts only the Gateway at the legacy 10-second offset. It
checks three serialized 15-second local-discovery windows, ordered one-hop
SNMP START/DONE handoffs, autonomous ARL convergence, and two-hop DATA
delivery.

OPNET discovery cadence/completion and destination-order test:
1. Copy csr-nwk-discovery-ordering-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-discovery-ordering-smoke
3. Run: ./ns3 run csr-nwk-discovery-ordering-smoke
4. Expected: PASS: OPNET discovery cadence/completion and destination order

This locks Gateway discovery broadcasts at 10, 15, and 20 seconds, completion
on the fourth discovery tick at 25 seconds, and newest-first logical
destination handoff. A reverse-only destination must emit SNMP START through
its selected reverse HOP neighbor while retaining the original final SNMP
destination.

Exact 24-bit node-ID wire serialization test:
1. Copy csr-wire-format-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-wire-format-smoke
3. Run: ./ns3 run csr-wire-format-smoke
4. Expected: PASS: exact CSR 24-bit wire serialization test

The demo includes headers via "ns3/csr-*.h" which resolves to the module.

OPNET scenario import and differential comparison:
1. Copy csr-opnet-scenario-runner.cc to ns-3-dev/scratch/.
2. Build: ./ns3 build csr-opnet-scenario-runner
3. Import a *.nt.m file or project ZIP with utils/import-opnet-scenario.py.
4. Run and compare with utils/run-opnet-differential.py.
5. See docs/opnet-scenario-differential-harness.md for schemas, commands,
   tolerances, artifacts, and the OPNET CSV export boundary.

Historical OPNET aggregate comparison:
1. Decode a recovered Modeler *.ov with utils/extract-opnet-ov.py, optionally
   cross-checking its paired *.pb.m definition.
2. Run an imported scenario and compare source-equivalent ns-3 buckets with
   utils/run-opnet-aggregate-differential.py.
3. See docs/opnet-aggregate-comparison.md for the canonical schema, exact
   source-level application packet sizing, measured historical mismatches,
   and the aggregate-versus-event evidence boundary.

ns-3 packet-path and NWK-residence diagnostic:
1. Run csr-opnet-scenario-runner with --aggregateTraceOnly=1 to retain compact
   app_send, nwk_enqueue, nwk_forward, nwk_delivery, and route_change events.
2. Run utils/analyze-ns3-packet-paths.py TRACE.csv --summary-json SUMMARY.json
   --packet-csv PACKETS.csv --stop-time STOP_SECONDS --strict.
3. The analyzer reconstructs each (source, destination, sequence) path and
   checks route context, loops, NWK residence, per-leg transit, and exact
   end-to-end decomposition. See docs/opnet-scenario-differential-harness.md.

The deterministic three-node reservation/collision reference, SHA-checked
OPNET instrumentation, evidence validator, and licensed-run handoff are in
docs/opnet-reservation-collision-reference.md.
