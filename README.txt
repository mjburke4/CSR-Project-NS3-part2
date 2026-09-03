CSR ns-3 Module

Structure:
- model/           CSR module sources and headers (placed in contrib/csr/model/)
- CMakeLists.txt   ns-3 module build configuration (placed in contrib/csr/)
- csr-*.cc         38 executable validation programs (placed in scratch/)

Setup:
1. Copy or link this repository as ns-3-dev/contrib/csr/.
2. Copy or link all 38 top-level csr-*.cc files into ns-3-dev/scratch/.
3. Configure: ./ns3 configure --disable-examples
4. Build every target in the complete inventory below. The module CMakeLists.txt
   builds the CSR library; ns-3 creates the executable targets from scratch/.

Complete executable workflow inventory:

There are 36 focused smoke workflows:

  csr-ack-window-smoke
  csr-hop-dack-hold-smoke
  csr-hop-group-security-smoke
  csr-hop-mac-link-control-smoke
  csr-hop-mac-sent-time-smoke
  csr-hop-multidest-routing-smoke
  csr-hop-neighbor-check-security-smoke
  csr-hop-no-route-relay-smoke
  csr-hop-remaining-security-smoke
  csr-hop-retry-dscp-smoke
  csr-hop-security-crypto-smoke
  csr-live-high-rate-dqpsk-smoke
  csr-mac-ack-queue-smoke
  csr-mac-concat-smoke
  csr-mac-hop-queue-limit-smoke
  csr-mac-preamble-selection-smoke
  csr-mac-receive-contention-smoke
  csr-mac-reservation-lifecycle-smoke
  csr-mac-slot-parity-smoke
  csr-nwk-arl-admission-security-smoke
  csr-nwk-arl-routing-stream-smoke
  csr-nwk-autonomous-convergence-smoke
  csr-nwk-discovery-ordering-smoke
  csr-nwk-grouped-route-propagation-smoke
  csr-nwk-hop-integration-smoke
  csr-nwk-legacy-strict-smoke
  csr-nwk-relay-holdoff-smoke
  csr-nwk-residual-routing-retry-smoke
  csr-nwk-same-pass-info-coupling-smoke
  csr-nwk-self-capability-smoke
  csr-opnet-packet-envelope-smoke
  csr-phy-ber-ecc-smoke
  csr-phy-front-end-smoke
  csr-queue-observation-smoke
  csr-wire-format-smoke
  csr-wireless-overhearing-smoke

Each smoke binary takes no arguments. Run each in its own writable working
directory and require exit status zero, exactly one output line beginning
"PASS:", and no output line beginning "FAIL:".

The other two workflows are:

  csr-mac-demo-split
  csr-opnet-scenario-runner

Their required isolated invocations are documented below. These 36 smoke
workflows plus the demo and fixture-backed runner are the complete 38-program
ledger; repeated runner invocations by the differential tools are not extra
executable targets.

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
4. Expected: PASS: exact MAC/HOP resend timer and scan-order parity test

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

Source-exact Pairwise16 NeighborCheck wrapper test:
1. Copy csr-hop-neighbor-check-security-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-neighbor-check-security-smoke
3. Run: ./ns3 run csr-hop-neighbor-check-security-smoke
4. Expected: PASS: source-exact Pairwise16 NeighborCheck wrapper test

OPNET NWK-owned residual routing retry parity test:
1. Copy csr-nwk-residual-routing-retry-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-residual-routing-retry-smoke
3. Run: ./ns3 run csr-nwk-residual-routing-retry-smoke
4. Expected: PASS: NWK-owned residual routing retry

OPNET same-pass routing INFO coupling parity test:
1. Copy csr-nwk-same-pass-info-coupling-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-nwk-same-pass-info-coupling-smoke
3. Run: ./ns3 run csr-nwk-same-pass-info-coupling-smoke
4. Expected: PASS: same-pass routing INFO coupling

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

The demo includes headers via "ns3/csr-*.h" which resolves to the module. Run
the built demo binary directly from a new isolated directory containing an
empty results/ directory. For example, with absolute paths in DEMO_BIN and
DEMO_RUN_DIR:

  mkdir -p "$DEMO_RUN_DIR/results"
  (cd "$DEMO_RUN_DIR" && "$DEMO_BIN" >program.log 2>&1)

A valid demo workflow exits zero and creates nonempty results/rx_metrics.csv
and results/nsdp_metrics.csv, each with a header and at least one data row. Do
not run the release check from the source tree, where pre-existing results
could be mistaken for fresh output.

OPNET fixture runner and differential comparison:
1. Copy csr-opnet-scenario-runner.cc to ns-3-dev/scratch/.
2. Build: ./ns3 build csr-opnet-scenario-runner
3. Set RUNNER_BIN and CSR_SOURCE to absolute paths, create a new isolated
   RUNNER_RUN_DIR, and run the built binary directly:

     (cd "$RUNNER_RUN_DIR" && "$RUNNER_BIN" \
       --scenario="$CSR_SOURCE/utils/testdata/csr-scenario-fixture.csv" \
       --trace="$RUNNER_RUN_DIR/ns3-trace.csv" \
       --quietModelLogs=1 >program.log 2>&1)

4. Require exit status zero and exactly six data rows in ns3-trace.csv. The
   --scenario argument is mandatory; a naked runner invocation is not a valid
   workflow.
5. Run utils/run-opnet-differential.py with that scenario, runner, and
   utils/testdata/opnet-trace-fixture.csv. Require six matched events and zero
   missing, extra, replaced, field-mismatch, or coverage-gap events.
6. For a recovered scenario, import a *.nt.m file or project ZIP with
   utils/import-opnet-scenario.py before running the appropriate differential.
7. See docs/opnet-scenario-differential-harness.md for schemas, commands,
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
1. Run csr-opnet-scenario-runner with --aggregateTraceOnly=1 and
   --admissionTrace=1 to retain compact packet-path events plus the exact HOP
   identities needed to split any source-proved repeated-DACK branches. A
   compact trace without the admission surface remains sufficient for an
   ordinary one-copy path, but fails closed if repeated branch observations
   occur.
2. Run utils/analyze-ns3-packet-paths.py TRACE.csv --summary-json SUMMARY.json
   --packet-csv PACKETS.csv --stop-time STOP_SECONDS --strict.
3. The analyzer reconstructs each (source, destination, sequence) path and
   checks route context, loops, NWK residence, per-leg transit, and exact
   end-to-end decomposition. See docs/opnet-scenario-differential-harness.md.

The deterministic three-node reservation/collision reference, SHA-checked
OPNET instrumentation, evidence validator, and licensed-run handoff are in
docs/opnet-reservation-collision-reference.md.

Python utility regression suite:

  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
    -s utils/tests -p 'test_*.py' -v

Discovery at this revision reports 202 tests. That is the current observed
inventory, not a permanent invariant: added coverage may legitimately increase
it. Release evidence should record and validate the count discovered at the
tested commit rather than carrying an older hard-coded total forward.
