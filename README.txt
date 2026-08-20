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

MAC/HOP sent-time parity test:
1. Copy csr-hop-mac-sent-time-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-mac-sent-time-smoke
3. Run: ./ns3 run csr-hop-mac-sent-time-smoke
4. Expected: PASS: MAC/HOP sent-time parity test

Staggered DACK hold parity test:
1. Copy csr-hop-dack-hold-smoke.cc to ns-3-dev/scratch/
2. Build: ./ns3 build csr-hop-dack-hold-smoke
3. Run: ./ns3 run csr-hop-dack-hold-smoke
4. Expected: PASS: staggered DACK hold parity test

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

The demo includes headers via "ns3/csr-*.h" which resolves to the module.
