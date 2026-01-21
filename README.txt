This folder contains a mechanical split of csr-mac-core-demo (17).cc into separate include files.

Files:
- csr-common.h: shared includes, constants/enums, CSV helpers, CsrHeader, forward decls.
- csr-mac-core.h: CsrMacCore class (header-only, includes csr-common.h).
- csr-phy-model.h: CsrPhyModel + CsrRxDecision (header-only).
- csr-net-device.h: CsrNetDevice glue (header-only).
- csr-hop-layer.h: CsrHopLayer (header-only).
- csr-nwk-layer.h: CsrNetLayer (header-only).
- csr-mac-demo-split.cc: scratch-style main that includes the above and runs the scenario.

Notes:
- This is header-only for speed and to avoid linker issues while you migrate into a proper ns-3 module.
- Next step is to convert these headers into .cc/.h pairs (move method bodies into .cc) and add src/csr/CMakeLists.txt.
