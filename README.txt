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

The demo includes headers via "ns3/csr-*.h" which resolves to the module.
