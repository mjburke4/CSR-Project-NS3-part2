build_lib(
  LIBNAME csr
  SOURCE_FILES
    model/csr-module.cc
    model/csr-hello-header.cc
  HEADER_FILES
    model/csr-common.h
    model/csr-hello-header.h
    model/csr-phy-model.h
    model/csr-mac-core.h
    model/csr-net-device.h
    model/csr-hop-layer.h
    model/csr-nwk-layer.h
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libspectrum}
)
