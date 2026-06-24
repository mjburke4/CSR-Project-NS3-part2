OPNET file/function        NS-3 location                  Status
br_nwk.start_discovery     CsrNetLayer::StartDiscovery    close, with optional repeat gated
br_nwk.proc_hello          CsrNetLayer::ProcessHello      approximate
br_nwk.update_route_table  AddOrUpdateRoute/ProcessHello  partial
br_nwk.link_calc           ComputeLinkCost                placeholder
br_hop ACK/resend          CsrHopLayer                    partial/close
br_mac tx scheduling       CsrMacCore                     partial
Routes_PAYLOAD            CsrHelloHeader vector           approximation