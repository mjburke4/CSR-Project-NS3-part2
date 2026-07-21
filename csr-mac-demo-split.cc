#include "ns3/csr-common.h"
#include "ns3/csr-mac-core.h"
#include "ns3/csr-phy-model.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"

// main()
// ------------------------------------------------------------

int
main (int argc, char *argv[])
{
  Time::SetResolution (Time::NS);

  OpenRxCsv ("results/rx_metrics.csv");
  OpenNsdpCsv ("results/nsdp_metrics.csv");
  // Create three devices / nodes
  Ptr<CsrNetDevice> dev0 = CreateObject<CsrNetDevice> (0);
  Ptr<CsrNetDevice> dev1 = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> dev2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> dev3 = CreateObject<CsrNetDevice> (3);

  dev0->EnableDutyCycling (true);
  dev1->EnableDutyCycling (true);
  dev2->EnableDutyCycling (true);
  dev3->EnableDutyCycling (true);

 /* // Line topology: 0 <-> 1 <-> 2
  dev0->AddPeer (dev1);   // 0 can talk to 1
  dev1->AddPeer (dev0);   // 1 can talk to 0
  dev1->AddPeer (dev2);   // 1 can talk to 2
  dev2->AddPeer (dev1);   // 2 can talk to 1
*/

  // Line topology: 0 <-> 1 <-> 2 <-> 3
  dev0->AddPeer (dev1);

  dev1->AddPeer (dev0);
  dev1->AddPeer (dev2);

  dev2->AddPeer (dev1);
  dev2->AddPeer (dev3);

  dev3->AddPeer (dev2);

  CsrPhyProfile csnw;
  csnw.txPowerDbm = 0.0;
  csnw.noiseFloorDbm = -106.975; //-100.0;
  //csnw.refLossDb = 60.0;
  //csnw.pathlossExp = 2.0;
  csnw.refLossDb   = 70.0;
  csnw.pathlossExp = 2.2;
  csnw.distanceScale = 1.0;

  // Apply to all nodes
  dev0->GetPhy().SetProfile(csnw);
  dev1->GetPhy().SetProfile(csnw);
  dev2->GetPhy().SetProfile(csnw);
  dev3->GetPhy().SetProfile(csnw);

  auto berTable = std::make_shared<CsrBerTableModel>();

  // Use the SAME curve for all rates for now (processing gain already depends on rate)
  std::vector<double> snr = { -3, 0, 3, 6, 9, 12, 15, 18, 21, 24 };
  std::vector<double> ber = { 1e-1, 3e-2, 1e-2, 3e-3, 1e-3, 3e-4, 1e-4, 3e-5, 1e-5, 3e-6 };

  berTable->AddCurve({8,   snr, ber});
  berTable->AddCurve({16,  snr, ber});
  berTable->AddCurve({32,  snr, ber});
  berTable->AddCurve({64,  snr, ber});
  berTable->AddCurve({128, snr, ber});

  // Hook into PHY (you can store berTable in the PHY, or wrap it in a lambda)
  // IMPORTANT: reuse perModel hook, but treat it as BER(effSNR) for now
  auto berHook3 = [berTable](int rateKbps, double effSnrDb, uint32_t nBits) -> double {
  return berTable->GetBerViaHook(rateKbps, effSnrDb, nBits);
  };

  auto berFn = [berTable](int rateKbps, double effSnrDb, uint32_t /*nBits*/) -> double {
  return berTable->GetBer(rateKbps, effSnrDb);
  };

  dev0->GetPhy().SetPerModel(berFn);
  dev1->GetPhy().SetPerModel(berFn);
  dev2->GetPhy().SetPerModel(berFn);
  dev3->GetPhy().SetPerModel(berFn);

  // --- simple fixed link distances for line topology ---
  auto setAllLinkDistances = [&](Ptr<CsrNetDevice> d)
  {
    d->GetPhy().SetLinkDistanceMeters(0, 1, 50.0);
    d->GetPhy().SetLinkDistanceMeters(1, 2, 120.0);
    d->GetPhy().SetLinkDistanceMeters(2, 3, 80.0);
    d->GetPhy().SetDefaultDistanceMeters(1.0); // only used if missing entry
  };

  dev0->SetActiveNodesForPostTx (1);
  dev1->SetActiveNodesForPostTx (1);
  dev2->SetActiveNodesForPostTx (1);
  dev3->SetActiveNodesForPostTx (1);
  
  setAllLinkDistances(dev0);
  setAllLinkDistances(dev1);
  setAllLinkDistances(dev2);
  setAllLinkDistances(dev3);

  // --- PER table stub (replace with MATLAB curves later) ---
  auto perTable = std::make_shared<CsrPerTableModel> ();

  // Example stub curves (totally placeholder numbers)
  perTable->AddCurve ({ 8,   {-10,-8,-6,-4,-2,0,2,4,6,8}, {1.0,1.0,0.9,0.7,0.4,0.2,0.1,0.03,0.01,0.0} });
  perTable->AddCurve ({ 16,  {-8,-6,-4,-2,0,2,4,6,8,10}, {1.0,0.9,0.7,0.45,0.25,0.12,0.06,0.02,0.005,0.0} });
  perTable->AddCurve ({ 32,  {-6,-4,-2,0,2,4,6,8,10,12}, {1.0,0.85,0.6,0.35,0.18,0.09,0.04,0.015,0.003,0.0} });
  perTable->AddCurve({ 64,   {-4,-2,0,2,4,6,8,10,12,14}, {1.0,0.95,0.8,0.6,0.35,0.18,0.08,0.03,0.01,0.0}});


  auto perFn = [perTable](int rateKbps, double snrDb, uint32_t nBits) -> double {
    return perTable->GetPer (rateKbps, snrDb, nBits);
  };
  

  dev0->GetPhy().SetPerModel (perFn);
  dev1->GetPhy().SetPerModel (perFn);
  dev2->GetPhy().SetPerModel (perFn);
  dev3->GetPhy().SetPerModel (perFn);

  // Create Hop layers
  Ptr<CsrHopLayer> hop0 = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> hop1 = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> hop2 = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> hop3 = CreateObject<CsrHopLayer> ();

  hop0->SetNodeId (0);
  hop1->SetNodeId (1);
  hop2->SetNodeId (2);
  hop3->SetNodeId (3);

  hop0->SetMac (&dev0->GetMac ());
  hop1->SetMac (&dev1->GetMac ());
  hop2->SetMac (&dev2->GetMac ());
  hop3->SetMac (&dev3->GetMac ());

  // Create Net layers
  Ptr<CsrNetLayer> net0 = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> net1 = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> net2 = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> net3 = CreateObject<CsrNetLayer> ();

  net0->SetNodeId (0);
  net1->SetNodeId (1);
  net2->SetNodeId (2);
  net3->SetNodeId (3);
  net1->SetRepeatDiscoveryHello (false);

  // Periodic NSDP sampler
  std::function<void()> sampleNsdp;
  sampleNsdp = [&]() {
    if (g_nsdpCsv.is_open ())
      {
        net0->DumpNsdp (g_nsdpCsv);
        net1->DumpNsdp (g_nsdpCsv);
        net2->DumpNsdp (g_nsdpCsv);
        net3->DumpNsdp (g_nsdpCsv);
      }
    Simulator::Schedule (Seconds (1.0), sampleNsdp);
  };
  Simulator::Schedule (Seconds (1.0), sampleNsdp);

  net0->SetHop (hop0);
  net1->SetHop (hop1);
  net2->SetHop (hop2);


 // ------------------------------------------------------------
  // Static seed routes for 4-node propagation test:
  // 0 -- 1 -- 2 -- 3
  // ------------------------------------------------------------

  // Node 1: immediate neighbors 0 and 2
  net1->AddStaticRouteWithPathloss (/*nwkDst*/ 0,
                                    /*nextHop*/ 0,
                                    /*pathlossDb*/ 107.377,
                                    /*immediate*/ true);

  net1->AddStaticRouteWithPathloss (/*nwkDst*/ 2,
                                    /*nextHop*/ 2,
                                    /*pathlossDb*/ 115.742,
                                    /*immediate*/ true);

  // Node 2: immediate neighbors 1 and 3
  net2->AddStaticRouteWithPathloss (/*nwkDst*/ 1,
                                    /*nextHop*/ 1,
                                    /*pathlossDb*/ 115.742,
                                    /*immediate*/ true);

  net2->AddStaticRouteWithPathloss (/*nwkDst*/ 3,
                                    /*nextHop*/ 3,
                                    /*pathlossDb*/ 111.868,
                                    /*immediate*/ true);

  // Net -> App callbacks
  net0->SetRxFromNetCallback (MakeCallback (&AppRxFromNet));
  net1->SetRxFromNetCallback (MakeCallback (&AppRxFromNet));
  net2->SetRxFromNetCallback (MakeCallback (&AppRxFromNet));
  net3->SetRxFromNetCallback (MakeCallback (&AppRxFromNet));

  // Hop -> Net callbacks
  hop0->SetRxFromHopCallback (MakeCallback (&CsrNetLayer::ReceiveFromHop, net0));
  hop1->SetRxFromHopCallback (MakeCallback (&CsrNetLayer::ReceiveFromHop, net1));
  hop2->SetRxFromHopCallback (MakeCallback (&CsrNetLayer::ReceiveFromHop, net2));
  hop3->SetRxFromHopCallback (MakeCallback (&CsrNetLayer::ReceiveFromHop, net3));

  hop0->SetShouldDackCallback (MakeCallback (&CsrNetLayer::ShouldDack, net0));
  hop1->SetShouldDackCallback (MakeCallback (&CsrNetLayer::ShouldDack, net1));
  hop2->SetShouldDackCallback (MakeCallback (&CsrNetLayer::ShouldDack, net2));
  hop3->SetShouldDackCallback (MakeCallback (&CsrNetLayer::ShouldDack, net3));

  // MAC -> Hop callbacks
  dev0->GetMac ().SetRxCallback (MakeCallback (&CsrHopLayer::ReceiveFromMac, hop0));
  dev1->GetMac ().SetRxCallback (MakeCallback (&CsrHopLayer::ReceiveFromMac, hop1));
  dev2->GetMac ().SetRxCallback (MakeCallback (&CsrHopLayer::ReceiveFromMac, hop2));
  dev3->GetMac ().SetRxCallback (MakeCallback (&CsrHopLayer::ReceiveFromMac, hop3));

  // Slot tick drives reservation decay (OPNET-like)
  dev0->GetMac ().StartSlotTick (Seconds (0.013));
  dev1->GetMac ().StartSlotTick (Seconds (0.013));
  dev2->GetMac ().StartSlotTick (Seconds (0.013));
  dev3->GetMac ().StartSlotTick (Seconds (0.013));

  net0->StartNeighborFreshnessMonitor (Seconds (20.0), Seconds (2.0));
  net1->StartNeighborFreshnessMonitor (Seconds (20.0), Seconds (2.0));
  net2->StartNeighborFreshnessMonitor (Seconds (20.0), Seconds (2.0));
  net3->StartNeighborFreshnessMonitor (Seconds (20.0), Seconds (2.0));

  net0->SetInvalidateRoutesOnStaleNeighbor (true);
  net1->SetInvalidateRoutesOnStaleNeighbor (true);
  net2->SetInvalidateRoutesOnStaleNeighbor (true);
  net3->SetInvalidateRoutesOnStaleNeighbor (true);

  // Discovery assist for direct 0->1 no-route test.
  // Node 0 will trigger on-demand discovery at t=1.
  // Node 1 sends HELLO shortly after so node 0 can learn route dst=1 -> nextHop=1.
  //net1->StartDiscovery (Seconds (1.5), Seconds (5.0));
  
  //net0->StartDiscovery (Seconds (10.0), Seconds (30.0));
  //net1->StartDiscovery (Seconds (10.0), Seconds (30.0));
  //net2->StartDiscovery (Seconds (10.0), Seconds (30.0));
  // Route propagation sequence:
  //
  // 1) Node 0 sends to 3 and gets stuck with no route.
  // 2) Node 2 advertises route to 3.
  // 3) Node 1 learns route to 3 via 2.
  // 4) Node 1 advertises route to 3.
  // 5) Node 0 learns route to 3 via 1 and drains queue.

 // net2->StartDiscovery (Seconds (1.4), Seconds (5.0));
 // net1->StartDiscovery (Seconds (2.8), Seconds (5.0));

  Simulator::Schedule (Seconds (1.4), [net2]() {
  net2->SendRoutingUpdate ();
  });

  Simulator::Schedule (Seconds (2.8), [net1]() {
    net1->SendRoutingUpdate ();
  });

  Simulator::Schedule (Seconds (3.4), [net1]() {
  net1->SendNeighborCheck (
    0,
    CsrNeighborCheckType::Message);
  });

  Simulator::Schedule (Seconds (12.0), [net2]() {
  net2->SendNoPath (1, 3);
  });

  Simulator::Schedule (Seconds (14.0), [net1]() {
  // Synthetic reverse-path removal test:
  // node 2 currently learned source 0 through node 1.
  net1->SendNoPath (2, 0);
  });

  /*Simulator::Schedule (Seconds (37.5), [net1]() {
    net1->SetDiscoveryResponseEnabled (false);
  });*/

  Simulator::Schedule (Seconds (38.0), [net0]() {
    net0->StartDiscovery (Seconds (0.0), Seconds (5.0));
  });

  Simulator::Schedule (Seconds (39.0), [net1]() {
  net1->SendNeighborCheck (
    0,
    CsrNeighborCheckType::Discovery,
    CSR_BROADCAST_ID,
    1);  // deliberately stale
  });

  // Print neighbor tables shortly after discovery ends (~40s)
  Simulator::Schedule (Seconds (40.5),
                      &CsrHopLayer::PrintNeighbors,
                      hop0);
  Simulator::Schedule (Seconds (40.5),
                      &CsrHopLayer::PrintNeighbors,
                      hop1);
  Simulator::Schedule (Seconds (40.5),
                      &CsrHopLayer::PrintNeighbors,
                      hop2);
  Simulator::Schedule (Seconds (40.5),
                      &CsrHopLayer::PrintNeighbors,
                      hop3);

  Simulator::Schedule (Seconds (45.0), [net1]() {
    net1->SetDiscoveryResponseEnabled (true);
  });
  // Traffic pattern similar to your earlier log

  // Burst 1 at t=1 s
  Simulator::Schedule (Seconds (1.0), [net0]() {
    for (int i = 0; i < 2; ++i)
      {
        uint32_t size = 100 + 20 * i;
        Ptr<Packet> payload = Create<Packet> (size);
        net0->Send (/*dst*/ 3,
                    /*dscp*/ 5,
                    payload,
                    /*ack*/ true);
      }
  });

 #if 0
  Simulator::Schedule (Seconds (8.0), [net2]() {
  for (int i = 0; i < 2; ++i)
    {
      uint32_t size = 90 + 20 * i;
      Ptr<Packet> payload = Create<Packet> (size);
      net2->Send (/*dst*/ 0,
                  /*dscp*/ 5,
                  payload,
                  /*ack*/ true);
    }
    });
#endif
    
  /* // Burst 2 at t=12 s
  Simulator::Schedule (Seconds (12.0), [net0]() {
    for (int i = 0; i < 2; ++i)
      {
        uint32_t size = 140 + 20 * i;
        Ptr<Packet> payload = Create<Packet> (size);
        net0->Send ( 2, //dst
                     0, //dscp
                    payload,
                    true); //ack
      }
  });

  // Burst 3 at t=40 s
  Simulator::Schedule (Seconds (40.0), [net0]() {
    Ptr<Packet> payload = Create<Packet> (200);
    net0->Send ( 2, //dst
                7, //dscp
                payload,
                true); //ack
  }); */

  Simulator::Stop (Seconds (60.0));
  Simulator::Run ();
  Simulator::Destroy ();
  CloseRxCsv ();
  if (g_nsdpCsv.is_open ()) g_nsdpCsv.close ();

  return 0;
}
