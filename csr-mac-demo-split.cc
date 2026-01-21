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

  dev0->EnableDutyCycling (true);
  dev1->EnableDutyCycling (true);
  dev2->EnableDutyCycling (true);

  // Line topology: 0 <-> 1 <-> 2
  dev0->AddPeer (dev1);   // 0 can talk to 1
  dev1->AddPeer (dev0);   // 1 can talk to 0
  dev1->AddPeer (dev2);   // 1 can talk to 2
  dev2->AddPeer (dev1);   // 2 can talk to 1

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

  // --- simple fixed link distances for line topology ---
  auto setAllLinkDistances = [&](Ptr<CsrNetDevice> d)
  {
    d->GetPhy().SetLinkDistanceMeters(0, 1, 50.0);
    d->GetPhy().SetLinkDistanceMeters(1, 2, 120.0);
    d->GetPhy().SetDefaultDistanceMeters(1.0); // only used if missing entry
  };

  setAllLinkDistances(dev0);
  setAllLinkDistances(dev1);
  setAllLinkDistances(dev2);

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

  // Create Hop layers
  Ptr<CsrHopLayer> hop0 = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> hop1 = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> hop2 = CreateObject<CsrHopLayer> ();

  hop0->SetNodeId (0);
  hop1->SetNodeId (1);
  hop2->SetNodeId (2);

  hop0->SetMac (&dev0->GetMac ());
  hop1->SetMac (&dev1->GetMac ());
  hop2->SetMac (&dev2->GetMac ());

  // Create Net layers
  Ptr<CsrNetLayer> net0 = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> net1 = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> net2 = CreateObject<CsrNetLayer> ();

  net0->SetNodeId (0);
  net1->SetNodeId (1);
  net2->SetNodeId (2);

  // Periodic NSDP sampler
  std::function<void()> sampleNsdp;
  sampleNsdp = [&]() {
    if (g_nsdpCsv.is_open ())
      {
        net0->DumpNsdp (g_nsdpCsv);
        net1->DumpNsdp (g_nsdpCsv);
        net2->DumpNsdp (g_nsdpCsv);
      }
    Simulator::Schedule (Seconds (1.0), sampleNsdp);
  };
  Simulator::Schedule (Seconds (1.0), sampleNsdp);

  net0->SetHop (hop0);
  net1->SetHop (hop1);
  net2->SetHop (hop2);

  // Static routes for 3-node line: 0 -> 1 -> 2
  // Node 0: can reach 1 directly, and 2 via 1
  net0->AddStaticRoute (/*nwkDst*/ 1, /*nextHop*/ 1);
  net0->AddStaticRoute (/*nwkDst*/ 2, /*nextHop*/ 1);

  // Node 1: can reach 0 directly, and 2 directly
  net1->AddStaticRoute (/*nwkDst*/ 0, /*nextHop*/ 0);
  net1->AddStaticRoute (/*nwkDst*/ 2, /*nextHop*/ 2);

  // Node 2: can reach 0 via 1, and 1 directly
  net2->AddStaticRoute (/*nwkDst*/ 0, /*nextHop*/ 1);
  net2->AddStaticRoute (/*nwkDst*/ 1, /*nextHop*/ 1);

  // Net -> App callbacks
  net0->SetRxFromNetCallback (MakeCallback (&AppRxFromNet));
  net1->SetRxFromNetCallback (MakeCallback (&AppRxFromNet));
  net2->SetRxFromNetCallback (MakeCallback (&AppRxFromNet));

  // Hop -> Net callbacks
  hop0->SetRxFromHopCallback (MakeCallback (&CsrNetLayer::ReceiveFromHop, net0));
  hop1->SetRxFromHopCallback (MakeCallback (&CsrNetLayer::ReceiveFromHop, net1));
  hop2->SetRxFromHopCallback (MakeCallback (&CsrNetLayer::ReceiveFromHop, net2));

  hop0->SetShouldDackCallback (MakeCallback (&CsrNetLayer::ShouldDack, net0));
  hop1->SetShouldDackCallback (MakeCallback (&CsrNetLayer::ShouldDack, net1));
  hop2->SetShouldDackCallback (MakeCallback (&CsrNetLayer::ShouldDack, net2));

  // MAC -> Hop callbacks
  dev0->GetMac ().SetRxCallback (MakeCallback (&CsrHopLayer::ReceiveFromMac, hop0));
  dev1->GetMac ().SetRxCallback (MakeCallback (&CsrHopLayer::ReceiveFromMac, hop1));
  dev2->GetMac ().SetRxCallback (MakeCallback (&CsrHopLayer::ReceiveFromMac, hop2));

  // Slot tick drives reservation decay (OPNET-like)
  dev0->GetMac ().StartSlotTick (Seconds (1.0));
  dev1->GetMac ().StartSlotTick (Seconds (1.0));
  dev2->GetMac ().StartSlotTick (Seconds (1.0));

  net0->StartDiscovery (Seconds (10.0), Seconds (30.0));
  net1->StartDiscovery (Seconds (10.0), Seconds (30.0));
  net2->StartDiscovery (Seconds (10.0), Seconds (30.0));

  // OPNET-style bounded discovery window
  /*dev0->GetMac ().StartDiscovery (Seconds (10.0), Seconds (30.0));
  dev1->GetMac ().StartDiscovery (Seconds (10.0), Seconds (30.0));
  dev2->GetMac ().StartDiscovery (Seconds (10.0), Seconds (30.0));*/

  /*dev0->GetNwk ().StartDiscovery (Seconds (10.0), Seconds (30.0));
  dev1->GetNwk ().StartDiscovery (Seconds (10.0), Seconds (30.0));
  dev2->GetNwk ().StartDiscovery (Seconds (10.0), Seconds (30.0));*/

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


  // Traffic pattern similar to your earlier log

  // Burst 1 at t=1 s
  Simulator::Schedule (Seconds (1.0), [net0]() {
    for (int i = 0; i < 2; ++i)
      {
        uint32_t size = 100 + 20 * i;
        Ptr<Packet> payload = Create<Packet> (size);
        net0->Send (/*dst*/ 2,
                    /*dscp*/ 5,
                    payload,
                    /*ack*/ true);
      }
  });

  // Burst 2 at t=12 s
  Simulator::Schedule (Seconds (12.0), [net0]() {
    for (int i = 0; i < 2; ++i)
      {
        uint32_t size = 140 + 20 * i;
        Ptr<Packet> payload = Create<Packet> (size);
        net0->Send (/*dst*/ 2,
                    /*dscp*/ 0,
                    payload,
                    /*ack*/ true);
      }
  });

  // Burst 3 at t=40 s
  Simulator::Schedule (Seconds (40.0), [net0]() {
    Ptr<Packet> payload = Create<Packet> (200);
    net0->Send (/*dst*/ 2,
                /*dscp*/ 7,
                payload,
                /*ack*/ true);
  });

  Simulator::Stop (Seconds (60.0));
  Simulator::Run ();
  Simulator::Destroy ();
  CloseRxCsv ();
  if (g_nsdpCsv.is_open ()) g_nsdpCsv.close ();

  return 0;
}
