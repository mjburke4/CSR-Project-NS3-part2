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

  // This legacy demo starts from pre-installed routes and intentionally skips
  // the discovery/key-admission phase covered by the dedicated smoke test.
  net0->SetArlNeighborAdmissionEnabled (false);
  net1->SetArlNeighborAdmissionEnabled (false);
  net2->SetArlNeighborAdmissionEnabled (false);
  net3->SetArlNeighborAdmissionEnabled (false);

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
  net3->SetHop (hop3);

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

  net0->SetNodeType (CsrNodeType::Gateway);
  net1->SetNodeType (CsrNodeType::Routable);
  net2->SetNodeType (CsrNodeType::Routable);
  net3->ConfigureAsLeaf ();

  // Discovery assist for direct 0->1 no-route test.
  // Node 0 will trigger on-demand discovery at t=1.
  // Node 1 sends HELLO shortly after so node 0 can learn route dst=1 -> nextHop=1.
  //net1->StartDiscovery (Seconds (1.5), Seconds (5.0));

  //net0->StartDiscovery (Seconds (10.0), Seconds (30.0));
  //net1->StartDiscovery (Seconds (10.0), Seconds (30.0));
  //net2->StartDiscovery (Seconds (10.0), Seconds (30.0));
  // Node 3 is configured as an Ordinary leaf below.  Legacy OPNET routing
  // does not advertise Ordinary destinations as multi-hop routes.  Traffic
  // from node 0 to node 3 therefore remains queued until a usable reverse
  // path exists; this is intentional parity behavior, not a HOP stall.

  // Use a short delay/window for the regression test.
  // Production defaults remain the legacy 10 s / 30 s.
  net0->ScheduleGatewayStartupDiscovery (
    Seconds (0.25),
    Seconds (5.0));

  // Confirm that non-Gateway roles cannot schedule automatic
  // startup discovery.
  net1->ScheduleGatewayStartupDiscovery (
    Seconds (0.25),
    Seconds (5.0));

  net3->ScheduleGatewayStartupDiscovery (
    Seconds (0.25),
    Seconds (5.0));

  net1->SetTemperatureLimitsCx10 (
    -123,
    456);

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

  Simulator::Schedule (Seconds (7.0), [net2]() {
    // Produces routing sequence 2.
    net2->SendRoutingUpdate ();
  });

  Simulator::Schedule (Seconds (8.5), [net2]() {
    // Replay the old sequence. Receivers should refresh their direct
    // link to node 2 but reject its stale route vector.
    net2->SendRoutingUpdateWithSequenceForTest (1);
  });

  Simulator::Schedule (
    Seconds (9.75),
    [net1]() {
      net1->SendReliableRoutingUpdate (2);
  });

  Simulator::Schedule (Seconds (10.0), [net3]() {
  net3->SendRoutingUpdate ();
  });

  Simulator::Schedule (Seconds (12.0), [net2]() {
  net2->SendNoPath (1, 3);
  });

  Simulator::Schedule (Seconds (14.0), [net1]() {
  // Synthetic reverse-path removal test:
  // node 2 currently learned source 0 through node 1.
  net1->SendNoPath (2, 0);
  });

  Simulator::Schedule (Seconds (16.0), [hop2]() {
  Ptr<Packet> payload = Create<Packet> (40);

  // This packet reaches node 3 at HOP, but its network
  // destination is node 0. Node 3 is configured as a leaf.
  CsrNetHeader nh (2, 0, 5);
  payload->AddHeader (nh);

  hop2->SendData (3, 5, payload, true);
  });

  Simulator::Schedule (Seconds (19.5), [net2]() {
    std::cout
      << "\n=== routesClearTable parity test on node 2 ==="
      << std::endl;

    net2->ClearRoutes ();
    });

  Simulator::Schedule (Seconds (19.55), [net2]() {
    net2->DumpRoutes ();
  });

  Simulator::Schedule (
    Seconds (22.5),
    [net2]() {
      std::cout
        << "\n=== reliable RoutingRequest test ==="
        << std::endl;

      net2->SendRoutingRequest (1);
    });

  Simulator::Schedule (Seconds (6.0), [net0]() {
  // Node 1 is fresh at node 0, so it should appear in the Chirp.
  net0->SendDiscoveryChirp ();
  });

  Simulator::Schedule (Seconds (18.0), [net0]() {
    // Keep node 0 fresh in node 1's table without refreshing
    // node 1 in node 0's table. When node 1 later becomes stale
    // at node 0, the automatic Chirp should omit node 1.
    net0->SendRoutingUpdate ();
  });

  Simulator::Schedule (
    Seconds (22.25),
    [net2]() {
      std::cout
        << "\n=== snapshot stale-route setup ==="
        << std::endl;

      // Pretend node 2 previously learned destination 99
      // from node 1. Node 1 will not include destination
      // 99 in its requested snapshot, so FLUSH must
      // invalidate it.
      net2->AddOrUpdateRoute (
        99,       // destination
        1,        // next hop
        false,    // not immediate
        2,        // hops
        115.742,  // test path loss
        64,       // link cost to node 1
        25,       // advertised downstream cost
        1,        // learned from node 1
        1);       // capability
    });

    Simulator::Schedule (
      Seconds (28.0),
      [net2]() {
        std::cout
          << "\n=== post-snapshot route table ==="
          << std::endl;

        net2->DumpRoutes ();
      });

  Simulator::Schedule (
  Seconds (31.0),
  [net2]() {
    std::cout
      << "\n=== targeted RoutingDelete setup ==="
      << std::endl;

    // Simulate an active route that node 2
    // previously learned from node 1.
    net2->AddOrUpdateRoute (
      98,       // destination
      1,        // next hop
      false,    // not immediate
      2,        // hops
      115.742,  // test path loss
      64,       // link cost
      20,       // advertised cost
      1,        // learned from node 1
      1);       // capability
  });

Simulator::Schedule (
  Seconds (31.25),
  [net1]() {
    net1->SendReliableRoutingDelete (
      2,   // neighbor
      98); // deleted destination
  });

  Simulator::Schedule (
  Seconds (33.0),
  [net1, net2]() {
    std::cout
      << "\n=== route path loop-rejection setup ==="
      << std::endl;

    // Pretend node 1 previously accepted destination 96
    // from node 2. The new snapshot advertises 96 with a
    // loop, so it must be rejected and then flushed.
    net1->AddOrUpdateRoute (
      96,
      2,
      false,
      3,
      115.742,
      64,
      25,
      2,
      1,
      std::vector<CsrNodeId> {
        2, 96
      });


    // First candidate becomes selected.
    net1->AddOrUpdateRoute (
      250,
      2,
      false,
      3,
      115.0,
      50,
      50,
      2,
      1,
      std::vector<CsrNodeId>{2, 250});

    net1->DumpBestRoute (250);

    // Exact tie through a LOWER node ID.
    // Legacy behavior must stay on nextHop 2.
    net1->AddOrUpdateRoute (
      250,
      0,
      false,
      3,
      107.0,
      50,
      50,
      0,
      1,
      std::vector<CsrNodeId>{0, 250});

    net1->DumpBestRoute (250);
    // Valid route: node 2 reaches 95
    // through node 3.
    net2->AddOrUpdateRoute (
      95,
      3,
      false,
      2,
      111.868,
      51,
      20,
      3,
      1,
      std::vector<CsrNodeId> {
        3, 95
      });

    // Deliberately malformed loop:
    // node 1 appears in the path that
    // will be advertised to node 1.
    net2->AddOrUpdateRoute (
      96,
      3,
      false,
      3,
      111.868,
      51,
      25,
      3,
      1,
      std::vector<CsrNodeId> {
        3, 1, 96
      });

    net2->StartReliableRoutingSnapshot (
      1);
  });

  Simulator::Schedule (
    Seconds (34.75),
    [net1]() {
      std::cout
        << "\n=== enable automatic failover propagation ==="
        << std::endl;

      net1->SetAutomaticRoutePropagationEnabled (
        true);
    });

  Simulator::Schedule (
    Seconds (35.0),
    [net1]() {
      std::cout
        << "\n=== genuine backup failover setup ==="
        << std::endl;

      // Node 1 should already have a genuine route to
      // node 3 through node 2.
      std::cout
        << "--- existing genuine backup ---"
        << std::endl;

      net1->DumpBestRoute (3);

      // Install a temporary lower-cost primary through
      // node 0. The candidate is identified as having
      // been learned from node 0 so a Delete from node 0
      // can withdraw only this candidate.
      //
      // total cost = 39 + 1 = 40, which should beat
      // the genuine multi-hop route through node 2.
      net1->AddOrUpdateRoute (
        3,          // destination
        0,          // next hop
        false,      // not immediate
        2,          // hop count
        107.377,    // path loss to node 0
        39,         // link cost to node 0
        1,          // advertised downstream cost
        0,          // learned from node 0
        1);         // capability

      std::cout
        << "--- temporary primary installed ---"
        << std::endl;

      net1->DumpBestRoute (3);
      net1->DumpRoutes ();
    });

  Simulator::Schedule (
  Seconds (35.25),
  [net0]() {
    std::cout
      << "\n=== withdraw primary route candidate ==="
      << std::endl;

    net0->SendReliableRoutingDelete (
      1,
      97);
  });

  Simulator::Schedule (
    Seconds (36.0),
    [net0]() {
      std::cout
        << "\n=== withdraw temporary primary to destination 3 ==="
        << std::endl;

      net0->SendReliableRoutingDelete (
        1,  // receiving neighbor
        3); // destination
    });

  Simulator::Schedule (
  Seconds (37.25),
  [net0, net1, net2]() {
    std::cout
      << "\n=== genuine backup failover result ==="
      << std::endl;

    std::cout
      << "--- node 1 selected route ---"
      << std::endl;

    net1->DumpBestRoute (3);
    net1->DumpRoutes ();

    std::cout
      << "--- node 0 learned fallback advertisement ---"
      << std::endl;

    net0->DumpBestRoute (3);

    std::cout
      << "--- node 2 retains direct route ---"
      << std::endl;

    net2->DumpBestRoute (3);
  });

  // Sequence-aware discovery verification regression test.
  Simulator::Schedule (Seconds (37.5), [net1]() {
    net1->SetDiscoveryResponseEnabled (false);
  });

  Simulator::Schedule (Seconds (38.0), [net0]() {
    net0->StartDiscovery (Seconds (0.0), Seconds (5.0));
  });

  Simulator::Schedule (
    Seconds (39.5),
    [net0, net1, net2]() {
      std::cout
        << "\n=== genuine backup failover result ==="
        << std::endl;

      std::cout
        << "--- node 1 selected route ---"
        << std::endl;

      net1->DumpBestRoute (3);

      std::cout
        << "--- node 0 learned fallback advertisement ---"
        << std::endl;

      net0->DumpBestRoute (3);

      std::cout
        << "--- node 2 retains direct route ---"
        << std::endl;

      net2->DumpBestRoute (3);
    });

  Simulator::Schedule (
  Seconds (42.0),
  [net1]() {
    std::cout
      << "\n=== confirmed alternate route failover ==="
      << std::endl;

    net1->DumpBestRoute (97);
  });

  Simulator::Schedule (Seconds (42.5), [net0]() {
    net0->SendNeighborCheck (
      1,
      CsrNeighborCheckType::Verify,
      CSR_BROADCAST_ID,
      1);  // stale response from the previous cycle
  });

  Simulator::Schedule (
    Seconds (43.0),
    [net0]() {
      std::cout
        << "\n=== unknown-route DELETE tombstone test ==="
        << std::endl;

      net0->SendReliableRoutingDelete (
        1,
        251);
    });

    Simulator::Schedule (
      Seconds (43.0),
      [net0]() {
        std::cout
          << "\n=== unknown-route DELETE tombstone test ==="
          << std::endl;

        net0->SendReliableRoutingDelete (
          1,
          251);
      });

  Simulator::Schedule (Seconds (45.0), [net1]() {
    net1->SetDiscoveryResponseEnabled (true);
  });

  Simulator::Schedule (
    Seconds (45.0),
    [net1]() {
      std::cout
        << "\n=== ordinary capability advertisement test ==="
        << std::endl;

      net1->AddOrUpdateRoute (
        252,
        0,
        false,
        2,
        107.377,
        39,
        20,
        0,
        0,  // capability = ORDINARY
        std::vector<CsrNodeId> {
          0, 252
        });

      net1->DumpBestRoute (252);

      net1->SendReliableRoutingUpdate (
        2);
    });

  Simulator::Schedule (
  Seconds (46.0),
  [net1]() {
    net1->SetRoutingSnapshotResponseEnabled (
      false);
  });

Simulator::Schedule (
  Seconds (46.25),
  [net2]() {
    std::cout
      << "\n=== RoutingRequest timeout/retry test ==="
      << std::endl;

    net2->SendRoutingRequest (1);
  });

Simulator::Schedule (
  Seconds (47.0),
  [net2]() {
    std::cout
      << "\n=== ordinary capability result ==="
      << std::endl;

    net2->DumpBestRoute (252);
  });

  Simulator::Schedule (
    Seconds (47.5),
    [net1]() {
      std::cout
        << "\n=== routable capability advertisement test ==="
        << std::endl;

      net1->AddOrUpdateRoute (
        252,
        0,
        false,
        2,
        107.377,
        39,
        20,
        0,
        1,  // capability = ROUTABLE
        std::vector<CsrNodeId> {
          0, 252
        });

      net1->SendReliableRoutingUpdate (
        2);
    });

Simulator::Schedule (
  Seconds (49.5),
  [net2]() {
    net2->DumpBestRoute (252);
  });

Simulator::Schedule (
  Seconds (50.0),
  [net3]() {
    std::cout
      << "\n=== reverse-route preference setup ==="
      << std::endl;

    net3->AddOrUpdateRoute (
      0,
      2,
      false,
      3,
      111.868,
      51,
      100,
      2,
      2,
      std::vector<CsrNodeId> {
        2, 1, 0
      });

    Ptr<Packet> payload =
      Create<Packet> (32);

    net3->Send (
      0,
      0,
      payload,
      true);
  });

Simulator::Schedule (
  Seconds (52.0),
  [net1]() {
    net1->AddOrUpdateRoute (
      3,
      0,
      false,
      2,
      107.377,
      39,
      20,
      0,
      0,   // Ordinary / non-capable
      std::vector<CsrNodeId> {
        0, 3
      });

    net1->DumpBestRoute (3);

    Ptr<Packet> payload =
      Create<Packet> (32);

    net1->Send (
      3,
      0,
      payload,
      true);
  });

Simulator::Schedule (
  Seconds (53.5),
  [net1]() {
    // The initial request was ACKed but received
    // no snapshot. Permit the retry to succeed.
    net1->SetRoutingSnapshotResponseEnabled (
      true);
  });

  Simulator::Schedule (
    Seconds (60.0),
    [net1]() {
      std::cout
        << "\n=== multi-section routing snapshot setup ==="
        << std::endl;

      net1->
        SetAutomaticRoutePropagationEnabled (
          false);

      for (CsrNodeId destination = 200;
          destination < 212;
          ++destination)
        {
          net1->AddOrUpdateRoute (
            destination,
            0,
            false,
            2,
            107.377,
            39,
            destination - 190,
            0,
            1,
            std::vector<CsrNodeId> {
              0,
              destination
            });
        }

      net1->StartReliableRoutingSnapshot (
        2);
    });

  Simulator::Schedule (
    Seconds (80.0),
    [net2]() {
      std::cout
        << "\n=== multi-section routing snapshot result ==="
        << std::endl;

      net2->DumpBestRoute (200);
      net2->DumpBestRoute (211);
    });


  // Traffic pattern similar to your earlier log

  // Ordinary-destination routing regression: four packets make
  // NSDP.count == 4, but node 3 is not advertised beyond its direct neighbor.
  // The deterministic NWK/HOP burst-and-drain check lives in
  // csr-nwk-hop-integration-smoke.cc with an already-converged Routable path.
  Simulator::Schedule (Seconds (1.0), [net0]() {
    for (int i = 0; i < 4; ++i)
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
    

  Simulator::Stop (Seconds (90.0));
  Simulator::Run ();
  Simulator::Destroy ();
  CloseRxCsv ();
  if (g_nsdpCsv.is_open ()) g_nsdpCsv.close ();

  return 0;
}
