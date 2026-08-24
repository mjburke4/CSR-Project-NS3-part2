#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId SOURCE_NODE = 0;
constexpr CsrNodeId RELAY_NODE = 1;
constexpr CsrNodeId DESTINATION_NODE = 2;
constexpr uint32_t BURST_SIZE = 4;

std::vector<uint32_t> g_receivedSizes;

void
Require (bool condition, const char* message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

void
ReceiveAtDestination (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE_NODE,
           "destination received a payload with the wrong NWK source");
  g_receivedSizes.push_back (payload->GetSize ());
}

void
ConnectStack (Ptr<CsrNetDevice> device,
              Ptr<CsrHopLayer> hop,
              Ptr<CsrNetLayer> nwk,
              CsrNodeId nodeId)
{
  hop->SetNodeId (nodeId);
  hop->SetMac (&device->GetMac ());
  nwk->SetNodeId (nodeId);
  // This regression isolates NWK/HOP custody with pre-installed routes.
  nwk->SetArlNeighborAdmissionEnabled (false);
  nwk->SetHop (hop);

  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));
}

void
CheckInitialAdmission (Ptr<CsrNetLayer> sourceNwk,
                       Ptr<CsrHopLayer> sourceHop)
{
  // OPNET increments NSDP when APP traffic enters NWK, but NSDP does not
  // prevent the first packet from entering HOP.  The legacy per-neighbor
  // threshold starts at zero, which means an effective window of one.
  Require (sourceNwk->GetNsdpCount (SOURCE_NODE, DESTINATION_NODE) ==
             BURST_SIZE,
           "source NSDP did not count the complete application burst");
  Require (sourceNwk->GetNwkQueueSize () == BURST_SIZE - 1,
           "NWK did not admit exactly one packet into the initial HOP window");
  Require (sourceHop->GetPendingDataCount () == 1,
           "source HOP pending DATA count is not one after initial admission");
  Require (sourceHop->GetOutstandingDataCount (RELAY_NODE) == 1,
           "source next-hop flow-control counter is not one");
}

void
CheckRelayAdmission (Ptr<CsrNetLayer> relayNwk,
                     Ptr<CsrHopLayer> relayHop)
{
  // With OPNET MAC holdoff and serialized airtime, the relay has admitted the
  // first packet while the source's initial one-packet HOP window waits for
  // the relay ACK.  The old immediate-MAC model unrealistically placed all
  // four packets here before this checkpoint.
  Require (relayNwk->GetNsdpCount (SOURCE_NODE, DESTINATION_NODE) ==
             1,
           "relay NSDP did not count the first forwarded packet");
  Require (relayNwk->GetNwkQueueSize () == 0,
           "relay did not admit its first packet into HOP");
  Require (relayHop->GetPendingDataCount () == 1,
           "relay HOP pending DATA count is not one before its first ACK");
  Require (relayHop->GetOutstandingDataCount (DESTINATION_NODE) == 1,
           "relay next-hop flow-control counter is not one before its first ACK");
}

void
CheckFinalState (Ptr<CsrNetLayer> sourceNwk,
                 Ptr<CsrNetLayer> relayNwk,
                 Ptr<CsrHopLayer> sourceHop,
                 Ptr<CsrHopLayer> relayHop)
{
  Require (g_receivedSizes.size () == BURST_SIZE,
           "destination did not receive the four-packet burst");

  const std::vector<uint32_t> expectedSizes {160, 140, 120, 100};
  Require (g_receivedSizes == expectedSizes,
           "positive-DSCP packets did not preserve OPNET NWK head insertion");

  Require (sourceNwk->GetNwkQueueSize () == 0,
           "source NWK queue did not drain");
  Require (relayNwk->GetNwkQueueSize () == 0,
           "relay NWK queue did not drain");

  Require (sourceNwk->GetNsdpCount (SOURCE_NODE, DESTINATION_NODE) == 0,
           "source NSDP count did not return to zero");
  Require (relayNwk->GetNsdpCount (SOURCE_NODE, DESTINATION_NODE) == 0,
           "relay NSDP count did not return to zero");

  Require (sourceHop->GetPendingDataCount () == 0,
           "source HOP pending DATA count did not return to zero");
  Require (relayHop->GetPendingDataCount () == 0,
           "relay HOP pending DATA count did not return to zero");
  Require (sourceHop->GetOutstandingDataCount (RELAY_NODE) == 0,
           "source next-hop flow-control counter did not return to zero");
  Require (relayHop->GetOutstandingDataCount (DESTINATION_NODE) == 0,
           "relay next-hop flow-control counter did not return to zero");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  Ptr<CsrNetDevice> device0 = CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrNetDevice> device1 = CreateObject<CsrNetDevice> (RELAY_NODE);
  Ptr<CsrNetDevice> device2 = CreateObject<CsrNetDevice> (DESTINATION_NODE);

  device0->AddPeer (device1);
  device1->AddPeer (device0);
  device1->AddPeer (device2);
  device2->AddPeer (device1);

  // Eliminate random PHY loss from this protocol integration test.  OPNET
  // loss/retry behavior remains covered separately; this scenario isolates
  // NWK admission, relay bookkeeping, ACK release, and HOP queue wakeup.
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };

  device0->GetPhy ().SetPerModel (noErrors);
  device1->GetPhy ().SetPerModel (noErrors);
  device2->GetPhy ().SetPerModel (noErrors);

  device0->GetPhy ().SetLinkDistanceMeters (SOURCE_NODE, RELAY_NODE, 1.0);
  device1->GetPhy ().SetLinkDistanceMeters (SOURCE_NODE, RELAY_NODE, 1.0);
  device1->GetPhy ().SetLinkDistanceMeters (RELAY_NODE, DESTINATION_NODE, 1.0);
  device2->GetPhy ().SetLinkDistanceMeters (RELAY_NODE, DESTINATION_NODE, 1.0);

  Ptr<CsrHopLayer> hop0 = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> hop1 = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> hop2 = CreateObject<CsrHopLayer> ();

  Ptr<CsrNetLayer> nwk0 = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> nwk1 = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> nwk2 = CreateObject<CsrNetLayer> ();

  ConnectStack (device0, hop0, nwk0, SOURCE_NODE);
  ConnectStack (device1, hop1, nwk1, RELAY_NODE);
  ConnectStack (device2, hop2, nwk2, DESTINATION_NODE);

  nwk0->SetNodeType (CsrNodeType::Gateway);
  nwk1->SetNodeType (CsrNodeType::Routable);
  nwk2->SetNodeType (CsrNodeType::Routable);

  const uint8_t routableCapability =
    static_cast<uint8_t> (CsrNodeType::Routable);

  // Install the already-converged 0 -> 1 -> 2 route.  Route discovery is not
  // part of this regression; OPNET's NWK/HOP behavior begins with a valid
  // selected route and a next-hop neighbor for each forwarding node.
  nwk0->AddStaticRouteWithPathloss (
    DESTINATION_NODE,
    RELAY_NODE,
    70.0,
    false,
    routableCapability);
  nwk1->AddStaticRouteWithPathloss (
    DESTINATION_NODE,
    DESTINATION_NODE,
    70.0,
    true,
    routableCapability);

  nwk2->SetRxFromNetCallback (MakeCallback (&ReceiveAtDestination));

  Simulator::ScheduleNow ([nwk0] () {
    for (uint32_t index = 0; index < BURST_SIZE; ++index)
      {
        nwk0->Send (DESTINATION_NODE,
                    5,
                    Create<Packet> (100 + 20 * index),
                    true);
      }
  });

  // OPNET's one-TIC queue check executes before this one-microsecond checkpoint.
  Simulator::Schedule (MicroSeconds (1),
                       &CheckInitialAdmission,
                       nwk0,
                       hop0);

  Simulator::Schedule (Seconds (1.7),
                       &CheckRelayAdmission,
                       nwk1,
                       hop1);

  Simulator::Stop (Seconds (20.0));
  Simulator::Run ();

  CheckFinalState (nwk0, nwk1, hop0, hop1);

  Simulator::Destroy ();
  std::cout << "PASS: NWK/HOP integration parity test" << std::endl;
  return 0;
}
