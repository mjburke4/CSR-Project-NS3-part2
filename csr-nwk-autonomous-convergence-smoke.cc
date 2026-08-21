#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"

#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

constexpr uint16_t SOURCE_NODE = 0;
constexpr uint16_t RELAY_NODE = 1;
constexpr uint16_t DESTINATION_NODE = 2;
constexpr uint32_t PAYLOAD_SIZE = 192;

bool g_forwardRouteAvailable = false;
bool g_reverseRouteAvailable = false;
uint32_t g_receivedPackets = 0;

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
ConnectStack (Ptr<CsrNetDevice> device,
              Ptr<CsrHopLayer> hop,
              Ptr<CsrNetLayer> nwk,
              uint16_t nodeId)
{
  hop->SetNodeId (nodeId);
  hop->SetMac (&device->GetMac ());

  nwk->SetNodeId (nodeId);
  nwk->SetHop (hop);

  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));
}

void
ReceiveAtDestination (Ptr<Packet> payload, uint16_t source)
{
  Require (source == SOURCE_NODE,
           "destination received DATA with the wrong NWK source");
  Require (payload->GetSize () == PAYLOAD_SIZE,
           "destination received DATA with the wrong payload size");
  g_receivedPackets++;
}

void
RequireSnapshotComplete (Ptr<CsrNetLayer> nwk,
                         uint16_t neighbor,
                         const char* message)
{
  uint8_t totalSections =
    nwk->GetOutboundRoutingSnapshotTotalSections (neighbor);

  Require (totalSections > 0 &&
             nwk->GetOutboundRoutingSnapshotAckedSections (neighbor) ==
               totalSections &&
             !nwk->IsOutboundRoutingSnapshotActive (neighbor),
           message);
}

void
CheckConvergence (Ptr<CsrNetLayer> sourceNwk,
                  Ptr<CsrNetLayer> relayNwk,
                  Ptr<CsrNetLayer> destinationNwk)
{
  uint32_t cost = 0;
  CsrHelloHeader::RoutingInfo routingInfo;

  g_forwardRouteAvailable =
    sourceNwk->GetSelectedRouteCost (DESTINATION_NODE, cost);
  Require (g_forwardRouteAvailable,
           "source did not learn the two-hop route to destination");

  g_reverseRouteAvailable =
    destinationNwk->GetSelectedRouteCost (SOURCE_NODE, cost);
  Require (g_reverseRouteAvailable,
           "destination did not learn the two-hop route to source");

  Require (relayNwk->GetSelectedRouteCost (SOURCE_NODE, cost),
           "relay did not learn its direct route to source");
  Require (relayNwk->GetSelectedRouteCost (DESTINATION_NODE, cost),
           "relay did not learn its direct route to destination");

  // A selected route alone is not sufficient evidence that the new ARL
  // byte-stream path ran.  Each adjacent pair must have exchanged and
  // atomically applied a complete INFO + UPDATE + FLUSH snapshot.
  Require (sourceNwk->GetNeighborRoutingInfo (RELAY_NODE, routingInfo),
           "source did not apply relay ARL INFO");
  Require (relayNwk->GetNeighborRoutingInfo (SOURCE_NODE, routingInfo),
           "relay did not apply source ARL INFO");
  Require (relayNwk->GetNeighborRoutingInfo (DESTINATION_NODE, routingInfo),
           "relay did not apply destination ARL INFO");
  Require (destinationNwk->GetNeighborRoutingInfo (RELAY_NODE, routingInfo),
           "destination did not apply relay ARL INFO");

  Require (sourceNwk->GetPendingArlRoutingMessageCount (RELAY_NODE) == 0,
           "source retained an incomplete ARL message from relay");
  Require (relayNwk->GetPendingArlRoutingMessageCount (SOURCE_NODE) == 0,
           "relay retained an incomplete ARL message from source");
  Require (relayNwk->GetPendingArlRoutingMessageCount (DESTINATION_NODE) == 0,
           "relay retained an incomplete ARL message from destination");
  Require (destinationNwk->GetPendingArlRoutingMessageCount (RELAY_NODE) == 0,
           "destination retained an incomplete ARL message from relay");

  RequireSnapshotComplete (
    sourceNwk,
    RELAY_NODE,
    "source ARL snapshot did not complete independent HOP ACKs");
  RequireSnapshotComplete (
    destinationNwk,
    RELAY_NODE,
    "destination ARL snapshot did not complete independent HOP ACKs");

  sourceNwk->Send (DESTINATION_NODE,
                   5,
                   Create<Packet> (PAYLOAD_SIZE),
                   true);
}

void
CheckFinalState (Ptr<CsrNetLayer> sourceNwk,
                 Ptr<CsrNetLayer> relayNwk,
                 Ptr<CsrHopLayer> sourceHop,
                 Ptr<CsrHopLayer> relayHop)
{
  Require (g_forwardRouteAvailable && g_reverseRouteAvailable,
           "convergence checkpoint did not complete");
  Require (g_receivedPackets == 1,
           "learned route did not deliver exactly one DATA packet");

  Require (sourceNwk->GetNwkQueueSize () == 0,
           "source NWK queue did not drain after convergence");
  Require (relayNwk->GetNwkQueueSize () == 0,
           "relay NWK queue did not drain after convergence");
  Require (sourceNwk->GetNsdpCount (SOURCE_NODE, DESTINATION_NODE) == 0,
           "source NSDP count did not return to zero");
  Require (relayNwk->GetNsdpCount (SOURCE_NODE, DESTINATION_NODE) == 0,
           "relay NSDP count did not return to zero");
  Require (sourceHop->GetPendingDataCount () == 0,
           "source HOP pending DATA count did not return to zero");
  Require (relayHop->GetPendingDataCount () == 0,
           "relay HOP pending DATA count did not return to zero");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  Ptr<CsrNetDevice> device0 =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrNetDevice> device1 =
    CreateObject<CsrNetDevice> (RELAY_NODE);
  Ptr<CsrNetDevice> device2 =
    CreateObject<CsrNetDevice> (DESTINATION_NODE);

  // The physical graph is a line.  Nodes 0 and 2 cannot hear one another.
  device0->AddPeer (device1);
  device1->AddPeer (device0);
  device1->AddPeer (device2);
  device2->AddPeer (device1);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };

  device0->GetPhy ().SetPerModel (noErrors);
  device1->GetPhy ().SetPerModel (noErrors);
  device2->GetPhy ().SetPerModel (noErrors);

  device0->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, RELAY_NODE, 1.0);
  device1->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, RELAY_NODE, 1.0);
  device1->GetPhy ().SetLinkDistanceMeters (
    RELAY_NODE, DESTINATION_NODE, 1.0);
  device2->GetPhy ().SetLinkDistanceMeters (
    RELAY_NODE, DESTINATION_NODE, 1.0);

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

  nwk2->SetRxFromNetCallback (
    MakeCallback (&ReceiveAtDestination));

  // No route is installed by the scenario.  Each local discovery is the
  // equivalent of an OPNET SNMP_START_DISCOVERY command reaching that node.
  nwk0->StartDiscovery (Seconds (0.0), Seconds (5.0));
  nwk1->StartDiscovery (Seconds (0.0), Seconds (5.0));
  nwk2->StartDiscovery (Seconds (0.0), Seconds (5.0));

  Simulator::Schedule (Seconds (30.0),
                       &CheckConvergence,
                       nwk0,
                       nwk1,
                       nwk2);

  Simulator::Stop (Seconds (50.0));
  Simulator::Run ();

  CheckFinalState (nwk0, nwk1, hop0, hop1);

  Simulator::Destroy ();
  std::cout << "PASS: autonomous ARL routing convergence test"
            << std::endl;
  return 0;
}
