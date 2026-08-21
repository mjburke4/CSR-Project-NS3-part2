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

constexpr CsrNodeId SOURCE_NODE = 0;
constexpr CsrNodeId RELAY_NODE = 1;
constexpr CsrNodeId DESTINATION_NODE = 2;
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
              CsrNodeId nodeId)
{
  hop->SetNodeId (nodeId);
  hop->SetMac (&device->GetMac ());

  nwk->SetNodeId (nodeId);
  nwk->SetHop (hop);

  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));
}

void
ReceiveAtDestination (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE_NODE,
           "destination received DATA with the wrong NWK source");
  Require (payload->GetSize () == PAYLOAD_SIZE,
           "destination received DATA with the wrong payload size");
  g_receivedPackets++;
}

void
RequireSnapshotComplete (Ptr<CsrNetLayer> nwk,
                         CsrNodeId neighbor,
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

  // Only the gateway was started by the scenario.  These checks prove that
  // the remaining local discoveries came from the legacy sequential SNMP
  // START/DONE lifecycle rather than test-side phase scheduling.
  Require (sourceNwk->GetDiscoveryStartCount () == 1,
           "gateway did not perform exactly one startup discovery");
  Require (relayNwk->GetDiscoveryStartCount () == 1,
           "relay was not started exactly once by SNMP handoff");
  Require (destinationNwk->GetDiscoveryStartCount () == 1,
           "destination was not started exactly once by SNMP handoff");

  Require (sourceNwk->GetDiscoveryBroadcastCount () == 3,
           "gateway did not send the legacy three discovery broadcasts");
  Require (relayNwk->GetDiscoveryBroadcastCount () == 3,
           "relay did not send the legacy three discovery broadcasts");
  Require (destinationNwk->GetDiscoveryBroadcastCount () == 3,
           "destination did not send the legacy three discovery broadcasts");

  Require (sourceNwk->GetSnmpStartSentCount () >= 1 &&
             relayNwk->GetSnmpStartReceivedCount () >= 1,
           "gateway-to-relay SNMP_START_DISCOVERY handoff failed");
  Require (relayNwk->GetSnmpDoneSentCount () >= 1 &&
             sourceNwk->GetSnmpDoneReceivedCount () >= 1,
           "relay-to-gateway SNMP_DISCOVERY_DONE report failed");
  Require (relayNwk->GetSnmpStartSentCount () >= 1 &&
             destinationNwk->GetSnmpStartReceivedCount () >= 1,
           "relay-to-destination SNMP_START_DISCOVERY handoff failed");
  Require (destinationNwk->GetSnmpDoneSentCount () >= 1 &&
             relayNwk->GetSnmpDoneReceivedCount () >= 1,
           "destination-to-relay SNMP_DISCOVERY_DONE report failed");

  // Relay DONE advertises node 2 to the gateway.  OPNET lets the gateway
  // address a START to node 2 through node 1, but br_hop then drops that SNMP
  // payload because its final destination is not the receiving hop.  Node 2
  // must therefore have received only node 1's direct START.
  Require (sourceNwk->GetSnmpStartSentCount () >= 2,
           "gateway did not consume the node list in relay DONE");
  Require (destinationNwk->GetSnmpStartReceivedCount () == 1,
           "legacy one-hop SNMP was incorrectly relayed end-to-end");

  Require (relayNwk->GetDiscoveryInitiatedBy () == SOURCE_NODE,
           "relay did not retain the gateway as its discovery initiator");
  Require (destinationNwk->GetDiscoveryInitiatedBy () == RELAY_NODE,
           "destination did not retain the relay as its discovery initiator");

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

  // No route and no non-gateway discovery phase is installed by the scenario.
  // OPNET starts the gateway after ten seconds; its discovery table and
  // one-hop SNMP START/DONE messages must start the remaining nodes in order.
  nwk0->ScheduleGatewayStartupDiscovery ();

  Simulator::Schedule (Seconds (40.0),
                       &CheckConvergence,
                       nwk0,
                       nwk1,
                       nwk2);

  Simulator::Stop (Seconds (55.0));
  Simulator::Run ();

  CheckFinalState (nwk0, nwk1, hop0, hop1);

  Simulator::Destroy ();
  std::cout << "PASS: gateway-driven discovery/SNMP convergence test"
            << std::endl;
  return 0;
}
