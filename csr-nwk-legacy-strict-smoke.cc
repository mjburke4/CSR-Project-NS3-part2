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

constexpr CsrNodeId SOURCE_NODE = 1;
constexpr CsrNodeId DESTINATION_NODE = 2;
constexpr CsrNodeId ALTERNATE_NODE = 3;

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
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
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
RecordPayload (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE_NODE,
           "strict NWK scenario received the wrong source");
  g_receivedSizes.push_back (payload->GetSize ());
}

void
CheckBeforeTic (Ptr<CsrNetLayer> nwk)
{
  Require (nwk->GetNwkQueueSize () == 4,
           "NWK queue drained at ScheduleNow instead of after one legacy TIC");
}

void
CheckAfterTic (Ptr<CsrNetLayer> nwk,
               Ptr<CsrHopLayer> hop)
{
  Require (nwk->GetNwkQueueSize () == 3,
           "NWK did not admit exactly one packet after the legacy TIC");
  Require (hop->GetPendingDataCount () == 1,
           "caller disabled reliability for legacy NWK DATA");
  Require (hop->GetResendQueueSize () == 1,
           "legacy NWK DATA was not entered in the HOP resend queue");
}

Ptr<Packet>
MakeHello (CsrNodeId nodeId, CsrNodeType nodeType)
{
  CsrHelloHeader hello;
  hello.SetNodeId (nodeId);
  hello.SetHelloSeq (1);
  hello.SetNodeType (nodeType);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::None);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  return packet;
}

Ptr<Packet>
MakeRoutingInfo (CsrNodeId nodeId)
{
  CsrHelloHeader info;
  info.SetNodeId (nodeId);
  info.SetHelloSeq (2);
  info.SetNodeType (CsrNodeType::Routable);
  info.SetSpeedKey (8);
  info.SetRxPowerDbmX10 (-1050);
  info.SetActiveNodes (1);
  info.SetArlRouteMsgType (CsrArlRouteMsgType::RoutingUpdate);
  info.SetRoutingSequence (1);
  info.SetRoutingSection (0);
  info.SetRoutingTotalSections (1);
  info.SetRoutingOperation (CsrRoutingOperation::Info);

  CsrHelloHeader::RoutingInfo limits;
  limits.minSpeedKbps = 8;
  limits.maxSpeedKbps = 8;
  limits.minPowerDbmX10 = -200;
  limits.maxPowerDbmX10 = -200;
  limits.linkMarginDbX10 = 120;
  limits.lowPowerDbmX10 = -200;
  limits.tempLowCx10 = -450;
  limits.tempHighCx10 = 600;
  info.SetRoutingInfo (limits);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (info);
  return packet;
}

void
RunQueueOrderingAndTicScenario ()
{
  g_receivedSizes.clear ();

  Ptr<CsrNetDevice> sourceDevice =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrNetDevice> destinationDevice =
    CreateObject<CsrNetDevice> (DESTINATION_NODE);
  Ptr<CsrHopLayer> sourceHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> destinationHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> sourceNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> destinationNwk = CreateObject<CsrNetLayer> ();

  sourceDevice->AddPeer (destinationDevice);
  destinationDevice->AddPeer (sourceDevice);
  ConfigureNoErrors (sourceDevice);
  ConfigureNoErrors (destinationDevice);

  ConnectStack (sourceDevice, sourceHop, sourceNwk, SOURCE_NODE);
  ConnectStack (destinationDevice,
                destinationHop,
                destinationNwk,
                DESTINATION_NODE);

  sourceNwk->AddStaticRouteWithPathloss (
    DESTINATION_NODE,
    DESTINATION_NODE,
    70.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));
  destinationNwk->SetRxFromNetCallback (MakeCallback (&RecordPayload));

  // OPNET inserts DSCP zero at the tail and every positive DSCP at the head.
  // Arrival order 10(BE), 30(P), 20(BE), 40(P) therefore becomes
  // 40(P), 30(P), 10(BE), 20(BE).
  // Requesting non-ACKable DATA deliberately exercises the legacy global
  // ENABLE_ACK_RESEND behavior: NWK must still make every DATA reliable.
  sourceNwk->Send (DESTINATION_NODE, 0, Create<Packet> (10), false);
  sourceNwk->Send (DESTINATION_NODE, 5, Create<Packet> (30), false);
  sourceNwk->Send (DESTINATION_NODE, 0, Create<Packet> (20), false);
  sourceNwk->Send (DESTINATION_NODE, 5, Create<Packet> (40), false);

  Simulator::ScheduleNow (&CheckBeforeTic, sourceNwk);
  Simulator::Schedule (MicroSeconds (1),
                       &CheckAfterTic,
                       sourceNwk,
                       sourceHop);

  Simulator::Stop (Seconds (20.0));
  Simulator::Run ();

  const std::vector<uint32_t> expected {40, 30, 10, 20};
  Require (g_receivedSizes == expected,
           "NWK did not reproduce OPNET head/tail queue ordering");

  Simulator::Destroy ();
}

void
RunNoRouteHoldScenario ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();

  ConnectStack (device, hop, nwk, SOURCE_NODE);

  nwk->AddStaticRouteWithPathloss (
    ALTERNATE_NODE,
    ALTERNATE_NODE,
    70.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));

  // The positive-DSCP route-less packet is at the head.  OPNET holds it,
  // continues scanning, and admits the routed best-effort packet behind it.
  nwk->Send (DESTINATION_NODE, 5, Create<Packet> (16), true);
  nwk->Send (ALTERNATE_NODE, 0, Create<Packet> (24), true);

  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (nwk->GetNwkQueueSize () == 1,
           "route-less NWK packet was not the only held queue entry");
  Require (!nwk->IsDiscoveryActive (),
           "route-less NWK packet triggered non-legacy on-demand discovery");
  Require (hop->GetPendingDataCount () == 1,
           "NWK failed to scan past a route-less queue head");
  Require (hop->GetOutstandingDataCount (DESTINATION_NODE) == 0,
           "route-less NWK packet leaked into HOP");
  Require (hop->GetOutstandingDataCount (ALTERNATE_NODE) == 1,
           "routed packet behind a blocked head did not reach HOP");

  Simulator::Destroy ();
}

void
RunDirectCapabilityScenario ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();

  ConnectStack (device, hop, nwk, SOURCE_NODE);

  nwk->ProcessHello (
    MakeHello (ALTERNATE_NODE, CsrNodeType::Routable),
    ALTERNATE_NODE,
    70.0,
    20.0);
  nwk->ProcessHello (
    MakeHello (DESTINATION_NODE, CsrNodeType::Routable),
    DESTINATION_NODE,
    70.0,
    20.0);

  // Learn a conflicting reverse route to node 2 through node 3.  A direct
  // Routable destination must still win when its HELLO capability is kept.
  Ptr<Packet> reverseTraffic = Create<Packet> (8);
  CsrNetHeader reverseHeader (DESTINATION_NODE, SOURCE_NODE, 0);
  reverseTraffic->AddHeader (reverseHeader);
  nwk->ReceiveFromHop (reverseTraffic, ALTERNATE_NODE);

  nwk->Send (DESTINATION_NODE, 5, Create<Packet> (16), true);

  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (hop->GetOutstandingDataCount (DESTINATION_NODE) == 1,
           "HELLO capability was lost and direct Routable path was bypassed");
  Require (hop->GetOutstandingDataCount (ALTERNATE_NODE) == 0,
           "reverse route incorrectly overrode a direct Routable path");

  Simulator::Destroy ();
}

void
RunMonotonicActiveNodeScenario ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();

  ConnectStack (device, hop, nwk, SOURCE_NODE);

  nwk->ProcessHello (
    MakeHello (DESTINATION_NODE, CsrNodeType::Routable),
    DESTINATION_NODE,
    70.0,
    20.0);
  nwk->ProcessHello (
    MakeHello (ALTERNATE_NODE, CsrNodeType::Routable),
    ALTERNATE_NODE,
    70.0,
    20.0);

  Require (device->GetMac ().GetReportedActiveNodesForSlotting () == 3,
           "active-node population did not grow with the neighbor table");

  nwk->ClearRoutes ();

  Require (device->GetMac ().GetReportedActiveNodesForSlotting () == 3,
           "active-node population decreased after neighbors became stale");

  Simulator::Destroy ();
}

void
RunLocalOnlyLinkCostScenario ()
{
  Ptr<CsrNetLayer> localOnly = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> withRemoteInfo = CreateObject<CsrNetLayer> ();

  localOnly->SetNodeId (SOURCE_NODE);
  withRemoteInfo->SetNodeId (SOURCE_NODE);

  localOnly->ProcessHello (
    MakeHello (DESTINATION_NODE, CsrNodeType::Routable),
    DESTINATION_NODE,
    70.0,
    20.0);
  withRemoteInfo->ProcessHello (
    MakeHello (DESTINATION_NODE, CsrNodeType::Routable),
    DESTINATION_NODE,
    70.0,
    20.0);

  // OPNET stores the ARL INFO fields, but its
  // linkCharSetRemoteParams() stub does nothing and linkCharGetCost()
  // reports only localCost as valid. These deliberately restrictive remote
  // limits therefore must not change the selected direct-route cost.
  localOnly->ProcessHello (
    MakeHello (DESTINATION_NODE, CsrNodeType::Routable),
    DESTINATION_NODE,
    70.0,
    20.0);
  withRemoteInfo->ProcessHello (
    MakeRoutingInfo (DESTINATION_NODE),
    DESTINATION_NODE,
    70.0,
    20.0);

  localOnly->ProcessHello (
    MakeHello (DESTINATION_NODE, CsrNodeType::Routable),
    DESTINATION_NODE,
    70.0,
    20.0);
  withRemoteInfo->ProcessHello (
    MakeHello (DESTINATION_NODE, CsrNodeType::Routable),
    DESTINATION_NODE,
    70.0,
    20.0);

  uint32_t localCost = 0;
  uint32_t remoteInfoCost = 0;
  Require (localOnly->GetSelectedRouteCost (DESTINATION_NODE, localCost),
           "local-only control node has no direct route");
  Require (withRemoteInfo->GetSelectedRouteCost (DESTINATION_NODE,
                                                  remoteInfoCost),
           "remote-INFO node has no direct route");
  Require (remoteInfoCost == localCost,
           "remote RoutingInfo incorrectly changed OPNET local-only cost");

  // routesReroute() asks linkCharGetCost() again after a failure. The remote
  // INFO values must remain irrelevant on this recomputation path too.
  localOnly->NoteLinkFailure (DESTINATION_NODE);
  withRemoteInfo->NoteLinkFailure (DESTINATION_NODE);

  Require (localOnly->GetSelectedRouteCost (DESTINATION_NODE, localCost),
           "local-only route disappeared after link-cost recomputation");
  Require (withRemoteInfo->GetSelectedRouteCost (DESTINATION_NODE,
                                                  remoteInfoCost),
           "remote-INFO route disappeared after link-cost recomputation");
  Require (remoteInfoCost == localCost,
           "remote RoutingInfo changed recomputed OPNET local-only cost");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  RunQueueOrderingAndTicScenario ();
  RunNoRouteHoldScenario ();
  RunDirectCapabilityScenario ();
  RunMonotonicActiveNodeScenario ();
  RunLocalOnlyLinkCostScenario ();

  std::cout << "PASS: strict OPNET NWK legacy behavior test"
            << std::endl;
  return 0;
}
