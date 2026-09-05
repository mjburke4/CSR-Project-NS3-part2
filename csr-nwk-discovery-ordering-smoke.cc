#include "ns3/core-module.h"
#include "ns3/csr-arl-routing-message.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId GATEWAY_NODE = 0;
constexpr CsrNodeId OLDER_DESTINATION = 1;
constexpr CsrNodeId NEWER_DESTINATION = 2;
constexpr CsrNodeId REVERSE_ONLY_DESTINATION = 3;

struct DiscoverySnapshot
{
  bool active {false};
  uint32_t starts {0};
  uint32_t broadcasts {0};
};

std::vector<std::string> g_failures;
uint32_t g_reverseStartFrames = 0;
CsrNodeId g_reverseStartHop = CSR_BROADCAST_ID;
CsrNodeId g_reverseStartDestination = CSR_BROADCAST_ID;
CsrDestType g_reverseStartDestinationType = CSR_DEST_BROADCAST;

Ptr<Packet>
MakePlainHello (CsrNodeId source)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (1);
  hello.SetNodeType (CsrNodeType::Routable);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::None);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  return packet;
}

Ptr<Packet>
MakeDiscoveryHello (CsrNodeId source, uint32_t sequence)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (static_cast<uint16_t> (sequence));
  hello.SetNodeType (CsrNodeType::Routable);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::Discover);
  hello.SetDiscoverType (CsrDiscoverType::Broadcast);
  hello.SetDiscoverySequence (sequence);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  return packet;
}

void
Expect (bool condition, const char *message)
{
  if (!condition)
    {
      g_failures.emplace_back (message);
    }
}

Ptr<Packet>
MakeArlPathUpdate (CsrNodeId source,
                   CsrNodeId destination,
                   CsrNodeId pathOnlyNode,
                   uint32_t sequence)
{
  CsrArlRoutingMessage::Builder builder;
  std::string error;
  NS_ABORT_MSG_IF (!builder.AddUpdate (
                     destination,
                     static_cast<uint8_t> (CsrNodeType::Routable),
                     1,
                     9,
                     {pathOnlyNode},
                     &error),
                   "could not build ARL path-only fixture: " << error);

  std::vector<std::vector<uint8_t>> sections;
  NS_ABORT_MSG_IF (!builder.BuildSections (sequence, sections, &error) ||
                     sections.size () != 1,
                   "ARL path-only fixture did not build one section: "
                     << error);

  CsrArlRoutingMessage::Section section;
  NS_ABORT_MSG_IF (!CsrArlRoutingMessage::DecodeSection (
                     sections.front (), section, &error),
                   "could not decode ARL path-only fixture: " << error);

  Ptr<Packet> packet = Create<Packet> (
    sections.front ().data (), sections.front ().size ());
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (static_cast<uint16_t> (sequence));
  hello.SetNodeType (CsrNodeType::Routable);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::RoutingUpdate);
  hello.SetRoutingSequence (section.sequence);
  hello.SetRoutingSection (section.section);
  hello.SetRoutingTotalSections (section.totalSections);
  hello.SetRoutingOperation (CsrRoutingOperation::Update);
  packet->AddHeader (hello);
  return packet;
}

Ptr<Packet>
MakeCompatibilityPathUpdate (CsrNodeId source,
                             CsrNodeId destination,
                             CsrNodeId pathOnlyNode,
                             uint32_t sequence)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (static_cast<uint16_t> (sequence));
  hello.SetNodeType (CsrNodeType::Routable);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::RoutingUpdate);
  hello.SetRoutingSequence (sequence);
  hello.SetRoutingSection (0);
  hello.SetRoutingTotalSections (1);
  hello.SetRoutingOperation (CsrRoutingOperation::Update);
  NS_ABORT_MSG_IF (!hello.AddAdvertisedRoute (
                     destination,
                     1,
                     9,
                     0,
                     static_cast<uint8_t> (CsrNodeType::Routable),
                     {pathOnlyNode}),
                   "could not build compatibility path-only fixture");

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  return packet;
}

void
CaptureDiscovery (Ptr<CsrNetLayer> nwk, DiscoverySnapshot *snapshot)
{
  snapshot->active = nwk->IsDiscoveryActive ();
  snapshot->starts = nwk->GetDiscoveryStartCount ();
  snapshot->broadcasts = nwk->GetDiscoveryBroadcastCount ();
}

void
ObserveReverseSnmp (Ptr<Packet> frame, double, double)
{
  CsrHeader hopHeader;
  if (!frame->RemoveHeader (hopHeader) ||
      hopHeader.GetType () != CSR_PKT_SNMP)
    {
      return;
    }

  CsrSnmpHeader snmpHeader;
  if (!frame->RemoveHeader (snmpHeader) ||
      snmpHeader.GetCommand () != CSR_SNMP_START_DISCOVERY)
    {
      return;
    }

  g_reverseStartFrames++;
  g_reverseStartHop = hopHeader.GetDst ();
  g_reverseStartDestination = snmpHeader.GetDestination ();
  g_reverseStartDestinationType = snmpHeader.GetDestinationType ();
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
RunCadenceAndCompletionCase ()
{
  Ptr<CsrNetLayer> gateway = CreateObject<CsrNetLayer> ();
  gateway->SetNodeId (GATEWAY_NODE);
  gateway->SetNodeType (CsrNodeType::Gateway);

  DiscoverySnapshot beforeStart;
  DiscoverySnapshot afterFirst;
  DiscoverySnapshot beforeSecond;
  DiscoverySnapshot afterSecond;
  DiscoverySnapshot beforeThird;
  DiscoverySnapshot afterThird;
  DiscoverySnapshot beforeCompletion;
  DiscoverySnapshot afterCompletion;

  // br_nwk schedules the gateway at +10 seconds.  routesDiscoveryInitLocal()
  // then broadcasts at 10, 15, and 20 seconds.  The fourth five-second
  // broadcaster callback ends discovery at 25 seconds; the +30-second NWK
  // stop is only a fallback.
  gateway->ScheduleGatewayStartupDiscovery ();

  Simulator::Schedule (Seconds (10.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeStart);
  Simulator::Schedule (Seconds (10.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterFirst);
  Simulator::Schedule (Seconds (15.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeSecond);
  Simulator::Schedule (Seconds (15.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterSecond);
  Simulator::Schedule (Seconds (20.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeThird);
  Simulator::Schedule (Seconds (20.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterThird);
  Simulator::Schedule (Seconds (25.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeCompletion);
  Simulator::Schedule (Seconds (25.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterCompletion);

  Simulator::Stop (Seconds (26.0));
  Simulator::Run ();
  Simulator::Destroy ();

  Expect (!beforeStart.active &&
            beforeStart.starts == 0 &&
            beforeStart.broadcasts == 0,
          "gateway discovery started before the OPNET +10-second event");
  Expect (afterFirst.active &&
            afterFirst.starts == 1 &&
            afterFirst.broadcasts == 1,
          "gateway did not emit its first discovery at 10 seconds");
  Expect (beforeSecond.active && beforeSecond.broadcasts == 1,
          "gateway emitted a repeat before the five-second OPNET interval");
  Expect (afterSecond.active && afterSecond.broadcasts == 2,
          "gateway did not emit its second discovery at 15 seconds");
  Expect (beforeThird.active && beforeThird.broadcasts == 2,
          "gateway emitted its third discovery before 20 seconds");
  Expect (afterThird.active && afterThird.broadcasts == 3,
          "gateway did not emit its third discovery at 20 seconds");
  Expect (beforeCompletion.active && beforeCompletion.broadcasts == 3,
          "gateway discovery completed before the 25-second fourth tick");
  Expect (!afterCompletion.active &&
            afterCompletion.starts == 1 &&
            afterCompletion.broadcasts == 3,
          "gateway discovery did not complete on the 25-second fourth tick");
}

void
RunNewestDestinationFirstCase ()
{
  Ptr<CsrNetDevice> gatewayDevice =
    CreateObject<CsrNetDevice> (GATEWAY_NODE);
  Ptr<CsrNetDevice> olderDevice =
    CreateObject<CsrNetDevice> (OLDER_DESTINATION);
  Ptr<CsrNetDevice> newerDevice =
    CreateObject<CsrNetDevice> (NEWER_DESTINATION);

  gatewayDevice->AddPeer (olderDevice);
  gatewayDevice->AddPeer (newerDevice);
  olderDevice->AddPeer (gatewayDevice);
  newerDevice->AddPeer (gatewayDevice);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  gatewayDevice->GetPhy ().SetPerModel (noErrors);
  olderDevice->GetPhy ().SetPerModel (noErrors);
  newerDevice->GetPhy ().SetPerModel (noErrors);

  gatewayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);
  olderDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);
  gatewayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, NEWER_DESTINATION, 1.0);
  newerDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, NEWER_DESTINATION, 1.0);

  Ptr<CsrHopLayer> gatewayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> olderHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> newerHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> gatewayNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> olderNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> newerNwk = CreateObject<CsrNetLayer> ();

  ConnectStack (
    gatewayDevice, gatewayHop, gatewayNwk, GATEWAY_NODE);
  ConnectStack (
    olderDevice, olderHop, olderNwk, OLDER_DESTINATION);
  ConnectStack (
    newerDevice, newerHop, newerNwk, NEWER_DESTINATION);

  gatewayNwk->SetNodeType (CsrNodeType::Gateway);
  olderNwk->SetNodeType (CsrNodeType::Routable);
  newerNwk->SetNodeType (CsrNodeType::Routable);

  // Keep this case focused on logical-destination ordering rather than ARL
  // admission.  routesFindDestination() inserts every newly created logical
  // destination at destHead, so destination 2 must precede destination 1
  // when clear_discovery() tail-appends that walk into the discovery table.
  gatewayNwk->SetArlNeighborAdmissionEnabled (false);
  olderNwk->SetArlNeighborAdmissionEnabled (false);
  newerNwk->SetArlNeighborAdmissionEnabled (false);

  // routesGetRelay() creates a logical destination even when lookup fails.
  // Create destination 1 first and destination 2 second, then add route
  // records in the oldest-first vector order used by the old model.
  Expect (!gatewayNwk->HasRelayRoute (OLDER_DESTINATION),
          "route lookup unexpectedly found the older destination");
  Expect (!gatewayNwk->HasRelayRoute (NEWER_DESTINATION),
          "route lookup unexpectedly found the newer destination");
  gatewayNwk->AddStaticRoute (OLDER_DESTINATION, OLDER_DESTINATION);
  gatewayNwk->AddStaticRoute (NEWER_DESTINATION, NEWER_DESTINATION);

  // Use the explicit stop solely to isolate handoff order from the cadence
  // case above.  The first discovery-table entry is marked complete before
  // the SNMP START is sent, and no second target is considered until DONE or
  // the 60-second watchdog.
  gatewayNwk->StartDiscovery (Seconds (0.0), MilliSeconds (100));

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();
  Simulator::Destroy ();

  Expect (newerNwk->GetSnmpStartReceivedCount () == 1,
          "newest logical destination did not receive the first SNMP START");
  Expect (olderNwk->GetSnmpStartReceivedCount () == 0,
          "oldest logical destination was selected before the destHead entry");
  Expect (gatewayNwk->GetSnmpStartSentCount () == 1,
          "discovery ordering case sent more than one serialized SNMP START");
}

void
RunReverseOnlyDestinationCase ()
{
  Ptr<CsrNetDevice> gatewayDevice =
    CreateObject<CsrNetDevice> (GATEWAY_NODE);
  Ptr<CsrNetDevice> relayDevice =
    CreateObject<CsrNetDevice> (OLDER_DESTINATION);

  gatewayDevice->AddPeer (relayDevice);
  relayDevice->AddPeer (gatewayDevice);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  gatewayDevice->GetPhy ().SetPerModel (noErrors);
  relayDevice->GetPhy ().SetPerModel (noErrors);
  gatewayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);
  relayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);

  Ptr<CsrHopLayer> gatewayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> relayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> gateway = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> relay = CreateObject<CsrNetLayer> ();

  ConnectStack (
    gatewayDevice, gatewayHop, gateway, GATEWAY_NODE);
  ConnectStack (
    relayDevice, relayHop, relay, OLDER_DESTINATION);
  relayDevice->GetMac ().SetRxCallback (
    MakeCallback (&ObserveReverseSnmp));

  gateway->SetNodeType (CsrNodeType::Gateway);
  relay->SetNodeType (CsrNodeType::Routable);
  gateway->SetArlNeighborAdmissionEnabled (false);
  relay->SetArlNeighborAdmissionEnabled (false);

  gateway->ProcessHello (
    MakePlainHello (OLDER_DESTINATION),
    OLDER_DESTINATION,
    70.0,
    20.0);

  Ptr<Packet> reversePacket = Create<Packet> (8);
  CsrNetHeader networkHeader (
    REVERSE_ONLY_DESTINATION,
    GATEWAY_NODE,
    0);
  reversePacket->AddHeader (networkHeader);
  gateway->ReceiveFromHop (reversePacket, OLDER_DESTINATION);

  std::vector<CsrNodeId> knownNodes = gateway->GetKnownDiscoveryNodes ();
  Expect (knownNodes.size () >= 2,
          "reverse-only destination was omitted from the discovery walk");
  if (knownNodes.size () >= 2)
    {
      Expect (knownNodes[0] == REVERSE_ONLY_DESTINATION,
              "reverse-only destination did not retain newest-first order");
      Expect (knownNodes[1] == OLDER_DESTINATION,
              "direct neighbor moved ahead of the newer reverse destination");
    }

  // clear_discovery() copies the selected reverse hop into the NWK route
  // table before send_snmp_pk() looks it up.  The one-hop HOP recipient is
  // not the final destination, but NWK must still emit and count the START.
  g_reverseStartFrames = 0;
  g_reverseStartHop = CSR_BROADCAST_ID;
  g_reverseStartDestination = CSR_BROADCAST_ID;
  g_reverseStartDestinationType = CSR_DEST_BROADCAST;
  gateway->StartDiscovery (Seconds (0.0), MilliSeconds (100));
  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Expect (gateway->GetSnmpStartSentCount () == 1,
          "reverse-only discovery target did not emit an SNMP START");
  Expect (g_reverseStartFrames == 1,
          "reverse-only SNMP START was not observed exactly once");
  Expect (g_reverseStartHop == OLDER_DESTINATION,
          "reverse-only SNMP START used the wrong HOP destination");
  Expect (g_reverseStartDestination == REVERSE_ONLY_DESTINATION,
          "reverse-only SNMP START changed its final destination");
  Expect (g_reverseStartDestinationType == CSR_DEST_UNICAST,
          "reverse-only SNMP START was not unicast");

  Simulator::Destroy ();
}

void
RunDiscoverySequenceAndChirpOrderCase ()
{
  constexpr CsrNodeId localNode = 10;
  constexpr CsrNodeId olderNeighbor = 2;
  constexpr CsrNodeId middleNeighbor = 7;
  constexpr CsrNodeId newerNeighbor = 4;

  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  nwk->SetNodeId (localNode);
  nwk->SetArlNeighborAdmissionEnabled (false);
  nwk->SetDiscoveryResponseEnabled (false);

  nwk->ProcessHello (
    MakeDiscoveryHello (olderNeighbor, 1),
    olderNeighbor,
    70.0,
    20.0);

  uint32_t retainedSequence = 0;
  Expect (nwk->GetNeighborDiscoverySequenceForTest (
            olderNeighbor,
            retainedSequence) &&
            retainedSequence == 1,
          "initial Discover sequence was not retained");

  // routesCompareSequence() deliberately treats the exact unsigned
  // half-range as non-newer in both directions.  Avoid relying on a signed
  // narrowing conversion for this source-visible boundary.
  nwk->ProcessHello (
    MakeDiscoveryHello (olderNeighbor, 0x80000001u),
    olderNeighbor,
    70.0,
    20.0);
  Expect (nwk->GetNeighborDiscoverySequenceForTest (
            olderNeighbor,
            retainedSequence) &&
            retainedSequence == 1,
          "exact-half-range Discover replaced the retained sequence");

  nwk->ProcessHello (
    MakePlainHello (middleNeighbor),
    middleNeighbor,
    70.0,
    20.0);
  nwk->ProcessHello (
    MakePlainHello (newerNeighbor),
    newerNeighbor,
    70.0,
    20.0);

  Expect (nwk->GetActiveChirpNeighborsForTest () ==
            std::vector<CsrNodeId> ({newerNeighbor,
                                     middleNeighbor,
                                     olderNeighbor}),
          "Chirp neighbors were not serialized newest-created first");

  Simulator::Destroy ();
}

void
RunPathOnlyNeighborCreationCase (bool arlByteStream)
{
  constexpr CsrNodeId localNode = 10;
  constexpr CsrNodeId reporter = 20;
  constexpr CsrNodeId middleNeighbor = 21;
  constexpr CsrNodeId pathOnlyNode = 22;
  constexpr CsrNodeId advertisedDestination = 23;

  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  nwk->SetNodeId (localNode);
  nwk->SetArlNeighborAdmissionEnabled (false);
  nwk->SetAutomaticRoutePropagationEnabled (false);

  nwk->ProcessHello (
    MakePlainHello (reporter), reporter, 70.0, 20.0);
  nwk->ProcessHello (
    arlByteStream
      ? MakeArlPathUpdate (
          reporter, advertisedDestination, pathOnlyNode, 10)
      : MakeCompatibilityPathUpdate (
          reporter, advertisedDestination, pathOnlyNode, 10),
    reporter,
    70.0,
    20.0);
  nwk->ProcessHello (
    MakePlainHello (middleNeighbor), middleNeighbor, 70.0, 20.0);

  // A path identifier is only a hop-list value in routes.c; it must not call
  // routesFindNeighbor().  Its first direct HELLO therefore creates it now at
  // neighborHead, ahead of the neighbor heard immediately before it.
  nwk->ProcessHello (
    MakePlainHello (pathOnlyNode), pathOnlyNode, 70.0, 20.0);

  Expect (nwk->GetActiveChirpNeighborsForTest () ==
            std::vector<CsrNodeId> ({pathOnlyNode,
                                     middleNeighbor,
                                     reporter}),
          arlByteStream
            ? "ARL path mention pre-created a Chirp neighbor"
            : "compatibility path mention pre-created a Chirp neighbor");

  Simulator::Destroy ();
}

void
RunActiveToInactiveChirpCase ()
{
  constexpr CsrNodeId localNode = 10;
  constexpr CsrNodeId olderNeighbor = 2;
  constexpr CsrNodeId newerNeighbor = 3;

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (localNode);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  ConnectStack (device, hop, nwk, localNode);
  nwk->SetArlNeighborAdmissionEnabled (false);
  nwk->SetAutomaticRoutePropagationEnabled (false);

  nwk->ProcessHello (
    MakePlainHello (olderNeighbor), olderNeighbor, 70.0, 20.0);
  nwk->ProcessHello (
    MakePlainHello (newerNeighbor), newerNeighbor, 70.0, 20.0);
  Simulator::Run ();
  Expect (nwk->IsArlNeighborActive (olderNeighbor) &&
            nwk->IsArlNeighborActive (newerNeighbor),
          "security-reset Chirp fixture neighbors were not actually active");

  const uint32_t chirpsBefore = nwk->GetDiscoveryChirpCountForTest ();
  const uint64_t transmissionsBefore =
    device->GetMac ().GetTransmittedFrameCount ();
  nwk->NoteSecurityCountChangeForTest (olderNeighbor);
  nwk->NoteSecurityCountChangeForTest (newerNeighbor);
  nwk->NoteSecurityCountChangeForTest (olderNeighbor);

  Expect (!nwk->IsArlNeighborActive (olderNeighbor) &&
            !nwk->IsArlNeighborActive (newerNeighbor) &&
            nwk->GetDiscoveryChirpCountForTest () == chirpsBefore,
          "active-to-inactive transition sent Chirp synchronously");

  Simulator::Schedule (NanoSeconds (1), [] {
    Simulator::Stop ();
  });
  Simulator::Run ();

  Expect (nwk->GetDiscoveryChirpCountForTest () == chirpsBefore + 1,
          "security-count active-to-inactive transition did not wake one Chirp");
  Expect (nwk->GetActiveChirpNeighborsForTest ().empty (),
          "inactive neighbor remained in the emitted Chirp population");
  Expect (device->GetMac ().GetQueuedFrameCount () != 0 ||
            device->GetMac ().GetTransmittedFrameCount () > transmissionsBefore,
          "scheduled Chirp was not handed to HOP/MAC");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  RunCadenceAndCompletionCase ();
  RunNewestDestinationFirstCase ();
  RunReverseOnlyDestinationCase ();
  RunDiscoverySequenceAndChirpOrderCase ();
  RunPathOnlyNeighborCreationCase (true);
  RunPathOnlyNeighborCreationCase (false);
  RunActiveToInactiveChirpCase ();

  if (!g_failures.empty ())
    {
      for (const std::string &failure : g_failures)
        {
          std::cerr << "FAIL: " << failure << std::endl;
        }
      return 1;
    }

  std::cout << "PASS: OPNET discovery cadence/completion and destination order"
            << std::endl;
  return 0;
}
