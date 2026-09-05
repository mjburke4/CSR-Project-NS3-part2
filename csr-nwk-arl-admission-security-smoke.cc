#include "ns3/core-module.h"
#include "ns3/csr-arl-routing-message.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId SOURCE_NODE = 1;
constexpr CsrNodeId DESTINATION_NODE = 2;
constexpr CsrNodeId CACHED_DESTINATION_NODE = 3;
constexpr CsrNodeId LOOP_RECOMPUTE_DESTINATION_NODE = 4;
constexpr CsrNodeId ALTERNATE_REPORTER_NODE = 5;

uint32_t g_deliveries = 0;
bool g_cachedRouteBoundaryChecked = false;

void
Require (bool condition, const char *message)
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
ReceivePayload (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE_NODE,
           "admitted DATA carried the wrong NWK source");
  Require (payload->GetSize () == 64,
           "admitted DATA payload changed size");
  g_deliveries++;
}

Ptr<Packet>
MakeRoutingUpdate (CsrNodeId reporter,
                   CsrNodeId destination,
                   uint32_t sequence,
                   uint32_t advertisedCost,
                   const std::vector<CsrNodeId> &path)
{
  CsrArlRoutingMessage::Builder builder;
  std::string error;
  Require (builder.AddUpdate (
             destination,
             static_cast<uint8_t> (CsrNodeType::Routable),
             1,
             advertisedCost,
             path,
             &error),
           "could not build transitive UPDATE");
  builder.AddFlush ();

  std::vector<std::vector<uint8_t>> sections;
  Require (builder.BuildSections (sequence, sections, &error) &&
             sections.size () == 1,
           "cached transitive UPDATE did not fit one routing section");

  CsrArlRoutingMessage::Section section;
  Require (CsrArlRoutingMessage::DecodeSection (
             sections.front (), section, &error),
           "could not decode cached transitive routing section");

  Ptr<Packet> packet = Create<Packet> (
    sections.front ().data (),
    sections.front ().size ());

  CsrHelloHeader hello;
  hello.SetNodeId (reporter);
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

void
InjectBeforeAdmission (Ptr<CsrNetLayer> destinationNwk)
{
  Ptr<Packet> packet = Create<Packet> (32);
  CsrNetHeader header (SOURCE_NODE, DESTINATION_NODE, 0);
  packet->AddHeader (header);

  destinationNwk->ReceiveFromHop (packet, SOURCE_NODE);

  Require (g_deliveries == 0,
           "DATA from an inactive neighbor bypassed ARL admission");
  Require (destinationNwk->GetInactiveNeighborDropCount () == 1,
           "inactive-neighbor DATA drop was not recorded");
  Require (!destinationNwk->IsArlNeighborActive (SOURCE_NODE),
           "Neighbor became active before key/check completion");
}

void
CheckAdmissionAndSend (Ptr<CsrNetLayer> sourceNwk,
                       Ptr<CsrNetLayer> destinationNwk)
{
  Require (sourceNwk->HasReceivedHopKey (DESTINATION_NODE) &&
           sourceNwk->HasSentHopKey (DESTINATION_NODE),
           "source did not complete the two-sided group-key exchange");
  Require (destinationNwk->HasReceivedHopKey (SOURCE_NODE) &&
           destinationNwk->HasSentHopKey (SOURCE_NODE),
           "destination did not complete the two-sided group-key exchange");

  Require (sourceNwk->IsArlNeighborActive (DESTINATION_NODE),
           "source did not admit the neighbor after NeighborCheck");
  Require (destinationNwk->IsArlNeighborActive (SOURCE_NODE),
           "destination did not admit the neighbor after NeighborCheck");

  Require (sourceNwk->GetKeyRequestSentCount () +
             destinationNwk->GetKeyRequestSentCount () >= 1,
           "admission did not transmit a KeyRequest");
  Require (sourceNwk->GetKeyUpdateSentCount () >= 1 &&
           destinationNwk->GetKeyUpdateSentCount () >= 1,
           "reciprocal reliable KeyUpdates were not transmitted");
  Require (sourceNwk->GetKeyUpdateReceivedCount () >= 1 &&
           destinationNwk->GetKeyUpdateReceivedCount () >= 1,
           "reciprocal KeyUpdates were not received");

  uint32_t routeCost = 0;
  Require (sourceNwk->GetSelectedRouteCost (
             DESTINATION_NODE, routeCost),
           "admitted direct route did not become selectable");

  sourceNwk->Send (DESTINATION_NODE,
                   5,
                   Create<Packet> (64),
                   true);
}

void
CheckCachedRouteAdmissionBoundary (Ptr<CsrNetLayer> sourceNwk)
{
  if (!sourceNwk->IsArlNeighborActive (DESTINATION_NODE) ||
      !sourceNwk->IsArlNeighborActive (ALTERNATE_REPORTER_NODE))
    {
      Simulator::Schedule (MicroSeconds (100),
                           &CheckCachedRouteAdmissionBoundary,
                           sourceNwk);
      return;
    }

  uint32_t routeCost = 0;
  Require (sourceNwk->GetSelectedRouteCost (
             DESTINATION_NODE, routeCost),
           "admission did not activate the direct neighbor route");

  // routesMakeNeighborActive() recomputes only the direct neighbor.  A
  // transitive route cached while that neighbor was inactive must wait for a
  // later destination recomputation; a fresh RoutingUpdate supplies one.
  Require (!sourceNwk->GetSelectedRouteCost (
              CACHED_DESTINATION_NODE, routeCost),
           "neighbor admission activated a cached transitive route");
  Require (sourceNwk->IsArlNeighborActive (ALTERNATE_REPORTER_NODE),
           "alternate reporter did not complete neighbor admission");
  Require (!sourceNwk->GetSelectedRouteCost (
              LOOP_RECOMPUTE_DESTINATION_NODE, routeCost),
           "alternate admission activated its cached transitive route");

  // The primary reporter failed while still inactive.  A looped UPDATE for
  // the same destination supplies a recomputation without adding a usable
  // route; the pre-admission candidate must already have been deleted.
  sourceNwk->ProcessHello (
    MakeRoutingUpdate (ALTERNATE_REPORTER_NODE,
                       CACHED_DESTINATION_NODE,
                       19,
                       22,
                       {SOURCE_NODE}),
    ALTERNATE_REPORTER_NODE,
    70.0,
    20.0);
  Require (!sourceNwk->GetSelectedRouteCost (
              CACHED_DESTINATION_NODE, routeCost),
           "pre-admission failure retained a cached transit route");

  sourceNwk->ProcessHello (
    MakeRoutingUpdate (DESTINATION_NODE,
                       CACHED_DESTINATION_NODE,
                       21,
                       21,
                       {CACHED_DESTINATION_NODE}),
    DESTINATION_NODE,
    70.0,
    20.0);
  Require (sourceNwk->GetSelectedRouteCost (
             CACHED_DESTINATION_NODE, routeCost),
           "fresh post-admission route update did not activate the route");

  // routesParseRouting() retains a looped UPDATE as an invalid candidate and
  // still calls routesFindBestRoute() for that destination.  The call must
  // release a different cached candidate whose reporter is now active.
  sourceNwk->ProcessHello (
    MakeRoutingUpdate (DESTINATION_NODE,
                       LOOP_RECOMPUTE_DESTINATION_NODE,
                       22,
                       22,
                       {SOURCE_NODE}),
    DESTINATION_NODE,
    70.0,
    20.0);
  Require (sourceNwk->GetSelectedRouteCost (
             LOOP_RECOMPUTE_DESTINATION_NODE, routeCost),
           "looped UPDATE omitted destination recomputation");

  // routesMakeNeighborInactive() deletes every transit candidate learned
  // from that reporter.  Re-admitting the peer must not resurrect the old
  // candidate; only a fresh UPDATE may restore it.
  sourceNwk->NoteLinkFailure (ALTERNATE_REPORTER_NODE);
  Require (!sourceNwk->GetSelectedRouteCost (
              LOOP_RECOMPUTE_DESTINATION_NODE, routeCost),
           "inactive reporter retained its learned transit route");

  sourceNwk->NoteNeighborCheckSuccess (
    ALTERNATE_REPORTER_NODE,
    CsrNeighborCheckType::Message,
    0);
  Require (sourceNwk->IsArlNeighborActive (ALTERNATE_REPORTER_NODE),
           "alternate reporter did not re-enter active state");
  Require (!sourceNwk->GetSelectedRouteCost (
              LOOP_RECOMPUTE_DESTINATION_NODE, routeCost),
           "re-admission resurrected a deleted transit route");

  sourceNwk->ProcessHello (
    MakeRoutingUpdate (ALTERNATE_REPORTER_NODE,
                       LOOP_RECOMPUTE_DESTINATION_NODE,
                       21,
                       31,
                       {LOOP_RECOMPUTE_DESTINATION_NODE}),
    ALTERNATE_REPORTER_NODE,
    70.0,
    20.0);
  Require (sourceNwk->GetSelectedRouteCost (
             LOOP_RECOMPUTE_DESTINATION_NODE, routeCost),
           "fresh UPDATE did not restore the deleted transit route");

  g_cachedRouteBoundaryChecked = true;
}

void
CheckFinalState (Ptr<CsrNetLayer> sourceNwk,
                 Ptr<CsrHopLayer> sourceHop)
{
  Require (g_cachedRouteBoundaryChecked,
           "cached-route admission boundary was not observed");
  Require (g_deliveries == 1,
           "DATA was not delivered exactly once after admission");
  Require (sourceNwk->GetNwkQueueSize () == 0,
           "source NWK queue did not drain after admission");
  Require (sourceHop->GetPendingDataCount () == 0,
           "source HOP DATA custody did not clear");
}

void
CheckSecurityCountSourceRetention ()
{
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  nwk->SetNodeId (SOURCE_NODE);

  CsrNetLayer::SecurityResetStateForTest before;
  before.discoverySequenceValid = true;
  before.keyUpdateComplete = true;
  before.keyRequestSentValid = true;
  before.keySendComplete = true;
  before.keySendActive = true;
  before.keySendValid = true;
  before.overheardValid = true;
  before.keyRequestDelay = Seconds (11);
  before.keySendDelay = Seconds (13);
  before.overheardDelay = Seconds (17);
  before.numFailures = 9;
  before.admissionRetryPending = true;
  nwk->SetSecurityResetStateForTest (DESTINATION_NODE, before);

  nwk->NoteSecurityCountChangeForTest (DESTINATION_NODE);
  CsrNetLayer::SecurityResetStateForTest after =
    nwk->GetSecurityResetStateForTest (DESTINATION_NODE);

  Require (!after.discoverySequenceValid &&
             !after.keyUpdateComplete &&
             !after.keyRequestSentValid &&
             !after.keySendComplete &&
             !after.keySendValid &&
             after.numFailures == 0,
           "security-count reset did not clear the exact routes.c fields");
  Require (after.keySendActive &&
             after.overheardValid &&
             after.keyRequestDelay == before.keyRequestDelay &&
             after.keySendDelay == before.keySendDelay &&
             after.overheardDelay == before.overheardDelay &&
             after.admissionRetryPending,
           "security-count reset discarded source-owned owner/backoff state");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  CheckSecurityCountSourceRetention ();

  Ptr<CsrNetDevice> sourceDevice =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrNetDevice> destinationDevice =
    CreateObject<CsrNetDevice> (DESTINATION_NODE);
  Ptr<CsrNetDevice> alternateDevice =
    CreateObject<CsrNetDevice> (ALTERNATE_REPORTER_NODE);

  sourceDevice->AddPeer (destinationDevice);
  sourceDevice->AddPeer (alternateDevice);
  destinationDevice->AddPeer (sourceDevice);
  alternateDevice->AddPeer (sourceDevice);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sourceDevice->GetPhy ().SetPerModel (noErrors);
  destinationDevice->GetPhy ().SetPerModel (noErrors);
  alternateDevice->GetPhy ().SetPerModel (noErrors);
  sourceDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, DESTINATION_NODE, 1.0);
  destinationDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, DESTINATION_NODE, 1.0);
  sourceDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, ALTERNATE_REPORTER_NODE, 1.0);
  alternateDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, ALTERNATE_REPORTER_NODE, 1.0);

  Ptr<CsrHopLayer> sourceHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> destinationHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> alternateHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> sourceNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> destinationNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> alternateNwk = CreateObject<CsrNetLayer> ();

  ConnectStack (sourceDevice, sourceHop, sourceNwk, SOURCE_NODE);
  ConnectStack (destinationDevice,
                destinationHop,
                destinationNwk,
                DESTINATION_NODE);
  ConnectStack (alternateDevice,
                alternateHop,
                alternateNwk,
                ALTERNATE_REPORTER_NODE);

  sourceNwk->SetNodeType (CsrNodeType::Routable);
  destinationNwk->SetNodeType (CsrNodeType::Routable);
  alternateNwk->SetNodeType (CsrNodeType::Routable);
  destinationNwk->SetRxFromNetCallback (MakeCallback (&ReceivePayload));

  g_cachedRouteBoundaryChecked = false;
  uint32_t cachedCost = 0;
  sourceNwk->ProcessHello (
    MakeRoutingUpdate (DESTINATION_NODE,
                       CACHED_DESTINATION_NODE,
                       20,
                       20,
                       {CACHED_DESTINATION_NODE}),
    DESTINATION_NODE,
    70.0,
    20.0);
  Require (!sourceNwk->GetSelectedRouteCost (
              CACHED_DESTINATION_NODE, cachedCost),
           "inactive-neighbor transitive route was selectable before admission");

  // routesMakeNeighborInactive() deletes learned routes even if the peer was
  // not active yet.  Exercise that cleanup before the scheduled key/check
  // exchange can admit the reporter.
  sourceNwk->NoteLinkFailure (DESTINATION_NODE);

  sourceNwk->ProcessHello (
    MakeRoutingUpdate (ALTERNATE_REPORTER_NODE,
                       LOOP_RECOMPUTE_DESTINATION_NODE,
                       20,
                       30,
                       {LOOP_RECOMPUTE_DESTINATION_NODE}),
    ALTERNATE_REPORTER_NODE,
    70.0,
    20.0);
  Require (!sourceNwk->GetSelectedRouteCost (
              LOOP_RECOMPUTE_DESTINATION_NODE, cachedCost),
           "alternate inactive-neighbor route was selectable");

  Simulator::ScheduleNow (
    &InjectBeforeAdmission,
    destinationNwk);

  Simulator::Schedule (MicroSeconds (100),
                       &CheckCachedRouteAdmissionBoundary,
                       sourceNwk);

  Simulator::Schedule (Seconds (15),
                       &CheckAdmissionAndSend,
                       sourceNwk,
                       destinationNwk);

  Simulator::Stop (Seconds (30));
  Simulator::Run ();

  CheckFinalState (sourceNwk, sourceHop);

  Simulator::Destroy ();
  std::cout << "PASS: ARL key exchange and neighbor admission test"
            << std::endl;
  return 0;
}
