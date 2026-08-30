#include "ns3/core-module.h"
#include "ns3/csr-arl-routing-message.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId LOCAL_NODE = 0x010203;
constexpr CsrNodeId RECEIVER_NODE = 1;
constexpr CsrNodeId REPORTER_NODE = 2;

void
Require (bool condition, const char *message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

Ptr<Packet>
MakePlainHello (CsrNodeId source,
                CsrNodeType nodeType,
                uint16_t helloSequence = 1)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (helloSequence);
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
MakeCompatibilitySelfUpdate (CsrNodeId source,
                             CsrNodeType capability,
                             uint32_t routingSequence)
{
  CsrHelloHeader update;
  update.SetNodeId (source);
  update.SetHelloSeq (5);
  update.SetNodeType (capability);
  update.SetSpeedKey (8);
  update.SetRxPowerDbmX10 (-1050);
  update.SetActiveNodes (1);
  update.SetArlRouteMsgType (CsrArlRouteMsgType::RoutingUpdate);
  update.SetRoutingSequence (routingSequence);
  update.SetRoutingSection (0);
  update.SetRoutingTotalSections (1);
  update.SetRoutingOperation (CsrRoutingOperation::Update);
  Require (update.AddAdvertisedRoute (
             source,
             0,
             0,
             0,
             static_cast<uint8_t> (capability),
             {}),
           "failed to build compatibility self UPDATE");

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (update);
  return packet;
}

Ptr<Packet>
MakeSectionEnvelope (CsrNodeId source,
                     CsrNodeType outerNodeType,
                     const std::vector<uint8_t> &sectionBytes)
{
  CsrArlRoutingMessage::Section section;
  std::string error;
  Require (CsrArlRoutingMessage::DecodeSection (
             sectionBytes,
             section,
             &error),
           "test could not decode its own routing section");

  Ptr<Packet> packet = Create<Packet> (
    sectionBytes.data (),
    sectionBytes.size ());

  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (
    static_cast<uint16_t> (section.section + 2));
  hello.SetNodeType (outerNodeType);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (
    CsrArlRouteMsgType::RoutingUpdate);
  hello.SetRoutingSequence (section.sequence);
  hello.SetRoutingSection (section.section);
  hello.SetRoutingTotalSections (section.totalSections);
  hello.SetRoutingOperation (CsrRoutingOperation::Update);
  packet->AddHeader (hello);
  return packet;
}

std::vector<std::vector<uint8_t>>
BuildSelfMessage (uint32_t sequence,
                  uint8_t capability,
                  bool deletion)
{
  CsrArlRoutingMessage::Builder builder;
  std::string error;

  if (deletion)
    {
      Require (builder.AddDelete (REPORTER_NODE, &error),
               "failed to encode self DELETE");
    }
  else
    {
      Require (builder.AddUpdate (
                 REPORTER_NODE,
                 capability,
                 0,
                 0,
                 {},
                 &error),
               "failed to encode zero-hop self UPDATE");
    }

  std::vector<std::vector<uint8_t>> sections;
  Require (builder.BuildSections (sequence, sections, &error),
           "failed to section self capability message");
  return sections;
}

std::vector<std::vector<uint8_t>>
BuildOrdinarySnapshot (uint32_t sequence)
{
  CsrArlRoutingMessage::Builder builder;
  CsrHelloHeader::RoutingInfo info;
  builder.AddInfo (info);
  builder.AddFlush ();

  std::vector<std::vector<uint8_t>> sections;
  std::string error;
  Require (builder.BuildSections (sequence, sections, &error),
           "failed to build ordinary-node snapshot");
  return sections;
}

void
DeliverSections (Ptr<CsrNetLayer> receiver,
                 CsrNodeType outerNodeType,
                 const std::vector<std::vector<uint8_t>> &sections)
{
  for (const auto &section : sections)
    {
      receiver->ProcessHello (
        MakeSectionEnvelope (
          REPORTER_NODE,
          outerNodeType,
          section),
        REPORTER_NODE,
        70.0,
        20.0);
    }
}

std::vector<uint8_t>
ExtractRecordStream (const std::vector<Ptr<Packet>> &payloads)
{
  std::vector<uint8_t> stream;

  for (const Ptr<Packet> &payload : payloads)
    {
      Ptr<Packet> sectionPacket = payload->Copy ();
      CsrHelloHeader envelope;
      Require (sectionPacket->RemoveHeader (envelope) > 0,
               "snapshot section is missing its HOP envelope");

      std::vector<uint8_t> bytes (sectionPacket->GetSize ());
      sectionPacket->CopyData (bytes.data (), bytes.size ());

      CsrArlRoutingMessage::Section section;
      std::string error;
      Require (CsrArlRoutingMessage::DecodeSection (
                 bytes,
                 section,
                 &error),
               "snapshot section did not contain a routes.c prefix");

      stream.insert (
        stream.end (),
        section.body.begin (),
        section.body.end ());
    }

  return stream;
}

std::vector<uint8_t>
ExtractSectionBytes (Ptr<Packet> payload)
{
  Ptr<Packet> sectionPacket = payload->Copy ();
  CsrHelloHeader envelope;
  Require (sectionPacket->RemoveHeader (envelope) > 0,
           "capability-change payload is missing its HOP envelope");

  std::vector<uint8_t> bytes (sectionPacket->GetSize ());
  sectionPacket->CopyData (bytes.data (), bytes.size ());
  return bytes;
}

void
CheckLocalChangeProducer ()
{
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  nwk->SetNodeId (LOCAL_NODE);

  Require (!nwk->HasPendingLocalSelfCapabilityChangeForTest (),
           "routesCreate unexpectedly queued its capability-zero self route");

  nwk->SetNodeType (CsrNodeType::Routable);
  Require (nwk->HasPendingLocalSelfCapabilityChangeForTest (),
           "initial routesSetCapability(1) did not queue the self route");

  Ptr<Packet> update =
    nwk->BuildLocalSelfCapabilityChangePayloadForTest (0x11223344);
  Require (update != nullptr,
           "source-owned self UPDATE producer returned no payload");

  const std::array<uint8_t, 17> expectedUpdate {
    0x11, 0x22, 0x33, 0x44, 0x00, 0x01,
    0x02, 0x01, 0x02, 0x03, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  Require (ExtractSectionBytes (update) ==
             std::vector<uint8_t> (
               expectedUpdate.begin (),
               expectedUpdate.end ()),
           "automatic producer self UPDATE bytes differ from routes.c");

  Simulator::Run ();
  Require (!nwk->HasPendingLocalSelfCapabilityChangeForTest (),
           "consolidated self UPDATE did not drain its changed state");

  nwk->SetNodeType (CsrNodeType::Routable);
  Require (!nwk->HasPendingLocalSelfCapabilityChangeForTest (),
           "identical routesSetCapability value queued another UPDATE");

  nwk->SetNodeType (CsrNodeType::Ordinary);
  Require (nwk->HasPendingLocalSelfCapabilityChangeForTest (),
           "routesSetCapability(0) did not queue the self DELETE");

  Ptr<Packet> deletion =
    nwk->BuildLocalSelfCapabilityChangePayloadForTest (0x55667788);
  Require (deletion != nullptr,
           "source-owned self DELETE producer returned no payload");

  const std::array<uint8_t, 10> expectedDelete {
    0x55, 0x66, 0x77, 0x88, 0x00, 0x01,
    0x01, 0x01, 0x02, 0x03
  };
  Require (ExtractSectionBytes (deletion) ==
             std::vector<uint8_t> (
               expectedDelete.begin (),
               expectedDelete.end ()),
           "automatic producer self DELETE bytes differ from routes.c");

  Simulator::Run ();
  nwk->SetNodeType (CsrNodeType::Ordinary);
  Require (!nwk->HasPendingLocalSelfCapabilityChangeForTest (),
           "identical capability zero queued another DELETE");

  Simulator::Destroy ();
}

void
CheckLocalSnapshot (CsrNodeType nodeType,
                    bool expectSelfUpdate)
{
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  nwk->SetNodeId (LOCAL_NODE);
  nwk->SetNodeType (nodeType);

  // Initial routesSetCapability() is consumed by the same-time routesProcess
  // pass even with no active neighbors.  A later neighbor snapshot then
  // carries the stable source-owned self route.
  Simulator::Run ();

  uint32_t localCost = 0;
  Require (!nwk->GetSelectedRouteCost (LOCAL_NODE, localCost),
           "source-owned self state leaked into DATA forwarding");

  std::vector<uint8_t> stream =
    ExtractRecordStream (
      nwk->BuildArlRoutingSnapshotPayloadsForTest (0x12345678));

  std::vector<CsrArlRoutingMessage::Record> records;
  std::string error;
  Require (CsrArlRoutingMessage::ParseRecordStream (
             stream,
             records,
             &error),
           "local snapshot record stream did not parse");

  if (!expectSelfUpdate)
    {
      Require (records.size () == 2 &&
                 records[0].operation == CsrRoutingOperation::Info &&
                 records[1].operation == CsrRoutingOperation::Flush,
               "ordinary local capability was not omitted from snapshot");
      Simulator::Destroy ();
      return;
    }

  Require (records.size () == 3 &&
             records[0].operation == CsrRoutingOperation::Info &&
             records[1].operation == CsrRoutingOperation::Update &&
             records[2].operation == CsrRoutingOperation::Flush,
           "snapshot did not preserve INFO/self-UPDATE/FLUSH ordering");

  const auto &self = records[1];
  Require (self.nodeId == LOCAL_NODE &&
             self.capability == static_cast<uint8_t> (nodeType) &&
             self.hopCount == 0 &&
             self.cost == 0 &&
             self.path.empty (),
           "source-owned self UPDATE fields differ from routes.c");

  constexpr std::size_t INFO_RECORD_SIZE = 17;
  const std::array<uint8_t, 11> expected {
    0x02,
    0x01, 0x02, 0x03,
    static_cast<uint8_t> (nodeType),
    0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };

  Require (stream.size () >= INFO_RECORD_SIZE + expected.size (),
           "snapshot is too short to contain the self UPDATE");
  Require (std::equal (
             expected.begin (),
             expected.end (),
             stream.begin () + INFO_RECORD_SIZE),
           "source-owned self UPDATE bytes changed");

  nwk->ClearRoutes ();
  std::vector<uint8_t> afterClear =
    ExtractRecordStream (
      nwk->BuildArlRoutingSnapshotPayloadsForTest (0x12345679));
  Require (CsrArlRoutingMessage::ParseRecordStream (
             afterClear,
             records,
             &error) &&
             records.size () == 3 &&
             records[1].operation == CsrRoutingOperation::Update &&
             records[1].nodeId == LOCAL_NODE &&
             records[1].capability == static_cast<uint8_t> (nodeType),
           "routesClearTable changed source-owned self capability state");

  Simulator::Destroy ();
}

void
RunReceiverCapabilityScenario ()
{
  Ptr<CsrNetLayer> receiver = CreateObject<CsrNetLayer> ();
  receiver->SetNodeId (RECEIVER_NODE);
  receiver->SetNodeType (CsrNodeType::Routable);
  receiver->SetArlNeighborAdmissionEnabled (false);

  receiver->ProcessHello (
    MakePlainHello (
      REPORTER_NODE,
      CsrNodeType::Gateway),
    REPORTER_NODE,
    70.0,
    20.0);

  uint32_t directCost = 0;
  uint8_t capability = 0xff;
  Require (receiver->GetSelectedRouteCost (
             REPORTER_NODE,
             directCost),
           "plain HELLO did not establish direct reachability");
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 0,
           "HELLO NodeType incorrectly bootstrapped routing capability");

  DeliverSections (
    receiver,
    CsrNodeType::Gateway,
    BuildSelfMessage (10, 1, false));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 1,
           "zero-hop self UPDATE did not establish routing capability");

  // Wrapper role metadata remains independent.  A later plain HELLO must
  // refresh link measurements without overwriting routes.c capability state.
  receiver->ProcessHello (
    MakePlainHello (
      REPORTER_NODE,
      CsrNodeType::Ordinary,
      3),
    REPORTER_NODE,
    72.0,
    18.0);
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 1,
           "plain HELLO overwrote source-owned routing capability");

  DeliverSections (
    receiver,
    CsrNodeType::Gateway,
    BuildSelfMessage (11, 2, false));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 2,
           "newer self UPDATE did not change routing capability");

  DeliverSections (
    receiver,
    CsrNodeType::Routable,
    BuildSelfMessage (11, 1, false));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 2,
           "duplicate self UPDATE changed routing capability");

  DeliverSections (
    receiver,
    CsrNodeType::Routable,
    BuildSelfMessage (10, 1, false));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 2,
           "older self UPDATE changed routing capability");

  receiver->ClearRoutes ();
  receiver->ProcessHello (
    MakePlainHello (
      REPORTER_NODE,
      CsrNodeType::Gateway,
      4),
    REPORTER_NODE,
    71.0,
    19.0);
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 0,
           "plain HELLO resurrected capability after routesClearTable");

  DeliverSections (
    receiver,
    CsrNodeType::Gateway,
    BuildSelfMessage (1, 2, false));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 2,
           "fresh self UPDATE did not restore capability after clear");

  receiver->ClearRoutes ();
  receiver->ProcessHello (
    MakePlainHello (
      REPORTER_NODE,
      CsrNodeType::Routable,
      6),
    REPORTER_NODE,
    71.0,
    19.0);
  receiver->ProcessHello (
    MakeCompatibilitySelfUpdate (
      REPORTER_NODE,
      CsrNodeType::Routable,
      1),
    REPORTER_NODE,
    71.0,
    19.0);
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 1,
           "compatibility self UPDATE retained a pre-clear sequence gate");

  DeliverSections (
    receiver,
    CsrNodeType::Ordinary,
    BuildOrdinarySnapshot (12));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 0,
           "ordinary snapshot omission did not clear old capability");

  DeliverSections (
    receiver,
    CsrNodeType::Gateway,
    BuildSelfMessage (13, 2, false));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 2,
           "capability did not recover after ordinary snapshot");

  DeliverSections (
    receiver,
    CsrNodeType::Ordinary,
    BuildSelfMessage (14, 0, true));
  Require (receiver->GetSelectedRouteCapability (
             REPORTER_NODE,
             capability) &&
             capability == 0,
           "self DELETE did not clear routing capability");

  uint32_t costAfterDelete = 0;
  Require (receiver->GetSelectedRouteCost (
             REPORTER_NODE,
             costAfterDelete) &&
             costAfterDelete > 0,
           "self DELETE incorrectly removed direct reachability");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  CheckLocalChangeProducer ();
  CheckLocalSnapshot (CsrNodeType::Ordinary, false);
  CheckLocalSnapshot (CsrNodeType::Routable, true);
  CheckLocalSnapshot (CsrNodeType::Gateway, true);
  RunReceiverCapabilityScenario ();

  std::cout << "PASS: source-owned self-route capability parity test"
            << std::endl;
  return 0;
}
