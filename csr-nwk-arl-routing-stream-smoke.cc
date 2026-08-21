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

void
Require (bool condition, const char *message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

CsrHelloHeader::RoutingInfo
MakeRoutingInfo ()
{
  CsrHelloHeader::RoutingInfo info;
  info.minSpeedKbps = 8;
  info.maxSpeedKbps = 128;
  info.minPowerDbmX10 = -175;
  info.maxPowerDbmX10 = 300;
  info.linkMarginDbX10 = 95;
  info.lowPowerDbmX10 = 140;
  info.tempLowCx10 = -450;
  info.tempHighCx10 = 625;
  return info;
}

Ptr<Packet>
MakePlainHello (uint16_t source)
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
MakeSectionEnvelope (uint16_t source,
                     const std::vector<uint8_t> &sectionBytes)
{
  CsrArlRoutingMessage::Section section;
  std::string error;
  Require (CsrArlRoutingMessage::DecodeSection (
             sectionBytes,
             section,
             &error),
           "test could not decode its own ARL section");

  Ptr<Packet> packet = Create<Packet> (
    sectionBytes.data (),
    sectionBytes.size ());

  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (
    static_cast<uint16_t> (section.section + 2));
  hello.SetNodeType (CsrNodeType::Routable);
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
BuildCrossBoundaryMessage (uint32_t sequence)
{
  CsrArlRoutingMessage::Builder builder;
  builder.AddInfo (MakeRoutingInfo ());

  // INFO is 17 bytes and each two-hop UPDATE is 17 bytes.  Forty
  // UPDATEs plus FLUSH total 698 record bytes, forcing the final UPDATE to
  // cross the 694-byte section-body boundary exactly as routes.c permits.
  for (uint32_t index = 0; index < 40; ++index)
    {
      uint32_t destination = 1000 + index;
      std::string error;
      Require (builder.AddUpdate (
                 destination,
                 1,
                 2,
                 100 + index,
                 {3, destination},
                 &error),
               "failed to add test ARL UPDATE");
    }

  builder.AddFlush ();

  std::vector<std::vector<uint8_t>> sections;
  std::string error;
  Require (builder.BuildSections (sequence, sections, &error),
           "failed to split test ARL routing message");
  return sections;
}

void
RunCodecScenario ()
{
  CsrArlRoutingMessage::Builder builder;
  builder.AddInfo (MakeRoutingInfo ());

  std::string error;
  Require (builder.AddUpdate (
             0x0abcde,
             2,
             2,
             0x89abcdef,
             {0x123456, 0x000042},
             &error),
           "24-bit UPDATE could not be encoded");
  Require (builder.AddDelete (0x654321, &error),
           "24-bit DELETE could not be encoded");
  builder.AddFlush ();

  std::vector<std::vector<uint8_t>> sections;
  Require (builder.BuildSections (0x12345678, sections, &error),
           "single-section ARL message could not be built");
  Require (sections.size () == 1,
           "small ARL message unexpectedly used multiple sections");
  Require (sections[0][0] == 0x12 &&
             sections[0][1] == 0x34 &&
             sections[0][2] == 0x56 &&
             sections[0][3] == 0x78 &&
             sections[0][4] == 0 &&
             sections[0][5] == 1,
           "six-byte ARL section prefix is not legacy big-endian layout");

  CsrArlRoutingMessage::Section decoded;
  Require (CsrArlRoutingMessage::DecodeSection (
             sections[0], decoded, &error),
           "single ARL section could not be decoded");

  std::vector<CsrArlRoutingMessage::Record> records;
  Require (CsrArlRoutingMessage::ParseRecordStream (
             decoded.body, records, &error),
           "ARL record stream could not be parsed");
  Require (records.size () == 4,
           "ARL parser returned the wrong record count");
  Require (records[1].nodeId == 0x0abcde &&
             records[1].hopCount == 2 &&
             records[1].cost == 0x89abcdef &&
             records[1].path.size () == 2 &&
             records[1].path[0] == 0x123456 &&
             records[1].path[1] == 0x000042,
           "ARL parser did not preserve 24/16/32-bit UPDATE fields");
  Require (records[2].nodeId == 0x654321,
           "ARL parser did not preserve a 24-bit DELETE target");

  // Use a value above 255 so a one-byte hop-count implementation cannot
  // accidentally satisfy the codec test.
  std::vector<uint32_t> widePath (0x0102);
  for (uint32_t index = 0; index < widePath.size (); ++index)
    {
      widePath[index] = index + 1;
    }

  CsrArlRoutingMessage::Builder wideBuilder;
  Require (wideBuilder.AddUpdate (
             0x010203,
             3,
             static_cast<uint16_t> (widePath.size ()),
             0xfedcba98,
             widePath,
             &error),
           "16-bit-hop UPDATE could not be encoded");

  std::vector<std::vector<uint8_t>> wideSections;
  Require (wideBuilder.BuildSections (9, wideSections, &error),
           "16-bit-hop UPDATE could not be sectioned");

  std::vector<uint8_t> wideStream;
  for (const auto &wireSection : wideSections)
    {
      Require (CsrArlRoutingMessage::DecodeSection (
                 wireSection, decoded, &error),
               "16-bit-hop section could not be decoded");
      wideStream.insert (wideStream.end (),
                         decoded.body.begin (),
                         decoded.body.end ());
    }

  Require (CsrArlRoutingMessage::ParseRecordStream (
             wideStream, records, &error),
           "16-bit-hop UPDATE could not be parsed");
  Require (records.size () == 1 &&
             records[0].nodeId == 0x010203 &&
             records[0].hopCount == 0x0102 &&
             records[0].cost == 0xfedcba98 &&
             records[0].path == widePath,
           "ARL codec truncated a legacy 24/16/32-bit wire field");

  auto crossed = BuildCrossBoundaryMessage (77);
  Require (crossed.size () == 2,
           "legacy 700-byte limit did not split the record stream");
  Require (crossed[0].size () ==
             CsrArlRoutingMessage::MAX_SECTION_SIZE,
           "first ARL section is not exactly 700 bytes");
  Require (crossed[1].size () == 10,
           "cross-boundary tail section has the wrong size");

  std::vector<uint8_t> reassembled;
  for (const auto &wireSection : crossed)
    {
      Require (CsrArlRoutingMessage::DecodeSection (
                 wireSection, decoded, &error),
               "split ARL section could not be decoded");
      reassembled.insert (
        reassembled.end (),
        decoded.body.begin (),
        decoded.body.end ());
    }

  Require (CsrArlRoutingMessage::ParseRecordStream (
             reassembled, records, &error),
           "record crossing a section boundary did not parse");
  Require (records.size () == 42 &&
             records.front ().operation == CsrRoutingOperation::Info &&
             records.back ().operation == CsrRoutingOperation::Flush &&
             records[40].nodeId == 1039,
           "cross-boundary INFO/UPDATE/FLUSH ordering changed");
}

void
RunOutOfOrderAtomicReassemblyScenario ()
{
  constexpr uint16_t receiverId = 1;
  constexpr uint16_t senderId = 2;
  constexpr uint16_t oldDestination = 900;

  Ptr<CsrNetLayer> receiver = CreateObject<CsrNetLayer> ();
  receiver->SetNodeId (receiverId);
  receiver->ProcessHello (
    MakePlainHello (senderId),
    senderId,
    70.0,
    20.0);

  receiver->AddOrUpdateRoute (
    oldDestination,
    senderId,
    false,
    2,
    70.0,
    29,
    5,
    senderId,
    1,
    {senderId, oldDestination});

  auto sections = BuildCrossBoundaryMessage (77);
  auto newerSections = BuildCrossBoundaryMessage (78);
  Require (sections.size () == 2,
           "atomic reassembly scenario requires two sections");

  // Deliver the tail first.  INFO, UPDATE, and FLUSH must remain unapplied.
  receiver->ProcessHello (
    MakeSectionEnvelope (senderId, sections[1]),
    senderId,
    70.0,
    20.0);

  uint32_t cost = 0;
  CsrHelloHeader::RoutingInfo receivedInfo;
  Require (receiver->GetPendingArlRoutingMessageCount (senderId) == 1,
           "out-of-order ARL section was not buffered");
  Require (!receiver->GetSelectedRouteCost (1000, cost),
           "partial ARL message changed the live route table");
  Require (receiver->GetSelectedRouteCost (oldDestination, cost),
           "partial ARL FLUSH invalidated an old route");
  Require (!receiver->GetNeighborRoutingInfo (senderId, receivedInfo),
           "partial ARL INFO changed neighbor state");

  // Interleave another routing sequence from the same neighbor.  Each
  // logical message must retain an independent section map.
  receiver->ProcessHello (
    MakeSectionEnvelope (senderId, newerSections[1]),
    senderId,
    70.0,
    20.0);
  Require (receiver->GetPendingArlRoutingMessageCount (senderId) == 2,
           "interleaved ARL sequences shared reassembly state");
  Require (!receiver->GetSelectedRouteCost (1000, cost),
           "interleaved partial ARL message changed the route table");

  // A duplicate reliable delivery is suppressed by HOP in normal operation;
  // direct injection here verifies it cannot poison the section ordering.
  receiver->ProcessHello (
    MakeSectionEnvelope (senderId, sections[1]),
    senderId,
    70.0,
    20.0);
  Require (receiver->GetPendingArlRoutingMessageCount (senderId) == 2,
           "duplicate ARL section created another transaction");

  receiver->ProcessHello (
    MakeSectionEnvelope (senderId, sections[0]),
    senderId,
    70.0,
    20.0);

  Require (receiver->GetPendingArlRoutingMessageCount (senderId) == 1,
           "completing one ARL sequence removed another sequence");
  Require (receiver->GetSelectedRouteCost (1000, cost) && cost == 129,
           "complete ARL UPDATE did not install the expected route cost");
  Require (!receiver->GetSelectedRouteCost (oldDestination, cost),
           "complete ARL FLUSH did not invalidate an omitted old route");
  Require (receiver->GetNeighborRoutingInfo (senderId, receivedInfo),
           "complete ARL INFO was not stored");
  Require (receivedInfo.minSpeedKbps == 8 &&
             receivedInfo.maxSpeedKbps == 128 &&
             receivedInfo.tempLowCx10 == -450 &&
             receivedInfo.tempHighCx10 == 625,
           "complete ARL INFO fields changed during reassembly");

  receiver->ProcessHello (
    MakeSectionEnvelope (senderId, newerSections[0]),
    senderId,
    70.0,
    20.0);
  Require (receiver->GetPendingArlRoutingMessageCount (senderId) == 0,
           "complete interleaved ARL messages left reassembly state");
  Require (receiver->GetSelectedRouteCost (1000, cost) && cost == 129,
           "newer interleaved ARL message did not apply independently");

  Simulator::Destroy ();
}

void
RunIndependentHopTransmissionScenario ()
{
  constexpr uint16_t sourceId = 1;
  constexpr uint16_t neighborId = 2;

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (sourceId);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();

  hop->SetNodeId (sourceId);
  hop->SetMac (&device->GetMac ());
  nwk->SetNodeId (sourceId);
  nwk->SetHop (hop);
  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));

  nwk->ProcessHello (
    MakePlainHello (neighborId),
    neighborId,
    70.0,
    20.0);

  for (uint16_t index = 0; index < 40; ++index)
    {
      uint16_t destination = static_cast<uint16_t> (1000 + index);
      nwk->AddOrUpdateRoute (
        destination,
        neighborId,
        false,
        2,
        70.0,
        29,
        100 + index,
        neighborId,
        1,
        {neighborId, destination});
    }

  nwk->StartReliableRoutingSnapshot (neighborId);

  uint8_t totalSections =
    nwk->GetOutboundRoutingSnapshotTotalSections (neighborId);
  Require (totalSections == 2,
           "full NWK snapshot did not use the legacy byte-size split");
  Require (hop->GetResendQueueSize () == totalSections,
           "NWK phase-gated ARL sections instead of queueing all to HOP");
  Require (nwk->IsOutboundRoutingSnapshotActive (neighborId) &&
             nwk->GetOutboundRoutingSnapshotAckedSections (neighborId) == 0,
           "outbound ARL snapshot completion state started incorrectly");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  RunCodecScenario ();
  RunOutOfOrderAtomicReassemblyScenario ();
  RunIndependentHopTransmissionScenario ();

  std::cout << "PASS: ARL routing byte-stream parity test" << std::endl;
  return 0;
}
