#include "ns3/core-module.h"
#include "ns3/csr-arl-routing-message.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId LOCAL_NODE = 0x010203;
constexpr CsrNodeId ROUTE_A = 0x0a0b01;
constexpr CsrNodeId ROUTE_B = 0x0a0b02;
constexpr CsrNodeId ROUTE_C = 0x0a0b03;

struct Observation
{
  Time when;
  std::vector<CsrNodeId> targets;
  uint32_t routingSequence {0};
  uint8_t section {0};
  uint8_t totalSections {0};
  std::vector<uint8_t> bytes;
};

struct Harness
{
  Ptr<CsrNetDevice> device;
  Ptr<CsrHopLayer> hop;
  Ptr<CsrNetLayer> nwk;
};

std::vector<Observation> g_observations;

void
Require (bool condition, const std::string &message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

void
AppendU16 (std::vector<uint8_t> &bytes, uint16_t value)
{
  bytes.push_back (static_cast<uint8_t> ((value >> 8) & 0xff));
  bytes.push_back (static_cast<uint8_t> (value & 0xff));
}

void
AppendU24 (std::vector<uint8_t> &bytes, CsrNodeId value)
{
  bytes.push_back (static_cast<uint8_t> ((value >> 16) & 0xff));
  bytes.push_back (static_cast<uint8_t> ((value >> 8) & 0xff));
  bytes.push_back (static_cast<uint8_t> (value & 0xff));
}

void
AppendU32 (std::vector<uint8_t> &bytes, uint32_t value)
{
  bytes.push_back (static_cast<uint8_t> ((value >> 24) & 0xff));
  bytes.push_back (static_cast<uint8_t> ((value >> 16) & 0xff));
  bytes.push_back (static_cast<uint8_t> ((value >> 8) & 0xff));
  bytes.push_back (static_cast<uint8_t> (value & 0xff));
}

void
AppendSectionPrefix (std::vector<uint8_t> &bytes,
                     uint32_t sequence,
                     uint8_t section,
                     uint8_t totalSections)
{
  AppendU32 (bytes, sequence);
  bytes.push_back (section);
  bytes.push_back (totalSections);
}

void
AppendUpdate (std::vector<uint8_t> &bytes,
              CsrNodeId destination,
              uint8_t capability,
              uint16_t hops,
              uint32_t cost,
              const std::vector<CsrNodeId> &path)
{
  bytes.push_back (2);
  AppendU24 (bytes, destination);
  bytes.push_back (capability);
  AppendU16 (bytes, hops);
  AppendU32 (bytes, cost);
  for (CsrNodeId node : path)
    {
      AppendU24 (bytes, node);
    }
}

void
AppendDelete (std::vector<uint8_t> &bytes, CsrNodeId destination)
{
  bytes.push_back (1);
  AppendU24 (bytes, destination);
}

Ptr<Packet>
MakePlainHello (CsrNodeId source, uint16_t sequence)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (sequence);
  hello.SetNodeType (CsrNodeType::Ordinary);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::None);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  return packet;
}

Ptr<Packet>
MakeSectionEnvelope (CsrNodeId source,
                     const std::vector<uint8_t> &sectionBytes)
{
  CsrArlRoutingMessage::Section section;
  std::string error;
  Require (CsrArlRoutingMessage::DecodeSection (
             sectionBytes,
             section,
             &error),
           "test stimulus section failed to decode: " + error);

  Ptr<Packet> packet = Create<Packet> (
    sectionBytes.data (), sectionBytes.size ());
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (2);
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
RecordAutomaticControl (std::vector<CsrNodeId> targets,
                        Ptr<Packet> payload)
{
  CsrHelloHeader envelope;
  Require (payload->RemoveHeader (envelope) > 0,
           "automatic routing payload has no HOP envelope");

  Observation observation;
  observation.when = Simulator::Now ();
  observation.targets = std::move (targets);
  observation.routingSequence = envelope.GetRoutingSequence ();
  observation.section = envelope.GetRoutingSection ();
  observation.totalSections = envelope.GetRoutingTotalSections ();
  observation.bytes.resize (payload->GetSize ());
  payload->CopyData (observation.bytes.data (), observation.bytes.size ());
  g_observations.push_back (std::move (observation));
}

Harness
MakeHarness ()
{
  Harness harness;
  harness.device = CreateObject<CsrNetDevice> (LOCAL_NODE);
  harness.hop = CreateObject<CsrHopLayer> ();
  harness.nwk = CreateObject<CsrNetLayer> ();

  harness.hop->SetNodeId (LOCAL_NODE);
  harness.hop->SetMac (&harness.device->GetMac ());
  harness.nwk->SetNodeId (LOCAL_NODE);
  harness.nwk->SetNodeType (CsrNodeType::Ordinary);
  harness.nwk->SetArlNeighborAdmissionEnabled (false);
  harness.nwk->SetAutomaticRoutePropagationEnabled (false);
  harness.nwk->SetHop (harness.hop);
  harness.nwk->SetAutomaticRoutePropagationEnabled (false);
  harness.nwk->SetAutomaticRoutingControlObserverForTest (
    MakeCallback (&RecordAutomaticControl));
  return harness;
}

void
AddRecipients (const Harness &harness,
               const std::vector<CsrNodeId> &creationOrder)
{
  uint16_t sequence = 1;
  for (CsrNodeId node : creationOrder)
    {
      harness.nwk->ProcessHello (
        MakePlainHello (node, sequence++),
        node,
        70.0,
        20.0);
    }
}

void
AddRoute (const Harness &harness,
          CsrNodeId destination,
          CsrNodeId nextHop,
          uint8_t hops,
          uint32_t linkCost,
          uint32_t advertisedCost,
          const std::vector<CsrNodeId> &path)
{
  Require (harness.nwk->AddOrUpdateRoute (
             destination,
             nextHop,
             false,
             hops,
             70.0,
             linkCost,
             advertisedCost,
             nextHop,
             1,
             path),
           "test route update was rejected");
}

std::vector<uint8_t>
ExtractRecordStream (const std::vector<Ptr<Packet>> &payloads)
{
  std::vector<uint8_t> recordStream;
  for (const Ptr<Packet> &payload : payloads)
    {
      Ptr<Packet> copy = payload->Copy ();
      CsrHelloHeader envelope;
      Require (copy->RemoveHeader (envelope) > 0,
               "snapshot payload has no envelope");
      std::vector<uint8_t> bytes (copy->GetSize ());
      copy->CopyData (bytes.data (), bytes.size ());

      CsrArlRoutingMessage::Section section;
      std::string error;
      Require (CsrArlRoutingMessage::DecodeSection (
                 bytes,
                 section,
                 &error),
               "snapshot section failed to decode: " + error);
      recordStream.insert (recordStream.end (),
                           section.body.begin (),
                           section.body.end ());
    }
  return recordStream;
}

void
TestSameCycleGoldenGroupingAndSequenceGap ()
{
  g_observations.clear ();
  Harness harness = MakeHarness ();

  const std::vector<CsrNodeId> recipientCreation {
    9, 3, 12, 5, 7, 2, 10, 4, 11, 6, 8
  };
  const std::vector<CsrNodeId> newestTen {
    8, 6, 11, 4, 10, 2, 7, 5, 12, 3
  };
  AddRecipients (harness, recipientCreation);

  AddRoute (harness, ROUTE_A, 9, 2, 1, 1, {9, ROUTE_A});
  AddRoute (harness, ROUTE_B, 9, 2, 1, 1, {9, ROUTE_B});
  AddRoute (harness, ROUTE_C, 9, 2, 1, 1, {9, ROUTE_C});
  Simulator::Run ();
  g_observations.clear ();

  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  std::vector<uint8_t> sameCycleSnapshot;

  Simulator::ScheduleNow ([&harness, &sameCycleSnapshot] {
    AddRoute (harness, ROUTE_A, 9, 2, 10, 1, {9, ROUTE_A});
    AddRoute (harness, ROUTE_C, 9, 2, 30, 3, {9, ROUTE_C});
    AddRoute (harness, ROUTE_B, 9, 2, 20, 2, {9, ROUTE_B});
    harness.nwk->SetNodeType (CsrNodeType::Routable);

    // MarkSelectedRouteChanged queued routesProcess first.  This later
    // same-time snapshot must still see the frozen changed flags and omit
    // every changed destination.
    Simulator::ScheduleNow ([&harness, &sameCycleSnapshot] {
      sameCycleSnapshot = ExtractRecordStream (
        harness.nwk->BuildArlRoutingSnapshotPayloadsForTest (99));
    });
  });

  Simulator::Schedule (NanoSeconds (100), [&harness] {
    AddRoute (harness, ROUTE_A, 9, 2, 40, 1, {9, ROUTE_A});
    AddRoute (harness, ROUTE_A, 9, 2, 50, 1, {9, ROUTE_A});
    harness.nwk->SetNodeType (CsrNodeType::Ordinary);
  });

  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (g_observations.size () == 4,
           "two same-cycle batches did not produce two grouped fanouts");

  for (uint32_t index = 0; index < 2; ++index)
    {
      Require (g_observations[index].when == Seconds (0),
               "first grouped stream retained the old 25-ms delay");
      Require (g_observations[index].routingSequence == 0,
               "first grouped stream did not start at legacy sequence zero");
      Require (g_observations[index].section == 0 &&
                 g_observations[index].totalSections == 1,
               "single-section grouped stream has wrong metadata");
    }

  Require (g_observations[0].targets == std::vector<CsrNodeId> ({9}),
           "LIFO handoff did not submit the final partial group first");
  Require (g_observations[1].targets == newestTen,
           "full group did not preserve newest-created neighbor order");
  Require (g_observations[0].bytes == g_observations[1].bytes,
           "recipient groups did not reuse byte-identical route sections");

  std::vector<uint8_t> expectedFirst;
  AppendSectionPrefix (expectedFirst, 0, 0, 1);
  AppendUpdate (expectedFirst, ROUTE_C, 1, 2, 33, {9, ROUTE_C});
  AppendUpdate (expectedFirst, ROUTE_B, 1, 2, 22, {9, ROUTE_B});
  AppendUpdate (expectedFirst, ROUTE_A, 1, 2, 11, {9, ROUTE_A});
  AppendUpdate (expectedFirst, LOCAL_NODE, 1, 0, 0, {});
  Require (g_observations[0].bytes == expectedFirst,
           "combined stream bytes/order differ from routesProcess");

  Require (sameCycleSnapshot.size () == 18 &&
             sameCycleSnapshot.front () == 4 &&
             sameCycleSnapshot.back () == 0,
           "same-cycle snapshot duplicated a changed destination");

  for (uint32_t index = 2; index < 4; ++index)
    {
      Require (g_observations[index].when == NanoSeconds (100),
               "second grouped stream was not processed at its event time");
      Require (g_observations[index].routingSequence == 2,
               "next stream did not advance by recipient-group count");
    }
  Require (g_observations[2].targets == std::vector<CsrNodeId> ({9}) &&
             g_observations[3].targets == newestTen,
           "second batch changed physical group order");

  std::vector<uint8_t> expectedSecond;
  AppendSectionPrefix (expectedSecond, 2, 0, 1);
  AppendUpdate (expectedSecond, ROUTE_A, 1, 2, 51, {9, ROUTE_A});
  AppendDelete (expectedSecond, LOCAL_NODE);
  Require (g_observations[2].bytes == expectedSecond &&
             g_observations[3].bytes == expectedSecond,
           "final-state dedup or mixed UPDATE/DELETE bytes are wrong");

  Simulator::Destroy ();
}

void
TestSectionAndGroupCrossProduct ()
{
  g_observations.clear ();
  Harness harness = MakeHarness ();
  std::vector<CsrNodeId> recipients;
  for (CsrNodeId node = 2; node <= 12; ++node)
    {
      recipients.push_back (node);
    }
  AddRecipients (harness, recipients);

  std::vector<CsrNodeId> destinations;
  std::vector<std::vector<CsrNodeId>> paths;
  for (uint32_t index = 0; index < 7; ++index)
    {
      CsrNodeId destination = 0x0b0000 + index;
      destinations.push_back (destination);

      std::vector<CsrNodeId> path;
      for (uint32_t hop = 0; hop < 31; ++hop)
        {
          path.push_back (0x020000 + index * 32 + hop);
        }
      path.push_back (destination);
      paths.push_back (path);

      AddRoute (harness,
                destination,
                2,
                32,
                1,
                1,
                path);
    }
  Simulator::Run ();
  g_observations.clear ();

  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  Simulator::ScheduleNow ([&harness, &destinations, &paths] {
    for (uint32_t index = 0; index < destinations.size (); ++index)
      {
        AddRoute (harness,
                  destinations[index],
                  2,
                  32,
                  1,
                  99 + index,
                  paths[index]);
      }
    harness.nwk->SetNodeType (CsrNodeType::Routable);
  });

  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (g_observations.size () == 4,
           "two sections across two groups did not form four transactions");
  Require (g_observations[0].targets == std::vector<CsrNodeId> ({2}) &&
             g_observations[1].targets == std::vector<CsrNodeId> ({2}),
           "final partial group was not submitted first");
  Require (g_observations[2].targets ==
             std::vector<CsrNodeId> ({12, 11, 10, 9, 8, 7, 6, 5, 4, 3}) &&
             g_observations[3].targets == g_observations[2].targets,
           "full group order changed in multi-section fanout");

  const std::vector<uint8_t> expectedSectionOrder {1, 0, 1, 0};
  for (uint32_t index = 0; index < g_observations.size (); ++index)
    {
      Require (g_observations[index].routingSequence == 0 &&
                 g_observations[index].section == expectedSectionOrder[index] &&
                 g_observations[index].totalSections == 2,
               "LIFO section metadata/order differs from routesSendMessage");
    }
  Require (g_observations[0].bytes == g_observations[2].bytes &&
             g_observations[1].bytes == g_observations[3].bytes,
           "section bytes changed across recipient groups");

  std::vector<uint8_t> recordStream;
  for (uint32_t index : {1u, 0u})
    {
      CsrArlRoutingMessage::Section section;
      std::string error;
      Require (CsrArlRoutingMessage::DecodeSection (
                 g_observations[index].bytes,
                 section,
                 &error),
               "observed grouped section did not decode: " + error);
      recordStream.insert (recordStream.end (),
                           section.body.begin (),
                           section.body.end ());
    }

  std::vector<uint8_t> expectedStream;
  for (int32_t index = 6; index >= 0; --index)
    {
      AppendUpdate (expectedStream,
                    destinations[index],
                    1,
                    32,
                    100 + index,
                    paths[index]);
    }
  AppendUpdate (expectedStream, LOCAL_NODE, 1, 0, 0, {});

  Require (expectedStream.size () == 760,
           "manual multi-section oracle has unexpected size");
  Require (recordStream == expectedStream,
           "record crossing the 694-byte section boundary changed");

  Simulator::Destroy ();
}

void
TestNoRecipientConsumesChangeWithoutSequenceAdvance ()
{
  g_observations.clear ();
  Harness harness = MakeHarness ();
  harness.nwk->SetAutomaticRoutePropagationEnabled (true);

  AddRoute (harness, ROUTE_A, 2, 2, 1, 1, {2, ROUTE_A});
  Simulator::Run ();
  Require (g_observations.empty (),
           "route change with no active neighbor was transmitted");
  Require (harness.nwk->m_routingSequence == 0,
           "zero-recipient change advanced the routing sequence");

  harness.nwk->SetAutomaticRoutePropagationEnabled (false);
  AddRecipients (harness, {2});
  Simulator::Run ();
  g_observations.clear ();

  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  harness.nwk->SetNodeType (CsrNodeType::Routable);
  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (g_observations.size () == 1,
           "new self change did not produce one recipient transaction");
  std::vector<uint8_t> expected;
  AppendSectionPrefix (expected, 0, 0, 1);
  AppendUpdate (expected, LOCAL_NODE, 1, 0, 0, {});
  Require (g_observations[0].bytes == expected,
           "consumed no-recipient destination was replayed later");

  Simulator::Destroy ();
}

void
TestAuthenticatedDiscoverCreationOrder ()
{
  g_observations.clear ();

  Harness harness;
  harness.device = CreateObject<CsrNetDevice> (LOCAL_NODE);
  harness.hop = CreateObject<CsrHopLayer> ();
  harness.nwk = CreateObject<CsrNetLayer> ();
  harness.hop->SetNodeId (LOCAL_NODE);
  harness.hop->SetMac (&harness.device->GetMac ());
  harness.nwk->SetNodeId (LOCAL_NODE);
  harness.nwk->SetNodeType (CsrNodeType::Ordinary);
  harness.nwk->SetArlNeighborAdmissionEnabled (false);

  const std::vector<CsrNodeId> creationOrder {
    9, 3, 12, 5, 7, 2, 10, 4, 11, 6, 8
  };
  for (CsrNodeId neighbor : creationOrder)
    {
      harness.nwk->NoteAuthenticatedGroupKeyNeededForTest (neighbor);
    }

  // Later RF/HELLO activation must update these existing route-neighbor
  // records without moving them.  Use a different order so the assertion
  // detects any accidental first-HELLO ordering.
  uint16_t helloSequence = 1;
  for (CsrNodeId neighbor :
       std::vector<CsrNodeId> ({2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}))
    {
      harness.nwk->ProcessHello (
        MakePlainHello (neighbor, helloSequence++),
        neighbor,
        70.0,
        20.0);
    }
  Simulator::Run ();

  // Attach HOP only after the authenticated-Discover creation path so its
  // KeyRequest side effect cannot add unrelated reliable transactions.
  harness.nwk->SetHop (harness.hop);
  harness.nwk->SetAutomaticRoutingControlObserverForTest (
    MakeCallback (&RecordAutomaticControl));
  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  harness.nwk->SetNodeType (CsrNodeType::Routable);
  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (g_observations.size () == 2,
           "authenticated-Discover neighbors did not form 10+1 groups");
  Require (g_observations[0].targets == std::vector<CsrNodeId> ({9}),
           "authenticated-Discover partial group lost creation order");
  Require (g_observations[1].targets ==
             std::vector<CsrNodeId> ({8, 6, 11, 4, 10, 2, 7, 5, 12, 3}),
           "authenticated-Discover full group fell back to node-ID order");

  Simulator::Destroy ();
}

void
TestDeleteFirstDestinationCreationOrder ()
{
  g_observations.clear ();
  Harness harness = MakeHarness ();
  AddRecipients (harness, {2});

  constexpr CsrNodeId earlyDestination = 0x0c0001;
  constexpr CsrNodeId lateDestination = 0x0c0002;
  CsrArlRoutingMessage::Builder builder;
  std::string error;
  Require (builder.AddDelete (earlyDestination, &error),
           "failed to build unknown-destination DELETE: " + error);
  std::vector<std::vector<uint8_t>> sections;
  Require (builder.BuildSections (77, sections, &error) &&
             sections.size () == 1,
           "failed to section unknown-destination DELETE: " + error);
  harness.nwk->ProcessHello (
    MakeSectionEnvelope (2, sections.front ()),
    2,
    70.0,
    20.0);

  // The DELETE created earlyDestination first.  Adding lateDestination later
  // prepends it, while adding a valid candidate for the tombstoned destination
  // must preserve that destination's original creation position.
  AddRoute (harness,
            lateDestination,
            2,
            2,
            1,
            1,
            {2, lateDestination});
  AddRoute (harness,
            earlyDestination,
            2,
            2,
            1,
            1,
            {2, earlyDestination});
  Simulator::Run ();
  g_observations.clear ();

  std::vector<std::vector<uint8_t>> deleteSections;
  Require (builder.BuildSections (78, deleteSections, &error) &&
             deleteSections.size () == 1,
           "failed to rebuild destination DELETE: " + error);

  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  Simulator::ScheduleNow ([&harness, deleteSections] {
    harness.nwk->ProcessHello (
      MakeSectionEnvelope (2, deleteSections.front ()),
      2,
      70.0,
      20.0);
    AddRoute (harness,
              lateDestination,
              2,
              2,
              1,
              10,
              {2, lateDestination});
  });
  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (g_observations.size () == 1,
           "DELETE-created destinations did not produce one grouped stream");
  std::vector<uint8_t> expected;
  AppendSectionPrefix (expected, 0, 0, 1);
  AppendUpdate (expected,
                lateDestination,
                1,
                2,
                11,
                {2, lateDestination});
  AppendDelete (expected, earlyDestination);
  Require (g_observations[0].bytes == expected,
           "DELETE creation order or ordinary DELETE serialization changed");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  TestSameCycleGoldenGroupingAndSequenceGap ();
  TestSectionAndGroupCrossProduct ();
  TestNoRecipientConsumesChangeWithoutSequenceAdvance ();
  TestAuthenticatedDiscoverCreationOrder ();
  TestDeleteFirstDestinationCreationOrder ();

  std::cout << "PASS: source-exact grouped route propagation test"
            << std::endl;
  return 0;
}
