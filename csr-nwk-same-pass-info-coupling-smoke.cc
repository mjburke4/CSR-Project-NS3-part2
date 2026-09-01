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
  bytes.push_back (static_cast<uint8_t> (value >> 8));
  bytes.push_back (static_cast<uint8_t> (value));
}

void
AppendU24 (std::vector<uint8_t> &bytes, CsrNodeId value)
{
  bytes.push_back (static_cast<uint8_t> (value >> 16));
  bytes.push_back (static_cast<uint8_t> (value >> 8));
  bytes.push_back (static_cast<uint8_t> (value));
}

void
AppendU32 (std::vector<uint8_t> &bytes, uint32_t value)
{
  bytes.push_back (static_cast<uint8_t> (value >> 24));
  bytes.push_back (static_cast<uint8_t> (value >> 16));
  bytes.push_back (static_cast<uint8_t> (value >> 8));
  bytes.push_back (static_cast<uint8_t> (value));
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
AppendInfo (std::vector<uint8_t> &bytes,
            uint16_t minSpeedKbps,
            uint16_t maxSpeedKbps,
            int16_t minPowerDbmX10,
            int16_t maxPowerDbmX10,
            int16_t linkMarginDbX10,
            int16_t lowPowerDbmX10,
            int16_t tempLowCx10,
            int16_t tempHighCx10)
{
  bytes.push_back (4);
  AppendU16 (bytes, minSpeedKbps);
  AppendU16 (bytes, maxSpeedKbps);
  AppendU16 (bytes, static_cast<uint16_t> (minPowerDbmX10));
  AppendU16 (bytes, static_cast<uint16_t> (maxPowerDbmX10));
  AppendU16 (bytes, static_cast<uint16_t> (linkMarginDbX10));
  AppendU16 (bytes, static_cast<uint16_t> (lowPowerDbmX10));
  AppendU16 (bytes, static_cast<uint16_t> (tempLowCx10));
  AppendU16 (bytes, static_cast<uint16_t> (tempHighCx10));
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

void
DrainSameTime ()
{
  Simulator::Schedule (NanoSeconds (1), [] {
    Simulator::Stop ();
  });
  Simulator::Run ();
}

void
ConfigureFinalInfo (const Harness &harness)
{
  harness.nwk->ConfigureLinkControl (
    16,
    128,
    -17.55,
    30.09,
    9.59);
  harness.nwk->SetTemperatureLimitsCx10 (-450, 625);
}

void
AppendFinalInfo (std::vector<uint8_t> &bytes)
{
  AppendInfo (bytes,
              16,
              128,
              -175,
              300,
              95,
              140,
              -450,
              625);
}

void
TestInfoFirstMixedGroupingAndSequence ()
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
  Simulator::Run ();
  g_observations.clear ();

  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  Simulator::ScheduleNow ([&harness] {
    // Exercise final-state coalescing as well as mixed INFO/UPDATE/DELETE.
    harness.nwk->ConfigureLinkControl (8, 64, -10.0, 20.0, 5.0);
    harness.nwk->SetTemperatureLimitsCx10 (-100, 300);
    ConfigureFinalInfo (harness);
    AddRoute (harness, ROUTE_A, 9, 2, 10, 1, {9, ROUTE_A});
    harness.nwk->SetNodeType (CsrNodeType::Routable);
    harness.nwk->SetNodeType (CsrNodeType::Ordinary);
  });
  DrainSameTime ();

  Require (g_observations.size () == 2,
           "same-cycle INFO/UPDATE/DELETE did not form a 10+1 fanout");
  Require (g_observations[0].targets == std::vector<CsrNodeId> ({9}),
           "final partial recipient group was not handed to HOP first");
  Require (g_observations[1].targets == newestTen,
           "full recipient group lost newest-created-first ordering");
  Require (g_observations[0].routingSequence == 0 &&
             g_observations[1].routingSequence == 0 &&
             g_observations[0].section == 0 &&
             g_observations[1].section == 0 &&
             g_observations[0].totalSections == 1 &&
             g_observations[1].totalSections == 1,
           "grouped same-pass stream has wrong routing metadata");
  Require (g_observations[0].bytes == g_observations[1].bytes,
           "recipient groups did not reuse byte-identical route sections");

  std::vector<uint8_t> expected;
  AppendSectionPrefix (expected, 0, 0, 1);
  AppendFinalInfo (expected);
  AppendUpdate (expected, ROUTE_A, 1, 2, 11, {9, ROUTE_A});
  AppendDelete (expected, LOCAL_NODE);
  Require (g_observations[0].bytes == expected,
           "INFO-first mixed record bytes/order differ from routesProcess");
  Require (harness.nwk->m_routingSequence == 2,
           "one shared stream did not advance once per recipient group");

  Simulator::Destroy ();
}

void
TestInfoOnlyAndNoOpCoalescing ()
{
  g_observations.clear ();
  Harness harness = MakeHarness ();
  AddRecipients (harness, {2});
  Simulator::Run ();
  g_observations.clear ();

  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  Simulator::ScheduleNow ([&harness] {
    harness.nwk->ConfigureLinkControl (8, 64, -10.0, 20.0, 5.0);
    harness.nwk->SetTemperatureLimitsCx10 (-100, 300);
    ConfigureFinalInfo (harness);
  });
  DrainSameTime ();

  Require (g_observations.size () == 1,
           "coalesced INFO-only mutation did not produce one transaction");
  Require (g_observations[0].targets == std::vector<CsrNodeId> ({2}) &&
             g_observations[0].routingSequence == 0 &&
             g_observations[0].section == 0 &&
             g_observations[0].totalSections == 1,
           "INFO-only transaction has wrong target or metadata");

  std::vector<uint8_t> expected;
  AppendSectionPrefix (expected, 0, 0, 1);
  AppendFinalInfo (expected);
  Require (g_observations[0].bytes == expected,
           "INFO-only transaction contains wrong fields or extra records");

  std::size_t observationCount = g_observations.size ();
  uint32_t nextSequence = harness.nwk->m_routingSequence;
  // Recovered source establishes INFO dirty coalescing but does not expose
  // the byte-identical notification branch.  The bounded ns-3 mapping treats
  // an unchanged serialized INFO value as a no-op; keep that choice explicit.
  Simulator::ScheduleNow ([&harness] {
    ConfigureFinalInfo (harness);
    ConfigureFinalInfo (harness);
  });
  DrainSameTime ();

  Require (g_observations.size () == observationCount,
           "unchanged INFO setters emitted another transaction");
  Require (harness.nwk->m_routingSequence == nextSequence,
           "unchanged INFO setters advanced the routing sequence");

  Simulator::Destroy ();
}

void
TestNoRecipientConsumesInfoWithoutReplay ()
{
  g_observations.clear ();
  Harness harness = MakeHarness ();
  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  ConfigureFinalInfo (harness);
  DrainSameTime ();

  Require (g_observations.empty (),
           "INFO mutation with no active recipient was transmitted");
  Require (harness.nwk->m_routingSequence == 0,
           "zero-recipient INFO mutation advanced the routing sequence");

  harness.nwk->SetAutomaticRoutePropagationEnabled (false);
  AddRecipients (harness, {2});
  Simulator::Run ();
  g_observations.clear ();

  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  harness.nwk->SetNodeType (CsrNodeType::Routable);
  DrainSameTime ();

  Require (g_observations.size () == 1,
           "later self-route change did not produce one transaction");
  std::vector<uint8_t> expected;
  AppendSectionPrefix (expected, 0, 0, 1);
  AppendUpdate (expected, LOCAL_NODE, 1, 0, 0, {});
  Require (g_observations[0].bytes == expected,
           "consumed zero-recipient INFO was replayed to a later neighbor");

  Simulator::Destroy ();
}

void
TestInfoShiftsExactSectionBoundary ()
{
  g_observations.clear ();
  Harness harness = MakeHarness ();
  AddRecipients (harness, {2});

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
    ConfigureFinalInfo (harness);
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
  DrainSameTime ();

  Require (g_observations.size () == 2,
           "777-byte INFO/change stream did not form two sections");
  Require (g_observations[0].section == 1 &&
             g_observations[1].section == 0 &&
             g_observations[0].totalSections == 2 &&
             g_observations[1].totalSections == 2 &&
             g_observations[0].routingSequence == 0 &&
             g_observations[1].routingSequence == 0,
           "two-section INFO stream lost legacy LIFO handoff metadata");

  std::vector<uint8_t> expectedStream;
  AppendFinalInfo (expectedStream);
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
  Require (expectedStream.size () == 777,
           "manual INFO boundary oracle has unexpected length");

  std::vector<uint8_t> expectedSection0;
  AppendSectionPrefix (expectedSection0, 0, 0, 2);
  expectedSection0.insert (expectedSection0.end (),
                           expectedStream.begin (),
                           expectedStream.begin () +
                             CsrArlRoutingMessage::MAX_SECTION_BODY_SIZE);
  std::vector<uint8_t> expectedSection1;
  AppendSectionPrefix (expectedSection1, 0, 1, 2);
  expectedSection1.insert (expectedSection1.end (),
                           expectedStream.begin () +
                             CsrArlRoutingMessage::MAX_SECTION_BODY_SIZE,
                           expectedStream.end ());

  Require (g_observations[1].bytes == expectedSection0 &&
             g_observations[0].bytes == expectedSection1,
           "INFO-first stream changed exact 694-byte section slicing");
  Require (g_observations[1].bytes.size () == 700 &&
             g_observations[0].bytes.size () == 89,
           "section prefix/body sizes differ from 700-byte source limit");
  Require (harness.nwk->m_routingSequence == 1,
           "multi-section stream advanced per section instead of group");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  TestInfoFirstMixedGroupingAndSequence ();
  TestInfoOnlyAndNoOpCoalescing ();
  TestNoRecipientConsumesInfoWithoutReplay ();
  TestInfoShiftsExactSectionBoundary ();

  std::cout << "PASS: same-pass routing INFO coupling"
            << std::endl;
  return 0;
}
