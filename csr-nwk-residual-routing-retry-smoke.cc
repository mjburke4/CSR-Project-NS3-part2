#include "ns3/core-module.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId LOCAL_NODE = 1;
constexpr CsrNodeId NEIGHBOR_2 = 2;
constexpr CsrNodeId NEIGHBOR_3 = 3;
constexpr CsrNodeId NEIGHBOR_4 = 4;

struct Observation
{
  std::vector<CsrNodeId> targets;
  uint32_t routingSequence {0};
  CsrRoutingOperation operation {CsrRoutingOperation::None};
  uint8_t section {0};
  uint8_t totalSections {0};
  std::vector<uint8_t> bytes;
  uint32_t resendQueueBeforeHandoff {0};
};

struct Harness
{
  Ptr<CsrNetDevice> device;
  Ptr<CsrHopLayer> hop;
  Ptr<CsrNetLayer> nwk;
};

std::vector<Observation> g_observations;
Ptr<CsrHopLayer> g_observedHop;
bool g_blockNeighbor3 = false;
bool g_ackScheduled = false;
std::vector<std::vector<CsrNodeId>> g_airTargetLists;
std::vector<CsrHeader::DestinationSequence> g_firstAirSequences;
std::vector<CsrHeader::DestinationSequence> g_residualAirSequences;

void
Require (bool condition, const std::string &message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
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
           "owned routing control has no routing envelope");

  Observation observation;
  observation.targets = std::move (targets);
  observation.routingSequence = envelope.GetRoutingSequence ();
  observation.operation = envelope.GetRoutingOperation ();
  observation.section = envelope.GetRoutingSection ();
  observation.totalSections = envelope.GetRoutingTotalSections ();
  observation.bytes.resize (payload->GetSize ());
  payload->CopyData (observation.bytes.data (), observation.bytes.size ());
  observation.resendQueueBeforeHandoff =
    g_observedHop == nullptr
      ? 0
      : g_observedHop->GetResendQueueSize ();
  g_observations.push_back (std::move (observation));
}

bool
BufferFullForTest (CsrNodeId destination,
                   uint8_t packetType)
{
  Require (packetType == 0x08,
           "buffer gate received a routing opcode instead of ARLPktTypeRoutingUpdate");
  return g_blockNeighbor3 && destination == NEIGHBOR_3;
}

void
DrainSameTime ()
{
  Simulator::Schedule (NanoSeconds (1), [] {
    Simulator::Stop ();
  });
  Simulator::Run ();
}

Harness
MakeCleanHarness ()
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

  uint16_t helloSequence = 1;
  for (CsrNodeId neighbor :
       std::vector<CsrNodeId> {NEIGHBOR_2, NEIGHBOR_3, NEIGHBOR_4})
    {
      harness.nwk->ProcessHello (
        MakePlainHello (neighbor, helloSequence++),
        neighbor,
        70.0,
        20.0);
    }

  // Drain admission snapshots and their watchdogs before creating the one
  // grouped transaction under test.
  Simulator::Run ();
  Require (harness.hop->GetResendQueueSize () == 0,
           "setup left reliable HOP traffic pending");
  g_observations.clear ();
  g_observedHop = harness.hop;
  return harness;
}

Observation
StartOneGroupedOwner (const Harness &harness)
{
  harness.nwk->SetAutomaticRoutePropagationEnabled (true);
  harness.nwk->SetNodeType (CsrNodeType::Routable);
  DrainSameTime ();

  Require (g_observations.size () == 1,
           "one self capability change did not create one grouped owner");
  Require (g_observations[0].targets ==
             std::vector<CsrNodeId> ({NEIGHBOR_4,
                                      NEIGHBOR_3,
                                      NEIGHBOR_2}),
           "initial owner lost newest-created-first destination order");
  Require (harness.nwk->GetOwnedRoutingControlCountForTest () == 1,
           "NWK did not retain the grouped routing-control owner");
  return g_observations[0];
}

void
ReportAck (const Harness &harness,
           const Observation &identity,
           CsrNodeId neighbor,
           bool lastOfInfo)
{
  harness.nwk->NoteRoutingControlSuccess (
    neighbor,
    identity.routingSequence,
    identity.operation,
    identity.section,
    identity.totalSections,
    lastOfInfo);
}

void
ReportFailure (const Harness &harness,
               const Observation &identity,
               const std::vector<CsrNodeId> &attempt)
{
  harness.nwk->NoteRoutingControlFailure (
    attempt,
    identity.routingSequence,
    identity.operation,
    identity.section,
    identity.totalSections,
    true);
}

void
TestRepeatedResidualReduction ()
{
  Harness harness = MakeCleanHarness ();
  Observation identity = StartOneGroupedOwner (harness);
  const std::vector<CsrNodeId> original = identity.targets;

  ReportAck (harness, identity, NEIGHBOR_4, false);
  ReportAck (harness, identity, NEIGHBOR_4, false);
  Require (harness.nwk->GetFirstOwnedRoutingControlResidualForTest () ==
             std::vector<CsrNodeId> ({NEIGHBOR_3, NEIGHBOR_2}),
           "partial or duplicate ACK corrupted the first residual set");
  Require (g_observations.size () == 1,
           "partial ACK synchronously resubmitted the NWK owner");

  ReportFailure (harness, identity, original);
  Require (g_observations.size () == 1 &&
             harness.nwk->IsFirstOwnedRoutingControlReadyForTest (),
           "final NACK re-entered HOP instead of marking the owner ready");
  DrainSameTime ();

  Require (g_observations.size () == 2 &&
             g_observations[1].targets ==
               std::vector<CsrNodeId> ({NEIGHBOR_3, NEIGHBOR_2}),
           "first retry did not contain only the ordered residual set");
  Require (g_observations[1].bytes == identity.bytes &&
             g_observations[1].routingSequence == identity.routingSequence &&
             g_observations[1].operation == identity.operation &&
             g_observations[1].section == identity.section &&
             g_observations[1].totalSections == identity.totalSections,
           "first retry rebuilt or renumbered the retained NWK payload");
  Require (harness.nwk->GetOwnedRoutingControlCountForTest () == 1,
           "first retry created a second NWK owner");

  ReportAck (harness, identity, NEIGHBOR_3, false);
  ReportFailure (harness,
                 identity,
                 std::vector<CsrNodeId> ({NEIGHBOR_3, NEIGHBOR_2}));
  DrainSameTime ();

  Require (g_observations.size () == 3 &&
             g_observations[2].targets ==
               std::vector<CsrNodeId> ({NEIGHBOR_2}) &&
             g_observations[2].bytes == identity.bytes,
           "second failure did not reduce the same owner to one target");
  Require (harness.nwk->GetOwnedRoutingControlCountForTest () == 1,
           "repeated retry duplicated the NWK owner");

  ReportAck (harness, identity, NEIGHBOR_2, true);
  Require (harness.nwk->GetOwnedRoutingControlCountForTest () == 0,
           "final ACK did not release the empty NWK owner");
  Require (g_observations.size () == 3,
           "final ACK emitted an unnecessary retry");
  Simulator::Destroy ();
}

void
TestInactivePruneAndAllOrNoneGate ()
{
  Harness harness = MakeCleanHarness ();
  Observation identity = StartOneGroupedOwner (harness);

  ReportAck (harness, identity, NEIGHBOR_4, false);
  harness.nwk->SetAutomaticRoutePropagationEnabled (false);
  harness.nwk->NoteLinkFailure (NEIGHBOR_2);

  g_blockNeighbor3 = true;
  harness.nwk->SetRoutingControlBufferFullCallbackForTest (
    MakeCallback (&BufferFullForTest));
  ReportFailure (harness, identity, identity.targets);
  DrainSameTime ();

  Require (g_observations.size () == 1,
           "one full residual destination allowed a partial group send");
  Require (harness.nwk->GetOwnedRoutingControlCountForTest () == 1 &&
             harness.nwk->IsFirstOwnedRoutingControlReadyForTest (),
           "buffer-full deferral did not retain a ready NWK owner");
  Require (harness.nwk->GetFirstOwnedRoutingControlResidualForTest () ==
             std::vector<CsrNodeId> ({NEIGHBOR_3}),
           "retry did not prune the inactive residual before admission");

  g_blockNeighbor3 = false;
  harness.nwk->SetRoutingControlBufferFullCallbackForTest (
    MakeCallback (&BufferFullForTest));
  DrainSameTime ();

  Require (g_observations.size () == 2 &&
             g_observations[1].targets ==
               std::vector<CsrNodeId> ({NEIGHBOR_3}) &&
             g_observations[1].bytes == identity.bytes,
           "unblocked retry did not send the entire retained residual");
  Simulator::Destroy ();
}

void
InjectExactAck (CsrNodeId source, uint16_t sequence)
{
  static std::map<CsrNodeId, CsrHopSecurityState> senders;
  CsrHeader ack (
    source,
    LOCAL_NODE,
    sequence,
    7,
    false,
    true);
  ack.SetType (CSR_PKT_ACK);
  ack.SetDestType (CSR_DEST_UNICAST);
  ack.SetSpeedKey (8);
  Ptr<Packet> body = Create<Packet> ();
  body->AddHeader (ack);
  std::vector<uint8_t> plaintext (body->GetSize ());
  body->CopyData (plaintext.data (), plaintext.size ());
  auto [it, inserted] = senders.try_emplace (source);
  if (inserted)
    {
      it->second.SetNodeId (source);
    }
  CsrProtectedPairwiseMessage secured =
    it->second.ProtectPairwiseMessage (
      LOCAL_NODE,
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      plaintext);
  Ptr<Packet> packet = Create<Packet> (secured.record.data (),
                                       secured.record.size ());
  ack.SetSecurityCount (it->second.GetOwnSecurityCount ());
  ack.SetLinkControl (8, 0.0, 0.0);
  packet->AddHeader (ack);
  g_observedHop->ReceiveFromMac (packet, 0.0, 100.0);
}

void
RecordAirFrame (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  if (!frame->PeekHeader (header) ||
      header.GetType () != CSR_PKT_ROUTING_CONTROL)
    {
      return;
    }

  std::vector<CsrHeader::DestinationSequence> targets =
    header.GetDestinationSequences ();
  std::vector<CsrNodeId> nodes;
  for (const auto &target : targets)
    {
      nodes.push_back (target.first);
    }
  g_airTargetLists.push_back (nodes);

  if (nodes.size () == 3 && g_firstAirSequences.empty ())
    {
      g_firstAirSequences = targets;
    }
  if (nodes.size () == 2 && g_residualAirSequences.empty ())
    {
      g_residualAirSequences = targets;
      Simulator::Stop ();
    }

  if (!g_ackScheduled && nodes.size () == 3)
    {
      for (const auto &target : targets)
        {
          if (target.first == NEIGHBOR_3)
            {
              g_ackScheduled = true;
              Simulator::ScheduleNow (
                &InjectExactAck,
                NEIGHBOR_3,
                target.second);
              break;
            }
        }
    }
}

uint16_t
FindSequence (const std::vector<CsrHeader::DestinationSequence> &targets,
              CsrNodeId destination)
{
  for (const auto &target : targets)
    {
      if (target.first == destination)
        {
          return target.second;
        }
    }
  return 0;
}

void
TestRealHopTimeoutOrdering ()
{
  Harness harness = MakeCleanHarness ();

  Ptr<CsrNetDevice> receiver2 = CreateObject<CsrNetDevice> (NEIGHBOR_2);
  Ptr<CsrNetDevice> receiver3 = CreateObject<CsrNetDevice> (NEIGHBOR_3);
  Ptr<CsrNetDevice> receiver4 = CreateObject<CsrNetDevice> (NEIGHBOR_4);
  harness.device->AddPeer (receiver2);
  harness.device->AddPeer (receiver3);
  harness.device->AddPeer (receiver4);
  receiver2->GetMac ().SetRxCallback (MakeCallback (&RecordAirFrame));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  harness.device->GetPhy ().SetPerModel (noErrors);
  receiver2->GetPhy ().SetPerModel (noErrors);
  receiver3->GetPhy ().SetPerModel (noErrors);
  receiver4->GetPhy ().SetPerModel (noErrors);

  g_airTargetLists.clear ();
  g_firstAirSequences.clear ();
  g_residualAirSequences.clear ();
  g_ackScheduled = false;
  Observation identity = StartOneGroupedOwner (harness);

  Simulator::Run ();

  Require (g_observations.size () == 2,
           "real HOP final timeout did not cause one NWK residual handoff");
  Require (g_observations[1].targets ==
             std::vector<CsrNodeId> ({NEIGHBOR_4, NEIGHBOR_2}),
           "real HOP timeout restored an already-ACKed destination");
  Require (g_observations[1].bytes == identity.bytes,
           "real HOP timeout retry changed the retained NWK payload");
  Require (g_observations[1].resendQueueBeforeHandoff == 0,
           "NWK retry ran before HOP erased the expired resend owner");

  uint32_t fullAttempts = static_cast<uint32_t> (
    std::count (g_airTargetLists.begin (),
                g_airTargetLists.end (),
                identity.targets));
  Require (fullAttempts == 3,
           "HOP did not transmit the full original group initially and twice more");
  Require (!g_residualAirSequences.empty (),
           "residual retry never reached the radio");
  Require (FindSequence (g_residualAirSequences, NEIGHBOR_3) == 0,
           "ACKed destination remained in the residual HOP frame");
  Require (FindSequence (g_residualAirSequences, NEIGHBOR_4) !=
             FindSequence (g_firstAirSequences, NEIGHBOR_4) &&
             FindSequence (g_residualAirSequences, NEIGHBOR_2) !=
             FindSequence (g_firstAirSequences, NEIGHBOR_2),
           "residual retry did not allocate fresh HOP sequences");
  Require (harness.nwk->GetOwnedRoutingControlCountForTest () == 1,
           "real residual retry duplicated or lost its NWK owner");
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  TestRepeatedResidualReduction ();
  TestInactivePruneAndAllOrNoneGate ();
  TestRealHopTimeoutOrdering ();
  std::cout << "PASS: NWK-owned residual routing retry" << std::endl;
  return 0;
}
