#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hello-header.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>

using namespace ns3;

namespace
{

struct SuccessEvent
{
  CsrNodeId neighbor;
  bool lastOfInfo;
};

struct FailureEvent
{
  std::vector<CsrNodeId> destinations;
  bool lastOfInfo;
};

std::vector<SuccessEvent> g_successEvents;
std::vector<FailureEvent> g_failureEvents;
uint32_t g_receivedAt2 = 0;
uint32_t g_receivedAt3 = 0;
uint16_t g_sequenceAt2 = 0;
uint16_t g_sequenceAt3 = 0;
uint16_t g_autoAckSequence = 0;
uint32_t g_linkFailures = 0;
uint32_t g_bareRoutingDeliveries = 0;

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
ResetObservations ()
{
  g_successEvents.clear ();
  g_failureEvents.clear ();
  g_receivedAt2 = 0;
  g_receivedAt3 = 0;
  g_sequenceAt2 = 0;
  g_sequenceAt3 = 0;
  g_autoAckSequence = 0;
  g_linkFailures = 0;
}

void
RecordLinkFailure (CsrNodeId)
{
  g_linkFailures++;
}

void
RecordRoutingFrame (CsrNodeId receiver,
                    Ptr<Packet> frame)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "multidestination frame has no CSR header");
  Require (header.GetType () == CSR_PKT_ROUTING_CONTROL,
           "receiver got an unexpected packet type");
  Require (header.GetDestType () == CSR_DEST_MULTICAST,
           "routing group was not marked multicast");
  Require (header.GetDestinationSequences ().size () == 2,
           "routing group did not retain both targets");

  uint16_t localSequence = 0;
  Require (header.GetSequenceForDestination (
             receiver,
             localSequence),
           "receiver was absent from routing target list");

  if (receiver == 2)
    {
      g_receivedAt2++;
      g_sequenceAt2 = localSequence;
    }
  else
    {
      g_receivedAt3++;
      g_sequenceAt3 = localSequence;
    }
}

void
RecordAt2 (Ptr<Packet> frame, double, double)
{
  RecordRoutingFrame (2, frame);
}

void
RecordAt3 (Ptr<Packet> frame, double, double)
{
  RecordRoutingFrame (3, frame);
}

void
RecordAutomaticAck (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "automatic routing ACK has no CSR header");
  Require (header.IsAck () && !header.HasAckWindow (),
           "multidestination routing receiver did not send an exact ACK");
  Require (CsrGetOpnetWireSize (frame) == 30,
           "Pairwise16 control ACK did not retain its 30-byte envelope");

  if (g_autoAckSequence == 0)
    {
      g_autoAckSequence = header.GetSeq ();
    }
}

void
RecordBareRoutingDelivery (Ptr<Packet>, CsrNodeId, double, double)
{
  g_bareRoutingDeliveries++;
}

void
RecordSuccess (CsrNodeId neighbor,
               uint32_t routingSequence,
               CsrRoutingOperation operation,
               uint8_t section,
               uint8_t totalSections,
               bool lastOfInfo)
{
  Require (routingSequence == 77,
           "success callback carried wrong routing sequence");
  Require (operation == CsrRoutingOperation::Update,
           "success callback carried wrong routing operation");
  Require (section == 0 && totalSections == 1,
           "success callback carried wrong routing section");
  g_successEvents.push_back ({neighbor, lastOfInfo});
}

void
RecordFailure (std::vector<CsrNodeId> destinations,
               uint32_t routingSequence,
               CsrRoutingOperation operation,
               uint8_t section,
               uint8_t totalSections,
               bool lastOfInfo)
{
  Require (routingSequence == 77,
           "failure callback carried wrong routing sequence");
  Require (operation == CsrRoutingOperation::Update,
           "failure callback carried wrong routing operation");
  Require (section == 0 && totalSections == 1,
           "failure callback carried wrong routing section");
  g_failureEvents.push_back ({destinations, lastOfInfo});
}

Ptr<Packet>
MakeRoutingPayload (CsrNodeId source = 1,
                    uint8_t section = 0,
                    uint8_t totalSections = 1)
{
  CsrHelloHeader control;
  control.SetNodeId (source);
  control.SetArlRouteMsgType (
    CsrArlRouteMsgType::RoutingUpdate);
  control.SetRoutingSequence (77);
  control.SetRoutingOperation (
    CsrRoutingOperation::Update);
  control.SetRoutingSection (section);
  control.SetRoutingTotalSections (totalSections);

  Ptr<Packet> payload = Create<Packet> (4);
  payload->AddHeader (control);
  return payload;
}

Ptr<Packet>
MakeNeighborCheckPayload ()
{
  CsrHelloHeader control;
  control.SetArlRouteMsgType (
    CsrArlRouteMsgType::NeighborCheck);
  control.SetNeighborCheckType (
    CsrNeighborCheckType::Verify);
  control.SetNeighborCheckTarget (2);
  control.SetDiscoverySequence (1);

  Ptr<Packet> payload = Create<Packet> ();
  payload->AddHeader (control);
  return payload;
}

void
InjectExactAck (Ptr<CsrHopLayer> hop,
                CsrNodeId source,
                uint16_t sequence)
{
  static std::map<CsrNodeId, CsrHopSecurityState> senders;
  CsrHeader ackHeader (
    source,
    1,
    sequence,
    7,
    false,
    true);
  ackHeader.SetType (CSR_PKT_ACK);
  ackHeader.SetDestType (CSR_DEST_UNICAST);
  ackHeader.SetSpeedKey (8);

  Ptr<Packet> body = Create<Packet> ();
  body->AddHeader (ackHeader);
  std::vector<uint8_t> plaintext (body->GetSize ());
  body->CopyData (plaintext.data (), plaintext.size ());
  auto [it, inserted] = senders.try_emplace (source);
  if (inserted)
    {
      it->second.SetNodeId (source);
    }
  CsrProtectedPairwiseMessage secured =
    it->second.ProtectPairwiseMessage (
      1,
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      plaintext);
  Ptr<Packet> ack = Create<Packet> (secured.record.data (),
                                    secured.record.size ());
  ackHeader.SetSecurityCount (it->second.GetOwnSecurityCount ());
  ackHeader.SetLinkControl (8, 0.0, 0.0);
  ack->AddHeader (ackHeader);
  hop->ReceiveFromMac (ack, 0.0, 100.0);
}

void
ConfigureNoErrorLink (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

void
CheckPartialProgress (Ptr<CsrHopLayer> hop)
{
  Require (g_successEvents.size () == 2,
           "duplicate partial ACK did not repeat OPNET success reporting");
  Require (g_successEvents[0].neighbor == 2 &&
             !g_successEvents[0].lastOfInfo &&
             g_successEvents[1].neighbor == 2 &&
             !g_successEvents[1].lastOfInfo,
           "partial routing ACK was incorrectly marked final");
  Require (hop->GetResendQueueSize () == 1,
           "partial routing ACK removed the group resend entry");
}

void
CheckRetryAndAckFinal (Ptr<CsrHopLayer> hop)
{
  Require (g_receivedAt2 == 2 && g_receivedAt3 == 2,
           "whole routing target list was not retransmitted once");
  Require (g_sequenceAt2 == 2 && g_sequenceAt3 == 1,
           "targets did not retain independent per-neighbor sequences");

  InjectExactAck (hop, 3, 1);

  Require (g_successEvents.size () == 3,
           "final routing ACK did not report success");
  Require (g_successEvents.back ().neighbor == 3 &&
             g_successEvents.back ().lastOfInfo,
           "final routing ACK did not set lastOfInfo");
  Require (hop->GetResendQueueSize () == 0,
           "fully ACKed routing group remained in resend queue");
  Require (g_failureEvents.empty (),
           "completed routing group also reported failure");
}

void
TestPartialAndFinalAck ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver3 = CreateObject<CsrNetDevice> (3);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();

  hop->SetNodeId (1);
  hop->SetMac (&sender->GetMac ());
  hop->SetRoutingControlSuccessCallback (
    MakeCallback (&RecordSuccess));
  hop->SetRoutingControlFailureCallback (
    MakeCallback (&RecordFailure));

  sender->AddPeer (receiver2);
  sender->AddPeer (receiver3);
  receiver2->GetMac ().SetRxCallback (MakeCallback (&RecordAt2));
  receiver3->GetMac ().SetRxCallback (MakeCallback (&RecordAt3));
  ConfigureNoErrorLink (sender);
  ConfigureNoErrorLink (receiver2);
  ConfigureNoErrorLink (receiver3);

  // Advance only node 2's shared HOP sequence namespace.  Remove the MAC copy
  // and ACK it directly so the seed transaction cannot affect observations.
  hop->SendNeighborCheck (2, MakeNeighborCheckPayload ());
  Require (sender->GetMac ().CancelAcknowledgedFrames (
             2, 1, 1, 0) == 1,
           "could not remove sequence-history seed from MAC");
  InjectExactAck (hop, 2, 1);

  hop->SendRoutingControl (
    std::vector<CsrNodeId> {2, 3},
    MakeRoutingPayload ());

  Simulator::Schedule (
    Seconds (1.7),
    &InjectExactAck,
    hop,
    2,
    2);
  Simulator::Schedule (
    Seconds (1.8),
    &InjectExactAck,
    hop,
    2,
    2);
  Simulator::Schedule (
    Seconds (2.0),
    &CheckPartialProgress,
    hop);
  Simulator::Schedule (
    Seconds (4.2),
    &CheckRetryAndAckFinal,
    hop);

  Simulator::Stop (Seconds (4.4));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
TestReceiverSelectsItsSequence ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> observer = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> responder = CreateObject<CsrNetDevice> (2);
  Ptr<CsrHopLayer> responderHop = CreateObject<CsrHopLayer> ();

  responderHop->SetNodeId (2);
  responderHop->SetMac (&responder->GetMac ());
  responder->AddPeer (observer);
  observer->GetMac ().SetRxCallback (
    MakeCallback (&RecordAutomaticAck));
  ConfigureNoErrorLink (observer);
  ConfigureNoErrorLink (responder);

  CsrHeader routingHeader (
    1,
    3,
    7,
    7,
    true,
    false);
  routingHeader.SetType (CSR_PKT_ROUTING_CONTROL);
  routingHeader.SetDestType (CSR_DEST_MULTICAST);
  routingHeader.SetSpeedKey (8);
  routingHeader.SetDestinationSequences (
    std::vector<CsrHeader::DestinationSequence> {
      {3, 7},
      {2, 9}});

  Ptr<Packet> frame = MakeRoutingPayload ();
  frame->AddHeader (routingHeader);
  responderHop->ReceiveFromMac (frame, 0.0, 100.0);

  Require (responder->GetMac ().GetAckQueuedFrameCount () == 1,
           "targeted receiver did not queue a routing ACK");

  // The observer is unknown to the responder, so OPNET sends this exact ACK
  // with a long preamble.  Allow the first ACK to finish over the air.
  Simulator::Stop (Seconds (1.8));
  Simulator::Run ();

  Require (g_autoAckSequence == 9,
           "receiver ACKed the primary sequence instead of its paired sequence");
  Simulator::Destroy ();
}

Ptr<Packet>
MakeBareRoutingFrame (Ptr<Packet> payload, uint16_t sequence)
{
  CsrHeader header (1, 2, sequence, 7, true, false);
  header.SetType (CSR_PKT_ROUTING_CONTROL);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);
  payload->AddHeader (header);
  return payload;
}

void
TestMalformedBareRoutingDoesNotPoisonReplay ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (2);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (2);
  hop->SetMac (&device->GetMac ());
  hop->SetRxHelloFromHopCallback (
    MakeCallback (&RecordBareRoutingDelivery));
  g_bareRoutingDeliveries = 0;

  hop->ReceiveFromMac (
    MakeBareRoutingFrame (MakeRoutingPayload (3), 40),
    0.0,
    100.0);
  Require (g_bareRoutingDeliveries == 0 &&
             device->GetMac ().GetAckQueuedFrameCount () == 0,
           "bare RoutingControl accepted a mismatched inner source");

  hop->ReceiveFromMac (
    MakeBareRoutingFrame (MakeRoutingPayload (), 40),
    0.0,
    100.0);
  Require (g_bareRoutingDeliveries == 1 &&
             device->GetMac ().GetAckQueuedFrameCount () == 1,
           "mismatched bare RoutingControl poisoned replay state");

  // This payload has a plausible fixed prefix, but omits the mandatory Chirp
  // and route counts.  It must not reserve sequence 41 in the replay window.
  std::vector<uint8_t> truncated (30, 0);
  truncated[10] = static_cast<uint8_t> (
    CsrArlRouteMsgType::RoutingUpdate);
  truncated[25] = 1;
  truncated[26] = static_cast<uint8_t> (
    CsrRoutingOperation::Update);
  Ptr<Packet> malformed = Create<Packet> (
    truncated.data (), truncated.size ());
  hop->ReceiveFromMac (
    MakeBareRoutingFrame (malformed, 41),
    0.0,
    100.0);

  Require (g_bareRoutingDeliveries == 1 &&
             device->GetMac ().GetAckQueuedFrameCount () == 1,
           "malformed bare RoutingControl earned delivery or ACK");

  hop->ReceiveFromMac (
    MakeBareRoutingFrame (MakeRoutingPayload (), 41),
    0.0,
    100.0);

  Require (g_bareRoutingDeliveries == 2 &&
             device->GetMac ().GetAckQueuedFrameCount () == 2,
           "malformed bare RoutingControl poisoned replay state");

  hop->ReceiveFromMac (
    MakeBareRoutingFrame (MakeRoutingPayload (1, 1, 1), 42),
    0.0,
    100.0);
  Require (g_bareRoutingDeliveries == 2 &&
             device->GetMac ().GetAckQueuedFrameCount () == 2,
           "out-of-range routing section earned delivery or ACK");

  hop->ReceiveFromMac (
    MakeBareRoutingFrame (MakeRoutingPayload (), 42),
    0.0,
    100.0);
  Require (g_bareRoutingDeliveries == 3 &&
             device->GetMac ().GetAckQueuedFrameCount () == 3,
           "out-of-range routing section poisoned replay state");
  Simulator::Destroy ();
}

void
TestResendCapacitySendsUntracked513thFrame ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());

  for (uint16_t sequence = 1;
       sequence <= CsrHopLayer::RESEND_QUEUE_SIZE;
       ++sequence)
    {
      hop->SendData (2, 5, Create<Packet> (1), true);
      Require (device->GetMac ().CancelAcknowledgedFrames (
                 2, sequence, 1, 0) == 1,
               "could not isolate HOP resend-capacity fixture from MAC");
    }

  Require (CsrHopLayer::RESEND_QUEUE_SIZE == 512 &&
             hop->GetResendQueueSize () == 512 &&
             hop->GetResendQueueOverflowCount () == 0,
           "HOP did not retain its source-exact 512 resend owners");

  hop->SendData (2, 5, Create<Packet> (1), true);
  Require (hop->GetResendQueueSize () == 512 &&
             hop->GetResendQueueOverflowCount () == 1 &&
             device->GetMac ().GetDataQueuedFrameCount () == 1,
           "HOP did not send the untracked 513th overflow frame");
  Simulator::Destroy ();
}

void
CheckGroupFailure (Ptr<CsrHopLayer> hop)
{
  Require (g_receivedAt2 == 3 && g_receivedAt3 == 3,
           "routing group did not make one initial TX plus two full retries");
  Require (g_successEvents.size () >= 1 &&
             g_successEvents.front ().neighbor == 2 &&
             !g_successEvents.front ().lastOfInfo,
           "single target ACK was not retained as partial progress");
  Require (g_failureEvents.size () == 1,
           "routing timeout did not produce exactly one group failure");
  Require (g_failureEvents[0].destinations ==
             std::vector<CsrNodeId> ({2, 3}),
           "routing failure did not retain the original ordered target list");
  Require (g_failureEvents[0].lastOfInfo,
           "routing group failure did not set lastOfInfo");
  Require (g_linkFailures == 0,
           "group timeout incorrectly penalized its already-ACKed primary target");
  Require (hop->GetResendQueueSize () == 0,
           "timed-out routing group remained in resend queue");
}

void
TestGroupFailure ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver3 = CreateObject<CsrNetDevice> (3);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();

  hop->SetNodeId (1);
  hop->SetMac (&sender->GetMac ());
  hop->SetRoutingControlSuccessCallback (
    MakeCallback (&RecordSuccess));
  hop->SetRoutingControlFailureCallback (
    MakeCallback (&RecordFailure));
  hop->SetLinkFailureCallback (
    MakeCallback (&RecordLinkFailure));

  sender->AddPeer (receiver2);
  sender->AddPeer (receiver3);
  receiver2->GetMac ().SetRxCallback (MakeCallback (&RecordAt2));
  receiver3->GetMac ().SetRxCallback (MakeCallback (&RecordAt3));
  ConfigureNoErrorLink (sender);
  ConfigureNoErrorLink (receiver2);
  ConfigureNoErrorLink (receiver3);

  hop->SendRoutingControl (
    std::vector<CsrNodeId> {2, 3},
    MakeRoutingPayload ());

  Simulator::Schedule (
    Seconds (1.7),
    &InjectExactAck,
    hop,
    2,
    1);
  Simulator::Schedule (
    Seconds (10.0),
    &CheckGroupFailure,
    hop);

  Simulator::Stop (Seconds (10.2));
  Simulator::Run ();
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  TestPartialAndFinalAck ();
  TestReceiverSelectsItsSequence ();
  TestMalformedBareRoutingDoesNotPoisonReplay ();
  TestResendCapacitySendsUntracked513thFrame ();
  TestGroupFailure ();

  std::cout << "PASS: OPNET multidestination routing ACK parity test"
            << std::endl;
  return 0;
}
