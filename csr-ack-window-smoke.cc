#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>

using namespace ns3;

namespace
{

std::vector<Time> g_nwkWakeTimes;
Ptr<Packet> g_capturedFeedback;
bool g_shouldDack = false;
uint32_t g_shouldDackCalls = 0;
uint32_t g_dataDeliveries = 0;
Ptr<CsrNetDevice> g_relayDeliveryDevice;
std::vector<uint32_t> g_ackQueueAtRelayDeliveries;
Ptr<CsrNetDevice> g_localDeliveryDevice;
uint32_t g_ackQueueAtLocalDelivery = 0;
uint32_t g_localDackPolicyCalls = 0;

struct CompletionOrderSnapshot
{
  uint32_t pendingData {0};
  uint32_t neighborOutstanding {0};
  uint32_t resendQueue {0};
  uint32_t macDataQueue {0};
};

Ptr<CsrHopLayer> g_completionOrderHop;
Ptr<CsrNetDevice> g_completionOrderDevice;
std::vector<CompletionOrderSnapshot> g_completionOrderSnapshots;

Ptr<Packet>
ProtectFeedback (CsrHeader header)
{
  static std::map<CsrNodeId, CsrHopSecurityState> senders;
  auto [it, inserted] = senders.try_emplace (header.GetSrc ());
  if (inserted)
    {
      it->second.SetNodeId (header.GetSrc ());
    }

  Ptr<Packet> body = Create<Packet> ();
  body->AddHeader (header);
  std::vector<uint8_t> plaintext (body->GetSize ());
  body->CopyData (plaintext.data (), plaintext.size ());
  CsrProtectedPairwiseMessage secured =
    it->second.ProtectPairwiseMessage (
      header.GetDst (),
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      plaintext);

  Ptr<Packet> frame = Create<Packet> (secured.record.data (),
                                      secured.record.size ());
  header.SetSecurityCount (it->second.GetOwnSecurityCount ());
  header.SetLinkControl (8, 0.0, 0.0);
  frame->AddHeader (header);
  return frame;
}

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
ObserveCompletionNsdpRelease (CsrNodeId source, CsrNodeId destination)
{
  Require (g_completionOrderHop != nullptr &&
             g_completionOrderDevice != nullptr,
           "completion-order callback has no active fixture");
  Require (source == 1 && destination == 2,
           "completion-order callback observed the wrong NWK flow");

  CompletionOrderSnapshot snapshot;
  snapshot.pendingData = g_completionOrderHop->GetPendingDataCount ();
  snapshot.neighborOutstanding =
    g_completionOrderHop->GetOutstandingDataCount (2);
  snapshot.resendQueue = g_completionOrderHop->GetResendQueueSize ();
  snapshot.macDataQueue =
    g_completionOrderDevice->GetMac ().GetDataQueuedFrameCount ();
  g_completionOrderSnapshots.push_back (snapshot);
}

Ptr<Packet>
MakeAck (uint16_t baseSeq, uint64_t ackBitmap, uint64_t dackBitmap)
{
  CsrHeader header (2, 1, baseSeq, 7, false, true);
  header.SetType (CSR_PKT_ACK);
  header.SetHasAckWindow (true);
  header.SetAckBitmap (ackBitmap);
  header.SetDackBitmap (dackBitmap);

  return ProtectFeedback (header);
}

Ptr<Packet>
MakeProtectedData (CsrHopSecurityState &sender,
                   uint16_t sequence,
                   CsrNodeId networkDestination = 3)
{
  Ptr<Packet> payload = Create<Packet> (3);
  CsrNetHeader networkHeader (2, networkDestination, 5);
  payload->AddHeader (networkHeader);

  std::vector<uint8_t> plaintext (payload->GetSize ());
  payload->CopyData (plaintext.data (), plaintext.size ());
  CsrProtectedPairwiseMessage secured =
    sender.ProtectPairwiseMessage (
      1,
      CsrPairwiseSecurityMode::Pairwise16,
      3,
      plaintext);
  Ptr<Packet> frame = Create<Packet> (secured.record.data (),
                                      secured.record.size ());

  CsrHeader header (2, 1, sequence, 5, true, false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSecurityCount (sender.GetOwnSecurityCount ());
  frame->AddHeader (header);
  return frame;
}

bool
ShouldDack (CsrNodeId source, CsrNodeId destination)
{
  Require (source == 2 && destination == 3,
           "DACK policy did not inspect the relayed NWK flow");
  g_shouldDackCalls++;
  return g_shouldDack;
}

bool
ForceLocalDack (CsrNodeId source, CsrNodeId destination)
{
  Require (source == 2 && destination == 1,
           "local DACK-policy probe inspected the wrong NWK flow");
  g_localDackPolicyCalls++;
  return true;
}

bool
RelayRouteAvailable (CsrNodeId destination)
{
  return destination == 3;
}

void
NoteDataDelivery (Ptr<Packet>, CsrNodeId)
{
  g_dataDeliveries++;
  if (g_relayDeliveryDevice != nullptr)
    {
      g_ackQueueAtRelayDeliveries.push_back (
        g_relayDeliveryDevice->GetMac ().GetAckQueuedFrameCount ());
    }
}

void
NoteLocalDataDelivery (Ptr<Packet> payload, CsrNodeId source)
{
  Require (g_localDeliveryDevice != nullptr,
           "local delivery ordering test has no receiver device");
  CsrNetHeader networkHeader;
  Require (source == 2 && payload->PeekHeader (networkHeader) &&
             networkHeader.GetDst () == 1,
           "local delivery ordering callback received the wrong flow");
  g_dataDeliveries++;
  g_ackQueueAtLocalDelivery =
    g_localDeliveryDevice->GetMac ().GetAckQueuedFrameCount ();
}

void
CaptureFeedback (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  if (frame->PeekHeader (header) && header.IsAck ())
    {
      g_capturedFeedback = frame->Copy ();
    }
}

CsrHeader
AuthenticateCapturedFeedback ()
{
  Require (g_capturedFeedback != nullptr,
           "cumulative feedback was not transmitted to the capture peer");

  Ptr<Packet> securedRecord = g_capturedFeedback->Copy ();
  CsrHeader outer;
  Require (securedRecord->RemoveHeader (outer) != 0 &&
             outer.HasSecurityCount (),
           "captured feedback lacks its Pairwise16 transport wrapper");

  std::vector<uint8_t> record (securedRecord->GetSize ());
  securedRecord->CopyData (record.data (), record.size ());
  CsrHopSecurityState verifier;
  verifier.SetNodeId (2);
  CsrReceivedPairwiseMessage received =
    verifier.ReceivePairwiseMessage (
      1,
      outer.GetSecurityCount (),
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      record);
  Require (received.status == CsrHopSecurityReceiveStatus::Accepted,
           "captured feedback failed Pairwise16 authentication");

  Ptr<Packet> body = Create<Packet> (received.payload.data (),
                                     received.payload.size ());
  CsrHeader authenticated;
  Require (body->RemoveHeader (authenticated) != 0,
           "authenticated feedback body lacks its CSR header");
  return authenticated;
}

void
CheckReceiverWindowProduction (bool retryAfterDack)
{
  Ptr<CsrNetDevice> producer = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> capture = CreateObject<CsrNetDevice> (2);
  producer->AddPeer (capture);

  CsrPerModelFn noErrors = [] (int, double, uint32_t) { return 0.0; };
  producer->GetPhy ().SetPerModel (noErrors);
  capture->GetPhy ().SetPerModel (noErrors);
  producer->GetPhy ().SetLinkDistanceMeters (1, 2, 1.0);
  capture->GetPhy ().SetLinkDistanceMeters (1, 2, 1.0);
  capture->GetMac ().SetRxCallback (MakeCallback (&CaptureFeedback));

  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&producer->GetMac ());
  hop->SetRxFromHopCallback (MakeCallback (&NoteDataDelivery));
  hop->SetRelayRouteAvailableCallback (
    MakeCallback (&RelayRouteAvailable));

  CsrHopSecurityState dataSender;
  dataSender.SetNodeId (2);

  g_capturedFeedback = nullptr;
  g_shouldDackCalls = 0;
  g_dataDeliveries = 0;
  g_relayDeliveryDevice = producer;
  g_ackQueueAtRelayDeliveries.clear ();
  if (retryAfterDack)
    {
      hop->SetShouldDackCallback (MakeCallback (&ShouldDack));
      Ptr<Packet> retransmitted = MakeProtectedData (dataSender, 77);
      g_shouldDack = true;
      hop->ReceiveFromMac (retransmitted->Copy (), 90.0, 10.0);
      g_shouldDack = false;
      hop->ReceiveFromMac (retransmitted->Copy (), 90.0, 10.0);
      Require (g_shouldDackCalls == 2,
               "DACKed retry was suppressed instead of reassessed");
      Require (g_dataDeliveries == 2,
               "DACKed authenticated retry was not re-enqueued to NWK");
    }
  else
    {
      hop->ReceiveFromMac (MakeProtectedData (dataSender, 77), 90.0, 10.0);
      hop->ReceiveFromMac (MakeProtectedData (dataSender, 76), 90.0, 10.0);
    }

  Require (!g_ackQueueAtRelayDeliveries.empty () &&
             g_ackQueueAtRelayDeliveries.front () == 0,
           "relay NWK callback did not run before ACK/DACK MAC admission");
  Require (producer->GetMac ().GetAckQueuedFrameCount () == 1,
           "relay DATA did not leave one cumulative feedback entry queued");
  g_relayDeliveryDevice = nullptr;
  Simulator::Stop (Seconds (4.0));
  Simulator::Run ();

  CsrHeader authenticated = AuthenticateCapturedFeedback ();
  Require (authenticated.GetSeq () == 77,
           "cumulative feedback base did not remain at receiver highest");
  if (retryAfterDack)
    {
      Require (authenticated.GetAckBitmap () == 1 &&
                 authenticated.GetDackBitmap () == 1,
               "DACK-to-accepted retry did not transmit raw overlapping maps");
    }
  else
    {
      Require (authenticated.GetAckBitmap () == 3 &&
                 authenticated.GetDackBitmap () == 0,
               "out-of-order reception produced the wrong cumulative map");
    }

  Simulator::Destroy ();
}

void
CheckLocalDeliveryQueuesFeedbackFirst ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> capture = CreateObject<CsrNetDevice> (2);
  device->AddPeer (capture);

  CsrPerModelFn noErrors = [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
  capture->GetPhy ().SetPerModel (noErrors);
  device->GetPhy ().SetLinkDistanceMeters (1, 2, 1.0);
  capture->GetPhy ().SetLinkDistanceMeters (1, 2, 1.0);
  capture->GetMac ().SetRxCallback (MakeCallback (&CaptureFeedback));

  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());
  hop->SetRxFromHopCallback (MakeCallback (&NoteLocalDataDelivery));
  hop->SetShouldDackCallback (MakeCallback (&ForceLocalDack));

  CsrHopSecurityState sender;
  sender.SetNodeId (2);
  g_capturedFeedback = nullptr;
  g_localDeliveryDevice = device;
  g_dataDeliveries = 0;
  g_ackQueueAtLocalDelivery = 0;
  g_localDackPolicyCalls = 0;

  hop->ReceiveFromMac (MakeProtectedData (sender, 91, 1), 90.0, 10.0);

  Require (g_dataDeliveries == 1,
           "locally addressed DATA was not delivered to NWK");
  Require (g_localDackPolicyCalls == 0,
           "local DATA incorrectly consulted the relay-only DACK policy");
  Require (g_ackQueueAtLocalDelivery == 1,
           "local NWK callback ran before its ACK reached the MAC queue");
  Require (device->GetMac ().GetAckQueuedFrameCount () == 1,
           "local DATA did not leave one cumulative ACK queued");

  g_localDeliveryDevice = nullptr;
  Simulator::Stop (Seconds (4.0));
  Simulator::Run ();

  CsrHeader authenticated = AuthenticateCapturedFeedback ();
  Require (authenticated.GetType () == CSR_PKT_ACK &&
             !authenticated.IsDack () &&
             authenticated.GetAckBitmap () == 1 &&
             authenticated.GetDackBitmap () == 0,
           "local DATA did not transmit an ordinary cumulative ACK");

  Simulator::Destroy ();
}

void
CheckAckWinsOverlap ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());

  hop->SendData (2, 5, Create<Packet> (8), true);
  Require (hop->GetPendingDataCount () == 1,
           "ACK-precedence setup did not create one pending DATA frame");
  hop->ReceiveFromMac (MakeAck (1, 1, 1), 90.0, 10.0);
  Require (hop->GetPendingDataCount () == 0 &&
             hop->GetResendQueueSize () == 0 &&
             device->GetMac ().GetDataQueuedFrameCount () == 0,
           "overlapping cumulative bits did not complete through ACK first");

  Simulator::Destroy ();
}

Ptr<Packet>
MakeExactFeedback (uint16_t sequence,
                   bool dack,
                   CsrNodeId destination = 1)
{
  CsrHeader header (2, destination, sequence, 7, false, true);
  header.SetType (dack ? CSR_PKT_DACK : CSR_PKT_ACK);
  header.SetIsDack (dack);
  header.SetDestType (CSR_DEST_UNICAST);

  return ProtectFeedback (header);
}

void
WakeNwkQueue ()
{
  g_nwkWakeTimes.push_back (Simulator::Now ());
}

void
InjectFeedback (Ptr<CsrHopLayer> hop, Ptr<Packet> packet)
{
  hop->ReceiveFromMac (packet, 90.0, 10.0);
}

void
CheckWakeCount (uint32_t expected, Time expectedLastWake)
{
  Require (g_nwkWakeTimes.size () == expected,
           "HOP-origin NWK wake count does not match source coalescing");
  if (expected > 0)
    {
      Require (g_nwkWakeTimes.back () == expectedLastWake,
               "HOP-origin NWK wake ran at the wrong TIC boundary");
    }
}

void
CheckSenderCompletionOrder (bool cumulative)
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());
  hop->SetNsdpDecrementCallback (
    MakeCallback (&ObserveCompletionNsdpRelease));
  hop->SetNwkQueueWakeCallback (MakeCallback (&WakeNwkQueue));

  const uint32_t transactionCount = cumulative ? 2 : 1;
  for (uint32_t index = 0; index < transactionCount; ++index)
    {
      Ptr<Packet> payload = Create<Packet> (8);
      payload->AddHeader (CsrNetHeader (1, 2, 5));
      hop->SendData (2, 5, payload, true);
    }

  Require (hop->GetPendingDataCount () == transactionCount &&
             hop->GetOutstandingDataCount (2) == transactionCount &&
             hop->GetResendQueueSize () == transactionCount &&
             device->GetMac ().GetDataQueuedFrameCount () == transactionCount,
           "completion-order fixture did not retain every DATA transaction");

  g_completionOrderHop = hop;
  g_completionOrderDevice = device;
  g_completionOrderSnapshots.clear ();
  g_nwkWakeTimes.clear ();

  hop->ReceiveFromMac (
    cumulative ? MakeAck (2, 0x3ULL, 0)
               : MakeExactFeedback (1, false),
    90.0,
    10.0);

  Require (g_completionOrderSnapshots.size () == transactionCount,
           "positive ACK did not release each NWK flow exactly once");
  for (uint32_t index = 0; index < transactionCount; ++index)
    {
      const CompletionOrderSnapshot &snapshot =
        g_completionOrderSnapshots[index];
      const uint32_t remaining = transactionCount - index - 1;

      Require (snapshot.pendingData == remaining &&
                 snapshot.neighborOutstanding == remaining,
               "NSDP release ran before HOP flow capacity was released");
      Require (snapshot.resendQueue == remaining + 1,
               "resend custody was erased before NSDP release");
      Require (snapshot.macDataQueue == transactionCount,
               "MAC queued-copy cancellation ran before HOP completion");
    }

  Require (hop->GetPendingDataCount () == 0 &&
             hop->GetOutstandingDataCount (2) == 0 &&
             hop->GetResendQueueSize () == 0 &&
             device->GetMac ().GetDataQueuedFrameCount () == 0,
           "positive ACK did not finish HOP custody before returning");
  Require (g_nwkWakeTimes.empty (),
           "positive ACK woke NWK synchronously");

  const Time tic = CsrOpnetTic ();
  Simulator::Stop (tic + NanoSeconds (1));
  Simulator::Run ();
  Require (g_nwkWakeTimes.size () == 1 && g_nwkWakeTimes[0] == tic,
           "positive ACK did not preserve its one-TIC NWK wake");

  g_nwkWakeTimes.clear ();
  g_completionOrderHop = nullptr;
  g_completionOrderDevice = nullptr;
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  CsrHeader exact (1, 2, 9, 7, false, true);
  Ptr<Packet> exactPacket = Create<Packet> ();
  exactPacket->AddHeader (exact);
  Require (exactPacket->GetSize () == 13,
           "exact ACK does not carry two 24-bit node identifiers");

  CsrHeader windowed (1, 2, 9, 7, false, true);
  windowed.SetHasAckWindow (true);
  windowed.SetAckBitmap (0x8000000000000005ULL);
  windowed.SetDackBitmap (0x2ULL);
  Ptr<Packet> windowedPacket = Create<Packet> ();
  windowedPacket->AddHeader (windowed);
  Require (windowedPacket->GetSize () == 29,
           "windowed ACK wire size is not 29");

  CsrHeader decoded;
  windowedPacket->RemoveHeader (decoded);
  Require (decoded.HasAckWindow (), "ACK window flag did not round-trip");
  Require (decoded.GetAckBitmap () == 0x8000000000000005ULL,
           "ACK bitmap did not round-trip");
  Require (decoded.GetDackBitmap () == 0x2ULL,
           "DACK bitmap did not round-trip");

  CheckReceiverWindowProduction (false);
  CheckReceiverWindowProduction (true);
  CheckLocalDeliveryQueuesFeedbackFirst ();
  CheckAckWinsOverlap ();
  CheckSenderCompletionOrder (false);
  CheckSenderCompletionOrder (true);

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());
  hop->SetNwkQueueWakeCallback (MakeCallback (&WakeNwkQueue));

  hop->SendData (2, 5, Create<Packet> (8), true);
  hop->SendData (2, 5, Create<Packet> (8), true);
  hop->SendData (2, 5, Create<Packet> (8), true);
  Require (!hop->CanSendToHop (2), "three pending DATA frames were not gated");

  // No exact ACKs for seq 1 or seq 3 are delivered.  A later cumulative ACK
  // completes both while leaving the missing seq 2 outstanding.
  hop->ReceiveFromMac (MakeAck (3, 0x5ULL, 0), 90.0, 10.0);
  Require (!hop->CanSendToHop (2), "missing seq 2 was incorrectly completed");

  // The next cumulative ACK repeats the historical bits.  Previously completed
  // entries must be ignored while seq 2 is released exactly once.
  hop->ReceiveFromMac (MakeAck (3, 0x7ULL, 0), 90.0, 10.0);
  Require (hop->CanSendToHop (2), "final cumulative ACK did not release HOP flow control");

  Require (device->GetMac ().CancelAcknowledgedFrames (2, 3, 0x7ULL, 0) == 0,
           "acknowledged retransmissions remained in the MAC queue");

  // The source hard-disables its legacy single-DACK path.  Keep one known
  // sequence outstanding so a non-window DACK proves to be a custody/MAC
  // no-op while still participating in the packet-level generic wake below.
  hop->SendData (2, 5, Create<Packet> (8), true);
  Require (hop->GetResendQueueSize () == 1 &&
             hop->GetPendingDataCount () == 1,
           "known non-window DACK setup did not retain sequence 4");

  Require (g_nwkWakeTimes.empty (),
           "generic NWK wake ran synchronously with ACK processing");

  const Time tic = CsrOpnetTic ();
  Simulator::Schedule (tic - NanoSeconds (1),
                       &CheckWakeCount,
                       0,
                       Seconds (0));
  Simulator::Schedule (tic,
                       &CheckWakeCount,
                       1,
                       tic);

  // Unknown ACK, known and unknown non-window DACKs, and a zero-bit cumulative
  // ACK all request the same HOP-owned remote interrupt.  They arrive within
  // one TIC, so the first pending wake is retained and later requests do not
  // postpone it.  The known single DACK must not change custody.
  Simulator::Schedule (2 * tic,
                       &InjectFeedback,
                       hop,
                       MakeExactFeedback (400, false));
  Simulator::Schedule (2 * tic + NanoSeconds (5),
                       &InjectFeedback,
                       hop,
                       MakeExactFeedback (4, true));
  Simulator::Schedule (2 * tic + NanoSeconds (7),
                       &InjectFeedback,
                       hop,
                       MakeExactFeedback (401, true));
  Simulator::Schedule (2 * tic + NanoSeconds (10),
                       &InjectFeedback,
                       hop,
                       MakeAck (402, 0, 0));
  // A readable ACK for another HOP destination is not locally addressed and
  // therefore must not request a queue wake.
  Simulator::Schedule (2 * tic + NanoSeconds (15),
                       &InjectFeedback,
                       hop,
                       MakeExactFeedback (403, false, 9));
  Simulator::Schedule (3 * tic + NanoSeconds (1),
                       &CheckWakeCount,
                       2,
                       3 * tic);

  // Feedback for an already completed sequence is stale/no-op state, but the
  // source still requests a fresh wake after the previous one has executed.
  Simulator::Schedule (4 * tic,
                       &InjectFeedback,
                       hop,
                       MakeExactFeedback (3, false));
  Simulator::Schedule (5 * tic + NanoSeconds (1),
                       &CheckWakeCount,
                       3,
                       5 * tic);

  Simulator::Stop (6 * tic);
  Simulator::Run ();

  Require (hop->GetResendQueueSize () == 1 &&
             hop->GetPendingDataCount () == 1 &&
             hop->GetOutstandingDataCount (2) == 1,
           "no-op feedback changed the known non-window DACK custody state");
  Require (device->GetMac ().GetDataQueuedFrameCount () == 1,
           "known non-window DACK canceled its pending MAC copy");

  Simulator::Destroy ();
  std::cout << "PASS: cumulative ACK window and generic wake smoke test"
            << std::endl;
  return 0;
}
