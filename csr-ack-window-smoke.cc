#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

std::vector<Time> g_nwkWakeTimes;

void
Require (bool condition, const char* message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

Ptr<Packet>
MakeAck (uint16_t baseSeq, uint64_t ackBitmap, uint64_t dackBitmap)
{
  CsrHeader header (2, 1, baseSeq, 7, false, true);
  header.SetType (CSR_PKT_ACK);
  header.SetHasAckWindow (true);
  header.SetAckBitmap (ackBitmap);
  header.SetDackBitmap (dackBitmap);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (header);
  return packet;
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

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (header);
  return packet;
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
