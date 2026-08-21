#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

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

} // namespace

int
main ()
{
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

  Simulator::Destroy ();
  std::cout << "PASS: cumulative ACK window smoke test" << std::endl;
  return 0;
}
