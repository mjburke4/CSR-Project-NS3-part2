#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

Ptr<CsrNetDevice> g_sender;
std::vector<uint16_t> g_ackSequences;
uint32_t g_dataFramesReceived = 0;
bool g_replacedAfterTwoSends = false;

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
BuildWindowAck (CsrNodeId src,
                CsrNodeId dst,
                uint16_t baseSeq,
                uint64_t ackBitmap,
                uint64_t dackBitmap)
{
  CsrHeader header (src, dst, baseSeq, 0, false, true);
  header.SetType (CSR_PKT_ACK);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);
  header.SetHasAckWindow (true);
  header.SetAckBitmap (ackBitmap);
  header.SetDackBitmap (dackBitmap);

  Ptr<Packet> frame = Create<Packet> ();
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildExactAck (CsrNodeId src, CsrNodeId dst, uint16_t seq)
{
  CsrHeader header (src, dst, seq, 0, false, true);
  header.SetType (CSR_PKT_ACK);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);

  Ptr<Packet> frame = Create<Packet> ();
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildData (CsrNodeId src, CsrNodeId dst, uint16_t seq)
{
  CsrHeader header (src, dst, seq, 7, true, false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);

  Ptr<Packet> frame = Create<Packet> (20);
  frame->AddHeader (header);
  return frame;
}

void
RecordReceivedFrame (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received frame did not contain a CSR header");

  if (header.IsAck () || header.IsDack ())
    {
      g_ackSequences.push_back (header.GetSeq ());

      // OPNET resets tx_count when a new cumulative window replaces an ACK
      // already pending for the same destination.  Replace after two actual
      // receptions so the test can distinguish reset from simple coalescing.
      if (g_ackSequences.size () == 2 && !g_replacedAfterTwoSends)
        {
          g_replacedAfterTwoSends = true;
          g_sender->GetMac ().EnqueueTxFrame (
            BuildWindowAck (1, 2, 11, 0x7, 0),
            2,
            0,
            false);
        }
      return;
    }

  Require (header.GetType () == CSR_PKT_DATA,
           "unexpected non-ACK frame type received");
  Require (header.GetSeq () == 77,
           "MAC transmitted the wrong queued data frame");
  Require (g_ackSequences.size () == 1,
           "data was not packed behind the first ACK as OPNET does");
  g_dataFramesReceived++;
}

void
CheckCompletedExchange (Ptr<CsrNetDevice> sender)
{
  Require (g_replacedAfterTwoSends,
           "test never replaced the pending cumulative ACK");
  Require (g_ackSequences.size () == 7,
           "ACK replacement did not reset the five-transmission budget");
  Require (g_ackSequences[0] == 10 && g_ackSequences[1] == 10,
           "initial coalesced ACK was not transmitted twice");

  for (std::size_t i = 2; i < g_ackSequences.size (); ++i)
    {
      Require (g_ackSequences[i] == 11,
               "replacement ACK did not supersede the old window");
    }

  Require (g_dataFramesReceived == 1,
           "queued data was not transmitted exactly once");
  Require (sender->GetMac ().GetTransmittedFrameCount () == 7,
           "ACK/DATA concatenation did not save one OTA transmission");
  Require (sender->GetMac ().GetAckQueuedFrameCount () == 0,
           "ACK queue did not remove the entry after five replacement sends");
  Require (sender->GetMac ().GetDataQueuedFrameCount () == 0,
           "data queue did not drain after ACK service");
}

void
CheckDeduplicationAndCapacity ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (3);

  device->GetMac ().EnqueueTxFrame (
    BuildExactAck (3, 4, 1), 4, 0, false);
  device->GetMac ().EnqueueTxFrame (
    BuildExactAck (3, 4, 1), 4, 0, false);

  Require (device->GetMac ().GetAckQueuedFrameCount () == 1,
           "duplicate exact ACK created a second queue entry");

  for (uint16_t seq = 2;
       seq <= CsrMacCore::ACK_QUEUE_SIZE;
       ++seq)
    {
      device->GetMac ().EnqueueTxFrame (
        BuildExactAck (3, 4, seq), 4, 0, false);
    }

  Require (device->GetMac ().GetAckQueuedFrameCount () ==
             CsrMacCore::ACK_QUEUE_SIZE,
           "ACK queue did not fill to the OPNET 256-entry limit");

  device->GetMac ().EnqueueTxFrame (
    BuildExactAck (3, 4, 257), 4, 0, false);

  Require (device->GetMac ().GetAckQueuedFrameCount () ==
             CsrMacCore::ACK_QUEUE_SIZE,
           "ACK queue accepted an entry beyond the OPNET limit");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  g_sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  g_sender->AddPeer (receiver);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  g_sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReceivedFrame));

  // Enqueue DATA first, then ACKs.  OPNET packs the ACK first and DATA behind
  // it in the same OTA transmission even though DATA was queued earlier.
  g_sender->GetMac ().EnqueueTxFrame (
    BuildData (1, 2, 77), 2, 7, true);
  g_sender->GetMac ().EnqueueTxFrame (
    BuildWindowAck (1, 2, 9, 0x1, 0), 2, 0, false);
  g_sender->GetMac ().EnqueueTxFrame (
    BuildWindowAck (1, 2, 10, 0x3, 0), 2, 0, false);

  Require (g_sender->GetMac ().GetAckQueuedFrameCount () == 1,
           "cumulative ACKs for one destination were not coalesced");
  Require (g_sender->GetMac ().GetDataQueuedFrameCount () == 1,
           "ACK coalescing disturbed the ordinary data queue");

  // ACK destinations now participate in OPNET preamble selection.  Node 1 has
  // not heard node 2, so all seven transmissions legitimately carry the
  // 0.9-second long preamble and need a wider observation window.
  Simulator::Schedule (Seconds (9.0),
                       &CheckCompletedExchange,
                       g_sender);
  Simulator::Stop (Seconds (9.1));
  Simulator::Run ();
  Simulator::Destroy ();

  CheckDeduplicationAndCapacity ();
  Simulator::Destroy ();

  std::cout << "PASS: OPNET MAC ACK queue parity test" << std::endl;
  return 0;
}
