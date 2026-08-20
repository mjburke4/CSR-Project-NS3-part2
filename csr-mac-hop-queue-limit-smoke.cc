#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

uint16_t g_firstReceivedSequence = 0;

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
MakeDataFrame (uint16_t sequence, uint8_t dscp)
{
  CsrHeader header (1, 2, sequence, dscp, false, false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);

  Ptr<Packet> frame = Create<Packet> (1);
  frame->AddHeader (header);
  return frame;
}

void
RecordFirstFrame (Ptr<Packet> frame, double, double)
{
  if (g_firstReceivedSequence != 0)
    {
      return;
    }

  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received MAC frame has no CSR header");
  g_firstReceivedSequence = header.GetSeq ();
}

void
TestMacLimitAndNoDisplacement ()
{
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordFirstFrame));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);

  for (uint16_t sequence = 1;
       sequence <= CsrMacCore::TX_QUEUE_SIZE;
       ++sequence)
    {
      sender->GetMac ().EnqueueTxFrame (
        MakeDataFrame (sequence, 1),
        2,
        1,
        false);
    }

  sender->GetMac ().EnqueueTxFrame (
    MakeDataFrame (513, 7),
    2,
    7,
    false);

  Require (sender->GetMac ().GetDataQueuedFrameCount () == 512,
           "MAC DATA queue did not stop at the OPNET 512-entry limit");
  Require (sender->GetMac ().GetDataQueueDropCount () == 1,
           "MAC did not count exactly one queue-limit drop");

  Simulator::Stop (Seconds (1.75));
  Simulator::Run ();

  Require (g_firstReceivedSequence == 1,
           "overflowing high-DSCP frame displaced an older MAC entry");
  Simulator::Destroy ();
}

void
TestHopOverflowRemainsUntracked ()
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
      uint32_t removed =
        device->GetMac ().CancelAcknowledgedFrames (
          2,
          sequence,
          1,
          0);
      Require (removed == 1,
               "test could not isolate a HOP resend entry from MAC");
    }

  hop->SendData (2, 5, Create<Packet> (1), true);

  Require (hop->GetResendQueueSize () == 512,
           "HOP resend queue exceeded its OPNET limit");
  Require (hop->GetResendQueueOverflowCount () == 1,
           "HOP did not count the rejected 513th resend entry");
  Require (device->GetMac ().GetDataQueuedFrameCount () == 1,
           "HOP overflowed frame was not still forwarded to MAC");
  Require (hop->GetPendingDataCount () == 513,
           "HOP overflow incorrectly rolled back global pending DATA");
  Require (hop->GetOutstandingDataCount (2) == 513,
           "HOP overflow incorrectly rolled back neighbor flow control");

  CsrHeader ackHeader (2, 1, 513, 7, false, true);
  ackHeader.SetType (CSR_PKT_ACK);
  ackHeader.SetDestType (CSR_DEST_UNICAST);
  ackHeader.SetSpeedKey (8);

  Ptr<Packet> ack = Create<Packet> ();
  ack->AddHeader (ackHeader);
  hop->ReceiveFromMac (ack, 0.0, 100.0);

  Require (device->GetMac ().GetDataQueuedFrameCount () == 0,
           "exact ACK did not remove the overflowed MAC copy");
  Require (hop->GetPendingDataCount () == 513,
           "unknown overflow ACK repaired OPNET's pending-count mismatch");
  Require (hop->GetOutstandingDataCount (2) == 513,
           "unknown overflow ACK repaired OPNET's flow-count mismatch");

  Simulator::Destroy ();
}

void
TestMacDropLeavesInitialResendUnarmed ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());

  CsrPerModelFn alwaysDrops =
    [] (int, double, uint32_t) { return 1.0; };
  device->GetPhy ().SetPerModel (alwaysDrops);

  for (uint16_t sequence = 1;
       sequence <= CsrMacCore::TX_QUEUE_SIZE;
       ++sequence)
    {
      device->GetMac ().EnqueueTxFrame (
        MakeDataFrame (sequence, 1),
        2,
        1,
        false);
    }

  hop->SendData (2, 5, Create<Packet> (1), true);

  Require (device->GetMac ().GetDataQueueDropCount () == 1,
           "full MAC queue did not reject the HOP frame");
  Require (hop->GetResendQueueSize () == 1,
           "MAC rejection incorrectly removed the HOP resend entry");

  Simulator::Stop (Seconds (9.0));
  Simulator::Run ();

  Require (hop->GetResendQueueSize () == 1,
           "unconfirmed initial HOP frame retried or timed out");
  Require (hop->GetPendingDataCount () == 1,
           "unconfirmed initial HOP frame released pending flow control");
  Require (device->GetMac ().GetTransmittedFrameCount () > 0,
           "PER=1 incorrectly suppressed all MAC transmissions");
  Require (device->GetMac ().GetDataQueuedFrameCount () <
             CsrMacCore::TX_QUEUE_SIZE,
           "PER=1 prevented the accepted blocker queue from draining");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  TestMacLimitAndNoDisplacement ();
  TestHopOverflowRemainsUntracked ();
  TestMacDropLeavesInitialResendUnarmed ();

  std::cout << "PASS: OPNET MAC/HOP queue-limit parity test"
            << std::endl;
  return 0;
}
