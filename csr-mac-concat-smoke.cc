#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

std::vector<uint16_t> g_sequences;
std::vector<double> g_receiveTimes;

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
BuildFrame (uint16_t seq, bool ack, uint32_t payloadBytes = 0)
{
  CsrHeader header (1, 2, seq, ack ? 0 : 5, false, ack);
  header.SetType (ack ? CSR_PKT_ACK : CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);

  Ptr<Packet> frame = Create<Packet> (payloadBytes);
  frame->AddHeader (header);
  return frame;
}

void
RecordFrame (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received concatenated segment has no CSR header");
  g_sequences.push_back (header.GetSeq ());
  g_receiveTimes.push_back (Simulator::Now ().GetSeconds ());
}

void
CheckMixedAggregate (Ptr<CsrNetDevice> sender)
{
  Require (g_sequences.size () == 13,
           "mixed aggregate did not deliver ten ACK and three DATA segments");
  Require (g_sequences[0] == 100 && g_sequences[1] == 101 &&
             g_sequences[2] == 1 && g_sequences[3] == 2 &&
             g_sequences[4] == 3,
           "first aggregate did not pack ACKs before queued DATA");

  for (std::size_t i = 5; i < g_sequences.size (); i += 2)
    {
      Require (g_sequences[i] == 100 && g_sequences[i + 1] == 101,
               "ACK retransmission aggregate changed queue order");
    }

  for (std::size_t i = 1; i < 5; ++i)
    {
      Require (g_receiveTimes[i] == g_receiveTimes[0],
               "first five segments did not share one OTA reception time");
    }

  Require (sender->GetMac ().GetTransmittedFrameCount () == 5,
           "mixed traffic did not use five OPNET aggregate transmissions");
  Require (sender->GetMac ().GetQueuedFrameCount () == 0,
           "mixed aggregate queues did not drain");
}

void
RunMixedAckDataScenario ()
{
  g_sequences.clear ();
  g_receiveTimes.clear ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordFrame));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);

  // Enqueue DATA first to prove the separate ACK queue is still packed first.
  sender->GetMac ().EnqueueTxFrame (BuildFrame (1, false, 20), 2, 5, false);
  sender->GetMac ().EnqueueTxFrame (BuildFrame (2, false, 20), 2, 5, false);
  sender->GetMac ().EnqueueTxFrame (BuildFrame (3, false, 20), 2, 5, false);
  sender->GetMac ().EnqueueTxFrame (BuildFrame (100, true), 2, 0, false);
  sender->GetMac ().EnqueueTxFrame (BuildFrame (101, true), 2, 0, false);

  // Unknown ACK receivers require OPNET long preambles, including the four
  // ACK-only retry aggregates after the initial mixed aggregate.
  Simulator::Schedule (Seconds (7.5), &CheckMixedAggregate, sender);
  Simulator::Stop (Seconds (7.6));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
CheckSegmentLimit (Ptr<CsrNetDevice> sender)
{
  Require (g_sequences.size () == 17,
           "17 queued DATA frames were not delivered");
  for (std::size_t i = 1; i < 16; ++i)
    {
      Require (g_receiveTimes[i] == g_receiveTimes[0],
               "first OTA aggregate did not contain 16 DATA segments");
    }
  Require (g_receiveTimes[16] > g_receiveTimes[15],
           "17th DATA segment bypassed OPNET's 16-segment limit");
  Require (sender->GetMac ().GetTransmittedFrameCount () == 2,
           "17 DATA frames did not require exactly two OTA transmissions");
  Require (sender->GetMac ().GetQueuedFrameCount () == 0,
           "segment-limit scenario left frames queued");
}

void
CheckByteLimit (Ptr<CsrNetDevice> sender)
{
  Require (g_sequences.size () == 10,
           "byte-limit scenario did not deliver all DATA frames");
  for (std::size_t i = 1; i < 7; ++i)
    {
      Require (g_receiveTimes[i] == g_receiveTimes[0],
               "8-kbps aggregate did not stop below 256 bytes");
    }
  Require (g_receiveTimes[7] > g_receiveTimes[6],
           "eighth 33-byte frame exceeded OPNET's strict byte limit");
  Require (g_receiveTimes[8] == g_receiveTimes[7] &&
             g_receiveTimes[9] == g_receiveTimes[7],
           "smaller tail frame bypassed the non-fitting queue head");
  Require (sender->GetMac ().GetTransmittedFrameCount () == 2,
           "byte-limit scenario did not use exactly two OTA transmissions");
}

void
RunByteLimitScenario ()
{
  g_sequences.clear ();
  g_receiveTimes.clear ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordFrame));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);

  for (uint16_t seq = 1; seq <= 9; ++seq)
    {
      sender->GetMac ().EnqueueTxFrame (
        BuildFrame (seq, false, 20), 2, 5, false);
    }
  // This smaller frame would fit behind the first seven, but OPNET never
  // scans past the eighth frame after that queue head fails the size test.
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (10, false), 2, 5, false);

  Simulator::Schedule (Seconds (4.5), &CheckByteLimit, sender);
  Simulator::Stop (Seconds (4.6));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
RunSegmentLimitScenario ()
{
  g_sequences.clear ();
  g_receiveTimes.clear ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordFrame));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);

  for (uint16_t seq = 1; seq <= 17; ++seq)
    {
      sender->GetMac ().EnqueueTxFrame (
        BuildFrame (seq, false), 2, 5, false);
    }

  Simulator::Schedule (Seconds (3.5), &CheckSegmentLimit, sender);
  Simulator::Stop (Seconds (3.6));
  Simulator::Run ();
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  RunMixedAckDataScenario ();
  RunByteLimitScenario ();
  RunSegmentLimitScenario ();
  std::cout << "PASS: OPNET MAC packet concatenation parity test"
            << std::endl;
  return 0;
}
