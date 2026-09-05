#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"

#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

using namespace ns3;

struct CsrMacSlotParitySmokeAccess
{
  static uint32_t
  GetConcatByteLimit (const CsrMacCore &mac, int rateKbps)
  {
    return mac.GetConcatByteLimit (rateKbps);
  }

  static bool
  FitsConcatFrame (const CsrMacCore &mac,
                   Ptr<Packet> frame,
                   int frameRateKbps,
                   uint32_t byteCount,
                   int currentRateKbps)
  {
    return mac.FitsConcatFrame (frame,
                                frameRateKbps,
                                byteCount,
                                currentRateKbps);
  }
};

namespace
{

std::vector<uint16_t> g_sequences;
std::vector<double> g_receiveTimes;
std::vector<uint8_t> g_observedAggregateDscp;
Ptr<CsrNetDevice> g_observedSender;

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
BuildFrame (uint16_t seq,
            bool ack,
            uint32_t payloadBytes = 0,
            int rateKbps = 8)
{
  CsrHeader header (1, 2, seq, ack ? 0 : 5, false, ack);
  header.SetType (ack ? CSR_PKT_ACK : CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);
  if (rateKbps != 8)
    {
      header.SetLinkControl (static_cast<uint8_t> (rateKbps),
                             0.0,
                             -90.0);
    }

  Ptr<Packet> frame = Create<Packet> (payloadBytes);
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildProtectedWindowAck (uint16_t seq)
{
  CsrHeader header (1, 2, seq, 0, false, true);
  header.SetType (CSR_PKT_ACK);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);
  header.SetHasAckWindow (true);
  header.SetAckBitmap (0x1);
  header.SetDackBitmap (0);
  header.SetSecurityCount (7);

  // Pairwise16 key/sequence(3) + cumulative logical body(29) + tag(2).
  Ptr<Packet> frame = Create<Packet> (34);
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildAckWithCompatibilityDscp (uint16_t seq, uint8_t dscp)
{
  Ptr<Packet> frame = BuildFrame (seq, true);
  CsrHeader header;
  Require (frame->RemoveHeader (header) > 0,
           "ACK DSCP fixture did not contain a CSR header");
  header.SetDscp (dscp);
  frame->AddHeader (header);
  return frame;
}

void
CheckStrictFitBoundaries ()
{
  CsrMacCore mac;
  const std::vector<std::pair<int, uint32_t>> limits {
    {8, 256},
    {16, 512},
    {32, 1024},
    {64, 2048},
    {128, 4096}
  };

  Ptr<Packet> oneByte = Create<Packet> (1);
  CsrSetOpnetEnvelope (oneByte, CsrOpnetPacketFormat::Mac, 1);
  for (const auto &[rateKbps, limit] : limits)
    {
      Require (CsrMacSlotParitySmokeAccess::GetConcatByteLimit (
                 mac, rateKbps) == limit,
               "source rate-to-concatenation limit map changed");
      Require (CsrMacSlotParitySmokeAccess::FitsConcatFrame (
                 mac, oneByte, rateKbps, limit - 2, rateKbps),
               "aggregate below the source byte limit did not fit");
      Require (!CsrMacSlotParitySmokeAccess::FitsConcatFrame (
                 mac, oneByte, rateKbps, limit - 1, rateKbps),
               "aggregate equal to the source byte limit incorrectly fit");
    }

  // new_pk_fit() uses the slower of the incoming packet and aggregate rate.
  Require (!CsrMacSlotParitySmokeAccess::FitsConcatFrame (
             mac, oneByte, 128, 255, 8),
           "fit check did not use the slower aggregate rate");

  // The supplied source defines no 500/1000-kbit/s packing capacity.  The
  // explicit compatibility behavior admits one queue head but never a second
  // segment, avoiding an invented limit or an indefinitely stuck head.
  for (int unsupportedRate : {500, 1000})
    {
      Require (CsrMacSlotParitySmokeAccess::GetConcatByteLimit (
                 mac, unsupportedRate) == 0,
               "high-rate compatibility path invented a packing limit");
      Require (CsrMacSlotParitySmokeAccess::FitsConcatFrame (
                 mac, oneByte, unsupportedRate, 0, unsupportedRate) &&
                 !CsrMacSlotParitySmokeAccess::FitsConcatFrame (
                   mac, oneByte, unsupportedRate, 1, unsupportedRate),
               "high-rate compatibility path did not isolate the queue head");
    }
}

void
RecordFrame (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received concatenated segment has no CSR header");
  g_sequences.push_back (header.GetSeq ());
  g_receiveTimes.push_back (Simulator::Now ().GetSeconds ());
  if (g_observedSender != nullptr)
    {
      g_observedAggregateDscp.push_back (
        g_observedSender->GetMac ().GetLastAggregateDscp ());
    }
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

  Require (g_observedAggregateDscp.size () == g_sequences.size (),
           "aggregate DSCP observations did not cover every mixed segment");
  for (std::size_t i = 0; i < 5; ++i)
    {
      Require (g_observedAggregateDscp[i] == 5,
               "mixed aggregate did not use maximum DATA-only DSCP");
    }
  for (std::size_t i = 5; i < g_observedAggregateDscp.size (); ++i)
    {
      Require (g_observedAggregateDscp[i] == 0,
               "ACK-only aggregate contributed DSCP metadata");
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
  g_observedAggregateDscp.clear ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordFrame));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);
  g_observedSender = sender;

  // Enqueue DATA first to prove the separate ACK queue is still packed first.
  sender->GetMac ().EnqueueTxFrame (BuildFrame (1, false, 20), 2, 5, false);
  sender->GetMac ().EnqueueTxFrame (BuildFrame (2, false, 20), 2, 5, false);
  sender->GetMac ().EnqueueTxFrame (BuildFrame (3, false, 20), 2, 5, false);
  // Compatibility CsrHeader can carry DSCP on every frame, whereas br_Ack
  // has no such contribution to br_OTA.  Give both ACKs values above DATA to
  // prove the selected aggregate metadata still comes from DATA alone.
  sender->GetMac ().EnqueueTxFrame (
    BuildAckWithCompatibilityDscp (100, 7), 2, 7, false);
  sender->GetMac ().EnqueueTxFrame (
    BuildAckWithCompatibilityDscp (101, 6), 2, 6, false);

  // Unknown ACK receivers require OPNET long preambles, including the four
  // ACK-only retry aggregates after the initial mixed aggregate.
  Simulator::Schedule (Seconds (7.5), &CheckMixedAggregate, sender);
  Simulator::Stop (Seconds (7.6));
  Simulator::Run ();
  Simulator::Destroy ();
  g_observedSender = nullptr;
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
  for (std::size_t i = 1; i < 5; ++i)
    {
      Require (g_receiveTimes[i] == g_receiveTimes[0],
               "8-kbps aggregate did not stop below 256 bytes");
    }
  Require (g_receiveTimes[5] > g_receiveTimes[4],
           "sixth 45-byte OPNET envelope exceeded the strict byte limit");
  Require (g_receiveTimes[6] == g_receiveTimes[5] &&
             g_receiveTimes[7] == g_receiveTimes[5] &&
             g_receiveTimes[8] == g_receiveTimes[5] &&
             g_receiveTimes[9] == g_receiveTimes[5],
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
  // Exact br_Mac + br_Hop sizing makes each preceding frame 45 bytes.  This
  // smaller 25-byte frame would fit behind the first five, but OPNET never
  // scans past the sixth frame after that queue head fails the size test.
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (10, false), 2, 5, false);

  Simulator::Schedule (Seconds (4.5), &CheckByteLimit, sender);
  Simulator::Stop (Seconds (4.6));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
CheckAckWindowByteLimit (Ptr<CsrNetDevice> sender)
{
  Require (g_sequences.size () == 10,
           "cumulative ACK byte-limit scenario lost a queued segment");
  Require (g_sequences[0] == 100 &&
             g_sequences[1] == 1 &&
             g_sequences[2] == 2 &&
             g_sequences[3] == 3 &&
             g_sequences[4] == 4,
           "46-byte cumulative ACK did not precede exactly four DATA frames");
  for (std::size_t i = 1; i < 5; ++i)
    {
      Require (g_receiveTimes[i] == g_receiveTimes[0],
               "first cumulative-ACK aggregate split below 214 bytes");
    }
  Require (g_sequences[5] == 100 && g_sequences[6] == 5 &&
             g_receiveTimes[5] > g_receiveTimes[4] &&
             g_receiveTimes[6] == g_receiveTimes[5],
           "fifth DATA frame incorrectly fit beside the 46-byte ACK");
  Require (sender->GetMac ().GetTransmittedFrameCount () == 5,
           "cumulative ACK did not retain its five-transmission budget");
  Require (sender->GetMac ().GetQueuedFrameCount () == 0,
           "cumulative ACK byte-limit scenario left frames queued");
}

void
RunAckWindowByteLimitScenario ()
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

  for (uint16_t seq = 1; seq <= 5; ++seq)
    {
      sender->GetMac ().EnqueueTxFrame (
        BuildFrame (seq, false, 17), 2, 5, false);
    }

  Ptr<Packet> ack = BuildProtectedWindowAck (100);
  Require (CsrAnnotateOpnetEnvelope (ack) &&
             CsrGetOpnetWireSize (ack) == 46,
           "protected cumulative ACK did not enter MAC as 46 bytes");
  sender->GetMac ().EnqueueTxFrame (ack, 2, 0, false);

  // At 8 kbit/s the strict limit is <256 bytes.  The first aggregate is one
  // 46-byte ACK plus four 42-byte DATA frames (214 bytes); a fifth DATA frame
  // would reach the strict 256-byte limit and must remain queued for the next
  // wake.  A wrong 41-byte cumulative ACK would total only 251 bytes and fit.
  Simulator::Schedule (Seconds (7.5),
                       &CheckAckWindowByteLimit,
                       sender);
  Simulator::Stop (Seconds (7.6));
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
        BuildFrame (seq, false, 0, 128), 2, 5, false);
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
  CheckStrictFitBoundaries ();
  RunMixedAckDataScenario ();
  RunByteLimitScenario ();
  RunAckWindowByteLimitScenario ();
  RunSegmentLimitScenario ();
  std::cout << "PASS: OPNET MAC packet concatenation parity test"
            << std::endl;
  return 0;
}
