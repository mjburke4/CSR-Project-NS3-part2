#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

struct CsrMacSlotParitySmokeAccess
{
  static void
  LoadLiveReservation (CsrMacCore &mac, int32_t slot)
  {
    mac.m_scheduledTxSlot = slot;
    mac.m_txCountdownCounter = slot;
  }

  static uint32_t
  GetFirstDataWireBytes (const CsrMacCore &mac)
  {
    return mac.m_queue.empty ()
             ? 0
             : CsrGetOpnetWireSize (mac.m_queue.front ().frame);
  }
};

namespace
{

Ptr<CsrNetDevice> g_sender;
Ptr<CsrNetDevice> g_receiver;
std::vector<double> g_txTimes;
std::vector<int32_t> g_usedSlots;
std::vector<int32_t> g_advertisedSlots;
uint32_t g_peerObservations = 0;
bool g_unknownFirstReservationIgnored = false;
bool g_knownNeighborReservationRetained = false;
double g_shortFrameDuration = 0.0;
double g_expiredEnqueueTime = -1.0;
Ptr<CsrNetDevice> g_idleDevice;
double g_idleTxTime = -1.0;

Ptr<CsrNetDevice> g_hopDevice;
Ptr<CsrHopLayer> g_hop;
std::vector<double> g_hopTxTimes;
std::vector<int32_t> g_hopUsedSlots;
std::vector<int32_t> g_hopAdvertisedSlots;

Ptr<CsrNetDevice> g_syncDevice;
Ptr<CsrNetDevice> g_zeroDevice;
Ptr<CsrNetDevice> g_trackDevice;
Ptr<CsrNetDevice> g_idleZeroDevice;
Ptr<CsrNetDevice> g_packingRetryDevice;
Ptr<CsrNetDevice> g_noSignalDevice;
int32_t g_trackAdvertisedSlot = -1;
int64_t g_trackExpectedTxNs = -1;
Time g_noSignalTxTime = Time::Min ();

Ptr<CsrNetDevice> g_chainSourceDevice;
Ptr<CsrNetDevice> g_chainReceiverDevice;
Ptr<CsrHopLayer> g_chainSourceHop;
Ptr<CsrHopLayer> g_chainReceiverHop;
Ptr<CsrNetLayer> g_chainSourceNwk;
Ptr<CsrNetLayer> g_chainReceiverNwk;
Time g_chainDataTxTime = Time::Min ();
Time g_chainDataDeliveryTime = Time::Min ();
Time g_chainAckTxTime = Time::Min ();
Time g_chainAckCompletionTime = Time::Min ();
uint32_t g_chainAckWireBytes = 0;
uint32_t g_chainDeliveries = 0;
bool g_chainAckHasPairwiseTransport = false;
bool g_chainAckHasWindow = false;

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
RequireTime (Time actual, Time expected, const char* message)
{
  if (actual != expected)
    {
      std::cerr << "FAIL: " << message
                << " actual_ns=" << actual.GetNanoSeconds ()
                << " expected_ns=" << expected.GetNanoSeconds ()
                << std::endl;
      std::exit (1);
    }
}

Ptr<Packet>
BuildDataFrame (uint16_t sequence, uint32_t payloadBytes = 20)
{
  CsrHeader header (1, 2, sequence, 5, true, false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);

  Ptr<Packet> frame = Create<Packet> (payloadBytes);
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildFixedRateDataFrame (uint16_t sequence,
                         uint32_t payloadBytes,
                         CsrRateKey rateKbps)
{
  Ptr<Packet> frame = BuildDataFrame (sequence, payloadBytes);
  CsrHeader header;
  frame->RemoveHeader (header);
  header.SetLinkControl (rateKbps, 0.0, 0.0);
  frame->AddHeader (header);
  return frame;
}

double
GetShortFrameDuration ()
{
  Ptr<Packet> frame = BuildDataFrame (99);
  uint32_t wireBytes = CsrGetOpnetWireSize (frame);
  uint32_t s0HeaderBits = 104 + 16 + 16 + 16;
  uint32_t payloadAndFcsBits = wireBytes * 8 + 32;
  return s0HeaderBits / 4.0 * 0.00051
       + payloadAndFcsBits / CsrRateKeyToBps (128);
}

void
EnqueueDuringTx ()
{
  Require (g_sender->GetMacState () == CsrMacCore::State::TX,
           "during-TX enqueue did not occur in Tx_st");
  Require (g_sender->GetMac ().GetLocalReservationCounter () ==
             g_advertisedSlots.front (),
           "advertised reservation changed while transmitter was active");
  g_sender->GetMac ().EnqueueTxFrame (
    BuildDataFrame (2), 2, 5, true);
}

void
EnqueueAfterTxBeforeExpiry ()
{
  if (g_sender->GetMacState () == CsrMacCore::State::TX)
    {
      Simulator::Schedule (MicroSeconds (1),
                           &EnqueueAfterTxBeforeExpiry);
      return;
    }
  Require (g_sender->GetMacState () == CsrMacCore::State::SEARCH,
           "post-TX enqueue did not occur after return to Search");
  Require (g_sender->GetMac ().GetLocalReservationCounter () ==
             g_advertisedSlots[1],
           "empty-queue Search did not retain the advertised reservation");
  g_sender->GetMac ().EnqueueTxFrame (
    BuildDataFrame (3), 2, 5, true);
}

void
WaitForReservationExpiry ()
{
  if (g_sender->GetMacState () != CsrMacCore::State::SEARCH ||
      g_sender->GetMac ().GetLocalReservationCounter () >= 0)
    {
      Simulator::Schedule (MicroSeconds (100),
                           &WaitForReservationExpiry);
      return;
    }

  g_expiredEnqueueTime = Simulator::Now ().GetSeconds ();
  g_sender->GetMac ().EnqueueTxFrame (
    BuildDataFrame (4), 2, 5, true);
}

void
RecordLifecycleTransmission (CsrNodeId destination,
                             uint16_t sequence,
                             Time sentTime)
{
  Require (destination == 2,
           "lifecycle transmission used the wrong destination");
  Require (sequence == g_txTimes.size () + 1,
           "lifecycle transmissions changed sequence order");

  g_txTimes.push_back (sentTime.GetSeconds ());
  g_usedSlots.push_back (
    g_sender->GetMac ().GetLastTxOpportunitySlot ());
  g_advertisedSlots.push_back (
    g_sender->GetMac ().GetLastAdvertisedReservationSlot ());

  if (sequence == 1)
    {
      int64_t txNanoseconds = sentTime.GetNanoSeconds ();
      Require (txNanoseconds % 13000000 == 0,
               "first TX moved off the TSLOT phase established at startup");
      Require (sentTime >= MilliSeconds (325) &&
                 sentTime <= MilliSeconds (559),
               "first TX fell outside the source holdoff/slot window");
      Simulator::Schedule (MilliSeconds (1), &EnqueueDuringTx);
    }
  else if (sequence == 2)
    {
      Require (g_usedSlots[1] == g_advertisedSlots[0],
               "frame queued during TX did not reuse the advertised slot");
      Simulator::Schedule (Seconds (g_shortFrameDuration) + MicroSeconds (1),
                           &EnqueueAfterTxBeforeExpiry);
    }
  else if (sequence == 3)
    {
      Require (g_usedSlots[2] == g_advertisedSlots[1],
               "post-TX arrival did not reuse the live advertised slot");
      Simulator::Schedule (Seconds (g_shortFrameDuration) + MicroSeconds (1),
                           &WaitForReservationExpiry);
    }
  else if (sequence == 4)
    {
      Require (g_expiredEnqueueTime >= 0.0,
               "expired-reservation enqueue time was not recorded");
      Require (sentTime.GetSeconds () - g_expiredEnqueueTime < 0.270,
               "expired reservation incorrectly restarted the 300-ms holdoff");
      Simulator::Stop (MilliSeconds (30));
    }
}

void
ObservePeerReservation (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "peer could not decode the lifecycle frame header");
  uint16_t sequence = header.GetSeq ();
  Require (sequence >= 1 && sequence <= g_advertisedSlots.size (),
           "peer observed an unexpected lifecycle sequence");
  int32_t peerCounter =
    g_receiver->GetMac ().GetNeighborReservationCounter (1);
  if (sequence == 1)
    {
      // br_mac applies Reserve TSlot before its later ACKable-reception path
      // adds an unknown sender to the neighbor table.  The first-contact
      // advertisement is therefore intentionally ignored.
      Require (peerCounter == -1,
               "unknown peer retained its first-contact reservation");
      g_unknownFirstReservationIgnored = true;
    }
  else
    {
      Require (peerCounter == g_advertisedSlots[sequence - 1],
               "known peer did not retain the exact advertised next slot");
      if (sequence == 2)
        {
          g_knownNeighborReservationRetained = true;
        }
    }
  g_peerObservations++;
}

void
CheckNoTxBeforeHoldoff ()
{
  Require (g_sender->GetMac ().GetTransmittedFrameCount () == 0,
           "MAC transmitted before the initial 300-ms holdoff");
}

void
RecordIdleTransitionTransmission (CsrNodeId,
                                  uint16_t,
                                  Time sentTime)
{
  g_idleTxTime = sentTime.GetSeconds ();
  Simulator::Stop (MilliSeconds (1));
}

void
EnqueueFromIdle ()
{
  Require (g_idleDevice->GetMacState () == CsrMacCore::State::IDLE,
           "non-grid enqueue did not begin in Idle");
  g_idleDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1), 2, 5, true);
}

void
CheckIdleHoldoff ()
{
  Require (g_idleDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "Idle-to-Search activation did not restart the 300-ms holdoff");
}

void
CheckIdleQueueWaitsForRtsTick ()
{
  Require (g_idleDevice->GetMacState () == CsrMacCore::State::IDLE,
           "Idle HOP arrival entered Search before its scheduled RTS TSLOT");
  Require (g_idleDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "Idle traffic transmitted before its scheduled RTS TSLOT");
}

void
CheckIdleRtsActivation ()
{
  Require (g_idleDevice->GetMacState () == CsrMacCore::State::SEARCH,
           "Idle RTS TSLOT did not run prep_tx and enter Search");
  Require (g_idleDevice->GetMac ().GetLocalReservationCounter () > 0,
           "Idle prep_tx did not select a positive reservation slot");
  Require (g_idleDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "Idle RTS activation bypassed its 300-ms holdoff");
}

void
TestMacReservationLifecycle ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);

  g_txTimes.clear ();
  g_usedSlots.clear ();
  g_advertisedSlots.clear ();
  g_peerObservations = 0;
  g_unknownFirstReservationIgnored = false;
  g_knownNeighborReservationRetained = false;
  g_expiredEnqueueTime = -1.0;
  g_shortFrameDuration = GetShortFrameDuration ();

  g_sender = CreateObject<CsrNetDevice> (1);
  g_receiver = CreateObject<CsrNetDevice> (2);
  g_sender->AddPeer (g_receiver);
  g_receiver->AddPeer (g_sender);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  g_sender->GetPhy ().SetPerModel (noErrors);
  g_receiver->GetPhy ().SetPerModel (noErrors);

  g_sender->GetMac ().NoteHeardFrom (2, 0.0);
  g_sender->GetMac ().SetNeighborPathloss (2, 0.0);
  g_sender->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordLifecycleTransmission));
  g_receiver->GetMac ().SetRxCallback (
    MakeCallback (&ObservePeerReservation));

  // Both clocks start together at time zero.  Repeating StartSlotTick() must
  // be idempotent rather than moving either node's established phase.
  g_sender->GetMac ().StartSlotTick (MilliSeconds (13));
  g_receiver->GetMac ().StartSlotTick (MilliSeconds (13));
  g_sender->GetMac ().StartSlotTick (MilliSeconds (13));

  g_sender->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1), 2, 5, true);
  Simulator::Schedule (MilliSeconds (299), &CheckNoTxBeforeHoldoff);

  Simulator::Stop (Seconds (3.0));
  Simulator::Run ();

  Require (g_txTimes.size () == 4,
           "reservation lifecycle did not complete four transmissions");
  Require (g_peerObservations == 4,
           "peer did not observe every advertised reservation");
  Require (g_unknownFirstReservationIgnored,
           "first-contact reservation ordering was not exercised");
  Require (g_knownNeighborReservationRetained,
           "known-neighbor reservation retention was not exercised");

  Simulator::Destroy ();
  g_sender = nullptr;
  g_receiver = nullptr;
}

void
TestIdleRtsStartsHoldoff ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (3);
  g_idleTxTime = -1.0;

  g_idleDevice = CreateObject<CsrNetDevice> (1);
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  g_idleDevice->GetPhy ().SetPerModel (noErrors);
  g_idleDevice->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordIdleTransitionTransmission));
  g_idleDevice->EnableDutyCycling (true);
  g_idleDevice->SetDutyCyclePhase (MilliSeconds (200));
  Require (g_idleDevice->GetMacState () == CsrMacCore::State::IDLE,
           "deterministic pending-TX wake test did not begin in Idle");

  // br_mac queues HOP_PK_RCVD at 5 ms, then Idle's enter executive schedules
  // its RTS on the next simulation-wide TSLOT at 13 ms. prep_tx establishes
  // the continuing 13-ms phase and a fresh holdoff ending at 313 ms. The
  // periodic WAKE at 200 ms is deliberately far from the RTS boundary.
  Simulator::Schedule (MilliSeconds (5), &EnqueueFromIdle);
  Simulator::Schedule (MilliSeconds (6), &CheckIdleQueueWaitsForRtsTick);
  Simulator::Schedule (MilliSeconds (14), &CheckIdleRtsActivation);
  Simulator::Schedule (MilliSeconds (312), &CheckIdleHoldoff);
  Simulator::Stop (Seconds (0.6));
  Simulator::Run ();

  Require (g_idleTxTime >= 0.338 && g_idleTxTime <= 0.559,
           "Idle-to-Search TX fell outside the holdoff/slot window");
  int64_t txNanoseconds =
    static_cast<int64_t> (std::llround (g_idleTxTime * 1e9));
  Require (txNanoseconds % 13000000 == 0,
           "Idle RTS did not retain the simulation-wide TSLOT phase");

  Simulator::Destroy ();
  g_idleDevice = nullptr;
}

void
RecordHopTransmission (CsrNodeId destination,
                       uint16_t sequence,
                       Time sentTime)
{
  g_hopUsedSlots.push_back (
    g_hopDevice->GetMac ().GetLastTxOpportunitySlot ());
  g_hopAdvertisedSlots.push_back (
    g_hopDevice->GetMac ().GetLastAdvertisedReservationSlot ());
  g_hopTxTimes.push_back (sentTime.GetSeconds ());

  // Preserve the production callback while adding test observation.
  g_hop->NotifyMacFrameSent (destination, sequence, sentTime);

  if (g_hopTxTimes.size () == 2)
    {
      Require (g_hopUsedSlots[1] == g_hopAdvertisedSlots[0],
               "HOP retry did not consume the first advertised reservation");
      double delta = g_hopTxTimes[1] - g_hopTxTimes[0];
      Require (delta >= 2.015 && delta <= 2.236,
               "HOP retry timing indicates a restarted holdoff or lost reservation");
      Simulator::Stop (MilliSeconds (1));
    }
}

void
TestHopRetryUsesLiveReservation ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (2);
  g_hopTxTimes.clear ();
  g_hopUsedSlots.clear ();
  g_hopAdvertisedSlots.clear ();

  g_hopDevice = CreateObject<CsrNetDevice> (1);
  g_hop = CreateObject<CsrHopLayer> ();
  g_hop->SetNodeId (1);
  g_hop->SetMac (&g_hopDevice->GetMac ());
  g_hopDevice->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordHopTransmission));

  CsrPerModelFn alwaysDrops =
    [] (int, double, uint32_t) { return 1.0; };
  g_hopDevice->GetPhy ().SetPerModel (alwaysDrops);

  // This produces a 960-byte modeled OTA frame at 8 kbit/s.  Its long-
  // preamble airtime is 1.99512 s, so the real two-second HOP retry arrives
  // after FinishTx but before the next 13-ms slot tick.
  Ptr<Packet> payload = Create<Packet> (923);
  CsrNetHeader networkHeader (1, 3, 5);
  payload->AddHeader (networkHeader);
  g_hop->SendData (2, 5, payload, true);

  Simulator::Stop (Seconds (3.2));
  Simulator::Run ();

  Require (g_hopTxTimes.size () == 2,
           "HOP active-reservation test did not reach its first retry");

  Simulator::Destroy ();
  g_hop = nullptr;
  g_hopDevice = nullptr;
}

void
BeginSyncActivation ()
{
  Require (g_syncDevice->GetMac ().GetNeighborReservationCounter (2) == -2,
           "expired neighbor counter did not continue below -1");
  g_syncDevice->GetMac ().SetSyncPresent (true);
  g_syncDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1), 2, 5, true);
}

void
CheckSyncActivation ()
{
  Require (g_syncDevice->GetMac ().GetLocalReservationCounter () > 0,
           "Search-state enqueue did not activate PREP_TX under SYNC");
  Require (g_syncDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "MAC transmitted while synchronization was present");
  Require (g_syncDevice->GetMac ().GetNeighborReservationCounter (2) == -2,
           "neighbor counter advanced while synchronization was present");
}

void
TestSearchSyncActivation ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (4);

  g_syncDevice = CreateObject<CsrNetDevice> (1);
  g_syncDevice->GetMac ().NoteHeardFrom (2, 0.0);
  Require (g_syncDevice->GetMac ().GetNeighborReservationCounter (2) == -1,
           "new neighbor began with a phantom slot-zero reservation");

  g_syncDevice->GetMac ().StartSlotTick (MilliSeconds (13));
  Simulator::Schedule (MilliSeconds (14), &BeginSyncActivation);
  Simulator::Schedule (MilliSeconds (28), &CheckSyncActivation);
  Simulator::Stop (MilliSeconds (30));
  Simulator::Run ();

  Simulator::Destroy ();
  g_syncDevice = nullptr;
}

void WaitForZeroReservation ();
void CheckZeroReservationRedraw ();

void
RecordZeroReservationTransmission (CsrNodeId,
                                   uint16_t sequence,
                                   Time)
{
  Require (sequence == 1,
           "zero-reservation test transmitted the queued second frame early");
  Simulator::Schedule (Seconds (g_shortFrameDuration) + MicroSeconds (1),
                       &WaitForZeroReservation);
}

void
WaitForZeroReservation ()
{
  if (g_zeroDevice->GetMacState () != CsrMacCore::State::SEARCH ||
      g_zeroDevice->GetMac ().GetLocalReservationCounter () != 0)
    {
      Simulator::Schedule (MicroSeconds (100),
                           &WaitForZeroReservation);
      return;
    }

  g_zeroDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (2), 2, 5, true);
  Simulator::Schedule (MicroSeconds (100),
                       &CheckZeroReservationRedraw);
}

void
CheckZeroReservationRedraw ()
{
  int32_t counter =
    g_zeroDevice->GetMac ().GetLocalReservationCounter ();
  if (counter == 0)
    {
      Simulator::Schedule (MicroSeconds (100),
                           &CheckZeroReservationRedraw);
      return;
    }

  Require (counter > 0,
           "Search arrival reused or failed to redraw an expiring zero reservation");
  Require (g_zeroDevice->GetMac ().GetTransmittedFrameCount () == 1,
           "Search arrival transmitted on the expired zero reservation");
  Simulator::Stop (MicroSeconds (1));
}

void
TestSearchZeroExpiresBeforeActivation ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (5);

  g_zeroDevice = CreateObject<CsrNetDevice> (1);
  g_zeroDevice->GetMac ().NoteHeardFrom (2, 0.0);
  g_zeroDevice->GetMac ().SetNeighborPathloss (2, 0.0);
  g_zeroDevice->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordZeroReservationTransmission));
  g_zeroDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1), 2, 5, true);

  Simulator::Stop (Seconds (1.5));
  Simulator::Run ();

  Require (g_zeroDevice->GetMac ().GetTransmittedFrameCount () == 1,
           "zero-reservation ordering test did not stop before its redraw");

  Simulator::Destroy ();
  g_zeroDevice = nullptr;
}

void WaitForTrackCounterZero ();

void
RecordTrackReturnTransmission (CsrNodeId,
                               uint16_t sequence,
                               Time sentTime)
{
  if (sequence == 1)
    {
      g_trackAdvertisedSlot =
        g_trackDevice->GetMac ().GetLastAdvertisedReservationSlot ();
      Simulator::Schedule (Seconds (g_shortFrameDuration) + MicroSeconds (1),
                           &WaitForTrackCounterZero);
      return;
    }

  Require (sequence == 2,
           "Track-return test changed queued sequence order");
  Require (sentTime.GetNanoSeconds () == g_trackExpectedTxNs,
           "Track-to-Search return did not activate PREP_TX immediately");
  Require (g_trackDevice->GetMac ().GetLastTxOpportunitySlot () ==
             g_trackAdvertisedSlot,
           "Track-to-Search return did not reuse the live zero reservation");
  Simulator::Stop (MicroSeconds (1));
}

void
WaitForTrackCounterZero ()
{
  if (g_trackDevice->GetMacState () != CsrMacCore::State::SEARCH ||
      g_trackDevice->GetMac ().GetLocalReservationCounter () != 0)
    {
      Simulator::Schedule (MicroSeconds (100),
                           &WaitForTrackCounterZero);
      return;
    }

  g_trackDevice->GetMac ().SetReceiveState (CsrMacCore::State::TRACK);
  g_trackDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (2), 2, 5, true);

  int64_t nowSteps = Simulator::Now ().GetTimeStep ();
  int64_t periodSteps = MilliSeconds (13).GetTimeStep ();
  g_trackExpectedTxNs =
    TimeStep ((nowSteps / periodSteps + 1) * periodSteps).GetNanoSeconds ();

  g_trackDevice->GetMac ().SetReceiveState (CsrMacCore::State::SEARCH);
}

void
TestTrackReturnActivatesImmediately ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (6);
  g_trackAdvertisedSlot = -1;
  g_trackExpectedTxNs = -1;

  g_trackDevice = CreateObject<CsrNetDevice> (1);
  g_trackDevice->GetMac ().NoteHeardFrom (2, 0.0);
  g_trackDevice->GetMac ().SetNeighborPathloss (2, 0.0);
  g_trackDevice->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordTrackReturnTransmission));
  g_trackDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1), 2, 5, true);

  Simulator::Stop (Seconds (1.5));
  Simulator::Run ();

  Require (g_trackDevice->GetMac ().GetTransmittedFrameCount () == 2,
           "Track-return test did not complete the reused opportunity");

  Simulator::Destroy ();
  g_trackDevice = nullptr;
}

void WaitForIdleZeroCounter ();

void
CheckIdleZeroRedraw ()
{
  Require (g_idleZeroDevice->GetMacState () == CsrMacCore::State::SEARCH,
           "Idle zero-reservation RTS did not enter Search");
  Require (g_idleZeroDevice->GetMac ().GetLocalReservationCounter () > 0,
           "Idle prep_tx reused counter zero instead of redrawing it");
  Require (g_idleZeroDevice->GetMac ().GetTransmittedFrameCount () == 1,
           "Idle zero-reservation traffic bypassed its restarted holdoff");
  Simulator::Stop (MicroSeconds (1));
}

void
CheckIdleZeroQueued ()
{
  Require (g_idleZeroDevice->GetMacState () == CsrMacCore::State::IDLE,
           "queued zero-reservation traffic entered Search before RTS");
  Require (g_idleZeroDevice->GetMac ().GetLocalReservationCounter () == 0,
           "Idle queueing changed the live zero counter");
  Require (g_idleZeroDevice->GetMac ().GetTransmittedFrameCount () == 1,
           "Idle queueing transmitted the second frame");
}

void
RecordIdleZeroFirstTransmission (CsrNodeId,
                                 uint16_t sequence,
                                 Time)
{
  Require (sequence == 1,
           "Idle-zero test transmitted its queued second frame early");
  Simulator::Schedule (Seconds (g_shortFrameDuration) + MicroSeconds (1),
                       &WaitForIdleZeroCounter);
}

void
WaitForIdleZeroCounter ()
{
  if (g_idleZeroDevice->GetMacState () != CsrMacCore::State::SEARCH ||
      g_idleZeroDevice->GetMac ().GetLocalReservationCounter () != 0)
    {
      Simulator::Schedule (MicroSeconds (100),
                           &WaitForIdleZeroCounter);
      return;
    }

  g_idleZeroDevice->GetMac ().SetReceiveState (CsrMacCore::State::IDLE);
  g_idleZeroDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (2), 2, 5, true);

  Simulator::Schedule (MicroSeconds (1),
                       &CheckIdleZeroQueued);

  int64_t nowSteps = Simulator::Now ().GetTimeStep ();
  int64_t periodSteps = MilliSeconds (13).GetTimeStep ();
  Time nextGlobalSlot = TimeStep (
    (nowSteps / periodSteps + 1) * periodSteps);
  Simulator::Schedule (nextGlobalSlot - Simulator::Now () + MicroSeconds (1),
                       &CheckIdleZeroRedraw);
}

void
TestIdleZeroRedrawsAtRts ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (7);

  g_idleZeroDevice = CreateObject<CsrNetDevice> (1);
  g_idleZeroDevice->GetMac ().NoteHeardFrom (2, 0.0);
  g_idleZeroDevice->GetMac ().SetNeighborPathloss (2, 0.0);
  g_idleZeroDevice->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordIdleZeroFirstTransmission));
  g_idleZeroDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1), 2, 5, true);

  Simulator::Stop (Seconds (1.5));
  Simulator::Run ();

  Require (g_idleZeroDevice->GetMac ().GetTransmittedFrameCount () == 1,
           "Idle-zero reservation test did not stop after prep_tx redraw");

  Simulator::Destroy ();
  g_idleZeroDevice = nullptr;
}

void
EnterIdleDuringPackingRetry ()
{
  Require (g_packingRetryDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "oversized aggregate unexpectedly entered the transmitter");
  Require (g_packingRetryDevice->GetMac ().CancelAcknowledgedFrames (
             2, 2, 3, 0) == 2,
           "packing-retry setup did not clear both queued frames");
  g_packingRetryDevice->GetMac ().SetReceiveState (
    CsrMacCore::State::IDLE);
}

void
CheckPackingRetryStayedIdle ()
{
  Require (g_packingRetryDevice->GetMacState () == CsrMacCore::State::IDLE,
           "packing retry changed the receiver out of Idle");
  Require (g_packingRetryDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "delayed packing retry bypassed Idle/holdoff reservation gates");
}

void
TestPackingRetryRespectsMacState ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (8);

  g_packingRetryDevice = CreateObject<CsrNetDevice> (1);
  g_packingRetryDevice->GetMac ().NoteHeardFrom (2, 0.0);
  g_packingRetryDevice->GetMac ().SetNeighborPathloss (2, 130.0);
  g_packingRetryDevice->GetMac ().EnqueueTxFrame (
    BuildFixedRateDataFrame (1, 300, 8), 2, 5, true);
  g_packingRetryDevice->GetMac ().EnqueueTxFrame (
    BuildFixedRateDataFrame (2, 300, 8), 2, 5, true);

  Simulator::Schedule (MilliSeconds (700),
                       &EnterIdleDuringPackingRetry);
  Simulator::Schedule (Seconds (2.0),
                       &CheckPackingRetryStayedIdle);
  Simulator::Stop (Seconds (2.1));
  Simulator::Run ();

  Simulator::Destroy ();
  g_packingRetryDevice = nullptr;
}

void
CheckOpnetWakeEnteredSearch ()
{
  Require (g_noSignalDevice->GetMacState () == CsrMacCore::State::SEARCH,
           "OPNET periodic WAKE did not enter Search");
}

void
CheckNoSignalSearchReturnedIdle ()
{
  Require (g_noSignalDevice->GetMacState () == CsrMacCore::State::IDLE,
           "no-signal Search did not return to Idle after 8.9 ms");
}

void ControlLongQuietReservationSlot ();

void
EnqueueAfterLongQuiet ()
{
  Require (g_noSignalDevice->GetMacState () == CsrMacCore::State::IDLE,
           "receiver was not Idle after the long no-signal interval");
  g_noSignalDevice->GetMac ().SetReservationSlotOverrideForDifferentialRun (
    13);
  g_noSignalDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1), 2, 5, true);
  // Schedule after EnqueueTxFrame() creates Idle's 300.001-s RTS event. Equal-
  // time event ordering therefore runs prep_tx first, then this test callback.
  Simulator::Schedule (MilliSeconds (1),
                       &ControlLongQuietReservationSlot);
}

void
CheckLongQuietQueueWaitsForRts ()
{
  Require (g_noSignalDevice->GetMacState () == CsrMacCore::State::IDLE,
           "long-quiet enqueue bypassed the Idle RTS gate");
  Require (g_noSignalDevice->GetMac ().GetQueuedFrameCount () == 1,
           "long-quiet frame was not retained while Idle");
  Require (g_noSignalDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "long-quiet frame transmitted before prep_tx");
}

void
ControlLongQuietReservationSlot ()
{
  Require (g_noSignalDevice->GetMacState () == CsrMacCore::State::SEARCH,
           "Idle RTS did not return the long-quiet receiver to Search");
  Require (g_noSignalDevice->GetMac ().IsTxPreparationActive (),
           "Idle RTS did not activate prep_tx for the long-quiet frame");
  Require (g_noSignalDevice->GetMac ().GetLocalReservationCounter () == 13,
           "controlled long-quiet prep_tx did not select slot 13");

  // The public differential override controls the RNG result but deliberately
  // aligns multi-sender fixtures to a common epoch. Restore the production
  // single-node relative clock at the same prep_tx instant, retaining slot 13
  // and the independently scheduled 300-ms holdoff.
  g_noSignalDevice->GetMac ().SetReservationSlotOverrideForDifferentialRun (
    -1);
  g_noSignalDevice->GetMac ().StopSlotTick ();
  g_noSignalDevice->GetMac ().StartSlotTick (MilliSeconds (13));
}

void
CheckLongQuietHoldoff ()
{
  Require (g_noSignalDevice->GetMacState () == CsrMacCore::State::SEARCH,
           "long-quiet prep_tx left Search before transmission");
  Require (g_noSignalDevice->GetMac ().IsTxPreparationActive (),
           "long-quiet prep_tx was cleared during holdoff");
  Require (g_noSignalDevice->GetMac ().GetLocalReservationCounter () == 13,
           "slot counter advanced before the restarted holdoff expired");
  Require (g_noSignalDevice->GetMac ().GetTransmittedFrameCount () == 0,
           "long-quiet frame bypassed the restarted 300-ms holdoff");
}

void
RecordNoSignalTransitionTransmission (CsrNodeId destination,
                                      uint16_t sequence,
                                      Time sentTime)
{
  Require (destination == 2 && sequence == 1,
           "long-quiet controlled-slot transmission changed identity");
  Require (sentTime == Seconds (300.482),
           "slot 13 did not transmit at the source-exact 300.482-s time");
  Require (g_noSignalDevice->GetMac ().GetLastTxOpportunitySlot () == 13,
           "long-quiet transmission did not consume controlled slot 13");
  g_noSignalTxTime = sentTime;
  Simulator::Stop (NanoSeconds (1));
}

void
TestNoSignalSearchReturnsIdleAndRestartsHoldoff ()
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (9);
  g_noSignalTxTime = Time::Min ();

  g_noSignalDevice = CreateObject<CsrNetDevice> (1);
  g_noSignalDevice->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordNoSignalTransitionTransmission));
  g_noSignalDevice->EnableOpnetAlignedDutyCycling (true);

  Require (g_noSignalDevice->GetMacState () == CsrMacCore::State::IDLE,
           "OPNET-aligned no-signal test did not begin in Idle");

  // br_mac's first unconditional WAKE is at 0.988 s. Search with no signal
  // lasts DSP_RX_BOOTTIME + NO_SIG_SRH_TIME = 8.9 ms, then dsp_off() returns
  // to Idle. One nanosecond separates boundary checks from event-ID ordering.
  Simulator::Schedule (Seconds (0.988),
                       &CheckOpnetWakeEnteredSearch);
  Simulator::Schedule (Seconds (0.9969) + NanoSeconds (1),
                       &CheckNoSignalSearchReturnedIdle);

  // The old fmod-based boundary check first demonstrably loses its one-shot
  // SLEEP on the recovered cycle-six boundary at 5.9369 s.
  Simulator::Schedule (Seconds (5.928) + NanoSeconds (1),
                       &CheckOpnetWakeEnteredSearch);
  Simulator::Schedule (Seconds (5.9369) + NanoSeconds (1),
                       &CheckNoSignalSearchReturnedIdle);

  // The recovered latency run admits its first DATA at 300 s.  Once the
  // no-signal transition is preserved, that arrival finds Idle. The global
  // Idle RTS is at 300.001 s; prep_tx restarts the holdoff to 300.301 s. With
  // controlled slot 13, the first post-holdoff countdown reaches -1 at the
  // source-observed 300.482-s transmission instant.
  Simulator::Schedule (Seconds (300.0), &EnqueueAfterLongQuiet);
  Simulator::Schedule (Seconds (300.0005),
                       &CheckLongQuietQueueWaitsForRts);
  Simulator::Schedule (Seconds (300.301) - NanoSeconds (1),
                       &CheckLongQuietHoldoff);
  Simulator::Stop (Seconds (300.6));
  Simulator::Run ();

  Require (g_noSignalTxTime == Seconds (300.482),
           "long-quiet controlled-slot transmission was not observed");

  Simulator::Destroy ();
  g_noSignalDevice = nullptr;
}

void
ConnectControlledChainStack (Ptr<CsrNetDevice> device,
                             Ptr<CsrHopLayer> hop,
                             Ptr<CsrNetLayer> nwk,
                             CsrNodeId nodeId)
{
  hop->SetNodeId (nodeId);
  hop->SetMac (&device->GetMac ());
  nwk->SetNodeId (nodeId);
  nwk->SetArlNeighborAdmissionEnabled (false);
  nwk->SetHop (hop);
  nwk->ConfigureLinkControl (8, 128, -36.0, 33.0, 6.0);
  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));
}

void
RecordControlledDataTransmission (CsrNodeId destination,
                                  uint16_t sequence,
                                  Time sentTime)
{
  Require (destination == 1 && sequence == 1,
           "controlled chain changed the first DATA identity");
  Require (g_chainDataTxTime == Time::Min (),
           "controlled chain transmitted another DATA before ACK release");
  g_chainDataTxTime = sentTime;
  g_chainSourceHop->NotifyMacFrameSent (destination, sequence, sentTime);
}

void
ReceiveControlledAck (Ptr<Packet> frame,
                      double pathlossDb,
                      double snrDb)
{
  CsrHeader header;
  Require (frame->PeekHeader (header) && header.IsAck (),
           "controlled source received a non-ACK frame");
  Require (g_chainAckCompletionTime == Time::Min (),
           "controlled source received a second ACK before the test stopped");

  g_chainAckCompletionTime = Simulator::Now ();
  g_chainAckWireBytes = CsrGetOpnetWireSize (frame);
  g_chainAckHasPairwiseTransport =
    header.HasSecurityCount () && !header.HasGroupSecurity ();
  g_chainAckHasWindow = header.HasAckWindow ();
  g_chainSourceHop->ReceiveFromMac (frame, pathlossDb, snrDb);
}

void
ControlReceiverAckReservation ()
{
  Require (g_chainReceiverDevice->GetMacState () ==
             CsrMacCore::State::SEARCH,
           "DATA completion did not return the receiver to Search");
  Require (g_chainReceiverDevice->GetMac ().GetAckQueuedFrameCount () == 1,
           "DATA delivery did not enqueue one cumulative ACK");
  Require (g_chainReceiverDevice->GetMac ().IsTxPreparationActive (),
           "Track-to-Search did not activate ACK preparation");

  // Test-only control of the selected reservation leaves the receiver's
  // live 13-ms clock untouched.  The public differential override cannot be
  // used here because it intentionally realigns controlled senders to a
  // 100-ms epoch, which would erase the Search/Track-frozen source phase.
  CsrMacSlotParitySmokeAccess::LoadLiveReservation (
    g_chainReceiverDevice->GetMac (), 13);
}

void
RecordControlledDataDelivery (Ptr<Packet>, CsrNodeId source)
{
  Require (source == 2,
           "controlled receiver delivered DATA from the wrong NWK source");
  Require (g_chainDeliveries == 0,
           "controlled receiver delivered the first DATA more than once");
  g_chainDeliveries++;
  g_chainDataDeliveryTime = Simulator::Now ();

  // Run after the receive event unwinds so both ACK enqueue and the device's
  // Track-to-Search transition are complete.  The test intentionally does
  // not depend on whether HOP's local ACK construction precedes or follows
  // its NWK delivery callback.
  Simulator::ScheduleNow (&ControlReceiverAckReservation);
}

void
PollControlledAckTransmission ()
{
  if (g_chainReceiverDevice->GetMac ().GetTransmittedFrameCount () == 0)
    {
      Simulator::Schedule (NanoSeconds (1),
                           &PollControlledAckTransmission);
      return;
    }

  Require (g_chainAckTxTime == Time::Min (),
           "controlled chain observed the ACK transmission twice");
  g_chainAckTxTime = Simulator::Now ();
}

void
SeedControlledSourceNeighbor ()
{
  // The operational latency scenario has completed discovery before its
  // 300-s application start.  Seed only that already-established HOP link
  // observation; use a frame addressed elsewhere so this creates no control,
  // reliability, NWK, or MAC traffic in the regression.
  CsrHeader observation (1, 3, 0, 0, false, false);
  observation.SetType (CSR_PKT_DATA);
  observation.SetDestType (CSR_DEST_UNICAST);
  observation.SetLinkControl (128, 33.0, -109.0);
  Ptr<Packet> frame = Create<Packet> ();
  frame->AddHeader (observation);
  g_chainSourceHop->ReceiveFromMac (frame, 120.0, 30.0);
}

void
TestControlledFirstDataAckReleaseChain ()
{
  constexpr CsrNodeId sourceNode = 2;
  constexpr CsrNodeId receiverNode = 1;
  constexpr uint32_t configuredPacketBytes = 600;
  const Time dataTxAt = Seconds (300.482);
  const Time receiverWakeAt = Seconds (301.340);
  const Time receiverTrackAt = Seconds (301.346630);
  const Time dataDeliveryAt = Seconds (301.531404);
  const Time ackTxAt = Seconds (301.821);
  const Time sourceTrackAt = Seconds (301.827634);
  const Time ackCompletionAt = Seconds (301.843384);
  const Time nwkWakeAt = ackCompletionAt + CsrOpnetTic ();

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (10);
  RngSeedManager::ResetNextStreamIndex ();

  g_chainDataTxTime = Time::Min ();
  g_chainDataDeliveryTime = Time::Min ();
  g_chainAckTxTime = Time::Min ();
  g_chainAckCompletionTime = Time::Min ();
  g_chainAckWireBytes = 0;
  g_chainDeliveries = 0;
  g_chainAckHasPairwiseTransport = false;
  g_chainAckHasWindow = false;

  g_chainSourceDevice = CreateObject<CsrNetDevice> (sourceNode);
  g_chainReceiverDevice = CreateObject<CsrNetDevice> (receiverNode);
  g_chainSourceDevice->AddPeer (g_chainReceiverDevice);
  g_chainReceiverDevice->AddPeer (g_chainSourceDevice);

  CsrPhyProfile profile;
  profile.txPowerDbm = 33.0;
  profile.txBaseFrequencyHz = 400.0e6;
  profile.rxBaseFrequencyHz = 400.0e6;
  profile.txBwHz = 1.0e6;
  profile.rxBwHz = 1.0e6;
  profile.txHeightMeters = 1.05;
  profile.rxHeightMeters = 1.05;
  profile.eccThreshold = 0.1;
  profile.propagationModel = CsrPropagationModel::OPNET_THREE_PATH;
  g_chainSourceDevice->GetPhy ().SetProfile (profile);
  g_chainReceiverDevice->GetPhy ().SetProfile (profile);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  g_chainSourceDevice->GetPhy ().SetPerModel (noErrors);
  g_chainReceiverDevice->GetPhy ().SetPerModel (noErrors);
  g_chainSourceDevice->GetPhy ().SetLinkDistanceMeters (
    sourceNode, receiverNode, 1200.0);
  g_chainReceiverDevice->GetPhy ().SetLinkDistanceMeters (
    sourceNode, receiverNode, 1200.0);

  g_chainSourceHop = CreateObject<CsrHopLayer> ();
  g_chainReceiverHop = CreateObject<CsrHopLayer> ();
  g_chainSourceNwk = CreateObject<CsrNetLayer> ();
  g_chainReceiverNwk = CreateObject<CsrNetLayer> ();
  ConnectControlledChainStack (g_chainSourceDevice,
                               g_chainSourceHop,
                               g_chainSourceNwk,
                               sourceNode);
  ConnectControlledChainStack (g_chainReceiverDevice,
                               g_chainReceiverHop,
                               g_chainReceiverNwk,
                               receiverNode);
  g_chainSourceNwk->SetNodeType (CsrNodeType::Routable);
  g_chainReceiverNwk->SetNodeType (CsrNodeType::Gateway);
  g_chainSourceNwk->AddStaticRouteWithPathloss (
    receiverNode,
    receiverNode,
    120.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Gateway));
  g_chainReceiverNwk->SetRxFromNetCallback (
    MakeCallback (&RecordControlledDataDelivery));

  SeedControlledSourceNeighbor ();

  // Preserve the production HOP sent indication while recording exact MAC
  // transmission times.  Overwriting this callback without forwarding would
  // silently disable resend-expiration ownership in the path under test.
  g_chainSourceDevice->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordControlledDataTransmission));
  g_chainSourceDevice->GetMac ().SetRxCallback (
    MakeCallback (&ReceiveControlledAck));

  g_chainSourceDevice->EnableOpnetAlignedDutyCycling (true);
  g_chainReceiverDevice->EnableOpnetAlignedDutyCycling (true);

  // Source slot 13 is retained across the no-signal Idle cycles.  Idle RTS at
  // 300.001 s restarts the holdoff and relative clock; the first eligible
  // countdown therefore transmits at 300.482 s without epoch realignment.
  CsrMacSlotParitySmokeAccess::LoadLiveReservation (
    g_chainSourceDevice->GetMac (), 13);

  Simulator::Schedule (Seconds (300.0), [] () {
    const uint32_t networkBytes = configuredPacketBytes - 8;
    const uint32_t networkHeaderBytes =
      CsrNetHeader ().GetSerializedSize ();
    Require (networkBytes > networkHeaderBytes,
             "controlled packet is too small for its NWK header");
    const uint32_t payloadBytes = networkBytes - networkHeaderBytes;

    // The second packet is an observation load only: the source-exact initial
    // HOP window holds it in NWK until the first ACK releases capacity.  It
    // makes the remote NWK wake at completion + TIC directly observable.
    g_chainSourceNwk->Send (receiverNode,
                            0,
                            Create<Packet> (payloadBytes),
                            true);
    g_chainSourceNwk->Send (receiverNode,
                            0,
                            Create<Packet> (payloadBytes),
                            true);
  });

  Simulator::Schedule (dataTxAt - NanoSeconds (1), [] () {
    Require (g_chainSourceDevice->GetMac ().GetTransmittedFrameCount () == 0,
             "controlled DATA transmitted before slot 13");
    Require (CsrMacSlotParitySmokeAccess::GetFirstDataWireBytes (
               g_chainSourceDevice->GetMac ()) == 622,
             "controlled DATA did not retain its 622-byte Pairwise16 wire size");
  });
  Simulator::Schedule (dataTxAt + NanoSeconds (1), [] () {
    Require (g_chainSourceDevice->GetMac ().GetTransmittedFrameCount () == 1 &&
               g_chainSourceDevice->GetMac ().GetLastTxOpportunitySlot () == 13,
             "controlled DATA did not consume slot 13");
  });
  Simulator::Schedule (receiverWakeAt - NanoSeconds (1), [] () {
    Require (g_chainReceiverDevice->GetMacState () == CsrMacCore::State::IDLE,
             "receiver left Idle before its OPNET periodic wake");
  });
  Simulator::Schedule (receiverWakeAt + NanoSeconds (1), [] () {
    Require (g_chainReceiverDevice->GetMacState () == CsrMacCore::State::SEARCH,
             "receiver did not enter Search at its OPNET periodic wake");
  });
  Simulator::Schedule (receiverTrackAt - NanoSeconds (1), [] () {
    Require (g_chainReceiverDevice->GetMacState () == CsrMacCore::State::SEARCH,
             "receiver entered Track before SYNC2TRACK_MIN");
  });
  Simulator::Schedule (receiverTrackAt + NanoSeconds (1), [] () {
    Require (g_chainReceiverDevice->GetMacState () == CsrMacCore::State::TRACK,
             "receiver did not enter Track after 6.63 ms");
  });
  Simulator::Schedule (ackTxAt - NanoSeconds (1), [] () {
    Require (g_chainReceiverDevice->GetMac ().GetTransmittedFrameCount () == 0,
             "controlled ACK transmitted before slot 13");
  });
  Simulator::Schedule (ackTxAt - NanoSeconds (1),
                       &PollControlledAckTransmission);
  Simulator::Schedule (ackTxAt + NanoSeconds (1), [] () {
    Require (g_chainReceiverDevice->GetMac ().GetTransmittedFrameCount () == 1 &&
               g_chainReceiverDevice->GetMac ().GetLastTxOpportunitySlot () == 13,
             "controlled ACK did not consume receiver slot 13");
  });
  Simulator::Schedule (sourceTrackAt - NanoSeconds (1), [] () {
    Require (g_chainSourceDevice->GetMacState () == CsrMacCore::State::SEARCH,
             "source entered ACK Track before short-preamble acquisition");
  });
  Simulator::Schedule (sourceTrackAt + NanoSeconds (1), [] () {
    Require (g_chainSourceDevice->GetMacState () == CsrMacCore::State::TRACK,
             "source did not acquire the short-preamble ACK");
  });
  Simulator::Schedule (ackCompletionAt - NanoSeconds (1), [] () {
    Require (g_chainSourceNwk->GetNsdpCount (sourceNode, receiverNode) == 2 &&
               g_chainSourceHop->GetPendingDataCount () == 1 &&
               g_chainSourceHop->GetOutstandingDataCount (receiverNode) == 1,
             "HOP/NWK capacity released before ACK completion");
  });
  Simulator::Schedule (ackCompletionAt + NanoSeconds (1), [] () {
    Require (g_chainSourceNwk->GetNsdpCount (sourceNode, receiverNode) == 1,
             "ACK did not release exactly one NWK NSDP entry");
    Require (g_chainSourceNwk->GetNwkQueueSize () == 1,
             "NWK queue ran synchronously with ACK capacity release");
    Require (g_chainSourceHop->GetPendingDataCount () == 0 &&
               g_chainSourceHop->GetOutstandingDataCount (receiverNode) == 0 &&
               g_chainSourceHop->GetResendQueueSize () == 0,
             "ACK did not release HOP custody and capacity together");
  });
  Simulator::Schedule (nwkWakeAt - NanoSeconds (1), [] () {
    Require (g_chainSourceNwk->GetNwkQueueSize () == 1 &&
               g_chainSourceHop->GetPendingDataCount () == 0,
             "held DATA advanced before the HOP-owned NWK wake");
  });
  Simulator::Schedule (nwkWakeAt + NanoSeconds (1), [] () {
    Require (g_chainSourceNwk->GetNwkQueueSize () == 0,
             "HOP-owned NWK wake did not run one TIC after ACK release");
    Require (g_chainSourceHop->GetPendingDataCount () == 1 &&
               g_chainSourceHop->GetOutstandingDataCount (receiverNode) == 1,
             "NWK wake did not admit the held second DATA into HOP");
  });

  Simulator::Stop (nwkWakeAt + NanoSeconds (2));
  Simulator::Run ();

  RequireTime (g_chainDataTxTime,
               dataTxAt,
               "controlled DATA TX timestamp changed from 300.482 s");
  Require (g_chainDeliveries == 1,
           "controlled Pairwise16 DATA was not delivered exactly once");
  RequireTime (g_chainDataDeliveryTime,
               dataDeliveryAt,
               "controlled Pairwise16 DATA delivery timestamp changed");
  RequireTime (g_chainAckTxTime,
               ackTxAt,
               "controlled slot-13 ACK TX timestamp changed");
  RequireTime (g_chainAckCompletionTime,
               ackCompletionAt,
               "Pairwise16 ACK completion timestamp changed");
  Require (g_chainAckWireBytes == 46 &&
             g_chainAckHasPairwiseTransport &&
             g_chainAckHasWindow,
           "ACK did not retain its 46-byte Pairwise16 cumulative wrapper");

  Simulator::Destroy ();
  g_chainSourceDevice = nullptr;
  g_chainReceiverDevice = nullptr;
  g_chainSourceHop = nullptr;
  g_chainReceiverHop = nullptr;
  g_chainSourceNwk = nullptr;
  g_chainReceiverNwk = nullptr;
}

void
TestNoSignalSleepCancelsStaleAcquisition ()
{
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);
  receiver->EnableOpnetAlignedDutyCycling (true);

  // Start a long preamble six milliseconds into the first Search window. Its
  // 6.63-ms RX_SIG_FOUND acquisition is due at 1.00063 s, after the already-
  // scheduled no-signal SLEEP at 0.9969 s. Source dsp_off() cancels that old
  // acquisition. Re-entering Search at 1.000 s must not resurrect it.
  Simulator::Schedule (Seconds (0.994), [sender] () {
    sender->SendToPeer (BuildDataFrame (1),
                        2,
                        128,
                        0.0,
                        PREAMBLE_LONG,
                        3,
                        false);
  });
  Simulator::Schedule (Seconds (0.9969) + NanoSeconds (1), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::IDLE,
             "pending acquisition prevented the no-signal SLEEP transition");
  });
  Simulator::Schedule (Seconds (1.000), [receiver] () {
    receiver->ForceAwakeFor (0.1);
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "stale-acquisition fixture did not re-enter Search");
  });
  Simulator::Schedule (Seconds (1.001), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "RX_SIG_FOUND survived dsp_off and entered Track after re-wake");
  });

  Simulator::Stop (Seconds (1.002));
  Simulator::Run ();
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  TestMacReservationLifecycle ();
  TestIdleRtsStartsHoldoff ();
  TestHopRetryUsesLiveReservation ();
  TestSearchSyncActivation ();
  TestSearchZeroExpiresBeforeActivation ();
  TestTrackReturnActivatesImmediately ();
  TestIdleZeroRedrawsAtRts ();
  TestPackingRetryRespectsMacState ();
  TestNoSignalSearchReturnsIdleAndRestartsHoldoff ();
  TestControlledFirstDataAckReleaseChain ();
  TestNoSignalSleepCancelsStaleAcquisition ();

  std::cout << "PASS: OPNET MAC reservation lifecycle test" << std::endl;
  return 0;
}
