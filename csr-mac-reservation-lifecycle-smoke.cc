#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

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
int32_t g_trackAdvertisedSlot = -1;
int64_t g_trackExpectedTxNs = -1;

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

  std::cout << "PASS: OPNET MAC reservation lifecycle test" << std::endl;
  return 0;
}
