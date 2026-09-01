#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

using namespace ns3;

struct CsrMacSlotParitySmokeAccess
{
  static int
  PickTxSlot (CsrMacCore &mac, CsrNodeId destination)
  {
    return mac.PickTxSlot (destination);
  }
};

namespace
{

std::vector<uint16_t> g_sequences;
std::vector<double> g_txTimes;

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
RecordTransmission (CsrNodeId destination,
                    uint16_t sequence,
                    Time sentTime)
{
  Require (destination == 2, "MAC transmitted to the wrong destination");
  g_sequences.push_back (sequence);
  g_txTimes.push_back (sentTime.GetSeconds ());
}

Ptr<Packet>
BuildDataFrame (uint16_t sequence, uint8_t dscp)
{
  CsrHeader header (1, 2, sequence, dscp, true, false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);

  Ptr<Packet> frame = Create<Packet> (20);
  frame->AddHeader (header);
  return frame;
}

void
CheckSlotSelectionProfileRanges (Ptr<CsrNetDevice> device)
{
  CsrMacCore &mac = device->GetMac ();
  using Profile = CsrMacCore::SlotSelectionProfile;

  mac.SetSlotSelectionProfile (Profile::NS3_CURRENT_FINE_FREE_SLOT);
  Require (mac.GetOpnetSlotRange (0) == 15 &&
             mac.GetOpnetSlotRange (5) == 37 &&
             mac.GetOpnetSlotRange (16) == 255,
           "current fine slot-range map changed");

  mac.SetSlotSelectionProfile (Profile::HIST_2014_ZERO_BASED_REBUILD_LIST);
  Require (mac.GetOpnetSlotRange (0) == 31 &&
             mac.GetOpnetSlotRange (4) == 31 &&
             mac.GetOpnetSlotRange (5) == 63 &&
             mac.GetOpnetSlotRange (9) == 127 &&
             mac.GetOpnetSlotRange (13) == 255,
           "historical rebuild-list coarse slot-range map is wrong");

  mac.SetSlotSelectionProfile (
    Profile::HIST_2014_COARSE_INCLUSIVE_NO_AVOID);
  struct RangeVector
  {
    uint32_t activeNodes;
    int expectedRange;
  };
  const RangeVector rangeVectors[] = {
    {0, 31},
    {4, 31},
    {5, 63},
    {8, 63},
    {9, 127},
    {12, 127},
    {13, 255},
    {std::numeric_limits<uint32_t>::max (), 255},
  };
  for (const RangeVector &vector : rangeVectors)
    {
      Require (mac.GetOpnetSlotRange (vector.activeNodes) ==
                 vector.expectedRange,
               "recovered coarse-inclusive slot-range map is wrong");
    }

  mac.SetSlotSelectionProfile (
    Profile::HIST_2015_FINE_ONE_BASED_TABLE_NO_AVOID);
  Require (mac.GetOpnetSlotRange (0) == 15 &&
             mac.GetOpnetSlotRange (5) == 37 &&
             mac.GetOpnetSlotRange (16) == 255,
           "historical one-based-table fine slot-range map is wrong");

  mac.SetSlotSelectionProfile (
    Profile::HIST_2014_NEXT_TSLOT_MODULO_PROBE);
  Require (mac.GetOpnetSlotRange (4) == 31 &&
             mac.GetOpnetSlotRange (8) == 63 &&
             mac.GetOpnetSlotRange (12) == 127 &&
             mac.GetOpnetSlotRange (17) == 255,
           "historical modulo-probe coarse slot-range map is wrong");

  mac.SetSlotSelectionProfile (Profile::NS3_CURRENT_FINE_FREE_SLOT);
}

void
CheckCoarseInclusiveNoAvoidDraw (uint64_t run,
                                 uint32_t localActiveNodes,
                                 uint32_t reportedActiveNodes,
                                 int occupiedSlot,
                                 int expectedRange,
                                 int expectedSlot)
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (run);
  RngSeedManager::ResetNextStreamIndex ();

  CsrMacCore mac;
  mac.SetNodeId (1);
  mac.SetSlotSelectionProfile (
    CsrMacCore::SlotSelectionProfile::
      HIST_2014_COARSE_INCLUSIVE_NO_AVOID);
  mac.SetActiveNodesForPostTx (localActiveNodes);
  mac.NoteReportedActiveNodes (reportedActiveNodes);
  mac.NoteNeighborReservedSlot (9, occupiedSlot);

  Require (mac.GetNeighborReservationCounter (9) == occupiedSlot,
           "coarse-inclusive no-avoid fixture did not occupy the draw");
  Require (mac.GetOpnetSlotRange (localActiveNodes) == expectedRange,
           "coarse-inclusive no-avoid fixture selected the wrong range");
  Require (CsrMacSlotParitySmokeAccess::PickTxSlot (mac, 2) == expectedSlot,
           "recovered coarse-inclusive draw or no-avoid behavior changed");

  Simulator::Destroy ();
}

void
CheckCoarseInclusiveNoAvoidVectors ()
{
  // Recovered get_slot@0x1e39f uses only local active_nodes, draws 0..R
  // inclusive, and returns occupied slots without reservation avoidance.
  // These ns-3 run numbers pin the selection branch and its endpoints; they
  // do not claim that OPNET and ns-3 generate the same random sequence.
  CheckCoarseInclusiveNoAvoidDraw (5, 4, 13, 0, 31, 0);
  CheckCoarseInclusiveNoAvoidDraw (47, 4, 13, 31, 31, 31);
  CheckCoarseInclusiveNoAvoidDraw (899, 13, 1, 255, 255, 255);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);
  RngSeedManager::ResetNextStreamIndex ();
}

void
CheckHoldoff (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 0,
           "MAC transmitted before OPNET's 300-ms holdoff ended");
}

void
CheckReservationMidCountdown (Ptr<CsrNetDevice> device)
{
  int32_t counter =
    device->GetMac ().GetNeighborReservationCounter (9);
  if (counter != 0)
    {
      std::cerr << "neighbor counter at 45 ms=" << counter << std::endl;
    }
  Require (counter == 0,
           "neighbor reservation did not count down once per 13-ms slot");
}

void
CheckReservationExpired (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetNeighborReservationCounter (9) == -1,
           "neighbor reservation did not expire at counter -1");
}

void
CheckFirstOpportunity (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "first aggregate missed its OPNET slot opportunity");
  Require (g_sequences.size () == 2 &&
             g_sequences[0] == 2 && g_sequences[1] == 1,
           "aggregate did not preserve DSCP queue order");
  Require (g_txTimes[0] == g_txTimes[1],
           "queued DATA frames did not share one OTA transmission");

  // The 300-ms holdoff and already-running 13-ms slot clock are independent.
  // active_nodes=1 selects an ordinal in [1,18], so the first opportunity is
  // an absolute slot boundary in the source-backed 0.325--0.559-s window.
  int64_t txNanoseconds =
    static_cast<int64_t> (std::llround (g_txTimes[0] * 1e9));
  Require (txNanoseconds % 13000000 == 0,
           "first TX did not retain the shared absolute 13-ms phase");
  Require (g_txTimes[0] >= 0.325 && g_txTimes[0] <= 0.559,
           "first TX time is outside the OPNET holdoff/slot window");
}

void
NoteReservationDuringTx (Ptr<CsrNetDevice> device)
{
  device->GetMac ().NoteNeighborReservedSlot (10, 5);
}

void
CheckReservationPausedInTx (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetNeighborReservationCounter (10) == 5,
           "neighbor reservation advanced while OPNET MAC was in Tx_st");
}

void
CheckReservationResumedAfterTx (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetNeighborReservationCounter (10) == -2,
           "neighbor counter did not keep advancing after expiration");
}

void
CheckFinalState (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "MAC did not concatenate both queued frames");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "MAC queue did not drain after the reserved opportunity");
  Require (g_sequences.size () == 2 &&
             g_sequences[0] == 2 &&
             g_sequences[1] == 1,
           "MAC did not preserve DSCP priority followed by FIFO service");

  Require (g_txTimes[0] == g_txTimes[1],
           "concatenated frames did not report one shared sent instant");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  CheckCoarseInclusiveNoAvoidVectors ();
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  CheckSlotSelectionProfileRanges (device);
  device->GetMac ().SetActiveNodesForPostTx (1);
  device->GetMac ().SetTxSentCallback (MakeCallback (&RecordTransmission));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);

  // OPNET stores the advertised slot itself as rtslot_counter.  Starting at
  // three produces 3,2,1,0,-1 on successive 13-ms ticks.
  device->GetMac ().NoteNeighborReservedSlot (9, 3);
  device->GetMac ().StartSlotTick (MilliSeconds (13));

  device->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1, 1), 2, 1, true);
  device->GetMac ().EnqueueTxFrame (
    BuildDataFrame (2, 7), 2, 7, true);

  Simulator::Schedule (MilliSeconds (45),
                       &CheckReservationMidCountdown,
                       device);
  Simulator::Schedule (MilliSeconds (55),
                       &CheckReservationExpired,
                       device);
  Simulator::Schedule (MilliSeconds (299),
                       &CheckHoldoff,
                       device);
  Simulator::Schedule (MilliSeconds (565),
                       &CheckFirstOpportunity,
                       device);
  Simulator::Schedule (MilliSeconds (600),
                       &NoteReservationDuringTx,
                       device);
  Simulator::Schedule (Seconds (1.0),
                       &CheckReservationPausedInTx,
                       device);
  // Exact br_OTA airtime keeps this long-preamble aggregate in Tx until
  // roughly 1.55 seconds.  The source keeps decrementing after expiration,
  // so this checkpoint observes -2 rather than clamping at -1.
  Simulator::Schedule (Seconds (1.65),
                       &CheckReservationResumedAfterTx,
                       device);
  Simulator::Schedule (Seconds (1.8),
                       &CheckFinalState,
                       device);

  Simulator::Stop (Seconds (1.9));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << "PASS: OPNET MAC slot scheduling parity test" << std::endl;
  return 0;
}
