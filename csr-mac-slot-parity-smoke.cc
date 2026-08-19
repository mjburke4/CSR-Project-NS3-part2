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
RecordTransmission (uint16_t destination,
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
CheckHoldoff (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 0,
           "MAC transmitted before OPNET's 300-ms holdoff ended");
}

void
CheckReservationMidCountdown (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetNeighborReservationCounter (9) == 0,
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

  // active_nodes=1 selects a free-slot ordinal in [1,18].  The first frame
  // must therefore start after 0.3 + 2*0.013 and no later than the largest
  // normal offset plus the one occupied neighbor reservation in this test.
  Require (g_txTimes[0] >= 0.326 && g_txTimes[0] <= 0.560,
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
  Require (device->GetMac ().GetNeighborReservationCounter (10) == -1,
           "neighbor reservation did not resume after MAC left Tx_st");
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

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
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
  Simulator::Schedule (Seconds (1.53),
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
