#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

uint32_t g_receivedFrames = 0;
int g_receivedRateKbps = 0;
double g_receivedTxPowerDbm = 0.0;
double g_receivedRxPowerDbm = 0.0;

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
RecordAtReceiver (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "receiver got a frame without a CSR header");
  Require (header.HasLinkControl (),
           "MAC transmission discarded HOP link-control metadata");

  g_receivedFrames++;
  g_receivedRateKbps = header.GetSpeedKey ();
  g_receivedTxPowerDbm = header.GetTxPowerDbm ();
  g_receivedRxPowerDbm = header.GetRxPowerDbm ();
}

void
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

Ptr<Packet>
BuildNeighborObservation (uint16_t source,
                          double advertisedS0PowerDbm)
{
  CsrHeader header (source, 1, 1, 5, false, false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetLinkControl (8, 0.0, advertisedS0PowerDbm);

  Ptr<Packet> frame = Create<Packet> (8);
  frame->AddHeader (header);
  return frame;
}

void
RunCase (bool establishNeighbor,
         double pathlossDb,
         int expectedRateKbps,
         double expectedTxPowerDbm)
{
  g_receivedFrames = 0;
  g_receivedRateKbps = 0;
  g_receivedTxPowerDbm = 0.0;
  g_receivedRxPowerDbm = 0.0;

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();

  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);

  hop->SetNodeId (1);
  hop->SetMac (&sender->GetMac ());
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordAtReceiver));

  if (establishNeighbor)
    {
      hop->ReceiveFromMac (BuildNeighborObservation (2, -105.0),
                           pathlossDb,
                           10.0);
    }

  hop->SendData (2, 5, Create<Packet> (24), false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (g_receivedFrames == 1,
           "receiver did not get exactly one DATA frame");
  Require (g_receivedRateKbps == expectedRateKbps,
           "MAC did not use HOP's selected link-control speed");
  Require (std::fabs (g_receivedTxPowerDbm - expectedTxPowerDbm) < 0.01,
           "MAC did not use HOP's selected transmit power");
  Require (std::fabs (g_receivedRxPowerDbm - (-105.0)) < 0.01,
           "MAC did not retain HOP's advertised receive threshold");
  Require (sender->GetMac ().GetLastTxRateKbps () == expectedRateKbps,
           "MAC rate diagnostic disagrees with the transmitted header");
  Require (std::fabs (sender->GetMac ().GetLastTxPowerDbm () -
                       expectedTxPowerDbm) < 0.01,
           "MAC power diagnostic disagrees with the transmitted header");

  if (establishNeighbor)
    {
      Require (hop->GetNeighborLinkCost (2) > 0,
               "HOP did not retain the live link-control cost");
    }

  Simulator::Destroy ();
}

void
RunAggregateCase ()
{
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();

  ConfigureNoErrors (sender);
  hop->SetNodeId (1);
  hop->SetMac (&sender->GetMac ());

  // Destination 2 permits 128 kbps at 14 dBm.  Destination 3 requires
  // 32 kbps at 29 dBm.  OPNET transmits the whole concatenated frame using
  // the slowest destination's result, irrespective of queue order.
  hop->ReceiveFromMac (BuildNeighborObservation (2, -105.0), 107.0, 10.0);
  hop->ReceiveFromMac (BuildNeighborObservation (3, -105.0), 128.0, 10.0);

  hop->SendData (2, 5, Create<Packet> (24), false);
  hop->SendData (3, 5, Create<Packet> (24), false);

  Simulator::Stop (Seconds (1.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "MAC did not concatenate the two queued destinations");
  Require (sender->GetMac ().GetLastTxRateKbps () == 32,
           "aggregate did not use the slowest destination's HOP speed");
  Require (std::fabs (sender->GetMac ().GetLastTxPowerDbm () - 29.0) < 0.01,
           "aggregate did not use the selected slow-destination power");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  // S0=-105 dBm and 107 dB pathloss select 128 kbps at 14 dBm.
  RunCase (true, 107.0, 128, 14.0);

  // A later/weaker observation must drive a new live result rather than
  // leaving MAC's independent PER-based choice in control.
  RunCase (true, 128.0, 32, 29.0);

  // OPNET link_control() falls back to minimum speed and maximum power when
  // the destination is not yet in HOP's neighbor table.
  RunCase (false, 0.0, 8, 30.0);

  RunAggregateCase ();

  std::cout << "PASS: OPNET HOP/MAC live link-control parity test"
            << std::endl;
  return 0;
}
