#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"
#include "ns3/csr-opnet-ber-tables.h"
#include "ns3/csr-opnet-envelope.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

uint32_t g_receiveCount = 0;
CsrRateKey g_receivedRateKbps = 0;
double g_receivedTxPowerDbm = 0.0;

void
Require (bool condition, const char *message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

void
RequireNear (double actual,
             double expected,
             double tolerance,
             const char *message)
{
  if (!std::isfinite (actual) ||
      std::fabs (actual - expected) > tolerance)
    {
      std::cerr << "FAIL: " << message
                << " expected=" << expected
                << " actual=" << actual
                << std::endl;
      std::exit (1);
    }
}

void
ResetReception ()
{
  g_receiveCount = 0;
  g_receivedRateKbps = 0;
  g_receivedTxPowerDbm = 0.0;
}

void
RecordReception (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received frame has no CSR header");
  g_receiveCount++;
  g_receivedRateKbps = header.GetSpeedKey ();
  g_receivedTxPowerDbm = header.GetTxPowerDbm ();
}

void
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  device->GetPhy ().SetPerModel (
    [] (int, double, uint32_t) { return 0.0; });
}

Ptr<Packet>
BuildFrame (CsrNodeId source,
            CsrNodeId destination,
            uint16_t sequence,
            CsrRateKey rateKbps,
            uint32_t payloadBytes,
            bool linkControl = false,
            double txPowerDbm = 0.0)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    5,
                    false,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  if (linkControl)
    {
      header.SetLinkControl (rateKbps, txPowerDbm, -105.0);
    }
  else
    {
      header.SetSpeedKey (rateKbps);
    }

  Ptr<Packet> frame = Create<Packet> (payloadBytes);
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildNeighborObservation (CsrNodeId source, double advertisedS0PowerDbm)
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
TestRateConfigurationAndPropagation ()
{
  const CsrRateKey supported[] = {8, 16, 32, 64, 128, 500, 1000};
  for (CsrRateKey rate : supported)
    {
      Require (CsrIsOperationalRateKey (rate),
               "defined live rate was rejected");
    }
  Require (!CsrIsOperationalRateKey (129) &&
           !CsrIsOperationalRateKey (130),
           "reserved compact codes were accepted as raw live rates");
  Require (CsrCanEncodeRateKey (500) &&
           CsrCanEncodeRateKey (1000) &&
           CsrCanEncodeRateKey (0x7e) &&
           CsrCanEncodeRateKey (0xbc) &&
           !CsrCanEncodeRateKey (129) &&
           !CsrCanEncodeRateKey (130) &&
           !CsrCanEncodeRateKey (256),
           "compact rate-code alias guard changed");

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();

  Require (device->GetRateProfile () ==
             CsrRateProfile::LEGACY_SOURCE_EXACT &&
           hop->GetRateProfile () ==
             CsrRateProfile::LEGACY_SOURCE_EXACT &&
           nwk->GetRateProfile () ==
             CsrRateProfile::LEGACY_SOURCE_EXACT,
           "legacy source-exact profile is not the default");
  Require (device->GetMac ().IsRateEnabled (128) &&
           !device->GetMac ().IsRateEnabled (500) &&
           !device->GetMac ().IsRateEnabled (1000),
           "legacy profile did not reject explicit high-rate transmission");

  nwk->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  hop->SetMac (&device->GetMac ());
  nwk->SetHop (hop);
  Require (nwk->GetMaxSpeedKbps () == 1000 &&
           hop->GetMaxSpeedKbps () == 1000 &&
           device->GetMac ().GetMaxRateKbps () == 1000 &&
           device->GetMac ().IsRateEnabled (500) &&
           device->GetMac ().IsRateEnabled (1000),
           "pre-attachment extended profile did not propagate NWK to MAC");

  nwk->SetMaxSpeedKbps (500);
  Require (nwk->GetMaxSpeedKbps () == 500 &&
           hop->GetMaxSpeedKbps () == 500 &&
           device->GetMac ().GetMaxRateKbps () == 500 &&
           device->GetMac ().IsRateEnabled (500) &&
           !device->GetMac ().IsRateEnabled (1000),
           "500-kbps scenario cap did not propagate NWK to MAC");

  nwk->SetRateProfile (CsrRateProfile::LEGACY_SOURCE_EXACT);
  Require (nwk->GetMaxSpeedKbps () == 128 &&
           hop->GetMaxSpeedKbps () == 128 &&
           device->GetMac ().GetMaxRateKbps () == 128,
           "legacy profile did not restore all three live caps");

  nwk->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  Require (nwk->GetMaxSpeedKbps () == 1000 &&
           hop->GetMaxSpeedKbps () == 1000 &&
           device->GetMac ().GetMaxRateKbps () == 1000,
           "post-attachment extended profile did not propagate NWK to MAC");
  Simulator::Destroy ();
}

void
RunLinkControlCase (CsrRateKey maxRateKbps,
                    CsrRateKey expectedRateKbps,
                    double expectedTxPowerDbm)
{
  ResetReception ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  hop->SetNodeId (1);
  hop->SetMaxSpeedKbps (maxRateKbps);
  hop->SetMac (&sender->GetMac ());
  hop->ReceiveFromMac (BuildNeighborObservation (2, -105.0),
                       107.0,
                       20.0);
  sender->GetMac ().NoteHeardFrom (2, Simulator::Now ().GetSeconds ());
  hop->SendData (2, 5, Create<Packet> (24), false);

  Simulator::Stop (Seconds (1.5));
  Simulator::Run ();
  Require (g_receiveCount == 1,
           "live high-rate link-control frame was not delivered");
  Require (g_receivedRateKbps == expectedRateKbps &&
           sender->GetMac ().GetLastTxRateKbps () == expectedRateKbps,
           "HOP and MAC disagree on the selected high rate");
  RequireNear (g_receivedTxPowerDbm,
               expectedTxPowerDbm,
               0.01,
               "high-rate link-control transmit power changed");
  RequireNear (sender->GetMac ().GetLastTxPowerDbm (),
               expectedTxPowerDbm,
               0.01,
               "MAC high-rate power diagnostic changed");
  Simulator::Destroy ();
}

void
TestAutomaticMacRateSelection ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  device->GetPhy ().SetLinkDistanceMeters (1, 2, 0.0);
  Require (device->GetMac ().SelectRateByPerTarget (2, 256, 0.50) == 128,
           "legacy MAC profile exceeded its 128-kbps ceiling");

  device->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  Require (device->GetMac ().SelectRateByPerTarget (2, 256, 0.50) == 1000,
           "extended MAC profile did not select 1000 kbps at high SNR");

  device->GetMac ().SetMaxRateKbps (500);
  Require (device->GetMac ().SelectRateByPerTarget (2, 256, 0.50) == 500,
           "MAC ignored the explicit 500-kbps scenario ceiling");
  Simulator::Destroy ();
}

void
TestAutomaticQueuedMacRateSelection ()
{
  ResetReception ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  sender->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));
  sender->GetMac ().NoteHeardFrom (2, Simulator::Now ().GetSeconds ());

  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (1, 2, 25, 8, 64),
    2,
    5,
    true);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();
  Require (sender->GetMac ().GetLastTxRateKbps () == 1000 &&
           sender->GetMac ().GetQueuedFrameCount () == 0 &&
           g_receiveCount == 1,
           "queued no-link-control frame retained the legacy 128-kbps cap");
  Simulator::Destroy ();
}

void
RunLiveDqpskSuccess (CsrRateKey rateKbps, double rawSnrDb)
{
  ResetReception ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  sender->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 2, 0.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  Ptr<Packet> frame = BuildFrame (1, 2, 10, rateKbps, 256);
  uint32_t payloadBytes = CsrGetOpnetWireSize (frame);
  double txPowerDbm = receiver->GetPhy ().profile.noiseFloorDbm + rawSnrDb;
  Time duration = sender->SendToPeer (frame,
                                      2,
                                      rateKbps,
                                      txPowerDbm,
                                      PREAMBLE_SHORT,
                                      3,
                                      false);
  double expectedDurationSec =
    static_cast<double> (104 + 48) / 4.0 * 0.00051 +
    static_cast<double> (payloadBytes * 8 + 32) /
      CsrRateKeyToBps (rateKbps);
  RequireNear (duration.GetSeconds (),
               expectedDurationSec,
               1.0e-12,
               "live high-rate OTA duration changed");

  Simulator::Stop (duration + MilliSeconds (10));
  Simulator::Run ();
  Require (g_receiveCount == 1 && receiver->HasLastRxDecision (),
           "live DQPSK frame did not complete the receive pipeline");
  const CsrRxDecision &decision = receiver->GetLastRxDecision ();
  double processingGainDb = 10.0 * std::log10 (
    receiver->GetPhy ().profile.rxBwHz /
    (2.0 * CsrRateKeyToBps (rateKbps)));
  double expectedDqpskBer = CsrOpnetBerTables::GetDqpskBer (
    decision.snrDb + processingGainDb);
  double standardBer = CsrOpnetBerTables::GetStandardBer (
    decision.snrDb + processingGainDb);
  Require (decision.success && !decision.eccDropped,
           "moderate-SNR DQPSK frame failed ECC unexpectedly");
  RequireNear (decision.payloadBer,
               expectedDqpskBer,
               1.0e-12,
               "live payload did not use the recovered DQPSK table");
  Require (expectedDqpskBer > standardBer,
           "live DQPSK curve did not remain distinct from CSR");
  Simulator::Destroy ();
}

void
TestLiveDqpskEccDrop ()
{
  ResetReception ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  sender->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 2, 0.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  Ptr<Packet> frame = BuildFrame (1, 2, 20, 500, 512);
  double txPowerDbm = receiver->GetPhy ().profile.noiseFloorDbm - 10.0;
  Time duration = sender->SendToPeer (frame,
                                      2,
                                      500,
                                      txPowerDbm,
                                      PREAMBLE_SHORT,
                                      3,
                                      false);
  Simulator::Stop (duration + MilliSeconds (10));
  Simulator::Run ();

  Require (g_receiveCount == 0 && receiver->HasLastRxDecision (),
           "high-BER DQPSK frame bypassed final accept/drop ordering");
  const CsrRxDecision &decision = receiver->GetLastRxDecision ();
  Require (!decision.success && decision.eccDropped,
           "high-BER DQPSK frame was not rejected specifically by ECC");
  Require (decision.payloadBer > 0.5,
           "live low-SNR path did not exercise the DQPSK p > 0.5 tail");
  Require (decision.totalErrors > decision.correctableBits &&
           receiver->GetRxEccDropCount () == 1,
           "high-rate ECC drop count or inclusive threshold changed");
  Simulator::Destroy ();
}

void
TestLiveSameRateDqpskCollision ()
{
  ResetReception ();
  Ptr<CsrNetDevice> desired = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> jammer = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  desired->AddPeer (receiver);
  jammer->AddPeer (receiver);
  desired->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  jammer->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 3, 0.0);
  receiver->GetPhy ().SetLinkDistanceMeters (2, 3, 0.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  Ptr<Packet> desiredFrame = BuildFrame (1, 3, 30, 500, 512);
  Time desiredDuration = desired->SendToPeer (desiredFrame,
                                              3,
                                              500,
                                              0.0,
                                              PREAMBLE_SHORT,
                                              3,
                                              false);
  Simulator::Schedule (
    Seconds (0.00765),
    [jammer] () {
      jammer->SendToPeer (BuildFrame (2, 3, 31, 500, 512),
                          3,
                          500,
                          0.0,
                          PREAMBLE_SHORT,
                          4,
                          false);
    });
  Simulator::Stop (desiredDuration + MicroSeconds (1));
  Simulator::Run ();

  Require (g_receiveCount == 0 && receiver->HasLastRxDecision (),
           "equal-power DQPSK collision reached MAC delivery");
  const CsrRxDecision &decision = receiver->GetLastRxDecision ();
  Require (decision.sameRateInterference,
           "live high-rate overlap did not record same-rate interference");
  RequireNear (decision.jsrDb,
               0.0,
               1.0e-9,
               "equal-power DQPSK overlap did not record 0-dB JSR");
  RequireNear (decision.payloadBer,
               CsrOpnetBerTables::GetDqpskBer (0.0),
               1.0e-9,
               "same-rate high-rate overlap did not use jammer-as-noise DQPSK");
  Require (!decision.success && decision.eccDropped,
           "same-rate DQPSK overlap was hard-dropped before BER/ECC");
  Simulator::Destroy ();
}

void
TestHighRateQueueHeadProgress ()
{
  ResetReception ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  sender->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  receiver->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));
  sender->GetMac ().NoteHeardFrom (2, Simulator::Now ().GetSeconds ());

  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (1, 2, 40, 1000, 64, true, 0.0),
    2,
    5,
    false);
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (1, 2, 41, 1000, 64, true, 0.0),
    2,
    5,
    false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();
  Require (sender->GetMac ().GetTransmittedFrameCount () == 2 &&
           sender->GetMac ().GetQueuedFrameCount () == 0 &&
           g_receiveCount == 2,
           "unknown high-rate concat limit stalled a valid queue head");
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  RngSeedManager::SetSeed (0x435352);
  RngSeedManager::SetRun (29);

  TestRateConfigurationAndPropagation ();
  RunLinkControlCase (128, 128, 14.0);
  RunLinkControlCase (500, 500, 22.0);
  RunLinkControlCase (1000, 1000, 25.0);
  TestAutomaticMacRateSelection ();
  TestAutomaticQueuedMacRateSelection ();
  RunLiveDqpskSuccess (500, 10.0);
  RunLiveDqpskSuccess (1000, 13.010299956639812);
  TestLiveDqpskEccDrop ();
  TestLiveSameRateDqpskCollision ();
  TestHighRateQueueHeadProgress ();

  std::cout << "PASS: live owner-confirmed high-rate/DQPSK integration test"
            << std::endl;
  return 0;
}
