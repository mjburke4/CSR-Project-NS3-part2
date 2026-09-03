#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-phy-model.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

uint32_t g_receiveCount = 0;
double g_receivedSnrDb = 0.0;

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
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  device->GetPhy ().SetPerModel (
    [] (int, double, uint32_t) { return 0.0; });
}

Ptr<Packet>
BuildFrame (CsrNodeId source,
            CsrNodeId destination,
            uint16_t sequence,
            int rateKbps)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    5,
                    false,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (rateKbps);

  Ptr<Packet> frame = Create<Packet> (16);
  frame->AddHeader (header);
  return frame;
}

void
SendFrame (Ptr<CsrNetDevice> sender,
           CsrNodeId destination,
           uint16_t sequence,
           int rateKbps,
           double txPowerDbm)
{
  sender->SendToPeer (BuildFrame (sender->GetId (),
                                  destination,
                                  sequence,
                                  rateKbps),
                      destination,
                      rateKbps,
                      txPowerDbm,
                      PREAMBLE_LONG,
                      3,
                      false);
}

void
RecordReception (Ptr<Packet>, double, double snrDb)
{
  g_receiveCount++;
  g_receivedSnrDb = snrDb;
}

void
TestExactPowerEquations ()
{
  CsrPhyModel phy;
  RequireNear (phy.profile.txBaseFrequencyHz,
               30.0e6,
               1.0e-6,
               "project transmitter frequency default changed");
  RequireNear (phy.profile.rxBwHz,
               1.0e6,
               1.0e-6,
               "project receiver bandwidth default changed");
  RequireNear (phy.profile.noiseFloorDbm,
               -106.975,
               1.0e-12,
               "br_support noise floor default changed");

  CsrPhyFrontEndResult result = phy.ComputeFrontEnd (
    0.0,
    100.0,
    phy.GetTxRadioProfile ());
  Require (result.channelMatched,
           "full-overlap default channel did not match");
  Require (result.pathModel == CsrPathGainModel::FLAT_EARTH,
           "three-path rule did not choose its minimum gain");
  RequireNear (result.bandOverlapHz,
               1.0e6,
               1.0e-6,
               "full-band overlap is incorrect");
  RequireNear (result.pathGainLinear,
               1.0e-8,
               1.0e-20,
               "flat-earth path gain differs from br_power");
  RequireNear (result.pathlossDb,
               80.0,
               1.0e-10,
               "source-backed path loss is incorrect");
  RequireNear (result.inBandTxPowerWatts,
               1.0e-3,
               1.0e-15,
               "in-band transmit power is incorrect");
  RequireNear (result.receivedPowerWatts,
               1.0e-11,
               1.0e-23,
               "received power differs from br_power");
  RequireNear (result.receivedPowerDbm,
               -80.0,
               1.0e-10,
               "received dBm conversion is incorrect");
  RequireNear (result.snrDb,
               26.975,
               1.0e-10,
               "background-noise SNR differs from br_snr");

  CsrPhyProfile widebandProfile = phy.profile;
  widebandProfile.rxBwHz = 2.0e6;
  phy.SetProfile (widebandProfile);
  CsrPhyFrontEndResult wideband = phy.ComputeFrontEnd (
    0.0,
    100.0,
    phy.GetTxRadioProfile ());
  RequireNear (wideband.noisePowerDbm,
               -103.96470004336019,
               1.0e-9,
               "background noise did not scale with receiver bandwidth");
  RequireNear (wideband.snrDb,
               23.96470004336018,
               1.0e-9,
               "wideband SNR is incorrect");
}

void
TestBandOverlapAndPathSelection ()
{
  CsrPhyModel phy;
  CsrPhyProfile receiver = phy.profile;
  receiver.rxBaseFrequencyHz = 30.5e6;
  receiver.txAntennaGainDb = 6.0;
  receiver.rxAntennaGainDb = 3.0;
  phy.SetProfile (receiver);

  CsrTxRadioProfile transmitter = phy.GetTxRadioProfile ();
  transmitter.baseFrequencyHz = 30.0e6;
  CsrPhyFrontEndResult partial = phy.ComputeFrontEnd (
    0.0,
    100.0,
    transmitter);
  RequireNear (partial.bandOverlapHz,
               500000.0,
               1.0e-6,
               "partial-band overlap is incorrect");
  RequireNear (partial.receivedPowerDbm,
               -74.01029995663981,
               1.0e-9,
               "overlap and antenna gains were not applied in order");
  RequireNear (partial.snrDb,
               32.96470004336018,
               1.0e-9,
               "partial-overlap SNR is incorrect");

  transmitter.baseFrequencyHz = 40.0e6;
  CsrPhyFrontEndResult mismatch = phy.ComputeFrontEnd (
    0.0,
    100.0,
    transmitter);
  Require (!mismatch.channelMatched,
           "nonoverlapping transmitter passed channel qualification");
  RequireNear (mismatch.bandOverlapHz,
               0.0,
               0.0,
               "nonoverlapping channels have nonzero overlap");

  CsrPhyModel highFrequencyPhy;
  CsrPhyProfile highFrequency = highFrequencyPhy.profile;
  highFrequency.txBaseFrequencyHz = 2.3995e9;
  highFrequency.rxBaseFrequencyHz = 2.3995e9;
  highFrequencyPhy.SetProfile (highFrequency);
  CsrPhyFrontEndResult wlan = highFrequencyPhy.ComputeFrontEnd (
    0.0,
    100.0,
    highFrequencyPhy.GetTxRadioProfile ());
  Require (wlan.pathModel == CsrPathGainModel::AD_HOC_WLAN,
           "three-path rule did not select the ad-hoc WLAN model");
  RequireNear (wlan.pathlossDb,
               87.60422483423213,
               1.0e-9,
               "ad-hoc WLAN path-loss vector is incorrect");
}

void
TestDifferentRateAdditiveNoise ()
{
  g_receiveCount = 0;
  g_receivedSnrDb = 0.0;
  Ptr<CsrNetDevice> desired = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> jammer = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  desired->AddPeer (receiver);
  jammer->AddPeer (receiver);
  ConfigureNoErrors (desired);
  ConfigureNoErrors (jammer);
  ConfigureNoErrors (receiver);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 3, 100.0);
  receiver->GetPhy ().SetLinkDistanceMeters (2, 3, 100.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendFrame (desired, 3, 100, 128, 0.0);
  SendFrame (jammer, 3, 101, 64, -20.0);
  Simulator::Stop (Seconds (1.2));
  Simulator::Run ();

  Require (g_receiveCount == 1,
           "different-rate additive noise caused a hard collision");
  Require (receiver->HasLastRxDecision (),
           "different-rate signal produced no PHY diagnostics");
  const CsrRxDecision &decision = receiver->GetLastRxDecision ();
  Require (!decision.sameRateInterference,
           "different-rate jammer entered the JSR path");
  RequireNear (decision.receivedPowerDbm,
               -80.0,
               1.0e-9,
               "runtime received power differs from br_power");
  RequireNear (decision.noisePowerDbm,
               -99.20573407436032,
               1.0e-9,
               "different-rate jammer was not accumulated in watts");
  RequireNear (decision.snrDb,
               19.20573407436033,
               1.0e-9,
               "runtime br_snr result is incorrect");
  RequireNear (g_receivedSnrDb,
               decision.snrDb,
               1.0e-12,
               "upper layer did not receive the front-end SNR");
  Simulator::Destroy ();
}

void
TestSameRateJsrAndTimeOffset ()
{
  g_receiveCount = 0;
  Ptr<CsrNetDevice> desired = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> jammer = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  desired->AddPeer (receiver);
  jammer->AddPeer (receiver);
  ConfigureNoErrors (desired);
  ConfigureNoErrors (jammer);
  ConfigureNoErrors (receiver);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 3, 100.0);
  receiver->GetPhy ().SetLinkDistanceMeters (2, 3, 100.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  Time firstDuration = desired->SendToPeer (
    BuildFrame (desired->GetId (), 3, 110, 128),
    3,
    128,
    0.0,
    PREAMBLE_LONG,
    3,
    false);
  Simulator::Schedule (MilliSeconds (20),
                       &SendFrame,
                       jammer,
                       3,
                       111,
                       128,
                       -6.0);

  Time propagation = Seconds (
    100.0 / CsrPhyModel::SPEED_OF_LIGHT_METERS_PER_SECOND);
  Time firstCompletion = propagation + firstDuration;
  Time backToSearch = firstCompletion + CsrOpnetTic ();
  Time replacementAcquisition = backToSearch + Seconds (0.00663);
  bool firstCollisionChecked = false;

  Simulator::Schedule (
    firstCompletion + NanoSeconds (1),
    [receiver, &firstCollisionChecked] () {
      Require (receiver->GetMacState () == CsrMacCore::State::TRACK,
               "pipeline collision returned before END_RX plus TIC");
      Require (receiver->HasLastRxDecision (),
               "same-rate collision produced no PHY diagnostics");
      const CsrRxDecision &decision = receiver->GetLastRxDecision ();
      Require (!decision.success,
               "temporary same-rate collision accepted the first frame");
      Require (decision.sameRateInterference,
               "same-rate jammer did not enter the modulation-table path");
      RequireNear (decision.jsrDb,
                   -6.0,
                   1.0e-9,
                   "same-rate JSR differs from br_inoise");
      RequireNear (decision.timeOffsetSeconds,
                   0.02,
                   1.0e-9,
                   "same-rate time offset differs from br_inoise");
      RequireNear (decision.noisePowerDbm,
                   -106.975,
                   1.0e-9,
                   "same-rate jammer incorrectly entered additive noise");
      firstCollisionChecked = true;
    });
  Simulator::Schedule (
    replacementAcquisition - NanoSeconds (1),
    [receiver] () {
      Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
               "surviving preamble reacquired before fresh 6.63-ms delay");
    });
  Simulator::Schedule (
    replacementAcquisition + NanoSeconds (1),
    [receiver] () {
      Require (receiver->GetMacState () == CsrMacCore::State::TRACK,
               "surviving preamble was not reacquired after 6.63 ms");
    });
  Simulator::Stop (Seconds (1.2));
  Simulator::Run ();

  Require (firstCollisionChecked,
           "same-rate first-packet decision checkpoint did not run");
  Require (g_receiveCount == 1,
           "surviving same-rate preamble was not delivered after reacquisition");
  Require (receiver->GetLastRxDecision ().success,
           "reacquired replacement failed final PHY delivery");
  Simulator::Destroy ();
}

void
TestChannelMismatchRuntime ()
{
  g_receiveCount = 0;
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);

  CsrPhyProfile receiverProfile = receiver->GetPhy ().profile;
  receiverProfile.rxBaseFrequencyHz = 40.0e6;
  receiver->GetPhy ().SetProfile (receiverProfile);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendFrame (sender, 2, 120, 128, 0.0);
  Simulator::Stop (Seconds (1.2));
  Simulator::Run ();

  Require (g_receiveCount == 0,
           "channel-mismatched signal reached the MAC payload path");
  Require (!receiver->HasLastRxDecision (),
           "channel-mismatched signal entered Track");
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  TestExactPowerEquations ();
  TestBandOverlapAndPathSelection ();
  TestDifferentRateAdditiveNoise ();
  TestSameRateJsrAndTimeOffset ();
  TestChannelMismatchRuntime ();
  std::cout << "PASS: OPNET PHY front-end parity test" << std::endl;
  return 0;
}
