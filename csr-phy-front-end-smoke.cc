#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-phy-model.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

struct CsrPhyFrontEndSmokeAccess
{
  struct PairResult
  {
    uint32_t firstCollisions {0};
    uint32_t secondCollisions {0};
    double firstInterferenceWatts {0.0};
    double secondInterferenceWatts {0.0};
    bool sameSpeed {false};
    double jsrDb {0.0};
    double timeOffsetSeconds {0.0};
    bool firstHasJammer {false};
    bool secondHasJammer {false};
  };

  struct AcquisitionResult
  {
    uint64_t trackedSignalId {0};
    double jsrDb {0.0};
    double timeOffsetSeconds {0.0};
    bool sameSpeed {false};
    bool firstRejected {false};
    bool secondRejected {false};
  };

  struct AcquisitionTimelineResult
  {
    std::vector<CsrBerInterval> intervals;
    CsrBerValues beforeBer;
    CsrBerValues afterBer;
    CsrErrorAllocation beforeAllocation;
    CsrErrorAllocation afterAllocation;
  };

  static CsrNetDevice::RxSignal
  MakeSignal (uint64_t id,
              uint64_t arrivalOrder,
              int rateKbps,
              double rxPowerDbm,
              double startSec,
              bool accepted = true)
  {
    CsrNetDevice::RxSignal signal;
    signal.id = id;
    signal.arrivalOrder = arrivalOrder;
    signal.txId = static_cast<CsrNodeId> (id & 0x00ffffffu);
    signal.rateKbps = rateKbps;
    signal.startSec = startSec;
    signal.endSec = 1.0;
    signal.preambleEndSec = 1.0;
    signal.rxPowerDbm = rxPowerDbm;
    signal.frontEnd.channelMatched = true;
    signal.frontEnd.receivedPowerDbm = rxPowerDbm;
    signal.frontEnd.receivedPowerWatts =
      CsrPhyModel::DbmToWatts (rxPowerDbm);
    signal.frontEnd.backgroundNoiseWatts =
      CsrPhyModel::DbmToWatts (-106.975);
    signal.syncEligible = true;
    signal.rejected = !accepted;
    return signal;
  }

  static PairResult
  ApplyPair (int arrivingRate,
             int previousRate,
             bool arrivingAccepted,
             bool previousAccepted,
             double arrivingPowerDbm,
             double previousPowerDbm,
             double arrivingStartSec = 0.02,
             double previousStartSec = 0.0,
             bool exactNearMiss = false)
  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (9);
    CsrNetDevice::RxSignal arriving = MakeSignal (
      2, 2, arrivingRate, arrivingPowerDbm, arrivingStartSec,
      arrivingAccepted);
    CsrNetDevice::RxSignal previous = MakeSignal (
      1, 1, previousRate, previousPowerDbm, previousStartSec,
      previousAccepted);
    if (exactNearMiss)
      {
        previous.endSec = Simulator::Now ().GetSeconds ();
      }

    device->ApplyInterferencePair (arriving, previous);
    return {
      arriving.collisionCount,
      previous.collisionCount,
      arriving.currentInterferenceWatts,
      previous.currentInterferenceWatts,
      device->m_rxSameSpeed,
      device->m_rxJsrDb,
      device->m_rxTimeOffsetSeconds,
      arriving.sameRateInterference,
      previous.sameRateInterference
    };
  }

  static bool
  UsesArrivalOrderForSingletonState ()
  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (9);
    device->SetSyncSnrThresholdOverrideDb (-1.0e9);

    // Signal IDs deliberately sort in the reverse of the first two arrivals.
    // The third 16-kbit/s packet must process prior rate 8 and then prior rate
    // 16, leaving same_speed=true after the last pair.
    CsrNetDevice::RxSignal first = MakeSignal (200, 0, 8, -80.0, 0.0);
    first.txPowerDbm = 0.0;
    CsrNetDevice::RxSignal second = MakeSignal (100, 0, 16, -80.0, 0.0);
    second.txPowerDbm = 0.0;
    CsrNetDevice::RxSignal arriving = MakeSignal (300, 0, 16, -80.0, 0.0);
    arriving.txPowerDbm = 0.0;
    device->BeginReceiveSignal (first);
    device->BeginReceiveSignal (second);
    device->BeginReceiveSignal (arriving);
    bool sameSpeed = device->m_rxSameSpeed;
    Simulator::Destroy ();
    return sameSpeed;
  }

  static double
  RemoveAsymmetricContributor ()
  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (9);
    CsrNetDevice::RxSignal arriving = MakeSignal (
      2, 2, 128, -80.0, 0.02, true);
    CsrNetDevice::RxSignal previous = MakeSignal (
      1, 1, 64, -90.0, 0.0, false);
    device->m_rxSignals.emplace (arriving.id, arriving);
    device->m_rxSignals.emplace (previous.id, previous);
    CsrNetDevice::RxSignal &storedArriving =
      device->m_rxSignals.at (arriving.id);
    CsrNetDevice::RxSignal &storedPrevious =
      device->m_rxSignals.at (previous.id);
    device->ApplyInterferencePair (storedArriving, storedPrevious);
    device->RemoveEndedInterference (storedPrevious);
    return storedArriving.currentInterferenceWatts;
  }

  static bool
  RuntimePersistsPkAcceptRejections ()
  {
    Ptr<CsrNetDevice> belowThresholdDevice =
      CreateObject<CsrNetDevice> (9);
    belowThresholdDevice->SetSyncSnrThresholdOverrideDb (1.0e9);
    CsrNetDevice::RxSignal below = MakeSignal (
      10, 0, 8, -80.0, 0.0);
    below.txPowerDbm = 0.0;
    belowThresholdDevice->BeginReceiveSignal (below);
    bool thresholdRejected =
      belowThresholdDevice->m_rxSignals.at (below.id).rejected &&
      !belowThresholdDevice->m_rxSignals.at (below.id).syncEligible;
    Simulator::Destroy ();

    Ptr<CsrNetDevice> expiredDevice = CreateObject<CsrNetDevice> (9);
    expiredDevice->SetSyncSnrThresholdOverrideDb (-1.0e9);
    CsrNetDevice::RxSignal expired = MakeSignal (
      11, 0, 8, -80.0, 0.0);
    expired.txPowerDbm = 0.0;
    expiredDevice->BeginReceiveSignal (expired);
    expiredDevice->EndReceivePreamble (expired.id);
    bool expiryRejected =
      expiredDevice->m_rxSignals.at (expired.id).rejected;
    Simulator::Destroy ();
    return thresholdRejected && expiryRejected;
  }

  static bool
  ReacceptsSurvivorOnlyAtStartTrack ()
  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (9);
    CsrNetDevice::RxSignal survivor = MakeSignal (
      12, 1, 128, -80.0, 0.0, false);
    survivor.rejectedDuringTrack = true;
    device->m_rxSignals.emplace (survivor.id, survivor);
    device->m_mac.SetReceiveState (CsrMacCore::State::TRACK);
    device->ReturnToSearchAfterReceive ();
    bool stayedRejectedInSearch =
      device->m_rxSignals.at (survivor.id).rejected;
    device->AcquireSignal ();
    bool acceptedAtTrack =
      device->m_trackedSignalId == survivor.id &&
      !device->m_rxSignals.at (survivor.id).rejected &&
      !device->m_rxSignals.at (survivor.id).rejectedDuringTrack;
    Simulator::Destroy ();
    return stayedRejectedInSearch && acceptedAtTrack;
  }

  static AcquisitionResult
  AcquireWithMixedRateCandidates ()
  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (9);
    CsrNetDevice::RxSignal selected = MakeSignal (
      300, 1, 8, -80.0, -0.020);
    CsrNetDevice::RxSignal strongest = MakeSignal (
      100, 2, 16, -70.0, -0.005);
    CsrNetDevice::RxSignal weaker = MakeSignal (
      200, 3, 8, -75.0, -0.004);
    device->m_rxSignals.emplace (selected.id, selected);
    device->m_rxSignals.emplace (strongest.id, strongest);
    device->m_rxSignals.emplace (weaker.id, weaker);
    device->m_rxSameSpeed = true;

    device->AcquireSignal ();
    AcquisitionResult result {
      device->m_trackedSignalId,
      device->m_rxJsrDb,
      device->m_rxTimeOffsetSeconds,
      device->m_rxSameSpeed,
      device->m_rxSignals.at (strongest.id).rejected,
      device->m_rxSignals.at (weaker.id).rejected
    };
    Simulator::Destroy ();
    return result;
  }

  static AcquisitionTimelineResult
  AcquireWithIntervalBoundary ()
  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (9);
    CsrNetDevice::RxSignal selected = MakeSignal (
      300, 1, 128, -80.0, 0.0);
    CsrNetDevice::RxSignal jammer = MakeSignal (
      100, 2, 128, -80.0, 0.001);
    constexpr uint32_t preambleBits = 104;
    double preambleSeconds = static_cast<double> (preambleBits) /
      CsrRateKeyToBps (8);
    double targetSnrDb = -5.740312677277188;

    for (CsrNetDevice::RxSignal *signal : {&selected, &jammer})
      {
        signal->preamble = PREAMBLE_SHORT;
        signal->preambleEndSec = signal->startSec + preambleSeconds;
        signal->endSec = 0.1;
        signal->intervalStartSec = signal->startSec;
        signal->packetBits = 8000;
        signal->collisionCount = 1;
        signal->frontEnd.backgroundNoiseWatts =
          signal->frontEnd.receivedPowerWatts /
          std::pow (10.0, targetSnrDb / 10.0);
        signal->peakNoiseWatts = signal->frontEnd.backgroundNoiseWatts;
        signal->snrDb = CsrPhyModel::ComputeSnrDb (
          signal->frontEnd.receivedPowerWatts,
          signal->frontEnd.backgroundNoiseWatts);
      }

    device->m_rxSignals.emplace (selected.id, selected);
    device->m_rxSignals.emplace (jammer.id, jammer);
    device->m_rxSameSpeed = true;
    device->m_rxJsrDb = -6.0;
    device->m_rxTimeOffsetSeconds = 0.0;

    AcquisitionTimelineResult result;
    Simulator::Schedule (Seconds (0.00663), [device] () {
      device->AcquireSignal ();
    });
    Simulator::Schedule (Seconds (0.030), [device, &result] () {
      device->CloseAllSignalIntervals (
        Simulator::Now ().GetSeconds ());
      const CsrNetDevice::RxSignal &stored =
        device->m_rxSignals.at (300);
      result.intervals = stored.berIntervals;
      if (result.intervals.size () != 2)
        {
          return;
        }
      result.beforeBer = device->m_phy.CalculateBer (
        stored.frontEnd,
        stored.rateKbps,
        result.intervals[0],
        stored.packetBits);
      result.afterBer = device->m_phy.CalculateBer (
        stored.frontEnd,
        stored.rateKbps,
        result.intervals[1],
        stored.packetBits);
      result.beforeAllocation = device->m_phy.AllocateErrors (
        stored.frontEnd,
        stored.startSec,
        preambleBits,
        stored.packetBits,
        stored.rateKbps,
        {result.intervals[0]},
        nullptr);
      result.afterAllocation = device->m_phy.AllocateErrors (
        stored.frontEnd,
        stored.startSec,
        preambleBits,
        stored.packetBits,
        stored.rateKbps,
        {result.intervals[1]},
        nullptr);
    });
    Simulator::Run ();
    Simulator::Destroy ();
    return result;
  }
};

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
TestPkAcceptAndSingletonCollisionState ()
{
  using Access = CsrPhyFrontEndSmokeAccess;

  Access::PairResult both = Access::ApplyPair (
    128, 64, true, true, -80.0, -90.0);
  Require (both.firstCollisions == 1 && both.secondCollisions == 1,
           "different-speed overlap did not increment both collision TDAs");
  RequireNear (both.firstInterferenceWatts,
               CsrPhyModel::DbmToWatts (-90.0),
               1.0e-24,
               "accepted arriving packet did not accumulate previous power");
  RequireNear (both.secondInterferenceWatts,
               CsrPhyModel::DbmToWatts (-80.0),
               1.0e-23,
               "accepted previous packet did not accumulate arriving power");

  Access::PairResult arrivingOnly = Access::ApplyPair (
    128, 64, true, false, -80.0, -90.0);
  RequireNear (arrivingOnly.firstInterferenceWatts,
               CsrPhyModel::DbmToWatts (-90.0),
               1.0e-24,
               "accepted arriving packet lost rejected-packet noise");
  RequireNear (arrivingOnly.secondInterferenceWatts,
               0.0,
               0.0,
               "PK_ACCEPT=false previous packet accumulated noise");

  Access::PairResult previousOnly = Access::ApplyPair (
    128, 64, false, true, -80.0, -90.0);
  RequireNear (previousOnly.firstInterferenceWatts,
               0.0,
               0.0,
               "PK_ACCEPT=false arriving packet accumulated noise");
  RequireNear (previousOnly.secondInterferenceWatts,
               CsrPhyModel::DbmToWatts (-80.0),
               1.0e-23,
               "accepted previous packet lost arriving-packet noise");

  Access::PairResult acceptedAgainstRejected = Access::ApplyPair (
    128, 128, true, false, -90.0, -80.0);
  Require (acceptedAgainstRejected.sameSpeed,
           "same-speed singleton flag was not set");
  RequireNear (acceptedAgainstRejected.jsrDb,
               10.0,
               1.0e-12,
               "same-speed JSR ignored PK_ACCEPT orientation");
  RequireNear (acceptedAgainstRejected.timeOffsetSeconds,
               -0.02,
               1.0e-12,
               "same-speed offset ignored PK_ACCEPT orientation");
  Require (acceptedAgainstRejected.firstHasJammer &&
             !acceptedAgainstRejected.secondHasJammer,
           "per-signal collision metadata ignored PK_ACCEPT orientation");

  Access::PairResult neither = Access::ApplyPair (
    128, 128, false, false, -70.0, -80.0);
  Require (neither.sameSpeed,
           "same_speed was not updated when both packets were rejected");
  RequireNear (neither.jsrDb,
               -1000.0,
               0.0,
               "both-rejected pair overwrote the initialized JSR");
  RequireNear (neither.timeOffsetSeconds,
               0.0,
               0.0,
               "both-rejected pair overwrote the initialized offset");

  Access::PairResult nearMiss = Access::ApplyPair (
    128, 64, true, true, -80.0, -90.0, 0.0, -1.0,
    true);
  Require (nearMiss.firstCollisions == 0 &&
             nearMiss.secondCollisions == 0,
           "exact END_RX/start boundary was treated as a collision");

  Require (Access::UsesArrivalOrderForSingletonState (),
           "last-collision singleton followed signal-ID rather than arrival order");
  RequireNear (Access::RemoveAsymmetricContributor (),
               0.0,
               0.0,
               "END_RX did not remove the exact asymmetric noise contributor");
  Require (Access::RuntimePersistsPkAcceptRejections (),
           "mark_sync/clear_sync PK_ACCEPT rejection was not retained");
  Require (Access::ReacceptsSurvivorOnlyAtStartTrack (),
           "back2search changed PK_ACCEPT before start_track reacquisition");

  Access::AcquisitionResult acquired =
    Access::AcquireWithMixedRateCandidates ();
  Require (acquired.trackedSignalId == 300 &&
             acquired.firstRejected && acquired.secondRejected,
           "start_track selection/rejection order changed");
  RequireNear (acquired.jsrDb,
               10.0,
               1.0e-12,
               "start_track did not use the strongest rejected candidate");
  RequireNear (acquired.timeOffsetSeconds,
               0.015,
               1.0e-12,
               "start_track did not preserve selected/jammer start offset");
  Require (acquired.sameSpeed,
           "start_track incorrectly rewrote br_inoise same_speed state");

  Access::AcquisitionTimelineResult timeline =
    Access::AcquireWithIntervalBoundary ();
  Require (timeline.intervals.size () == 2,
           "start_track did not split the receiver BER timeline");
  RequireNear (timeline.intervals[0].startSec,
               0.0,
               1.0e-15,
               "pre-track BER interval started at the wrong boundary");
  RequireNear (timeline.intervals[0].endSec,
               0.00663,
               1.0e-15,
               "pre-track BER interval ended at the wrong boundary");
  RequireNear (timeline.intervals[0].jsrDb,
               -6.0,
               0.0,
               "start_track retroactively changed the preceding JSR");
  RequireNear (timeline.intervals[0].timeOffsetSeconds,
               0.0,
               0.0,
               "start_track retroactively changed the preceding offset");
  RequireNear (timeline.intervals[1].startSec,
               0.00663,
               1.0e-15,
               "post-track BER interval did not start at acquisition");
  RequireNear (timeline.intervals[1].endSec,
               0.030,
               1.0e-15,
               "post-track BER interval ended at the wrong boundary");
  RequireNear (timeline.intervals[1].jsrDb,
               0.0,
               0.0,
               "post-track BER interval missed the selected/jammer JSR");
  RequireNear (timeline.intervals[1].timeOffsetSeconds,
               0.001,
               1.0e-15,
               "post-track BER interval missed the selected/jammer offset");
  RequireNear (timeline.beforeBer.payloadBer,
               CsrOpnetBerTables::GetCollisionBer (
                 128,
                 -6.0,
                 0.0,
                 timeline.beforeBer.payloadEffectiveSnrDb),
               0.0,
               "pre-track interval selected BER with post-track metadata");
  RequireNear (timeline.afterBer.payloadBer,
               CsrOpnetBerTables::GetCollisionBer (
                 128,
                 0.0,
                 0.001,
                 timeline.afterBer.payloadEffectiveSnrDb),
               0.0,
               "post-track interval did not select its own collision BER");
  Require (timeline.beforeBer.payloadBer != timeline.afterBer.payloadBer,
           "BER timeline fixture did not distinguish the two snapshots");
  Require (timeline.beforeAllocation.headerBits == 0 &&
             timeline.beforeAllocation.payloadBits == 0,
           "pre-acquisition preamble interval unexpectedly allocated bits");
  Require (timeline.afterAllocation.headerBits > 0 &&
             timeline.afterAllocation.payloadBits > 0,
           "post-acquisition interval did not allocate header and payload bits");
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
  TestPkAcceptAndSingletonCollisionState ();
  TestDifferentRateAdditiveNoise ();
  TestSameRateJsrAndTimeOffset ();
  TestChannelMismatchRuntime ();
  std::cout << "PASS: OPNET PHY front-end parity test" << std::endl;
  return 0;
}
