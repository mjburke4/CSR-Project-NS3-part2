#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hello-header.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-opnet-ber-tables.h"
#include "ns3/csr-phy-model.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

uint32_t g_receiveCount = 0;

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
RequireEqual (uint32_t actual,
              uint32_t expected,
              const char *message)
{
  if (actual != expected)
    {
      std::cerr << "FAIL: " << message
                << " expected=" << expected
                << " actual=" << actual
                << std::endl;
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
RecordReception (Ptr<Packet>, double, double)
{
  g_receiveCount++;
}

Ptr<Packet>
BuildFrame (CsrNodeId source,
            CsrNodeId destination,
            uint16_t sequence,
            uint32_t payloadBytes = 512)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    5,
                    false,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (128);

  Ptr<Packet> frame = Create<Packet> (payloadBytes);
  frame->AddHeader (header);
  return frame;
}

void
SendFrame (Ptr<CsrNetDevice> sender,
           CsrNodeId destination,
           uint16_t sequence,
           double txPowerDbm)
{
  sender->SendToPeer (BuildFrame (sender->GetId (),
                                  destination,
                                  sequence),
                      destination,
                      128,
                      txPowerDbm,
                      PREAMBLE_SHORT,
                      3,
                      false);
}

void
TestExactRates ()
{
  struct RateVector
  {
    int key;
    double symbolSeconds;
    double bitRate;
    double processingGainDb;
  };

  const RateVector vectors[] = {
    {8, 0.000510, 7843.137254901961, 18.044801891059926},
    {16, 0.000254, 15748.031496062993, 15.017437296279944},
    {32, 0.000126, 31746.031746031746, 11.972805581256194},
    {64, 0.000062, 64516.12903225806, 8.893017025063104},
    {128, 0.000030, 133333.33333333334, 5.740312677277188},
    {500, 0.000008, 500000.0, 0.0},
    {1000, 0.000004, 1000000.0, -3.010299956639812}
  };

  for (const auto &vector : vectors)
    {
      RequireNear (CsrSymbolDurationSeconds (vector.key),
                   vector.symbolSeconds,
                   1.0e-15,
                   "source symbol duration changed");
      RequireNear (CsrOpnetBerTables::SymbolDurationSeconds (vector.key),
                   vector.symbolSeconds,
                   1.0e-15,
                   "modulation-table symbol duration changed");
      RequireNear (CsrRateKeyToBps (vector.key),
                   vector.bitRate,
                   1.0e-9,
                   "rate key was rounded instead of derived from symbol time");
      double processingGainDb = 10.0 * std::log10 (
        1.0e6 / (2.0 * CsrRateKeyToBps (vector.key)));
      RequireNear (processingGainDb,
                   vector.processingGainDb,
                   1.0e-12,
                   "source processing gain changed");
    }

}

void
TestHighRateHeaderRoundTrip ()
{
  RequireEqual (CsrEncodeRateKey (500),
                CSR_RATE_CODE_500_KBPS,
                "500-kbps compact wire code changed");
  RequireEqual (CsrEncodeRateKey (1000),
                CSR_RATE_CODE_1000_KBPS,
                "1000-kbps compact wire code changed");

  const CsrRateKey rates[] = {500, 1000};
  for (CsrRateKey rate : rates)
    {
      CsrHeader header (1, 2, 3, 5, false, false);
      header.SetSpeedKey (rate);
      RequireEqual (header.GetSerializedSize (),
                    13,
                    "high-rate CSR header changed compatibility size");
      Ptr<Packet> packet = Create<Packet> ();
      packet->AddHeader (header);
      CsrHeader decoded;
      packet->RemoveHeader (decoded);
      RequireEqual (decoded.GetSpeedKey (),
                    rate,
                    "high-rate CSR header did not round-trip");

      CsrHelloHeader hello;
      hello.SetSpeedKey (rate);
      Ptr<Packet> helloPacket = Create<Packet> ();
      helloPacket->AddHeader (hello);
      CsrHelloHeader decodedHello;
      helloPacket->RemoveHeader (decodedHello);
      RequireEqual (decodedHello.GetSpeedKey (),
                    rate,
                    "high-rate HELLO metadata did not round-trip");
    }
}

void
TestJsrAndOffsetQuantization ()
{
  struct JsrVector
  {
    double input;
    int bucket;
  };

  const JsrVector jsrVectors[] = {
    {-1000.0, -12},
    {-10.500001, -12},
    {-10.5, -12},
    {-10.499999, -9},
    {-7.5, -9},
    {-7.499999, -6},
    {-4.5, -6},
    {-4.499999, -3},
    {-1.5, -3},
    {-1.499999, 0},
    {20.0, 0}
  };
  for (const auto &vector : jsrVectors)
    {
      Require (CsrOpnetBerTables::QuantizeJsrDb (vector.input) ==
                 vector.bucket,
               "JSR bucket boundary differs from br_ber");
    }

  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (128, 0.0),
               0.0,
               0.0,
               "zero timing offset was not circular origin");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (128, 0.4999e-6),
               0.0,
               0.0,
               "sub-half-microsecond offset rounded up");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (128, 0.5001e-6),
               0.5,
               0.0,
               "half-chip offset did not round up");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (128, 1.5001e-6),
               1.0,
               0.0,
               "one-chip offset quantized incorrectly");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (128, -0.4999e-6),
               0.0,
               0.0,
               "negative circular offset did not wrap to zero");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (128, -0.5001e-6),
               14.5,
               0.0,
               "negative circular offset selected the wrong final table");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (128, 30.0e-6),
               0.0,
               0.0,
               "one symbol did not wrap to offset zero");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (8, 509.5001e-6),
               0.0,
               0.0,
               "S0 final half-chip did not wrap to table zero");
  RequireNear (CsrOpnetBerTables::QuantizeChipOffset (16, 127.0e-6),
               63.5,
               0.0,
               "16-kbps half-chip table selection changed");
}

void
TestIndependentBerVectors ()
{
  struct StandardVector
  {
    double effectiveSnrDb;
    double ber;
  };
  const StandardVector standardVectors[] = {
    {-20.0, 0.496806122489},
    {-10.0, 0.466562489451},
    {0.0, 0.174688707266},
    {10.0, 8.2e-9},
    {11.7, 1.0e-12},
    {11.8, 0.0}
  };
  for (const auto &vector : standardVectors)
    {
      RequireNear (CsrOpnetBerTables::GetStandardBer (
                     vector.effectiveSnrDb),
                   vector.ber,
                   1.0e-15,
                   "standard csr modulation-table vector changed");
    }

  struct DqpskVector
  {
    double effectiveSnrDb;
    double ber;
  };
  const DqpskVector dqpskVectors[] = {
    {-20.0, 0.658547624308},
    {-10.0, 0.554134144522},
    {0.0, 0.204905205908},
    {5.0, 0.039282506101},
    {10.0, 0.000445472657},
    {16.25, 1.0e-12},
    {20.0, 0.0}
  };
  for (const auto &vector : dqpskVectors)
    {
      RequireNear (CsrOpnetBerTables::GetDqpskBer (
                     vector.effectiveSnrDb),
                   vector.ber,
                   1.0e-15,
                   "DQPSK modulation-table vector changed");
    }

  struct CollisionVector
  {
    int rateKbps;
    double jsrDb;
    double timeOffsetSeconds;
    double effectiveSnrDb;
    double ber;
  };
  const CollisionVector collisionVectors[] = {
    {8, -12.0, 0.0, 3.9794, 0.019057765152},
    {16, -9.0, 127.0e-6, 3.9794, 0.017468631629},
    {32, -3.0, 63.0e-6, 3.9794, 0.020387961648},
    {64, 0.0, 31.0e-6, 3.9794, 0.037136008523},
    {128, 0.0, 0.0, -6.0206, 0.421677468040}
  };
  for (const auto &vector : collisionVectors)
    {
      RequireNear (CsrOpnetBerTables::GetCollisionBer (
                     vector.rateKbps,
                     vector.jsrDb,
                     vector.timeOffsetSeconds,
                     vector.effectiveSnrDb),
                   vector.ber,
                   1.0e-15,
                   "collision modulation-table vector changed");
    }
}

void
TestInterpolationAndClamp ()
{
  RequireNear (CsrOpnetBerTables::GetStandardBer (-100.0),
               0.496806122489,
               1.0e-15,
               "standard BER did not clamp below its first sample");
  RequireNear (CsrOpnetBerTables::GetStandardBer (100.0),
               0.0,
               0.0,
               "standard BER did not clamp above its last sample");
  RequireNear (CsrOpnetBerTables::GetStandardBer (-19.95),
               0.49676869544900004,
               1.0e-14,
               "standard BER interpolation is not arithmetic linear");

  RequireNear (CsrOpnetBerTables::GetCollisionBer (
                 128, 0.0, 0.0, -100.0),
               0.421677468040,
               1.0e-15,
               "collision BER did not clamp below its first sample");
  RequireNear (CsrOpnetBerTables::GetCollisionBer (
                 128, 0.0, 0.0, 100.0),
               0.250015536222,
               1.0e-15,
               "collision BER incorrectly used display bounds as clamps");
  RequireNear (CsrOpnetBerTables::GetCollisionBer (
                 128, 0.0, 0.0, -5.5206),
               0.413053200462,
               1.0e-14,
               "collision BER interpolation is not arithmetic linear");
}

void
TestBerSelectionAndProcessingGain ()
{
  CsrPhyModel phy;
  CsrPhyFrontEndResult frontEnd;
  frontEnd.channelMatched = true;
  frontEnd.receivedPowerWatts = 1.0;

  CsrBerInterval standard;
  standard.noisePowerWatts = std::pow (
    10.0, 5.740312677277188 / 10.0);
  CsrBerValues values = phy.CalculateBer (frontEnd, 128, standard);
  RequireNear (values.snrDb,
               -5.740312677277188,
               1.0e-12,
               "interval SNR calculation changed");
  RequireNear (values.payloadEffectiveSnrDb,
               0.0,
               1.0e-12,
               "payload processing gain changed");
  RequireNear (values.headerEffectiveSnrDb,
               12.304489213782738,
               1.0e-12,
               "S0 header processing gain changed");
  RequireNear (values.payloadBer,
               0.174688707266,
               1.0e-15,
               "standard payload table was not selected");
  RequireNear (values.headerBer,
               0.0,
               0.0,
               "standard header table was not selected");

  CsrBerInterval differentRate = standard;
  differentRate.collisionCount = 1;
  differentRate.sameRateInterference = false;
  differentRate.jsrDb = 0.0;
  values = phy.CalculateBer (frontEnd, 128, differentRate);
  RequireNear (values.payloadBer,
               0.174688707266,
               1.0e-15,
               "different-rate collision did not keep standard payload BER");

  CsrBerInterval sameRate;
  sameRate.noisePowerWatts = std::pow (
    10.0, 11.760912677277188 / 10.0);
  sameRate.collisionCount = 1;
  sameRate.sameRateInterference = true;
  sameRate.jsrDb = 0.0;
  sameRate.timeOffsetSeconds = 0.0;
  values = phy.CalculateBer (frontEnd, 128, sameRate);
  RequireNear (values.payloadEffectiveSnrDb,
               -6.0206,
               1.0e-12,
               "same-rate payload effective SNR changed");
  RequireNear (values.payloadBer,
               0.421677468040,
               1.0e-15,
               "same-rate collision table was not selected");

  CsrBerInterval dqpsk;
  dqpsk.noisePowerWatts = 1.0;
  values = phy.CalculateBer (frontEnd, 500, dqpsk);
  RequireNear (values.payloadEffectiveSnrDb,
               0.0,
               1.0e-12,
               "500-kbps processing gain changed");
  RequireNear (values.payloadBer,
               0.204905205908,
               1.0e-15,
               "500-kbps payload did not select DQPSK");

  dqpsk.noisePowerWatts = 0.5;
  values = phy.CalculateBer (frontEnd, 1000, dqpsk);
  RequireNear (values.payloadEffectiveSnrDb,
               0.0,
               1.0e-12,
               "1000-kbps processing gain changed");
  RequireNear (values.payloadBer,
               0.204905205908,
               1.0e-15,
               "1000-kbps payload did not select DQPSK");

  dqpsk.noisePowerWatts = 1.0;
  dqpsk.collisionCount = 1;
  dqpsk.sameRateInterference = true;
  dqpsk.jsrDb = 0.0;
  values = phy.CalculateBer (frontEnd, 500, dqpsk);
  RequireNear (values.payloadEffectiveSnrDb,
               -3.010299956639812,
               1.0e-12,
               "same-rate DQPSK jammer was not treated as payload noise");
  RequireNear (values.payloadBer,
               CsrOpnetBerTables::GetDqpskBer (
                 -3.010299956639812),
               1.0e-15,
               "same-rate high-rate overlap selected a spread collision curve");
}

void
TestIntervalAccounting ()
{
  CsrPhyModel phy;
  CsrPhyFrontEndResult frontEnd;
  frontEnd.channelMatched = true;
  frontEnd.receivedPowerWatts = 1.0;

  static constexpr uint32_t preambleBits = 104;
  static constexpr uint32_t fixedHeaderBits = 48;
  static constexpr uint32_t payloadBits = 8;
  static constexpr uint32_t fcsBits = 32;
  static constexpr uint32_t packetBits =
    preambleBits + fixedHeaderBits + payloadBits + fcsBits;
  const double headerRate = CsrRateKeyToBps (8);
  const double payloadRate = CsrRateKeyToBps (128);
  const double signalStart = 0.0;
  const double headerStart = signalStart + preambleBits / headerRate;
  const double payloadStart = headerStart + fixedHeaderBits / headerRate;
  const double signalEnd = payloadStart +
    (payloadBits + fcsBits) / payloadRate;

  CsrBerInterval complete;
  complete.startSec = signalStart;
  complete.endSec = signalEnd;
  complete.noisePowerWatts = 1.0e-30;
  CsrErrorAllocation allocation = phy.AllocateErrors (
    frontEnd,
    signalStart,
    preambleBits,
    packetBits,
    128,
    {complete},
    nullptr);
  RequireEqual (allocation.headerBits,
                fixedHeaderBits,
                "preamble was included in interval header accounting");
  RequireEqual (allocation.payloadBits,
                payloadBits + fcsBits,
                "payload accounting omitted the FCS");
  RequireEqual (allocation.totalErrors,
                0,
                "zero-BER interval allocated errors");

  CsrBerInterval first = complete;
  first.endSec = headerStart + 3.75 / headerRate;
  CsrBerInterval second = complete;
  second.startSec = first.endSec;
  second.endSec = payloadStart + 2.75 / payloadRate;
  CsrBerInterval third = complete;
  third.startSec = second.endSec;
  third.endSec = signalEnd + 1.0;
  allocation = phy.AllocateErrors (frontEnd,
                                   signalStart,
                                   preambleBits,
                                   packetBits,
                                   128,
                                   {first, second, third},
                                   nullptr);
  RequireEqual (allocation.headerBits,
                47,
                "header interval bit counts were not truncated separately");
  RequireEqual (allocation.payloadBits,
                39,
                "payload interval bit counts were not truncated separately");
  RequireEqual (allocation.totalErrors,
                0,
                "split zero-BER intervals allocated errors");
}

void
TestSourceBinomialVectors ()
{
  struct BinomialVector
  {
    uint32_t bits;
    double ber;
    double uniform;
    uint32_t errors;
  };
  const BinomialVector vectors[] = {
    {0, 0.2, 0.5, 0},
    {10, 0.0, 0.9, 0},
    {10, 1.0, 0.1, 10},
    {10, 0.2, 0.1, 0},
    {10, 0.2, 0.2, 1},
    {10, 0.2, 0.5, 2},
    {10, 0.2, 0.9, 4},
    {10, 0.8, 0.1, 10},
    {10, 0.8, 0.5, 8},
    {10, 0.8, 0.9, 6},
    {7, 0.5, 0.49, 3},
    {7, 0.5, 0.51, 4},
    {100, 0.01, 0.99, 4}
  };
  for (const auto &vector : vectors)
    {
      RequireEqual (CsrPhyModel::SampleSourceBinomial (
                      vector.bits,
                      vector.ber,
                      vector.uniform),
                    vector.errors,
                    "source inverse-CDF binomial vector changed");
    }
}

void
TestEccOrderingAndBoundaries ()
{
  CsrPhyModel phy;
  CsrPhyProfile profile = phy.profile;
  profile.eccThreshold = 0.1;
  phy.SetProfile (profile);

  static constexpr uint32_t preambleBits = 104;
  static constexpr uint32_t packetBits = preambleBits + 48 + 8 + 32;
  CsrEccResult result = phy.EvaluateEcc (
    packetBits, preambleBits, 8, true, false, false);
  Require (result.accepted,
           "inclusive ECC threshold rejected its boundary error count");
  Require (!result.eccDropped,
           "accepted packet was counted as an ECC drop");
  RequireEqual (result.protectedBits,
                88,
                "ECC denominator did not exclude only the preamble");
  RequireEqual (result.correctableBits,
                8,
                "ECC correction limit was not integer-truncated");

  result = phy.EvaluateEcc (
    packetBits, preambleBits, 9, true, false, false);
  Require (!result.accepted && result.eccDropped,
           "ECC did not reject one error above its inclusive threshold");

  result = phy.EvaluateEcc (
    packetBits, preambleBits, 6, true, false, false);
  Require (result.accepted,
           "32-bit FCS was omitted from the ECC denominator");

  result = phy.EvaluateEcc (
    packetBits, preambleBits, 0, false, false, false);
  Require (!result.accepted && !result.eccDropped,
           "ECC overrode or claimed an earlier-stage rejection");

  result = phy.EvaluateEcc (
    packetBits, preambleBits, 0, true, true, false);
  Require (!result.accepted && !result.eccDropped,
           "locked receiver was accepted or counted as an ECC failure");

  result = phy.EvaluateEcc (
    packetBits, preambleBits, 0, true, false, true);
  Require (!result.accepted && !result.eccDropped,
           "node-failed packet was accepted or counted as an ECC failure");

  result = phy.EvaluateEcc (preambleBits,
                            preambleBits,
                            200,
                            true,
                            false,
                            false);
  Require (result.accepted,
           "zero-length protected packet was rejected");
}

void
TestClosureModes ()
{
  CsrPhyModel phy;
  CsrTxRadioProfile transmitter = phy.GetTxRadioProfile ();
  transmitter.antennaHeightMeters = 1.0;

  CsrPhyProfile profile = phy.profile;
  profile.rxHeightMeters = 1.0;
  profile.closureMode = CsrClosureMode::NEVER_OCCLUDED;
  phy.SetProfile (profile);
  Require (phy.HasClosure (1.0e9, transmitter),
           "never-occluded closure rejected a path");

  profile.closureMode = CsrClosureMode::EARTH_LINE_OF_SIGHT;
  phy.SetProfile (profile);
  double oneMeterHorizon = std::sqrt (
    1.0 * (2.0 * profile.earthRadiusMeters + 1.0));
  double combinedHorizon = 2.0 * oneMeterHorizon;
  Require (phy.HasClosure (combinedHorizon, transmitter),
           "spherical-Earth closure rejected its inclusive horizon");
  Require (!phy.HasClosure (combinedHorizon + 1.0, transmitter),
           "spherical-Earth closure accepted an occluded path");

  profile.closureMode = CsrClosureMode::DELEGATE;
  phy.SetProfile (profile);
  uint32_t delegateCalls = 0;
  phy.SetClosureModel (
    [&delegateCalls] (double distance, double txHeight, double rxHeight) {
      delegateCalls++;
      return distance == 123.0 && txHeight == 1.0 && rxHeight == 1.0;
    });
  Require (phy.HasClosure (123.0, transmitter),
           "closure delegate acceptance was ignored");
  Require (!phy.HasClosure (124.0, transmitter),
           "closure delegate rejection was ignored");
  RequireEqual (delegateCalls,
                2,
                "closure delegate was not called exactly once per query");

  phy.SetClosureModel (CsrClosureModelFn {});
  Require (!phy.HasClosure (combinedHorizon + 1.0, transmitter),
           "empty closure delegate did not fall back to Earth LOS");
}

void
TestClosureBeforeReceivePipeline ()
{
  g_receiveCount = 0;
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 2, 100.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  CsrPhyProfile profile = receiver->GetPhy ().profile;
  profile.closureMode = CsrClosureMode::DELEGATE;
  receiver->GetPhy ().SetProfile (profile);
  uint32_t delegateCalls = 0;
  receiver->GetPhy ().SetClosureModel (
    [&delegateCalls] (double, double, double) {
      delegateCalls++;
      return false;
    });

  SendFrame (sender, 2, 90, 0.0);
  Simulator::Stop (Seconds (0.08));
  Simulator::Run ();

  RequireEqual (delegateCalls,
                1,
                "closure was not evaluated exactly once per arriving signal");
  Require (g_receiveCount == 0,
           "occluded signal reached the MAC delivery path");
  Require (!receiver->HasLastRxDecision (),
           "occluded signal entered the post-closure receive pipeline");
  Simulator::Destroy ();
}

CsrRxDecision
RunSelectedCollision (double jammerPowerDbm,
                      bool expectDelivery)
{
  g_receiveCount = 0;
  Ptr<CsrNetDevice> desired = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> jammer = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  desired->AddPeer (receiver);
  jammer->AddPeer (receiver);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 3, 100.0);
  receiver->GetPhy ().SetLinkDistanceMeters (2, 3, 100.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendFrame (desired, 3, 100, 0.0);
  Simulator::Schedule (Seconds (0.00765),
                       &SendFrame,
                       jammer,
                       3,
                       101,
                       jammerPowerDbm);
  Simulator::Stop (Seconds (0.12));
  Simulator::Run ();

  Require (receiver->GetRxCollisionCount () >= 1,
           "post-lock overlap did not record a collision");
  Require (receiver->HasLastRxDecision (),
           "selected collision produced no final PHY decision");
  CsrRxDecision decision = receiver->GetLastRxDecision ();
  Require (decision.sameRateInterference,
           "selected collision did not use the JSR table path");
  Require ((g_receiveCount == 1) == expectDelivery,
           "selected collision delivery disagrees with BER/ECC outcome");
  return decision;
}

void
TestSelectedCollisionFinalOrdering ()
{
  CsrRxDecision captured = RunSelectedCollision (-20.0, true);
  Require (captured.success,
           "selected weak collision was hard-dropped before BER/ECC");
  Require (captured.totalErrors <= captured.correctableBits,
           "accepted selected collision exceeded its ECC allowance");
  Simulator::Destroy ();

  CsrRxDecision failed = RunSelectedCollision (0.0, false);
  Require (!failed.success,
           "selected strong collision bypassed its final drop");
  Require (failed.eccDropped,
           "selected strong collision was hard-dropped before ECC");
  Require (failed.totalErrors > failed.correctableBits,
           "selected strong collision did not exceed its ECC allowance");
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  TestExactRates ();
  TestHighRateHeaderRoundTrip ();
  TestJsrAndOffsetQuantization ();
  TestIndependentBerVectors ();
  TestInterpolationAndClamp ();
  TestBerSelectionAndProcessingGain ();
  TestIntervalAccounting ();
  TestSourceBinomialVectors ();
  TestEccOrderingAndBoundaries ();
  TestClosureModes ();
  TestClosureBeforeReceivePipeline ();
  TestSelectedCollisionFinalOrdering ();
  std::cout << "PASS: OPNET modulation BER, ECC, and closure parity test"
            << std::endl;
  return 0;
}
