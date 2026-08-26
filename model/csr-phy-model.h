#pragma once

#include "csr-common.h"
#include "csr-opnet-ber-tables.h"

#include <limits>

/** Propagation equations available to the CSR radio front end. */
enum class CsrPropagationModel
{
  OPNET_THREE_PATH,
  LOG_DISTANCE
};

/** Equation selected by the OPNET three-path minimum-gain rule. */
enum class CsrPathGainModel
{
  UNIT_GAIN,
  FREE_SPACE,
  FLAT_EARTH,
  AD_HOC_WLAN,
  LOG_DISTANCE
};

/** Visibility gate executed before the remaining receive pipeline. */
enum class CsrClosureMode
{
  NEVER_OCCLUDED,
  EARTH_LINE_OF_SIGHT,
  DELEGATE
};

/** Configurable transmitter values carried with an over-the-air signal. */
struct CsrTxRadioProfile
{
  double baseFrequencyHz {30.0e6}; ///< Transmitter band lower edge.
  double bandwidthHz {1.0e6}; ///< Transmitter channel bandwidth.
  double antennaGainDb {0.0}; ///< Directional gain toward the receiver.
  double antennaHeightMeters {1.0}; ///< Transmitter altitude/height.
};

/**
 * Radio attributes used by the source-backed CSR PHY front end.
 *
 * The frequency, bandwidth, and one-meter altitude defaults come from the
 * supplied br_node_v1 model. The noise and synchronization defaults come
 * from br_support.h. The log-distance fields remain available for legacy
 * scenarios that explicitly select LOG_DISTANCE.
 */
struct CsrPhyProfile
{
  double txPowerDbm {0.0}; ///< Default transmit power.
  double noiseFloorDbm {-106.975}; ///< Noise at the reference bandwidth.
  double noiseReferenceBwHz {1.0e6}; ///< Bandwidth for noiseFloorDbm.
  bool scaleNoiseWithBandwidth {true}; ///< Apply thermal-noise bandwidth scaling.
  double txBaseFrequencyHz {30.0e6}; ///< Transmitter band lower edge.
  double rxBaseFrequencyHz {30.0e6}; ///< Receiver band lower edge.
  double txBwHz {1.0e6}; ///< Transmitter channel bandwidth.
  double rxBwHz {1.0e6}; ///< Receiver channel bandwidth.
  double txAntennaGainDb {0.0}; ///< Transmitter antenna gain.
  double rxAntennaGainDb {0.0}; ///< Receiver antenna gain.
  double txHeightMeters {1.0}; ///< Transmitter altitude/height.
  double rxHeightMeters {1.0}; ///< Receiver altitude/height.
  double syncSnrThresholdDb {-11.0}; ///< Deterministic SYNC threshold mean.
  double eccThreshold {0.1}; ///< Correctable fraction of non-preamble bits.
  CsrClosureMode closureMode {
    CsrClosureMode::EARTH_LINE_OF_SIGHT
  }; ///< Visibility rule applied before channel matching.
  double earthRadiusMeters {6371000.0}; ///< Radius used by the LOS fallback.
  CsrPropagationModel propagationModel {
    CsrPropagationModel::OPNET_THREE_PATH
  }; ///< Active propagation equation.

  // Compatibility parameters for pre-parity log-distance scenarios.
  double refLossDb {60.0}; ///< Legacy one-meter log-distance loss.
  double pathlossExp {2.0}; ///< Legacy log-distance exponent.
  double distanceScale {1.0}; ///< Scale applied to configured distances.
};

/** Interference state that remains constant over one receive interval. */
struct CsrBerInterval
{
  double startSec {0.0}; ///< Absolute interval start time.
  double endSec {0.0}; ///< Absolute interval end time.
  double noisePowerWatts {0.0}; ///< Background plus different-rate noise.
  uint32_t collisionCount {0}; ///< Cumulative OPNET collision count.
  bool sameRateInterference {false}; ///< Select the payload collision table.
  double jsrDb {-1000.0}; ///< Receiver-global jammer-to-signal ratio.
  double timeOffsetSeconds {0.0}; ///< Receiver-global signed time offset.
};

/** Header and payload BER values selected for one interval. */
struct CsrBerValues
{
  double snrDb {-std::numeric_limits<double>::infinity ()}; ///< Raw SNR.
  double headerEffectiveSnrDb {
    -std::numeric_limits<double>::infinity ()
  }; ///< Header SNR after processing gain.
  double payloadEffectiveSnrDb {
    -std::numeric_limits<double>::infinity ()
  }; ///< Payload SNR after processing gain.
  double headerBer {1.0}; ///< SOF/speed/length BER.
  double payloadBer {1.0}; ///< Payload/FCS BER.
};

/** Realized source-style error counts across all receive intervals. */
struct CsrErrorAllocation
{
  uint32_t headerBits {0}; ///< Tested SOF/speed/length bits.
  uint32_t payloadBits {0}; ///< Tested payload and FCS bits.
  uint32_t headerErrors {0}; ///< Realized header errors.
  uint32_t payloadErrors {0}; ///< Realized payload/FCS errors.
  uint32_t totalErrors {0}; ///< Realized total protected errors.
  double actualBer {0.0}; ///< Realized BER from the most recent interval.
  double headerBer {0.0}; ///< Most recently selected header BER.
  double payloadBer {0.0}; ///< Most recently selected payload BER.
  double minimumSnrDb {std::numeric_limits<double>::infinity ()}; ///< Worst SNR.
  double peakNoisePowerWatts {0.0}; ///< Largest interval noise power.
  double packetErrorProbability {0.0}; ///< Probability of at least one error.
};

/** Final decision made by the full-packet br_ecc rule. */
struct CsrEccResult
{
  bool accepted {false}; ///< Final packet accept flag.
  bool eccDropped {false}; ///< Whether ECC, rather than an earlier stage, dropped it.
  uint32_t protectedBits {0}; ///< Total packet bits excluding preamble.
  uint32_t correctableBits {0}; ///< Inclusive integer correction limit.
};

/** Deterministic result of br_power followed by background br_snr. */
struct CsrPhyFrontEndResult
{
  bool channelMatched {false}; ///< Whether transmitter and receiver overlap.
  CsrPathGainModel pathModel {CsrPathGainModel::UNIT_GAIN}; ///< Selected path model.
  double distanceMeters {0.0}; ///< Propagation distance.
  double bandOverlapHz {0.0}; ///< Intersecting channel bandwidth.
  double pathGainLinear {0.0}; ///< Selected propagation gain.
  double pathlossDb {std::numeric_limits<double>::infinity ()}; ///< Propagation loss.
  double inBandTxPowerWatts {0.0}; ///< Transmitter power inside the receive band.
  double receivedPowerWatts {0.0}; ///< Antenna-adjusted received power.
  double receivedPowerDbm {-std::numeric_limits<double>::infinity ()}; ///< Received dBm.
  double backgroundNoiseWatts {0.0}; ///< Receiver background noise.
  double noisePowerWatts {0.0}; ///< Current total noise.
  double noisePowerDbm {-std::numeric_limits<double>::infinity ()}; ///< Total noise dBm.
  double snrDb {-std::numeric_limits<double>::infinity ()}; ///< Signal-to-noise ratio.
};

/** Final receive decision and the PHY values that produced it. */
struct CsrRxDecision
{
  bool success {false}; ///< Final delivery decision.
  bool closure {true}; ///< Whether the pre-channel visibility stage passed.
  bool channelMatched {false}; ///< Whether the channels overlap.
  bool sameRateInterference {false}; ///< Whether JSR lookup is required.
  CsrPathGainModel pathModel {CsrPathGainModel::UNIT_GAIN}; ///< Selected path model.
  double pathlossDb {std::numeric_limits<double>::infinity ()}; ///< Propagation loss.
  double receivedPowerDbm {-std::numeric_limits<double>::infinity ()}; ///< Signal power.
  double noisePowerDbm {-std::numeric_limits<double>::infinity ()}; ///< Peak noise.
  double snrDb {-std::numeric_limits<double>::infinity ()}; ///< Minimum observed SNR.
  double per {1.0}; ///< Probability of at least one protected-bit error.
  double jsrDb {-std::numeric_limits<double>::infinity ()}; ///< Strongest same-rate JSR.
  double timeOffsetSeconds {0.0}; ///< Same-rate jammer offset.
  double headerBer {1.0}; ///< BER selected for the last header interval.
  double payloadBer {1.0}; ///< BER selected for the last payload interval.
  double actualBer {1.0}; ///< Realized BER over the final allocated interval.
  uint32_t headerErrors {0}; ///< Allocated SOF/speed/length errors.
  uint32_t payloadErrors {0}; ///< Allocated payload/FCS errors.
  uint32_t totalErrors {0}; ///< Allocated protected errors.
  uint32_t protectedBits {0}; ///< Total bits checked by ECC.
  uint32_t correctableBits {0}; ///< Inclusive ECC error limit.
  bool eccDropped {false}; ///< Whether the ECC threshold caused the drop.
};

/** BER model hook indexed by payload rate, effective SNR, and packet bits. */
using CsrPerModelFn = std::function<double(int, double, uint32_t)>;

/** Optional externally supplied visibility test. */
using CsrClosureModelFn = std::function<bool(double, double, double)>;

/** Return the pre-table placeholder error value. */
inline double
CsrPerModelPlaceholder (int rateKbps, double snrDb, uint32_t nBits)
{
  double snrThresholdDb;
  if (rateKbps <= 8)
    {
      snrThresholdDb = -6.0;
    }
  else if (rateKbps <= 16)
    {
      snrThresholdDb = -3.0;
    }
  else if (rateKbps <= 32)
    {
      snrThresholdDb = 0.0;
    }
  else if (rateKbps <= 64)
    {
      snrThresholdDb = 3.0;
    }
  else
    {
      snrThresholdDb = 6.0;
    }

  double error = 1.0 /
    (1.0 + std::exp (3.0 * (snrDb - snrThresholdDb)));
  double sizeFactor = std::min (
    5.0,
    std::max (1.0, static_cast<double> (nBits) / 800.0));
  error = 1.0 - std::pow (1.0 - error, sizeFactor);
  return std::clamp (error, 0.0, 1.0);
}

/**
 * Return the four-bit payload interval for an operational rate key.
 *
 * The 8-128-kbit/s values are source-exact nibble durations. The 500/1000
 * values are four-bit accounting intervals derived from the owner-confirmed
 * bit rates; the recovered modem files do not establish their physical DQPSK
 * symbol framing.
 */
inline double
CsrSymbolDurationSeconds (int rateKbpsKey)
{
  NS_ABORT_MSG_IF (
    rateKbpsKey < 0 ||
    static_cast<uint64_t> (rateKbpsKey) >
      std::numeric_limits<CsrRateKey>::max () ||
    !CsrIsOperationalRateKey (
      static_cast<CsrRateKey> (rateKbpsKey)),
    "CSR payload-rate key has no defined interval");
  switch (rateKbpsKey)
    {
    case 8:
      return 0.00051;
    case 16:
      return 0.000254;
    case 32:
      return 0.000126;
    case 64:
      return 0.000062;
    case 128:
      return 0.000030;
    case 500:
      return 4.0 / 500000.0;
    case 1000:
      return 4.0 / 1000000.0;
    default:
      NS_ABORT_MSG ("unreachable CSR payload-rate key");
      return 0.0;
    }
}

/** Convert the legacy rate key to its exact payload bit rate. */
inline double
CsrRateKeyToBps (int rateKbpsKey)
{
  return 4.0 / CsrSymbolDurationSeconds (rateKbpsKey);
}

/** Return true for owner-confirmed high-rate keys using the DQPSK curve. */
inline bool
CsrIsDqpskRate (int rateKbpsKey)
{
  return rateKbpsKey == 500 || rateKbpsKey == 1000;
}

/** Source-backed received-power, noise, SNR, and error front end. */
class CsrPhyModel
{
public:
  static constexpr double SPEED_OF_LIGHT_METERS_PER_SECOND = 3.0e8;
  static constexpr double PI = 3.14159265358979323846;

  CsrPhyProfile profile; ///< Configurable local radio attributes.
  CsrPerModelFn perModel; ///< Optional compatibility BER-model hook.

  /** Replace every configurable radio attribute. */
  void SetProfile (const CsrPhyProfile &newProfile)
  {
    ValidateProfile (newProfile);
    profile = newProfile;
  }

  /** Set the temporary BER/PER lookup hook. */
  void SetPerModel (CsrPerModelFn function)
  {
    perModel = std::move (function);
  }

  /** Set the BER lookup hook used by EstimatePer. */
  void SetBerModel (CsrPerModelFn function)
  {
    perModel = std::move (function);
  }

  /** Return whether a compatibility BER hook overrides the OPNET tables. */
  bool UsesCustomErrorModel () const
  {
    return static_cast<bool> (perModel);
  }

  /** Install the terrain/visibility delegate used by DELEGATE closure. */
  void SetClosureModel (CsrClosureModelFn function)
  {
    m_closureModel = std::move (function);
  }

  /**
   * Execute br_closure before channel matching and received-power stages.
   *
   * The external OPNET spherical-Earth helper was not supplied.  The built-in
   * mode therefore uses the standard geometric horizon, while DELEGATE lets a
   * calibrated terrain or captured OPNET closure model replace it.
   */
  bool HasClosure (double distanceMeters,
                   const CsrTxRadioProfile &transmitter) const
  {
    NS_ABORT_MSG_IF (!std::isfinite (distanceMeters) ||
                     distanceMeters < 0.0,
                     "CSR PHY closure distance must be finite and nonnegative");
    if (profile.closureMode == CsrClosureMode::NEVER_OCCLUDED)
      {
        return true;
      }
    if (profile.closureMode == CsrClosureMode::DELEGATE && m_closureModel)
      {
        return m_closureModel (distanceMeters,
                               transmitter.antennaHeightMeters,
                               profile.rxHeightMeters);
      }

    double txHeight = std::max (0.0, transmitter.antennaHeightMeters);
    double rxHeight = std::max (0.0, profile.rxHeightMeters);
    double radius = profile.earthRadiusMeters;
    double txHorizon = std::sqrt (txHeight * (2.0 * radius + txHeight));
    double rxHorizon = std::sqrt (rxHeight * (2.0 * radius + rxHeight));
    return distanceMeters <= txHorizon + rxHorizon;
  }

  /** Configure a symmetric distance for one node pair. */
  void SetLinkDistanceMeters (CsrNodeId first,
                              CsrNodeId second,
                              double meters)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (first) ||
                     !CsrIsValidNodeId (second),
                     "CSR PHY endpoint exceeds 24 bits");
    NS_ABORT_MSG_IF (!std::isfinite (meters) || meters < 0.0,
                     "CSR PHY link distance must be finite and nonnegative");
    m_linkDistanceMeters[{first, second}] = meters;
    m_linkDistanceMeters[{second, first}] = meters;
  }

  /** Configure the fallback distance used for an unspecified node pair. */
  void SetDefaultDistanceMeters (double meters)
  {
    NS_ABORT_MSG_IF (!std::isfinite (meters) || meters < 0.0,
                     "CSR PHY default distance must be finite and nonnegative");
    m_defaultDistanceMeters = meters;
  }

  /** Return the configured and scaled distance for a node pair. */
  double GetDistanceMeters (CsrNodeId txId, CsrNodeId rxId) const
  {
    auto iterator = m_linkDistanceMeters.find ({txId, rxId});
    double distance = iterator == m_linkDistanceMeters.end ()
      ? m_defaultDistanceMeters
      : iterator->second;
    return distance * profile.distanceScale;
  }

  /** Return transmitter settings to carry with an emitted signal. */
  CsrTxRadioProfile GetTxRadioProfile () const
  {
    return {
      profile.txBaseFrequencyHz,
      profile.txBwHz,
      profile.txAntennaGainDb,
      profile.txHeightMeters
    };
  }

  /** Convert dBm to watts. */
  static double DbmToWatts (double dbm)
  {
    return std::pow (10.0, (dbm - 30.0) / 10.0);
  }

  /** Convert watts to dBm, returning negative infinity for zero. */
  static double WattsToDbm (double watts)
  {
    return watts > 0.0
      ? 10.0 * std::log10 (watts) + 30.0
      : -std::numeric_limits<double>::infinity ();
  }

  /** Calculate the overlap between transmitter and receiver passbands. */
  double ComputeBandOverlapHz (const CsrTxRadioProfile &transmitter) const
  {
    double bandMinimum = std::max (profile.rxBaseFrequencyHz,
                                   transmitter.baseFrequencyHz);
    double bandMaximum = std::min (
      profile.rxBaseFrequencyHz + profile.rxBwHz,
      transmitter.baseFrequencyHz + transmitter.bandwidthHz);
    return std::max (0.0, bandMaximum - bandMinimum);
  }

  /**
   * Execute the exact non-TMM br_power equations and baseline br_snr.
   *
   * The supplied DES configurations all disable TMM, so the minimum linear
   * gain among free-space, flat-earth, and ad-hoc WLAN is authoritative.
   */
  CsrPhyFrontEndResult ComputeFrontEnd (
    double txPowerDbm,
    double distanceMeters,
    const CsrTxRadioProfile &transmitter) const
  {
    ValidateProfile (profile);
    NS_ABORT_MSG_IF (!std::isfinite (distanceMeters) ||
                     distanceMeters < 0.0,
                     "CSR PHY propagation distance must be finite and nonnegative");
    NS_ABORT_MSG_IF (!std::isfinite (txPowerDbm),
                     "CSR PHY transmit power must be finite");
    NS_ABORT_MSG_IF (transmitter.baseFrequencyHz <= 0.0 ||
                     transmitter.bandwidthHz <= 0.0 ||
                     transmitter.antennaHeightMeters < 0.0,
                     "CSR PHY transmitter profile is invalid");

    CsrPhyFrontEndResult result;
    result.distanceMeters = distanceMeters;
    result.bandOverlapHz = ComputeBandOverlapHz (transmitter);
    result.channelMatched = result.bandOverlapHz > 0.0;
    result.backgroundNoiseWatts = DbmToWatts (profile.noiseFloorDbm);
    if (profile.scaleNoiseWithBandwidth)
      {
        result.backgroundNoiseWatts *=
          profile.rxBwHz / profile.noiseReferenceBwHz;
      }
    result.noisePowerWatts = result.backgroundNoiseWatts;
    result.noisePowerDbm = WattsToDbm (result.noisePowerWatts);

    ComputePathGain (distanceMeters, transmitter, result);
    result.inBandTxPowerWatts = DbmToWatts (txPowerDbm) *
      result.bandOverlapHz / transmitter.bandwidthHz;

    if (result.channelMatched && result.pathGainLinear > 0.0)
      {
        double txGain = std::pow (10.0,
                                  transmitter.antennaGainDb / 10.0);
        double rxGain = std::pow (10.0,
                                  profile.rxAntennaGainDb / 10.0);
        result.receivedPowerWatts = result.inBandTxPowerWatts *
          txGain * result.pathGainLinear * rxGain;
        result.receivedPowerDbm = WattsToDbm (
          result.receivedPowerWatts);
        result.snrDb = ComputeSnrDb (result.receivedPowerWatts,
                                     result.noisePowerWatts);
      }
    return result;
  }

  /** Compute the source-backed baseline SNR for link control. */
  double PredictSnrDb (CsrNodeId txId,
                       CsrNodeId rxId,
                       double txPowerDbm) const
  {
    return ComputeFrontEnd (txPowerDbm,
                            GetDistanceMeters (txId, rxId),
                            GetTxRadioProfile ()).snrDb;
  }

  /** Return the selected propagation loss without antenna or band gains. */
  double ComputePathlossDb (double distanceMeters) const
  {
    CsrPhyFrontEndResult result;
    ComputePathGain (distanceMeters, GetTxRadioProfile (), result);
    return result.pathlossDb;
  }

  /** Compute SNR from linear received signal and total noise powers. */
  static double ComputeSnrDb (double receivedPowerWatts,
                              double totalNoiseWatts)
  {
    return receivedPowerWatts > 0.0 && totalNoiseWatts > 0.0
      ? 10.0 * std::log10 (receivedPowerWatts / totalNoiseWatts)
      : -std::numeric_limits<double>::infinity ();
  }

  /** Select source-backed header and payload BER for one interval. */
  CsrBerValues CalculateBer (
    const CsrPhyFrontEndResult &frontEnd,
    int rateKbps,
    const CsrBerInterval &interval,
    uint32_t packetBits = 0) const
  {
    CsrBerValues result;
    result.snrDb = ComputeSnrDb (frontEnd.receivedPowerWatts,
                                 interval.noisePowerWatts);
    double headerRate = CsrRateKeyToBps (8);
    double payloadRate = CsrRateKeyToBps (rateKbps);
    // br_ber establishes this processing-gain equation for spread rates. The
    // extended DQPSK path deliberately reuses it as an integration rule.
    result.headerEffectiveSnrDb = result.snrDb + 10.0 * std::log10 (
      profile.rxBwHz / (2.0 * headerRate));
    result.payloadEffectiveSnrDb = result.snrDb + 10.0 * std::log10 (
      profile.rxBwHz / (2.0 * payloadRate));

    if (perModel)
      {
        result.headerBer = std::clamp (
          perModel (8, result.headerEffectiveSnrDb, 48),
          0.0,
          1.0);
        result.payloadBer = std::clamp (
          perModel (rateKbps,
                    result.payloadEffectiveSnrDb,
                    packetBits),
          0.0,
          1.0);
        return result;
      }

    if (CsrIsDqpskRate (rateKbps) &&
        interval.sameRateInterference)
      {
        // No DQPSK-specific JSR/offset curves were supplied.  Treat the
        // recorded same-rate jammer as payload noise while leaving the S0
        // header on its source collision-table path.
        double jammerWatts = frontEnd.receivedPowerWatts * std::pow (
          10.0,
          interval.jsrDb / 10.0);
        double payloadSnrDb = ComputeSnrDb (
          frontEnd.receivedPowerWatts,
          interval.noisePowerWatts + jammerWatts);
        result.payloadEffectiveSnrDb = payloadSnrDb + 10.0 * std::log10 (
          profile.rxBwHz / (2.0 * payloadRate));
      }

    if (interval.collisionCount == 0)
      {
        result.headerBer = CsrOpnetBerTables::GetStandardBer (
          result.headerEffectiveSnrDb);
        result.payloadBer = CsrIsDqpskRate (rateKbps)
          ? CsrOpnetBerTables::GetDqpskBer (
              result.payloadEffectiveSnrDb)
          : CsrOpnetBerTables::GetStandardBer (
              result.payloadEffectiveSnrDb);
      }
    else
      {
        result.headerBer = CsrOpnetBerTables::GetCollisionBer (
          8,
          interval.jsrDb,
          interval.timeOffsetSeconds,
          result.headerEffectiveSnrDb);
        if (CsrIsDqpskRate (rateKbps))
          {
            result.payloadBer = CsrOpnetBerTables::GetDqpskBer (
              result.payloadEffectiveSnrDb);
          }
        else
          {
            result.payloadBer = interval.sameRateInterference
              ? CsrOpnetBerTables::GetCollisionBer (
                  rateKbps,
                  interval.jsrDb,
                  interval.timeOffsetSeconds,
                  result.payloadEffectiveSnrDb)
              : CsrOpnetBerTables::GetStandardBer (
                  result.payloadEffectiveSnrDb);
          }
      }
    return result;
  }

  /** Invert the source binomial CDF using a caller-provided uniform value. */
  static uint32_t SampleSourceBinomial (uint32_t bits,
                                        double probability,
                                        double uniform)
  {
    if (bits == 0 || probability <= 0.0)
      {
        return 0;
      }
    if (probability >= 1.0)
      {
        return bits;
      }

    bool inverted = probability > 0.5;
    double p = inverted ? 1.0 - probability : probability;
    double target = std::clamp (uniform,
                                0.0,
                                std::nextafter (1.0, 0.0));
    double accumulated = 0.0;
    uint32_t errors = 0;
    for (; errors <= bits; ++errors)
      {
        double logProbability =
          std::lgamma (static_cast<double> (bits) + 1.0) -
          std::lgamma (static_cast<double> (errors) + 1.0) -
          std::lgamma (static_cast<double> (bits - errors) + 1.0) +
          static_cast<double> (errors) * std::log (p) +
          static_cast<double> (bits - errors) * std::log (1.0 - p);
        accumulated += std::exp (logProbability);
        if (accumulated >= target)
          {
            break;
          }
      }
    errors = std::min (errors, bits);
    return inverted ? bits - errors : errors;
  }

  /** Sample one source-style binomial outcome from the ns-3 stream. */
  static uint32_t SampleSourceBinomial (
    uint32_t bits,
    double probability,
    const Ptr<UniformRandomVariable> &rng)
  {
    if (bits == 0 || probability <= 0.0)
      {
        return 0;
      }
    if (probability >= 1.0)
      {
        return bits;
      }
    double uniform = rng == nullptr ? 1.0 : rng->GetValue (0.0, 1.0);
    return SampleSourceBinomial (bits, probability, uniform);
  }

  /** Allocate errors interval by interval using br_error_all_stats rules. */
  CsrErrorAllocation AllocateErrors (
    const CsrPhyFrontEndResult &frontEnd,
    double signalStartSec,
    uint32_t preambleBits,
    uint32_t packetBits,
    int rateKbps,
    const std::vector<CsrBerInterval> &intervals,
    const Ptr<UniformRandomVariable> &rng) const
  {
    CsrErrorAllocation result;
    double headerRate = CsrRateKeyToBps (8);
    double payloadRate = CsrRateKeyToBps (rateKbps);
    double headerStartSec = signalStartSec +
      static_cast<double> (preambleBits) / headerRate;
    double payloadStartSec = signalStartSec +
      static_cast<double> (preambleBits + 48) / headerRate;
    uint32_t payloadRegionBits = packetBits > preambleBits + 48
      ? packetBits - preambleBits - 48
      : 0;
    double packetEndSec = payloadStartSec +
      static_cast<double> (payloadRegionBits) / payloadRate;
    double noErrorProbability = 1.0;

    std::vector<CsrBerInterval> ordered = intervals;
    std::stable_sort (
      ordered.begin (),
      ordered.end (),
      [] (const CsrBerInterval &left, const CsrBerInterval &right) {
        return left.startSec < right.startSec;
      });

    for (const CsrBerInterval &interval : ordered)
      {
        double intervalEndSec = std::min (interval.endSec, packetEndSec);
        if (intervalEndSec < interval.startSec ||
            interval.startSec >= packetEndSec)
          {
            continue;
          }
        CsrBerValues ber = CalculateBer (frontEnd,
                                        rateKbps,
                                        interval,
                                        packetBits);
        result.headerBer = ber.headerBer;
        result.payloadBer = ber.payloadBer;
        result.minimumSnrDb = std::min (result.minimumSnrDb, ber.snrDb);
        result.peakNoisePowerWatts = std::max (
          result.peakNoisePowerWatts,
          interval.noisePowerWatts);

        if (intervalEndSec < headerStartSec)
          {
            result.actualBer = ber.headerBer;
            continue;
          }

        double headerBeginSec = std::max (interval.startSec,
                                          headerStartSec);
        double headerEndSec = std::min (intervalEndSec,
                                        payloadStartSec);
        uint32_t headerBits = headerEndSec > headerBeginSec
          ? static_cast<uint32_t> (
              (headerEndSec - headerBeginSec) * headerRate)
          : 0;
        double payloadBeginSec = std::max (interval.startSec,
                                           payloadStartSec);
        uint32_t payloadBits = intervalEndSec > payloadBeginSec
          ? static_cast<uint32_t> (
              (intervalEndSec - payloadBeginSec) * payloadRate)
          : 0;

        uint32_t headerErrors = SampleSourceBinomial (
          headerBits,
          ber.headerBer,
          rng);
        uint32_t payloadErrors = SampleSourceBinomial (
          payloadBits,
          ber.payloadBer,
          rng);
        result.headerBits += headerBits;
        result.payloadBits += payloadBits;
        result.headerErrors += headerErrors;
        result.payloadErrors += payloadErrors;
        result.totalErrors += headerErrors + payloadErrors;
        noErrorProbability *= std::pow (1.0 - ber.headerBer,
                                        static_cast<double> (headerBits));
        noErrorProbability *= std::pow (1.0 - ber.payloadBer,
                                        static_cast<double> (payloadBits));

        uint32_t testedBits = headerBits + payloadBits;
        if (testedBits > 0)
          {
            result.actualBer = static_cast<double> (
              headerErrors + payloadErrors) / testedBits;
          }
        else
          {
            result.actualBer = intervalEndSec < payloadStartSec
              ? ber.headerBer
              : ber.payloadBer;
          }
      }

    if (!std::isfinite (result.minimumSnrDb))
      {
        result.minimumSnrDb = -std::numeric_limits<double>::infinity ();
      }
    result.packetErrorProbability = std::clamp (
      1.0 - noErrorProbability,
      0.0,
      1.0);
    return result;
  }

  /** Apply the exact inclusive, full-packet br_ecc acceptance rule. */
  CsrEccResult EvaluateEcc (uint32_t packetBits,
                            uint32_t preambleBits,
                            uint32_t totalErrors,
                            bool priorAccepted = true,
                            bool signalLocked = false,
                            bool nodeFailed = false) const
  {
    CsrEccResult result;
    result.protectedBits = packetBits > preambleBits
      ? packetBits - preambleBits
      : 0;
    result.correctableBits = static_cast<uint32_t> (
      profile.eccThreshold * result.protectedBits);

    if (!priorAccepted && !signalLocked)
      {
        return result;
      }
    if (nodeFailed || signalLocked)
      {
        return result;
      }

    result.accepted = result.protectedBits == 0 ||
      totalErrors <= result.correctableBits;
    result.eccDropped = !result.accepted;
    return result;
  }

  /** Apply final ECC to an already allocated tracked-signal error record. */
  CsrRxDecision EvaluateAllocatedRx (
    const CsrPhyFrontEndResult &frontEnd,
    uint32_t preambleBits,
    uint32_t packetBits,
    const CsrErrorAllocation &errors,
    const std::vector<CsrBerInterval> &intervals,
    bool priorAccepted,
    bool signalLocked,
    bool nodeFailed) const
  {
    CsrEccResult ecc = EvaluateEcc (packetBits,
                                    preambleBits,
                                    errors.totalErrors,
                                    priorAccepted &&
                                      frontEnd.channelMatched,
                                    signalLocked,
                                    nodeFailed);

    CsrRxDecision result;
    result.success = ecc.accepted;
    result.channelMatched = frontEnd.channelMatched;
    result.pathModel = frontEnd.pathModel;
    result.pathlossDb = frontEnd.pathlossDb;
    result.receivedPowerDbm = frontEnd.receivedPowerDbm;
    result.noisePowerDbm = WattsToDbm (errors.peakNoisePowerWatts);
    result.snrDb = errors.minimumSnrDb;
    result.per = errors.packetErrorProbability;
    result.headerBer = errors.headerBer;
    result.payloadBer = errors.payloadBer;
    result.actualBer = errors.actualBer;
    result.headerErrors = errors.headerErrors;
    result.payloadErrors = errors.payloadErrors;
    result.totalErrors = errors.totalErrors;
    result.protectedBits = ecc.protectedBits;
    result.correctableBits = ecc.correctableBits;
    result.eccDropped = ecc.eccDropped;
    for (const CsrBerInterval &interval : intervals)
      {
        if (interval.sameRateInterference)
          {
            result.sameRateInterference = true;
            result.jsrDb = interval.jsrDb;
            result.timeOffsetSeconds = interval.timeOffsetSeconds;
          }
      }
    return result;
  }

  /** Run interval error allocation and final ECC for a tracked signal. */
  CsrRxDecision EvaluateRxIntervals (
    const CsrPhyFrontEndResult &frontEnd,
    double signalStartSec,
    uint32_t preambleBits,
    uint32_t packetBits,
    int rateKbps,
    const std::vector<CsrBerInterval> &intervals,
    bool priorAccepted,
    bool signalLocked,
    bool nodeFailed,
    const Ptr<UniformRandomVariable> &rng) const
  {
    CsrErrorAllocation errors = AllocateErrors (frontEnd,
                                                signalStartSec,
                                                preambleBits,
                                                packetBits,
                                                rateKbps,
                                                intervals,
                                                rng);
    return EvaluateAllocatedRx (frontEnd,
                                preambleBits,
                                packetBits,
                                errors,
                                intervals,
                                priorAccepted,
                                signalLocked,
                                nodeFailed);
  }

  /** Return the direct model output before processing gain. */
  double EstimateBer (int rateKbps,
                      double snrDb,
                      uint32_t packetBits) const
  {
    double ber = perModel
      ? perModel (rateKbps, snrDb, packetBits)
      : CsrIsDqpskRate (rateKbps)
          ? CsrOpnetBerTables::GetDqpskBer (snrDb)
          : CsrOpnetBerTables::GetStandardBer (snrDb);
    return std::clamp (ber, 0.0, 1.0);
  }

  /** Apply OPNET processing gain and convert BER to packet error rate. */
  double EstimatePer (int rateKbps,
                      double snrDb,
                      uint32_t packetBits) const
  {
    double bitRate = CsrRateKeyToBps (rateKbps);
    double processingGainDb = 10.0 * std::log10 (
      profile.rxBwHz / (2.0 * bitRate));
    double ber = EstimateBer (rateKbps,
                              snrDb + processingGainDb,
                              packetBits);
    double per = 1.0 - std::pow (1.0 - ber,
                                 static_cast<double> (packetBits));
    return std::clamp (per, 0.0, 1.0);
  }

  /** Make an error decision from a previously calculated PHY front end. */
  CsrRxDecision EvaluateRx (
    const CsrPhyFrontEndResult &frontEnd,
    int rateKbps,
    uint32_t packetBits,
    double observedNoiseWatts,
    bool sameRateInterference,
    double jsrDb,
    double timeOffsetSeconds,
    const Ptr<UniformRandomVariable> &rng) const
  {
    double snrDb = ComputeSnrDb (frontEnd.receivedPowerWatts,
                                 observedNoiseWatts);
    double per = frontEnd.channelMatched && std::isfinite (snrDb)
      ? EstimatePer (rateKbps, snrDb, packetBits)
      : 1.0;
    double outcome = rng == nullptr ? 1.0 : rng->GetValue (0.0, 1.0);

    CsrRxDecision result;
    result.success = frontEnd.channelMatched && outcome >= per;
    result.channelMatched = frontEnd.channelMatched;
    result.sameRateInterference = sameRateInterference;
    result.pathModel = frontEnd.pathModel;
    result.pathlossDb = frontEnd.pathlossDb;
    result.receivedPowerDbm = frontEnd.receivedPowerDbm;
    result.noisePowerDbm = WattsToDbm (observedNoiseWatts);
    result.snrDb = snrDb;
    result.per = per;
    result.jsrDb = jsrDb;
    result.timeOffsetSeconds = timeOffsetSeconds;
    return result;
  }

  /** Make a baseline receive decision for one configured node pair. */
  CsrRxDecision EvaluateRx (
    CsrNodeId txId,
    CsrNodeId rxId,
    int rateKbps,
    double txPowerDbm,
    uint32_t packetBits,
    const Ptr<UniformRandomVariable> &rng) const
  {
    double distanceMeters = GetDistanceMeters (txId, rxId);
    CsrTxRadioProfile transmitter = GetTxRadioProfile ();
    if (!HasClosure (distanceMeters, transmitter))
      {
        CsrRxDecision result;
        result.closure = false;
        return result;
      }
    CsrPhyFrontEndResult frontEnd = ComputeFrontEnd (
      txPowerDbm,
      distanceMeters,
      transmitter);
    return EvaluateRx (frontEnd,
                       rateKbps,
                       packetBits,
                       frontEnd.backgroundNoiseWatts,
                       false,
                       -std::numeric_limits<double>::infinity (),
                       0.0,
                       rng);
  }

private:
  static void ValidateProfile (const CsrPhyProfile &candidate)
  {
    NS_ABORT_MSG_IF (!std::isfinite (candidate.noiseFloorDbm) ||
                     candidate.txBaseFrequencyHz <= 0.0 ||
                     candidate.rxBaseFrequencyHz <= 0.0 ||
                     candidate.noiseReferenceBwHz <= 0.0 ||
                     candidate.txBwHz <= 0.0 ||
                     candidate.rxBwHz <= 0.0 ||
                     candidate.txHeightMeters < 0.0 ||
                     candidate.rxHeightMeters < 0.0 ||
                     !std::isfinite (candidate.eccThreshold) ||
                     candidate.eccThreshold < 0.0 ||
                     candidate.eccThreshold > 1.0 ||
                     !std::isfinite (candidate.earthRadiusMeters) ||
                     candidate.earthRadiusMeters <= 0.0 ||
                     candidate.distanceScale <= 0.0,
                     "CSR PHY profile contains an invalid radio attribute");
  }

  void ComputePathGain (double distanceMeters,
                        const CsrTxRadioProfile &transmitter,
                        CsrPhyFrontEndResult &result) const
  {
    if (distanceMeters <= 0.0)
      {
        result.pathModel = CsrPathGainModel::UNIT_GAIN;
        result.pathGainLinear = 1.0;
        result.pathlossDb = 0.0;
        return;
      }

    if (profile.propagationModel == CsrPropagationModel::LOG_DISTANCE)
      {
        double boundedDistance = std::max (distanceMeters, 1.0e-6);
        result.pathModel = CsrPathGainModel::LOG_DISTANCE;
        result.pathlossDb = profile.refLossDb +
          10.0 * profile.pathlossExp * std::log10 (boundedDistance);
        result.pathGainLinear = std::pow (10.0,
                                          -result.pathlossDb / 10.0);
        return;
      }

    double centerFrequencyHz = transmitter.baseFrequencyHz +
      transmitter.bandwidthHz / 2.0;
    double wavelengthMeters = SPEED_OF_LIGHT_METERS_PER_SECOND /
      centerFrequencyHz;
    double distanceSquared = distanceMeters * distanceMeters;
    double distanceFourth = distanceSquared * distanceSquared;
    double heightProductSquared =
      transmitter.antennaHeightMeters *
      transmitter.antennaHeightMeters *
      profile.rxHeightMeters *
      profile.rxHeightMeters;

    double freeSpaceGain = wavelengthMeters * wavelengthMeters /
      (16.0 * PI * PI * distanceSquared);
    double flatEarthGain = heightProductSquared / distanceFourth;
    double adHocWlanGain = heightProductSquared * 1.0e18 /
      (distanceFourth * centerFrequencyHz * centerFrequencyHz);

    result.pathGainLinear = 1.0;
    result.pathModel = CsrPathGainModel::UNIT_GAIN;
    if (result.pathGainLinear > freeSpaceGain)
      {
        result.pathGainLinear = freeSpaceGain;
        result.pathModel = CsrPathGainModel::FREE_SPACE;
      }
    if (result.pathGainLinear > flatEarthGain)
      {
        result.pathGainLinear = flatEarthGain;
        result.pathModel = CsrPathGainModel::FLAT_EARTH;
      }
    if (result.pathGainLinear > adHocWlanGain)
      {
        result.pathGainLinear = adHocWlanGain;
        result.pathModel = CsrPathGainModel::AD_HOC_WLAN;
      }
    result.pathlossDb = result.pathGainLinear > 0.0
      ? -10.0 * std::log10 (result.pathGainLinear)
      : std::numeric_limits<double>::infinity ();
  }

  std::map<std::pair<CsrNodeId, CsrNodeId>, double>
    m_linkDistanceMeters;
  double m_defaultDistanceMeters {1.0};
  CsrClosureModelFn m_closureModel;
};

/** One table of BER samples for a payload rate. */
struct BerCurve
{
  int rateKbps;
  std::vector<double> snrDb;
  std::vector<double> ber;
};

/** Table-driven BER lookup scaffold used by existing scenarios. */
class CsrBerTableModel
{
public:
  void AddCurve (BerCurve curve)
  {
    m_curves[curve.rateKbps] = std::move (curve);
  }

  double GetBer (int rateKbps, double effectiveSnrDb) const
  {
    auto iterator = m_curves.find (rateKbps);
    if (iterator == m_curves.end ())
      {
        return Clamp01 (1.0 /
          (1.0 + std::exp (0.9 * (effectiveSnrDb - 2.0))));
      }
    return Interpolate (iterator->second.snrDb,
                        iterator->second.ber,
                        effectiveSnrDb);
  }

  double GetBerViaHook (int rateKbps,
                        double effectiveSnrDb,
                        uint32_t) const
  {
    return GetBer (rateKbps, effectiveSnrDb);
  }

private:
  static double Clamp01 (double value)
  {
    return std::clamp (value, 0.0, 1.0);
  }

  static double Interpolate (const std::vector<double> &x,
                             const std::vector<double> &y,
                             double query)
  {
    if (x.empty () || y.empty () || x.size () != y.size ())
      {
        return 0.0;
      }
    if (query <= x.front ())
      {
        return y.front ();
      }
    if (query >= x.back ())
      {
        return y.back ();
      }
    for (std::size_t index = 1; index < x.size (); ++index)
      {
        if (query <= x[index])
          {
            double fraction = (query - x[index - 1]) /
              (x[index] - x[index - 1]);
            return y[index - 1] +
              fraction * (y[index] - y[index - 1]);
          }
      }
    return y.back ();
  }

  std::map<int, BerCurve> m_curves;
};

/** One table of PER samples for a payload rate. */
struct PerCurve
{
  int rateKbps;
  std::vector<double> snrDb;
  std::vector<double> per;
};

/** Table-driven PER lookup scaffold used by the legacy demo. */
class CsrPerTableModel
{
public:
  void AddCurve (PerCurve curve)
  {
    m_curves[curve.rateKbps] = std::move (curve);
  }

  double GetPer (int rateKbps,
                 double snrDb,
                 uint32_t) const
  {
    auto iterator = m_curves.find (rateKbps);
    if (iterator == m_curves.end ())
      {
        return std::clamp (
          1.0 / (1.0 + std::exp (snrDb)),
          0.0,
          1.0);
      }
    return Interpolate (iterator->second.snrDb,
                        iterator->second.per,
                        snrDb);
  }

private:
  static double Interpolate (const std::vector<double> &x,
                             const std::vector<double> &y,
                             double query)
  {
    if (x.empty () || y.empty () || x.size () != y.size ())
      {
        return 0.0;
      }
    if (query <= x.front ())
      {
        return y.front ();
      }
    if (query >= x.back ())
      {
        return y.back ();
      }
    for (std::size_t index = 1; index < x.size (); ++index)
      {
        if (query <= x[index])
          {
            double fraction = (query - x[index - 1]) /
              (x[index] - x[index - 1]);
            return y[index - 1] +
              fraction * (y[index] - y[index - 1]);
          }
      }
    return y.back ();
  }

  std::map<int, PerCurve> m_curves;
};
