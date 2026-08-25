#pragma once

#include "csr-common.h"

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
  CsrPropagationModel propagationModel {
    CsrPropagationModel::OPNET_THREE_PATH
  }; ///< Active propagation equation.

  // Compatibility parameters for pre-parity log-distance scenarios.
  double refLossDb {60.0}; ///< Legacy one-meter log-distance loss.
  double pathlossExp {2.0}; ///< Legacy log-distance exponent.
  double distanceScale {1.0}; ///< Scale applied to configured distances.
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
  bool channelMatched {false}; ///< Whether the channels overlap.
  bool sameRateInterference {false}; ///< Whether JSR lookup is required.
  CsrPathGainModel pathModel {CsrPathGainModel::UNIT_GAIN}; ///< Selected path model.
  double pathlossDb {std::numeric_limits<double>::infinity ()}; ///< Propagation loss.
  double receivedPowerDbm {-std::numeric_limits<double>::infinity ()}; ///< Signal power.
  double noisePowerDbm {-std::numeric_limits<double>::infinity ()}; ///< Peak noise.
  double snrDb {-std::numeric_limits<double>::infinity ()}; ///< Minimum observed SNR.
  double per {1.0}; ///< Packet error probability.
  double jsrDb {-std::numeric_limits<double>::infinity ()}; ///< Strongest same-rate JSR.
  double timeOffsetSeconds {0.0}; ///< Same-rate jammer offset.
};

/** BER model hook indexed by payload rate, effective SNR, and packet bits. */
using CsrPerModelFn = std::function<double(int, double, uint32_t)>;

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

/** Convert the legacy rate key to its exact payload bit rate. */
inline double
CsrRateKeyToBps (int rateKbpsKey)
{
  switch (rateKbpsKey)
    {
    case 8:
      return 7812.5;
    case 16:
      return 15625.0;
    case 32:
      return 31250.0;
    case 64:
      return 62500.0;
    case 128:
      return 125000.0;
    default:
      return rateKbpsKey * 1000.0;
    }
}

/** Source-backed received-power, noise, SNR, and error front end. */
class CsrPhyModel
{
public:
  static constexpr double SPEED_OF_LIGHT_METERS_PER_SECOND = 3.0e8;
  static constexpr double PI = 3.14159265358979323846;

  CsrPhyProfile profile; ///< Configurable local radio attributes.
  CsrPerModelFn perModel = CsrPerModelPlaceholder; ///< Current error-model hook.

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

  /** Return the direct model output before processing gain. */
  double EstimateBer (int rateKbps,
                      double snrDb,
                      uint32_t packetBits) const
  {
    double ber = perModel
      ? perModel (rateKbps, snrDb, packetBits)
      : 0.0;
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
    CsrPhyFrontEndResult frontEnd = ComputeFrontEnd (
      txPowerDbm,
      GetDistanceMeters (txId, rxId),
      GetTxRadioProfile ());
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
