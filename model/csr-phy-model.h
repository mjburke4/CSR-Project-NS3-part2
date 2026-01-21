#pragma once
#include "csr-common.h"

class CsrPhyModel
{
public:
  CsrPhyProfile profile;
  CsrPerModelFn perModel = CsrPerModelPlaceholder;

  void SetProfile (const CsrPhyProfile &p) { profile = p; }
  void SetPerModel (CsrPerModelFn fn) { perModel = std::move(fn); }
  void SetBerModel (CsrPerModelFn fn) { perModel = std::move(fn); }

  void SetLinkDistanceMeters(uint16_t a, uint16_t b, double meters)
  {
    m_linkDist[{a,b}] = meters;
    m_linkDist[{b,a}] = meters;
  }

  void SetDefaultDistanceMeters(double meters) { m_defaultDist = meters; }

  double GetDistanceMeters (uint16_t txId, uint16_t rxId) const
  {
    auto it = m_linkDist.find({txId, rxId});
    if (it != m_linkDist.end())
      return it->second * profile.distanceScale;
    return m_defaultDist * profile.distanceScale;
  }

  double PredictSnrDb (uint16_t txId, uint16_t rxId, double txPowerDbm) const
  {
    double dist = GetDistanceMeters (txId, rxId);
    double plDb = ComputePathlossDb (dist);
    double rxDbm = txPowerDbm - plDb;
    return rxDbm - profile.noiseFloorDbm;
  }

  double EstimateBer (int rateKbps, double snrDb, uint32_t nBits) const
  {
    // For now, don't apply processing gain until we're sure about BW accounting.
    double effSnrDb = snrDb;

    // perModel is interpreted as BER(effSNR) now
    double ber = perModel ? perModel (rateKbps, effSnrDb, nBits) : 0.0;
    ber = std::min (1.0, std::max (0.0, ber));

    double per = 1.0 - std::pow (1.0 - ber, (double)nBits);
    return std::min (1.0, std::max (0.0, per));
  }

  double EstimatePer (int rateKbps, double snrDb, uint32_t nBits) const
  {
    //double rb = rateKbps * 1000.0;
    double rb = CsrRateKeyToBps (rateKbps);

    // OPNET-style processing gain: 10log10(rx_bw/(2*Rb))
    double procGainDb = 10.0 * std::log10(profile.rxBwHz / (2.0 * rb));
    double effSnrDb = snrDb + procGainDb;

    // perModel is currently your BER model hook
    double ber = perModel ? perModel(rateKbps, effSnrDb, nBits) : 0.0;
    ber = std::min(1.0, std::max(0.0, ber));

    double per = 1.0 - std::pow(1.0 - ber, (double)nBits);
    return std::min(1.0, std::max(0.0, per));
  }


  double ComputePathlossDb (double distMeters) const
  {
    if (distMeters < 1e-6) distMeters = 1e-6;
    return profile.refLossDb + 10.0 * profile.pathlossExp * std::log10 (distMeters);
  }

  double ComputeSnrDb (double pathlossDb) const
  {
    double rxPowerDbm = profile.txPowerDbm - pathlossDb;
    return rxPowerDbm - profile.noiseFloorDbm;
  }

  CsrRxDecision EvaluateRx (uint16_t txId,
                            uint16_t rxId,
                            int rateKbps,
                            uint32_t packetBits,
                            const Ptr<UniformRandomVariable>& rng) const
  {
    double dist = GetDistanceMeters (txId, rxId);
    double plDb = ComputePathlossDb (dist);
    double snrDb = ComputeSnrDb (plDb);

    // IMPORTANT: use the scaled PER (depends on packetBits)
    double per = EstimatePer (rateKbps, snrDb, packetBits);

    // Draw once
    double u = rng->GetValue (0.0, 1.0);
    bool success = (u >= per);

    CsrRxDecision out;
    out.success    = success;
    out.pathlossDb = plDb;
    out.snrDb      = snrDb;
    out.per        = per;
    return out;
  }

private:
  std::map<std::pair<uint16_t,uint16_t>, double> m_linkDist;
  double m_defaultDist { 1.0 };
};

// --- Table-driven PER model (stub scaffold) --------------------
struct PerCurve
{
  int rateKbps;
  std::vector<double> snrDb; // ascending
  std::vector<double> per;   // 0..1, same length
};

struct BerCurve
{
  int rateKbps;
  std::vector<double> snrDb;
  std::vector<double> ber;   // 0..1
};

class CsrBerTableModel
{
public:
  void AddCurve (BerCurve c) { m_curves[c.rateKbps] = std::move (c); }

  // Pure BER lookup: BER vs (effective) SNR
  double GetBer (int rateKbps, double effSnrDb) const
  {
    auto it = m_curves.find (rateKbps);
    if (it == m_curves.end ())
      {
        // fallback logistic BER (placeholder)
        double k = 0.9;
        double snr0 = 2.0;
        double ber = 1.0 / (1.0 + std::exp (k * (effSnrDb - snr0)));
        return Clamp01 (ber);
      }
    return Interp1 (it->second.snrDb, it->second.ber, effSnrDb);
  }

  // Adapter to match your existing CsrPerModelFn signature (3 args)
  double GetBerViaHook (int rateKbps, double effSnrDb, uint32_t /*nBits*/) const
  {
    return GetBer (rateKbps, effSnrDb);
  }

private:
  static double Clamp01 (double v)
  {
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
  }

  static double Interp1 (const std::vector<double>& x,
                        const std::vector<double>& y,
                        double xq)
  {
    if (x.empty () || y.empty () || x.size () != y.size ())
      return 0.0;

    if (xq <= x.front ()) return y.front ();
    if (xq >= x.back  ()) return y.back  ();

    for (size_t i = 1; i < x.size (); ++i)
      {
        if (xq <= x[i])
          {
            double t = (xq - x[i-1]) / (x[i] - x[i-1]);
            return y[i-1] + t * (y[i] - y[i-1]);
          }
      }
    return y.back ();
  }

  std::map<int, BerCurve> m_curves;
};

/*class CsrBerTableModel
{
public:
  void AddCurve (BerCurve c) { m_curves[c.rateKbps] = std::move(c); }

  double GetBer (int rateKbps, double effSnrDb) const
  {
    auto it = m_curves.find(rateKbps);
    if (it == m_curves.end())
      {
        // fallback logistic BER (placeholder)
        double k = 0.9;
        double snr0 = 2.0;
        double ber = 1.0 / (1.0 + std::exp(k * (effSnrDb - snr0)));
        return std::min(1.0, std::max(0.0, ber));
      }
    return Interp1(it->second.snrDb, it->second.ber, effSnrDb);
  }

private:
  // reuse your Interp1(...)
  std::map<int, BerCurve> m_curves;
};*/

/*double GetBer(int rateKbps, double snrDb) const
{
  auto it = m_curves.find(rateKbps);
  if (it == m_curves.end())
    {
      // fallback BER logistic
      double k = 1.0;
      double snr0 = 0.0;
      double ber = 1.0 / (1.0 + std::exp( (snrDb - snr0) * k ));
      return std::min(1.0, std::max(0.0, ber));
    }
  return Interp1(it->second.snrDb, it->second.ber, snrDb);
}*/

class CsrPerTableModel
{
public:
  void AddCurve (PerCurve c) { m_curves[c.rateKbps] = std::move (c); }

  double GetPer (int rateKbps, double snrDb, uint32_t /*nBits*/) const
  {
    auto it = m_curves.find (rateKbps);
    if (it == m_curves.end ())
      {
        // fallback logistic (same shape as your placeholder)
        double k = 1.0;
        double snr0 = 0.0;
        double p = 1.0 / (1.0 + std::exp(k * (snrDb - snr0)));
        return std::min(1.0, std::max(0.0, p));
      }
    return Interp1 (it->second.snrDb, it->second.per, snrDb);
  }

private:
  static double Interp1 (const std::vector<double>& x,
                         const std::vector<double>& y,
                         double xq)
  {
    if (x.empty () || y.empty () || x.size () != y.size ())
      {
        return 0.0;
      }

    if (xq <= x.front ()) return y.front ();
    if (xq >= x.back  ()) return y.back  ();

    for (size_t i = 1; i < x.size (); ++i)
      {
        if (xq <= x[i])
          {
            double t = (xq - x[i-1]) / (x[i] - x[i-1]);
            return y[i-1] + t * (y[i] - y[i-1]);
          }
      }
    return y.back ();
  }

  std::map<int, PerCurve> m_curves;
};

// ------------------------------------------------------------
// CsrNetDevice: wraps a MAC and models the channel
// ------------------------------------------------------------

