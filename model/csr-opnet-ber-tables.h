#pragma once

namespace ns3 {

/**
 * Exact BER lookup tables exported from the supplied OPNET modulation models.
 *
 * The standard curve is named csr in the legacy model.  Collision curves are
 * selected by payload rate, quantized jammer-to-signal ratio, and circular
 * half-chip timing offset.  Lookups clamp at the curve endpoints and use
 * arithmetic linear interpolation between adjacent samples, matching the
 * behavior expected from op_tbl_mod_ber().
 */
class CsrOpnetBerTables
{
public:
  /**
   * Look up the standard CSR bit-error rate.
   *
   * @param effectiveSnrDb Effective Eb/N0-style input in decibels.
   * @return Interpolated BER from the csr table.
   */
  static double GetStandardBer (double effectiveSnrDb);

  /**
   * Evaluate the recovered 500-kbit/s differential-BPSK BER law.
   *
   * DPSK_PB.m defines PB = exp(-Eb/No) / 2.  Unlike the 1000-kbit/s
   * DQPSK mode, no separate OPNET lookup-table file was supplied for this
   * mode, so the original analytical expression is evaluated directly.
   *
   * @param effectiveSnrDb Eb/N0 in decibels.
   * @return Differential-BPSK bit-error probability.
   */
  static double GetDpskBer (double effectiveSnrDb);

  /**
   * Look up the recovered DQPSK curve used by the 1000-kbit/s mode.
   *
   * The curve samples agree with the default equation in DQPSK_PB.m to the
   * table's decimal precision.  Its 1000-kbit/s mapping is an extension
   * because the supplied C receive pipeline does not reference dqpsk directly.
   *
   * @param effectiveSnrDb Effective Eb/N0-style input in decibels.
   * @return Interpolated BER from the dqpsk table.
   */
  static double GetDqpskBer (double effectiveSnrDb);

  /**
   * Look up a same-rate collision bit-error rate.
   *
   * This lookup is defined only for the spread-rate keys.  The JSR and timing
   * inputs are quantized exactly as in br_ber.ps.c before lookup.
   *
   * @param rateKbps Legacy payload-rate key (8, 16, 32, 64, or 128).
   * @param jsrDb Jammer-to-signal ratio in decibels.
   * @param timeOffsetSeconds Relative signal start-time offset.
   * @param effectiveSnrDb Effective Eb/N0-style input in decibels.
   * @return Interpolated BER from the selected collision table.
   */
  static double GetCollisionBer (int rateKbps,
                                 double jsrDb,
                                 double timeOffsetSeconds,
                                 double effectiveSnrDb);

  /**
   * Quantize JSR to one of the legacy table buckets.
   *
   * @param jsrDb Jammer-to-signal ratio in decibels.
   * @return One of -12, -9, -6, -3, or 0.
   */
  static int QuantizeJsrDb (double jsrDb);

  /**
   * Quantize a circular timing delta to a half-chip table offset.
   *
   * @param rateKbps Legacy payload-rate key.
   * @param timeOffsetSeconds Relative signal start-time offset.
   * @return Chip offset in 0.5-chip increments.
   */
  static double QuantizeChipOffset (int rateKbps,
                                    double timeOffsetSeconds);

  /**
   * Return the four-bit payload interval used by runtime rate accounting.
   *
   * The spread-rate intervals are source-defined. The DPSK/DQPSK accounting
   * intervals are derived from the owner-confirmed high-rate bit rates.
   *
   * @param rateKbps Legacy payload-rate key.
   * @return Four-bit payload interval in seconds.
   */
  static double SymbolDurationSeconds (int rateKbps);
};

} // namespace ns3
