#include "csr-opnet-ber-tables.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ns3 {

namespace {

constexpr double CSR_CHIP_SECONDS = 0.000002;
constexpr std::size_t CSR_RATE_COUNT = 5;
constexpr std::size_t CSR_JSR_BUCKET_COUNT = 5;

struct CsrRateTableLayout
{
  int rateKbps;
  double symbolDurationSeconds;
  std::uint16_t halfChipOffsetCount;
  std::uint32_t firstCurve;
};

constexpr std::array<CsrRateTableLayout, CSR_RATE_COUNT> CSR_RATE_LAYOUT {{
  {8, 0.00051, 510, 0},
  {16, 0.000254, 254, 2550},
  {32, 0.000126, 126, 3820},
  {64, 0.000062, 62, 4450},
  {128, 0.00003, 30, 4760}
}};

constexpr std::array<int, CSR_JSR_BUCKET_COUNT> CSR_JSR_BUCKETS {{
  -12,
  -9,
  -6,
  -3,
  0
}};

#include "csr-opnet-ber-tables-data.inc"

const CsrRateTableLayout &
GetRateLayout (int rateKbps)
{
  for (const auto &layout : CSR_RATE_LAYOUT)
    {
      if (layout.rateKbps == rateKbps)
        {
          return layout;
        }
    }

  return CSR_RATE_LAYOUT.front ();
}

std::size_t
GetJsrBucketIndex (int bucketDb)
{
  for (std::size_t index = 0; index < CSR_JSR_BUCKETS.size (); ++index)
    {
      if (CSR_JSR_BUCKETS[index] == bucketDb)
        {
          return index;
        }
    }

  return 0;
}

std::uint16_t
QuantizeHalfChipOffset (const CsrRateTableLayout &layout,
                        double timeOffsetSeconds)
{
  if (!std::isfinite (timeOffsetSeconds))
    {
      return 0;
    }

  double delta = std::fmod (timeOffsetSeconds,
                            layout.symbolDurationSeconds);
  if (delta < 0.0)
    {
      delta += layout.symbolDurationSeconds;
    }

  auto halfChips = static_cast<std::uint16_t> (
    0.5 + delta / (CSR_CHIP_SECONDS / 2.0));
  if (halfChips == layout.halfChipOffsetCount)
    {
      halfChips = 0;
    }
  return halfChips;
}

double
GetSparseValue (const double *values,
                std::size_t storedCount,
                std::size_t index)
{
  return index < storedCount ? values[index] : 0.0;
}

double
Interpolate (const double *values,
             std::size_t storedCount,
             std::size_t sampleCount,
             double xStart,
             double xStep,
             double x)
{
  if (!(x > xStart))
    {
      return GetSparseValue (values, storedCount, 0);
    }

  double xEnd = xStart +
    xStep * static_cast<double> (sampleCount - 1);
  if (x >= xEnd)
    {
      return GetSparseValue (values, storedCount, sampleCount - 1);
    }

  double position = (x - xStart) / xStep;
  double nearest = std::round (position);
  double tolerance = 8.0 * std::numeric_limits<double>::epsilon () *
    std::max (1.0, std::fabs (position));
  if (std::fabs (position - nearest) <= tolerance)
    {
      return GetSparseValue (values,
                             storedCount,
                             static_cast<std::size_t> (nearest));
    }

  auto lower = static_cast<std::size_t> (std::floor (position));
  double fraction = position - static_cast<double> (lower);
  double lowerValue = GetSparseValue (values, storedCount, lower);
  double upperValue = GetSparseValue (values, storedCount, lower + 1);
  return lowerValue + fraction * (upperValue - lowerValue);
}

} // namespace

double
CsrOpnetBerTables::GetStandardBer (double effectiveSnrDb)
{
  return Interpolate (CSR_STANDARD_VALUES,
                      CSR_STANDARD_STORED_COUNT,
                      CSR_STANDARD_SAMPLE_COUNT,
                      CSR_STANDARD_X_START,
                      CSR_STANDARD_X_STEP,
                      effectiveSnrDb);
}

double
CsrOpnetBerTables::GetDqpskBer (double effectiveSnrDb)
{
  return Interpolate (CSR_DQPSK_VALUES,
                      CSR_DQPSK_STORED_COUNT,
                      CSR_DQPSK_SAMPLE_COUNT,
                      CSR_DQPSK_X_START,
                      CSR_DQPSK_X_STEP,
                      effectiveSnrDb);
}

double
CsrOpnetBerTables::GetCollisionBer (int rateKbps,
                                    double jsrDb,
                                    double timeOffsetSeconds,
                                    double effectiveSnrDb)
{
  const auto &layout = GetRateLayout (rateKbps);
  std::size_t jsrIndex = GetJsrBucketIndex (QuantizeJsrDb (jsrDb));
  std::uint16_t halfChipOffset = QuantizeHalfChipOffset (
    layout,
    timeOffsetSeconds);
  std::size_t curve = layout.firstCurve +
    jsrIndex * layout.halfChipOffsetCount + halfChipOffset;
  std::size_t firstValue = CSR_COLLISION_VALUE_OFFSETS[curve];
  std::size_t storedCount = CSR_COLLISION_VALUE_OFFSETS[curve + 1] -
    firstValue;

  return Interpolate (CSR_COLLISION_VALUES + firstValue,
                      storedCount,
                      CSR_COLLISION_SAMPLE_COUNT,
                      CSR_COLLISION_X_START,
                      CSR_COLLISION_X_STEP,
                      effectiveSnrDb);
}

int
CsrOpnetBerTables::QuantizeJsrDb (double jsrDb)
{
  if (jsrDb <= -10.5)
    {
      return -12;
    }
  if (jsrDb <= -7.5)
    {
      return -9;
    }
  if (jsrDb <= -4.5)
    {
      return -6;
    }
  if (jsrDb <= -1.5)
    {
      return -3;
    }
  return 0;
}

double
CsrOpnetBerTables::QuantizeChipOffset (int rateKbps,
                                       double timeOffsetSeconds)
{
  const auto &layout = GetRateLayout (rateKbps);
  return static_cast<double> (
    QuantizeHalfChipOffset (layout, timeOffsetSeconds)) / 2.0;
}

double
CsrOpnetBerTables::SymbolDurationSeconds (int rateKbps)
{
  if (rateKbps == 500)
    {
      return 4.0 / 500000.0;
    }
  if (rateKbps == 1000)
    {
      return 4.0 / 1000000.0;
    }
  return GetRateLayout (rateKbps).symbolDurationSeconds;
}

} // namespace ns3
