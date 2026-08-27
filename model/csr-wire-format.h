#pragma once

#include "ns3/abort.h"
#include "ns3/buffer.h"

#include <cstdint>

namespace ns3 {

/** Node identifier stored in the legacy CSR 24-bit address space. */
using CsrNodeId = uint32_t;

/** Legacy payload-rate key expressed in kbit/s. */
using CsrRateKey = uint16_t;

/** Runtime rate sets available to the CSR stack. */
enum class CsrRateProfile
{
  LEGACY_SOURCE_EXACT, ///< Source-exact 8-128-kbit/s spread-rate set.
  EXTENDED_DQPSK      ///< Add recovered 500-DPSK/1000-DQPSK modes (compatibility name).
};

// These ns-3 headers are compact compatibility envelopes, not the OPNET
// packet's 16-bit OTA Speed field.  Preserve their existing one-byte layout
// by assigning otherwise-unused wire codes to the two high-rate keys.
static constexpr uint8_t CSR_RATE_CODE_500_KBPS = 0x81;
static constexpr uint8_t CSR_RATE_CODE_1000_KBPS = 0x82;

static constexpr CsrNodeId CSR_NODE_ID_MAX = 0x00ffffffu;
static constexpr CsrNodeId CSR_BROADCAST_ID = CSR_NODE_ID_MAX;
static constexpr uint32_t CSR_NODE_ID_WIRE_SIZE = 3;

/**
 * Check whether a rate key is valid for live radio operation.
 *
 * Arbitrary one-byte values remain serializable for packet-format tests and
 * unknown legacy metadata, but only these seven values have defined airtime,
 * link-control, and BER behavior.
 *
 * @param rateKbps Candidate payload rate in kbit/s.
 * @return True when the live stack implements the rate.
 */
inline bool
CsrIsOperationalRateKey (CsrRateKey rateKbps)
{
  switch (rateKbps)
    {
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 500:
    case 1000:
      return true;
    default:
      return false;
    }
}

/**
 * Return the default maximum rate for a runtime profile.
 *
 * @param profile Requested legacy or owner-confirmed extended profile.
 * @return Maximum operational rate in kbit/s.
 */
inline CsrRateKey
CsrGetProfileMaxRateKbps (CsrRateProfile profile)
{
  switch (profile)
    {
    case CsrRateProfile::LEGACY_SOURCE_EXACT:
      return 128;
    case CsrRateProfile::EXTENDED_DQPSK:
      return 1000;
    default:
      NS_ABORT_MSG ("unknown CSR rate profile");
      return 128;
    }
}

/**
 * Check whether a rate key has an unambiguous compact representation.
 *
 * @param rateKbps Candidate payload-rate key.
 * @return True when CsrEncodeRateKey can encode the value.
 */
inline bool
CsrCanEncodeRateKey (CsrRateKey rateKbps)
{
  if (rateKbps == 500 || rateKbps == 1000)
    {
      return true;
    }
  return rateKbps <= 255 &&
         rateKbps != CSR_RATE_CODE_500_KBPS &&
         rateKbps != CSR_RATE_CODE_1000_KBPS;
}

/**
 * Check whether a value can be represented by a legacy CSR node-ID field.
 *
 * @param nodeId Candidate node identifier.
 * @return True when the value fits in 24 bits.
 */
inline bool
CsrIsValidNodeId (CsrNodeId nodeId)
{
  return nodeId <= CSR_NODE_ID_MAX;
}

/**
 * Serialize one unsigned legacy CSR node identifier in big-endian order.
 *
 * @param iterator Buffer position to write and advance.
 * @param nodeId Node identifier to serialize.
 */
inline void
CsrWriteNodeId (Buffer::Iterator &iterator, CsrNodeId nodeId)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (nodeId),
                   "CSR node identifier exceeds the legacy 24-bit field");
  iterator.WriteU8 (static_cast<uint8_t> (nodeId >> 16));
  iterator.WriteU8 (static_cast<uint8_t> (nodeId >> 8));
  iterator.WriteU8 (static_cast<uint8_t> (nodeId));
}

/**
 * Deserialize one unsigned legacy CSR node identifier in big-endian order.
 *
 * @param iterator Buffer position to read and advance.
 * @return Decoded 24-bit node identifier.
 */
inline CsrNodeId
CsrReadNodeId (Buffer::Iterator &iterator)
{
  CsrNodeId nodeId = static_cast<CsrNodeId> (iterator.ReadU8 ()) << 16;
  nodeId |= static_cast<CsrNodeId> (iterator.ReadU8 ()) << 8;
  nodeId |= iterator.ReadU8 ();
  return nodeId;
}

/** Encode a payload-rate key in the compact ns-3 compatibility header. */
inline uint8_t
CsrEncodeRateKey (CsrRateKey rateKbps)
{
  if (rateKbps == 500)
    {
      return CSR_RATE_CODE_500_KBPS;
    }
  if (rateKbps == 1000)
    {
      return CSR_RATE_CODE_1000_KBPS;
    }
  NS_ABORT_MSG_IF (!CsrCanEncodeRateKey (rateKbps),
                   "unsupported or ambiguous CSR payload-rate key");
  return static_cast<uint8_t> (rateKbps);
}

/** Decode a payload-rate key from the compact ns-3 compatibility header. */
inline CsrRateKey
CsrDecodeRateKey (uint8_t encoded)
{
  if (encoded == CSR_RATE_CODE_500_KBPS)
    {
      return 500;
    }
  if (encoded == CSR_RATE_CODE_1000_KBPS)
    {
      return 1000;
    }
  return encoded;
}

} // namespace ns3
