#pragma once

#include "ns3/abort.h"
#include "ns3/buffer.h"

#include <cstdint>

namespace ns3 {

/** Node identifier stored in the legacy CSR 24-bit address space. */
using CsrNodeId = uint32_t;

/** Legacy payload-rate key expressed in kbit/s. */
using CsrRateKey = uint16_t;

// These ns-3 headers are compact compatibility envelopes, not the OPNET
// packet's 16-bit OTA Speed field.  Preserve their existing one-byte layout
// by assigning otherwise-unused wire codes to the two high-rate keys.
static constexpr uint8_t CSR_RATE_CODE_500_KBPS = 0x81;
static constexpr uint8_t CSR_RATE_CODE_1000_KBPS = 0x82;

static constexpr CsrNodeId CSR_NODE_ID_MAX = 0x00ffffffu;
static constexpr CsrNodeId CSR_BROADCAST_ID = CSR_NODE_ID_MAX;
static constexpr uint32_t CSR_NODE_ID_WIRE_SIZE = 3;

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
  NS_ABORT_MSG_IF (rateKbps > 255,
                   "unsupported CSR payload-rate key");
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
