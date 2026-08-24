#pragma once

#include "csr-wire-format.h"

#include "ns3/packet.h"
#include "ns3/tag.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace ns3 {

/** Packet formats declared by the supplied OPNET br_*.pk.m models. */
enum class CsrOpnetPacketFormat : uint8_t
{
  Unknown = 0,
  Ack,
  Hello,
  Hop,
  Mac,
  MacHopInstance,
  Network,
  OtaLongPreamble,
  OtaShortPreamble,
  Routes,
  Snmp,
  SentInfo
};

/**
 * Superset of the fixed-width fields in the supplied OPNET packet models.
 *
 * Fields represented by an OPNET pointer, structure, packet annotation, or
 * zero-bit integer are deliberately absent.  They are simulator metadata and
 * do not contribute to the modeled packet size.
 */
struct CsrOpnetPacketFields
{
  uint32_t globalTime {0};
  CsrNodeId globalAddress {0};
  uint8_t globalCost {0};
  uint8_t txPower {0};
  uint8_t rxPower {0};
  uint8_t active {0};
  CsrNodeId source {0};
  uint16_t payloadLength {0};
  uint8_t type {0};
  uint8_t numberOfDestinations {0};
  CsrNodeId destination {0};
  uint16_t sequence16 {0};
  uint8_t sequence8 {0};
  uint16_t message {0};
  uint32_t value {0};
  CsrNodeId nodeId {0};
  uint8_t capabilityNumber {0};
  uint8_t capability {0};
  uint16_t cost {0};
  uint32_t timeOffset {0};
  uint8_t dscp {0};
  uint16_t startOfFrame {0};
  uint16_t speed {0};
  uint16_t length {0};
  uint32_t fcs {0};
};

/** Exact fixed-bit serializer for the supplied OPNET packet definitions. */
class CsrOpnetPacketModel
{
public:
  /** @return Human-readable legacy format name. */
  static const char *GetName (CsrOpnetPacketFormat format);

  /** @return Fixed field size from the .pk.m definition, in bits. */
  static uint32_t GetFixedSizeBits (CsrOpnetPacketFormat format);

  /** @return Fixed field size from the .pk.m definition, in bytes. */
  static uint32_t GetFixedSizeBytes (CsrOpnetPacketFormat format);

  /** @return True when the format has an inherited packet payload. */
  static bool HasInheritedPayload (CsrOpnetPacketFormat format);

  /**
   * Return the modeled size of one packet including an inherited payload.
   *
   * @param format Legacy packet format.
   * @param payloadBytes Inherited payload size in bytes.
   * @return Complete modeled packet size, or zero for an unknown format.
   */
  static uint32_t GetModeledSizeBytes (CsrOpnetPacketFormat format,
                                       uint32_t payloadBytes = 0);

  /**
   * Serialize all modeled fields and the optional inherited payload.
   *
   * @param format Legacy packet format.
   * @param fields Fixed-width field values.
   * @param payload Inherited payload bytes.
   * @param bytes Serialized bytes in packet-model field order.
   * @param error Optional validation error text.
   * @return True when serialization succeeds.
   */
  static bool Serialize (CsrOpnetPacketFormat format,
                         const CsrOpnetPacketFields &fields,
                         std::span<const uint8_t> payload,
                         std::vector<uint8_t> &bytes,
                         std::string *error = nullptr);

  /**
   * Deserialize all modeled fields and the inherited payload, when present.
   *
   * @param format Legacy packet format.
   * @param bytes Complete serialized packet bytes.
   * @param fields Decoded fixed-width values.
   * @param payload Decoded inherited payload bytes.
   * @param error Optional validation error text.
   * @return True when the packet has the exact required layout.
   */
  static bool Deserialize (CsrOpnetPacketFormat format,
                           std::span<const uint8_t> bytes,
                           CsrOpnetPacketFields &fields,
                           std::vector<uint8_t> &payload,
                           std::string *error = nullptr);
};

/**
 * Packet tag carrying the exact OPNET-modeled envelope size.
 *
 * The current ns-3 compatibility headers retain zero-bit OPNET metadata needed
 * by the model.  This tag keeps those storage bytes out of MAC concatenation,
 * PHY packet length, and airtime calculations without changing packet data.
 */
class CsrOpnetEnvelopeTag : public Tag
{
public:
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const override;

  uint32_t GetSerializedSize () const override;
  void Serialize (TagBuffer buffer) const override;
  void Deserialize (TagBuffer buffer) override;
  void Print (std::ostream &os) const override;

  void SetFormat (CsrOpnetPacketFormat format);
  CsrOpnetPacketFormat GetFormat () const;

  void SetModeledSizeBytes (uint32_t size);
  uint32_t GetModeledSizeBytes () const;

private:
  CsrOpnetPacketFormat m_format {CsrOpnetPacketFormat::Unknown};
  uint32_t m_modeledSizeBytes {0};
};

/** Attach or replace an exact OPNET envelope-size annotation. */
void CsrSetOpnetEnvelope (Ptr<Packet> packet,
                          CsrOpnetPacketFormat format,
                          uint32_t modeledSizeBytes);

/** Read an exact OPNET envelope-size annotation. */
bool CsrGetOpnetEnvelope (Ptr<const Packet> packet,
                          CsrOpnetPacketFormat &format,
                          uint32_t &modeledSizeBytes);

/** Return the OPNET-modeled size, falling back to Packet::GetSize(). */
uint32_t CsrGetOpnetWireSize (Ptr<const Packet> packet);

/** Return the sum of the modeled sizes in an OPNET segmentation aggregate. */
uint32_t CsrGetOpnetAggregateWireSize (
  const std::vector<Ptr<Packet>> &packets);

} // namespace ns3
