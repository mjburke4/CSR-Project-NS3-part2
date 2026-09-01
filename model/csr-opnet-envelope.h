#pragma once

#include "csr-common.h"
#include "csr-hello-header.h"
#include "csr-opnet-packet-model.h"

#include <cstdint>
#include <span>
#include <vector>

namespace ns3 {

namespace csr_opnet_detail {

static constexpr uint32_t GROUP_ESTABLISH_OVERHEAD = 7;
static constexpr uint32_t GROUP16_OVERHEAD = 5;
static constexpr uint32_t PAIRWISE16_OVERHEAD = 5;
// br_Ack carries two optional 64-bit cumulative receive registers.  They are
// set for ordinary DATA ACK/DACK feedback and remain unset for exact control
// ACKs.
static constexpr uint32_t ACK_WINDOW_OVERHEAD = 16;

inline std::vector<uint8_t>
CopyPacketBytes (Ptr<const Packet> packet)
{
  std::vector<uint8_t> bytes (packet->GetSize ());
  if (!bytes.empty ())
    {
      packet->CopyData (bytes.data (), bytes.size ());
    }
  return bytes;
}

inline uint32_t
GetRawHelloPayloadSize (Ptr<Packet> packet)
{
  CsrHelloHeader compatibilityHeader;
  if (packet->RemoveHeader (compatibilityHeader) == 0)
    {
      return packet->GetSize ();
    }
  return packet->GetSize ();
}

inline uint32_t
GetRoutingControlRecordSize (Ptr<const Packet> frame,
                             const CsrHeader &header)
{
  Ptr<Packet> recordPacket = frame->Copy ();
  CsrHeader removedHeader;
  if (recordPacket->RemoveHeader (removedHeader) !=
      header.GetSerializedSize ())
    {
      return recordPacket->GetSize ();
    }

  std::vector<uint8_t> record = CopyPacketBytes (recordPacket);
  if (record.size () < GROUP16_OVERHEAD)
    {
      return static_cast<uint32_t> (record.size ());
    }

  static constexpr std::size_t keySequenceSize = 3;
  static constexpr std::size_t authTagSize = 2;
  std::span<const uint8_t> plaintext (
    record.data () + keySequenceSize,
    record.size () - keySequenceSize - authTagSize);

  Ptr<Packet> plaintextPacket = Create<Packet> (plaintext.data (),
                                                plaintext.size ());
  return GROUP16_OVERHEAD + GetRawHelloPayloadSize (plaintextPacket);
}

} // namespace csr_opnet_detail

/**
 * Infer and attach the legacy envelope tree represented by a compatibility
 * packet.  Existing annotations are preserved.
 *
 * @param packet Compatibility packet to annotate.
 * @return True when a known CSR packet type was annotated.
 */
inline bool
CsrAnnotateOpnetEnvelope (Ptr<Packet> packet)
{
  if (packet == nullptr)
    {
      return false;
    }

  CsrOpnetPacketFormat existingFormat;
  uint32_t existingSize = 0;
  if (CsrGetOpnetEnvelope (packet, existingFormat, existingSize))
    {
      return true;
    }

  CsrHeader header;
  if (!packet->PeekHeader (header) ||
      packet->GetSize () < header.GetSerializedSize ())
    {
      return false;
    }

  uint32_t compatibilityPayloadSize =
    packet->GetSize () - header.GetSerializedSize ();
  CsrOpnetPacketFormat rootFormat = CsrOpnetPacketFormat::Unknown;
  uint32_t modeledSize = 0;

  switch (header.GetType ())
    {
    case CSR_PKT_DATA:
    case CSR_PKT_PAIRWISE32_DATA:
    case CSR_PKT_NEIGHBORCAST:
      rootFormat = CsrOpnetPacketFormat::Mac;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Mac) +
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Hop) +
        compatibilityPayloadSize;
      break;
    case CSR_PKT_ACK:
    case CSR_PKT_DACK:
      rootFormat = CsrOpnetPacketFormat::Mac;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Mac) +
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Ack) +
        (header.HasAckWindow () ?
           csr_opnet_detail::ACK_WINDOW_OVERHEAD : 0) +
        (header.HasSecurityCount () ?
           csr_opnet_detail::PAIRWISE16_OVERHEAD : 0);
      break;
    case CSR_PKT_HELLO:
      rootFormat = CsrOpnetPacketFormat::Hello;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Hello) +
        (header.HasGroupSecurity () ?
           csr_opnet_detail::GROUP16_OVERHEAD : 0);
      break;
    case CSR_PKT_DISCOVER:
      rootFormat = CsrOpnetPacketFormat::Hello;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Hello) +
        (header.HasGroupSecurity () ?
           csr_opnet_detail::GROUP_ESTABLISH_OVERHEAD : 0);
      break;
    case CSR_PKT_NEIGHBOR_CHECK:
      rootFormat = CsrOpnetPacketFormat::Routes;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Routes);
      break;
    case CSR_PKT_ROUTING_CONTROL:
      rootFormat = CsrOpnetPacketFormat::Routes;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Routes) +
        (header.HasGroupSecurity ()
           ? csr_opnet_detail::GetRoutingControlRecordSize (packet, header)
           : compatibilityPayloadSize);
      break;
    case CSR_PKT_SNMP:
      rootFormat = CsrOpnetPacketFormat::Mac;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Mac) +
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Hop) +
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Snmp);
      break;
    case CSR_PKT_KEY_REQUEST:
    case CSR_PKT_KEY_UPDATE:
      rootFormat = CsrOpnetPacketFormat::Routes;
      modeledSize =
        CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat::Routes) +
        compatibilityPayloadSize;
      break;
    default:
      return false;
    }

  CsrSetOpnetEnvelope (packet, rootFormat, modeledSize);
  return true;
}

} // namespace ns3
