#include "csr-opnet-packet-model.h"

#include "ns3/abort.h"

#include <algorithm>
#include <limits>

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (CsrOpnetEnvelopeTag);

namespace {

constexpr uint32_t CSR_OPNET_ACK_BITS = 64;
constexpr uint32_t CSR_OPNET_HELLO_BITS = 96;
constexpr uint32_t CSR_OPNET_HOP_BITS = 64;
constexpr uint32_t CSR_OPNET_MAC_BITS = 136;
constexpr uint32_t CSR_OPNET_MAC_HOP_INSTANCE_BITS = 64;
constexpr uint32_t CSR_OPNET_NETWORK_BITS = 56;
constexpr uint32_t CSR_OPNET_OTA_LONG_BITS = 7968;
constexpr uint32_t CSR_OPNET_OTA_SHORT_BITS = 184;
constexpr uint32_t CSR_OPNET_ROUTES_BITS = 88;
constexpr uint32_t CSR_OPNET_SNMP_BITS = 48;
constexpr uint32_t CSR_OPNET_SENT_INFO_BITS = 24;

void
SetError (std::string *error, const std::string &message)
{
  if (error != nullptr)
    {
      *error = message;
    }
}

void
AppendU16 (std::vector<uint8_t> &bytes, uint16_t value)
{
  bytes.push_back (static_cast<uint8_t> (value >> 8));
  bytes.push_back (static_cast<uint8_t> (value));
}

void
AppendU24 (std::vector<uint8_t> &bytes, CsrNodeId value)
{
  bytes.push_back (static_cast<uint8_t> (value >> 16));
  bytes.push_back (static_cast<uint8_t> (value >> 8));
  bytes.push_back (static_cast<uint8_t> (value));
}

void
AppendU32 (std::vector<uint8_t> &bytes, uint32_t value)
{
  bytes.push_back (static_cast<uint8_t> (value >> 24));
  bytes.push_back (static_cast<uint8_t> (value >> 16));
  bytes.push_back (static_cast<uint8_t> (value >> 8));
  bytes.push_back (static_cast<uint8_t> (value));
}

bool
RequireNodeId (CsrNodeId value, std::string *error)
{
  if (CsrIsValidNodeId (value))
    {
      return true;
    }

  SetError (error, "OPNET packet-model node identifier exceeds 24 bits");
  return false;
}

bool
RequireBytes (std::span<const uint8_t> bytes,
              std::size_t offset,
              std::size_t required)
{
  return offset <= bytes.size () && required <= bytes.size () - offset;
}

uint8_t
ReadU8 (std::span<const uint8_t> bytes, std::size_t &offset)
{
  return bytes[offset++];
}

uint16_t
ReadU16 (std::span<const uint8_t> bytes, std::size_t &offset)
{
  uint16_t value = static_cast<uint16_t> (bytes[offset]) << 8;
  value |= bytes[offset + 1];
  offset += 2;
  return value;
}

CsrNodeId
ReadU24 (std::span<const uint8_t> bytes, std::size_t &offset)
{
  CsrNodeId value = static_cast<CsrNodeId> (bytes[offset]) << 16;
  value |= static_cast<CsrNodeId> (bytes[offset + 1]) << 8;
  value |= bytes[offset + 2];
  offset += 3;
  return value;
}

uint32_t
ReadU32 (std::span<const uint8_t> bytes, std::size_t &offset)
{
  uint32_t value = static_cast<uint32_t> (bytes[offset]) << 24;
  value |= static_cast<uint32_t> (bytes[offset + 1]) << 16;
  value |= static_cast<uint32_t> (bytes[offset + 2]) << 8;
  value |= bytes[offset + 3];
  offset += 4;
  return value;
}

bool
RequireExactSize (CsrOpnetPacketFormat format,
                  std::span<const uint8_t> bytes,
                  std::string *error)
{
  uint32_t fixedSize = CsrOpnetPacketModel::GetFixedSizeBytes (format);
  if (fixedSize == 0)
    {
      SetError (error, "unknown OPNET packet format");
      return false;
    }

  if (!CsrOpnetPacketModel::HasInheritedPayload (format) &&
      bytes.size () != fixedSize)
    {
      SetError (error, "OPNET fixed packet has the wrong serialized size");
      return false;
    }

  if (CsrOpnetPacketModel::HasInheritedPayload (format) &&
      bytes.size () < fixedSize)
    {
      SetError (error, "OPNET inherited-payload packet is truncated");
      return false;
    }

  return true;
}

} // namespace

const char *
CsrOpnetPacketModel::GetName (CsrOpnetPacketFormat format)
{
  switch (format)
    {
    case CsrOpnetPacketFormat::Ack:
      return "br_Ack";
    case CsrOpnetPacketFormat::Hello:
      return "br_Hello";
    case CsrOpnetPacketFormat::Hop:
      return "br_Hop";
    case CsrOpnetPacketFormat::Mac:
      return "br_Mac";
    case CsrOpnetPacketFormat::MacHopInstance:
      return "br_Mac_Hop_Inst";
    case CsrOpnetPacketFormat::Network:
      return "br_Network";
    case CsrOpnetPacketFormat::OtaLongPreamble:
      return "br_OTA_LongPream";
    case CsrOpnetPacketFormat::OtaShortPreamble:
      return "br_OTA_ShortPream";
    case CsrOpnetPacketFormat::Routes:
      return "br_Routes";
    case CsrOpnetPacketFormat::Snmp:
      return "br_SNMP";
    case CsrOpnetPacketFormat::SentInfo:
      return "br_Sent_Info";
    case CsrOpnetPacketFormat::Unknown:
      return "unknown";
    }
  return "unknown";
}

uint32_t
CsrOpnetPacketModel::GetFixedSizeBits (CsrOpnetPacketFormat format)
{
  switch (format)
    {
    case CsrOpnetPacketFormat::Ack:
      return CSR_OPNET_ACK_BITS;
    case CsrOpnetPacketFormat::Hello:
      return CSR_OPNET_HELLO_BITS;
    case CsrOpnetPacketFormat::Hop:
      return CSR_OPNET_HOP_BITS;
    case CsrOpnetPacketFormat::Mac:
      return CSR_OPNET_MAC_BITS;
    case CsrOpnetPacketFormat::MacHopInstance:
      return CSR_OPNET_MAC_HOP_INSTANCE_BITS;
    case CsrOpnetPacketFormat::Network:
      return CSR_OPNET_NETWORK_BITS;
    case CsrOpnetPacketFormat::OtaLongPreamble:
      return CSR_OPNET_OTA_LONG_BITS;
    case CsrOpnetPacketFormat::OtaShortPreamble:
      return CSR_OPNET_OTA_SHORT_BITS;
    case CsrOpnetPacketFormat::Routes:
      return CSR_OPNET_ROUTES_BITS;
    case CsrOpnetPacketFormat::Snmp:
      return CSR_OPNET_SNMP_BITS;
    case CsrOpnetPacketFormat::SentInfo:
      return CSR_OPNET_SENT_INFO_BITS;
    case CsrOpnetPacketFormat::Unknown:
      return 0;
    }
  return 0;
}

uint32_t
CsrOpnetPacketModel::GetFixedSizeBytes (CsrOpnetPacketFormat format)
{
  uint32_t bits = GetFixedSizeBits (format);
  NS_ABORT_MSG_IF (bits % 8 != 0,
                   "CSR OPNET packet model is not byte aligned");
  return bits / 8;
}

bool
CsrOpnetPacketModel::HasInheritedPayload (CsrOpnetPacketFormat format)
{
  switch (format)
    {
    case CsrOpnetPacketFormat::Hop:
    case CsrOpnetPacketFormat::Mac:
    case CsrOpnetPacketFormat::Network:
    case CsrOpnetPacketFormat::OtaLongPreamble:
    case CsrOpnetPacketFormat::OtaShortPreamble:
      return true;
    case CsrOpnetPacketFormat::Ack:
    case CsrOpnetPacketFormat::Hello:
    case CsrOpnetPacketFormat::MacHopInstance:
    case CsrOpnetPacketFormat::Routes:
    case CsrOpnetPacketFormat::Snmp:
    case CsrOpnetPacketFormat::SentInfo:
    case CsrOpnetPacketFormat::Unknown:
      return false;
    }
  return false;
}

uint32_t
CsrOpnetPacketModel::GetModeledSizeBytes (CsrOpnetPacketFormat format,
                                          uint32_t payloadBytes)
{
  uint32_t fixedSize = GetFixedSizeBytes (format);
  if (fixedSize == 0)
    {
      return 0;
    }
  return fixedSize + (HasInheritedPayload (format) ? payloadBytes : 0);
}

bool
CsrOpnetPacketModel::Serialize (CsrOpnetPacketFormat format,
                                const CsrOpnetPacketFields &fields,
                                std::span<const uint8_t> payload,
                                std::vector<uint8_t> &bytes,
                                std::string *error)
{
  bytes.clear ();
  uint32_t fixedSize = GetFixedSizeBytes (format);
  if (fixedSize == 0)
    {
      SetError (error, "unknown OPNET packet format");
      return false;
    }

  if (!HasInheritedPayload (format) && !payload.empty ())
    {
      SetError (error, "OPNET packet format has no inherited payload");
      return false;
    }

  bytes.reserve (fixedSize + payload.size ());

  switch (format)
    {
    case CsrOpnetPacketFormat::Ack:
      if (!RequireNodeId (fields.source, error) ||
          !RequireNodeId (fields.destination, error))
        {
          return false;
        }
      AppendU24 (bytes, fields.source);
      AppendU24 (bytes, fields.destination);
      AppendU16 (bytes, fields.sequence16);
      break;
    case CsrOpnetPacketFormat::Hello:
      if (!RequireNodeId (fields.nodeId, error))
        {
          return false;
        }
      bytes.push_back (fields.sequence8);
      AppendU24 (bytes, fields.nodeId);
      bytes.push_back (fields.capabilityNumber);
      bytes.push_back (fields.capability);
      AppendU16 (bytes, fields.cost);
      AppendU32 (bytes, fields.timeOffset);
      break;
    case CsrOpnetPacketFormat::Hop:
      if (!RequireNodeId (fields.source, error) ||
          !RequireNodeId (fields.destination, error))
        {
          return false;
        }
      AppendU24 (bytes, fields.source);
      bytes.push_back (fields.numberOfDestinations);
      AppendU24 (bytes, fields.destination);
      bytes.push_back (fields.sequence8);
      bytes.insert (bytes.end (), payload.begin (), payload.end ());
      break;
    case CsrOpnetPacketFormat::Mac:
      if (!RequireNodeId (fields.globalAddress, error) ||
          !RequireNodeId (fields.source, error))
        {
          return false;
        }
      AppendU32 (bytes, fields.globalTime);
      AppendU24 (bytes, fields.globalAddress);
      bytes.push_back (fields.globalCost);
      bytes.push_back (fields.txPower);
      bytes.push_back (fields.rxPower);
      bytes.push_back (fields.active);
      AppendU24 (bytes, fields.source);
      AppendU16 (bytes, fields.payloadLength);
      bytes.push_back (fields.type);
      bytes.insert (bytes.end (), payload.begin (), payload.end ());
      break;
    case CsrOpnetPacketFormat::MacHopInstance:
      if (!RequireNodeId (fields.source, error) ||
          !RequireNodeId (fields.destination, error))
        {
          return false;
        }
      AppendU24 (bytes, fields.source);
      AppendU24 (bytes, fields.destination);
      AppendU16 (bytes, fields.sequence16);
      break;
    case CsrOpnetPacketFormat::Network:
      if (!RequireNodeId (fields.source, error) ||
          !RequireNodeId (fields.destination, error))
        {
          return false;
        }
      AppendU24 (bytes, fields.source);
      AppendU24 (bytes, fields.destination);
      bytes.insert (bytes.end (), payload.begin (), payload.end ());
      bytes.push_back (fields.dscp);
      break;
    case CsrOpnetPacketFormat::OtaLongPreamble:
    case CsrOpnetPacketFormat::OtaShortPreamble:
      {
        uint32_t preambleBytes =
          (format == CsrOpnetPacketFormat::OtaLongPreamble) ? 986 : 13;
        bytes.insert (bytes.end (), preambleBytes, 0);
        AppendU16 (bytes, fields.startOfFrame);
        AppendU16 (bytes, fields.speed);
        AppendU16 (bytes, fields.length);
        bytes.insert (bytes.end (), payload.begin (), payload.end ());
        AppendU32 (bytes, fields.fcs);
        break;
      }
    case CsrOpnetPacketFormat::Routes:
      if (!RequireNodeId (fields.nodeId, error))
        {
          return false;
        }
      AppendU24 (bytes, fields.nodeId);
      bytes.push_back (fields.capabilityNumber);
      bytes.push_back (fields.capability);
      AppendU16 (bytes, fields.cost);
      AppendU32 (bytes, fields.timeOffset);
      break;
    case CsrOpnetPacketFormat::Snmp:
      AppendU16 (bytes, fields.message);
      AppendU32 (bytes, fields.value);
      break;
    case CsrOpnetPacketFormat::SentInfo:
      if (!RequireNodeId (fields.source, error))
        {
          return false;
        }
      AppendU24 (bytes, fields.source);
      break;
    case CsrOpnetPacketFormat::Unknown:
      return false;
    }

  NS_ABORT_MSG_IF (bytes.size () != fixedSize + payload.size (),
                   "CSR OPNET serializer emitted an unexpected size");
  return true;
}

bool
CsrOpnetPacketModel::Deserialize (CsrOpnetPacketFormat format,
                                  std::span<const uint8_t> bytes,
                                  CsrOpnetPacketFields &fields,
                                  std::vector<uint8_t> &payload,
                                  std::string *error)
{
  fields = {};
  payload.clear ();
  if (!RequireExactSize (format, bytes, error))
    {
      return false;
    }

  std::size_t offset = 0;
  switch (format)
    {
    case CsrOpnetPacketFormat::Ack:
      fields.source = ReadU24 (bytes, offset);
      fields.destination = ReadU24 (bytes, offset);
      fields.sequence16 = ReadU16 (bytes, offset);
      break;
    case CsrOpnetPacketFormat::Hello:
      fields.sequence8 = ReadU8 (bytes, offset);
      fields.nodeId = ReadU24 (bytes, offset);
      fields.capabilityNumber = ReadU8 (bytes, offset);
      fields.capability = ReadU8 (bytes, offset);
      fields.cost = ReadU16 (bytes, offset);
      fields.timeOffset = ReadU32 (bytes, offset);
      break;
    case CsrOpnetPacketFormat::Hop:
      fields.source = ReadU24 (bytes, offset);
      fields.numberOfDestinations = ReadU8 (bytes, offset);
      fields.destination = ReadU24 (bytes, offset);
      fields.sequence8 = ReadU8 (bytes, offset);
      payload.assign (bytes.begin () + offset, bytes.end ());
      offset = bytes.size ();
      break;
    case CsrOpnetPacketFormat::Mac:
      fields.globalTime = ReadU32 (bytes, offset);
      fields.globalAddress = ReadU24 (bytes, offset);
      fields.globalCost = ReadU8 (bytes, offset);
      fields.txPower = ReadU8 (bytes, offset);
      fields.rxPower = ReadU8 (bytes, offset);
      fields.active = ReadU8 (bytes, offset);
      fields.source = ReadU24 (bytes, offset);
      fields.payloadLength = ReadU16 (bytes, offset);
      fields.type = ReadU8 (bytes, offset);
      payload.assign (bytes.begin () + offset, bytes.end ());
      offset = bytes.size ();
      break;
    case CsrOpnetPacketFormat::MacHopInstance:
      fields.source = ReadU24 (bytes, offset);
      fields.destination = ReadU24 (bytes, offset);
      fields.sequence16 = ReadU16 (bytes, offset);
      break;
    case CsrOpnetPacketFormat::Network:
      fields.source = ReadU24 (bytes, offset);
      fields.destination = ReadU24 (bytes, offset);
      payload.assign (bytes.begin () + offset, bytes.end () - 1);
      fields.dscp = bytes.back ();
      offset = bytes.size ();
      break;
    case CsrOpnetPacketFormat::OtaLongPreamble:
    case CsrOpnetPacketFormat::OtaShortPreamble:
      {
        uint32_t preambleBytes =
          (format == CsrOpnetPacketFormat::OtaLongPreamble) ? 986 : 13;
        offset += preambleBytes;
        fields.startOfFrame = ReadU16 (bytes, offset);
        fields.speed = ReadU16 (bytes, offset);
        fields.length = ReadU16 (bytes, offset);
        std::size_t payloadEnd = bytes.size () - 4;
        payload.assign (bytes.begin () + offset,
                        bytes.begin () + payloadEnd);
        offset = payloadEnd;
        fields.fcs = ReadU32 (bytes, offset);
        break;
      }
    case CsrOpnetPacketFormat::Routes:
      fields.nodeId = ReadU24 (bytes, offset);
      fields.capabilityNumber = ReadU8 (bytes, offset);
      fields.capability = ReadU8 (bytes, offset);
      fields.cost = ReadU16 (bytes, offset);
      fields.timeOffset = ReadU32 (bytes, offset);
      break;
    case CsrOpnetPacketFormat::Snmp:
      fields.message = ReadU16 (bytes, offset);
      fields.value = ReadU32 (bytes, offset);
      break;
    case CsrOpnetPacketFormat::SentInfo:
      fields.source = ReadU24 (bytes, offset);
      break;
    case CsrOpnetPacketFormat::Unknown:
      return false;
    }

  if (!RequireBytes (bytes, 0, offset) || offset != bytes.size ())
    {
      SetError (error, "OPNET packet decoder did not consume the packet");
      return false;
    }
  return true;
}

TypeId
CsrOpnetEnvelopeTag::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::CsrOpnetEnvelopeTag")
    .SetParent<Tag> ()
    .SetGroupName ("Csr")
    .AddConstructor<CsrOpnetEnvelopeTag> ();
  return tid;
}

TypeId
CsrOpnetEnvelopeTag::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
CsrOpnetEnvelopeTag::GetSerializedSize () const
{
  return 5;
}

void
CsrOpnetEnvelopeTag::Serialize (TagBuffer buffer) const
{
  buffer.WriteU8 (static_cast<uint8_t> (m_format));
  buffer.WriteU32 (m_modeledSizeBytes);
}

void
CsrOpnetEnvelopeTag::Deserialize (TagBuffer buffer)
{
  m_format = static_cast<CsrOpnetPacketFormat> (buffer.ReadU8 ());
  m_modeledSizeBytes = buffer.ReadU32 ();
}

void
CsrOpnetEnvelopeTag::Print (std::ostream &os) const
{
  os << CsrOpnetPacketModel::GetName (m_format)
     << " modeledBytes=" << m_modeledSizeBytes;
}

void
CsrOpnetEnvelopeTag::SetFormat (CsrOpnetPacketFormat format)
{
  m_format = format;
}

CsrOpnetPacketFormat
CsrOpnetEnvelopeTag::GetFormat () const
{
  return m_format;
}

void
CsrOpnetEnvelopeTag::SetModeledSizeBytes (uint32_t size)
{
  m_modeledSizeBytes = size;
}

uint32_t
CsrOpnetEnvelopeTag::GetModeledSizeBytes () const
{
  return m_modeledSizeBytes;
}

void
CsrSetOpnetEnvelope (Ptr<Packet> packet,
                     CsrOpnetPacketFormat format,
                     uint32_t modeledSizeBytes)
{
  NS_ABORT_MSG_IF (packet == nullptr,
                   "cannot tag a null CSR packet");
  NS_ABORT_MSG_IF (format == CsrOpnetPacketFormat::Unknown ||
                   modeledSizeBytes == 0,
                   "cannot attach an invalid CSR OPNET envelope");

  CsrOpnetEnvelopeTag tag;
  tag.SetFormat (format);
  tag.SetModeledSizeBytes (modeledSizeBytes);
  packet->ReplacePacketTag (tag);
}

bool
CsrGetOpnetEnvelope (Ptr<const Packet> packet,
                     CsrOpnetPacketFormat &format,
                     uint32_t &modeledSizeBytes)
{
  if (packet == nullptr)
    {
      return false;
    }

  CsrOpnetEnvelopeTag tag;
  if (!packet->PeekPacketTag (tag) ||
      tag.GetFormat () == CsrOpnetPacketFormat::Unknown ||
      tag.GetModeledSizeBytes () == 0)
    {
      return false;
    }

  format = tag.GetFormat ();
  modeledSizeBytes = tag.GetModeledSizeBytes ();
  return true;
}

uint32_t
CsrGetOpnetWireSize (Ptr<const Packet> packet)
{
  if (packet == nullptr)
    {
      return 0;
    }

  CsrOpnetPacketFormat format;
  uint32_t modeledSize = 0;
  return CsrGetOpnetEnvelope (packet, format, modeledSize)
    ? modeledSize
    : packet->GetSize ();
}

uint32_t
CsrGetOpnetAggregateWireSize (const std::vector<Ptr<Packet>> &packets)
{
  uint64_t total = 0;
  for (const auto &packet : packets)
    {
      total += CsrGetOpnetWireSize (packet);
    }

  NS_ABORT_MSG_IF (total > std::numeric_limits<uint32_t>::max (),
                   "CSR OPNET aggregate exceeds 32-bit size accounting");
  return static_cast<uint32_t> (total);
}

} // namespace ns3
