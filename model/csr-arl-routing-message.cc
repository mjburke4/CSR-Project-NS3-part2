#include "csr-arl-routing-message.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace ns3 {

namespace {

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
AppendU24 (std::vector<uint8_t> &bytes, uint32_t value)
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
RequireBytes (std::size_t offset,
              std::size_t required,
              std::size_t available)
{
  return offset <= available && required <= available - offset;
}

uint16_t
ReadU16 (const std::vector<uint8_t> &bytes, std::size_t &offset)
{
  uint16_t value =
    static_cast<uint16_t> (bytes[offset]) << 8;
  value |= bytes[offset + 1];
  offset += 2;
  return value;
}

uint32_t
ReadU24 (const std::vector<uint8_t> &bytes, std::size_t &offset)
{
  uint32_t value =
    static_cast<uint32_t> (bytes[offset]) << 16;
  value |= static_cast<uint32_t> (bytes[offset + 1]) << 8;
  value |= bytes[offset + 2];
  offset += 3;
  return value;
}

uint32_t
ReadU32 (const std::vector<uint8_t> &bytes, std::size_t &offset)
{
  uint32_t value =
    static_cast<uint32_t> (bytes[offset]) << 24;
  value |= static_cast<uint32_t> (bytes[offset + 1]) << 16;
  value |= static_cast<uint32_t> (bytes[offset + 2]) << 8;
  value |= bytes[offset + 3];
  offset += 4;
  return value;
}

} // namespace

void
CsrArlRoutingMessage::Builder::AddFlush ()
{
  m_stream.push_back (
    static_cast<uint8_t> (CsrRoutingOperation::Flush));
}

void
CsrArlRoutingMessage::Builder::AddRequest ()
{
  m_stream.push_back (
    static_cast<uint8_t> (CsrRoutingOperation::Request));
}

void
CsrArlRoutingMessage::Builder::AddInfo (
  const CsrHelloHeader::RoutingInfo &info)
{
  m_stream.push_back (
    static_cast<uint8_t> (CsrRoutingOperation::Info));

  AppendU16 (m_stream, info.minSpeedKbps);
  AppendU16 (m_stream, info.maxSpeedKbps);
  AppendU16 (m_stream,
             static_cast<uint16_t> (info.minPowerDbmX10));
  AppendU16 (m_stream,
             static_cast<uint16_t> (info.maxPowerDbmX10));
  AppendU16 (m_stream,
             static_cast<uint16_t> (info.linkMarginDbX10));
  AppendU16 (m_stream,
             static_cast<uint16_t> (info.lowPowerDbmX10));
  AppendU16 (m_stream,
             static_cast<uint16_t> (info.tempLowCx10));
  AppendU16 (m_stream,
             static_cast<uint16_t> (info.tempHighCx10));
}

bool
CsrArlRoutingMessage::Builder::AddDelete (
  uint32_t nodeId,
  std::string *error)
{
  if (nodeId > MAX_NODE_ID)
    {
      SetError (error, "ARL node identifier exceeds 24 bits");
      return false;
    }

  m_stream.push_back (
    static_cast<uint8_t> (CsrRoutingOperation::Delete));
  AppendU24 (m_stream, nodeId);
  return true;
}

bool
CsrArlRoutingMessage::Builder::AddUpdate (
  uint32_t nodeId,
  uint8_t capability,
  uint16_t hopCount,
  uint32_t cost,
  const std::vector<uint32_t> &path,
  std::string *error)
{
  if (nodeId > MAX_NODE_ID)
    {
      SetError (error, "ARL node identifier exceeds 24 bits");
      return false;
    }

  if (path.size () != hopCount)
    {
      SetError (error,
                "ARL UPDATE path length does not match 16-bit hop count");
      return false;
    }

  for (uint32_t hop : path)
    {
      if (hop > MAX_NODE_ID)
        {
          SetError (error, "ARL path identifier exceeds 24 bits");
          return false;
        }
    }

  m_stream.push_back (
    static_cast<uint8_t> (CsrRoutingOperation::Update));
  AppendU24 (m_stream, nodeId);
  m_stream.push_back (capability);
  AppendU16 (m_stream, hopCount);
  AppendU32 (m_stream, cost);

  for (uint32_t hop : path)
    {
      AppendU24 (m_stream, hop);
    }

  return true;
}

const std::vector<uint8_t> &
CsrArlRoutingMessage::Builder::GetRecordStream () const
{
  return m_stream;
}

bool
CsrArlRoutingMessage::Builder::BuildSections (
  uint32_t sequence,
  std::vector<std::vector<uint8_t>> &sections,
  std::string *error) const
{
  sections.clear ();

  if (m_stream.empty ())
    {
      SetError (error, "cannot section an empty ARL routing message");
      return false;
    }

  std::size_t sectionCount =
    (m_stream.size () + MAX_SECTION_BODY_SIZE - 1) /
      MAX_SECTION_BODY_SIZE;

  if (sectionCount == 0 ||
      sectionCount > std::numeric_limits<uint8_t>::max ())
    {
      SetError (error, "ARL routing message exceeds 255 sections");
      return false;
    }

  sections.reserve (sectionCount);

  for (std::size_t sectionIndex = 0;
       sectionIndex < sectionCount;
       ++sectionIndex)
    {
      std::size_t first =
        sectionIndex * MAX_SECTION_BODY_SIZE;

      std::size_t count =
        std::min (MAX_SECTION_BODY_SIZE,
                  m_stream.size () - first);

      std::vector<uint8_t> section;
      section.reserve (SECTION_PREFIX_SIZE + count);

      AppendU32 (section, sequence);
      section.push_back (static_cast<uint8_t> (sectionIndex));
      section.push_back (static_cast<uint8_t> (sectionCount));
      section.insert (section.end (),
                      m_stream.begin () + first,
                      m_stream.begin () + first + count);

      sections.push_back (std::move (section));
    }

  return true;
}

bool
CsrArlRoutingMessage::DecodeSection (
  const uint8_t *bytes,
  std::size_t length,
  Section &section,
  std::string *error)
{
  section = Section {};

  if (bytes == nullptr || length < SECTION_PREFIX_SIZE)
    {
      SetError (error, "ARL routing section is shorter than six bytes");
      return false;
    }

  if (length > MAX_SECTION_SIZE)
    {
      SetError (error, "ARL routing section exceeds 700 bytes");
      return false;
    }

  section.sequence =
    static_cast<uint32_t> (bytes[0]) << 24;
  section.sequence |=
    static_cast<uint32_t> (bytes[1]) << 16;
  section.sequence |=
    static_cast<uint32_t> (bytes[2]) << 8;
  section.sequence |= bytes[3];
  section.section = bytes[4];
  section.totalSections = bytes[5];

  if (section.totalSections == 0 ||
      section.section >= section.totalSections)
    {
      SetError (error, "ARL routing section index/total is invalid");
      return false;
    }

  section.body.assign (bytes + SECTION_PREFIX_SIZE,
                       bytes + length);
  return true;
}

bool
CsrArlRoutingMessage::DecodeSection (
  const std::vector<uint8_t> &bytes,
  Section &section,
  std::string *error)
{
  return DecodeSection (bytes.data (), bytes.size (), section, error);
}

bool
CsrArlRoutingMessage::ParseRecordStream (
  const std::vector<uint8_t> &stream,
  std::vector<Record> &records,
  std::string *error)
{
  records.clear ();
  std::size_t offset = 0;

  while (offset < stream.size ())
    {
      Record record;
      uint8_t operation = stream[offset++];
      record.operation =
        static_cast<CsrRoutingOperation> (operation);

      switch (record.operation)
        {
        case CsrRoutingOperation::Flush:
        case CsrRoutingOperation::Request:
          break;

        case CsrRoutingOperation::Info:
          {
            if (!RequireBytes (offset, 16, stream.size ()))
              {
                SetError (error, "truncated ARL INFO record");
                records.clear ();
                return false;
              }

            record.info.minSpeedKbps = ReadU16 (stream, offset);
            record.info.maxSpeedKbps = ReadU16 (stream, offset);
            record.info.minPowerDbmX10 =
              static_cast<int16_t> (ReadU16 (stream, offset));
            record.info.maxPowerDbmX10 =
              static_cast<int16_t> (ReadU16 (stream, offset));
            record.info.linkMarginDbX10 =
              static_cast<int16_t> (ReadU16 (stream, offset));
            record.info.lowPowerDbmX10 =
              static_cast<int16_t> (ReadU16 (stream, offset));
            record.info.tempLowCx10 =
              static_cast<int16_t> (ReadU16 (stream, offset));
            record.info.tempHighCx10 =
              static_cast<int16_t> (ReadU16 (stream, offset));
            break;
          }

        case CsrRoutingOperation::Delete:
          {
            if (!RequireBytes (offset, 3, stream.size ()))
              {
                SetError (error, "truncated ARL DELETE record");
                records.clear ();
                return false;
              }

            record.nodeId = ReadU24 (stream, offset);
            break;
          }

        case CsrRoutingOperation::Update:
          {
            if (!RequireBytes (offset, 10, stream.size ()))
              {
                SetError (error, "truncated ARL UPDATE fixed fields");
                records.clear ();
                return false;
              }

            record.nodeId = ReadU24 (stream, offset);
            record.capability = stream[offset++];
            record.hopCount = ReadU16 (stream, offset);
            record.cost = ReadU32 (stream, offset);

            std::size_t pathBytes =
              static_cast<std::size_t> (record.hopCount) * 3;

            if (!RequireBytes (offset, pathBytes, stream.size ()))
              {
                SetError (error, "truncated ARL UPDATE path");
                records.clear ();
                return false;
              }

            record.path.reserve (record.hopCount);
            for (uint32_t index = 0;
                 index < record.hopCount;
                 ++index)
              {
                record.path.push_back (ReadU24 (stream, offset));
              }
            break;
          }

        case CsrRoutingOperation::None:
        default:
          SetError (error, "unknown ARL routing operation");
          records.clear ();
          return false;
        }

      records.push_back (std::move (record));
    }

  return true;
}

} // namespace ns3
