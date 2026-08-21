#pragma once

#include "csr-hello-header.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ns3 {

/**
 * Encoder/decoder for the byte stream produced by the legacy ARL routes.c
 * implementation.
 *
 * A logical routing message is a sequence of operation records.  The stream
 * is split into independently reliable sections.  Every section starts with
 * the same six-byte prefix:
 *
 *   routing sequence (32 bits), section (8 bits), total sections (8 bits)
 *
 * Record bytes are allowed to cross section boundaries, matching
 * routesUpdateAddByte().
 */
class CsrArlRoutingMessage
{
public:
  static constexpr uint32_t MAX_NODE_ID = 0x00ffffffu; ///< Largest wire node ID.
  static constexpr std::size_t SECTION_PREFIX_SIZE = 6; ///< Prefix size in bytes.

  // Disassembly of the supplied 2013 routes.obj confirms that
  // RoutesHopSecPacketSize is 0x2bc (700) bytes.
  static constexpr std::size_t MAX_SECTION_SIZE = 700; ///< Maximum wire section size.
  static constexpr std::size_t MAX_SECTION_BODY_SIZE =
    MAX_SECTION_SIZE - SECTION_PREFIX_SIZE; ///< Maximum record bytes per section.

  /** Decoded six-byte section prefix and its record bytes. */
  struct Section
  {
    uint32_t sequence {0}; ///< Logical routing-message sequence.
    uint8_t section {0}; ///< Zero-based section index.
    uint8_t totalSections {0}; ///< Section count for the logical message.
    std::vector<uint8_t> body; ///< Record-stream fragment after the prefix.
  };

  /** Decoded ARL routing operation and its operation-specific fields. */
  struct Record
  {
    CsrRoutingOperation operation {CsrRoutingOperation::None}; ///< Record opcode.
    CsrHelloHeader::RoutingInfo info; ///< INFO fields.

    // ARL places node identifiers on the wire as unsigned 24-bit values.
    uint32_t nodeId {0}; ///< DELETE or UPDATE 24-bit node identifier.
    uint8_t capability {0}; ///< UPDATE routing capability.
    uint16_t hopCount {0}; ///< UPDATE 16-bit hop count.
    uint32_t cost {0}; ///< UPDATE 32-bit route cost.
    std::vector<uint32_t> path; ///< UPDATE 24-bit path identifiers.
  };

  /** Builds one logical ARL record stream before sectioning it. */
  class Builder
  {
  public:
    /** Append a FLUSH record. */
    void AddFlush ();

    /** Append a REQUEST record. */
    void AddRequest ();

    /**
     * Append an INFO record.
     *
     * @param info Legacy link and operating-limit fields.
     */
    void AddInfo (const CsrHelloHeader::RoutingInfo &info);

    /**
     * Append a DELETE record.
     *
     * @param nodeId Unsigned 24-bit destination identifier.
     * @param error Optional validation error text.
     * @return True when the record was appended.
     */
    bool AddDelete (uint32_t nodeId,
                    std::string *error = nullptr);

    /**
     * Append an UPDATE record.
     *
     * @param nodeId Unsigned 24-bit destination identifier.
     * @param capability Routing capability byte.
     * @param hopCount Unsigned 16-bit hop count.
     * @param cost Unsigned 32-bit route cost.
     * @param path Exactly hopCount unsigned 24-bit node identifiers.
     * @param error Optional validation error text.
     * @return True when the record was appended.
     */
    bool AddUpdate (uint32_t nodeId,
                    uint8_t capability,
                    uint16_t hopCount,
                    uint32_t cost,
                    const std::vector<uint32_t> &path,
                    std::string *error = nullptr);

    /** @return The unsectioned logical record stream. */
    const std::vector<uint8_t> &GetRecordStream () const;

    /**
     * Split the record stream into legacy six-byte-prefixed sections.
     *
     * @param sequence Logical routing-message sequence.
     * @param sections Encoded sections in ascending index order.
     * @param error Optional validation error text.
     * @return True when at least one section was produced.
     */
    bool BuildSections (
      uint32_t sequence,
      std::vector<std::vector<uint8_t>> &sections,
      std::string *error = nullptr) const;

  private:
    std::vector<uint8_t> m_stream; ///< Logical operation-record byte stream.
  };

  /**
   * Decode and validate one legacy ARL section.
   *
   * @param bytes Encoded section bytes.
   * @param length Number of encoded bytes.
   * @param section Decoded prefix and body.
   * @param error Optional validation error text.
   * @return True when the section is valid.
   */
  static bool DecodeSection (
    const uint8_t *bytes,
    std::size_t length,
    Section &section,
    std::string *error = nullptr);

  /**
   * Decode and validate one legacy ARL section.
   *
   * @param bytes Encoded section bytes.
   * @param section Decoded prefix and body.
   * @param error Optional validation error text.
   * @return True when the section is valid.
   */
  static bool DecodeSection (
    const std::vector<uint8_t> &bytes,
    Section &section,
    std::string *error = nullptr);

  /**
   * Parse a completely reassembled operation-record stream.
   *
   * @param stream Reassembled record bytes with section prefixes removed.
   * @param records Parsed records in wire order.
   * @param error Optional validation error text.
   * @return True when the complete stream is valid.
   */
  static bool ParseRecordStream (
    const std::vector<uint8_t> &stream,
    std::vector<Record> &records,
    std::string *error = nullptr);
};

} // namespace ns3
