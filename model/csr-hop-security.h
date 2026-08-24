#pragma once

#include "csr-wire-format.h"

#include "ns3/header.h"
#include "ns3/nstime.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <ostream>

namespace ns3 {

/**
 * Pairwise32 envelope used by the legacy ARLPktTypeKeyRequest packet.
 *
 * The key identifier and sequence number are both 12-bit values packed into
 * three bytes.  The legacy KeyRequest has no payload and appends a four-byte
 * pairwise authentication tag, for an exact seven-byte HOP-security record.
 */
class CsrKeyRequestHeader : public Header
{
public:
  static constexpr uint32_t AUTH_TAG_SIZE = 4;
  static constexpr uint32_t SERIALIZED_SIZE = 3 + AUTH_TAG_SIZE;

  using AuthTag = std::array<uint8_t, AUTH_TAG_SIZE>;

  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const override;

  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  void SetKeyId (uint16_t keyId);
  uint16_t GetKeyId () const;

  void SetSequence (uint16_t sequence);
  uint16_t GetSequence () const;

  void SetAuthTag (const AuthTag &authTag);
  const AuthTag& GetAuthTag () const;

private:
  uint16_t m_keyId {0};
  uint16_t m_sequence {0};
  AuthTag m_authTag {};
};

/**
 * Legacy ARLPktTypeKeyUpdate HOP-security record.
 *
 * Wire layout (51 bytes): pairwise key/sequence (3), reserved/group-key ID
 * (2), data PRN (6), wrapped group key (32), and key authentication tag (8).
 */
class CsrKeyUpdateHeader : public Header
{
public:
  static constexpr uint32_t DATA_PRN_SIZE = 6;
  static constexpr uint32_t WRAPPED_KEY_SIZE = 32;
  static constexpr uint32_t AUTH_TAG_SIZE = 8;
  static constexpr uint32_t SERIALIZED_SIZE =
    3 + 2 + DATA_PRN_SIZE + WRAPPED_KEY_SIZE + AUTH_TAG_SIZE;

  using DataPrn = std::array<uint8_t, DATA_PRN_SIZE>;
  using WrappedKey = std::array<uint8_t, WRAPPED_KEY_SIZE>;
  using AuthTag = std::array<uint8_t, AUTH_TAG_SIZE>;

  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const override;

  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  void SetKeyId (uint16_t keyId);
  uint16_t GetKeyId () const;

  void SetSequence (uint16_t sequence);
  uint16_t GetSequence () const;

  void SetGroupKeyId (uint16_t groupKeyId);
  uint16_t GetGroupKeyId () const;

  void SetDataPrn (const DataPrn &dataPrn);
  const DataPrn& GetDataPrn () const;

  void SetWrappedKey (const WrappedKey &wrappedKey);
  const WrappedKey& GetWrappedKey () const;

  void SetAuthTag (const AuthTag &authTag);
  const AuthTag& GetAuthTag () const;

private:
  uint16_t m_keyId {0};
  uint16_t m_sequence {0};
  uint16_t m_groupKeyId {0};
  DataPrn m_dataPrn {};
  WrappedKey m_wrappedKey {};
  AuthTag m_authTag {};
};

enum class CsrHopSecurityReceiveStatus : uint8_t
{
  Accepted,
  AcceptedSecurityCountChanged,
  DuplicateOrStale
};

/**
 * Behavioral foundation for the legacy hopSec SecurityCTX/NeighborCTX state.
 *
 * This class intentionally models key/sequence bookkeeping, replay rejection,
 * security-count resets, and the ACK-driven group-key lifecycle.  The byte
 * arrays are zero-filled, matching the legacy non-SECURITY build.  Replacing
 * those arrays with AES/HMAC output can therefore be done without changing
 * routing admission or packet lifecycle code.
 */
class CsrHopSecurityState
{
public:
  void SetNodeId (CsrNodeId nodeId);
  CsrNodeId GetNodeId () const;

  void SetOwnSecurityCount (uint16_t securityCount);
  uint16_t GetOwnSecurityCount () const;

  void SetGroupKeyId (uint16_t groupKeyId);
  uint16_t GetGroupKeyId () const;

  CsrKeyRequestHeader BuildKeyRequest (CsrNodeId destination);
  CsrKeyUpdateHeader BuildKeyUpdate (CsrNodeId destination);

  CsrHopSecurityReceiveStatus ReceiveKeyRequest (
    CsrNodeId source,
    uint16_t securityCount,
    const CsrKeyRequestHeader &header);

  CsrHopSecurityReceiveStatus ReceiveKeyUpdate (
    CsrNodeId source,
    uint16_t securityCount,
    const CsrKeyUpdateHeader &header);

  bool IsKeyUpdateSendActive (CsrNodeId neighbor) const;
  bool HasGroupKeySentTo (CsrNodeId neighbor) const;
  bool HasGroupKeyReceivedFrom (CsrNodeId neighbor) const;
  bool GetGroupKeySentWhen (CsrNodeId neighbor, Time &when) const;

  void MarkKeyUpdateAcked (CsrNodeId neighbor, Time when);
  void MarkKeyUpdateFailed (CsrNodeId neighbor);
  void ResetNeighbor (CsrNodeId neighbor);

private:
  struct NeighborState
  {
    uint16_t outboundPairwiseKeyId {0};
    uint16_t outboundPairwiseSequence {0};

    bool inboundPairwiseValid {false};
    uint16_t inboundPairwiseKeyId {0};
    uint16_t inboundPairwiseSequence {0};

    bool securityCountValid {false};
    uint16_t securityCount {0};

    bool keyUpdateSendActive {false};
    bool groupKeySent {false};
    Time groupKeySentWhen {Seconds (0.0)};

    bool groupKeyReceived {false};
    uint16_t lastGroupKeyIdReceived {0};

    bool previousGroupKeyReceived {false};
    uint16_t previousGroupKeyIdReceived {0};
  };

  NeighborState& GetOrCreateNeighbor (CsrNodeId neighbor);
  const NeighborState* FindNeighbor (CsrNodeId neighbor) const;

  void NextPairwiseKeySequence (
    NeighborState &neighbor,
    uint16_t &keyId,
    uint16_t &sequence);

  CsrHopSecurityReceiveStatus AcceptPairwiseSequence (
    NeighborState &neighbor,
    uint16_t securityCount,
    uint16_t keyId,
    uint16_t sequence);

  CsrNodeId m_nodeId {0};
  uint16_t m_ownSecurityCount {1};
  // The legacy non-SECURITY KeyUpdate path writes groupKeyId=0.
  uint16_t m_groupKeyId {0};
  std::map<CsrNodeId, NeighborState> m_neighbors;
};

} // namespace ns3
