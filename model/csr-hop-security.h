#pragma once

#include "csr-legacy-crypto.h"
#include "csr-wire-format.h"

#include "ns3/header.h"
#include "ns3/nstime.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <ostream>
#include <span>
#include <vector>

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
  AuthenticatedDuplicate,
  DuplicateOrStale,
  AuthenticationFailed,
  AuthenticatedGroupKeyNeeded,
  AuthenticatedGroupKeyNeededSecurityCountChanged,
  GroupKeyUnavailable,
  Malformed
};

/** Security modes that use the sender's rotating group-key stream. */
enum class CsrGroupSecurityMode : uint8_t
{
  GroupEstablish,
  Group16,
  Group32Encrypt
};

/** Complete legacy HOP-security record produced for one group message. */
struct CsrProtectedGroupMessage
{
  uint16_t groupKeyId {0};
  uint16_t groupSequence {0};
  bool generatedNewGroupKey {false};
  std::vector<uint8_t> record;
};

/** Result of authenticating and, when applicable, decrypting a group record. */
struct CsrReceivedGroupMessage
{
  CsrHopSecurityReceiveStatus status {
    CsrHopSecurityReceiveStatus::Malformed
  };
  uint16_t groupKeyId {0};
  uint16_t groupSequence {0};
  std::vector<uint8_t> payload;
};

/**
 * Enabled legacy hopSec SecurityCTX/NeighborCTX behavior.
 *
 * The implementation uses the target build's PBKDF2-HMAC-SHA1 derivation,
 * HMAC authentication, AES-256-CBC group-key wrapping, pairwise replay state,
 * security-count resets, and ACK-driven group-key lifecycle. Embedded flash,
 * key-manager, and hardware-randomness functions are replaced by injectable
 * key material and deterministic simulation PRNs.
 */
class CsrHopSecurityState
{
public:
  using MissionKey = CsrLegacyCrypto::MissionKey;
  using PairwiseKeyMaterial = CsrLegacyCrypto::PairwiseKeyMaterial;
  using GroupKeyMaterial = CsrLegacyCrypto::GroupKeyMaterial;
  using CryptoPrn = CsrLegacyCrypto::CryptoPrn;

  CsrHopSecurityState ();

  void SetNodeId (CsrNodeId nodeId);
  CsrNodeId GetNodeId () const;

  void SetOwnSecurityCount (uint16_t securityCount);
  uint16_t GetOwnSecurityCount () const;

  void SetGroupKeyId (uint16_t groupKeyId);
  uint16_t GetGroupKeyId () const;

  void SetGroupSequence (uint16_t groupSequence);
  uint16_t GetGroupSequence () const;

  /** Set the synthetic or externally provisioned mission key. */
  void SetMissionKey (const MissionKey &missionKey);

  /** Set pairwise material used when a neighbor-specific value is absent. */
  void SetDefaultPairwiseKeyMaterial (
    const PairwiseKeyMaterial &pairwiseMaterial);

  /** Set pairwise material for one neighbor. */
  void SetPairwiseKeyMaterial (
    CsrNodeId neighbor,
    const PairwiseKeyMaterial &pairwiseMaterial);

  /** Set this node's outbound group-key material. */
  void SetGroupKeyMaterial (const GroupKeyMaterial &groupKeyMaterial);

  /** Return this node's outbound group-key material. */
  const GroupKeyMaterial& GetGroupKeyMaterial () const;

  /** Override the next newly generated epoch key for deterministic tests. */
  void SetNextGeneratedGroupKeyMaterial (
    const GroupKeyMaterial &groupKeyMaterial);

  /**
   * Return the current authenticated group key received from a neighbor.
   *
   * @param neighbor Neighbor identifier.
   * @param groupKey Destination for the recovered key.
   * @return True when an authenticated group key is available.
   */
  bool GetReceivedGroupKeyMaterial (
    CsrNodeId neighbor,
    GroupKeyMaterial &groupKey) const;

  /**
   * Override the five cryptographic PRN bytes for the next KeyUpdate.
   *
   * The sixth wire byte remains reserved and is serialized as zero.
   */
  void SetNextDataPrn (const CryptoPrn &prn);

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

  /**
   * Protect one GroupEstablish, Group16, or Group32Encrypt payload.
   *
   * The returned record is byte-for-byte in legacy order: packed 12-bit
   * group key/sequence, payload, then the mode-specific authentication tag.
   */
  CsrProtectedGroupMessage ProtectGroupMessage (
    CsrGroupSecurityMode mode,
    uint8_t legacyPacketType,
    std::span<const uint8_t> payload);

  /** Authenticate and optionally decrypt one complete legacy group record. */
  CsrReceivedGroupMessage ReceiveGroupMessage (
    CsrNodeId source,
    uint16_t securityCount,
    CsrGroupSecurityMode mode,
    uint8_t legacyPacketType,
    std::span<const uint8_t> record);

  bool IsKeyUpdateSendActive (CsrNodeId neighbor) const;
  bool HasGroupKeySentTo (CsrNodeId neighbor) const;
  bool HasGroupKeyReceivedFrom (CsrNodeId neighbor) const;
  bool GetGroupKeySentWhen (CsrNodeId neighbor, Time &when) const;

  void MarkKeyUpdateAcked (CsrNodeId neighbor, Time when);
  void MarkKeyUpdateFailed (CsrNodeId neighbor);
  void ResetNeighbor (CsrNodeId neighbor);

private:
  struct ReplayWindow
  {
    static constexpr uint32_t WINDOW_SIZE = 128;

    bool IsBeforeWindow (uint32_t value) const;
    bool IsDuplicate (uint32_t value) const;
    void MarkReceived (uint32_t value);

    uint32_t leftEdge {0};
    std::array<bool, WINDOW_SIZE> received {};
  };

  struct NeighborState
  {
    uint16_t outboundPairwiseKeyId {1};
    uint16_t outboundPairwiseSequence {1};

    ReplayWindow inboundPairwise;

    bool securityCountValid {false};
    uint16_t securityCount {0};

    ReplayWindow inboundGroup;

    bool keyUpdateSendActive {false};
    bool groupKeySent {false};
    Time groupKeySentWhen {Seconds (0.0)};

    bool groupKeyReceived {false};
    uint16_t lastGroupKeyIdReceived {0};
    GroupKeyMaterial groupKeyMaterial {};

    bool previousGroupKeyReceived {false};
    uint16_t previousGroupKeyIdReceived {0};
    GroupKeyMaterial previousGroupKeyMaterial {};
  };

  NeighborState& GetOrCreateNeighbor (CsrNodeId neighbor);
  const NeighborState* FindNeighbor (CsrNodeId neighbor) const;

  void NextPairwiseKeySequence (
    NeighborState &neighbor,
    uint16_t &keyId,
    uint16_t &sequence);

  bool NextGroupKeySequence ();
  GroupKeyMaterial GenerateNewGroupKeyMaterial ();

  CsrHopSecurityReceiveStatus CommitAuthenticatedPairwise (
    NeighborState &neighbor,
    uint16_t securityCount,
    uint16_t keyId,
    uint16_t sequence,
    bool securityCountChanged);

  bool IsSecurityCountChangeValid (uint16_t current,
                                   uint16_t received) const;
  void ResetInboundForSecurityCountChange (NeighborState &neighbor);

  const PairwiseKeyMaterial& GetPairwiseKeyMaterial (
    CsrNodeId neighbor) const;
  CsrLegacyCrypto::ProtectionKeys DerivePairwiseProtectionKeys (
    CsrNodeId peer,
    CsrNodeId source,
    uint16_t securityCount,
    uint16_t keyId) const;
  CryptoPrn GenerateDataPrn ();

  CsrNodeId m_nodeId {0};
  uint16_t m_ownSecurityCount {1};
  uint16_t m_groupKeyId {1};
  uint16_t m_groupSequence {1};
  MissionKey m_missionKey {};
  PairwiseKeyMaterial m_defaultPairwiseKeyMaterial {};
  GroupKeyMaterial m_groupKeyMaterial {};
  std::map<CsrNodeId, PairwiseKeyMaterial> m_pairwiseKeyMaterial;
  std::optional<CryptoPrn> m_nextDataPrn;
  std::optional<GroupKeyMaterial> m_nextGeneratedGroupKeyMaterial;
  uint64_t m_prnCounter {0};
  uint64_t m_groupKeyGenerationCounter {0};
  std::map<CsrNodeId, NeighborState> m_neighbors;
};

} // namespace ns3
