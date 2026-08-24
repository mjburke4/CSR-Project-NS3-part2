#pragma once

#include "csr-wire-format.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace ns3 {

/**
 * Portable implementation of the cryptographic transforms used by the
 * enabled legacy ARL hop-security build.
 *
 * This class contains no operational keys and no embedded key-store access.
 * Callers supply mission, pairwise, and group-key material explicitly.
 */
class CsrLegacyCrypto
{
public:
  static constexpr std::size_t SHA1_SIZE = 20;
  static constexpr std::size_t AES_BLOCK_SIZE = 16;
  static constexpr std::size_t MISSION_KEY_SIZE = 32;
  static constexpr std::size_t PAIRWISE_KEY_MATERIAL_SIZE = 64;
  static constexpr std::size_t GROUP_KEY_MATERIAL_SIZE = 32;
  static constexpr std::size_t ENCRYPTION_KEY_SIZE = 32;
  static constexpr std::size_t AUTHENTICATION_KEY_SIZE = 20;
  static constexpr std::size_t CRYPTO_PRN_SIZE = 5;

  using Sha1Digest = std::array<uint8_t, SHA1_SIZE>;
  using AesBlock = std::array<uint8_t, AES_BLOCK_SIZE>;
  using MissionKey = std::array<uint8_t, MISSION_KEY_SIZE>;
  using PairwiseKeyMaterial =
    std::array<uint8_t, PAIRWISE_KEY_MATERIAL_SIZE>;
  using GroupKeyMaterial =
    std::array<uint8_t, GROUP_KEY_MATERIAL_SIZE>;
  using EncryptionKey = std::array<uint8_t, ENCRYPTION_KEY_SIZE>;
  using AuthenticationKey =
    std::array<uint8_t, AUTHENTICATION_KEY_SIZE>;
  using CryptoPrn = std::array<uint8_t, CRYPTO_PRN_SIZE>;

  /** Derived AES-256 and HMAC-SHA1 key pair. */
  struct ProtectionKeys
  {
    EncryptionKey encryption {};
    AuthenticationKey authentication {};
  };

  /**
   * Compute a standard SHA-1 digest.
   *
   * @param data Input bytes.
   * @return Twenty-byte digest in network byte order.
   */
  static Sha1Digest Sha1 (std::span<const uint8_t> data);

  /**
   * Compute HMAC-SHA1.
   *
   * @param key HMAC key bytes.
   * @param data Input bytes.
   * @return Twenty-byte HMAC.
   */
  static Sha1Digest HmacSha1 (std::span<const uint8_t> key,
                              std::span<const uint8_t> data);

  /**
   * Compute PBKDF2-HMAC-SHA1.
   *
   * @param password Password bytes.
   * @param salt Salt bytes.
   * @param iterations Iteration count; must be nonzero.
   * @param outputSize Requested output length.
   * @return Derived bytes.
   */
  static std::vector<uint8_t> Pbkdf2HmacSha1 (
    std::span<const uint8_t> password,
    std::span<const uint8_t> salt,
    uint32_t iterations,
    std::size_t outputSize);

  /**
   * Derive the legacy AES and HMAC protection keys.
   *
   * @param missionKey Thirty-two-byte mission key.
   * @param keyMaterial Pairwise or group-key material.
   * @param sourceId Originating node identifier.
   * @param securityCount Originating restart/security counter.
   * @param keyId Twelve-bit key identifier.
   * @return Derived protection keys.
   */
  static ProtectionKeys DeriveProtectionKeys (
    const MissionKey &missionKey,
    std::span<const uint8_t> keyMaterial,
    CsrNodeId sourceId,
    uint16_t securityCount,
    uint16_t keyId);

  /**
   * Apply the legacy one-way group-key evolution function.
   *
   * The source invokes this when its 12-bit group sequence wraps and the new
   * key identifier is not a GROUP_KEY_ID_STEP_SIZE boundary.  Receivers use
   * the same transform to catch up to a future key inside the same epoch.
   */
  static GroupKeyMaterial DeriveNextGroupKeyMaterial (
    const MissionKey &missionKey,
    const GroupKeyMaterial &currentGroupKey,
    CsrNodeId sourceId,
    uint16_t securityCount,
    uint16_t nextGroupKeyId);

  /**
   * Derive the AES-256 wrapping key used by KeyUpdate.
   *
   * @param missionKey Thirty-two-byte mission key.
   * @param pairwiseMaterial Sixty-four-byte pairwise key material.
   * @param sourceId KeyUpdate source.
   * @param securityCount Source restart/security counter.
   * @param groupKeyId Group-key identifier.
   * @param prn Five-byte cryptographic PRN.
   * @return AES-256 wrapping key.
   */
  static EncryptionKey DerivePairwiseWrappingKey (
    const MissionKey &missionKey,
    const PairwiseKeyMaterial &pairwiseMaterial,
    CsrNodeId sourceId,
    uint16_t securityCount,
    uint16_t groupKeyId,
    const CryptoPrn &prn);

  /**
   * Compute a Pairwise16/Pairwise32 authentication tag.
   *
   * @param authenticationKey Derived HMAC key.
   * @param sourceId Message source.
   * @param destinationId Message destination.
   * @param packetType Legacy ARLPktType byte.
   * @param data Authenticated security record excluding its tag.
   * @param authLength Requested truncation length.
   * @return Truncated tag bytes.
   */
  static std::vector<uint8_t> ComputePairwiseAuthTag (
    const AuthenticationKey &authenticationKey,
    CsrNodeId sourceId,
    CsrNodeId destinationId,
    uint8_t packetType,
    std::span<const uint8_t> data,
    std::size_t authLength);

  /**
   * Compute a legacy Group16 or Group32Encrypt tag.
   *
   * Group traffic uses the generic authentication input with destination ID
   * zero.  The authenticated data is the packed three-byte group key/sequence
   * field followed by the plaintext (Group16) or ciphertext (Group32Encrypt).
   */
  static std::vector<uint8_t> ComputeGroupAuthTag (
    const AuthenticationKey &authenticationKey,
    CsrNodeId sourceId,
    uint8_t packetType,
    std::span<const uint8_t> data,
    std::size_t authLength);

  /**
   * Compute the four-byte GroupEstablish/Discover tag.
   *
   * Bytes 0-1 authenticate with the first twenty bytes of the mission key;
   * bytes 2-3 authenticate with the sender's derived group authentication
   * key.  A receiver without that group key can therefore authenticate the
   * source while deferring payload decryption until KeyUpdate completes.
   */
  static std::array<uint8_t, 4> ComputeGroupEstablishAuthTag (
    const MissionKey &missionKey,
    const AuthenticationKey *groupAuthenticationKey,
    uint8_t packetType,
    CsrNodeId sourceId,
    uint16_t securityCount,
    uint16_t groupKeyId,
    uint16_t groupSequence,
    std::span<const uint8_t> encryptedPayload);

  /**
   * Compute the eight-byte KeyUpdate tag over the unwrapped group key.
   *
   * @param authenticationKey Derived pairwise HMAC key.
   * @param packetType Legacy ARLPktType byte.
   * @param sourceId KeyUpdate source.
   * @param securityCount Source restart/security counter.
   * @param keyId Pairwise key identifier.
   * @param sequence Pairwise sequence.
   * @param destinationId KeyUpdate destination.
   * @param reserved Four-bit reserved value.
   * @param groupKeyId Group-key identifier.
   * @param prn Five-byte cryptographic PRN.
   * @param groupKey Plaintext group-key material.
   * @return Eight-byte truncated HMAC.
   */
  static std::array<uint8_t, 8> ComputeKeyUpdateAuthTag (
    const AuthenticationKey &authenticationKey,
    uint8_t packetType,
    CsrNodeId sourceId,
    uint16_t securityCount,
    uint16_t keyId,
    uint16_t sequence,
    CsrNodeId destinationId,
    uint8_t reserved,
    uint16_t groupKeyId,
    const CryptoPrn &prn,
    const GroupKeyMaterial &groupKey);

  /**
   * Construct the legacy KeyUpdate CBC initialization vector.
   *
   * @param authTag Eight-byte KeyUpdate authentication tag.
   * @param prn Five-byte cryptographic PRN.
   * @param sourceId KeyUpdate source.
   * @return Sixteen-byte initialization vector.
   */
  static AesBlock BuildKeyUpdateIv (const std::array<uint8_t, 8> &authTag,
                                    const CryptoPrn &prn,
                                    CsrNodeId sourceId);

  /**
   * Encrypt two complete AES blocks using AES-256-CBC.
   *
   * @param key AES-256 key.
   * @param iv Initialization vector.
   * @param plaintext Thirty-two-byte plaintext.
   * @return Thirty-two-byte ciphertext.
   */
  static GroupKeyMaterial Aes256CbcEncrypt (
    const EncryptionKey &key,
    const AesBlock &iv,
    const GroupKeyMaterial &plaintext);

  /**
   * Decrypt two complete AES blocks using AES-256-CBC.
   *
   * @param key AES-256 key.
   * @param iv Initialization vector.
   * @param ciphertext Thirty-two-byte ciphertext.
   * @return Thirty-two-byte plaintext.
   */
  static GroupKeyMaterial Aes256CbcDecrypt (
    const EncryptionKey &key,
    const AesBlock &iv,
    const GroupKeyMaterial &ciphertext);

  /**
   * Apply the legacy AES-256-CTR transform in place.
   *
   * @param key AES-256 key.
   * @param sourceId Message source.
   * @param destinationId Message destination or zero for broadcast.
   * @param securityCount Source restart/security counter.
   * @param keyId Twelve-bit key identifier.
   * @param sequence Twelve-bit sequence.
   * @param data Bytes to encrypt or decrypt.
   */
  static void ApplyAes256Ctr (const EncryptionKey &key,
                              CsrNodeId sourceId,
                              CsrNodeId destinationId,
                              uint16_t securityCount,
                              uint16_t keyId,
                              uint16_t sequence,
                              std::span<uint8_t> data);

  /**
   * Compare two byte strings without data-dependent early exit.
   *
   * @param left First byte string.
   * @param right Second byte string.
   * @return True when lengths and all bytes match.
   */
  static bool ConstantTimeEqual (std::span<const uint8_t> left,
                                 std::span<const uint8_t> right);
};

} // namespace ns3
