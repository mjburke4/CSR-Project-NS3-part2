#include "csr-legacy-crypto.h"

#include "ns3/abort.h"

#include <algorithm>
#include <array>
#include <limits>

namespace ns3 {

namespace
{

constexpr uint16_t SECURITY_FIELD_MAX = 0x0fff;
constexpr std::size_t AES_EXPANDED_KEY_SIZE = 240;

constexpr std::array<uint8_t, 256> AES_SBOX {
  0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5,
  0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
  0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
  0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
  0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc,
  0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
  0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a,
  0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
  0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
  0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
  0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b,
  0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
  0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85,
  0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
  0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
  0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
  0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17,
  0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
  0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88,
  0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
  0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
  0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
  0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9,
  0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
  0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6,
  0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
  0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
  0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
  0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94,
  0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
  0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68,
  0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};

constexpr std::array<uint8_t, 256> AES_INV_SBOX {
  0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38,
  0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
  0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87,
  0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
  0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d,
  0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
  0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2,
  0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
  0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16,
  0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
  0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda,
  0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
  0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a,
  0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
  0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02,
  0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
  0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea,
  0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
  0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85,
  0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
  0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89,
  0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
  0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20,
  0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
  0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31,
  0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
  0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d,
  0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
  0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0,
  0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
  0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26,
  0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d
};

constexpr std::array<uint8_t, 15> AES_RCON {
  0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40,
  0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d
};

uint32_t
RotateLeft (uint32_t value, uint32_t count)
{
  return (value << count) | (value >> (32 - count));
}

void
AppendNodeId (std::vector<uint8_t> &bytes, CsrNodeId nodeId)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (nodeId),
                   "CSR crypto node identifier exceeds 24 bits");
  bytes.push_back (static_cast<uint8_t> (nodeId >> 16));
  bytes.push_back (static_cast<uint8_t> (nodeId >> 8));
  bytes.push_back (static_cast<uint8_t> (nodeId));
}

void
AppendBe16 (std::vector<uint8_t> &bytes, uint16_t value)
{
  bytes.push_back (static_cast<uint8_t> (value >> 8));
  bytes.push_back (static_cast<uint8_t> (value));
}

void
AppendAuthPackedKeySequence (std::vector<uint8_t> &bytes,
                             uint16_t keyId,
                             uint16_t sequence)
{
  bytes.push_back (static_cast<uint8_t> (keyId));
  bytes.push_back (static_cast<uint8_t> (((keyId >> 8) & 0x0f) |
                                         ((sequence << 4) & 0xf0)));
  bytes.push_back (static_cast<uint8_t> (sequence >> 4));
}

void
RequireSecurityField (uint16_t value, const char *field)
{
  NS_ABORT_MSG_IF (value > SECURITY_FIELD_MAX,
                   field << " exceeds the legacy 12-bit field");
}

uint8_t
AesXtime (uint8_t value)
{
  return static_cast<uint8_t> ((value << 1) ^
                               ((value & 0x80) ? 0x1b : 0x00));
}

uint8_t
AesMultiply (uint8_t left, uint8_t right)
{
  uint8_t result = 0;
  while (right != 0)
    {
      if ((right & 1) != 0)
        {
          result ^= left;
        }
      left = AesXtime (left);
      right >>= 1;
    }
  return result;
}

using ExpandedAesKey = std::array<uint8_t, AES_EXPANDED_KEY_SIZE>;

ExpandedAesKey
ExpandAes256Key (const CsrLegacyCrypto::EncryptionKey &key)
{
  ExpandedAesKey expanded {};
  std::copy (key.begin (), key.end (), expanded.begin ());

  std::size_t generated = key.size ();
  uint8_t rconIndex = 1;
  std::array<uint8_t, 4> temp {};

  while (generated < expanded.size ())
    {
      std::copy_n (expanded.begin () + generated - 4,
                   4,
                   temp.begin ());

      if ((generated % key.size ()) == 0)
        {
          std::rotate (temp.begin (), temp.begin () + 1, temp.end ());
          for (uint8_t &byte : temp)
            {
              byte = AES_SBOX[byte];
            }
          temp[0] ^= AES_RCON[rconIndex++];
        }
      else if ((generated % key.size ()) == 16)
        {
          for (uint8_t &byte : temp)
            {
              byte = AES_SBOX[byte];
            }
        }

      for (uint8_t byte : temp)
        {
          expanded[generated] =
            static_cast<uint8_t> (expanded[generated - key.size ()] ^ byte);
          generated++;
        }
    }

  return expanded;
}

void
AddAesRoundKey (CsrLegacyCrypto::AesBlock &state,
                const ExpandedAesKey &expanded,
                uint8_t round)
{
  std::size_t offset = static_cast<std::size_t> (round) * state.size ();
  for (std::size_t i = 0; i < state.size (); ++i)
    {
      state[i] ^= expanded[offset + i];
    }
}

void
SubAesBytes (CsrLegacyCrypto::AesBlock &state)
{
  for (uint8_t &byte : state)
    {
      byte = AES_SBOX[byte];
    }
}

void
InvSubAesBytes (CsrLegacyCrypto::AesBlock &state)
{
  for (uint8_t &byte : state)
    {
      byte = AES_INV_SBOX[byte];
    }
}

void
ShiftAesRows (CsrLegacyCrypto::AesBlock &state)
{
  CsrLegacyCrypto::AesBlock original = state;
  for (std::size_t column = 0; column < 4; ++column)
    {
      for (std::size_t row = 0; row < 4; ++row)
        {
          state[4 * column + row] =
            original[4 * ((column + row) % 4) + row];
        }
    }
}

void
InvShiftAesRows (CsrLegacyCrypto::AesBlock &state)
{
  CsrLegacyCrypto::AesBlock original = state;
  for (std::size_t column = 0; column < 4; ++column)
    {
      for (std::size_t row = 0; row < 4; ++row)
        {
          state[4 * column + row] =
            original[4 * ((column + 4 - row) % 4) + row];
        }
    }
}

void
MixAesColumns (CsrLegacyCrypto::AesBlock &state)
{
  for (std::size_t column = 0; column < 4; ++column)
    {
      std::size_t offset = 4 * column;
      uint8_t a = state[offset];
      uint8_t b = state[offset + 1];
      uint8_t c = state[offset + 2];
      uint8_t d = state[offset + 3];

      state[offset] = static_cast<uint8_t> (AesXtime (a) ^
                                            (AesXtime (b) ^ b) ^ c ^ d);
      state[offset + 1] = static_cast<uint8_t> (a ^ AesXtime (b) ^
                                                (AesXtime (c) ^ c) ^ d);
      state[offset + 2] = static_cast<uint8_t> (a ^ b ^ AesXtime (c) ^
                                                (AesXtime (d) ^ d));
      state[offset + 3] = static_cast<uint8_t> ((AesXtime (a) ^ a) ^ b ^ c ^
                                                AesXtime (d));
    }
}

void
InvMixAesColumns (CsrLegacyCrypto::AesBlock &state)
{
  for (std::size_t column = 0; column < 4; ++column)
    {
      std::size_t offset = 4 * column;
      uint8_t a = state[offset];
      uint8_t b = state[offset + 1];
      uint8_t c = state[offset + 2];
      uint8_t d = state[offset + 3];

      state[offset] = static_cast<uint8_t> (AesMultiply (a, 14) ^
                                            AesMultiply (b, 11) ^
                                            AesMultiply (c, 13) ^
                                            AesMultiply (d, 9));
      state[offset + 1] = static_cast<uint8_t> (AesMultiply (a, 9) ^
                                                AesMultiply (b, 14) ^
                                                AesMultiply (c, 11) ^
                                                AesMultiply (d, 13));
      state[offset + 2] = static_cast<uint8_t> (AesMultiply (a, 13) ^
                                                AesMultiply (b, 9) ^
                                                AesMultiply (c, 14) ^
                                                AesMultiply (d, 11));
      state[offset + 3] = static_cast<uint8_t> (AesMultiply (a, 11) ^
                                                AesMultiply (b, 13) ^
                                                AesMultiply (c, 9) ^
                                                AesMultiply (d, 14));
    }
}

CsrLegacyCrypto::AesBlock
EncryptAesBlock (const CsrLegacyCrypto::AesBlock &plaintext,
                 const ExpandedAesKey &expanded)
{
  CsrLegacyCrypto::AesBlock state = plaintext;
  AddAesRoundKey (state, expanded, 0);

  for (uint8_t round = 1; round < 14; ++round)
    {
      SubAesBytes (state);
      ShiftAesRows (state);
      MixAesColumns (state);
      AddAesRoundKey (state, expanded, round);
    }

  SubAesBytes (state);
  ShiftAesRows (state);
  AddAesRoundKey (state, expanded, 14);
  return state;
}

CsrLegacyCrypto::AesBlock
DecryptAesBlock (const CsrLegacyCrypto::AesBlock &ciphertext,
                 const ExpandedAesKey &expanded)
{
  CsrLegacyCrypto::AesBlock state = ciphertext;
  AddAesRoundKey (state, expanded, 14);

  for (uint8_t round = 13; round > 0; --round)
    {
      InvShiftAesRows (state);
      InvSubAesBytes (state);
      AddAesRoundKey (state, expanded, round);
      InvMixAesColumns (state);
    }

  InvShiftAesRows (state);
  InvSubAesBytes (state);
  AddAesRoundKey (state, expanded, 0);
  return state;
}

void
IncrementAesCounter (CsrLegacyCrypto::AesBlock &counter)
{
  for (auto it = counter.rbegin (); it != counter.rend (); ++it)
    {
      (*it)++;
      if (*it != 0)
        {
          break;
        }
    }
}

} // namespace

CsrLegacyCrypto::Sha1Digest
CsrLegacyCrypto::Sha1 (std::span<const uint8_t> data)
{
  uint64_t bitLength = static_cast<uint64_t> (data.size ()) * 8;
  std::vector<uint8_t> padded (data.begin (), data.end ());
  padded.push_back (0x80);
  while ((padded.size () % 64) != 56)
    {
      padded.push_back (0);
    }
  for (int shift = 56; shift >= 0; shift -= 8)
    {
      padded.push_back (static_cast<uint8_t> (bitLength >> shift));
    }

  uint32_t h0 = 0x67452301;
  uint32_t h1 = 0xefcdab89;
  uint32_t h2 = 0x98badcfe;
  uint32_t h3 = 0x10325476;
  uint32_t h4 = 0xc3d2e1f0;

  for (std::size_t offset = 0; offset < padded.size (); offset += 64)
    {
      std::array<uint32_t, 80> words {};
      for (std::size_t i = 0; i < 16; ++i)
        {
          std::size_t wordOffset = offset + 4 * i;
          words[i] = (static_cast<uint32_t> (padded[wordOffset]) << 24) |
                     (static_cast<uint32_t> (padded[wordOffset + 1]) << 16) |
                     (static_cast<uint32_t> (padded[wordOffset + 2]) << 8) |
                     static_cast<uint32_t> (padded[wordOffset + 3]);
        }
      for (std::size_t i = 16; i < words.size (); ++i)
        {
          words[i] = RotateLeft (words[i - 3] ^ words[i - 8] ^
                                 words[i - 14] ^ words[i - 16],
                                 1);
        }

      uint32_t a = h0;
      uint32_t b = h1;
      uint32_t c = h2;
      uint32_t d = h3;
      uint32_t e = h4;

      for (std::size_t i = 0; i < words.size (); ++i)
        {
          uint32_t f = 0;
          uint32_t k = 0;
          if (i < 20)
            {
              f = (b & c) | ((~b) & d);
              k = 0x5a827999;
            }
          else if (i < 40)
            {
              f = b ^ c ^ d;
              k = 0x6ed9eba1;
            }
          else if (i < 60)
            {
              f = (b & c) | (b & d) | (c & d);
              k = 0x8f1bbcdc;
            }
          else
            {
              f = b ^ c ^ d;
              k = 0xca62c1d6;
            }

          uint32_t next = RotateLeft (a, 5) + f + e + k + words[i];
          e = d;
          d = c;
          c = RotateLeft (b, 30);
          b = a;
          a = next;
        }

      h0 += a;
      h1 += b;
      h2 += c;
      h3 += d;
      h4 += e;
    }

  Sha1Digest digest {};
  const std::array<uint32_t, 5> words {h0, h1, h2, h3, h4};
  for (std::size_t i = 0; i < words.size (); ++i)
    {
      digest[4 * i] = static_cast<uint8_t> (words[i] >> 24);
      digest[4 * i + 1] = static_cast<uint8_t> (words[i] >> 16);
      digest[4 * i + 2] = static_cast<uint8_t> (words[i] >> 8);
      digest[4 * i + 3] = static_cast<uint8_t> (words[i]);
    }
  return digest;
}

CsrLegacyCrypto::Sha1Digest
CsrLegacyCrypto::HmacSha1 (std::span<const uint8_t> key,
                           std::span<const uint8_t> data)
{
  constexpr std::size_t blockSize = 64;
  std::array<uint8_t, blockSize> normalizedKey {};

  if (key.size () > blockSize)
    {
      Sha1Digest hashed = Sha1 (key);
      std::copy (hashed.begin (), hashed.end (), normalizedKey.begin ());
    }
  else
    {
      std::copy (key.begin (), key.end (), normalizedKey.begin ());
    }

  std::array<uint8_t, blockSize> innerPad {};
  std::array<uint8_t, blockSize> outerPad {};
  for (std::size_t i = 0; i < blockSize; ++i)
    {
      innerPad[i] = static_cast<uint8_t> (normalizedKey[i] ^ 0x36);
      outerPad[i] = static_cast<uint8_t> (normalizedKey[i] ^ 0x5c);
    }

  std::vector<uint8_t> inner (innerPad.begin (), innerPad.end ());
  inner.insert (inner.end (), data.begin (), data.end ());
  Sha1Digest innerDigest = Sha1 (inner);

  std::vector<uint8_t> outer (outerPad.begin (), outerPad.end ());
  outer.insert (outer.end (), innerDigest.begin (), innerDigest.end ());
  return Sha1 (outer);
}

std::vector<uint8_t>
CsrLegacyCrypto::Pbkdf2HmacSha1 (std::span<const uint8_t> password,
                                 std::span<const uint8_t> salt,
                                 uint32_t iterations,
                                 std::size_t outputSize)
{
  NS_ABORT_MSG_IF (iterations == 0,
                   "CSR PBKDF2 iteration count must be nonzero");
  NS_ABORT_MSG_IF (outputSize >
                     static_cast<std::size_t> (
                       std::numeric_limits<uint32_t>::max ()) * SHA1_SIZE,
                   "CSR PBKDF2 output is too large");

  std::vector<uint8_t> output;
  output.reserve (outputSize);
  uint32_t blockIndex = 1;

  while (output.size () < outputSize)
    {
      std::vector<uint8_t> blockSalt (salt.begin (), salt.end ());
      blockSalt.push_back (static_cast<uint8_t> (blockIndex >> 24));
      blockSalt.push_back (static_cast<uint8_t> (blockIndex >> 16));
      blockSalt.push_back (static_cast<uint8_t> (blockIndex >> 8));
      blockSalt.push_back (static_cast<uint8_t> (blockIndex));

      Sha1Digest u = HmacSha1 (password, blockSalt);
      Sha1Digest result = u;
      for (uint32_t iteration = 1; iteration < iterations; ++iteration)
        {
          u = HmacSha1 (password, u);
          for (std::size_t i = 0; i < result.size (); ++i)
            {
              result[i] ^= u[i];
            }
        }

      std::size_t remaining = outputSize - output.size ();
      std::size_t copySize = std::min (remaining, result.size ());
      output.insert (output.end (), result.begin (), result.begin () + copySize);
      blockIndex++;
    }

  return output;
}

CsrLegacyCrypto::ProtectionKeys
CsrLegacyCrypto::DeriveProtectionKeys (
  const MissionKey &missionKey,
  std::span<const uint8_t> keyMaterial,
  CsrNodeId sourceId,
  uint16_t securityCount,
  uint16_t keyId)
{
  RequireSecurityField (keyId, "CSR KDF key identifier");
  NS_ABORT_MSG_IF (keyMaterial.empty () ||
                     keyMaterial.size () > PAIRWISE_KEY_MATERIAL_SIZE,
                   "CSR KDF key material has an invalid size");

  std::vector<uint8_t> salt;
  salt.reserve (7);
  AppendNodeId (salt, sourceId);
  AppendBe16 (salt, securityCount);
  AppendBe16 (salt, static_cast<uint16_t> (keyId & SECURITY_FIELD_MAX));

  std::vector<uint8_t> password (missionKey.begin (), missionKey.end ());
  password.insert (password.end (), keyMaterial.begin (), keyMaterial.end ());
  password.insert (password.end (), 32, 0xa5);

  std::vector<uint8_t> encryption =
    Pbkdf2HmacSha1 (password, salt, 1, ENCRYPTION_KEY_SIZE);

  std::fill (password.end () - 32, password.end (), 0x5a);
  std::vector<uint8_t> authentication =
    Pbkdf2HmacSha1 (password, salt, 1, AUTHENTICATION_KEY_SIZE);

  ProtectionKeys keys;
  std::copy (encryption.begin (), encryption.end (), keys.encryption.begin ());
  std::copy (authentication.begin (),
             authentication.end (),
             keys.authentication.begin ());
  std::fill (password.begin (), password.end (), 0);
  std::fill (encryption.begin (), encryption.end (), 0);
  std::fill (authentication.begin (), authentication.end (), 0);
  return keys;
}

CsrLegacyCrypto::EncryptionKey
CsrLegacyCrypto::DerivePairwiseWrappingKey (
  const MissionKey &missionKey,
  const PairwiseKeyMaterial &pairwiseMaterial,
  CsrNodeId sourceId,
  uint16_t securityCount,
  uint16_t groupKeyId,
  const CryptoPrn &prn)
{
  RequireSecurityField (groupKeyId,
                        "CSR wrapping group-key identifier");

  std::vector<uint8_t> salt;
  salt.reserve (12);
  AppendNodeId (salt, sourceId);
  AppendBe16 (salt, securityCount);
  AppendBe16 (salt, static_cast<uint16_t> (groupKeyId &
                                           SECURITY_FIELD_MAX));
  salt.insert (salt.end (), prn.begin (), prn.end ());

  std::vector<uint8_t> password (missionKey.begin (), missionKey.end ());
  password.insert (password.end (),
                   pairwiseMaterial.begin (),
                   pairwiseMaterial.end ());

  std::vector<uint8_t> derived =
    Pbkdf2HmacSha1 (password, salt, 1, ENCRYPTION_KEY_SIZE);
  EncryptionKey key {};
  std::copy (derived.begin (), derived.end (), key.begin ());
  std::fill (password.begin (), password.end (), 0);
  std::fill (derived.begin (), derived.end (), 0);
  return key;
}

std::vector<uint8_t>
CsrLegacyCrypto::ComputePairwiseAuthTag (
  const AuthenticationKey &authenticationKey,
  CsrNodeId sourceId,
  CsrNodeId destinationId,
  uint8_t packetType,
  std::span<const uint8_t> data,
  std::size_t authLength)
{
  NS_ABORT_MSG_IF (data.size () > std::numeric_limits<uint16_t>::max (),
                   "CSR authenticated record exceeds 16-bit length");
  NS_ABORT_MSG_IF (authLength > SHA1_SIZE,
                   "CSR requested authentication tag is too large");

  std::vector<uint8_t> input;
  input.reserve (9 + data.size ());
  AppendNodeId (input, sourceId);
  AppendNodeId (input, destinationId);
  input.push_back (packetType);
  AppendBe16 (input, static_cast<uint16_t> (data.size ()));
  input.insert (input.end (), data.begin (), data.end ());

  Sha1Digest digest = HmacSha1 (authenticationKey, input);
  return std::vector<uint8_t> (digest.begin (),
                               digest.begin () + authLength);
}

std::array<uint8_t, 8>
CsrLegacyCrypto::ComputeKeyUpdateAuthTag (
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
  const GroupKeyMaterial &groupKey)
{
  RequireSecurityField (keyId, "CSR KeyUpdate pairwise key identifier");
  RequireSecurityField (sequence, "CSR KeyUpdate pairwise sequence");
  RequireSecurityField (groupKeyId,
                        "CSR KeyUpdate group-key identifier");
  NS_ABORT_MSG_IF (reserved > 0x0f,
                   "CSR KeyUpdate reserved field exceeds four bits");

  std::vector<uint8_t> input;
  input.reserve (1 + 3 + 2 + 3 + 3 + 2 + CRYPTO_PRN_SIZE +
                 GROUP_KEY_MATERIAL_SIZE);
  input.push_back (packetType);
  AppendNodeId (input, sourceId);
  AppendBe16 (input, securityCount);
  AppendAuthPackedKeySequence (input, keyId, sequence);
  AppendNodeId (input, destinationId);
  input.push_back (static_cast<uint8_t> (groupKeyId));
  input.push_back (static_cast<uint8_t> (((groupKeyId >> 8) & 0x0f) |
                                         (reserved << 4)));
  input.insert (input.end (), prn.begin (), prn.end ());
  input.insert (input.end (), groupKey.begin (), groupKey.end ());

  Sha1Digest digest = HmacSha1 (authenticationKey, input);
  std::array<uint8_t, 8> tag {};
  std::copy_n (digest.begin (), tag.size (), tag.begin ());
  return tag;
}

CsrLegacyCrypto::AesBlock
CsrLegacyCrypto::BuildKeyUpdateIv (const std::array<uint8_t, 8> &authTag,
                                   const CryptoPrn &prn,
                                   CsrNodeId sourceId)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (sourceId),
                   "CSR KeyUpdate IV source exceeds 24 bits");
  AesBlock iv {};
  std::copy (authTag.begin (), authTag.end (), iv.begin ());
  std::copy (prn.begin (), prn.end (), iv.begin () + authTag.size ());
  iv[13] = static_cast<uint8_t> (sourceId >> 16);
  iv[14] = static_cast<uint8_t> (sourceId >> 8);
  iv[15] = static_cast<uint8_t> (sourceId);
  return iv;
}

CsrLegacyCrypto::GroupKeyMaterial
CsrLegacyCrypto::Aes256CbcEncrypt (const EncryptionKey &key,
                                   const AesBlock &iv,
                                   const GroupKeyMaterial &plaintext)
{
  ExpandedAesKey expanded = ExpandAes256Key (key);
  GroupKeyMaterial ciphertext {};
  AesBlock previous = iv;

  for (std::size_t offset = 0; offset < plaintext.size ();
       offset += AES_BLOCK_SIZE)
    {
      AesBlock block {};
      for (std::size_t i = 0; i < block.size (); ++i)
        {
          block[i] = static_cast<uint8_t> (plaintext[offset + i] ^
                                           previous[i]);
        }
      block = EncryptAesBlock (block, expanded);
      std::copy (block.begin (), block.end (), ciphertext.begin () + offset);
      previous = block;
    }
  std::fill (expanded.begin (), expanded.end (), 0);
  return ciphertext;
}

CsrLegacyCrypto::GroupKeyMaterial
CsrLegacyCrypto::Aes256CbcDecrypt (const EncryptionKey &key,
                                   const AesBlock &iv,
                                   const GroupKeyMaterial &ciphertext)
{
  ExpandedAesKey expanded = ExpandAes256Key (key);
  GroupKeyMaterial plaintext {};
  AesBlock previous = iv;

  for (std::size_t offset = 0; offset < ciphertext.size ();
       offset += AES_BLOCK_SIZE)
    {
      AesBlock block {};
      std::copy_n (ciphertext.begin () + offset,
                   block.size (),
                   block.begin ());
      AesBlock decrypted = DecryptAesBlock (block, expanded);
      for (std::size_t i = 0; i < block.size (); ++i)
        {
          plaintext[offset + i] =
            static_cast<uint8_t> (decrypted[i] ^ previous[i]);
        }
      previous = block;
    }
  std::fill (expanded.begin (), expanded.end (), 0);
  return plaintext;
}

void
CsrLegacyCrypto::ApplyAes256Ctr (const EncryptionKey &key,
                                 CsrNodeId sourceId,
                                 CsrNodeId destinationId,
                                 uint16_t securityCount,
                                 uint16_t keyId,
                                 uint16_t sequence,
                                 std::span<uint8_t> data)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (sourceId) ||
                   !CsrIsValidNodeId (destinationId),
                   "CSR CTR node identifier exceeds 24 bits");
  RequireSecurityField (keyId, "CSR CTR key identifier");
  RequireSecurityField (sequence, "CSR CTR sequence");

  AesBlock counter {};
  counter[0] = static_cast<uint8_t> (sourceId >> 16);
  counter[1] = static_cast<uint8_t> (sourceId >> 8);
  counter[2] = static_cast<uint8_t> (sourceId);
  counter[3] = static_cast<uint8_t> (destinationId >> 16);
  counter[4] = static_cast<uint8_t> (destinationId >> 8);
  counter[5] = static_cast<uint8_t> (destinationId);
  counter[6] = static_cast<uint8_t> (securityCount >> 8);
  counter[7] = static_cast<uint8_t> (securityCount);

  std::vector<uint8_t> packed;
  packed.reserve (3);
  AppendAuthPackedKeySequence (packed, keyId, sequence);
  std::copy (packed.begin (), packed.end (), counter.begin () + 8);

  ExpandedAesKey expanded = ExpandAes256Key (key);
  for (std::size_t offset = 0; offset < data.size ();
       offset += AES_BLOCK_SIZE)
    {
      AesBlock stream = EncryptAesBlock (counter, expanded);
      std::size_t blockSize =
        std::min (AES_BLOCK_SIZE, data.size () - offset);
      for (std::size_t i = 0; i < blockSize; ++i)
        {
          data[offset + i] ^= stream[i];
        }
      IncrementAesCounter (counter);
    }
  std::fill (expanded.begin (), expanded.end (), 0);
}

bool
CsrLegacyCrypto::ConstantTimeEqual (std::span<const uint8_t> left,
                                    std::span<const uint8_t> right)
{
  if (left.size () != right.size ())
    {
      return false;
    }

  uint8_t difference = 0;
  for (std::size_t i = 0; i < left.size (); ++i)
    {
      difference |= static_cast<uint8_t> (left[i] ^ right[i]);
    }
  return difference == 0;
}

} // namespace ns3
