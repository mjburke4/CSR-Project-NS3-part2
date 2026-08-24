#include "ns3/csr-hop-security.h"
#include "ns3/csr-legacy-crypto.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/packet.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId SOURCE = 0x010203;
constexpr CsrNodeId DESTINATION = 0x040506;

uint32_t g_keyUpdateCallbacks = 0;

void
Require (bool condition, const char *message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

void
NoteKeyUpdate (CsrNodeId source)
{
  Require (source == SOURCE, "KeyUpdate callback reported the wrong source");
  g_keyUpdateCallbacks++;
}

template <typename HeaderType>
std::vector<uint8_t>
SerializeHeader (const HeaderType &header)
{
  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (header);
  std::vector<uint8_t> bytes (packet->GetSize ());
  packet->CopyData (bytes.data (), bytes.size ());
  return bytes;
}

void
CheckStandardPrimitives ()
{
  const std::array<uint8_t, 3> abc {'a', 'b', 'c'};
  const CsrLegacyCrypto::Sha1Digest expectedSha1 {
    0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a,
    0xba, 0x3e, 0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c,
    0x9c, 0xd0, 0xd8, 0x9d
  };
  Require (CsrLegacyCrypto::Sha1 (abc) == expectedSha1,
           "SHA-1 does not match the standard abc vector");

  std::array<uint8_t, 20> hmacKey {};
  hmacKey.fill (0x0b);
  const std::array<uint8_t, 8> hmacData {
    'H', 'i', ' ', 'T', 'h', 'e', 'r', 'e'
  };
  const CsrLegacyCrypto::Sha1Digest expectedHmac {
    0xb6, 0x17, 0x31, 0x86, 0x55, 0x05, 0x72, 0x64,
    0xe2, 0x8b, 0xc0, 0xb6, 0xfb, 0x37, 0x8c, 0x8e,
    0xf1, 0x46, 0xbe, 0x00
  };
  Require (CsrLegacyCrypto::HmacSha1 (hmacKey, hmacData) == expectedHmac,
           "HMAC-SHA1 does not match RFC 2202");

  const std::array<uint8_t, 8> password {
    'p', 'a', 's', 's', 'w', 'o', 'r', 'd'
  };
  const std::array<uint8_t, 4> salt {'s', 'a', 'l', 't'};
  const std::vector<uint8_t> expectedPbkdf2 {
    0x0c, 0x60, 0xc8, 0x0f, 0x96, 0x1f, 0x0e, 0x71,
    0xf3, 0xa9, 0xb5, 0x24, 0xaf, 0x60, 0x12, 0x06,
    0x2f, 0xe0, 0x37, 0xa6
  };
  Require (CsrLegacyCrypto::Pbkdf2HmacSha1 (
             password, salt, 1, expectedPbkdf2.size ()) == expectedPbkdf2,
           "PBKDF2-HMAC-SHA1 does not match RFC 6070");

  const CsrLegacyCrypto::EncryptionKey aesKey {
    0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe,
    0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
    0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
    0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4
  };
  const CsrLegacyCrypto::AesBlock iv {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
  };
  const CsrLegacyCrypto::GroupKeyMaterial plaintext {
    0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
    0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
    0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
    0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51
  };
  const CsrLegacyCrypto::GroupKeyMaterial expectedCiphertext {
    0xf5, 0x8c, 0x4c, 0x04, 0xd6, 0xe5, 0xf1, 0xba,
    0x77, 0x9e, 0xab, 0xfb, 0x5f, 0x7b, 0xfb, 0xd6,
    0x9c, 0xfc, 0x4e, 0x96, 0x7e, 0xdb, 0x80, 0x8d,
    0x67, 0x9f, 0x77, 0x7b, 0xc6, 0x70, 0x2c, 0x7d
  };
  CsrLegacyCrypto::GroupKeyMaterial ciphertext =
    CsrLegacyCrypto::Aes256CbcEncrypt (aesKey, iv, plaintext);
  Require (ciphertext == expectedCiphertext,
           "AES-256-CBC does not match NIST SP 800-38A");
  Require (CsrLegacyCrypto::Aes256CbcDecrypt (
             aesKey, iv, ciphertext) == plaintext,
           "AES-256-CBC decrypt did not recover its plaintext");

  std::array<uint8_t, 37> ctrData {};
  for (std::size_t i = 0; i < ctrData.size (); ++i)
    {
      ctrData[i] = static_cast<uint8_t> (3 * i + 1);
    }
  std::array<uint8_t, 37> original = ctrData;
  const std::array<uint8_t, 37> expectedCtr {
    0xfc, 0x5b, 0xd1, 0x38, 0x20, 0x35, 0xa4, 0x9e,
    0xc5, 0x31, 0x62, 0xf8, 0xb5, 0x82, 0x4d, 0xc5,
    0x32, 0x3e, 0x23, 0x61, 0xaa, 0xd2, 0x48, 0x20,
    0x3d, 0xbc, 0x9b, 0x99, 0xae, 0x1d, 0x60, 0xc1,
    0x10, 0x30, 0xe3, 0xde, 0x15
  };
  CsrLegacyCrypto::ApplyAes256Ctr (
    aesKey, SOURCE, DESTINATION, 7, 1, 3, ctrData);
  Require (ctrData == expectedCtr,
           "AES-256-CTR does not match the legacy counter vector");
  CsrLegacyCrypto::ApplyAes256Ctr (
    aesKey, SOURCE, DESTINATION, 7, 1, 3, ctrData);
  Require (ctrData == original, "AES-256-CTR did not round-trip data");
}

void
Provision (CsrHopSecurityState &state,
           CsrNodeId nodeId,
           CsrNodeId peer,
           const CsrLegacyCrypto::MissionKey &missionKey,
           const CsrLegacyCrypto::PairwiseKeyMaterial &pairwiseMaterial)
{
  state.SetNodeId (nodeId);
  state.SetOwnSecurityCount (7);
  state.SetMissionKey (missionKey);
  state.SetPairwiseKeyMaterial (peer, pairwiseMaterial);
}

void
CheckLegacySecurityRecords ()
{
  CsrLegacyCrypto::MissionKey missionKey {};
  CsrLegacyCrypto::PairwiseKeyMaterial pairwiseMaterial {};
  CsrLegacyCrypto::GroupKeyMaterial groupKey {};
  for (std::size_t i = 0; i < missionKey.size (); ++i)
    {
      missionKey[i] = static_cast<uint8_t> (i);
      groupKey[i] = static_cast<uint8_t> (0xa0 + i);
    }
  for (std::size_t i = 0; i < pairwiseMaterial.size (); ++i)
    {
      pairwiseMaterial[i] = static_cast<uint8_t> (0x40 + i);
    }

  const CsrLegacyCrypto::EncryptionKey expectedProtectionEncryption {
    0x04, 0x1b, 0xdf, 0x22, 0xe5, 0x9d, 0x82, 0xe1,
    0x42, 0x3f, 0xda, 0x9d, 0xb4, 0x23, 0x5c, 0x83,
    0xcd, 0x28, 0xbd, 0x6c, 0x30, 0xe9, 0xf8, 0xf2,
    0x7a, 0xb5, 0x0e, 0x44, 0x1d, 0xe3, 0x14, 0xbd
  };
  const CsrLegacyCrypto::AuthenticationKey expectedProtectionAuthentication {
    0xfa, 0x95, 0x5c, 0x7b, 0xef, 0x19, 0x22, 0xe8,
    0x15, 0x11, 0x7a, 0x62, 0x8a, 0x05, 0x15, 0xa3,
    0x7a, 0x76, 0x6f, 0x9a
  };
  CsrLegacyCrypto::ProtectionKeys protectionKeys =
    CsrLegacyCrypto::DeriveProtectionKeys (
      missionKey, pairwiseMaterial, SOURCE, 7, 1);
  Require (protectionKeys.encryption == expectedProtectionEncryption &&
           protectionKeys.authentication == expectedProtectionAuthentication,
           "legacy pairwise A5/5A protection KDF vector changed");

  CsrHopSecurityState sender;
  CsrHopSecurityState receiver;
  Provision (sender, SOURCE, DESTINATION, missionKey, pairwiseMaterial);
  Provision (receiver, DESTINATION, SOURCE, missionKey, pairwiseMaterial);
  sender.SetGroupKeyMaterial (groupKey);

  CsrKeyRequestHeader request = sender.BuildKeyRequest (DESTINATION);
  const CsrKeyRequestHeader::AuthTag expectedRequestTag {
    0x33, 0x5f, 0x8d, 0x9b
  };
  const std::vector<uint8_t> expectedRequestWire {
    0x00, 0x10, 0x02, 0x33, 0x5f, 0x8d, 0x9b
  };
  Require (request.GetKeyId () == 1 && request.GetSequence () == 2 &&
           request.GetAuthTag () == expectedRequestTag &&
           SerializeHeader (request) == expectedRequestWire,
           "KeyRequest does not match the legacy golden vector");
  Require (receiver.ReceiveKeyRequest (SOURCE, 7, request) ==
             CsrHopSecurityReceiveStatus::Accepted,
           "valid KeyRequest authentication failed");

  const CsrLegacyCrypto::CryptoPrn prn {0x10, 0x20, 0x30, 0x40, 0x50};
  sender.SetNextDataPrn (prn);
  CsrKeyUpdateHeader update = sender.BuildKeyUpdate (DESTINATION);
  const CsrKeyUpdateHeader::WrappedKey expectedWrappedKey {
    0x8d, 0x42, 0x5e, 0x50, 0x9a, 0x7b, 0x6d, 0xf2,
    0xcf, 0x15, 0xd1, 0xf6, 0xd3, 0x8b, 0x16, 0x79,
    0x9c, 0x07, 0x13, 0x4a, 0x9f, 0xec, 0x91, 0xff,
    0x4d, 0x39, 0x63, 0x0e, 0x0b, 0x8f, 0xe0, 0xed
  };
  const CsrKeyUpdateHeader::AuthTag expectedUpdateTag {
    0x46, 0x3c, 0x0b, 0xe5, 0x05, 0x8e, 0x67, 0x38
  };
  const std::vector<uint8_t> expectedUpdateWire {
    0x00, 0x10, 0x03, 0x00, 0x01, 0x10, 0x20, 0x30,
    0x40, 0x50, 0x00, 0x8d, 0x42, 0x5e, 0x50, 0x9a,
    0x7b, 0x6d, 0xf2, 0xcf, 0x15, 0xd1, 0xf6, 0xd3,
    0x8b, 0x16, 0x79, 0x9c, 0x07, 0x13, 0x4a, 0x9f,
    0xec, 0x91, 0xff, 0x4d, 0x39, 0x63, 0x0e, 0x0b,
    0x8f, 0xe0, 0xed, 0x46, 0x3c, 0x0b, 0xe5, 0x05,
    0x8e, 0x67, 0x38
  };
  Require (update.GetKeyId () == 1 && update.GetSequence () == 3 &&
           update.GetGroupKeyId () == 1 &&
           update.GetDataPrn () == CsrKeyUpdateHeader::DataPrn {
             0x10, 0x20, 0x30, 0x40, 0x50, 0x00} &&
           update.GetWrappedKey () == expectedWrappedKey &&
           update.GetAuthTag () == expectedUpdateTag &&
           SerializeHeader (update) == expectedUpdateWire,
           "KeyUpdate does not match the legacy golden vector");

  Require (receiver.ReceiveKeyUpdate (SOURCE, 7, update) ==
             CsrHopSecurityReceiveStatus::Accepted,
           "valid KeyUpdate authentication or unwrap failed");
  CsrLegacyCrypto::GroupKeyMaterial recovered {};
  Require (receiver.GetReceivedGroupKeyMaterial (SOURCE, recovered) &&
           recovered == groupKey,
           "KeyUpdate did not recover the authenticated group key");
  Require (receiver.ReceiveKeyUpdate (SOURCE, 7, update) ==
             CsrHopSecurityReceiveStatus::AuthenticatedDuplicate,
           "valid KeyUpdate retransmit was not authenticated as a duplicate");

  CsrHopSecurityState tamperReceiver;
  Provision (tamperReceiver,
             DESTINATION,
             SOURCE,
             missionKey,
             pairwiseMaterial);
  CsrKeyRequestHeader badRequest = request;
  CsrKeyRequestHeader::AuthTag badRequestTag = badRequest.GetAuthTag ();
  badRequestTag[0] ^= 0x80;
  badRequest.SetAuthTag (badRequestTag);
  Require (tamperReceiver.ReceiveKeyRequest (SOURCE, 7, badRequest) ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed &&
           tamperReceiver.ReceiveKeyRequest (SOURCE, 7, request) ==
             CsrHopSecurityReceiveStatus::Accepted,
           "failed KeyRequest authentication consumed replay state");

  CsrKeyUpdateHeader badUpdate = update;
  CsrKeyUpdateHeader::WrappedKey badWrappedKey = badUpdate.GetWrappedKey ();
  badWrappedKey[0] ^= 0x01;
  badUpdate.SetWrappedKey (badWrappedKey);
  CsrHopSecurityState updateTamperReceiver;
  Provision (updateTamperReceiver,
             DESTINATION,
             SOURCE,
             missionKey,
             pairwiseMaterial);
  Require (updateTamperReceiver.ReceiveKeyUpdate (SOURCE, 7, badUpdate) ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed &&
           updateTamperReceiver.ReceiveKeyUpdate (SOURCE, 7, update) ==
             CsrHopSecurityReceiveStatus::Accepted,
           "failed KeyUpdate authentication consumed replay or key state");

  CsrLegacyCrypto::PairwiseKeyMaterial wrongPairwise = pairwiseMaterial;
  wrongPairwise[0] ^= 0xff;
  CsrHopSecurityState wrongKeyReceiver;
  Provision (wrongKeyReceiver,
             DESTINATION,
             SOURCE,
             missionKey,
             wrongPairwise);
  Require (wrongKeyReceiver.ReceiveKeyUpdate (SOURCE, 7, update) ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "KeyUpdate authenticated with the wrong pairwise key");

  CsrKeyUpdateHeader reservedByteUpdate = update;
  CsrKeyUpdateHeader::DataPrn reservedPrn = reservedByteUpdate.GetDataPrn ();
  reservedPrn.back () = 0xff;
  reservedByteUpdate.SetDataPrn (reservedPrn);
  CsrHopSecurityState reservedByteReceiver;
  Provision (reservedByteReceiver,
             DESTINATION,
             SOURCE,
             missionKey,
             pairwiseMaterial);
  Require (reservedByteReceiver.ReceiveKeyUpdate (
             SOURCE, 7, reservedByteUpdate) ==
             CsrHopSecurityReceiveStatus::Accepted,
           "reserved sixth KeyUpdate PRN byte entered the crypto calculation");
}

void
CheckAuthenticationBeforeHopAck ()
{
  CsrLegacyCrypto::MissionKey missionKey {};
  CsrLegacyCrypto::PairwiseKeyMaterial pairwiseMaterial {};
  CsrLegacyCrypto::GroupKeyMaterial groupKey {};
  for (std::size_t i = 0; i < missionKey.size (); ++i)
    {
      missionKey[i] = static_cast<uint8_t> (0x20 + i);
      groupKey[i] = static_cast<uint8_t> (0xc0 + i);
    }
  for (std::size_t i = 0; i < pairwiseMaterial.size (); ++i)
    {
      pairwiseMaterial[i] = static_cast<uint8_t> (0x80 + i);
    }

  CsrHopSecurityState sender;
  Provision (sender, SOURCE, DESTINATION, missionKey, pairwiseMaterial);
  sender.SetGroupKeyMaterial (groupKey);
  sender.SetNextDataPrn ({0x01, 0x23, 0x45, 0x67, 0x89});
  CsrKeyUpdateHeader validUpdate = sender.BuildKeyUpdate (DESTINATION);

  CsrKeyUpdateHeader badUpdate = validUpdate;
  CsrKeyUpdateHeader::AuthTag badTag = badUpdate.GetAuthTag ();
  badTag[0] ^= 0x01;
  badUpdate.SetAuthTag (badTag);

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
  receiver->SetNodeId (DESTINATION);
  receiver->SetMac (&device->GetMac ());
  receiver->SetMissionKey (missionKey);
  receiver->SetPairwiseKeyMaterial (SOURCE, pairwiseMaterial);
  receiver->SetKeyUpdateReceivedCallback (MakeCallback (&NoteKeyUpdate));

  auto buildFrame = [] (const CsrKeyUpdateHeader &securityHeader) {
    CsrHeader hopHeader (SOURCE, DESTINATION, 42, 7, true, false);
    hopHeader.SetType (CSR_PKT_KEY_UPDATE);
    hopHeader.SetDestType (CSR_DEST_UNICAST);
    hopHeader.SetSecurityCount (7);
    Ptr<Packet> frame = Create<Packet> ();
    frame->AddHeader (securityHeader);
    frame->AddHeader (hopHeader);
    return frame;
  };

  g_keyUpdateCallbacks = 0;
  receiver->ReceiveFromMac (buildFrame (badUpdate), 60.0, 40.0);
  Require (device->GetMac ().GetAckQueuedFrameCount () == 0 &&
           g_keyUpdateCallbacks == 0 &&
           !receiver->HasGroupKeyReceivedFrom (SOURCE),
           "unauthenticated KeyUpdate earned an ACK or changed admission state");

  receiver->ReceiveFromMac (buildFrame (validUpdate), 60.0, 40.0);
  Require (device->GetMac ().GetAckQueuedFrameCount () == 1 &&
           g_keyUpdateCallbacks == 1 &&
           receiver->HasGroupKeyReceivedFrom (SOURCE),
           "valid KeyUpdate was poisoned by the earlier tampered frame");

  Simulator::Destroy ();
}

void
CheckReplayWindowAndSecurityCount ()
{
  CsrHopSecurityState sender;
  CsrHopSecurityState receiver;
  sender.SetNodeId (SOURCE);
  sender.SetOwnSecurityCount (0xfffe);
  receiver.SetNodeId (DESTINATION);

  std::vector<CsrKeyRequestHeader> requests;
  for (uint32_t i = 0; i < 130; ++i)
    {
      requests.push_back (sender.BuildKeyRequest (DESTINATION));
    }

  Require (receiver.ReceiveKeyRequest (SOURCE, 0xfffe, requests.back ()) ==
             CsrHopSecurityReceiveStatus::Accepted,
           "full-width security count or future replay-window entry failed");
  Require (receiver.ReceiveKeyRequest (SOURCE, 0xfffe, requests[50]) ==
             CsrHopSecurityReceiveStatus::Accepted,
           "unseen out-of-order packet inside the 128-entry window failed");
  Require (receiver.ReceiveKeyRequest (SOURCE, 0xfffe, requests.front ()) ==
             CsrHopSecurityReceiveStatus::DuplicateOrStale,
           "packet before the 128-entry replay window was accepted");
  Require (receiver.ReceiveKeyRequest (SOURCE, 0xfffe, requests.back ()) ==
             CsrHopSecurityReceiveStatus::AuthenticatedDuplicate,
           "received replay-window entry was not identified as a duplicate");

  CsrHopSecurityState restartedSender;
  restartedSender.SetNodeId (SOURCE);
  restartedSender.SetOwnSecurityCount (1);
  CsrKeyRequestHeader restartedRequest =
    restartedSender.BuildKeyRequest (DESTINATION);
  Require (receiver.ReceiveKeyRequest (SOURCE, 1, restartedRequest) ==
             CsrHopSecurityReceiveStatus::AcceptedSecurityCountChanged,
           "16-bit security-count wrap was not accepted as forward progress");

  CsrKeyRequestHeader oldCountRequest =
    sender.BuildKeyRequest (DESTINATION);
  Require (receiver.ReceiveKeyRequest (SOURCE, 0xfffe, oldCountRequest) ==
             CsrHopSecurityReceiveStatus::DuplicateOrStale,
           "past security count was accepted after wrap");
}

} // namespace

int
main ()
{
  CheckStandardPrimitives ();
  CheckLegacySecurityRecords ();
  CheckAuthenticationBeforeHopAck ();
  CheckReplayWindowAndSecurityCount ();
  std::cout << "PASS: legacy CSR hop-security crypto test" << std::endl;
  return 0;
}
