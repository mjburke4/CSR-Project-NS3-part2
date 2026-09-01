#include "csr-hop-security.h"

#include "ns3/abort.h"

#include <algorithm>

namespace ns3 {

namespace
{

static constexpr uint16_t CSR_SECURITY_COUNTER_MAX = 0x0fff;
static constexpr uint8_t CSR_LEGACY_KEY_UPDATE_TYPE = 0x01;
static constexpr uint8_t CSR_LEGACY_KEY_REQUEST_TYPE = 0x02;
static constexpr uint16_t CSR_GROUP_KEY_ID_STEP_SIZE = 16;
static constexpr std::size_t CSR_GROUP_KEY_SEQUENCE_SIZE = 3;
static constexpr std::size_t CSR_PAIRWISE_KEY_SEQUENCE_SIZE = 3;

std::size_t
GetPairwiseAuthTagSize (CsrPairwiseSecurityMode mode)
{
  switch (mode)
    {
    case CsrPairwiseSecurityMode::Pairwise16:
      return 2;
    case CsrPairwiseSecurityMode::Pairwise32:
    case CsrPairwiseSecurityMode::Pairwise32Encrypt:
      return 4;
    }
  return 0;
}

bool
PairwiseModeEncryptsPayload (CsrPairwiseSecurityMode mode)
{
  return mode == CsrPairwiseSecurityMode::Pairwise32Encrypt;
}

std::size_t
GetGroupAuthTagSize (CsrGroupSecurityMode mode)
{
  switch (mode)
    {
    case CsrGroupSecurityMode::Group16:
      return 2;
    case CsrGroupSecurityMode::GroupEstablish:
    case CsrGroupSecurityMode::Group32Encrypt:
      return 4;
    }
  return 0;
}

bool
GroupModeEncryptsPayload (CsrGroupSecurityMode mode)
{
  return mode == CsrGroupSecurityMode::GroupEstablish ||
         mode == CsrGroupSecurityMode::Group32Encrypt;
}

std::array<uint8_t, 3>
SerializePackedKeySequence (uint16_t keyId, uint16_t sequence)
{
  return {
    static_cast<uint8_t> (keyId >> 4),
    static_cast<uint8_t> ((keyId << 4) | (sequence >> 8)),
    static_cast<uint8_t> (sequence)
  };
}

void
WritePackedKeySequence (Buffer::Iterator &start,
                        uint16_t keyId,
                        uint16_t sequence)
{
  start.WriteU8 (static_cast<uint8_t> (keyId >> 4));
  start.WriteU8 (static_cast<uint8_t> ((keyId << 4) |
                                       (sequence >> 8)));
  start.WriteU8 (static_cast<uint8_t> (sequence));
}

void
ReadPackedKeySequence (Buffer::Iterator &start,
                       uint16_t &keyId,
                       uint16_t &sequence)
{
  uint8_t first = start.ReadU8 ();
  uint8_t second = start.ReadU8 ();
  uint8_t third = start.ReadU8 ();

  keyId = static_cast<uint16_t> ((first << 4) | (second >> 4));
  sequence = static_cast<uint16_t> (((second & 0x0f) << 8) | third);
}

template <std::size_t Size>
void
WriteArray (Buffer::Iterator &start,
            const std::array<uint8_t, Size> &bytes)
{
  for (uint8_t byte : bytes)
    {
      start.WriteU8 (byte);
    }
}

template <std::size_t Size>
void
ReadArray (Buffer::Iterator &start,
           std::array<uint8_t, Size> &bytes)
{
  for (uint8_t &byte : bytes)
    {
      byte = start.ReadU8 ();
    }
}

void
RequireTwelveBit (uint16_t value, const char *field)
{
  NS_ABORT_MSG_IF (value > CSR_SECURITY_COUNTER_MAX,
                   field << " exceeds the legacy 12-bit field");
}

} // namespace

NS_OBJECT_ENSURE_REGISTERED (CsrKeyRequestHeader);
NS_OBJECT_ENSURE_REGISTERED (CsrKeyUpdateHeader);

TypeId
CsrKeyRequestHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::CsrKeyRequestHeader")
    .SetParent<Header> ()
    .SetGroupName ("Csr")
    .AddConstructor<CsrKeyRequestHeader> ();
  return tid;
}

TypeId
CsrKeyRequestHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
CsrKeyRequestHeader::GetSerializedSize () const
{
  return SERIALIZED_SIZE;
}

void
CsrKeyRequestHeader::Serialize (Buffer::Iterator start) const
{
  WritePackedKeySequence (start, m_keyId, m_sequence);
  WriteArray (start, m_authTag);
}

uint32_t
CsrKeyRequestHeader::Deserialize (Buffer::Iterator start)
{
  ReadPackedKeySequence (start, m_keyId, m_sequence);
  ReadArray (start, m_authTag);
  return SERIALIZED_SIZE;
}

void
CsrKeyRequestHeader::Print (std::ostream &os) const
{
  os << "keyId=" << m_keyId
     << " sequence=" << m_sequence
     << " authBytes=" << AUTH_TAG_SIZE;
}

void
CsrKeyRequestHeader::SetKeyId (uint16_t keyId)
{
  RequireTwelveBit (keyId, "CSR KeyRequest key identifier");
  m_keyId = keyId;
}

uint16_t
CsrKeyRequestHeader::GetKeyId () const
{
  return m_keyId;
}

void
CsrKeyRequestHeader::SetSequence (uint16_t sequence)
{
  RequireTwelveBit (sequence, "CSR KeyRequest sequence");
  m_sequence = sequence;
}

uint16_t
CsrKeyRequestHeader::GetSequence () const
{
  return m_sequence;
}

void
CsrKeyRequestHeader::SetAuthTag (const AuthTag &authTag)
{
  m_authTag = authTag;
}

const CsrKeyRequestHeader::AuthTag&
CsrKeyRequestHeader::GetAuthTag () const
{
  return m_authTag;
}

TypeId
CsrKeyUpdateHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::CsrKeyUpdateHeader")
    .SetParent<Header> ()
    .SetGroupName ("Csr")
    .AddConstructor<CsrKeyUpdateHeader> ();
  return tid;
}

TypeId
CsrKeyUpdateHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
CsrKeyUpdateHeader::GetSerializedSize () const
{
  return SERIALIZED_SIZE;
}

void
CsrKeyUpdateHeader::Serialize (Buffer::Iterator start) const
{
  WritePackedKeySequence (start, m_keyId, m_sequence);

  // The upper nibble is reserved in hopSecLayer.c.
  start.WriteHtonU16 (static_cast<uint16_t> (m_groupKeyId & 0x0fff));
  WriteArray (start, m_dataPrn);
  WriteArray (start, m_wrappedKey);
  WriteArray (start, m_authTag);
}

uint32_t
CsrKeyUpdateHeader::Deserialize (Buffer::Iterator start)
{
  ReadPackedKeySequence (start, m_keyId, m_sequence);
  m_groupKeyId = static_cast<uint16_t> (start.ReadNtohU16 () & 0x0fff);
  ReadArray (start, m_dataPrn);
  ReadArray (start, m_wrappedKey);
  ReadArray (start, m_authTag);
  return SERIALIZED_SIZE;
}

void
CsrKeyUpdateHeader::Print (std::ostream &os) const
{
  os << "keyId=" << m_keyId
     << " sequence=" << m_sequence
     << " groupKeyId=" << m_groupKeyId
     << " prnBytes=" << DATA_PRN_SIZE
     << " wrappedKeyBytes=" << WRAPPED_KEY_SIZE
     << " authBytes=" << AUTH_TAG_SIZE;
}

void
CsrKeyUpdateHeader::SetKeyId (uint16_t keyId)
{
  RequireTwelveBit (keyId, "CSR KeyUpdate key identifier");
  m_keyId = keyId;
}

uint16_t
CsrKeyUpdateHeader::GetKeyId () const
{
  return m_keyId;
}

void
CsrKeyUpdateHeader::SetSequence (uint16_t sequence)
{
  RequireTwelveBit (sequence, "CSR KeyUpdate sequence");
  m_sequence = sequence;
}

uint16_t
CsrKeyUpdateHeader::GetSequence () const
{
  return m_sequence;
}

void
CsrKeyUpdateHeader::SetGroupKeyId (uint16_t groupKeyId)
{
  RequireTwelveBit (groupKeyId, "CSR KeyUpdate group-key identifier");
  m_groupKeyId = groupKeyId;
}

uint16_t
CsrKeyUpdateHeader::GetGroupKeyId () const
{
  return m_groupKeyId;
}

void
CsrKeyUpdateHeader::SetDataPrn (const DataPrn &dataPrn)
{
  m_dataPrn = dataPrn;
}

const CsrKeyUpdateHeader::DataPrn&
CsrKeyUpdateHeader::GetDataPrn () const
{
  return m_dataPrn;
}

void
CsrKeyUpdateHeader::SetWrappedKey (const WrappedKey &wrappedKey)
{
  m_wrappedKey = wrappedKey;
}

const CsrKeyUpdateHeader::WrappedKey&
CsrKeyUpdateHeader::GetWrappedKey () const
{
  return m_wrappedKey;
}

void
CsrKeyUpdateHeader::SetAuthTag (const AuthTag &authTag)
{
  m_authTag = authTag;
}

const CsrKeyUpdateHeader::AuthTag&
CsrKeyUpdateHeader::GetAuthTag () const
{
  return m_authTag;
}

CsrHopSecurityState::CsrHopSecurityState ()
{
  // These deterministic values are simulation-only defaults. Applications
  // can inject synthetic scenario keys through the provisioning methods.
  for (std::size_t i = 0; i < m_missionKey.size (); ++i)
    {
      m_missionKey[i] = static_cast<uint8_t> (i);
    }
  for (std::size_t i = 0; i < m_defaultPairwiseKeyMaterial.size (); ++i)
    {
      m_defaultPairwiseKeyMaterial[i] =
        static_cast<uint8_t> (0x40 + i);
    }
  for (std::size_t i = 0; i < m_groupKeyMaterial.size (); ++i)
    {
      m_groupKeyMaterial[i] = static_cast<uint8_t> (0xa0 + i);
    }
}

void
CsrHopSecurityState::SetNodeId (CsrNodeId nodeId)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (nodeId),
                   "CSR hop-security node identifier exceeds 24 bits");
  m_nodeId = nodeId;
}

CsrNodeId
CsrHopSecurityState::GetNodeId () const
{
  return m_nodeId;
}

void
CsrHopSecurityState::SetOwnSecurityCount (uint16_t securityCount)
{
  m_ownSecurityCount = securityCount;
}

uint16_t
CsrHopSecurityState::GetOwnSecurityCount () const
{
  return m_ownSecurityCount;
}

void
CsrHopSecurityState::SetGroupKeyId (uint16_t groupKeyId)
{
  RequireTwelveBit (groupKeyId, "CSR group-key identifier");
  m_groupKeyId = groupKeyId;
}

uint16_t
CsrHopSecurityState::GetGroupKeyId () const
{
  return m_groupKeyId;
}

void
CsrHopSecurityState::SetGroupSequence (uint16_t groupSequence)
{
  RequireTwelveBit (groupSequence, "CSR group sequence");
  m_groupSequence = groupSequence;
}

uint16_t
CsrHopSecurityState::GetGroupSequence () const
{
  return m_groupSequence;
}

void
CsrHopSecurityState::SetMissionKey (const MissionKey &missionKey)
{
  m_missionKey = missionKey;
}

void
CsrHopSecurityState::SetDefaultPairwiseKeyMaterial (
  const PairwiseKeyMaterial &pairwiseMaterial)
{
  m_defaultPairwiseKeyMaterial = pairwiseMaterial;
}

void
CsrHopSecurityState::SetPairwiseKeyMaterial (
  CsrNodeId neighbor,
  const PairwiseKeyMaterial &pairwiseMaterial)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (neighbor) ||
                   neighbor == CSR_BROADCAST_ID,
                   "CSR pairwise-key neighbor must be a unicast node");
  m_pairwiseKeyMaterial[neighbor] = pairwiseMaterial;
}

void
CsrHopSecurityState::SetGroupKeyMaterial (
  const GroupKeyMaterial &groupKeyMaterial)
{
  m_groupKeyMaterial = groupKeyMaterial;
}

const CsrHopSecurityState::GroupKeyMaterial&
CsrHopSecurityState::GetGroupKeyMaterial () const
{
  return m_groupKeyMaterial;
}

void
CsrHopSecurityState::SetNextGeneratedGroupKeyMaterial (
  const GroupKeyMaterial &groupKeyMaterial)
{
  m_nextGeneratedGroupKeyMaterial = groupKeyMaterial;
}

bool
CsrHopSecurityState::GetReceivedGroupKeyMaterial (
  CsrNodeId neighbor,
  GroupKeyMaterial &groupKey) const
{
  const NeighborState *state = FindNeighbor (neighbor);
  if (state == nullptr || !state->groupKeyReceived)
    {
      return false;
    }
  groupKey = state->groupKeyMaterial;
  return true;
}

void
CsrHopSecurityState::SetNextDataPrn (const CryptoPrn &prn)
{
  m_nextDataPrn = prn;
}

CsrKeyRequestHeader
CsrHopSecurityState::BuildKeyRequest (CsrNodeId destination)
{
  NeighborState &neighbor = GetOrCreateNeighbor (destination);
  uint16_t keyId = 0;
  uint16_t sequence = 0;
  NextPairwiseKeySequence (neighbor, keyId, sequence);

  CsrKeyRequestHeader header;
  header.SetKeyId (keyId);
  header.SetSequence (sequence);

  CsrLegacyCrypto::ProtectionKeys keys =
    DerivePairwiseProtectionKeys (destination,
                                  m_nodeId,
                                  m_ownSecurityCount,
                                  keyId);
  std::array<uint8_t, 3> data =
    SerializePackedKeySequence (keyId, sequence);
  std::vector<uint8_t> tag = CsrLegacyCrypto::ComputePairwiseAuthTag (
    keys.authentication,
    m_nodeId,
    destination,
    CSR_LEGACY_KEY_REQUEST_TYPE,
    data,
    CsrKeyRequestHeader::AUTH_TAG_SIZE);
  CsrKeyRequestHeader::AuthTag authTag {};
  std::copy (tag.begin (), tag.end (), authTag.begin ());
  header.SetAuthTag (authTag);
  return header;
}

CsrKeyUpdateHeader
CsrHopSecurityState::BuildKeyUpdate (CsrNodeId destination)
{
  NeighborState &neighbor = GetOrCreateNeighbor (destination);
  uint16_t keyId = 0;
  uint16_t sequence = 0;
  NextPairwiseKeySequence (neighbor, keyId, sequence);

  neighbor.keyUpdateSendActive = true;

  CsrKeyUpdateHeader header;
  header.SetKeyId (keyId);
  header.SetSequence (sequence);
  header.SetGroupKeyId (m_groupKeyId);

  CryptoPrn prn = GenerateDataPrn ();
  CsrKeyUpdateHeader::DataPrn wirePrn {};
  std::copy (prn.begin (), prn.end (), wirePrn.begin ());
  header.SetDataPrn (wirePrn);

  CsrLegacyCrypto::ProtectionKeys protectionKeys =
    DerivePairwiseProtectionKeys (destination,
                                  m_nodeId,
                                  m_ownSecurityCount,
                                  keyId);
  CsrKeyUpdateHeader::AuthTag authTag =
    CsrLegacyCrypto::ComputeKeyUpdateAuthTag (
      protectionKeys.authentication,
      CSR_LEGACY_KEY_UPDATE_TYPE,
      m_nodeId,
      m_ownSecurityCount,
      keyId,
      sequence,
      destination,
      0,
      m_groupKeyId,
      prn,
      m_groupKeyMaterial);
  header.SetAuthTag (authTag);

  CsrLegacyCrypto::EncryptionKey wrappingKey =
    CsrLegacyCrypto::DerivePairwiseWrappingKey (
      m_missionKey,
      GetPairwiseKeyMaterial (destination),
      m_nodeId,
      m_ownSecurityCount,
      m_groupKeyId,
      prn);
  CsrLegacyCrypto::AesBlock iv =
    CsrLegacyCrypto::BuildKeyUpdateIv (authTag, prn, m_nodeId);
  header.SetWrappedKey (CsrLegacyCrypto::Aes256CbcEncrypt (
    wrappingKey,
    iv,
    m_groupKeyMaterial));
  return header;
}

CsrHopSecurityReceiveStatus
CsrHopSecurityState::ReceiveKeyRequest (
  CsrNodeId source,
  uint16_t securityCount,
  const CsrKeyRequestHeader &header)
{
  NeighborState &neighbor = GetOrCreateNeighbor (source);

  bool securityCountChanged =
    neighbor.securityCountValid && neighbor.securityCount != securityCount;
  if (securityCountChanged &&
      !IsSecurityCountChangeValid (neighbor.securityCount, securityCount))
    {
      return CsrHopSecurityReceiveStatus::DuplicateOrStale;
    }

  uint32_t value =
    (static_cast<uint32_t> (header.GetKeyId ()) << 12) |
    header.GetSequence ();
  if (!securityCountChanged && neighbor.inboundPairwise.IsBeforeWindow (value))
    {
      return CsrHopSecurityReceiveStatus::DuplicateOrStale;
    }

  CsrLegacyCrypto::ProtectionKeys keys =
    DerivePairwiseProtectionKeys (source,
                                  source,
                                  securityCount,
                                  header.GetKeyId ());
  std::array<uint8_t, 3> data = SerializePackedKeySequence (
    header.GetKeyId (),
    header.GetSequence ());
  std::vector<uint8_t> expected = CsrLegacyCrypto::ComputePairwiseAuthTag (
    keys.authentication,
    source,
    m_nodeId,
    CSR_LEGACY_KEY_REQUEST_TYPE,
    data,
    CsrKeyRequestHeader::AUTH_TAG_SIZE);
  if (!CsrLegacyCrypto::ConstantTimeEqual (expected, header.GetAuthTag ()))
    {
      return CsrHopSecurityReceiveStatus::AuthenticationFailed;
    }

  return CommitAuthenticatedPairwise (neighbor,
                                      securityCount,
                                      header.GetKeyId (),
                                      header.GetSequence (),
                                      securityCountChanged);
}

CsrHopSecurityReceiveStatus
CsrHopSecurityState::ReceiveKeyUpdate (
  CsrNodeId source,
  uint16_t securityCount,
  const CsrKeyUpdateHeader &header)
{
  NeighborState &neighbor = GetOrCreateNeighbor (source);

  bool securityCountChanged =
    neighbor.securityCountValid && neighbor.securityCount != securityCount;
  if (securityCountChanged &&
      !IsSecurityCountChangeValid (neighbor.securityCount, securityCount))
    {
      return CsrHopSecurityReceiveStatus::DuplicateOrStale;
    }

  uint32_t value =
    (static_cast<uint32_t> (header.GetKeyId ()) << 12) |
    header.GetSequence ();
  if (!securityCountChanged && neighbor.inboundPairwise.IsBeforeWindow (value))
    {
      return CsrHopSecurityReceiveStatus::DuplicateOrStale;
    }

  CryptoPrn prn {};
  std::copy_n (header.GetDataPrn ().begin (),
               prn.size (),
               prn.begin ());
  CsrLegacyCrypto::EncryptionKey wrappingKey =
    CsrLegacyCrypto::DerivePairwiseWrappingKey (
      m_missionKey,
      GetPairwiseKeyMaterial (source),
      source,
      securityCount,
      header.GetGroupKeyId (),
      prn);
  CsrLegacyCrypto::AesBlock iv = CsrLegacyCrypto::BuildKeyUpdateIv (
    header.GetAuthTag (),
    prn,
    source);
  GroupKeyMaterial groupKey = CsrLegacyCrypto::Aes256CbcDecrypt (
    wrappingKey,
    iv,
    header.GetWrappedKey ());

  CsrLegacyCrypto::ProtectionKeys protectionKeys =
    DerivePairwiseProtectionKeys (source,
                                  source,
                                  securityCount,
                                  header.GetKeyId ());
  CsrKeyUpdateHeader::AuthTag expected =
    CsrLegacyCrypto::ComputeKeyUpdateAuthTag (
      protectionKeys.authentication,
      CSR_LEGACY_KEY_UPDATE_TYPE,
      source,
      securityCount,
      header.GetKeyId (),
      header.GetSequence (),
      m_nodeId,
      0,
      header.GetGroupKeyId (),
      prn,
      groupKey);
  if (!CsrLegacyCrypto::ConstantTimeEqual (expected, header.GetAuthTag ()))
    {
      return CsrHopSecurityReceiveStatus::AuthenticationFailed;
    }

  CsrHopSecurityReceiveStatus status = CommitAuthenticatedPairwise (
    neighbor,
    securityCount,
    header.GetKeyId (),
    header.GetSequence (),
    securityCountChanged);
  if (status == CsrHopSecurityReceiveStatus::AuthenticatedDuplicate)
    {
      return status;
    }

  if (neighbor.groupKeyReceived &&
      header.GetGroupKeyId () != neighbor.lastGroupKeyIdReceived)
    {
      neighbor.previousGroupKeyReceived = true;
      neighbor.previousGroupKeyIdReceived = neighbor.lastGroupKeyIdReceived;
      neighbor.previousGroupKeyMaterial = neighbor.groupKeyMaterial;
    }
  neighbor.groupKeyReceived = true;
  neighbor.lastGroupKeyIdReceived = header.GetGroupKeyId ();
  neighbor.groupKeyMaterial = groupKey;
  return status;
}

CsrProtectedPairwiseMessage
CsrHopSecurityState::ProtectPairwiseMessage (
  CsrNodeId destination,
  CsrPairwiseSecurityMode mode,
  uint8_t legacyPacketType,
  std::span<const uint8_t> payload)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (destination) ||
                   destination == CSR_BROADCAST_ID,
                   "CSR pairwise destination must be a unicast node");

  std::size_t authLength = GetPairwiseAuthTagSize (mode);
  NS_ABORT_MSG_IF (authLength == 0,
                   "CSR pairwise security mode has no authentication tag");

  NeighborState &neighbor = GetOrCreateNeighbor (destination);
  uint16_t keyId = 0;
  uint16_t sequence = 0;
  NextPairwiseKeySequence (neighbor, keyId, sequence);

  CsrLegacyCrypto::ProtectionKeys keys =
    DerivePairwiseProtectionKeys (destination,
                                  m_nodeId,
                                  m_ownSecurityCount,
                                  keyId);

  std::vector<uint8_t> protectedPayload (payload.begin (), payload.end ());
  if (PairwiseModeEncryptsPayload (mode) && !protectedPayload.empty ())
    {
      CsrLegacyCrypto::ApplyAes256Ctr (keys.encryption,
                                       m_nodeId,
                                       destination,
                                       m_ownSecurityCount,
                                       keyId,
                                       sequence,
                                       protectedPayload);
    }

  std::array<uint8_t, CSR_PAIRWISE_KEY_SEQUENCE_SIZE> packed =
    SerializePackedKeySequence (keyId, sequence);
  std::vector<uint8_t> record (packed.begin (), packed.end ());
  record.insert (record.end (),
                 protectedPayload.begin (),
                 protectedPayload.end ());

  std::vector<uint8_t> tag = CsrLegacyCrypto::ComputePairwiseAuthTag (
    keys.authentication,
    m_nodeId,
    destination,
    legacyPacketType,
    record,
    authLength);
  record.insert (record.end (), tag.begin (), tag.end ());

  return {keyId, sequence, std::move (record)};
}

CsrReceivedPairwiseMessage
CsrHopSecurityState::ReceivePairwiseMessage (
  CsrNodeId source,
  uint16_t securityCount,
  CsrPairwiseSecurityMode mode,
  uint8_t legacyPacketType,
  std::span<const uint8_t> record)
{
  CsrReceivedPairwiseMessage result;
  std::size_t authLength = GetPairwiseAuthTagSize (mode);

  if (!CsrIsValidNodeId (source) || source == CSR_BROADCAST_ID ||
      authLength == 0 ||
      record.size () < CSR_PAIRWISE_KEY_SEQUENCE_SIZE + authLength)
    {
      return result;
    }

  uint16_t keyId = static_cast<uint16_t> (
    (static_cast<uint16_t> (record[0]) << 4) | (record[1] >> 4));
  uint16_t sequence = static_cast<uint16_t> (
    (static_cast<uint16_t> (record[1] & 0x0f) << 8) | record[2]);
  result.keyId = keyId;
  result.sequence = sequence;

  NeighborState &neighbor = GetOrCreateNeighbor (source);
  bool securityCountChanged =
    neighbor.securityCountValid && neighbor.securityCount != securityCount;
  if (securityCountChanged &&
      !IsSecurityCountChangeValid (neighbor.securityCount, securityCount))
    {
      result.status = CsrHopSecurityReceiveStatus::DuplicateOrStale;
      return result;
    }

  uint32_t replayValue =
    (static_cast<uint32_t> (keyId) << 12) | sequence;
  if (!securityCountChanged &&
      neighbor.inboundPairwise.IsBeforeWindow (replayValue))
    {
      result.status = CsrHopSecurityReceiveStatus::DuplicateOrStale;
      return result;
    }

  CsrLegacyCrypto::ProtectionKeys keys =
    DerivePairwiseProtectionKeys (source,
                                  source,
                                  securityCount,
                                  keyId);
  std::span<const uint8_t> authenticatedRecord =
    record.first (record.size () - authLength);
  std::span<const uint8_t> receivedTag = record.last (authLength);
  std::vector<uint8_t> expected = CsrLegacyCrypto::ComputePairwiseAuthTag (
    keys.authentication,
    source,
    m_nodeId,
    legacyPacketType,
    authenticatedRecord,
    authLength);
  if (!CsrLegacyCrypto::ConstantTimeEqual (expected, receivedTag))
    {
      result.status = CsrHopSecurityReceiveStatus::AuthenticationFailed;
      return result;
    }

  CsrHopSecurityReceiveStatus status = CommitAuthenticatedPairwise (
    neighbor,
    securityCount,
    keyId,
    sequence,
    securityCountChanged);
  result.status = status;

  std::span<const uint8_t> protectedPayload = authenticatedRecord.subspan (
    CSR_PAIRWISE_KEY_SEQUENCE_SIZE);
  result.payload.assign (protectedPayload.begin (), protectedPayload.end ());
  if (PairwiseModeEncryptsPayload (mode) && !result.payload.empty ())
    {
      CsrLegacyCrypto::ApplyAes256Ctr (keys.encryption,
                                       source,
                                       m_nodeId,
                                       securityCount,
                                       keyId,
                                       sequence,
                                       result.payload);
    }
  return result;
}

CsrProtectedGroupMessage
CsrHopSecurityState::ProtectGroupMessage (
  CsrGroupSecurityMode mode,
  uint8_t legacyPacketType,
  std::span<const uint8_t> payload)
{
  bool generatedNewGroupKey = NextGroupKeySequence ();

  CsrLegacyCrypto::ProtectionKeys keys =
    CsrLegacyCrypto::DeriveProtectionKeys (m_missionKey,
                                           m_groupKeyMaterial,
                                           m_nodeId,
                                           m_ownSecurityCount,
                                           m_groupKeyId);

  std::vector<uint8_t> protectedPayload (payload.begin (), payload.end ());
  if (GroupModeEncryptsPayload (mode) && !protectedPayload.empty ())
    {
      CsrLegacyCrypto::ApplyAes256Ctr (keys.encryption,
                                       m_nodeId,
                                       0,
                                       m_ownSecurityCount,
                                       m_groupKeyId,
                                       m_groupSequence,
                                       protectedPayload);
    }

  std::array<uint8_t, CSR_GROUP_KEY_SEQUENCE_SIZE> packed =
    SerializePackedKeySequence (m_groupKeyId, m_groupSequence);
  std::vector<uint8_t> record (packed.begin (), packed.end ());
  record.insert (record.end (),
                 protectedPayload.begin (),
                 protectedPayload.end ());

  if (mode == CsrGroupSecurityMode::GroupEstablish)
    {
      std::array<uint8_t, 4> tag =
        CsrLegacyCrypto::ComputeGroupEstablishAuthTag (
          m_missionKey,
          &keys.authentication,
          legacyPacketType,
          m_nodeId,
          m_ownSecurityCount,
          m_groupKeyId,
          m_groupSequence,
          protectedPayload);
      record.insert (record.end (), tag.begin (), tag.end ());
    }
  else
    {
      std::size_t authLength = GetGroupAuthTagSize (mode);
      std::vector<uint8_t> tag = CsrLegacyCrypto::ComputeGroupAuthTag (
        keys.authentication,
        m_nodeId,
        legacyPacketType,
        record,
        authLength);
      record.insert (record.end (), tag.begin (), tag.end ());
    }

  return {
    m_groupKeyId,
    m_groupSequence,
    generatedNewGroupKey,
    std::move (record)
  };
}

CsrReceivedGroupMessage
CsrHopSecurityState::ReceiveGroupMessage (
  CsrNodeId source,
  uint16_t securityCount,
  CsrGroupSecurityMode mode,
  uint8_t legacyPacketType,
  std::span<const uint8_t> record)
{
  CsrReceivedGroupMessage result;
  std::size_t authLength = GetGroupAuthTagSize (mode);

  if (!CsrIsValidNodeId (source) || source == CSR_BROADCAST_ID ||
      authLength == 0 ||
      record.size () < CSR_GROUP_KEY_SEQUENCE_SIZE + authLength)
    {
      return result;
    }

  uint16_t groupKeyId = static_cast<uint16_t> (
    (static_cast<uint16_t> (record[0]) << 4) | (record[1] >> 4));
  uint16_t groupSequence = static_cast<uint16_t> (
    (static_cast<uint16_t> (record[1] & 0x0f) << 8) | record[2]);
  result.groupKeyId = groupKeyId;
  result.groupSequence = groupSequence;

  std::size_t payloadLength =
    record.size () - CSR_GROUP_KEY_SEQUENCE_SIZE - authLength;
  std::span<const uint8_t> protectedPayload = record.subspan (
    CSR_GROUP_KEY_SEQUENCE_SIZE,
    payloadLength);
  std::span<const uint8_t> receivedTag = record.last (authLength);

  NeighborState *neighbor = nullptr;
  auto neighborIt = m_neighbors.find (source);
  if (neighborIt != m_neighbors.end ())
    {
      neighbor = &neighborIt->second;
    }

  bool securityCountChanged =
    neighbor != nullptr && neighbor->securityCountValid &&
    neighbor->securityCount != securityCount;
  if (securityCountChanged &&
      !IsSecurityCountChangeValid (neighbor->securityCount, securityCount))
    {
      result.status = CsrHopSecurityReceiveStatus::DuplicateOrStale;
      return result;
    }

  if (mode == CsrGroupSecurityMode::GroupEstablish)
    {
      std::array<uint8_t, 4> missionTag =
        CsrLegacyCrypto::ComputeGroupEstablishAuthTag (
          m_missionKey,
          nullptr,
          legacyPacketType,
          source,
          securityCount,
          groupKeyId,
          groupSequence,
          protectedPayload);
      if (!CsrLegacyCrypto::ConstantTimeEqual (
            std::span<const uint8_t> (missionTag.data (), 2),
            receivedTag.first (2)))
        {
          result.status = CsrHopSecurityReceiveStatus::AuthenticationFailed;
          return result;
        }

      neighbor = &GetOrCreateNeighbor (source);
      if (securityCountChanged)
        {
          ResetInboundForSecurityCountChange (*neighbor);
        }
    }
  else if (neighbor == nullptr || securityCountChanged)
    {
      result.status = CsrHopSecurityReceiveStatus::GroupKeyUnavailable;
      return result;
    }

  GroupKeyMaterial candidateGroupKey {};
  bool haveCandidate = false;
  bool candidateIsDerived = false;

  if (neighbor->groupKeyReceived &&
      groupKeyId == neighbor->lastGroupKeyIdReceived)
    {
      candidateGroupKey = neighbor->groupKeyMaterial;
      haveCandidate = true;
    }
  else if (neighbor->previousGroupKeyReceived &&
           groupKeyId == neighbor->previousGroupKeyIdReceived)
    {
      candidateGroupKey = neighbor->previousGroupKeyMaterial;
      haveCandidate = true;
    }
  else if (neighbor->groupKeyReceived &&
           groupKeyId > neighbor->lastGroupKeyIdReceived &&
           groupKeyId / CSR_GROUP_KEY_ID_STEP_SIZE ==
             neighbor->lastGroupKeyIdReceived / CSR_GROUP_KEY_ID_STEP_SIZE)
    {
      candidateGroupKey = neighbor->groupKeyMaterial;
      for (uint16_t keyId = static_cast<uint16_t> (
             neighbor->lastGroupKeyIdReceived + 1);
           keyId <= groupKeyId;
           ++keyId)
        {
          candidateGroupKey =
            CsrLegacyCrypto::DeriveNextGroupKeyMaterial (
              m_missionKey,
              candidateGroupKey,
              source,
              securityCount,
              keyId);
        }
      haveCandidate = true;
      candidateIsDerived = true;
    }

  if (!haveCandidate)
    {
      if (mode == CsrGroupSecurityMode::GroupEstablish)
        {
          if (neighbor->groupKeyReceived)
            {
              neighbor->previousGroupKeyReceived = true;
              neighbor->previousGroupKeyIdReceived =
                neighbor->lastGroupKeyIdReceived;
              neighbor->previousGroupKeyMaterial =
                neighbor->groupKeyMaterial;
              neighbor->groupKeyReceived = false;
            }
          neighbor->securityCountValid = true;
          neighbor->securityCount = securityCount;
          neighbor->lastGroupKeyIdReceived = groupKeyId;
          result.status = securityCountChanged
            ? CsrHopSecurityReceiveStatus::
                AuthenticatedGroupKeyNeededSecurityCountChanged
            : CsrHopSecurityReceiveStatus::AuthenticatedGroupKeyNeeded;
        }
      else
        {
          result.status = CsrHopSecurityReceiveStatus::GroupKeyUnavailable;
        }
      return result;
    }

  CsrLegacyCrypto::ProtectionKeys keys =
    CsrLegacyCrypto::DeriveProtectionKeys (m_missionKey,
                                           candidateGroupKey,
                                           source,
                                           securityCount,
                                           groupKeyId);

  bool authenticated = false;
  if (mode == CsrGroupSecurityMode::GroupEstablish)
    {
      std::array<uint8_t, 4> expected =
        CsrLegacyCrypto::ComputeGroupEstablishAuthTag (
          m_missionKey,
          &keys.authentication,
          legacyPacketType,
          source,
          securityCount,
          groupKeyId,
          groupSequence,
          protectedPayload);
      authenticated = CsrLegacyCrypto::ConstantTimeEqual (expected,
                                                          receivedTag);
    }
  else
    {
      std::span<const uint8_t> authenticatedRecord =
        record.first (record.size () - authLength);
      std::vector<uint8_t> expected =
        CsrLegacyCrypto::ComputeGroupAuthTag (keys.authentication,
                                              source,
                                              legacyPacketType,
                                              authenticatedRecord,
                                              authLength);
      authenticated = CsrLegacyCrypto::ConstantTimeEqual (expected,
                                                          receivedTag);
    }

  if (!authenticated)
    {
      result.status = CsrHopSecurityReceiveStatus::AuthenticationFailed;
      return result;
    }

  uint32_t replayValue =
    (static_cast<uint32_t> (groupKeyId) << 12) | groupSequence;
  if (neighbor->inboundGroup.IsBeforeWindow (replayValue))
    {
      result.status = CsrHopSecurityReceiveStatus::DuplicateOrStale;
      return result;
    }
  if (neighbor->inboundGroup.IsDuplicate (replayValue))
    {
      result.status = CsrHopSecurityReceiveStatus::AuthenticatedDuplicate;
      return result;
    }

  result.payload.assign (protectedPayload.begin (), protectedPayload.end ());
  if (GroupModeEncryptsPayload (mode) && !result.payload.empty ())
    {
      CsrLegacyCrypto::ApplyAes256Ctr (keys.encryption,
                                       source,
                                       0,
                                       securityCount,
                                       groupKeyId,
                                       groupSequence,
                                       result.payload);
    }

  if (candidateIsDerived)
    {
      neighbor->previousGroupKeyReceived = neighbor->groupKeyReceived;
      neighbor->previousGroupKeyIdReceived = neighbor->lastGroupKeyIdReceived;
      neighbor->previousGroupKeyMaterial = neighbor->groupKeyMaterial;
      neighbor->groupKeyReceived = true;
      neighbor->lastGroupKeyIdReceived = groupKeyId;
      neighbor->groupKeyMaterial = candidateGroupKey;
    }

  neighbor->inboundGroup.MarkReceived (replayValue);
  neighbor->securityCountValid = true;
  neighbor->securityCount = securityCount;
  result.status = securityCountChanged
    ? CsrHopSecurityReceiveStatus::AcceptedSecurityCountChanged
    : CsrHopSecurityReceiveStatus::Accepted;
  return result;
}

bool
CsrHopSecurityState::IsKeyUpdateSendActive (CsrNodeId neighbor) const
{
  const NeighborState *state = FindNeighbor (neighbor);
  return state != nullptr && state->keyUpdateSendActive;
}

bool
CsrHopSecurityState::HasGroupKeySentTo (CsrNodeId neighbor) const
{
  const NeighborState *state = FindNeighbor (neighbor);
  return state != nullptr && state->groupKeySent;
}

bool
CsrHopSecurityState::HasGroupKeyReceivedFrom (CsrNodeId neighbor) const
{
  const NeighborState *state = FindNeighbor (neighbor);
  return state != nullptr && state->groupKeyReceived;
}

bool
CsrHopSecurityState::GetGroupKeySentWhen (
  CsrNodeId neighbor,
  Time &when) const
{
  const NeighborState *state = FindNeighbor (neighbor);

  if (state == nullptr || !state->groupKeySent)
    {
      return false;
    }

  when = state->groupKeySentWhen;
  return true;
}

void
CsrHopSecurityState::MarkKeyUpdateAcked (CsrNodeId neighbor, Time when)
{
  NeighborState &state = GetOrCreateNeighbor (neighbor);
  state.keyUpdateSendActive = false;
  state.groupKeySent = true;
  state.groupKeySentWhen = when;
}

void
CsrHopSecurityState::MarkKeyUpdateFailed (CsrNodeId neighbor)
{
  NeighborState &state = GetOrCreateNeighbor (neighbor);
  state.keyUpdateSendActive = false;
}

void
CsrHopSecurityState::ResetNeighbor (CsrNodeId neighbor)
{
  m_neighbors.erase (neighbor);
}

bool
CsrHopSecurityState::ReplayWindow::IsBeforeWindow (uint32_t value) const
{
  return value < leftEdge;
}

bool
CsrHopSecurityState::ReplayWindow::IsDuplicate (uint32_t value) const
{
  if (IsBeforeWindow (value))
    {
      return true;
    }
  if (value - leftEdge >= WINDOW_SIZE)
    {
      return false;
    }
  return received[value & (WINDOW_SIZE - 1)];
}

void
CsrHopSecurityState::ReplayWindow::MarkReceived (uint32_t value)
{
  if (IsBeforeWindow (value))
    {
      return;
    }

  if (value - leftEdge >= WINDOW_SIZE)
    {
      uint32_t newLeftEdge = value - WINDOW_SIZE + 1;
      uint32_t shift = newLeftEdge - leftEdge;
      if (shift >= WINDOW_SIZE)
        {
          received.fill (false);
        }
      else
        {
          for (uint32_t old = leftEdge; old < newLeftEdge; ++old)
            {
              received[old & (WINDOW_SIZE - 1)] = false;
            }
        }
      leftEdge = newLeftEdge;
    }

  received[value & (WINDOW_SIZE - 1)] = true;
}

CsrHopSecurityState::NeighborState&
CsrHopSecurityState::GetOrCreateNeighbor (CsrNodeId neighbor)
{
  NS_ABORT_MSG_IF (!CsrIsValidNodeId (neighbor) ||
                   neighbor == CSR_BROADCAST_ID,
                   "CSR hop-security neighbor must be a unicast node");
  return m_neighbors[neighbor];
}

const CsrHopSecurityState::NeighborState*
CsrHopSecurityState::FindNeighbor (CsrNodeId neighbor) const
{
  auto it = m_neighbors.find (neighbor);
  return it == m_neighbors.end () ? nullptr : &it->second;
}

void
CsrHopSecurityState::NextPairwiseKeySequence (
  NeighborState &neighbor,
  uint16_t &keyId,
  uint16_t &sequence)
{
  if (neighbor.outboundPairwiseSequence == CSR_SECURITY_COUNTER_MAX)
    {
      neighbor.outboundPairwiseSequence = 0;
      neighbor.outboundPairwiseKeyId = static_cast<uint16_t> (
        (neighbor.outboundPairwiseKeyId + 1) & CSR_SECURITY_COUNTER_MAX);
    }
  else
    {
      neighbor.outboundPairwiseSequence++;
    }

  keyId = neighbor.outboundPairwiseKeyId;
  sequence = neighbor.outboundPairwiseSequence;
}

bool
CsrHopSecurityState::NextGroupKeySequence ()
{
  if (m_groupSequence != CSR_SECURITY_COUNTER_MAX)
    {
      m_groupSequence++;
      return false;
    }

  m_groupSequence = 0;
  m_groupKeyId = static_cast<uint16_t> (
    (m_groupKeyId + 1) & CSR_SECURITY_COUNTER_MAX);

  bool generatedNewGroupKey =
    (m_groupKeyId % CSR_GROUP_KEY_ID_STEP_SIZE) == 0;
  if (generatedNewGroupKey)
    {
      m_groupKeyMaterial = GenerateNewGroupKeyMaterial ();

      // setGroupKeySentUpdate() invalidates every neighbor's distribution
      // state when the source crosses a fresh-key epoch boundary.
      for (auto &item : m_neighbors)
        {
          item.second.groupKeySent = false;
        }
    }
  else
    {
      m_groupKeyMaterial = CsrLegacyCrypto::DeriveNextGroupKeyMaterial (
        m_missionKey,
        m_groupKeyMaterial,
        m_nodeId,
        m_ownSecurityCount,
        m_groupKeyId);
    }

  return generatedNewGroupKey;
}

CsrHopSecurityState::GroupKeyMaterial
CsrHopSecurityState::GenerateNewGroupKeyMaterial ()
{
  if (m_nextGeneratedGroupKeyMaterial.has_value ())
    {
      GroupKeyMaterial groupKey = *m_nextGeneratedGroupKeyMaterial;
      m_nextGeneratedGroupKeyMaterial.reset ();
      return groupKey;
    }

  // The radio obtains fresh bytes from its hardware noise source.  The
  // simulator substitutes a deterministic PBKDF2 stream so runs remain
  // reproducible while preserving the observable fresh-key transition.
  std::vector<uint8_t> salt {
    'C', 'S', 'R', '-', 'G', 'R', 'O', 'U', 'P', '-', 'K', 'E', 'Y',
    static_cast<uint8_t> (m_nodeId >> 16),
    static_cast<uint8_t> (m_nodeId >> 8),
    static_cast<uint8_t> (m_nodeId),
    static_cast<uint8_t> (m_ownSecurityCount >> 8),
    static_cast<uint8_t> (m_ownSecurityCount),
    static_cast<uint8_t> (m_groupKeyId >> 8),
    static_cast<uint8_t> (m_groupKeyId),
    static_cast<uint8_t> (m_groupKeyGenerationCounter >> 56),
    static_cast<uint8_t> (m_groupKeyGenerationCounter >> 48),
    static_cast<uint8_t> (m_groupKeyGenerationCounter >> 40),
    static_cast<uint8_t> (m_groupKeyGenerationCounter >> 32),
    static_cast<uint8_t> (m_groupKeyGenerationCounter >> 24),
    static_cast<uint8_t> (m_groupKeyGenerationCounter >> 16),
    static_cast<uint8_t> (m_groupKeyGenerationCounter >> 8),
    static_cast<uint8_t> (m_groupKeyGenerationCounter)
  };
  m_groupKeyGenerationCounter++;

  std::vector<uint8_t> bytes = CsrLegacyCrypto::Pbkdf2HmacSha1 (
    m_missionKey,
    salt,
    1,
    CsrLegacyCrypto::GROUP_KEY_MATERIAL_SIZE);
  GroupKeyMaterial groupKey {};
  std::copy (bytes.begin (), bytes.end (), groupKey.begin ());
  std::fill (bytes.begin (), bytes.end (), 0);
  return groupKey;
}

CsrHopSecurityReceiveStatus
CsrHopSecurityState::CommitAuthenticatedPairwise (
  NeighborState &neighbor,
  uint16_t securityCount,
  uint16_t keyId,
  uint16_t sequence,
  bool securityCountChanged)
{
  RequireTwelveBit (keyId, "CSR received pairwise key identifier");
  RequireTwelveBit (sequence, "CSR received pairwise sequence");

  if (securityCountChanged)
    {
      ResetInboundForSecurityCountChange (neighbor);
    }

  uint32_t value = (static_cast<uint32_t> (keyId) << 12) | sequence;
  if (neighbor.inboundPairwise.IsDuplicate (value))
    {
      return CsrHopSecurityReceiveStatus::AuthenticatedDuplicate;
    }

  neighbor.inboundPairwise.MarkReceived (value);
  neighbor.securityCountValid = true;
  neighbor.securityCount = securityCount;

  return securityCountChanged
    ? CsrHopSecurityReceiveStatus::AcceptedSecurityCountChanged
    : CsrHopSecurityReceiveStatus::Accepted;
}

bool
CsrHopSecurityState::IsSecurityCountChangeValid (
  uint16_t current,
  uint16_t received) const
{
  return static_cast<uint16_t> (received - current) <= 32768;
}

void
CsrHopSecurityState::ResetInboundForSecurityCountChange (
  NeighborState &neighbor)
{
  uint16_t outboundKeyId = neighbor.outboundPairwiseKeyId;
  uint16_t outboundSequence = neighbor.outboundPairwiseSequence;
  neighbor = NeighborState {};
  neighbor.outboundPairwiseKeyId = outboundKeyId;
  neighbor.outboundPairwiseSequence = outboundSequence;
}

const CsrHopSecurityState::PairwiseKeyMaterial&
CsrHopSecurityState::GetPairwiseKeyMaterial (CsrNodeId neighbor) const
{
  auto it = m_pairwiseKeyMaterial.find (neighbor);
  return it == m_pairwiseKeyMaterial.end ()
    ? m_defaultPairwiseKeyMaterial
    : it->second;
}

CsrLegacyCrypto::ProtectionKeys
CsrHopSecurityState::DerivePairwiseProtectionKeys (
  CsrNodeId peer,
  CsrNodeId source,
  uint16_t securityCount,
  uint16_t keyId) const
{
  return CsrLegacyCrypto::DeriveProtectionKeys (
    m_missionKey,
    GetPairwiseKeyMaterial (peer),
    source,
    securityCount,
    keyId);
}

CsrHopSecurityState::CryptoPrn
CsrHopSecurityState::GenerateDataPrn ()
{
  if (m_nextDataPrn.has_value ())
    {
      CryptoPrn prn = *m_nextDataPrn;
      m_nextDataPrn.reset ();
      return prn;
    }

  std::array<uint8_t, 15> seed {
    static_cast<uint8_t> (m_nodeId >> 16),
    static_cast<uint8_t> (m_nodeId >> 8),
    static_cast<uint8_t> (m_nodeId),
    static_cast<uint8_t> (m_ownSecurityCount >> 8),
    static_cast<uint8_t> (m_ownSecurityCount),
    static_cast<uint8_t> (m_groupKeyId >> 8),
    static_cast<uint8_t> (m_groupKeyId),
    static_cast<uint8_t> (m_prnCounter >> 56),
    static_cast<uint8_t> (m_prnCounter >> 48),
    static_cast<uint8_t> (m_prnCounter >> 40),
    static_cast<uint8_t> (m_prnCounter >> 32),
    static_cast<uint8_t> (m_prnCounter >> 24),
    static_cast<uint8_t> (m_prnCounter >> 16),
    static_cast<uint8_t> (m_prnCounter >> 8),
    static_cast<uint8_t> (m_prnCounter)
  };
  m_prnCounter++;
  CsrLegacyCrypto::Sha1Digest digest =
    CsrLegacyCrypto::HmacSha1 (m_missionKey, seed);
  CryptoPrn prn {};
  std::copy_n (digest.begin (), prn.size (), prn.begin ());
  return prn;
}

} // namespace ns3
