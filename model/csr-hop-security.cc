#include "csr-hop-security.h"

#include "ns3/abort.h"

namespace ns3 {

namespace
{

static constexpr uint16_t CSR_SECURITY_COUNTER_MAX = 0x0fff;

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
  RequireTwelveBit (securityCount, "CSR security count");
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

  // The legacy non-SECURITY path zeroes this field.  Crypto can populate it
  // later without changing the seven-byte record or state transitions.
  header.SetAuthTag (CsrKeyRequestHeader::AuthTag {});
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

  // These fields match the legacy non-SECURITY build.  Their exact sizes and
  // positions are already final for a future AES/HMAC implementation.
  header.SetDataPrn (CsrKeyUpdateHeader::DataPrn {});
  header.SetWrappedKey (CsrKeyUpdateHeader::WrappedKey {});
  header.SetAuthTag (CsrKeyUpdateHeader::AuthTag {});
  return header;
}

CsrHopSecurityReceiveStatus
CsrHopSecurityState::ReceiveKeyRequest (
  CsrNodeId source,
  uint16_t securityCount,
  const CsrKeyRequestHeader &header)
{
  NeighborState &neighbor = GetOrCreateNeighbor (source);
  return AcceptPairwiseSequence (neighbor,
                                 securityCount,
                                 header.GetKeyId (),
                                 header.GetSequence ());
}

CsrHopSecurityReceiveStatus
CsrHopSecurityState::ReceiveKeyUpdate (
  CsrNodeId source,
  uint16_t securityCount,
  const CsrKeyUpdateHeader &header)
{
  NeighborState &neighbor = GetOrCreateNeighbor (source);
  CsrHopSecurityReceiveStatus status =
    AcceptPairwiseSequence (neighbor,
                            securityCount,
                            header.GetKeyId (),
                            header.GetSequence ());

  if (status == CsrHopSecurityReceiveStatus::DuplicateOrStale)
    {
      return status;
    }

  if (!neighbor.previousGroupKeyReceived ||
      neighbor.previousGroupKeyIdReceived !=
        neighbor.lastGroupKeyIdReceived)
    {
      neighbor.previousGroupKeyReceived = neighbor.groupKeyReceived;
      neighbor.previousGroupKeyIdReceived =
        neighbor.lastGroupKeyIdReceived;
    }

  neighbor.groupKeyReceived = true;
  neighbor.lastGroupKeyIdReceived = header.GetGroupKeyId ();
  return status;
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

CsrHopSecurityReceiveStatus
CsrHopSecurityState::AcceptPairwiseSequence (
  NeighborState &neighbor,
  uint16_t securityCount,
  uint16_t keyId,
  uint16_t sequence)
{
  RequireTwelveBit (securityCount, "CSR received security count");
  RequireTwelveBit (keyId, "CSR received pairwise key identifier");
  RequireTwelveBit (sequence, "CSR received pairwise sequence");

  bool securityCountChanged =
    neighbor.securityCountValid &&
    neighbor.securityCount != securityCount;

  if (securityCountChanged)
    {
      // Mirrors routesSecurityCountChange(): the old group-key relationship
      // and admission proof are invalid after a remote restart count change.
      neighbor = NeighborState {};
    }

  if (neighbor.inboundPairwiseValid)
    {
      if (keyId < neighbor.inboundPairwiseKeyId ||
          (keyId == neighbor.inboundPairwiseKeyId &&
           sequence <= neighbor.inboundPairwiseSequence))
        {
          return CsrHopSecurityReceiveStatus::DuplicateOrStale;
        }
    }

  neighbor.inboundPairwiseValid = true;
  neighbor.inboundPairwiseKeyId = keyId;
  neighbor.inboundPairwiseSequence = sequence;
  neighbor.securityCountValid = true;
  neighbor.securityCount = securityCount;

  return securityCountChanged
    ? CsrHopSecurityReceiveStatus::AcceptedSecurityCountChanged
    : CsrHopSecurityReceiveStatus::Accepted;
}

} // namespace ns3
