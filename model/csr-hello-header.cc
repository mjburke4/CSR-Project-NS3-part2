#include "csr-hello-header.h"
#include "ns3/uinteger.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (CsrHelloHeader);

CsrHelloHeader::CsrHelloHeader () = default;

TypeId
CsrHelloHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::CsrHelloHeader")
    .SetParent<Header> ()
    .SetGroupName ("Csr")
    .AddConstructor<CsrHelloHeader> ();
  return tid;
}

TypeId
CsrHelloHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

void CsrHelloHeader::SetNodeId (uint16_t id) { m_nodeId = id; }
uint16_t CsrHelloHeader::GetNodeId () const { return m_nodeId; }

void CsrHelloHeader::SetHelloSeq (uint16_t s) { m_helloSeq = s; }
uint16_t CsrHelloHeader::GetHelloSeq () const { return m_helloSeq; }

void CsrHelloHeader::SetSpeedKey (uint8_t k) { m_speedKey = k; }
uint8_t CsrHelloHeader::GetSpeedKey () const { return m_speedKey; }

void CsrHelloHeader::SetRxPowerDbmX10 (int16_t p) { m_rxPowerDbmX10 = p; }
int16_t CsrHelloHeader::GetRxPowerDbmX10 () const { return m_rxPowerDbmX10; }

void CsrHelloHeader::SetActiveNodes (uint8_t n) { m_activeNodes = n; }
uint8_t CsrHelloHeader::GetActiveNodes () const { return m_activeNodes; }

void CsrHelloHeader::SetAdvertisedDst (uint16_t dst) { m_advDst = dst; }
uint16_t CsrHelloHeader::GetAdvertisedDst () const { return m_advDst; }

void CsrHelloHeader::SetAdvertisedHops (uint8_t hops) { m_advHops = hops; }
uint8_t CsrHelloHeader::GetAdvertisedHops () const { return m_advHops; }

uint32_t
CsrHelloHeader::GetSerializedSize () const
{
  // nodeId, helloSeq, speedKey, rxPowerDbmX10, activeNodes, advDst, advHops
  return 2 + 2 + 1 + 2 + 1 + 2 + 1;
}

void
CsrHelloHeader::Serialize (Buffer::Iterator i) const
{
  i.WriteHtonU16 (m_nodeId);
  i.WriteHtonU16 (m_helloSeq);
  i.WriteU8 (m_speedKey);
  i.WriteHtonU16 (static_cast<uint16_t>(m_rxPowerDbmX10));
  i.WriteU8 (m_activeNodes);
  i.WriteHtonU16 (m_advDst);
  i.WriteU8 (m_advHops);
}

uint32_t
CsrHelloHeader::Deserialize (Buffer::Iterator i)
{
  m_nodeId = i.ReadNtohU16 ();
  m_helloSeq = i.ReadNtohU16 ();
  m_speedKey = i.ReadU8 ();
  m_rxPowerDbmX10 = static_cast<int16_t>(i.ReadNtohU16 ());
  m_activeNodes = i.ReadU8 ();
  m_advDst = i.ReadNtohU16 ();
  m_advHops = i.ReadU8 ();
  return GetSerializedSize ();
}

void
CsrHelloHeader::Print (std::ostream &os) const
{
  os << "nodeId=" << m_nodeId
     << " helloSeq=" << m_helloSeq
     << " speedKey=" << unsigned(m_speedKey)
     << " rxPwr(dBm*10)=" << m_rxPowerDbmX10
     << " activeNodes=" << unsigned(m_activeNodes)
     << " advDst=" << m_advDst
     << " advHops=" << unsigned(m_advHops);
}

} // namespace ns3
