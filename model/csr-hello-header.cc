#include "csr-hello-header.h"
#include "ns3/uinteger.h"
#include <algorithm>

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

  void
  CsrHelloHeader::SetArlRouteMsgType (CsrArlRouteMsgType t)
  {
    m_arlRouteMsgType = static_cast<uint8_t> (t);
  }

  CsrArlRouteMsgType
  CsrHelloHeader::GetArlRouteMsgType () const
  {
    return static_cast<CsrArlRouteMsgType> (m_arlRouteMsgType);
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

  uint32_t
  CsrHelloHeader::GetSerializedSize () const
  {
    // base:
    // nodeId(2), helloSeq(2), speedKey(1), rxPowerDbmX10(2), activeNodes(1)
    // routeCount(1)
    // each advertised route:
    // dst(2), hops(1), cost(4), pathlossDbX10(2), capability(1)
    return 2 + 2 + 1 + 2 + 1 + 1 + 1
        + static_cast<uint32_t> (m_advertisedRoutes.size ()) * (2 + 1 + 4 + 2 + 1);
  }

  void
  CsrHelloHeader::Serialize (Buffer::Iterator i) const
  {
    i.WriteHtonU16 (m_nodeId);
    i.WriteHtonU16 (m_helloSeq);
    i.WriteU8 (m_speedKey);
    i.WriteHtonU16 (static_cast<uint16_t> (m_rxPowerDbmX10));
    i.WriteU8 (m_activeNodes);
    i.WriteU8 (m_arlRouteMsgType);

    uint8_t count = static_cast<uint8_t> (
        std::min<size_t> (m_advertisedRoutes.size (), MAX_ADVERTISED_ROUTES));

    i.WriteU8 (count);

    for (uint8_t idx = 0; idx < count; ++idx)
      {
        const auto &ar = m_advertisedRoutes[idx];

        i.WriteHtonU16 (ar.dst);
        i.WriteU8 (ar.hops);
        i.WriteHtonU32 (ar.cost);
        i.WriteHtonU16 (static_cast<uint16_t> (ar.pathlossDbX10));
        i.WriteU8 (ar.capability);
      }
  }

  uint32_t
  CsrHelloHeader::Deserialize (Buffer::Iterator i)
  {
    m_nodeId = i.ReadNtohU16 ();
    m_helloSeq = i.ReadNtohU16 ();
    m_speedKey = i.ReadU8 ();
    m_rxPowerDbmX10 = static_cast<int16_t> (i.ReadNtohU16 ());
    m_activeNodes = i.ReadU8 ();
    m_arlRouteMsgType = i.ReadU8 ();

    m_advertisedRoutes.clear ();

    uint8_t count = i.ReadU8 ();
    if (count > MAX_ADVERTISED_ROUTES)
      {
        count = MAX_ADVERTISED_ROUTES;
      }

    for (uint8_t idx = 0; idx < count; ++idx)
      {
        AdvertisedRoute ar;
        ar.dst = i.ReadNtohU16 ();
        ar.hops = i.ReadU8 ();
        ar.cost = i.ReadNtohU32 ();
        ar.pathlossDbX10 = static_cast<int16_t> (i.ReadNtohU16 ());
        ar.capability = i.ReadU8 ();

        m_advertisedRoutes.push_back (ar);
      }

    return GetSerializedSize ();
  }

  void
  CsrHelloHeader::Print (std::ostream &os) const
  {
    os << "nodeId=" << m_nodeId
      << " helloSeq=" << m_helloSeq
      << " speedKey=" << unsigned (m_speedKey)
      << " rxPwr(dBm*10)=" << m_rxPowerDbmX10
      << " activeNodes=" << unsigned (m_activeNodes)
      << " arlType=" << unsigned (m_arlRouteMsgType)
      << " advRoutes=" << unsigned (GetAdvertisedRouteCount ());
  }

  void
  CsrHelloHeader::ClearAdvertisedRoutes ()
  {
    m_advertisedRoutes.clear ();
  }

  bool
  CsrHelloHeader::AddAdvertisedRoute (uint16_t dst,
                                      uint8_t hops,
                                      uint32_t cost,
                                      int16_t pathlossDbX10,
                                      uint8_t capability)
  {
    if (m_advertisedRoutes.size () >= MAX_ADVERTISED_ROUTES)
      {
        return false;
      }

    if (dst == 0xFFFF)
      {
        return false;
      }

    AdvertisedRoute ar;
    ar.dst = dst;
    ar.hops = hops;
    ar.cost = cost;
    ar.pathlossDbX10 = pathlossDbX10;
    ar.capability = capability;

    m_advertisedRoutes.push_back (ar);
    return true;
  }

  uint8_t
  CsrHelloHeader::GetAdvertisedRouteCount () const
  {
    return static_cast<uint8_t> (m_advertisedRoutes.size ());
  }

  CsrHelloHeader::AdvertisedRoute
  CsrHelloHeader::GetAdvertisedRoute (uint8_t index) const
  {
    if (index >= m_advertisedRoutes.size ())
      {
        return AdvertisedRoute {};
      }

    return m_advertisedRoutes[index];
  }

} // namespace ns3
