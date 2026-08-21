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

  void
  CsrHelloHeader::SetNeighborCheckType (CsrNeighborCheckType t)
  {
    m_neighborCheckType = static_cast<uint8_t> (t);
  }

  CsrNeighborCheckType
  CsrHelloHeader::GetNeighborCheckType () const
  {
    return static_cast<CsrNeighborCheckType> (m_neighborCheckType);
  }

  void
  CsrHelloHeader::SetNeighborCheckTarget (CsrNodeId target)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (target),
                     "CSR neighbor-check target exceeds 24 bits");
    m_neighborCheckTarget = target;
  }

  CsrNodeId
  CsrHelloHeader::GetNeighborCheckTarget () const
  {
    return m_neighborCheckTarget;
  }

  void
  CsrHelloHeader::SetDiscoverType (CsrDiscoverType type)
  {
    m_discoverType = static_cast<uint8_t> (type);
  }

  CsrDiscoverType
  CsrHelloHeader::GetDiscoverType () const
  {
    return static_cast<CsrDiscoverType> (m_discoverType);
  }

  void
  CsrHelloHeader::SetDiscoverySequence (uint32_t sequence)
  {
    m_discoverySequence = sequence;
  }

  uint32_t
  CsrHelloHeader::GetDiscoverySequence () const
  {
    return m_discoverySequence;
  }

  void
  CsrHelloHeader::SetNodeType (CsrNodeType type)
  {
    m_nodeType = static_cast<uint8_t> (type);
  }

  CsrNodeType
  CsrHelloHeader::GetNodeType () const
  {
    return static_cast<CsrNodeType> (m_nodeType);
  }

  CsrArlRouteMsgType
  CsrHelloHeader::GetArlRouteMsgType () const
  {
    return static_cast<CsrArlRouteMsgType> (m_arlRouteMsgType);
  }

  void
  CsrHelloHeader::SetNodeId (CsrNodeId id)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (id),
                     "CSR HELLO node identifier exceeds 24 bits");
    m_nodeId = id;
  }

  CsrNodeId CsrHelloHeader::GetNodeId () const { return m_nodeId; }

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
    uint32_t size =
        3  // nodeId
      + 2  // helloSeq
      + 1  // speedKey
      + 2  // rxPowerDbmX10
      + 1  // activeNodes
      + 1  // nodeType
      + 1  // arlRouteMsgType
      + 1  // neighborCheckType
      + 3  // neighborCheckTarget
      + 1  // discoverType
      + 4  // discoverySequence
      + 4  // routingSequence
      + 1  // routingSection
      + 1  // routingTotalSections
      + 1  // routingOperation
      + 3  // routingTarget
      + 1  // chirpNeighborCount
      + static_cast<uint32_t> (
          m_chirpNeighbors.size ()) * 3
      + 1; // routeCount

    uint8_t routeCount =
      static_cast<uint8_t> (
        std::min<size_t> (
          m_advertisedRoutes.size (),
          MAX_ADVERTISED_ROUTES));

    for (uint8_t index = 0;
        index < routeCount;
        ++index)
      {
        const AdvertisedRoute &route =
          m_advertisedRoutes[index];

        uint8_t pathCount =
          static_cast<uint8_t> (
            std::min<size_t> (
              route.path.size (),
              CSR_MAX_ROUTE_PATH_HOPS));

        size +=
            3  // destination
          + 1  // hops
          + 4  // cost
          + 2  // pathloss
          + 1  // capability
          + 1  // pathCount
          + static_cast<uint32_t> (
              pathCount) * 3;
      }

    if (GetRoutingOperation () ==
        CsrRoutingOperation::Info)
      {
        size += 16;
      }

    return size;
  }

  void
  CsrHelloHeader::Serialize (Buffer::Iterator i) const
  {
    CsrWriteNodeId (i, m_nodeId);
    i.WriteHtonU16 (m_helloSeq);
    i.WriteU8 (m_speedKey);
    i.WriteHtonU16 (static_cast<uint16_t> (m_rxPowerDbmX10));
    i.WriteU8 (m_activeNodes);
    i.WriteU8 (m_nodeType);
    i.WriteU8 (m_arlRouteMsgType);
    i.WriteU8 (m_neighborCheckType);
    CsrWriteNodeId (i, m_neighborCheckTarget);
    i.WriteU8 (m_discoverType);
    i.WriteHtonU32 (m_discoverySequence);
    i.WriteHtonU32 (m_routingSequence);
    i.WriteU8 (m_routingSection);
    i.WriteU8 (m_routingTotalSections);
    i.WriteU8 (m_routingOperation);
    CsrWriteNodeId (i, m_routingTarget);

    if (GetRoutingOperation () ==
        CsrRoutingOperation::Info)
      {
        i.WriteHtonU16 (
          m_routingInfo.minSpeedKbps);

        i.WriteHtonU16 (
          m_routingInfo.maxSpeedKbps);

        i.WriteHtonU16 (
          static_cast<uint16_t> (
            m_routingInfo.minPowerDbmX10));

        i.WriteHtonU16 (
          static_cast<uint16_t> (
            m_routingInfo.maxPowerDbmX10));

        i.WriteHtonU16 (
          static_cast<uint16_t> (
            m_routingInfo.linkMarginDbX10));

        i.WriteHtonU16 (
          static_cast<uint16_t> (
            m_routingInfo.lowPowerDbmX10));

        i.WriteHtonU16 (
          static_cast<uint16_t> (
            m_routingInfo.tempLowCx10));

        i.WriteHtonU16 (
          static_cast<uint16_t> (
            m_routingInfo.tempHighCx10));
      }

    uint8_t chirpCount = static_cast<uint8_t> (
    std::min<size_t> (m_chirpNeighbors.size (),
                    MAX_CHIRP_NEIGHBORS));

    i.WriteU8 (chirpCount);

    for (uint8_t index = 0; index < chirpCount; ++index)
      {
        CsrWriteNodeId (i, m_chirpNeighbors[index]);
      }

    uint8_t count = static_cast<uint8_t> (
        std::min<size_t> (m_advertisedRoutes.size (), MAX_ADVERTISED_ROUTES));

    i.WriteU8 (count);

    for (uint8_t idx = 0; idx < count; ++idx)
      {
        const auto &ar = m_advertisedRoutes[idx];

        CsrWriteNodeId (i, ar.dst);
        i.WriteU8 (ar.hops);
        i.WriteHtonU32 (ar.cost);
        i.WriteHtonU16 (static_cast<uint16_t> (ar.pathlossDbX10));
        i.WriteU8 (ar.capability);

        uint8_t pathCount =
          static_cast<uint8_t> (
            std::min<size_t> (
              ar.path.size (),
              CSR_MAX_ROUTE_PATH_HOPS));

        i.WriteU8 (pathCount);

        for (uint8_t pathIndex = 0;
            pathIndex < pathCount;
            ++pathIndex)
          {
            CsrWriteNodeId (i, ar.path[pathIndex]);
          }
      }
  }

  uint32_t
  CsrHelloHeader::Deserialize (Buffer::Iterator i)
  {
    Buffer::Iterator start = i;

    m_nodeId = CsrReadNodeId (i);
    m_helloSeq = i.ReadNtohU16 ();
    m_speedKey = i.ReadU8 ();
    m_rxPowerDbmX10 = static_cast<int16_t> (i.ReadNtohU16 ());
    m_activeNodes = i.ReadU8 ();
    m_nodeType = i.ReadU8 ();
    m_arlRouteMsgType = i.ReadU8 ();
    m_neighborCheckType = i.ReadU8 ();
    m_neighborCheckTarget = CsrReadNodeId (i);

    m_discoverType = i.ReadU8 ();
    m_discoverySequence = i.ReadNtohU32 ();
    m_routingSequence = i.ReadNtohU32 ();
    m_routingSection =
     i.ReadU8 ();

    m_routingTotalSections =
      i.ReadU8 ();

    if (m_routingTotalSections == 0)
      {
        m_routingTotalSections = 1;
      }
    m_routingOperation = i.ReadU8 ();
    m_routingTarget = CsrReadNodeId (i);

    m_routingInfo =
  RoutingInfo {};

  if (GetRoutingOperation () ==
      CsrRoutingOperation::Info)
    {
      m_routingInfo.minSpeedKbps =
        i.ReadNtohU16 ();

      m_routingInfo.maxSpeedKbps =
        i.ReadNtohU16 ();

      m_routingInfo.minPowerDbmX10 =
        static_cast<int16_t> (
          i.ReadNtohU16 ());

      m_routingInfo.maxPowerDbmX10 =
        static_cast<int16_t> (
          i.ReadNtohU16 ());

      m_routingInfo.linkMarginDbX10 =
        static_cast<int16_t> (
          i.ReadNtohU16 ());

      m_routingInfo.lowPowerDbmX10 =
        static_cast<int16_t> (
          i.ReadNtohU16 ());

      m_routingInfo.tempLowCx10 =
        static_cast<int16_t> (
          i.ReadNtohU16 ());

      m_routingInfo.tempHighCx10 =
        static_cast<int16_t> (
          i.ReadNtohU16 ());
    }

    m_chirpNeighbors.clear ();

    uint8_t chirpCount = i.ReadU8 ();

    for (uint8_t index = 0; index < chirpCount; ++index)
      {
        m_chirpNeighbors.push_back (CsrReadNodeId (i));
      }

    m_advertisedRoutes.clear ();

    uint8_t wireRouteCount =
      i.ReadU8 ();

    for (uint8_t routeIndex = 0;
        routeIndex < wireRouteCount;
        ++routeIndex)
      {
        AdvertisedRoute route;

        route.dst = CsrReadNodeId (i);

        route.hops =
          i.ReadU8 ();

        route.cost =
          i.ReadNtohU32 ();

        route.pathlossDbX10 =
          static_cast<int16_t> (
            i.ReadNtohU16 ());

        route.capability =
          i.ReadU8 ();

        uint8_t wirePathCount =
          i.ReadU8 ();

        for (uint8_t pathIndex = 0;
            pathIndex < wirePathCount;
            ++pathIndex)
          {
            CsrNodeId hop = CsrReadNodeId (i);

            if (pathIndex <
                  CSR_MAX_ROUTE_PATH_HOPS)
              {
                route.path.push_back (
                  hop);
              }
          }

        if (routeIndex <
              MAX_ADVERTISED_ROUTES)
          {
            m_advertisedRoutes.push_back (
              route);
          }
      }

    return i.GetDistanceFrom (start);
  }

  void
  CsrHelloHeader::Print (std::ostream &os) const
  {
    os << "nodeId=" << m_nodeId
      << " helloSeq=" << m_helloSeq
      << " speedKey=" << unsigned (m_speedKey)
      << " rxPwr(dBm*10)=" << m_rxPowerDbmX10
      << " activeNodes=" << unsigned (m_activeNodes)
      << " nodeType=" << unsigned (m_nodeType)
      << " arlType=" << unsigned (m_arlRouteMsgType)
      << " neighborCheckType=" << unsigned (m_neighborCheckType)
      << " neighborCheckTarget=" << m_neighborCheckTarget
      << " discoverType=" << unsigned (m_discoverType)
      << " discoverySequence=" << m_discoverySequence
      << " routingSequence=" << m_routingSequence
      << " routingSection="
      << unsigned (m_routingSection)
      << " routingTotalSections="
      << unsigned (m_routingTotalSections)
      << " chirpNeighbors=" << unsigned (GetChirpNeighborCount ())
      << " advRoutes=" << unsigned (GetAdvertisedRouteCount ())
      << " routingOperation=" << unsigned (m_routingOperation)
      << " routingTarget=" << m_routingTarget;
  }

  void
  CsrHelloHeader::ClearAdvertisedRoutes ()
  {
    m_advertisedRoutes.clear ();
  }

  bool
  CsrHelloHeader::AddAdvertisedRoute (
    CsrNodeId dst,
    uint8_t hops,
    uint32_t cost,
    int16_t pathlossDbX10,
    uint8_t capability,
    const std::vector<CsrNodeId> &path)
  {
    if (!CsrIsValidNodeId (dst))
      {
        return false;
      }

    for (CsrNodeId pathNode : path)
      {
        if (!CsrIsValidNodeId (pathNode))
          {
            return false;
          }
      }

    if (m_advertisedRoutes.size () >= MAX_ADVERTISED_ROUTES)
      {
        return false;
      }

    if (dst == CSR_BROADCAST_ID)
      {
        return false;
      }

    AdvertisedRoute ar;
    ar.dst = dst;
    ar.hops = hops;
    ar.cost = cost;
    ar.pathlossDbX10 = pathlossDbX10;
    ar.capability = capability;
    ar.path = path;

    if (ar.path.size () >
        CSR_MAX_ROUTE_PATH_HOPS)
      {
        ar.path.resize (
          CSR_MAX_ROUTE_PATH_HOPS);
      }

    m_advertisedRoutes.push_back (ar);
    return true;
  }

  void
  CsrHelloHeader::SetRoutingSequence (uint32_t sequence)
  {
    m_routingSequence = sequence;
  }

  uint32_t
  CsrHelloHeader::GetRoutingSequence () const
  {
    return m_routingSequence;
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

  void
  CsrHelloHeader::SetRoutingOperation (
    CsrRoutingOperation operation)
  {
    m_routingOperation =
      static_cast<uint8_t> (operation);
  }

  CsrRoutingOperation
  CsrHelloHeader::GetRoutingOperation () const
  {
    return static_cast<CsrRoutingOperation> (
      m_routingOperation);
  }

  void
  CsrHelloHeader::ClearChirpNeighbors ()
  {
    m_chirpNeighbors.clear ();
  }

  bool
  CsrHelloHeader::AddChirpNeighbor (CsrNodeId nodeId)
  {
    if (!CsrIsValidNodeId (nodeId) ||
        nodeId == CSR_BROADCAST_ID ||
        m_chirpNeighbors.size () >= MAX_CHIRP_NEIGHBORS)
      {
        return false;
      }

    if (std::find (m_chirpNeighbors.begin (),
                  m_chirpNeighbors.end (),
                  nodeId) != m_chirpNeighbors.end ())
      {
        return false;
      }

    m_chirpNeighbors.push_back (nodeId);
    return true;
  }

  uint8_t
  CsrHelloHeader::GetChirpNeighborCount () const
  {
    return static_cast<uint8_t> (m_chirpNeighbors.size ());
  }

  CsrNodeId
  CsrHelloHeader::GetChirpNeighbor (uint8_t index) const
  {
    if (index >= m_chirpNeighbors.size ())
      {
        return CSR_BROADCAST_ID;
      }

    return m_chirpNeighbors[index];
  }

  void
  CsrHelloHeader::SetRoutingTarget (
    CsrNodeId target)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (target),
                     "CSR routing target exceeds 24 bits");
    m_routingTarget = target;
  }

  CsrNodeId
  CsrHelloHeader::GetRoutingTarget () const
  {
    return m_routingTarget;
  }

  void
  CsrHelloHeader::SetRoutingSection (
    uint8_t section)
  {
    m_routingSection = section;
  }

  uint8_t
  CsrHelloHeader::GetRoutingSection () const
  {
    return m_routingSection;
  }

  void
  CsrHelloHeader::SetRoutingTotalSections (
    uint8_t totalSections)
  {
    m_routingTotalSections =
      std::max<uint8_t> (
        1,
        totalSections);
  }

  uint8_t
  CsrHelloHeader::GetRoutingTotalSections () const
  {
    return m_routingTotalSections;
  }

  void
  CsrHelloHeader::SetRoutingInfo (
    const RoutingInfo &info)
  {
    m_routingInfo = info;
  }

  CsrHelloHeader::RoutingInfo
  CsrHelloHeader::GetRoutingInfo () const
  {
    return m_routingInfo;
  }
} // namespace ns3
