#pragma once
#include "csr-wire-format.h"
#include "ns3/header.h"
#include "ns3/buffer.h"
#include <vector>

namespace ns3 {

enum class CsrArlRouteMsgType : uint8_t
{
  None          = 0,
  Discover      = 1,
  RoutingUpdate = 2,
  NeighborCheck = 3,
  KeyRequest    = 4
};

enum class CsrNeighborCheckType : uint8_t
{
  Discovery = 0,  // CHECK_DISCOVERY
  Message   = 1,  // CHECK_MESSAGE
  NoPath    = 2,  // CHECK_NO_PATH
  Overheard = 3,  // CHECK_OVERHEARD
  Verify    = 4,  // CHECK_VERIFY
  None      = 0xFF
};

enum class CsrDiscoverType : uint8_t
{
  Broadcast = 0,  // DISCOVER_BROADCAST
  Chirp     = 1,  // DISCOVER_CHIRP
  None      = 0xFF
};

enum class CsrNodeType : uint8_t
{
  Ordinary = 0,  // NodeTypeORDINARY
  Routable = 1,  // NodeTypeROUTABLE
  Gateway  = 2   // NodeTypeGATEWAY
};

enum class CsrRoutingOperation : uint8_t
{
  Flush   = 0,  // ROUTING_FLUSH
  Delete  = 1,  // ROUTING_DELETE
  Update  = 2,  // ROUTING_UPDATE
  Request = 3,  // ROUTING_REQUEST
  Info    = 4,  // ROUTING_INFO
  None    = 0xFF
};

static constexpr uint8_t
  CSR_MAX_ROUTE_PATH_HOPS = 32;

class CsrHelloHeader : public Header
{
public:
  CsrHelloHeader ();

  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const override;

  // Setters/Getters
  void SetNodeId (CsrNodeId id);
  CsrNodeId GetNodeId () const;

  void SetHelloSeq (uint16_t s);
  uint16_t GetHelloSeq () const;

  void SetSpeedKey (CsrRateKey k);
  CsrRateKey GetSpeedKey () const;

  void SetRxPowerDbmX10 (int16_t p);   // dBm * 10 to keep it integer
  int16_t GetRxPowerDbmX10 () const;

  void SetActiveNodes (uint8_t n);
  uint8_t GetActiveNodes () const;

  void SetArlRouteMsgType (CsrArlRouteMsgType t);
  CsrArlRouteMsgType GetArlRouteMsgType () const;

  void SetNeighborCheckType (CsrNeighborCheckType t);
  CsrNeighborCheckType GetNeighborCheckType () const;

  void SetDiscoverType (CsrDiscoverType type);
  CsrDiscoverType GetDiscoverType () const;

  void SetDiscoverySequence (uint32_t sequence);
  uint32_t GetDiscoverySequence () const;

  // Header overrides
  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  void SetNeighborCheckTarget (CsrNodeId target);
  CsrNodeId GetNeighborCheckTarget () const;

  void ClearChirpNeighbors ();
  bool AddChirpNeighbor (CsrNodeId nodeId);
  uint8_t GetChirpNeighborCount () const;
  CsrNodeId GetChirpNeighbor (uint8_t index) const;

  void SetNodeType (CsrNodeType type);
  CsrNodeType GetNodeType () const;

  void SetRoutingSequence (uint32_t sequence);
  uint32_t GetRoutingSequence () const;

  void SetRoutingSection (uint8_t section);
  uint8_t GetRoutingSection () const;

  void SetRoutingTotalSections (
    uint8_t totalSections);
  uint8_t GetRoutingTotalSections () const;

  void SetRoutingOperation (CsrRoutingOperation operation);
  CsrRoutingOperation GetRoutingOperation () const;

  struct RoutingInfo
  {
    uint16_t minSpeedKbps {0};
    uint16_t maxSpeedKbps {0};

    int16_t minPowerDbmX10 {0};
    int16_t maxPowerDbmX10 {0};

    int16_t linkMarginDbX10 {0};
    int16_t lowPowerDbmX10 {0};

    int16_t tempLowCx10 {0};
    int16_t tempHighCx10 {0};
  };

  void SetRoutingInfo (
    const RoutingInfo &info);

  RoutingInfo GetRoutingInfo () const;

  struct AdvertisedRoute
  {
    CsrNodeId dst {CSR_BROADCAST_ID};
    uint8_t  hops {0};
    uint32_t cost {0};
    int16_t  pathlossDbX10 {0};
    uint8_t  capability {0};

    std::vector<CsrNodeId> path;
  };

  void ClearAdvertisedRoutes ();

  bool AddAdvertisedRoute (
    CsrNodeId dst,
    uint8_t hops,
    uint32_t cost,
    int16_t pathlossDbX10,
    uint8_t capability,
    const std::vector<CsrNodeId> &path = {});

  uint8_t GetAdvertisedRouteCount () const;
  AdvertisedRoute GetAdvertisedRoute (uint8_t index) const;

  void SetRoutingTarget (CsrNodeId target);
  CsrNodeId GetRoutingTarget () const;

private:
  CsrNodeId m_nodeId {0};
  uint16_t m_helloSeq {0};
  uint8_t  m_speedKey {0};
  int16_t  m_rxPowerDbmX10 {0};
  uint8_t  m_activeNodes {0};
  uint32_t m_routingSequence {0};
  uint8_t m_routingSection {0};
  uint8_t m_routingTotalSections {1};
  uint8_t  m_arlRouteMsgType {static_cast<uint8_t> (CsrArlRouteMsgType::None)};
  static constexpr uint8_t MAX_ADVERTISED_ROUTES = 8;
  std::vector<AdvertisedRoute> m_advertisedRoutes;

  static constexpr uint16_t MAX_CHIRP_NEIGHBORS = 255;
  std::vector<CsrNodeId> m_chirpNeighbors;

  uint8_t m_neighborCheckType {
  static_cast<uint8_t> (CsrNeighborCheckType::None)
  };

  uint8_t m_discoverType {
  static_cast<uint8_t> (CsrDiscoverType::None)
  };

  uint32_t m_discoverySequence {0};

  CsrNodeId m_neighborCheckTarget {CSR_BROADCAST_ID};

  uint8_t m_nodeType {
  static_cast<uint8_t> (CsrNodeType::Ordinary)
  };

  uint8_t m_routingOperation {
    static_cast<uint8_t> (CsrRoutingOperation::None)
  };

  CsrNodeId m_routingTarget {CSR_BROADCAST_ID};

  RoutingInfo m_routingInfo;
};

} // namespace ns3
