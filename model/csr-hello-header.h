#pragma once
#include "ns3/header.h"
#include "ns3/buffer.h"
#include <vector>

namespace ns3 {

class CsrHelloHeader : public Header
{
public:
  CsrHelloHeader ();

  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const override;

  // Setters/Getters
  void SetNodeId (uint16_t id);
  uint16_t GetNodeId () const;

  void SetHelloSeq (uint16_t s);
  uint16_t GetHelloSeq () const;

  void SetSpeedKey (uint8_t k);
  uint8_t GetSpeedKey () const;

  void SetRxPowerDbmX10 (int16_t p);   // dBm * 10 to keep it integer
  int16_t GetRxPowerDbmX10 () const;

  void SetActiveNodes (uint8_t n);
  uint8_t GetActiveNodes () const;

  // Header overrides
  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  struct AdvertisedRoute
  {
    uint16_t dst {0xFFFF};
    uint8_t  hops {0};
    uint32_t cost {0};
    int16_t  pathlossDbX10 {0};
    uint8_t  capability {0};
  };

  void ClearAdvertisedRoutes ();
  bool AddAdvertisedRoute (uint16_t dst,
                          uint8_t hops,
                          uint32_t cost,
                          int16_t pathlossDbX10,
                          uint8_t capability);

  uint8_t GetAdvertisedRouteCount () const;
  AdvertisedRoute GetAdvertisedRoute (uint8_t index) const;

private:
  uint16_t m_nodeId {0};
  uint16_t m_helloSeq {0};
  uint8_t  m_speedKey {0};
  int16_t  m_rxPowerDbmX10 {0};
  uint8_t  m_activeNodes {0};
  //uint16_t m_advDst {0xFFFF};
  //uint8_t  m_advHops {0};
  static constexpr uint8_t MAX_ADVERTISED_ROUTES = 8;
  std::vector<AdvertisedRoute> m_advertisedRoutes;
  
};

} // namespace ns3
