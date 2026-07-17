#pragma once
#include "csr-common.h"
#include "csr-hello-header.h"
#include "csr-hop-layer.h"
#include <cmath>


class CsrNetLayer : public Object
{
public:

  enum class DiscoveryState { IDLE, SCHEDULED, ACTIVE, COOLDOWN };

  DiscoveryState m_discState { DiscoveryState::IDLE };
  EventId m_discoveryStartEvent, m_discoveryStopEvent, m_discoveryCooldownEvent;
  Time m_discoveryCooldown { Seconds(5) }; // tune later
  EventId m_discoveryHelloEvent;
  Time    m_discoveryHelloInterval { Seconds (2.0) };

  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::CsrNetLayer")
      .SetParent<Object> ()
      .AddConstructor<CsrNetLayer> ();
    return tid;
  }

  CsrNetLayer ()
    : m_nodeId (0)
  {}

  void SetNodeId (uint16_t id)
  {
    m_nodeId = id;
  }

  void StartDiscovery (Time startDelay, Time duration);
  void SetRepeatDiscoveryHello (bool enable);
  void SendRoutingUpdate ();
  //void SendNeighborCheck ();
  void SendNeighborCheck (
    CsrNeighborCheckType type = CsrNeighborCheckType::Message);
  const char* NeighborCheckTypeName (CsrNeighborCheckType t) const;
  void StartNeighborFreshnessMonitor (Time timeout = Seconds (20.0),
                                    Time period = Seconds (2.0));
  void SetInvalidateRoutesOnStaleNeighbor (bool enable)
  {
    m_invalidateRoutesOnStaleNeighbor = enable;

    std::cout << "[NWK " << m_nodeId
              << "] invalidate_routes_on_stale_neighbor="
              << (m_invalidateRoutesOnStaleNeighbor ? "true" : "false")
              << std::endl;
  }
  bool IsDiscoveryActive () const { return m_discoveryActive; }

  void ProcessHello (Ptr<Packet> helloPayload,
                   uint16_t hopSrc,
                   double pathlossDb,
                   double snrDb);

  void DumpNsdp (std::ostream& os) const
  {
    for (auto const& kv : m_nsdp)
      {
        auto const& e = kv.second;
        os << Simulator::Now ().GetSeconds ()
           << "," << m_nodeId
           << "," << e.src
           << "," << e.dst
           << "," << e.count
           << "," << e.limit
           << "," << m_nwkQueue.size ()
           << "\n";
      }
  }

  void NoteLinkFailure (uint16_t nextHop)
  {
    auto &ne = m_nwkNeighbors[nextHop];

    ne.nodeId = nextHop;
    ne.numFailures++;

    std::cout << "[NWK " << m_nodeId
              << "] Link failure to nextHop=" << nextHop
              << " numFailures=" << ne.numFailures
              << std::endl;

    RecomputeRoutesViaNextHop (nextHop);
  }

  void
  RecomputeRoutesViaNextHop (uint16_t nextHop)
  {
    auto nit = m_nwkNeighbors.find (nextHop);
    if (nit == m_nwkNeighbors.end ())
      {
        return;
      }

    NwkNeighborEntry &ne = nit->second;

    if (std::isnan (ne.lastPathlossDb))
      {
        return;
      }

    double s0PowerDbm = static_cast<double> (ne.rxPowerDbmX10) / 10.0;
    if (s0PowerDbm == 0.0)
      {
        s0PowerDbm = m_rxS0BaseLevelDbm + m_linkMarginDb;
      }

    int chosenSpeed = 0;
    double chosenTxPower = 0.0;
    int estDistance = 0;
    double speedMargin = 0.0;
    double totalMargin = 0.0;

    uint32_t newLinkCost =
      ComputeLinkCost (s0PowerDbm,
                      ne.lastPathlossDb,
                      ne.numFailures,
                      &chosenSpeed,
                      &chosenTxPower,
                      &estDistance,
                      &speedMargin,
                      &totalMargin);

    for (auto &re : m_routes)
      {
        if (!re.valid || re.nextHop != nextHop)
          {
            continue;
          }

        uint32_t oldCost = re.cost;

        re.linkCostToNextHop = newLinkCost;
        re.cost = re.linkCostToNextHop + re.advertisedCost;
        re.lastUpdated = Simulator::Now ();

        std::cout << "[NWK " << m_nodeId
                  << "] Recomputed route after link failure"
                  << " dst=" << re.nwkDst
                  << " nextHop=" << nextHop
                  << " oldCost=" << oldCost
                  << " newCost=" << re.cost
                  << " linkCost=" << re.linkCostToNextHop
                  << " advertisedCost=" << re.advertisedCost
                  << " numFailures=" << ne.numFailures
                  << std::endl;
      }
  }

  void UpdateMacActiveNodes ()
  {
    if (m_hop != nullptr)
      {
        uint32_t active = GetActiveNodeCount ();
        m_hop->SetActiveNodesForPostTx (active);

        std::cout << "[NWK " << m_nodeId
                  << "] active_nodes=" << active
                  << " pushed to MAC"
                  << std::endl;
      }
  }
  void ClearRoutes ()
  {
    m_routes.clear ();

    std::cout << "[NWK " << m_nodeId
              << "] Cleared route table"
              << std::endl;
  }

  void
  DumpRoutes () const
  {
    std::cout << "[NWK " << m_nodeId << "] Routes:";

    if (m_routes.empty ())
      {
        std::cout << " <none>";
      }
    else
      {
        for (const auto &re : m_routes)
          {
            std::cout << " dst=" << re.nwkDst
                      << "->nh=" << re.nextHop
                      << "(hops=" << unsigned (re.numHop)
                      << ",cost=" << re.cost
                      << ",linkCost=" << re.linkCostToNextHop
                      << ",advCost=" << re.advertisedCost
                      << ",from=" << re.learnedFrom
                      << ",imm=" << (re.immediate ? 1 : 0)
                      << ",pl=" << re.pathlossDb
                      << ",valid=" << (re.valid ? 1 : 0)
                      << ")";
          }
      }

    std::cout << std::endl;
  }

  void SetHop (Ptr<CsrHopLayer> hop)
  {
    m_hop = hop;

    if (m_hop != nullptr)
      {
        // Data/control payloads from HOP up to NWK
        m_hop->SetRxFromHopCallback (
          MakeCallback (&CsrNetLayer::ReceiveFromHop, this));

        // HELLO/control discovery path: OPNET proc_hello() equivalent
        m_hop->SetRxHelloFromHopCallback (
          MakeCallback (&CsrNetLayer::ProcessHello, this));

        // HOP ACK/DACK/resend completion releases NWK NSDP flow count
        m_hop->SetNsdpDecrementCallback (
          MakeCallback (&CsrNetLayer::DecrementNsdp, this));

        // HOP asks NWK whether this flow should use DACK-style delayed release
        m_hop->SetShouldDackCallback (
          MakeCallback (&CsrNetLayer::ShouldDack, this));

        m_hop->SetLinkFailureCallback (
          MakeCallback (&CsrNetLayer::NoteLinkFailure, this));
      }
  }

  // Net -> App callback: payload + network source node ID
  void SetRxFromNetCallback (Callback<void, Ptr<Packet>, uint16_t> cb)
  {
    m_rxFromNetCb = cb;
  }

  bool ShouldDack (uint16_t src, uint16_t dst)
  {
    NsdpEntry &e = GetNsdpEntry (src, dst);
    return e.count >= e.limit;
  }

  // Called by App to send a payload to some destination
  void Send (uint16_t dst,
             uint8_t dscp,
             Ptr<Packet> payload,
             bool ack)
  {
    // Wrap the app payload in a CsrNetHeader carrying nwk src/dst/DSCP.
    Ptr<Packet> framed = payload->Copy ();
    CsrNetHeader nh (m_nodeId, dst, dscp);
    framed->AddHeader (nh);

    NwkQueueEntry e;
    e.nwkSrc  = m_nodeId;
    e.nwkDst  = dst;
    e.dscp    = dscp;
    e.ack     = ack;
    e.payload = framed;

    // NSDP: increment count for (nwkSrc, nwkDst)
    NsdpEntry &nsdp = GetNsdpEntry (e.nwkSrc, e.nwkDst);
    nsdp.count++;

    std::cout << "[NWK " << m_nodeId << "] NSDP(" << e.nwkSrc
            << "->" << e.nwkDst << ") incremented to "
            << nsdp.count << std::endl;

    // Simple priority: higher DSCP closer to front
    auto it = m_nwkQueue.begin ();
    for (; it != m_nwkQueue.end (); ++it)
      {
        if (dscp > it->dscp)
          {
            break;
          }
      }
    m_nwkQueue.insert (it, e);

    ScheduleCheckNwkQueue ();
  }

  bool CanSendForFlow (uint16_t src, uint16_t dst)
  {
    NsdpEntry &e = GetNsdpEntry (src, dst);

    // Debug print so we can see when NSDP is gating sends
    std::cout << "[NWK " << m_nodeId << "] CanSendForFlow "
              << src << "->" << dst
              << " NSDP.count=" << e.count
              << " limit=" << e.limit
              << " -> " << (e.count < e.limit ? "YES" : "NO")
              << std::endl;

    return e.count < e.limit;
  }

  void DecrementNsdp (uint16_t src, uint16_t dst)
  {
    NsdpEntry &e = GetNsdpEntry (src, dst);
    if (e.count > 0)
      {
        e.count--;
      std::cout << "[NWK " << m_nodeId << "] NSDP("
                << src << "->" << dst
                << ") decremented to "
                << e.count << std::endl;
      }
  }

  // Called by Hop when a payload arrives from the MAC
  void ReceiveFromHop (Ptr<Packet> packetFromHop, uint16_t hopSrc)
  {
    // Peek network header to inspect nwkSrc/nwkDst/DSCP
    CsrNetHeader nh;
    if (!packetFromHop->PeekHeader (nh))
      {
        NS_LOG_ERROR ("CsrNetLayer::ReceiveFromHop(): missing CsrNetHeader");
        return;
      }

    uint16_t nwkSrc = nh.GetSrc ();
    uint16_t nwkDst = nh.GetDst ();
    uint8_t  dscp   = nh.GetDscp ();

    if (nwkDst == m_nodeId)
      {
        // Final destination: strip header and deliver payload to App
        packetFromHop->RemoveHeader (nh);

        if (!m_rxFromNetCb.IsNull ())
          {
            m_rxFromNetCb (packetFromHop, nwkSrc);
          }
      }
    else
      {
        // Not for us: enqueue as a relay and bump NSDP,
        // similar to br_nwk.proc_hop_pk() for br_Network. 

        NwkQueueEntry e;
        e.nwkSrc  = nwkSrc;
        e.nwkDst  = nwkDst;
        e.dscp    = dscp;
        e.ack     = true;          // hop-level reliability for relays
        e.payload = packetFromHop; // keep CsrNetHeader for next hop

        // NSDP increment for relayed traffic at this node
        NsdpEntry &nsdp = GetNsdpEntry (e.nwkSrc, e.nwkDst);
        nsdp.count++;

        // Priority insert by DSCP
        auto it = m_nwkQueue.begin ();
        for (; it != m_nwkQueue.end (); ++it)
          {
            if (dscp > it->dscp)
              {
                break;
              }
          }
        m_nwkQueue.insert (it, e);

        ScheduleCheckNwkQueue ();
      }
  }

  bool
  AddOrUpdateRoute (uint16_t nwkDst,
                    uint16_t nextHop,
                    bool immediate,
                    uint8_t numHop,
                    double pathlossDb,
                    uint32_t linkCostToNextHop,
                    uint32_t advertisedCost,
                    uint16_t learnedFrom,
                    uint8_t capability = 0)
  {
    if (nwkDst == m_nodeId)
      {
          return false;
      }

    uint32_t totalCost = linkCostToNextHop + advertisedCost;
    if (totalCost == 0)
      {
        totalCost = 1;
      }

    for (auto &re : m_routes)
      {
        if (re.nwkDst == nwkDst)
          {
            bool sameNextHop = (re.nextHop == nextHop);
            bool betterCost  = (totalCost + 5 < re.cost);
            bool invalid     = !re.valid;

            if (sameNextHop || betterCost || invalid)
              {
                re.capability = capability;
                re.immediate = immediate;
                re.nextHop = nextHop;
                re.pathlossDb = pathlossDb;
                re.numHop = numHop;

                re.linkCostToNextHop = linkCostToNextHop;
                re.advertisedCost = advertisedCost;
                re.cost = totalCost;

                re.lastUpdated = Simulator::Now ();
                re.valid = true;
                re.learnedFrom = learnedFrom;

                std::cout << "[NWK " << m_nodeId
                          << "] Updated route dst=" << nwkDst
                          << " nextHop=" << nextHop
                          << " hops=" << unsigned (numHop)
                          << " cost=" << re.cost
                          << " linkCost=" << re.linkCostToNextHop
                          << " advCost=" << re.advertisedCost
                          << " pathloss=" << pathlossDb
                          << std::endl;

                return true;
              }

            std::cout << "[NWK " << m_nodeId
            << "] Ignored route dst=" << nwkDst
            << " via nextHop=" << nextHop
            << " cost=" << totalCost
            << " existingCost=" << re.cost
            << " existingNextHop=" << re.nextHop
            << std::endl;

            return false;
          }
      }

    RouteEntry re;
    re.nwkDst = nwkDst;
    re.capability = capability;
    re.immediate = immediate;
    re.nextHop = nextHop;
    re.pathlossDb = pathlossDb;
    re.numHop = numHop;

    re.linkCostToNextHop = linkCostToNextHop;
    re.advertisedCost = advertisedCost;
    re.cost = totalCost;
    re.learnedFrom = learnedFrom;

    re.energyLevel = 100;
    re.lastUpdated = Simulator::Now ();
    re.valid = true;

    m_routes.push_back (re);

    std::cout << "[NWK " << m_nodeId
              << "] Added route dst=" << nwkDst
              << " nextHop=" << nextHop
              << " hops=" << unsigned (numHop)
              << " cost=" << re.cost
              << " linkCost=" << re.linkCostToNextHop
              << " advCost=" << re.advertisedCost
              << " pathloss=" << pathlossDb
              << std::endl;

    return true;
  }

  void AddStaticRoute (uint16_t nwkDst, uint16_t nextHop)
  {
    bool immediate = (nwkDst == nextHop);
    uint8_t hops = immediate ? 1 : 2;

    AddOrUpdateRoute (nwkDst,
                      nextHop,
                      immediate,
                      hops,
                      std::numeric_limits<double>::quiet_NaN (),
                      1,
                      0,
                      CSR_BROADCAST_ID,
                      0);
  }

  void
  AddStaticRouteWithPathloss (uint16_t nwkDst,
                              uint16_t nextHop,
                              double pathlossDb,
                              bool immediate = true,
                              uint8_t capability = 0)
  {
    double s0PowerDbm = m_rxS0BaseLevelDbm + m_linkMarginDb;

    int chosenSpeed = 0;
    double chosenTxPower = 0.0;
    int estDistance = 0;
    double speedMargin = 0.0;
    double totalMargin = 0.0;

    uint32_t cost = ComputeLinkCost (s0PowerDbm,
                                    pathlossDb,
                                    0, // num_failures placeholder
                                    &chosenSpeed,
                                    &chosenTxPower,
                                    &estDistance,
                                    &speedMargin,
                                    &totalMargin);

    uint8_t hops = immediate ? 1 : 2;

    AddOrUpdateRoute (nwkDst,
                      nextHop,
                      immediate,
                      hops,
                      pathlossDb,
                      cost,
                      0,
                      CSR_BROADCAST_ID,
                      capability);

    std::cout << "[NWK " << m_nodeId
              << "] Static route link_calc dst=" << nwkDst
              << " nextHop=" << nextHop
              << " pathloss=" << pathlossDb
              << " speed=" << chosenSpeed
              << " txPower=" << chosenTxPower
              << " estDistance=" << estDistance
              << " speedMargin=" << speedMargin
              << " totalMargin=" << totalMargin
              << " cost=" << cost
              << std::endl;
  }

private:

  struct RouteEntry
  {
    uint16_t nwkDst {0};        // OPNET node_addr
    uint8_t  capability {0};    // OPNET capability
    bool     immediate {false}; // OPNET immediate neighbor flag
    uint16_t nextHop {0};       // OPNET next_hop

    double   pathlossDb {std::numeric_limits<double>::quiet_NaN ()};
    uint8_t  numHop {0};        // OPNET num_hop
    uint32_t cost {0};          // OPNET cost
    uint8_t  energyLevel {100}; // OPNET energy_level placeholder

    Time     lastUpdated {Seconds (0.0)};
    bool     valid {true};
    uint32_t linkCostToNextHop {0};
    uint32_t advertisedCost {0};

    uint16_t learnedFrom {CSR_BROADCAST_ID}; // node that taught us this route
  };

  struct NwkQueueEntry
  {
    uint16_t   nwkSrc;
    uint16_t   nwkDst;
    uint8_t    dscp;
    bool       ack;
    Ptr<Packet> payload;
  };

  struct NwkNeighborEntry
  {
    uint16_t nodeId {0};
    double lastHeardSec {-1.0};
    double lastPathlossDb {std::numeric_limits<double>::quiet_NaN ()};
    double lastSnrDb {std::numeric_limits<double>::quiet_NaN ()};
    uint8_t speedKey {0};
    int16_t rxPowerDbmX10 {0};
    uint8_t activeNodes {0};

    // OPNET BrT_Neighbor_Entry::num_failures
    uint32_t numFailures {0};
    bool stale {false};
  };

  // NSDP: Network Source–Destination Pair entry
  struct NsdpEntry
  {
    uint16_t src;
    uint16_t dst;
    uint32_t count;
    uint32_t limit;  // max in-flight packets allowed for this flow
  };

  // Keyed by (src,dst)
  NsdpEntry& GetNsdpEntry (uint16_t src, uint16_t dst)
  {
    std::pair<uint16_t,uint16_t> key (src, dst);
    auto it = m_nsdp.find (key);
    if (it == m_nsdp.end ())
      {
        NsdpEntry e;
        e.src   = src;
        e.dst   = dst;
        e.count = 0;
        e.limit = 4;     // you can tune this; similar magnitude to Hop threshold
        it = m_nsdp.insert (std::make_pair (key, e)).first;
      }
    return it->second;
  }

  void ScheduleCheckNwkQueue ()
  {
    if (!m_checkNwkQueueEvent.IsPending ())
      {
        m_checkNwkQueueEvent =
          Simulator::ScheduleNow (&CsrNetLayer::CheckNwkQueue, this);
      }
  }

  void CheckNwkQueue ()
  {
    // Minimal version of check_nwk_queue():
    // just push everything down to Hop in priority order for now.
    if (m_hop == nullptr)
      {
        return;
      }

    std::cout << "[NWK " << m_nodeId << "] CheckNwkQueue: size="
            << m_nwkQueue.size () << std::endl;

    while (!m_nwkQueue.empty ())
    {
      NwkQueueEntry e = m_nwkQueue.front ();

      uint16_t hopDest;
      if (!LookupNextHop (e.nwkDst, hopDest))
        {
          // OPNET-like behavior: if we have data but no route/next hop,
          // trigger on-demand discovery and keep the packet queued.
          EnsureDiscoveryForTx ();

          std::cout << "[NWK " << m_nodeId << "] No route to nwkDst="
                    << e.nwkDst << " -> on-demand discovery; holding packet"
                    << std::endl;

          // Do NOT pop; stop draining for now and try again later
          break;
        }

        // --- NEW: NSDP-based flow gating (per-flow policy) ---
      if (!CanSendForFlow (e.nwkSrc, e.nwkDst))
        {
          // NSDP says we have too many packets in flight for this flow.
          // Leave this entry in the queue and stop draining for now.
          std::cout << "[NWK " << m_nodeId << "] NSDP gating flow "
                    << e.nwkSrc << "->" << e.nwkDst
                    << " (queue stays at size=" << m_nwkQueue.size () << ")"
                    << std::endl;
          break;
        }
        
      // Existing Hop flow-control gating (per-next-hop)
      if (!m_hop->CanSendToHop (hopDest))
        {
          // Leave e in the queue; try again later when some ACKs free capacity
          break;
        }

      // Hop can accept another frame → remove from queue and send
      m_nwkQueue.pop_front ();
      m_hop->SendData (hopDest, e.dscp, e.payload, e.ack);
    }
  }

  bool LookupNextHop (uint16_t nwkDst, uint16_t &nextHopOut)
  {
    RouteEntry const* best = nullptr;

    for (const auto &re : m_routes)
      {
        if (re.nwkDst == nwkDst && re.valid)
          {
            if (best == nullptr || re.cost < best->cost)
              {
                best = &re;
              }
          }
      }

    if (best == nullptr)
      {
        return false;
      }

    nextHopOut = best->nextHop;
    return true;
  }

  uint32_t
  ComputeLinkCost (double s0PowerDbm,
                   double pathlossDb,
                   uint32_t numFailures,
                   int *speedOut = nullptr,
                   double *txPowerOut = nullptr,
                   int *estDistanceOut = nullptr,
                   double *speedMarginOut = nullptr,
                   double *totalMarginOut = nullptr) const
  {
    // Port of legacy OPNET br_hop.link_calc(), simplified for NS-3.
    // TX0_power is the power needed to meet link margin at 8 kbps.
    double tx0PowerDbm = s0PowerDbm + pathlossDb;

    // Txm_power is power required at configured minimum speed.
    double txmPowerDbm = tx0PowerDbm;
    switch (m_minCfgSpeedKbps)
      {
      case 1000: txmPowerDbm = tx0PowerDbm + 23.0; break;
      case 500:  txmPowerDbm = tx0PowerDbm + 20.0; break;
      case 128:  txmPowerDbm = tx0PowerDbm + 12.0; break;
      case 64:   txmPowerDbm = tx0PowerDbm + 9.0;  break;
      case 32:   txmPowerDbm = tx0PowerDbm + 6.0;  break;
      case 16:   txmPowerDbm = tx0PowerDbm + 3.0;  break;
      case 8:    txmPowerDbm = tx0PowerDbm;        break;
      default:   txmPowerDbm = tx0PowerDbm;        break;
      }

    int speed = m_minCfgSpeedKbps;

    // Highest speed that can meet link margin at max configured TX power.
    double marginAt8K = m_maxTxPowerDbm - tx0PowerDbm;

    if      (marginAt8K >= 23.0) speed = 1000;
    else if (marginAt8K >= 20.0) speed = 500;
    else if (marginAt8K >= 12.0) speed = 128;
    else if (marginAt8K >= 9.0)  speed = 64;
    else if (marginAt8K >= 6.0)  speed = 32;
    else if (marginAt8K >= 3.0)  speed = 16;
    else if (marginAt8K > 0.0)   speed = 8;
    else                         speed = m_minCfgSpeedKbps;

    // Limit speed to configured min/max.
    if (speed < m_minCfgSpeedKbps)
      {
        speed = m_minCfgSpeedKbps;
      }
    if (speed > m_maxCfgSpeedKbps)
      {
        speed = m_maxCfgSpeedKbps;
      }

    // Required TX power for selected speed.
    double txPowerDbm = tx0PowerDbm;
    switch (speed)
      {
      case 1000: txPowerDbm = tx0PowerDbm + 23.0; break;
      case 500:  txPowerDbm = tx0PowerDbm + 20.0; break;
      case 128:  txPowerDbm = tx0PowerDbm + 12.0; break;
      case 64:   txPowerDbm = tx0PowerDbm + 9.0;  break;
      case 32:   txPowerDbm = tx0PowerDbm + 6.0;  break;
      case 16:   txPowerDbm = tx0PowerDbm + 3.0;  break;
      case 8:    txPowerDbm = tx0PowerDbm;        break;
      default:   txPowerDbm = tx0PowerDbm;        break;
      }

    double totalMargin = m_maxTxPowerDbm - txmPowerDbm;
    double speedMargin = m_maxTxPowerDbm - txPowerDbm;

    // Limit actual power to configured min/max.
    if (txPowerDbm < m_minTxPowerDbm)
      {
        txPowerDbm = m_minTxPowerDbm;
      }
    if (txPowerDbm > m_maxTxPowerDbm)
      {
        txPowerDbm = m_maxTxPowerDbm;
      }

    // Scaled distance from transmit power using r^4 model.
    int estDistance;
    if (txPowerDbm <= m_txAmpBreakpointDbm)
      {
        estDistance = 75;
      }
    else
      {
        estDistance = static_cast<int> (
          std::floor (std::pow (10.0, (txPowerDbm - m_txAmpBreakpointDbm) / 40.0) * 100.0));
      }

    uint32_t cost = static_cast<uint32_t> (
      std::floor (static_cast<double> (estDistance) * 100.0 / static_cast<double> (speed)));

    // OPNET link-quality penalties/bonuses.
    if ((m_maxTxPowerDbm - txmPowerDbm - 3.0 * static_cast<double> (numFailures)) < 0.0)
      {
        cost *= 2;
      }

    if ((m_maxTxPowerDbm - txPowerDbm - 3.0 * static_cast<double> (numFailures)) > 3.0)
      {
        cost = static_cast<uint32_t> (std::floor (static_cast<double> (cost) / 2.0));
      }

    // Quantize TX power like OPNET.
    txPowerDbm = std::ceil (txPowerDbm);

    if (speedOut)       { *speedOut = speed; }
    if (txPowerOut)     { *txPowerOut = txPowerDbm; }
    if (estDistanceOut) { *estDistanceOut = estDistance; }
    if (speedMarginOut) { *speedMarginOut = speedMargin; }
    if (totalMarginOut) { *totalMarginOut = totalMargin; }

    return std::max<uint32_t> (1, cost);
  }

private:
  uint16_t                              m_nodeId;
  Ptr<CsrHopLayer>                      m_hop;
  Callback<void, Ptr<Packet>, uint16_t> m_rxFromNetCb;

  std::deque<NwkQueueEntry>             m_nwkQueue;
  EventId                               m_checkNwkQueueEvent;
  std::vector<RouteEntry>               m_routes;
  std::map<uint16_t, NwkNeighborEntry>  m_nwkNeighbors;
  std::map<std::pair<uint16_t,uint16_t>, NsdpEntry> m_nsdp;

  bool    m_discoveryActive { false };

  uint32_t GetNeighborCount () const;
  uint32_t GetActiveNodeCount () const;

  uint8_t m_minSpeedKey { 8 };  // temporary default; map to your actual rate table later

  // OPNET-ish link_calc configuration.
  // These mirror OPNET attributes: Link Margin, Max Power, Min Power,
  // Max Speed, Min Speed, and TX_AMP_BREAKPOINT.
  double   m_linkMarginDb       { 10.0 };
  double   m_rxS0BaseLevelDbm   { -115.0 };
  double   m_maxTxPowerDbm      { 30.0 };
  double   m_minTxPowerDbm      { 0.0 };
  int      m_maxCfgSpeedKbps    { 128 };
  int      m_minCfgSpeedKbps    { 8 };
  double   m_txAmpBreakpointDbm { 14.0 };

  void DiscoveryStart ();
  void DiscoveryStop ();
  void DiscoveryCooldownOver ();

  //void SendHelloBroadcast (CsrArlRouteMsgType type = CsrArlRouteMsgType::Discover);

  void SendHelloBroadcast (
    CsrArlRouteMsgType type = CsrArlRouteMsgType::Discover,
      CsrNeighborCheckType checkType = CsrNeighborCheckType::None);

  void EnsureDiscoveryForTx ();

  void ProcessRoutesPayload (const CsrHelloHeader &hh,
                             uint16_t helloSrc,
                             double pathlossDb,
                             double snrDb,
                             uint32_t linkCost);

  void ProcessArlRouteMessage (const CsrHelloHeader &hh,
                              uint16_t helloSrc,
                              double pathlossDb,
                              double snrDb,
                              uint32_t linkCost);

  void ProcessDiscover (const CsrHelloHeader &hh,
                        uint16_t helloSrc,
                        double pathlossDb,
                        double snrDb,
                        uint32_t linkCost);

  void ProcessRoutingUpdate (const CsrHelloHeader &hh,
                            uint16_t helloSrc,
                            double pathlossDb,
                            double snrDb,
                            uint32_t linkCost);

  void ProcessNeighborCheck (const CsrHelloHeader &hh,
                           uint16_t helloSrc,
                           double pathlossDb,
                           double snrDb,
                           uint32_t linkCost);

  const char* ArlRouteMsgTypeName (CsrArlRouteMsgType t) const;

  bool ShouldAdvertiseRoute (const RouteEntry &re) const;

  void TryDrainQueueAfterDiscovery ();

  void ScheduleDiscoveryHello ();
  void DiscoveryHelloTick ();
  // Experimental NS-3 robustness mode.
  // Legacy bare OPNET start_discovery() sends one HELLO.
  // Do not enable for strict OPNET parity tests unless modeling routing-module-driven repeats.
  bool m_repeatDiscoveryHello { false }; // false = legacy OPNET-style one-shot discovery HELLO

  void CheckNeighborFreshness ();
  void InvalidateRoutesViaNextHop (uint16_t nextHop, const char *reason);
  bool m_invalidateRoutesOnStaleNeighbor { false };

  EventId m_neighborFreshnessEvent;
  Time m_neighborFreshnessTimeout { Seconds (20.0) };
  Time m_neighborFreshnessCheckPeriod { Seconds (2.0) };
};


// ------------------------------------------------------------
// Simple "App" callbacks
// ------------------------------------------------------------

static void
AppRxFromNet (Ptr<Packet> payload, uint16_t src)
{
    std::cout << "  [APP] Node received payload from " << src
              << " (size=" << payload->GetSize () << " B)"
              << std::endl;
}

void
CsrNetLayer::StartDiscovery (Time startDelay, Time duration)
{
    // If already in discovery lifecycle, do not restart or cancel existing events.
    if (m_discState == DiscoveryState::SCHEDULED ||
        m_discState == DiscoveryState::ACTIVE ||
        m_discState == DiscoveryState::COOLDOWN)
      {
        return;
      }

    if (m_discoveryStartEvent.IsPending ())
      {
        Simulator::Cancel (m_discoveryStartEvent);
      }

    if (m_discoveryStopEvent.IsPending ())
      {
        Simulator::Cancel (m_discoveryStopEvent);
      }

    if (m_discoveryHelloEvent.IsPending ())
      {
        Simulator::Cancel (m_discoveryHelloEvent);
      }

    m_discState = DiscoveryState::SCHEDULED;

    m_discoveryStartEvent =
      Simulator::Schedule (startDelay, &CsrNetLayer::DiscoveryStart, this);

    m_discoveryStopEvent =
      Simulator::Schedule (startDelay + duration, &CsrNetLayer::DiscoveryStop, this);
}

void
CsrNetLayer::ProcessHello (Ptr<Packet> helloPayload,
                            uint16_t hopSrc,
                            double pathlossDb,
                            double snrDb)
{
    CsrHelloHeader hh;
    if (!helloPayload->RemoveHeader (hh))
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RX HELLO from hopSrc=" << hopSrc
                  << " but missing CsrHelloHeader"
                  << std::endl;
        return;
      }

    uint16_t src = hh.GetNodeId ();

  if (src == m_nodeId)
    {
        return;
    }

  double now = Simulator::Now ().GetSeconds ();

    // ------------------------------------------------------------
    // 1) Update NWK neighbor table, similar to OPNET proc_hello()
    // ------------------------------------------------------------
  auto &ne = m_nwkNeighbors[src];
  bool isNew = (ne.lastHeardSec < 0.0);
  bool wasStale = ne.stale;

  ne.nodeId = src;
  ne.lastHeardSec = now;
  ne.lastPathlossDb = pathlossDb;
  ne.lastSnrDb = snrDb;
  ne.speedKey = hh.GetSpeedKey ();
  ne.rxPowerDbmX10 = hh.GetRxPowerDbmX10 ();
  ne.activeNodes = hh.GetActiveNodes ();
  ne.stale = false;

  if (wasStale)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Neighbor " << src
                << " is fresh again after receiving "
                << ArlRouteMsgTypeName (hh.GetArlRouteMsgType ())
                << std::endl;
    }

  UpdateMacActiveNodes ();

  if (m_hop != nullptr)
    {
      m_hop->NoteReportedActiveNodes (ne.activeNodes);
    }

  std::cout << "[NWK " << m_nodeId << "] "
              << (isNew ? "New" : "Updated")
              << " HELLO neighbor=" << src
              << " hopSrc=" << hopSrc
              << " speedKey=" << unsigned (ne.speedKey)
              << " activeNodes=" << unsigned (ne.activeNodes)
              << " pathloss=" << pathlossDb
              << " snr=" << snrDb
              << std::endl;

    // ------------------------------------------------------------
    // 2) Direct route to HELLO sender
    //    If we heard src directly, route dst=src via nextHop=src.
    // ------------------------------------------------------------

  double s0PowerDbm = static_cast<double> (hh.GetRxPowerDbmX10 ()) / 10.0;

  int chosenSpeed = 0;
  double chosenTxPower = 0.0;
  int estDistance = 0;
  double speedMargin = 0.0;
  double totalMargin = 0.0;

  uint32_t linkCost = ComputeLinkCost (s0PowerDbm,
                                      pathlossDb,
                                      ne.numFailures,
                                      &chosenSpeed,
                                      &chosenTxPower,
                                      &estDistance,
                                      &speedMargin,
                                      &totalMargin);

  std::cout << "[NWK " << m_nodeId
              << "] link_calc neighbor=" << src
              << " s0=" << s0PowerDbm
              << " pathloss=" << pathlossDb
              << " numFailures=" << ne.numFailures
              << " speed=" << chosenSpeed
              << " txPower=" << chosenTxPower
              << " estDistance=" << estDistance
              << " speedMargin=" << speedMargin
              << " totalMargin=" << totalMargin
              << " cost=" << linkCost
              << std::endl;

  AddOrUpdateRoute (src,
                  src,
                  true,
                  1,
                  pathlossDb,
                  linkCost,
                  0,
                  src,   // learned from the neighbor itself
                  0);

    // ------------------------------------------------------------
    // 3) Advertised route from HELLO sender
    //    If src advertises dst=X, then we can reach X via src.
    // ------------------------------------------------------------

  ProcessArlRouteMessage (hh,
                        src,
                        pathlossDb,
                        snrDb,
                        linkCost);

    DumpRoutes ();

    // Discovery/route update may have unblocked queued packets.
    ScheduleCheckNwkQueue ();
}

  const char*
  CsrNetLayer::ArlRouteMsgTypeName (CsrArlRouteMsgType t) const
{
  switch (t)
    {
     case CsrArlRouteMsgType::None:
      return "None";
    case CsrArlRouteMsgType::Discover:
      return "Discover";
    case CsrArlRouteMsgType::RoutingUpdate:
      return "RoutingUpdate";
    case CsrArlRouteMsgType::NeighborCheck:
      return "NeighborCheck";
    case CsrArlRouteMsgType::KeyRequest:
      return "KeyRequest";
    default:
      return "Unknown";
    }
}

void
CsrNetLayer::ProcessArlRouteMessage (const CsrHelloHeader &hh,
                                      uint16_t helloSrc,
                                      double pathlossDb,
                                      double snrDb,
                                      uint32_t linkCost)
{
    CsrArlRouteMsgType type = hh.GetArlRouteMsgType ();

    std::cout << "[NWK " << m_nodeId
              << "] ARL route message from " << helloSrc
              << " type=" << ArlRouteMsgTypeName (type)
              << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
              << std::endl;

    switch (type)
      {
      case CsrArlRouteMsgType::Discover:
        ProcessDiscover (hh, helloSrc, pathlossDb, snrDb, linkCost);
        break;

      case CsrArlRouteMsgType::RoutingUpdate:
        ProcessRoutingUpdate (hh, helloSrc, pathlossDb, snrDb, linkCost);
        break;

      /*case CsrArlRouteMsgType::NeighborCheck:
        std::cout << "[NWK " << m_nodeId
                  << "] NeighborCheck handling is not implemented yet"
                  << std::endl;
        break;*/

      case CsrArlRouteMsgType::NeighborCheck:
        ProcessNeighborCheck (hh, helloSrc, pathlossDb, snrDb, linkCost);
        break;

      case CsrArlRouteMsgType::KeyRequest:
        std::cout << "[NWK " << m_nodeId
                  << "] KeyRequest handling is not implemented yet"
                  << std::endl;
        break;

      case CsrArlRouteMsgType::None:
      default:
        // Backward-compatible fallback: if a packet has route ads but no type,
        // process them as a routing update.
        if (hh.GetAdvertisedRouteCount () > 0)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Untyped ARL payload has routes; treating as RoutingUpdate"
                      << std::endl;

            ProcessRoutingUpdate (hh, helloSrc, pathlossDb, snrDb, linkCost);
          }
        break;
      }
}

void
CsrNetLayer::ProcessRoutesPayload (const CsrHelloHeader &hh,
                                    uint16_t helloSrc,
                                    double pathlossDb,
                                    double snrDb,
                                    uint32_t linkCost)
{
    uint8_t advCount = hh.GetAdvertisedRouteCount ();

    if (advCount == 0)
      {
        return;
      }

    std::cout << "[NWK " << m_nodeId
              << "] Processing Routes_PAYLOAD from "
              << helloSrc
              << " advCount=" << unsigned (advCount)
              << std::endl;

    for (uint8_t idx = 0; idx < advCount; ++idx)
      {
        auto ar = hh.GetAdvertisedRoute (idx);

        if (ar.dst == CSR_BROADCAST_ID ||
            ar.dst == m_nodeId ||
            ar.dst == helloSrc)
          {
            continue;
          }

        uint8_t totalHops = static_cast<uint8_t> (ar.hops + 1);
        uint32_t totalCost = linkCost + ar.cost;

        bool accepted =
          AddOrUpdateRoute (ar.dst,
                            helloSrc,
                            false,
                            totalHops,
                            pathlossDb,
                            linkCost,
                            ar.cost,
                            helloSrc,
                            ar.capability);

        if (accepted)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Accepted advertised route dst=" << ar.dst
                      << " via nextHop=" << helloSrc
                      << " hops=" << unsigned (totalHops)
                      << " advCost=" << ar.cost
                      << " totalCost=" << totalCost
                      << std::endl;
          }
        else
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Rejected advertised route dst=" << ar.dst
                      << " via nextHop=" << helloSrc
                      << " hops=" << unsigned (totalHops)
                      << " advCost=" << ar.cost
                      << " totalCost=" << totalCost
                      << std::endl;
          }
      }
}

void
CsrNetLayer::ProcessRoutingUpdate (const CsrHelloHeader &hh,
                                   uint16_t helloSrc,
                                   double pathlossDb,
                                   double snrDb,
                                   uint32_t linkCost)
{
  std::cout << "[NWK " << m_nodeId
            << "] ProcessRoutingUpdate from " << helloSrc
            << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
            << std::endl;

  ProcessRoutesPayload (hh, helloSrc, pathlossDb, snrDb, linkCost);
}

void
CsrNetLayer::StartNeighborFreshnessMonitor (Time timeout, Time period)
{
  m_neighborFreshnessTimeout = timeout;
  m_neighborFreshnessCheckPeriod = period;

  if (m_neighborFreshnessEvent.IsPending ())
    {
      Simulator::Cancel (m_neighborFreshnessEvent);
    }

  std::cout << "[NWK " << m_nodeId
            << "] Starting neighbor freshness monitor timeout="
            << m_neighborFreshnessTimeout.GetSeconds ()
            << "s period="
            << m_neighborFreshnessCheckPeriod.GetSeconds ()
            << "s"
            << std::endl;

  m_neighborFreshnessEvent =
    Simulator::Schedule (m_neighborFreshnessCheckPeriod,
                         &CsrNetLayer::CheckNeighborFreshness,
                         this);
}

void
CsrNetLayer::CheckNeighborFreshness ()
{
  double now = Simulator::Now ().GetSeconds ();
  double timeoutSec = m_neighborFreshnessTimeout.GetSeconds ();

  for (auto &kv : m_nwkNeighbors)
    {
      NwkNeighborEntry &ne = kv.second;

      if (ne.lastHeardSec < 0.0)
        {
          continue;
        }

      double ageSec = now - ne.lastHeardSec;

      if (!ne.stale && ageSec > timeoutSec)
        {
          ne.stale = true;

          std::cout << "[NWK " << m_nodeId
                    << "] Neighbor stale nextHop=" << ne.nodeId
                    << " age=" << ageSec
                    << "s timeout=" << timeoutSec
                    << "s"
                    << std::endl;

          if (m_invalidateRoutesOnStaleNeighbor)
            {
              InvalidateRoutesViaNextHop (ne.nodeId, "neighbor stale");
            }
          else
            {
              std::cout << "[NWK " << m_nodeId
                        << "] Stale neighbor " << ne.nodeId
                        << " marked stale, but routes are preserved"
                        << " because invalidate_routes_on_stale_neighbor=false"
                        << std::endl;
            }
        }
    }

  m_neighborFreshnessEvent =
    Simulator::Schedule (m_neighborFreshnessCheckPeriod,
                         &CsrNetLayer::CheckNeighborFreshness,
                         this);
}

void
CsrNetLayer::InvalidateRoutesViaNextHop (uint16_t nextHop, const char *reason)
{
  bool anyInvalidated = false;

  for (auto &re : m_routes)
    {
      if (!re.valid)
        {
          continue;
        }

      if (re.nextHop != nextHop)
        {
          continue;
        }

      re.valid = false;
      re.lastUpdated = Simulator::Now ();
      anyInvalidated = true;

      std::cout << "[NWK " << m_nodeId
                << "] Invalidated route dst=" << re.nwkDst
                << " nextHop=" << re.nextHop
                << " cost=" << re.cost
                << " reason=" << reason
                << std::endl;
    }

  if (anyInvalidated)
    {
      DumpRoutes ();
      ScheduleCheckNwkQueue ();
    }
}

void
CsrNetLayer::ProcessDiscover (const CsrHelloHeader &hh,
                                uint16_t helloSrc,
                                double pathlossDb,
                                double snrDb,
                                uint32_t linkCost)
{
  std::cout << "[NWK " << m_nodeId
            << "] ProcessDiscover from " << helloSrc
            << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
            << std::endl;

    if (hh.GetAdvertisedRouteCount () > 0)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Ignoring Routes_PAYLOAD carried inside Discover from "
                  << helloSrc
                  << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
                  << " ; routes are only processed from RoutingUpdate"
                  << std::endl;
      }

    // Discovery has already updated neighbor state and the direct route in
    // ProcessHello(). Do not process advertised routes here.
    (void) pathlossDb;
    (void) snrDb;
    (void) linkCost;
    /*std::cout << "[NWK " << m_nodeId
              << "] ProcessDiscover from " << helloSrc
              << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
              << std::endl;

    // In current NS-3 approximation, Discover may still carry route info.
    // Legacy ARL routes.c treats Discover as part of discovery state; we keep
    // route payload processing here for now to preserve behavior.
    ProcessRoutesPayload (hh, helloSrc, pathlossDb, snrDb, linkCost);*/
}

/*void
CsrNetLayer::ProcessNeighborCheck (const CsrHelloHeader &hh,
                                   uint16_t helloSrc,
                                   double pathlossDb,
                                   double snrDb,
                                   uint32_t linkCost)
{
  std::cout << "[NWK " << m_nodeId
            << "] ProcessNeighborCheck from " << helloSrc
            << " pathloss=" << pathlossDb
            << " snr=" << snrDb
            << " linkCost=" << linkCost
            << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
            << std::endl;

  if (hh.GetAdvertisedRouteCount () > 0)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring Routes_PAYLOAD carried inside NeighborCheck from "
                << helloSrc
                << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
                << " ; routes are only processed from RoutingUpdate"
                << std::endl;
    }

  // Neighbor state and direct-route refresh already happened in ProcessHello().
  // NeighborCheck is a liveness/link-quality probe for now.
}*/

void
CsrNetLayer::ProcessNeighborCheck (const CsrHelloHeader &hh,
                                   uint16_t helloSrc,
                                   double pathlossDb,
                                   double snrDb,
                                   uint32_t linkCost)
{
  CsrNeighborCheckType type = hh.GetNeighborCheckType ();

  std::cout << "[NWK " << m_nodeId
            << "] ProcessNeighborCheck from " << helloSrc
            << " subtype=" << NeighborCheckTypeName (type)
            << " pathloss=" << pathlossDb
            << " snr=" << snrDb
            << " linkCost=" << linkCost
            << " advCount=" << unsigned (hh.GetAdvertisedRouteCount ())
            << std::endl;

  switch (type)
    {
    case CsrNeighborCheckType::Discovery:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck Discovery confirms discovery reception from "
                << helloSrc
                << std::endl;
      break;

    case CsrNeighborCheckType::Message:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck Message confirms link with "
                << helloSrc
                << std::endl;
      break;

    case CsrNeighborCheckType::Verify:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck Verify received from "
                << helloSrc
                << std::endl;
      break;

    case CsrNeighborCheckType::Overheard:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck Overheard received from "
                << helloSrc
                << std::endl;
      break;

    case CsrNeighborCheckType::NoPath:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck NoPath received from "
                << helloSrc
                << " ; destination payload not implemented yet"
                << std::endl;
      break;

    case CsrNeighborCheckType::None:
    default:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck missing or unknown subtype from "
                << helloSrc
                << std::endl;
      break;
    }

  if (hh.GetAdvertisedRouteCount () > 0)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring Routes_PAYLOAD inside NeighborCheck"
                << std::endl;
    }

  // Neighbor freshness and the direct route were already refreshed
  // by ProcessHello() before this subtype handler was called.
}

void CsrNetLayer::EnsureDiscoveryForTx ()
{
  if (m_discState != DiscoveryState::IDLE)
    {
      return;
    }

  std::cout << "[NWK " << m_nodeId << "] No route/next-hop -> starting on-demand discovery\n";
  StartDiscovery (Seconds (0.0), Seconds (30.0));
}

void CsrNetLayer::SetRepeatDiscoveryHello (bool enable)
{
  m_repeatDiscoveryHello = enable;
}

void
CsrNetLayer::DiscoveryStart ()
{
  m_discState = DiscoveryState::ACTIVE;
  m_discoveryActive = true;

  std::cout << "[NWK " << m_nodeId
              << "] DiscoveryStart: sending HELLO advertisement"
              << std::endl;

    // OPNET-style behavior: send one HELLO at discovery start
  SendHelloBroadcast (CsrArlRouteMsgType::Discover);
    //SendHelloBroadcast ();

    // Optional NS-3 robustness mode, disabled by default
  if (m_repeatDiscoveryHello)
    {
      ScheduleDiscoveryHello ();
    }
}

  void
  CsrNetLayer::DiscoveryStop ()
  {
    if (m_discoveryHelloEvent.IsPending ())
      {
        Simulator::Cancel (m_discoveryHelloEvent);
      }

    m_discState = DiscoveryState::COOLDOWN;
    m_discoveryActive = false;

    std::cout << "[NWK " << m_nodeId
              << "] DiscoveryStop"
              << std::endl;

    Simulator::Schedule (m_discoveryCooldown,
                        &CsrNetLayer::DiscoveryCooldownOver,
                        this);

    TryDrainQueueAfterDiscovery ();
  }

void
CsrNetLayer::TryDrainQueueAfterDiscovery ()
{
    // OPNET-equivalent of re-entering the NWK process after discovery
  CheckNwkQueue ();
}

void
CsrNetLayer::ScheduleDiscoveryHello ()
{
  if (m_discState != DiscoveryState::ACTIVE)
    {
      return;
    }

  if (!m_discoveryHelloEvent.IsPending ())
    {
      m_discoveryHelloEvent =
        Simulator::Schedule (m_discoveryHelloInterval,
                              &CsrNetLayer::DiscoveryHelloTick,
                              this);
    }
}

void
CsrNetLayer::DiscoveryHelloTick ()
{
  if (m_discState != DiscoveryState::ACTIVE)
    {
      return;
    }

  std::cout << "[NWK " << m_nodeId
              << "] Discovery HELLO repeat"
              << std::endl;

  SendHelloBroadcast ();

  ScheduleDiscoveryHello ();
}

void
CsrNetLayer::SendHelloBroadcast (
  CsrArlRouteMsgType type,
  CsrNeighborCheckType checkType)
{
  if (!m_hop) return;

  Ptr<Packet> p = Create<Packet> ();

  static uint16_t helloSeq = 0;

  CsrHelloHeader hh;
  hh.SetNodeId (m_nodeId);
  hh.SetHelloSeq (++helloSeq);

    // Keep it simple: speedKey is what you actually use in CSR headers anyway
  hh.SetSpeedKey (m_minSpeedKey);   // define m_minSpeedKey or hardcode 8 temporarily

    // Integer scaled dBm*10, placeholder until you compute it properly
    //hh.SetRxPowerDbmX10 (-900);        // -90.0 dBm
  double s0PowerDbm = m_rxS0BaseLevelDbm + m_linkMarginDb;
  hh.SetRxPowerDbmX10 (static_cast<int16_t> (std::round (s0PowerDbm * 10.0)));

    // OPNET-ish “active” proxy: neighbor count (or 0 for now)
    //hh.SetActiveNodes (static_cast<uint8_t>(GetNeighborCount ()));
  hh.SetActiveNodes (static_cast<uint8_t> (GetActiveNodeCount ()));

  hh.ClearAdvertisedRoutes ();

  hh.SetArlRouteMsgType (type);

  hh.SetNeighborCheckType (checkType);

  /*for (const auto &re : m_routes)
    {
      if (!ShouldAdvertiseRoute (re))
        {
          continue;
        }

      int16_t plX10 = 0;
      if (!std::isnan (re.pathlossDb))
        {
          plX10 = static_cast<int16_t> (std::round (re.pathlossDb * 10.0));
         }

      if (hh.AddAdvertisedRoute (re.nwkDst,
                                  re.numHop,
                                  re.cost,
                                  plX10,
                                  re.capability))
        {
          added++;

          std::cout << "[NWK " << m_nodeId
                      << "] HELLO add route adv dst=" << re.nwkDst
                      << " hops=" << unsigned (re.numHop)
                      << " cost=" << re.cost
                      << " linkCost=" << re.linkCostToNextHop
                      << " advCost=" << re.advertisedCost
                      << " learnedFrom=" << re.learnedFrom
                      << std::endl;
        }

      if (added >= 8)
        {
           break;
        }
    }*/

  uint8_t added = 0;

  if (type == CsrArlRouteMsgType::RoutingUpdate)
    {
      for (const auto &re : m_routes)
        {
          if (!ShouldAdvertiseRoute (re))
            {
              continue;
            }

          int16_t plX10 = 0;
          if (!std::isnan (re.pathlossDb))
            {
              plX10 = static_cast<int16_t> (std::round (re.pathlossDb * 10.0));
            }

          if (hh.AddAdvertisedRoute (re.nwkDst,
                                    re.numHop,
                                    re.cost,
                                    plX10,
                                    re.capability))
            {
              added++;

              std::cout << "[NWK " << m_nodeId
                        << "] HELLO add route adv dst=" << re.nwkDst
                        << " hops=" << unsigned (re.numHop)
                        << " cost=" << re.cost
                        << " linkCost=" << re.linkCostToNextHop
                        << " advCost=" << re.advertisedCost
                        << " learnedFrom=" << re.learnedFrom
                        << std::endl;
            }

          if (added >= 8)
            {
              break;
            }
        }
    }
  else
    {
      std::cout << "[NWK " << m_nodeId
                << "] ARL " << ArlRouteMsgTypeName (type)
                << " sent without Routes_PAYLOAD"
                << std::endl;
    }

    std::cout << "[NWK " << m_nodeId
              << "] HELLO advertising "
              << unsigned (added)
              << " routes"
              << std::endl;

    std::cout << "[NWK " << m_nodeId
              << "] HELLO ARL route msg type="
              << unsigned (static_cast<uint8_t> (hh.GetArlRouteMsgType ()))
              << std::endl;

    p->AddHeader (hh);

    m_hop->SendHello (p); // HOP wraps outer CsrHeader + broadcasts

}

uint32_t
CsrNetLayer::GetActiveNodeCount () const
{
  return static_cast<uint32_t> (m_nwkNeighbors.size () + 1);
}

uint32_t
CsrNetLayer::GetNeighborCount () const
{
  return static_cast<uint32_t> (m_nwkNeighbors.size ());
}

void
CsrNetLayer::SendRoutingUpdate ()
{
  std::cout << "[NWK " << m_nodeId
            << "] Sending ARL RoutingUpdate"
            << std::endl;

  SendHelloBroadcast (CsrArlRouteMsgType::RoutingUpdate);
}

/*void
CsrNetLayer::SendNeighborCheck ()
{
  std::cout << "[NWK " << m_nodeId
            << "] Sending ARL NeighborCheck"
            << std::endl;

  SendHelloBroadcast (CsrArlRouteMsgType::NeighborCheck);
}*/

void
CsrNetLayer::SendNeighborCheck (CsrNeighborCheckType type)
{
  std::cout << "[NWK " << m_nodeId
            << "] Sending ARL NeighborCheck subtype="
            << unsigned (static_cast<uint8_t> (type))
            << std::endl;

  SendHelloBroadcast (CsrArlRouteMsgType::NeighborCheck, type);
}

void
CsrNetLayer::DiscoveryCooldownOver ()
{
    // OPNET-equivalent: discovery is fully complete and can be triggered again
  m_discState = DiscoveryState::IDLE;

    // Clear legacy flag if you still have it
  m_discoveryActive = false;

    // Optional but safe: try to forward anything that may now succeed
   CheckNwkQueue ();
}

bool
CsrNetLayer::ShouldAdvertiseRoute (const RouteEntry &re) const
{
  static constexpr uint8_t MAX_ADVERTISED_HOPS = 8;

  if (!re.valid)
    {
      return false;
    }

  if (re.nwkDst == m_nodeId)
    {
      return false;
    }

  if (re.cost == 0)
    {
      return false;
    }

  if (re.numHop == 0)
    {
      return false;
    }

  if (re.numHop >= MAX_ADVERTISED_HOPS)
    {
      return false;
    }

  return true;
}

const char*
CsrNetLayer::NeighborCheckTypeName (CsrNeighborCheckType t) const
{
  switch (t)
    {
    case CsrNeighborCheckType::Discovery:
      return "Discovery";

    case CsrNeighborCheckType::Message:
      return "Message";

    case CsrNeighborCheckType::NoPath:
      return "NoPath";

    case CsrNeighborCheckType::Overheard:
      return "Overheard";

    case CsrNeighborCheckType::Verify:
      return "Verify";

    case CsrNeighborCheckType::None:
    default:
      return "None";
    }
}
// ------------------------------------------------------------
