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

 void AddOrUpdateRoute (uint16_t nwkDst,
                  uint16_t nextHop,
                  bool immediate,
                  uint8_t numHop,
                  double pathlossDb,
                  uint32_t cost,
                  uint8_t capability = 0)
  {
    if (nwkDst == m_nodeId)
      {
        return;
      }

    for (auto &re : m_routes)
      {
        if (re.nwkDst == nwkDst)
          {
            bool sameNextHop = (re.nextHop == nextHop);
            bool betterCost  = (cost + 5 < re.cost);
            bool invalid     = !re.valid;

            if (sameNextHop || betterCost || invalid)
              {
                re.capability = capability;
                re.immediate = immediate;
                re.nextHop = nextHop;
                re.pathlossDb = pathlossDb;
                re.numHop = numHop;
                re.cost = cost;
                re.lastUpdated = Simulator::Now ();
                re.valid = true;

                std::cout << "[NWK " << m_nodeId
                          << "] Updated route dst=" << nwkDst
                          << " nextHop=" << nextHop
                          << " hops=" << unsigned (numHop)
                          << " cost=" << cost
                          << " pathloss=" << pathlossDb
                          << std::endl;
              }

            return;
          }
      }

    RouteEntry re;
    re.nwkDst = nwkDst;
    re.capability = capability;
    re.immediate = immediate;
    re.nextHop = nextHop;
    re.pathlossDb = pathlossDb;
    re.numHop = numHop;
    re.cost = cost;
    re.energyLevel = 100;
    re.lastUpdated = Simulator::Now ();
    re.valid = true;

    m_routes.push_back (re);

    std::cout << "[NWK " << m_nodeId
              << "] Added route dst=" << nwkDst
              << " nextHop=" << nextHop
              << " hops=" << unsigned (numHop)
              << " cost=" << cost
              << " pathloss=" << pathlossDb
              << std::endl;
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
                      0);
  }

    // Add a static route (for now): nwkDst -> nextHop
  /*  void AddStaticRoute (uint16_t nwkDst, uint16_t nextHop)
  {
    for (auto &re : m_routes)
      {
        if (re.nwkDst == nwkDst)
          {
            re.nextHop = nextHop;
            re.cost = 1;

            std::cout << "[NWK " << m_nodeId
                      << "] Updated route dst=" << nwkDst
                      << " nextHop=" << nextHop
                      << std::endl;
            return;
          }
      }

    RouteEntry re;
    re.nwkDst = nwkDst;
    re.nextHop = nextHop;
    re.cost = 1;
    m_routes.push_back (re);

    std::cout << "[NWK " << m_nodeId
              << "] Added route dst=" << nwkDst
              << " nextHop=" << nextHop
              << std::endl;
  }*/

  /*void AddStaticRoute (uint16_t nwkDst, uint16_t nextHop)
  {
    RouteEntry re;
    re.nwkDst = nwkDst;
    re.nextHop = nextHop;
    re.cost = 1;
    m_routes.push_back (re);
  }*/

private:
 /* struct RouteEntry
  {
    uint16_t nwkDst;   // network-layer destination
    uint16_t nextHop;  // hop-level next hop
    uint32_t cost;     // optional, for future use
  };*/

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

  /*bool LookupNextHop (uint16_t nwkDst, uint16_t &nextHopOut)
  {
    for (const auto &re : m_routes)
      {
        if (re.nwkDst == nwkDst)
          {
            nextHopOut = re.nextHop;
            return true;
          }
      }
    return false;
  }*/

  uint32_t
  ComputeLinkCost (double pathlossDb, double snrDb) const
  {
    // Temporary NS-3 cost model.
    // Later we can replace this with the exact OPNET link_calc() behavior.
    double plPenalty = std::max (0.0, pathlossDb - 80.0);
    double snrPenalty = std::max (0.0, 10.0 - snrDb) * 5.0;

    uint32_t cost = static_cast<uint32_t> (std::round (10.0 + plPenalty + snrPenalty));

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

  uint8_t m_minSpeedKey { 8 };  // temporary default; map to your actual rate table later

  void DiscoveryStart ();
  void DiscoveryStop ();
  void DiscoveryCooldownOver ();
  void SendHelloBroadcast ();

  void EnsureDiscoveryForTx ();

  void TryDrainQueueAfterDiscovery ();

  void ScheduleDiscoveryHello ();
  void DiscoveryHelloTick ();

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

  /*void CsrNetLayer::StartDiscovery (Time startDelay, Time duration)
  {
    if (m_discoveryStartEvent.IsPending ()) Simulator::Cancel (m_discoveryStartEvent);
    if (m_discoveryStopEvent.IsPending  ()) Simulator::Cancel (m_discoveryStopEvent);

    // If we're already scheduled/active/cooldown, don't restart (OPNET-like gating)
    if (m_discState == DiscoveryState::SCHEDULED ||
        m_discState == DiscoveryState::ACTIVE ||
        m_discState == DiscoveryState::COOLDOWN)
      {
        return;
      }

    // IMPORTANT: mark as scheduled immediately (prevents duplicate triggers)
    m_discState = DiscoveryState::SCHEDULED;

    m_discoveryStartEvent = Simulator::Schedule (startDelay, &CsrNetLayer::DiscoveryStart, this);
    m_discoveryStopEvent  = Simulator::Schedule (startDelay + duration, &CsrNetLayer::DiscoveryStop, this);
  }*/

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

    ne.nodeId = src;
    ne.lastHeardSec = now;
    ne.lastPathlossDb = pathlossDb;
    ne.lastSnrDb = snrDb;
    ne.speedKey = hh.GetSpeedKey ();
    ne.rxPowerDbmX10 = hh.GetRxPowerDbmX10 ();
    ne.activeNodes = hh.GetActiveNodes ();

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
    uint32_t linkCost = ComputeLinkCost (pathlossDb, snrDb);

    AddOrUpdateRoute (src,
                      src,
                      true,       // immediate neighbor
                      1,          // one hop
                      pathlossDb,
                      linkCost,
                      0);         // capability placeholder

    // ------------------------------------------------------------
    // 3) Advertised route from HELLO sender
    //    If src advertises dst=X, then we can reach X via src.
    // ------------------------------------------------------------
    
    uint8_t advCount = hh.GetAdvertisedRouteCount ();

    for (uint8_t idx = 0; idx < advCount; ++idx)
      {
        auto ar = hh.GetAdvertisedRoute (idx);

        if (ar.dst == CSR_BROADCAST_ID ||
            ar.dst == m_nodeId ||
            ar.dst == src)
          {
            continue;
          }

        uint8_t totalHops = static_cast<uint8_t> (ar.hops + 1);
        uint32_t totalCost = linkCost + ar.cost;

        AddOrUpdateRoute (ar.dst,
                          src,          // next hop is HELLO sender
                          false,        // not immediate
                          totalHops,
                          pathlossDb,
                          totalCost,
                          ar.capability);

        std::cout << "[NWK " << m_nodeId
                  << "] Learned advertised route dst=" << ar.dst
                  << " via nextHop=" << src
                  << " hops=" << unsigned (totalHops)
                  << " advCost=" << ar.cost
                  << " totalCost=" << totalCost
                  << std::endl;
      }

    DumpRoutes ();

    // Discovery/route update may have unblocked queued packets.
    ScheduleCheckNwkQueue ();
  }
  /*void
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

    uint16_t advDst = hh.GetAdvertisedDst ();
    uint8_t advHops = hh.GetAdvertisedHops ();  

    uint16_t src = hh.GetNodeId ();

    if (src == m_nodeId)
      {
        return;
      }

    if (advDst != CSR_BROADCAST_ID &&
      advDst != m_nodeId &&
      advDst != src)
    {
      AddStaticRoute (advDst, src);  // destination via HELLO sender

      std::cout << "[NWK " << m_nodeId
                << "] Added advertised route dst=" << advDst
                << " nextHop=" << src
                << " advertisedHops=" << unsigned (advHops)
                << std::endl;
    }
    
    double now = Simulator::Now ().GetSeconds ();

    auto &ne = m_nwkNeighbors[src];
    bool isNew = (ne.lastHeardSec < 0.0);

    ne.nodeId = src;
    ne.lastHeardSec = now;
    ne.lastPathlossDb = pathlossDb;
    ne.lastSnrDb = snrDb;
    ne.speedKey = hh.GetSpeedKey ();
    ne.rxPowerDbmX10 = hh.GetRxPowerDbmX10 ();
    ne.activeNodes = hh.GetActiveNodes ();

    std::cout << "[NWK " << m_nodeId << "] "
              << (isNew ? "New" : "Updated")
              << " HELLO neighbor=" << src
              << " hopSrc=" << hopSrc
              << " speedKey=" << unsigned (ne.speedKey)
              << " activeNodes=" << unsigned (ne.activeNodes)
              << " pathloss=" << pathlossDb
              << " snr=" << snrDb
              << std::endl;

    // Minimal OPNET-parity route population:
    // if we heard node src directly, route to src via src.
    bool haveRoute = false;
    for (auto &re : m_routes)
      {
        if (re.nwkDst == src)
          {
            re.nextHop = src;
            re.cost = 1;
            haveRoute = true;
            break;
          }
      }

    if (!haveRoute)
      {
        AddStaticRoute (src, src);
      }

    DumpRoutes ();

    ScheduleCheckNwkQueue ();
  }*/

/*void CsrNetLayer::EnsureDiscoveryForTx ()
{
  if (!m_discoveryActive)
    {
      std::cout << "[NWK " << m_nodeId << "] No route/next-hop -> starting on-demand discovery" << std::endl;
      StartDiscovery (Seconds (0.0), Seconds (30.0));
    }
}*/

/*void CsrNetLayer::EnsureDiscoveryForTx ()
{
  if (m_discoveryActive)
    {
      return;
    }

  // Set state first so repeated callers in same time slice won't retrigger
  m_discoveryActive = true;
  m_discoveryInitiatedBy = DISCOVERY_REASON_NO_ROUTE;   // optional but very OPNET-ish
  m_discoveryAttempts++;                                // optional counter (OPNET vibe)

  std::cout << "[NWK " << m_nodeId << "] No route/next-hop -> starting on-demand discovery\n";

  StartDiscovery (Seconds (0.0), Seconds (30.0));
}*/

  void CsrNetLayer::EnsureDiscoveryForTx ()
  {
    if (m_discState != DiscoveryState::IDLE)
      {
        return;
      }

    std::cout << "[NWK " << m_nodeId << "] No route/next-hop -> starting on-demand discovery\n";
    StartDiscovery (Seconds (0.0), Seconds (30.0));
  }


  /*void CsrNetLayer::DiscoveryStart ()
  {
    m_discState = DiscoveryState::ACTIVE;

    // OPNET start_discovery(): schedule stop already done outside (you did that), now send HELLO
    SendHelloBroadcast();   // NWK->HOP HELLO, with OPNET-ish fields
  }*/

  void
  CsrNetLayer::DiscoveryStart ()
  {
    m_discState = DiscoveryState::ACTIVE;
    m_discoveryActive = true;

    std::cout << "[NWK " << m_nodeId
              << "] DiscoveryStart: sending HELLO advertisements"
              << std::endl;

    SendHelloBroadcast ();

    ScheduleDiscoveryHello ();
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

  /*void CsrNetLayer::DiscoveryStop ()
  {
    m_discState = DiscoveryState::COOLDOWN;

    // OPNET clear_discovery(): finalize + notify + stop discovery behavior
    // (your minimal version can just switch off + trigger queue drain attempt)
    Simulator::Schedule (m_discoveryCooldown, &CsrNetLayer::DiscoveryCooldownOver, this);

    TryDrainQueueAfterDiscovery(); // re-run CheckNwkQueue or schedule it
  }*/

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
  CsrNetLayer::SendHelloBroadcast ()
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
    hh.SetRxPowerDbmX10 (-900);        // -90.0 dBm

    // OPNET-ish “active” proxy: neighbor count (or 0 for now)
    hh.SetActiveNodes (static_cast<uint8_t>(GetNeighborCount ()));

    hh.ClearAdvertisedRoutes ();

    uint8_t added = 0;
    for (const auto &re : m_routes)
      {
        if (!re.valid)
          {
            continue;
          }

        if (re.nwkDst == m_nodeId)
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
          }

        if (added >= 8)
          {
            break;
          }
      }

    std::cout << "[NWK " << m_nodeId
              << "] HELLO advertising "
              << unsigned (added)
              << " routes"
              << std::endl;
              
    // Temporary/simple: advertise first route in m_routes, if any.
    if (!m_routes.empty ())
      {
        hh.SetAdvertisedDst (m_routes.front ().nwkDst);
        hh.SetAdvertisedHops (static_cast<uint8_t> (m_routes.front ().cost));
      }
    else
      {
        hh.SetAdvertisedDst (CSR_BROADCAST_ID);
        hh.SetAdvertisedHops (0);
      }

    p->AddHeader (hh);

    m_hop->SendHello (p); // HOP wraps outer CsrHeader + broadcasts
  }

  uint32_t
  CsrNetLayer::GetNeighborCount () const
  {
    return static_cast<uint32_t> (m_nwkNeighbors.size ());
  }

/*uint32_t
CsrNetLayer::GetNeighborCount () const
{
  return 0;
}*/

/*void
CsrNetLayer::SendHelloBroadcast ()
{
  if (!m_hop) return;

  // Build HELLO payload with CsrHelloHeader
  Ptr<Packet> p = Create<Packet> ();

  CsrHelloHeader hh;
  hh.SetNodeId (m_nodeId);
  hh.SetSource (m_nodeId);
  hh.SetDestination (CSR_BROADCAST_ID);
  hh.SetDestType (CSR_DEST_BROADCAST);

  // Default values - can be configured later
  hh.SetTxPower (30.0);      // max TX power dBm
  hh.SetSpeed (8.0);         // min speed key
  hh.SetRxPower (-90.0);     // RX threshold
  hh.SetCapability (0);
  hh.SetActive (0);

  p->AddHeader (hh);

  m_hop->SendHello (p);
}*/

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

// ------------------------------------------------------------
