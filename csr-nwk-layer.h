#pragma once
#include "csr-common.h"

class CsrNetLayer : public Object
{
public:
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

  void SetHop (Ptr<CsrHopLayer> hop)
  {
    m_hop = hop;
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
        //std::cout << "[NWK " << m_nodeId << "] NSDP(" << src
        //  << "->" << dst << ") decremented to " << e.count << std::endl;
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

    // Add a static route (for now): nwkDst -> nextHop
  void AddStaticRoute (uint16_t nwkDst, uint16_t nextHop)
  {
    RouteEntry re;
    re.nwkDst = nwkDst;
    re.nextHop = nextHop;
    re.cost = 1;
    m_routes.push_back (re);
  }

private:
  struct RouteEntry
  {
    uint16_t nwkDst;   // network-layer destination
    uint16_t nextHop;  // hop-level next hop
    uint32_t cost;     // optional, for future use
  };

  struct NwkQueueEntry
  {
    uint16_t   nwkSrc;
    uint16_t   nwkDst;
    uint8_t    dscp;
    bool       ack;
    Ptr<Packet> payload;
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
          NS_LOG_INFO ("CsrNetLayer[" << m_nodeId << "]: no route to nwkDst="
                                      << e.nwkDst << "; dropping packet.");
          m_nwkQueue.pop_front ();
          continue;
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
    for (const auto &re : m_routes)
      {
        if (re.nwkDst == nwkDst)
          {
            nextHopOut = re.nextHop;
            return true;
          }
      }
    return false;
  }

private:
  uint16_t                              m_nodeId;
  Ptr<CsrHopLayer>                      m_hop;
  Callback<void, Ptr<Packet>, uint16_t> m_rxFromNetCb;

  std::deque<NwkQueueEntry>             m_nwkQueue;
  EventId                               m_checkNwkQueueEvent;
  std::vector<RouteEntry>               m_routes;

  std::map<std::pair<uint16_t,uint16_t>, NsdpEntry> m_nsdp;
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

// ------------------------------------------------------------
