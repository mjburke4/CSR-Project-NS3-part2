#pragma once
#include "csr-common.h"
#include "csr-hello-header.h"
#include "csr-hop-layer.h"
#include <cmath>
#include <set>
#include <algorithm>


class CsrNetLayer : public Object
{
public:

  enum class DiscoveryState { IDLE, SCHEDULED, ACTIVE, COOLDOWN };

  DiscoveryState m_discState { DiscoveryState::IDLE };
  EventId m_discoveryStartEvent, m_discoveryStopEvent, m_discoveryCooldownEvent;
  Time m_discoveryCooldown { Seconds(5) }; // tune later
  EventId m_discoveryHelloEvent;
  Time m_discoveryHelloInterval { Seconds (1.0) };
  uint32_t m_routingSequence {0};

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
  void ScheduleGatewayStartupDiscovery (
    Time delay = Seconds (10.0),
    Time duration = Seconds (30.0));
  void SetRepeatDiscoveryHello (bool enable);
  void SetDiscoveryResponseEnabled (bool enable);
  void SendRoutingUpdate ();
  void SendRoutingUpdateWithSequenceForTest (uint32_t sequence);

  void SendDiscoveryChirp ();

  void SendNeighborCheck (
    uint16_t neighbor,
    CsrNeighborCheckType type = CsrNeighborCheckType::Message,
    uint16_t target = CSR_BROADCAST_ID,
    uint32_t discoverySequence = 0);

  void StartReliableRoutingSnapshot (
    uint16_t neighbor);

  void
  RefreshRoutesAfterDiscovery ();

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

  void
  SendNoPath (uint16_t neighbor, uint16_t unreachableDest)
  {
    SendNeighborCheck (neighbor,
                      CsrNeighborCheckType::NoPath,
                      unreachableDest);
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
  NoteNeighborCheckSuccess (
    uint16_t neighbor,
    CsrNeighborCheckType type,
    uint32_t discoverySequence)
  {
    auto it = m_nwkNeighbors.find (neighbor);

    if (type == CsrNeighborCheckType::Verify &&
      discoverySequence != 0 &&
      discoverySequence != m_discoverySequence)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Ignoring stale discovery Verify completion"
                  << " neighbor=" << neighbor
                  << " completedSequence="
                  << discoverySequence
                  << " currentSequence="
                  << m_discoverySequence
                  << std::endl;

        return;
      }

    if (it == m_nwkNeighbors.end ())
      {
        std::cout << "[NWK " << m_nodeId
                  << "] NeighborCheck ACK from unknown neighbor="
                  << neighbor
                  << std::endl;
        return;
      }

    NwkNeighborEntry &ne = it->second;

    uint32_t oldFailures = ne.numFailures;
    bool wasStale = ne.stale;

    ne.stale = false;
    ne.lastHeardSec = Simulator::Now ().GetSeconds ();

    // Conservative recovery: one successful check removes one failure.
    // Do not immediately erase all historical failures.
    if (ne.numFailures > 0)
      {
        ne.numFailures--;
      }

    std::cout << "[NWK " << m_nodeId
              << "] NeighborCheck success"
              << " neighbor=" << neighbor
              << " subtype="
              << NeighborCheckTypeName (type)
              << " discoverySequence="
              << discoverySequence
              << " stale=" << (wasStale ? 1 : 0)
              << "->0"
              << " numFailures=" << oldFailures
              << "->" << ne.numFailures
              << std::endl;

      SelectedRouteState directRouteBefore =
        CaptureSelectedRouteState (
          neighbor);

      if (wasStale)
        {
          uint32_t directRoutesReactivated = 0;
          uint32_t transitiveRoutesPreservedInvalid = 0;

          for (auto &route : m_routes)
            {
              if (route.nextHop != neighbor)
                {
                  continue;
                }

              bool directNeighborRoute =
                route.nwkDst == neighbor &&
                route.nextHop == neighbor &&
                route.immediate;

              if (directNeighborRoute)
                {
                  route.valid = true;
                  route.lastUpdated =
                    Simulator::Now ();

                  directRoutesReactivated++;

                  std::cout << "[NWK " << m_nodeId
                            << "] Reactivated direct neighbor route"
                            << " dst=" << route.nwkDst
                            << " nextHop=" << neighbor
                            << " after successful NeighborCheck"
                            << std::endl;
                }
              else if (!route.valid)
                {
                  // A successful NeighborCheck proves only that
                  // the neighbor is reachable. It does not prove
                  // that destinations previously advertised by
                  // that neighbor remain reachable.
                  transitiveRoutesPreservedInvalid++;

                  std::cout << "[NWK " << m_nodeId
                            << "] Preserving invalid transitive route"
                            << " dst=" << route.nwkDst
                            << " nextHop=" << neighbor
                            << " pending fresh RoutingUpdate"
                            << std::endl;
                }
            }

          std::cout << "[NWK " << m_nodeId
                    << "] Neighbor recovery route policy"
                    << " neighbor=" << neighbor
                    << " directRoutesReactivated="
                    << directRoutesReactivated
                    << " transitiveRoutesPreservedInvalid="
                    << transitiveRoutesPreservedInvalid
                    << std::endl;
        }

      // Recompute only currently valid routes.
      // RecomputeRoutesViaNextHop() already skips invalid entries.
      RecomputeRoutesViaNextHop (
        neighbor);

      SelectedRouteState directRouteAfter =
        CaptureSelectedRouteState (
          neighbor);

      if (!SameSelectedRouteState (
            directRouteBefore,
            directRouteAfter))
        {
          MarkSelectedRouteChanged (
            neighbor,
            "direct neighbor recovery");
        }

      ScheduleCheckNwkQueue ();
  }

  void
  RecomputeRoutesViaNextHop (
    uint16_t nextHop);

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

  void
  ClearRoutes ()
  {
    Time now = Simulator::Now ();

    uint32_t activeNeighborsCleared = 0;
    uint32_t routesInvalidated = 0;
    uint32_t reverseRoutesInvalidated = 0;

    bool chirpNeeded = false;

    // Legacy routesClearTable() makes every known neighbor inactive.
    // Keep the entries and measurements so later discovery can rebuild them.
    for (auto &kv : m_nwkNeighbors)
      {
        NwkNeighborEntry &neighbor = kv.second;

        bool wasActive =
          neighbor.lastHeardSec >= 0.0 &&
          !neighbor.stale;

        if (wasActive)
          {
            activeNeighborsCleared++;
            chirpNeeded = true;
          }

        neighbor.stale = true;
        neighbor.wasActiveBeforeLastHello = false;

        // Legacy routesMakeNeighborInactive() increments failures,
        // capped below an effectively unreachable upper bound.
        if (neighbor.numFailures < 200)
          {
            neighbor.numFailures++;
          }
      }

    // Preserve route records but mark them unusable. This allows later
    // discovery, Verify, or RoutingUpdate traffic to rebuild them.
    for (auto &route : m_routes)
      {
        if (route.valid)
          {
            route.valid = false;
            route.lastUpdated = now;
            routesInvalidated++;
          }
      }

    // Legacy routesClearTable() explicitly invalidates reverse routes.
    for (auto &kv : m_reverseRoutes)
      {
        ReverseRouteEntry &reverse = kv.second;

        if (reverse.valid)
          {
            reverse.valid = false;
            reverse.lastUpdated = now;
            reverseRoutesInvalidated++;
          }
      }

    uint32_t selectedPreferencesCleared =
      m_selectedRoutePreferredNextHop.size ();

    m_selectedRoutePreferredNextHop.clear ();

    UpdateMacActiveNodes ();

    std::cout << "[NWK " << m_nodeId
              << "] routesClearTable parity reset"
              << " activeNeighborsCleared="
              << activeNeighborsCleared
              << " routesInvalidated="
              << routesInvalidated
              << " reverseRoutesInvalidated="
              << reverseRoutesInvalidated
              << " selectedPreferencesCleared="
              << selectedPreferencesCleared
              << std::endl;

    DumpRoutes ();

    // Legacy code collapses multiple active-to-inactive transitions into
    // one pending Chirp.
    if (chirpNeeded && m_hop != nullptr)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] routesClearTable scheduling consolidated Chirp"
                  << std::endl;

        Simulator::ScheduleNow (
          &CsrNetLayer::SendDiscoveryChirp,
          this);
      }

    // Any queued packet must be reevaluated against the now-invalid routes.
    ScheduleCheckNwkQueue ();
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
	                      << ",path=[";

	                      for (size_t index = 0;
	                          index < re.path.size ();
	                          ++index)
                        {
                          if (index > 0)
                            {
                              std::cout << "->";
                            }

                          std::cout << re.path[index];
                        }

	                      std::cout << "]";
	                      std::cout << ")";
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

        m_hop->SetNeighborCheckSuccessCallback (
          MakeCallback (&CsrNetLayer::NoteNeighborCheckSuccess, this));

        m_hop->SetRoutingControlFailureCallback (
          MakeCallback (
            &CsrNetLayer::
              NoteRoutingControlFailure,
            this));

        m_hop->SetRoutingControlSuccessCallback (
          MakeCallback (
            &CsrNetLayer::
              NoteRoutingControlSuccess,
            this));
              }

        m_hop->SetNwkQueueWakeCallback (
          MakeCallback (
            &CsrNetLayer::
              ScheduleCheckNwkQueue,
            this));
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
    NsdpEntry &e =
      GetNsdpEntry (src, dst);

    if (e.count > 0)
      {
        e.count--;

        std::cout << "[NWK " << m_nodeId
                  << "] NSDP("
                  << src << "->" << dst
                  << ") decremented to "
                  << e.count
                  << std::endl;

        // Legacy HOP schedules a NWK queue check whenever
        // ACK/DACK/final-timeout processing releases flow
        // control capacity.
        std::cout << "[NWK " << m_nodeId
                  << "] HOP completion released NSDP slot;"
                  << " scheduling NWK queue check"
                  << " flow=" << src
                  << "->" << dst
                  << std::endl;

        ScheduleCheckNwkQueue ();
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
    UpdateReverseRoute (nwkSrc, hopSrc);

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

        if (!m_transitForwardingEnabled)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Refusing transit packet"
                      << " role=" << NodeTypeName (m_nodeType)
                      << " nwkSrc=" << nwkSrc
                      << " nwkDst=" << nwkDst
                      << " arrivedVia=" << hopSrc
                      << std::endl;

            SendNoPath (hopSrc, nwkDst);
            return;
          }
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
  AddOrUpdateRoute (
    uint16_t nwkDst,
    uint16_t nextHop,
    bool immediate,
    uint8_t numHop,
    double pathlossDb,
	    uint32_t linkCostToNextHop,
	    uint32_t advertisedCost,
	    uint16_t learnedFrom,
	    uint8_t capability = 0,
	    const std::vector<uint16_t> &path = {})
  {
    if (nwkDst == m_nodeId ||
        nwkDst == CSR_BROADCAST_ID)
      {
        return false;
      }

    SelectedRouteState selectedBefore =
      CaptureSelectedRouteState (
        nwkDst);

    uint32_t totalCost =
      linkCostToNextHop +
      advertisedCost;

    if (totalCost == 0)
      {
        totalCost = 1;
      }

    std::vector<uint16_t>
      normalizedPath = path;

    if (normalizedPath.empty ())
      {
        normalizedPath.push_back (
          nextHop);
      }

    if (normalizedPath.size () >
        CSR_MAX_ROUTE_PATH_HOPS)
      {
        normalizedPath.resize (
          CSR_MAX_ROUTE_PATH_HOPS);
      }

    // A candidate is identified by destination and next hop.
    // Different next hops for the same destination are retained.
    for (auto &route : m_routes)
      {
        if (route.nwkDst != nwkDst ||
            route.nextHop != nextHop)
          {
            continue;
          }

        uint32_t oldCost =
          route.cost;

        bool wasValid =
          route.valid;

        route.capability =
          capability;

        route.immediate =
          immediate;

        route.pathlossDb =
          pathlossDb;

        route.numHop =
          numHop;

        route.linkCostToNextHop =
          linkCostToNextHop;

        route.advertisedCost =
          advertisedCost;

        route.cost =
          totalCost;

        route.learnedFrom =
          learnedFrom;

        route.path = normalizedPath;

        route.lastUpdated =
          Simulator::Now ();

        route.valid = true;

        const RouteEntry *best =
          FindBestRoute (nwkDst);


        std::cout << "[NWK " << m_nodeId
                  << "] Updated route candidate"
                  << " dst=" << nwkDst
                  << " nextHop=" << nextHop
                  << " hops="
                  << unsigned (numHop)
                  << " oldCost=" << oldCost
                  << " newCost=" << totalCost
                  << " valid="
                  << (wasValid ? 1 : 0)
                  << "->1"
                  << " selected="
                  << (best == &route ? 1 : 0)
                  << std::endl;

        SelectedRouteState selectedAfter =
          CaptureSelectedRouteState (
            nwkDst);

        if (!SameSelectedRouteState (
              selectedBefore,
              selectedAfter))
          {
            MarkSelectedRouteChanged (
              nwkDst,
              "candidate update");
          }
        return true;
      }

    RouteEntry route;

    route.nwkDst =
      nwkDst;

    route.capability =
      capability;

    route.immediate =
      immediate;

    route.nextHop =
      nextHop;

    route.pathlossDb =
      pathlossDb;

    route.numHop =
      numHop;

    route.linkCostToNextHop =
      linkCostToNextHop;

    route.advertisedCost =
      advertisedCost;

    route.cost =
      totalCost;

    route.learnedFrom =
      learnedFrom;

    route.path = normalizedPath;

    route.energyLevel = 100;

    route.lastUpdated =
      Simulator::Now ();

    route.valid = true;

    m_routes.push_back (route);

    RouteEntry *stored =
      &m_routes.back ();

    const RouteEntry *best =
      FindBestRoute (nwkDst);

    std::cout << "[NWK " << m_nodeId
              << "] Added route candidate"
              << " dst=" << nwkDst
              << " nextHop=" << nextHop
              << " hops="
              << unsigned (numHop)
              << " cost=" << totalCost
              << " linkCost="
              << linkCostToNextHop
              << " advCost="
              << advertisedCost
              << " learnedFrom="
              << learnedFrom
              << " selected="
              << (best == stored ? 1 : 0)
              << std::endl;

    SelectedRouteState selectedAfter =
      CaptureSelectedRouteState (
        nwkDst);

    if (!SameSelectedRouteState (
          selectedBefore,
          selectedAfter))
      {
        MarkSelectedRouteChanged (
          nwkDst,
          "candidate update");
      }

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

  void
  SendRoutingRequest (
    uint16_t neighbor)
  {
    if (m_hop == nullptr)
      {
        return;
      }

    auto neighborIt =
      m_nwkNeighbors.find (neighbor);

    if (neighborIt ==
        m_nwkNeighbors.end ())
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingRequest rejected"
                  << " unknownNeighbor="
                  << neighbor
                  << std::endl;
        return;
      }

    NwkNeighborEntry &entry =
      neighborIt->second;

    if (entry.stale)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingRequest rejected"
                  << " staleNeighbor="
                  << neighbor
                  << std::endl;
        return;
      }

    if (entry.routingRequestPending)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingRequest already pending"
                  << " neighbor=" << neighbor
                  << " requestSequence="
                  << entry.routingRequestSequence
                  << std::endl;
        return;
      }

    entry.routingRequestPending = true;
    entry.routingRequestRetryCount = 0;

    // Remove any residue from an older incomplete snapshot.
    entry.routingSnapshotActive = false;
    entry.routingSnapshotInfoSequence = 0;
    entry.routingSnapshotSeenDestinations.clear ();

    SendRoutingRequestAttempt (
      neighbor,
      false);
  }

  void
  SendReliableRoutingDelete (
    uint16_t neighbor,
    uint16_t destination)
  {
    if (m_hop == nullptr)
      {
        return;
      }

    if (destination == CSR_BROADCAST_ID ||
        destination == m_nodeId)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingDelete rejected"
                  << " invalidDestination="
                  << destination
                  << std::endl;
        return;
      }

    auto neighborIt =
      m_nwkNeighbors.find (neighbor);

    if (neighborIt == m_nwkNeighbors.end ())
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingDelete rejected"
                  << " unknownNeighbor="
                  << neighbor
                  << std::endl;
        return;
      }

    if (neighborIt->second.stale)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingDelete rejected"
                  << " staleNeighbor="
                  << neighbor
                  << std::endl;
        return;
      }

    uint32_t sequence =
      NextRoutingSequence ();

    Ptr<Packet> payload =
      BuildRoutingMarkerPayload (
        CsrRoutingOperation::Delete,
        sequence,
        destination);

    std::cout << "[NWK " << m_nodeId
              << "] Sending reliable RoutingDelete"
              << " neighbor=" << neighbor
              << " destination="
              << destination
              << " routingSequence="
              << sequence
              << std::endl;

    m_hop->SendRoutingControl (
      neighbor,
      payload);
  }

  void DumpBestRoute (
    uint16_t destination) const;

  void
  SetAutomaticRoutePropagationEnabled (
    bool enable)
  {
    m_automaticRoutePropagationEnabled =
      enable;

    std::cout << "[NWK " << m_nodeId
              << "] automatic_route_propagation="
              << (enable ? "enabled" : "disabled")
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
    uint16_t learnedFrom {
      CSR_BROADCAST_ID
    }; // node that taught us this route

    std::vector<uint16_t> path;
  };

  struct SelectedRouteState
  {
    bool available {false};

    uint16_t nextHop {
      CSR_BROADCAST_ID
    };

    uint32_t cost {0};
    uint8_t numHop {0};
    bool immediate {false};
    uint8_t capability {0};

    std::vector<uint16_t> path;
  };

  const RouteEntry*
  FindBestRoute (
    uint16_t destination) const;

  SelectedRouteState
  CaptureSelectedRouteState (
    uint16_t destination) const;

  static bool
  SameSelectedRouteState (
    const SelectedRouteState &first,
    const SelectedRouteState &second);

  void
  MarkSelectedRouteChanged (
    uint16_t destination,
    const char *reason);

  void
  ReportPendingSelectedRouteChanges ();

  void
  TrySendAutomaticRouteUpdates ();

  struct ReverseRouteEntry
  {
    uint16_t netSrc {CSR_BROADCAST_ID};
    uint16_t reverseHop {CSR_BROADCAST_ID};
    Time lastUpdated {Seconds (0.0)};
    bool valid {false};
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
    bool wasActiveBeforeLastHello {false};

    // OPNET BrT_Neighbor_Entry::num_failures
    uint32_t numFailures {0};
    bool stale {false};

    bool discoverySequenceValid {false};
    uint32_t discoverySequence {0};

    bool discoveryVerified {false};

    CsrNodeType nodeType {CsrNodeType::Ordinary};

	    bool routingSequenceValid {false};
	    uint32_t routingSequence {0};

	    bool routingUpdateSectionStateValid {false};
	    uint32_t routingUpdateSectionSequence {0};
	    uint8_t routingUpdateLastSection {0};
	    uint8_t routingUpdateTotalSections {1};

	    bool routingRequestPending {false};
    uint32_t routingRequestSequence {0};

    uint32_t routingRequestRetryCount {0};
    EventId routingRequestTimeoutEvent;

    bool routingSnapshotActive {false};

    uint32_t routingSnapshotInfoSequence {0};

    std::set<uint16_t>
      routingSnapshotSeenDestinations;

    // Complete routing UPDATE sections are buffered here
    // until the final section arrives. This mirrors the
    // legacy behavior of reconstructing the full routing
    // message before modifying the live route table.
    std::vector<CsrHelloHeader>
      routingSnapshotBufferedUpdates;

    bool routingInfoValid {false};
    uint32_t routingInfoSequence {0};

    uint16_t remoteMinSpeedKbps {0};
    uint16_t remoteMaxSpeedKbps {0};

    int16_t remoteMinPowerDbmX10 {0};
    int16_t remoteMaxPowerDbmX10 {0};

    int16_t remoteLinkMarginDbX10 {0};
    int16_t remoteLowPowerDbmX10 {0};

    int16_t remoteTempLowCx10 {0};
    int16_t remoteTempHighCx10 {0};

    bool negotiatedLinkProfileValid {
      false
    };

    bool negotiatedSpeedCompatible {
      false
    };

    bool negotiatedPowerCompatible {
      false
    };

    uint16_t negotiatedMinSpeedKbps {0};
    uint16_t negotiatedMaxSpeedKbps {0};

    int16_t negotiatedMinPowerDbmX10 {0};
    int16_t negotiatedMaxPowerDbmX10 {0};

    int16_t negotiatedLinkMarginDbX10 {0};
    int16_t negotiatedLowPowerDbmX10 {0};
      };

  Ptr<Packet> BuildRoutingRequestPayload (
    uint32_t routingSequence);

  void SendRoutingRequestAttempt (
    uint16_t neighbor,
    bool retry);

  void RoutingRequestTimeout (
    uint16_t neighbor,
    uint32_t expectedSequence);

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

  bool
  LookupNextHop (
    uint16_t nwkDst,
    uint16_t &nextHopOut)
  {
    const RouteEntry *best =
      FindBestRoute (
        nwkDst);

    // --------------------------------------------------
    // Legacy priority 1:
    // A routable/capable destination uses the selected
    // forward route.
    // --------------------------------------------------
    if (best != nullptr &&
        best->capability != 0)
      {
        nextHopOut =
          best->nextHop;

        return true;
      }

    // --------------------------------------------------
    // Legacy priority 2:
    // For an Ordinary/non-capable destination, prefer
    // a valid reverse route learned from its traffic.
    // The reverse hop must still be an active neighbor.
    // --------------------------------------------------
    auto reverseIt =
      m_reverseRoutes.find (
        nwkDst);

    if (reverseIt !=
        m_reverseRoutes.end () &&
        reverseIt->second.valid)
      {
        uint16_t reverseHop =
          reverseIt->second.reverseHop;

        auto neighborIt =
          m_nwkNeighbors.find (
            reverseHop);

        bool reverseNeighborActive =
          neighborIt !=
            m_nwkNeighbors.end () &&
          neighborIt->second.lastHeardSec >=
            0.0 &&
          !neighborIt->second.stale;

        if (reverseNeighborActive)
          {
            nextHopOut =
              reverseHop;

            std::cout << "[NWK " << m_nodeId
                      << "] Using reverse route"
                      << " dst=" << nwkDst
                      << " reverseHop="
                      << reverseHop
                      << " forwardAvailable="
                      << (best != nullptr ? 1 : 0)
                      << " forwardCapability="
                      << (best != nullptr
                            ? unsigned (
                                best->capability)
                            : 0)
                      << std::endl;

            return true;
          }
      }

    // --------------------------------------------------
    // Legacy priority 3:
    // With no usable reverse path, fall back to the
    // ordinary forward/direct route if one exists.
    // --------------------------------------------------
    if (best != nullptr)
      {
        nextHopOut =
          best->nextHop;

        return true;
      }

    return false;
  }

  uint32_t
  ComputeLinkCost (double s0PowerDbm,
                   double pathlossDb,
                   uint32_t numFailures,
                   int *speedOut = nullptr,
                   double *txPowerOut = nullptr,
                   int *estDistanceOut = nullptr,
                   double *speedMarginOut = nullptr,
                   double *totalMarginOut = nullptr,
                   const NwkNeighborEntry *neighborProfile =
                    nullptr) const
  {
    // Port of legacy OPNET br_hop.link_calc(), simplified for NS-3.
    // TX0_power is the power needed to meet link margin at 8 kbps.
    double tx0PowerDbm = s0PowerDbm + pathlossDb;

    int effectiveMinSpeedKbps =
      m_minCfgSpeedKbps;

    int effectiveMaxSpeedKbps =
      m_maxCfgSpeedKbps;

    double effectiveMinTxPowerDbm =
      m_minTxPowerDbm;

    double effectiveMaxTxPowerDbm =
      m_maxTxPowerDbm;

    double effectiveTxAmpBreakpointDbm =
      m_txAmpBreakpointDbm;

    bool usingNegotiatedProfile =
      neighborProfile != nullptr &&
      neighborProfile->
        negotiatedLinkProfileValid;

    if (usingNegotiatedProfile)
      {
        effectiveMinSpeedKbps =
          static_cast<int> (
            neighborProfile->
              negotiatedMinSpeedKbps);

        effectiveMaxSpeedKbps =
          static_cast<int> (
            neighborProfile->
              negotiatedMaxSpeedKbps);

        effectiveMinTxPowerDbm =
          static_cast<double> (
            neighborProfile->
              negotiatedMinPowerDbmX10) /
          10.0;

        effectiveMaxTxPowerDbm =
          static_cast<double> (
            neighborProfile->
              negotiatedMaxPowerDbmX10) /
          10.0;

        effectiveTxAmpBreakpointDbm =
          static_cast<double> (
            neighborProfile->
              negotiatedLowPowerDbmX10) /
          10.0;
      }

    // Txm_power is power required at configured minimum speed.
    double txmPowerDbm = tx0PowerDbm;
    switch (effectiveMinSpeedKbps)
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

	    int speed = effectiveMinSpeedKbps;

    // Highest speed that can meet link margin at max configured TX power.
    double marginAt8K =
      effectiveMaxTxPowerDbm -
      tx0PowerDbm;

    if      (marginAt8K >= 23.0) speed = 1000;
    else if (marginAt8K >= 20.0) speed = 500;
    else if (marginAt8K >= 12.0) speed = 128;
    else if (marginAt8K >= 9.0)  speed = 64;
    else if (marginAt8K >= 6.0)  speed = 32;
    else if (marginAt8K >= 3.0)  speed = 16;
    else if (marginAt8K > 0.0)   speed = 8;
	    else                         speed = effectiveMinSpeedKbps;

	    // Limit speed to configured min/max.
	    if (speed < effectiveMinSpeedKbps)
	      {
	        speed = effectiveMinSpeedKbps;
	      }
	    if (speed > effectiveMaxSpeedKbps)
	      {
	        speed = effectiveMaxSpeedKbps;
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

	    double totalMargin =
	      effectiveMaxTxPowerDbm -
	      txmPowerDbm;

	    double speedMargin =
	      effectiveMaxTxPowerDbm -
	      txPowerDbm;

	    // Limit actual power to configured min/max.
	    if (txPowerDbm < effectiveMinTxPowerDbm)
	      {
	        txPowerDbm = effectiveMinTxPowerDbm;
	      }
	    if (txPowerDbm > effectiveMaxTxPowerDbm)
	      {
	        txPowerDbm = effectiveMaxTxPowerDbm;
	      }

    // Scaled distance from transmit power using r^4 model.
    int estDistance;

    if (txPowerDbm <=
        effectiveTxAmpBreakpointDbm)
      {
        estDistance = 75;
      }
    else
      {
        estDistance =
          static_cast<int> (
            std::floor (
              std::pow (
                10.0,
                (txPowerDbm -
                effectiveTxAmpBreakpointDbm) /
                  40.0) *
              100.0));
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

  static int
  CompareRoutingSequence (uint32_t first, uint32_t second)
  {
    uint32_t diff = first - second;

    if (diff == 0)
      {
        return 0;
      }

    if (diff > 0x80000000u)
      {
        return -1;
      }

    return 1;
  }

  void
  UpdateNegotiatedLinkProfile (
    NwkNeighborEntry &neighbor);

 /*uint32_t
  ComputeNeighborHopCost (
    NwkNeighborEntry &neighbor,
    double pathlossDb,
    const char *reason);*/

public:
  void SetNodeType (CsrNodeType type)
  {
    m_nodeType = type;

    if (m_nodeType != CsrNodeType::Gateway &&
        m_gatewayStartupDiscoveryEvent.IsPending ())
      {
        Simulator::Cancel (m_gatewayStartupDiscoveryEvent);

        std::cout << "[NWK " << m_nodeId
                  << "] Canceled pending Gateway startup discovery"
                  << " after role change"
                  << std::endl;
      }

    std::cout << "[NWK " << m_nodeId
              << "] nodeType=" << NodeTypeName (m_nodeType)
              << std::endl;
  }

  CsrNodeType GetNodeType () const
  {
    return m_nodeType;
  }

  void SetTransitForwardingEnabled (bool enable)
  {
    m_transitForwardingEnabled = enable;

    std::cout << "[NWK " << m_nodeId
              << "] transitForwarding="
              << (enable ? "enabled" : "disabled")
              << std::endl;
  }

  void ConfigureAsLeaf ()
  {
    SetNodeType (CsrNodeType::Ordinary);
    SetTransitForwardingEnabled (false);
  }

  void
  SetTemperatureLimitsCx10 (
    int16_t lowCx10,
    int16_t highCx10)
  {
    m_tempLowCx10 =
      lowCx10;

    m_tempHighCx10 =
      highCx10;

    std::cout << "[NWK " << m_nodeId
              << "] temperature limits"
              << " lowCx10="
              << m_tempLowCx10
              << " highCx10="
              << m_tempHighCx10
              << std::endl;
  }

  void
  NoteRoutingControlSuccess (
    uint16_t neighbor,
    uint32_t routingSequence,
    CsrRoutingOperation operation,
    uint8_t routingSection,
    uint8_t routingTotalSections)
  {
    std::cout << "[NWK " << m_nodeId
              << "] Reliable RoutingControl ACKed"
              << " neighbor=" << neighbor
	              << " routingSequence="
	              << routingSequence
	              << " operation="
	              << RoutingOperationName (operation)
	              << " section="
	              << unsigned (routingSection)
	              << "/"
	              << unsigned (routingTotalSections)
	              << std::endl;

	    auto snapshotIt =
      m_outboundRoutingSnapshots.find (
        neighbor);

    if (snapshotIt ==
        m_outboundRoutingSnapshots.end ())
      {
        return;
      }

    OutboundRoutingSnapshot &snapshot =
      snapshotIt->second;

    if (!snapshot.active)
      {
        return;
      }

    if (operation ==
          CsrRoutingOperation::Info &&
        routingSequence ==
          snapshot.infoSequence)
      {
        snapshot.currentUpdateSection = 0;

        Ptr<Packet> updatePayload =
          BuildRoutingUpdatePayload (
            snapshot.updateSequence,
            snapshot.currentUpdateSection,
            snapshot.totalUpdateSections);

        std::cout << "[NWK " << m_nodeId
                  << "] RoutingSnapshot INFO ACKed;"
                  << " sending UPDATE section="
                  << unsigned (
                      snapshot
                        .currentUpdateSection)
                  << "/"
                  << unsigned (
                      snapshot
                        .totalUpdateSections)
                  << " neighbor=" << neighbor
                  << " routingSequence="
                  << snapshot.updateSequence
                  << std::endl;

        m_hop->SendRoutingControl (
          neighbor,
          updatePayload);

        ArmRoutingSnapshotWatchdog (
          neighbor,
          "awaiting-update-section-ack");

        return;
      }

    if (operation ==
          CsrRoutingOperation::Update &&
        routingSequence ==
          snapshot.updateSequence)
      {
        if (routingSection !=
            snapshot.currentUpdateSection)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Ignoring unexpected snapshot"
                      << " Update completion"
                      << " neighbor=" << neighbor
                      << " completedSection="
                      << unsigned (routingSection)
                      << " expectedSection="
                      << unsigned (
                          snapshot
                            .currentUpdateSection)
                      << std::endl;

            return;
          }

        uint8_t nextSection =
          static_cast<uint8_t> (
            routingSection + 1);

        if (nextSection <
            snapshot.totalUpdateSections)
          {
            snapshot.currentUpdateSection =
              nextSection;

            Ptr<Packet> nextPayload =
              BuildRoutingUpdatePayload (
                snapshot.updateSequence,
                snapshot.currentUpdateSection,
                snapshot.totalUpdateSections);

            std::cout << "[NWK " << m_nodeId
                      << "] RoutingSnapshot UPDATE ACKed;"
                      << " sending next section="
                      << unsigned (
                          snapshot
                            .currentUpdateSection)
                      << "/"
                      << unsigned (
                          snapshot
                            .totalUpdateSections)
                      << " neighbor=" << neighbor
                      << " routingSequence="
                      << snapshot.updateSequence
                      << std::endl;

            m_hop->SendRoutingControl (
              neighbor,
              nextPayload);

            ArmRoutingSnapshotWatchdog (
                neighbor,
                "awaiting-update-section-ack");

            return;
          }

        Ptr<Packet> flushPayload =
          BuildRoutingMarkerPayload (
            CsrRoutingOperation::Flush,
            snapshot.flushSequence);

        std::cout << "[NWK " << m_nodeId
                  << "] RoutingSnapshot final UPDATE ACKed;"
                  << " sending FLUSH"
                  << " neighbor=" << neighbor
                  << " routingSequence="
                  << snapshot.flushSequence
                  << std::endl;

        m_hop->SendRoutingControl (
          neighbor,
          flushPayload);

        ArmRoutingSnapshotWatchdog (
          neighbor,
          "awaiting-flush-ack");

        return;
      }

    if (operation ==
          CsrRoutingOperation::Flush &&
        routingSequence ==
          snapshot.flushSequence)
      {
        if (snapshot.watchdogEvent.IsPending ())
          {
            Simulator::Cancel (
              snapshot.watchdogEvent);
          }

        // Prevent an already-scheduled older callback
        // from affecting a later snapshot.
        snapshot.watchdogGeneration++;

        snapshot.watchdogPhase =
          "completed";

        snapshot.active = false;

        std::cout << "[NWK " << m_nodeId
                  << "] Reliable RoutingSnapshot completed"
                  << " neighbor=" << neighbor
                  << " finalSequence="
                  << routingSequence
                  << std::endl;

        // Resume any automatic route propagation that
        // was blocked while this snapshot was active.
        if (!m_pendingAutomaticRouteChanges.empty () &&
            !m_automaticRouteUpdateEvent.IsPending ())
          {
            m_automaticRouteUpdateEvent =
              Simulator::ScheduleNow (
                &CsrNetLayer::
                  TrySendAutomaticRouteUpdates,
                this);
          }
      }
  }

  void
  NoteRoutingControlFailure (
    uint16_t neighbor,
    uint32_t routingSequence,
    CsrRoutingOperation operation,
    uint8_t routingSection,
    uint8_t routingTotalSections)
  {
    std::cout << "[NWK " << m_nodeId
              << "] Reliable RoutingControl failure"
              << " neighbor="
              << neighbor
              << " routingSequence="
              << routingSequence
              << " operation="
              << RoutingOperationName (
                  operation)
              << " section="
              << unsigned (
                  routingSection)
              << "/"
              << unsigned (
                  routingTotalSections)
              << std::endl;

    // For now, leave recovery to the existing
    // snapshot watchdog. The next parity step can
    // use this callback to retry the exact failed
    // INFO/UPDATE/FLUSH immediately.
  }

  const char*
  RoutingOperationName (
    CsrRoutingOperation operation) const
  {
    switch (operation)
      {
      case CsrRoutingOperation::Flush:
        return "Flush";

      case CsrRoutingOperation::Delete:
        return "Delete";

      case CsrRoutingOperation::Update:
        return "Update";

      case CsrRoutingOperation::Request:
        return "Request";

      case CsrRoutingOperation::Info:
        return "Info";

      case CsrRoutingOperation::None:
      default:
        return "None";
      }
  }

  void
  SendReliableRoutingUpdate (
    uint16_t neighbor)
  {
    if (m_hop == nullptr)
      {
        return;
      }

    auto neighborIt =
      m_nwkNeighbors.find (neighbor);

    if (neighborIt ==
        m_nwkNeighbors.end ())
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Reliable RoutingUpdate rejected"
                  << " unknownNeighbor="
                  << neighbor
                  << std::endl;
        return;
      }

    if (neighborIt->second.stale)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Reliable RoutingUpdate rejected"
                  << " staleNeighbor="
                  << neighbor
                  << std::endl;
        return;
      }

    ++m_routingSequence;

    if (m_routingSequence == 0)
      {
        ++m_routingSequence;
      }

    Ptr<Packet> payload =
      BuildRoutingUpdatePayload (
        m_routingSequence);

    std::cout << "[NWK " << m_nodeId
              << "] Sending reliable RoutingUpdate"
              << " neighbor=" << neighbor
              << " routingSequence="
              << m_routingSequence
              << std::endl;

    m_hop->SendRoutingControl (
      neighbor,
      payload);
  }

  void
  SetRoutingSnapshotResponseEnabled (
    bool enable)
  {
    m_routingSnapshotResponseEnabled =
      enable;

    std::cout << "[NWK " << m_nodeId
              << "] routing_snapshot_response_enabled="
              << (enable ? "true" : "false")
              << std::endl;
  }

  void
  ArmRoutingSnapshotWatchdog (
    uint16_t neighbor,
    const char *phase);

  void
  RoutingSnapshotWatchdogExpired (
    uint16_t neighbor,
    uint32_t expectedGeneration);

private:
  uint16_t                              m_nodeId;
  Ptr<CsrHopLayer>                      m_hop;
  Callback<void, Ptr<Packet>, uint16_t> m_rxFromNetCb;

  std::deque<NwkQueueEntry>             m_nwkQueue;
  EventId                               m_checkNwkQueueEvent;
  std::vector<RouteEntry>               m_routes;
  std::map<uint16_t, ReverseRouteEntry> m_reverseRoutes;
  std::map<uint16_t, NwkNeighborEntry>  m_nwkNeighbors;
  std::map<std::pair<uint16_t,uint16_t>, NsdpEntry> m_nsdp;
  std::set<uint16_t>
    m_pendingSelectedRouteChanges;

  // Legacy routesFindBestRoute() prefers the
  // currently selected neighbor when cost and
  // hop count are exactly tied.
  std::map<uint16_t, uint16_t>
    m_selectedRoutePreferredNextHop;

  EventId m_selectedRouteChangeEvent;

  Time m_selectedRouteChangeDelay {
    MilliSeconds (25)
  };

  bool    m_discoveryActive { false };

  bool m_discoveryResponseEnabled {true};

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
  // Legacy ROUTING_INFO temperature limits.
  // Stored as degrees C x10, matching routes.c.
  int16_t m_tempLowCx10  {0};
  int16_t m_tempHighCx10 {0};

  void DiscoveryStart ();
  void DiscoveryStop ();
  void DiscoveryCooldownOver ();

  void SendHelloBroadcast (
    CsrArlRouteMsgType type = CsrArlRouteMsgType::Discover,
    CsrNeighborCheckType checkType = CsrNeighborCheckType::None,
    CsrDiscoverType discoverType = CsrDiscoverType::None,
    uint32_t discoverySequence = 0,
    uint32_t routingSequence = 0);

  void EnsureDiscoveryForTx ();

  std::set<uint16_t>
  ProcessRoutesPayload (
    const CsrHelloHeader &hh,
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

  void UpdateReverseRoute (uint16_t netSrc, uint16_t hopSrc);
  bool RemoveReverseRouteFromReporter (uint16_t netSrc, uint16_t reporter);

  const char* ArlRouteMsgTypeName (CsrArlRouteMsgType t) const;

  bool ShouldAdvertiseRoute (const RouteEntry &re) const;

  void TryDrainQueueAfterDiscovery ();

  void ScheduleDiscoveryHello ();
  void DiscoveryHelloTick ();
  void VerifyUnresponsiveDiscoveryNeighbors ();
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

  Time m_routingSnapshotWatchdogTimeout {
    Seconds (20.0)
  };

  uint16_t m_neighborCheckSeq {0};
  uint32_t m_discoverySequence {0};

  CsrNodeType m_nodeType {CsrNodeType::Routable};

  // Independent from legacy capability. This provides a guaranteed
  // non-relaying node for UAV/mobile-leaf experiments.
  bool m_transitForwardingEnabled {true};

  uint16_t m_gatewayNodeId {CSR_BROADCAST_ID};
  EventId m_gatewayStartupDiscoveryEvent;
  Time m_gatewayStartupDiscoveryDuration {Seconds (30.0)};

  void GatewayStartupDiscoveryFire ();

  const char*
  NodeTypeName (CsrNodeType type) const
  {
    switch (type)
      {
      case CsrNodeType::Ordinary:
        return "Ordinary";

      case CsrNodeType::Routable:
        return "Routable";

      case CsrNodeType::Gateway:
        return "Gateway";

      default:
        return "Unknown";
      }
  }

  Ptr<Packet> BuildTargetedRoutingUpdatePayload (
    uint16_t destination,
    uint32_t routingSequence);

  Ptr<Packet> BuildRoutingUpdatePayload (
    uint32_t routingSequence,
    uint8_t routingSection = 0,
    uint8_t routingTotalSections = 1);

  uint32_t
  CountAdvertisableSelectedRoutes () const;

  uint16_t m_routingControlHeaderSeq {0};

  Ptr<Packet> BuildRoutingMarkerPayload (
    CsrRoutingOperation operation,
    uint32_t routingSequence,
    uint16_t routingTarget =
      CSR_BROADCAST_ID);

  struct OutboundRoutingSnapshot
  {
    bool active {false};

    uint32_t infoSequence {0};

    // All Update sections share this sequence,
    // matching the legacy routing message.
    uint32_t updateSequence {0};

    uint32_t flushSequence {0};

    uint8_t totalUpdateSections {1};
    uint8_t currentUpdateSection {0};

    EventId watchdogEvent;

    uint32_t watchdogGeneration {0};

    const char *watchdogPhase {
      "inactive"
    };
  };

  std::map<uint16_t, OutboundRoutingSnapshot>
    m_outboundRoutingSnapshots;

  uint32_t
  NextRoutingSequence ()
  {
    ++m_routingSequence;

    if (m_routingSequence == 0)
      {
        ++m_routingSequence;
      }

    return m_routingSequence;
  }

  Time m_routingRequestTimeout {
    Seconds (8.0)
  };

  uint32_t m_maxRoutingRequestRetries {2};

  bool m_routingSnapshotResponseEnabled {true};

  bool m_automaticRoutePropagationEnabled {
    false
  };

  // Changed destinations waiting to be advertised,
  // grouped by receiving neighbor.
  std::map<uint16_t, std::set<uint16_t>>
    m_pendingAutomaticRouteChanges;

  EventId m_automaticRouteUpdateEvent;

  Time m_automaticRouteUpdateSpacing {
    MilliSeconds (20)
  };

  Time m_automaticRouteUpdateRetryDelay {
    MilliSeconds (100)
};
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
CsrNetLayer::ScheduleGatewayStartupDiscovery (
  Time delay,
  Time duration)
{
  if (m_nodeType != CsrNodeType::Gateway)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Gateway startup discovery not scheduled"
                << " role=" << NodeTypeName (m_nodeType)
                << std::endl;
      return;
    }

  if (m_gatewayStartupDiscoveryEvent.IsPending ())
    {
      Simulator::Cancel (m_gatewayStartupDiscoveryEvent);
    }

  m_gatewayStartupDiscoveryDuration = duration;

  std::cout << "[NWK " << m_nodeId
            << "] Scheduling Gateway startup discovery"
            << " delay=" << delay.GetSeconds ()
            << "s duration=" << duration.GetSeconds ()
            << "s"
            << std::endl;

  m_gatewayStartupDiscoveryEvent =
    Simulator::Schedule (
      delay,
      &CsrNetLayer::GatewayStartupDiscoveryFire,
      this);
}

void
CsrNetLayer::GatewayStartupDiscoveryFire ()
{
  if (m_nodeType != CsrNodeType::Gateway)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Gateway startup discovery canceled"
                << " because current role="
                << NodeTypeName (m_nodeType)
                << std::endl;
      return;
    }

  if (m_discState != DiscoveryState::IDLE)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Gateway startup discovery skipped"
                << " because discovery state is not IDLE"
                << std::endl;
      return;
    }

  std::cout << "[NWK " << m_nodeId
            << "] Gateway startup discovery firing"
            << " duration="
            << m_gatewayStartupDiscoveryDuration.GetSeconds ()
            << "s"
            << std::endl;

  StartDiscovery (
    Seconds (0.0),
    m_gatewayStartupDiscoveryDuration);
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

  CsrNodeType senderType = hh.GetNodeType ();

  if (senderType == CsrNodeType::Gateway)
    {
      if (m_gatewayNodeId != src)
        {
          std::cout << "[NWK " << m_nodeId
                    << "] Gateway learned node=" << src
                    << std::endl;
        }

      m_gatewayNodeId = src;
    }

  double now = Simulator::Now ().GetSeconds ();

    // ------------------------------------------------------------
    // 1) Update NWK neighbor table, similar to OPNET proc_hello()
    // ------------------------------------------------------------
  auto &ne = m_nwkNeighbors[src];
  bool isNew = (ne.lastHeardSec < 0.0);
  bool wasStale = ne.stale;
  ne.nodeType = senderType;

  ne.wasActiveBeforeLastHello =
    !isNew && !wasStale;

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

  // ----------------------------------------------------------
  // Local direction:
  // This node transmits to the neighbor.
  //
  // The neighbor's advertised S0 value represents the
  // receive threshold that our transmission must satisfy.
  // ----------------------------------------------------------
  uint32_t localDirectionalCost =
    ComputeLinkCost (
      s0PowerDbm,
      pathlossDb,
      ne.numFailures,
      &chosenSpeed,
      &chosenTxPower,
      &estDistance,
      &speedMargin,
      &totalMargin,
      nullptr);

  uint32_t linkCost =
    localDirectionalCost;

  uint32_t remoteDirectionalCost = 0;

  bool usingBidirectionalCost = false;

  // Preserve the local-direction results before combining
  // them with the reverse direction.
  int localSpeed =
    chosenSpeed;

  double localTxPower =
    chosenTxPower;

  int localEstDistance =
    estDistance;

  double localSpeedMargin =
    speedMargin;

  double localTotalMargin =
    totalMargin;

  // The remote configuration must be internally valid.
  // This is different from requiring the local and remote
  // ranges to be identical.
  bool remoteLimitsValid =
    ne.routingInfoValid &&
    ne.remoteMinSpeedKbps > 0 &&
    ne.remoteMinSpeedKbps <=
      ne.remoteMaxSpeedKbps &&
    ne.remoteMinPowerDbmX10 <=
      ne.remoteMaxPowerDbmX10;

  int remoteSpeed = 0;
  double remoteTxPower = 0.0;
  int remoteEstDistance = 0;
  double remoteSpeedMargin = 0.0;
  double remoteTotalMargin = 0.0;

  if (remoteLimitsValid)
    {
      // ComputeLinkCost already accepts a profile through
      // its negotiated fields. Use a temporary profile to
      // represent the remote transmitter's actual limits.
      NwkNeighborEntry remoteTxProfile;

      remoteTxProfile
        .negotiatedLinkProfileValid =
          true;

      remoteTxProfile
        .negotiatedMinSpeedKbps =
          ne.remoteMinSpeedKbps;

      remoteTxProfile
        .negotiatedMaxSpeedKbps =
          ne.remoteMaxSpeedKbps;

      remoteTxProfile
        .negotiatedMinPowerDbmX10 =
          ne.remoteMinPowerDbmX10;

      remoteTxProfile
        .negotiatedMaxPowerDbmX10 =
          ne.remoteMaxPowerDbmX10;

      remoteTxProfile
        .negotiatedLowPowerDbmX10 =
          ne.remoteLowPowerDbmX10;

      // Reverse direction:
      // The neighbor transmits to this node, so use our
      // local receiver threshold and the remote TX limits.
      double localS0PowerDbm =
        m_rxS0BaseLevelDbm +
        m_linkMarginDb;

      remoteDirectionalCost =
        ComputeLinkCost (
          localS0PowerDbm,
          pathlossDb,
          ne.numFailures,
          &remoteSpeed,
          &remoteTxPower,
          &remoteEstDistance,
          &remoteSpeedMargin,
          &remoteTotalMargin,
          &remoteTxProfile);

      // Legacy behavior uses the weakest characteristic
      // of the two directions:
      //   lowest speed
      //   lowest margins
      //   highest required distance/energy estimate
      chosenSpeed =
        std::min (
          localSpeed,
          remoteSpeed);

      estDistance =
        std::max (
          localEstDistance,
          remoteEstDistance);

      speedMargin =
        std::min (
          localSpeedMargin,
          remoteSpeedMargin);

      totalMargin =
        std::min (
          localTotalMargin,
          remoteTotalMargin);

      // chosenTxPower describes this node's own
      // transmission requirement, so retain the local
      // directional value.
      chosenTxPower =
        localTxPower;

      double failureMarginDb =
        3.0 *
        static_cast<double> (
          ne.numFailures);

      double adjustedTotalMarginDb =
        totalMargin -
        failureMarginDb;

      double adjustedSpeedMarginDb =
        speedMargin -
        failureMarginDb;

      // Legacy routesComputeHopCost() adjusts the
      // required distance BEFORE dividing by speed.
      // Keep x100 units explicitly to preserve the
      // original integer cost semantics.
      uint64_t adjustedDistanceX100 =
        static_cast<uint64_t> (
          std::max (
            0,
            estDistance)) *
        100ULL;

      // Legacy:
      //   if total margin is negative,
      //       double required link distance;
      //   else if speed margin is at least 3 dB,
      //       halve required link distance.
      if (adjustedTotalMarginDb < 0.0)
        {
          adjustedDistanceX100 *= 2ULL;
        }
      else if (adjustedSpeedMarginDb >= 3.0)
        {
          adjustedDistanceX100 /= 2ULL;
        }

      if (chosenSpeed > 0)
        {
          uint64_t rawCost =
            adjustedDistanceX100 /
            static_cast<uint64_t> (
              chosenSpeed);

          linkCost =
            static_cast<uint32_t> (
              std::min<uint64_t> (
                rawCost,
                std::numeric_limits<uint32_t>::max ()));
        }
      else
        {
          linkCost = 1;
        }

      std::cout << "[NWK " << m_nodeId
                << "] legacy hop-cost adjustment"
                << " neighbor=" << src
                << " failures="
                << ne.numFailures
                << " adjustedTotalMarginDb="
                << adjustedTotalMarginDb
                << " adjustedSpeedMarginDb="
                << adjustedSpeedMarginDb
                << " adjustedDistanceX100="
                << adjustedDistanceX100
                << " speed="
                << chosenSpeed
                << " cost="
                << linkCost
                << std::endl;
    }

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
              << " costMode="
              << (usingBidirectionalCost
                    ? "bidirectional"
                    : "local-only")
              << " localDirectionalCost="
              << localDirectionalCost
              << " remoteDirectionalCost="
              << remoteDirectionalCost
              << " localSpeed="
              << localSpeed
              << " remoteSpeed="
              << remoteSpeed
              << " localDistance="
              << localEstDistance
              << " remoteDistance="
              << remoteEstDistance
              << " cost="
              << linkCost
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

std::set<uint16_t>
CsrNetLayer::ProcessRoutesPayload (const CsrHelloHeader &hh,
                                    uint16_t helloSrc,
                                    double pathlossDb,
                                    double snrDb,
                                    uint32_t linkCost)
{
    std::set<uint16_t>
      acceptedDestinations;

    uint8_t advCount = hh.GetAdvertisedRouteCount ();

    auto neighborIt = m_nwkNeighbors.find (helloSrc);

    if (neighborIt != m_nwkNeighbors.end () &&
        neighborIt->second.nodeType == CsrNodeType::Ordinary)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Ignoring transit route advertisements from "
                  << "Ordinary neighbor=" << helloSrc
                  << std::endl;

        return acceptedDestinations;
      }

    if (advCount == 0)
      {
        return acceptedDestinations;
      }

    std::cout << "[NWK " << m_nodeId
              << "] Processing Routes_PAYLOAD from "
              << helloSrc
              << " advCount=" << unsigned (advCount)
              << std::endl;

    for (uint8_t idx = 0; idx < advCount; ++idx)
      {
        auto ar = hh.GetAdvertisedRoute (idx);

        bool pathContainsLocalNode = false;

        for (uint16_t pathNode :
            ar.path)
          {
            if (pathNode == m_nodeId)
              {
                pathContainsLocalNode = true;
                break;
              }
          }

        if (pathContainsLocalNode)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Rejecting looped advertised route"
                      << " dst=" << ar.dst
                      << " from=" << helloSrc
                      << " reason=local-node-in-path"
                      << std::endl;

            continue;
          }

        if (ar.dst == CSR_BROADCAST_ID ||
            ar.dst == m_nodeId ||
            ar.dst == helloSrc)
          {
            continue;
          }

        uint8_t totalHops = static_cast<uint8_t> (ar.hops + 1);
        uint32_t totalCost = linkCost + ar.cost;

        std::vector<uint16_t> candidatePath;

        candidatePath.push_back (
          helloSrc);

        for (uint16_t pathNode :
            ar.path)
          {
            candidatePath.push_back (
              pathNode);

            if (candidatePath.size () >=
                CSR_MAX_ROUTE_PATH_HOPS)
              {
                break;
              }
          }

        bool accepted =
          AddOrUpdateRoute (
            ar.dst,
            helloSrc,
            false,
            totalHops,
            pathlossDb,
            linkCost,
            ar.cost,
            helloSrc,
            ar.capability,
            candidatePath);

        if (accepted)
          {
            acceptedDestinations.insert (
              ar.dst);

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
  return acceptedDestinations;
}

void
CsrNetLayer::ProcessRoutingUpdate (
  const CsrHelloHeader &hh,
  uint16_t helloSrc,
  double pathlossDb,
  double snrDb,
  uint32_t linkCost)
{
  auto neighborIt =
    m_nwkNeighbors.find (helloSrc);

  if (neighborIt == m_nwkNeighbors.end ())
    {
      std::cout << "[NWK " << m_nodeId
                << "] RoutingControl from unknown neighbor="
                << helloSrc
                << std::endl;
      return;
    }

  NwkNeighborEntry &neighbor =
    neighborIt->second;

  uint32_t incomingSequence =
    hh.GetRoutingSequence ();

  CsrRoutingOperation operation =
    hh.GetRoutingOperation ();

  if (operation == CsrRoutingOperation::None &&
      hh.GetAdvertisedRouteCount () > 0)
    {
      operation =
        CsrRoutingOperation::Update;
    }

  uint8_t incomingSection =
    hh.GetRoutingSection ();

  uint8_t incomingTotalSections =
    std::max<uint8_t> (
      1,
      hh.GetRoutingTotalSections ());

  // Reject duplicate or older routing-control messages.
  if (operation ==
        CsrRoutingOperation::Update &&
      incomingSection >=
        incomingTotalSections)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Rejecting malformed RoutingUpdate section"
                << " from=" << helloSrc
                << " section="
                << unsigned (incomingSection)
                << " totalSections="
                << unsigned (
                    incomingTotalSections)
                << std::endl;

      return;
    }

  int sequenceComparison = -1;

  if (neighbor.routingSequenceValid)
    {
      sequenceComparison =
        CompareRoutingSequence (
          neighbor.routingSequence,
          incomingSequence);
    }

  bool snapshotUpdateStart =
    neighbor.routingSequenceValid &&
    sequenceComparison == 0 &&
    operation ==
      CsrRoutingOperation::Update &&
    neighbor.routingSnapshotActive &&
    neighbor.routingSnapshotInfoSequence ==
      incomingSequence &&
    incomingSection == 0 &&
    !neighbor.routingUpdateSectionStateValid;

  bool continuationSection =
    neighbor.routingSequenceValid &&
    sequenceComparison == 0 &&
    operation ==
      CsrRoutingOperation::Update &&
    neighbor
      .routingUpdateSectionStateValid &&
    neighbor.routingUpdateSectionSequence ==
      incomingSequence &&
    incomingSection ==
      static_cast<uint8_t> (
        neighbor.routingUpdateLastSection + 1) &&
    incomingTotalSections ==
      neighbor.routingUpdateTotalSections;

  bool snapshotFlush =
    neighbor.routingSequenceValid &&
    sequenceComparison == 0 &&
    operation ==
      CsrRoutingOperation::Flush &&
    neighbor.routingSnapshotActive &&
    neighbor.routingSnapshotInfoSequence ==
      incomingSequence &&
    neighbor.routingUpdateSectionStateValid &&
    neighbor.routingUpdateSectionSequence ==
      incomingSequence &&
    static_cast<uint8_t> (
      neighbor.routingUpdateLastSection + 1) ==
        neighbor.routingUpdateTotalSections;

  bool validSameSequenceTransition =
    snapshotUpdateStart ||
    continuationSection ||
    snapshotFlush;

  if (neighbor.routingSequenceValid &&
      (sequenceComparison > 0 ||
      (sequenceComparison == 0 &&
        !validSameSequenceTransition)))
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring stale/duplicate RoutingControl"
                << " from=" << helloSrc
                << " incomingSequence="
                << incomingSequence
                << " lastSequence="
                << neighbor.routingSequence
                << " operation="
                << RoutingOperationName (
                    operation)
                << " section="
                << unsigned (
                    incomingSection)
                << std::endl;

      return;
    }

  bool newSequence =
    !neighbor.routingSequenceValid ||
    sequenceComparison < 0;

  if (newSequence &&
      operation ==
        CsrRoutingOperation::Update &&
      incomingSection != 0)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Rejecting RoutingUpdate"
                << " missing first section"
                << " from=" << helloSrc
                << " firstReceivedSection="
                << unsigned (incomingSection)
                << std::endl;

      return;
    }

  uint32_t previousSequence =
    neighbor.routingSequence;

  if (newSequence)
    {
      neighbor.routingSequence =
        incomingSequence;

      neighbor.routingSequenceValid = true;

      if (operation ==
          CsrRoutingOperation::Update)
        {
          neighbor
            .routingUpdateSectionStateValid =
              true;

          neighbor
            .routingUpdateSectionSequence =
              incomingSequence;

          neighbor
            .routingUpdateLastSection =
              incomingSection;

          neighbor
            .routingUpdateTotalSections =
              incomingTotalSections;
        }
      else
        {
          neighbor
            .routingUpdateSectionStateValid =
              false;
        }
    }
  else if (snapshotUpdateStart)
    {
      neighbor
        .routingUpdateSectionStateValid =
          true;

      neighbor
        .routingUpdateSectionSequence =
          incomingSequence;

      neighbor
        .routingUpdateLastSection =
          incomingSection;

      neighbor
        .routingUpdateTotalSections =
          incomingTotalSections;

      std::cout << "[NWK " << m_nodeId
                << "] RoutingSnapshot UPDATE sequence transition"
                << " from=" << helloSrc
                << " routingSequence="
                << incomingSequence
                << " section="
                << unsigned (
                    incomingSection)
                << "/"
                << unsigned (
                    incomingTotalSections)
                << std::endl;
    }
  else if (continuationSection)
    {
      neighbor.routingUpdateLastSection =
        incomingSection;
    }

	  switch (operation)
    {
    case CsrRoutingOperation::Request:
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Received RoutingRequest"
                  << " from=" << helloSrc
                  << " routingSequence="
                  << incomingSequence
                  << "; scheduling reliable INFO/UPDATE/FLUSH snapshot"
                  << std::endl;

        if (!m_routingSnapshotResponseEnabled)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Suppressing RoutingSnapshot response"
                      << " requester=" << helloSrc
                      << " requestSequence="
                      << incomingSequence
                      << " testMode=true"
                      << std::endl;

            return;
          }
        // Reply specifically to the requesting neighbor.
        Simulator::Schedule (
          MilliSeconds (20),
          &CsrNetLayer::
            StartReliableRoutingSnapshot,
          this,
          helloSrc);

        return;
      }

    case CsrRoutingOperation::Update:
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Accepted RoutingUpdate"
                  << " from=" << helloSrc
                  << " routingSequence="
                  << incomingSequence
                  << " previousSequence="
                  << previousSequence
                  << " advCount="
                  << unsigned (
                       hh.GetAdvertisedRouteCount ())
                  << " section="
                  << unsigned (incomingSection)
                  << "/"
                  << unsigned (incomingTotalSections)
                  << std::endl;

        // An ACK only proved that our Request arrived.
        // Receiving this Update proves that the requested
        // routing state came back.

        if (neighbor.routingSnapshotActive)
          {
            // Do not modify the live routing table yet.
            // Store this complete section until every section
            // in the snapshot has arrived.
            neighbor
              .routingSnapshotBufferedUpdates
              .push_back (hh);

            std::cout << "[NWK " << m_nodeId
                      << "] Buffered RoutingSnapshot UPDATE"
                      << " from=" << helloSrc
                      << " routingSequence="
                      << incomingSequence
                      << " section="
                      << unsigned (incomingSection)
                      << "/"
                      << unsigned (incomingTotalSections)
                      << " bufferedSections="
                      << neighbor
                          .routingSnapshotBufferedUpdates
                          .size ()
                      << std::endl;

            bool finalSection =
              incomingSection + 1 ==
                incomingTotalSections;

            if (!finalSection)
              {
                return;
              }

            // --------------------------------------------------
            // We now have section 0 ... section N-1.
            // Apply the complete reconstructed update during
            // this single simulator event.
            // --------------------------------------------------

            std::set<uint16_t>
              acceptedDestinations;

            uint32_t bufferedRouteCount = 0;

            for (const CsrHelloHeader &bufferedHeader :
                neighbor.routingSnapshotBufferedUpdates)
              {
                bufferedRouteCount +=
                  bufferedHeader
                    .GetAdvertisedRouteCount ();

                std::set<uint16_t>
                  sectionAccepted =
                    ProcessRoutesPayload (
                      bufferedHeader,
                      helloSrc,
                      pathlossDb,
                      snrDb,
                      linkCost);

                acceptedDestinations.insert (
                  sectionAccepted.begin (),
                  sectionAccepted.end ());
              }

            neighbor
              .routingSnapshotSeenDestinations
              .insert (
                acceptedDestinations.begin (),
                acceptedDestinations.end ());

            std::cout << "[NWK " << m_nodeId
                      << "] Applied complete RoutingSnapshot UPDATE"
                      << " from=" << helloSrc
                      << " routingSequence="
                      << incomingSequence
                      << " totalSections="
                      << unsigned (
                          incomingTotalSections)
                      << " bufferedRoutes="
                      << bufferedRouteCount
                      << " acceptedDestinations="
                      << acceptedDestinations.size ()
                      << std::endl;

            std::cout << "[NWK " << m_nodeId
                      << "] RoutingSnapshot received all UPDATE sections"
                      << " from=" << helloSrc
                      << " routingSequence="
                      << incomingSequence
                      << " totalSections="
                      << unsigned (
                          incomingTotalSections)
                      << std::endl;

            return;
          }

        // --------------------------------------------------
        // This is an unsolicited/single UPDATE rather than
        // part of an INFO -> UPDATE -> FLUSH snapshot.
        // Process it immediately.
        // --------------------------------------------------

        std::set<uint16_t>
          acceptedDestinations =
            ProcessRoutesPayload (
              hh,
              helloSrc,
              pathlossDb,
              snrDb,
              linkCost);

        std::cout << "[NWK " << m_nodeId
                  << "] Applied standalone RoutingUpdate"
                  << " from=" << helloSrc
                  << " acceptedDestinations="
                  << acceptedDestinations.size ()
                  << std::endl;

        return;
      }

    case CsrRoutingOperation::Info:
      {
        neighbor.routingSnapshotActive =
          true;

        neighbor.routingSnapshotInfoSequence =
          incomingSequence;

        neighbor
          .routingSnapshotSeenDestinations
          .clear ();

        neighbor
          .routingSnapshotBufferedUpdates
          .clear ();

        CsrHelloHeader::RoutingInfo info =
          hh.GetRoutingInfo ();

        neighbor.routingInfoValid = true;
        neighbor.routingInfoSequence =
          incomingSequence;

        neighbor.remoteMinSpeedKbps =
          info.minSpeedKbps;

        neighbor.remoteMaxSpeedKbps =
          info.maxSpeedKbps;

        neighbor.remoteMinPowerDbmX10 =
          info.minPowerDbmX10;

        neighbor.remoteMaxPowerDbmX10 =
          info.maxPowerDbmX10;

        neighbor.remoteLinkMarginDbX10 =
          info.linkMarginDbX10;

        neighbor.remoteLowPowerDbmX10 =
          info.lowPowerDbmX10;

        neighbor.remoteTempLowCx10 =
          info.tempLowCx10;

        neighbor.remoteTempHighCx10 =
          info.tempHighCx10;

        UpdateNegotiatedLinkProfile (
          neighbor);

        std::cout << "[NWK " << m_nodeId
                  << "] RoutingSnapshot INFO received"
                  << " from=" << helloSrc
                  << " routingSequence="
                  << incomingSequence
                  << std::endl;

        std::cout << "[NWK " << m_nodeId
          << "] Stored RoutingInfo"
          << " from=" << helloSrc
          << " sequence="
          << incomingSequence
          << " speedKbps="
          << info.minSpeedKbps
          << "-"
          << info.maxSpeedKbps
          << " powerDbmX10="
          << info.minPowerDbmX10
          << "-"
          << info.maxPowerDbmX10
          << " marginDbX10="
          << info.linkMarginDbX10
          << " lowPowerDbmX10="
          << info.lowPowerDbmX10
          << " tempCx10="
          << info.tempLowCx10
          << "-"
          << info.tempHighCx10
          << std::endl;

        return;
      }

    case CsrRoutingOperation::Delete:
      {
        uint16_t destination =
          hh.GetRoutingTarget ();

        if (destination ==
            CSR_BROADCAST_ID)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Ignoring malformed RoutingDelete"
                      << " from=" << helloSrc
                      << " with no destination"
                      << std::endl;

            return;
          }

        SelectedRouteState selectedBefore =
            CaptureSelectedRouteState (
               destination);

        bool matchingRouteFound = false;
        bool routeInvalidated = false;
        bool directRouteProtected = false;

        for (auto &route : m_routes)
          {
            if (route.nwkDst != destination)
              {
                continue;
              }

            // Hearing the packet itself proves that the direct
            // route to the reporting neighbor still exists.
            if (route.immediate)
              {
                if (destination == helloSrc)
                  {
                    directRouteProtected = true;
                  }

                continue;
              }

            // A neighbor may delete only the route state that
            // was previously learned from that same neighbor.
            if (route.learnedFrom != helloSrc)
              {
                continue;
              }

            matchingRouteFound = true;
            route.lastUpdated =
              Simulator::Now ();

            if (route.valid)
              {
                route.valid = false;
                routeInvalidated = true;

                std::cout << "[NWK " << m_nodeId
                          << "] RoutingDelete invalidated"
                          << " dst=" << destination
                          << " learnedFrom="
                          << helloSrc
                          << " routingSequence="
                          << incomingSequence
                          << std::endl;
              }
            else
              {
                std::cout << "[NWK " << m_nodeId
                          << "] RoutingDelete refreshed"
                          << " already-invalid route"
                          << " dst=" << destination
                          << " learnedFrom="
                          << helloSrc
                          << std::endl;
              }
          }

        if (directRouteProtected)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] RoutingDelete ignored for direct neighbor"
                      << " neighbor=" << helloSrc
                      << std::endl;
          }

        if (!matchingRouteFound &&
            !directRouteProtected)
          {
            // Legacy routes.c creates an invalid route entry
            // when DELETE arrives for a destination that was
            // not previously known from this neighbor.
            RouteEntry tombstone;

            tombstone.nwkDst =
              destination;

            tombstone.capability = 0;

            tombstone.immediate =
              false;

            // This candidate belongs specifically to the
            // neighbor that sent the DELETE.
            tombstone.nextHop =
              helloSrc;

            tombstone.pathlossDb =
              std::numeric_limits<double>::quiet_NaN ();

            tombstone.numHop = 0;

            tombstone.linkCostToNextHop = 0;
            tombstone.advertisedCost = 0;
            tombstone.cost = 0;

            tombstone.learnedFrom =
              helloSrc;

            tombstone.path.clear ();

            tombstone.energyLevel = 100;

            tombstone.lastUpdated =
              Simulator::Now ();

            tombstone.valid = false;

            m_routes.push_back (
              tombstone);

            matchingRouteFound = true;

            std::cout << "[NWK " << m_nodeId
                      << "] RoutingDelete created invalid tombstone"
                      << " dst=" << destination
                      << " learnedFrom="
                      << helloSrc
                      << " nextHop="
                      << helloSrc
                      << " routingSequence="
                      << incomingSequence
                      << std::endl;
          }

        std::cout << "[NWK " << m_nodeId
                  << "] RoutingDelete completed"
                  << " from=" << helloSrc
                  << " destination="
                  << destination
                  << " invalidated="
                  << (routeInvalidated ? 1 : 0)
                  << std::endl;

        SelectedRouteState selectedAfter =
          CaptureSelectedRouteState (
            destination);

        if (!SameSelectedRouteState (
              selectedBefore,
              selectedAfter))
          {
            MarkSelectedRouteChanged (
              destination,
              "RoutingDelete");
          }

        DumpRoutes ();
        ScheduleCheckNwkQueue ();

        return;
      }

    case CsrRoutingOperation::Flush:
      {
        if (!neighbor.routingSnapshotActive)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] RoutingSnapshot FLUSH received"
                      << " without active snapshot"
                      << " from=" << helloSrc
                      << " routingSequence="
                      << incomingSequence
                      << std::endl;

            return;
          }

        uint32_t invalidated = 0;

        // Preserve the selected state for every destination
        // that FLUSH may invalidate. Multiple candidates may
        // exist for the same destination, so capture each
        // destination only once before modifying anything.
        std::map<uint16_t, SelectedRouteState>
          selectedBeforeFlush;

        for (auto &route : m_routes)
          {
            // The direct link to the reporting neighbor is
            // established by hearing the packet itself and
            // must not be flushed.
            if (!route.valid ||
                route.immediate ||
                route.learnedFrom != helloSrc)
              {
                continue;
              }

            bool present =
              neighbor
                .routingSnapshotSeenDestinations
                .find (route.nwkDst) !=
              neighbor
                .routingSnapshotSeenDestinations
                .end ();

            if (!present)
              {
                // Capture the destination's selected state before
                // the first candidate for that destination is
                // invalidated.
                if (selectedBeforeFlush.find (
                      route.nwkDst) ==
                    selectedBeforeFlush.end ())
                  {
                    selectedBeforeFlush.emplace (
                      route.nwkDst,
                      CaptureSelectedRouteState (
                        route.nwkDst));
                  }

                route.valid = false;
                route.lastUpdated =
                  Simulator::Now ();

                invalidated++;

                std::cout << "[NWK " << m_nodeId
                          << "] RoutingSnapshot FLUSH invalidated"
                          << " dst=" << route.nwkDst
                          << " learnedFrom="
                          << helloSrc
                          << std::endl;
              }
          }

          uint32_t selectedRoutesChanged = 0;

        for (const auto &entry :
            selectedBeforeFlush)
          {
            uint16_t destination =
              entry.first;

            const SelectedRouteState &before =
              entry.second;

            SelectedRouteState after =
              CaptureSelectedRouteState (
                destination);

            if (!SameSelectedRouteState (
                  before,
                  after))
              {
                MarkSelectedRouteChanged (
                  destination,
                  "RoutingSnapshot FLUSH");

                selectedRoutesChanged++;

                std::cout << "[NWK " << m_nodeId
                          << "] RoutingSnapshot FLUSH changed selection"
                          << " dst=" << destination
                          << " oldAvailable="
                          << (before.available ? 1 : 0)
                          << " oldNextHop="
                          << before.nextHop
                          << " oldCost="
                          << before.cost
                          << " newAvailable="
                          << (after.available ? 1 : 0)
                          << " newNextHop="
                          << after.nextHop
                          << " newCost="
                          << after.cost
                          << std::endl;
              }
          }

        neighbor.routingSnapshotActive =
          false;

        neighbor.routingUpdateSectionStateValid =
          false;

        neighbor.routingUpdateSectionSequence =
          0;

        neighbor.routingUpdateLastSection =
          0;

        neighbor.routingUpdateTotalSections =
          1;

        neighbor
          .routingSnapshotSeenDestinations
          .clear ();

        neighbor
          .routingSnapshotBufferedUpdates
          .clear ();

        std::cout << "[NWK " << m_nodeId
                  << "] RoutingSnapshot FLUSH completed"
                  << " from=" << helloSrc
                  << " routingSequence="
                  << incomingSequence
                  << " routesInvalidated="
                  << invalidated
                  << " selectedRoutesChanged="
                  << selectedRoutesChanged
                  << std::endl;

        if (neighbor.routingRequestPending)
          {
            if (neighbor
                  .routingRequestTimeoutEvent
                  .IsPending ())
              {
                Simulator::Cancel (
                  neighbor
                    .routingRequestTimeoutEvent);
              }

            std::cout << "[NWK " << m_nodeId
                      << "] RoutingRequest fulfilled"
                      << " neighbor=" << helloSrc
                      << " requestSequence="
                      << neighbor.routingRequestSequence
                      << " flushSequence="
                      << incomingSequence
                      << " retries="
                      << neighbor.routingRequestRetryCount
                      << std::endl;

            neighbor.routingRequestPending = false;
            neighbor.routingRequestSequence = 0;
            neighbor.routingRequestRetryCount = 0;
          }

        DumpRoutes ();
        ScheduleCheckNwkQueue ();

        return;
      }

    case CsrRoutingOperation::None:
    default:
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Ignoring RoutingControl with no operation"
                  << " from=" << helloSrc
                  << " routingSequence="
                  << incomingSequence
                  << std::endl;

        return;
      }
    }
}

void
CsrNetLayer::UpdateReverseRoute (uint16_t netSrc, uint16_t hopSrc)
{
  if (netSrc == m_nodeId ||
      netSrc == CSR_BROADCAST_ID ||
      hopSrc == CSR_BROADCAST_ID)
    {
      return;
    }

  // Legacy ARL only accepts a reverse path through a known neighbor.
  auto nit = m_nwkNeighbors.find (hopSrc);
  if (nit == m_nwkNeighbors.end ())
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring reverse path for netSrc=" << netSrc
                << " via unknown hop=" << hopSrc
                << std::endl;
      return;
    }

  ReverseRouteEntry &rr = m_reverseRoutes[netSrc];
  bool wasValid = rr.valid;
  uint16_t oldHop = rr.reverseHop;

  rr.netSrc = netSrc;
  rr.reverseHop = hopSrc;
  rr.lastUpdated = Simulator::Now ();
  rr.valid = true;

  std::cout << "[NWK " << m_nodeId
            << "] " << (wasValid ? "Updated" : "Added")
            << " reverse path netSrc=" << netSrc
            << " reverseHop=" << hopSrc;

  if (wasValid && oldHop != hopSrc)
    {
      std::cout << " oldReverseHop=" << oldHop;
    }

  std::cout << std::endl;
}

bool
CsrNetLayer::RemoveReverseRouteFromReporter (uint16_t netSrc,
                                uint16_t reporter)
{
  auto it = m_reverseRoutes.find (netSrc);

  if (it == m_reverseRoutes.end () || !it->second.valid)
    {
      std::cout << "[NWK " << m_nodeId
                << "] NoPath found no valid reverse path for netSrc="
                << netSrc
                << std::endl;
      return false;
    }

  ReverseRouteEntry &rr = it->second;

  if (rr.reverseHop != reporter)
    {
      std::cout << "[NWK " << m_nodeId
                << "] NoPath reporter=" << reporter
                << " does not match reverseHop=" << rr.reverseHop
                << " for netSrc=" << netSrc
                << "; treating NoPath as stale or informational"
                << std::endl;
      return false;
    }

  rr.valid = false;
  rr.lastUpdated = Simulator::Now ();

  std::cout << "[NWK " << m_nodeId
            << "] Removed reverse path netSrc=" << netSrc
            << " reverseHop=" << reporter
            << " reason=NoPath"
            << std::endl;

  return true;
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
  bool chirpNeeded = false;
  uint32_t newlyStaleCount = 0;

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

          bool hadPendingRoutingRequest =
            ne.routingRequestPending;

          bool hadInboundSnapshot =
            ne.routingSnapshotActive;

          bool hadOutboundSnapshot = false;

          if (ne.routingRequestTimeoutEvent.IsPending ())
            {
              Simulator::Cancel (
                ne.routingRequestTimeoutEvent);
            }

          ne.routingRequestPending = false;
          ne.routingRequestSequence = 0;
          ne.routingRequestRetryCount = 0;

          ne.routingSnapshotActive = false;
          ne.routingSnapshotInfoSequence = 0;
          ne.routingSnapshotSeenDestinations.clear ();

          ne
            .routingSnapshotBufferedUpdates
            .clear ();

          // Discard any partially received multi-section Update.
          ne.routingUpdateSectionStateValid = false;
          ne.routingUpdateSectionSequence = 0;
          ne.routingUpdateLastSection = 0;
          ne.routingUpdateTotalSections = 1;

          auto outboundSnapshotIt =
            m_outboundRoutingSnapshots.find (
              ne.nodeId);

          if (outboundSnapshotIt !=
                m_outboundRoutingSnapshots.end ())
            {
              OutboundRoutingSnapshot &snapshot =
                outboundSnapshotIt->second;

              hadOutboundSnapshot =
                snapshot.active;

              if (snapshot.watchdogEvent.IsPending ())
                {
                  Simulator::Cancel (
                    snapshot.watchdogEvent);
                }

              // Prevent any previously scheduled watchdog
              // callback from acting on a future snapshot.
              snapshot.watchdogGeneration++;

              snapshot.watchdogPhase =
                "neighbor-stale";

              snapshot.active = false;
            }

          // Do not leave a stale neighbor queued for
          // automatic route-update fanout.
          bool removedPendingAutomaticUpdate =
            m_pendingAutomaticRouteChanges.erase (
              ne.nodeId) > 0;

          std::cout << "[NWK " << m_nodeId
                    << "] Cleared stale routing transaction state"
                    << " neighbor=" << ne.nodeId
                    << " pendingRequest="
                    << (hadPendingRoutingRequest ? 1 : 0)
                    << " inboundSnapshot="
                    << (hadInboundSnapshot ? 1 : 0)
                    << " outboundSnapshot="
                    << (hadOutboundSnapshot ? 1 : 0)
                    << " removedAutomaticUpdate="
                    << (removedPendingAutomaticUpdate
                          ? 1
                          : 0)
                    << std::endl;

          // Do not reuse radio limits learned before
          // the neighbor became stale. A fresh INFO
          // exchange must re-establish them.
          ne.routingInfoValid = false;
          ne.routingInfoSequence = 0;

          ne.negotiatedLinkProfileValid = false;
          ne.negotiatedSpeedCompatible = false;
          ne.negotiatedPowerCompatible = false;

          std::cout << "[NWK " << m_nodeId
                    << "] Cleared stale RoutingInfo"
                    << " neighbor=" << ne.nodeId
                    << std::endl;

          chirpNeeded = true;
          newlyStaleCount++;

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
  if (chirpNeeded)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Neighbor transitions to stale="
                << newlyStaleCount
                << "; scheduling automatic Discover Chirp"
                << std::endl;

      Simulator::ScheduleNow (
        &CsrNetLayer::SendDiscoveryChirp,
        this);
    }

  m_neighborFreshnessEvent =
    Simulator::Schedule (m_neighborFreshnessCheckPeriod,
                         &CsrNetLayer::CheckNeighborFreshness,
                         this);
}

void
CsrNetLayer::InvalidateRoutesViaNextHop (
  uint16_t nextHop,
  const char *reason)
{
  bool anyInvalidated = false;

  // Preserve the selected route for each affected
  // destination before invalidating candidates.
  std::map<uint16_t, SelectedRouteState>
    selectedBefore;

  for (auto &route : m_routes)
    {
      if (!route.valid)
        {
          continue;
        }

      if (route.nextHop != nextHop)
        {
          continue;
        }

      if (selectedBefore.find (
            route.nwkDst) ==
          selectedBefore.end ())
        {
          selectedBefore.emplace (
            route.nwkDst,
            CaptureSelectedRouteState (
              route.nwkDst));
        }

      route.valid = false;
      route.lastUpdated =
        Simulator::Now ();

      anyInvalidated = true;

      std::cout << "[NWK " << m_nodeId
                << "] Invalidated route"
                << " dst=" << route.nwkDst
                << " nextHop="
                << route.nextHop
                << " cost=" << route.cost
                << " reason=" << reason
                << std::endl;
    }

  // Determine whether invalidation actually changed
  // the selected route. Invalidating a non-selected
  // backup candidate should not generate an update.
  for (const auto &entry :
       selectedBefore)
    {
      uint16_t destination =
        entry.first;

      const SelectedRouteState
        &before = entry.second;

      SelectedRouteState after =
        CaptureSelectedRouteState (
          destination);

      if (!SameSelectedRouteState (
            before,
            after))
        {
          MarkSelectedRouteChanged (
            destination,
            reason);
        }
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
  CsrDiscoverType discoverType = hh.GetDiscoverType ();

  std::cout << "[NWK " << m_nodeId
            << "] ProcessDiscover from " << helloSrc
            << " subtype="
            << (discoverType == CsrDiscoverType::Broadcast
                  ? "Broadcast"
                  : discoverType == CsrDiscoverType::Chirp
                      ? "Chirp"
                      : "None")
            << " sequence=" << hh.GetDiscoverySequence ()
            << std::endl;

  switch (discoverType)
    {
    case CsrDiscoverType::Broadcast:
      {
        auto neighborIt = m_nwkNeighbors.find (helloSrc);
        if (neighborIt == m_nwkNeighbors.end ())
          {
            break;
          }

        NwkNeighborEntry &neighbor = neighborIt->second;
        uint32_t receivedSequence = hh.GetDiscoverySequence ();

        bool isNewSequence =
          !neighbor.discoverySequenceValid ||
          static_cast<int32_t> (
            receivedSequence - neighbor.discoverySequence) > 0;

        if (isNewSequence)
          {
            uint32_t oldSequence = neighbor.discoverySequence;

            neighbor.discoverySequence = receivedSequence;
            neighbor.discoverySequenceValid = true;

            std::cout << "[NWK " << m_nodeId
                      << "] New discovery sequence from "
                      << helloSrc
                      << " old=" << oldSequence
                      << " new=" << receivedSequence
                      << std::endl;

            if (m_discoveryResponseEnabled)
              {
                std::cout << "[NWK " << m_nodeId
                          << "] Scheduling Discovery NeighborCheck response to "
                          << helloSrc
                          << " for sequence=" << receivedSequence
                          << std::endl;

                Simulator::Schedule (
                  MilliSeconds (20),
                  &CsrNetLayer::SendNeighborCheck,
                  this,
                  helloSrc,
                  CsrNeighborCheckType::Discovery,
                  CSR_BROADCAST_ID,
                  receivedSequence);
              }
            else
              {
                std::cout << "[NWK " << m_nodeId
                          << "] Suppressing Discovery response to "
                          << helloSrc
                          << " for sequence=" << receivedSequence
                          << " testMode=true"
                          << std::endl;
              }
          }
        else
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Duplicate/old discovery sequence from "
                      << helloSrc
                      << " received=" << receivedSequence
                      << " current=" << neighbor.discoverySequence
                      << std::endl;
          }

        break;
      }

    case CsrDiscoverType::Chirp:
      {
        bool selfListed = false;

        for (uint8_t index = 0;
            index < hh.GetChirpNeighborCount ();
            ++index)
          {
            if (hh.GetChirpNeighbor (index) == m_nodeId)
              {
                selfListed = true;
                break;
              }
          }

        auto neighborIt = m_nwkNeighbors.find (helloSrc);

        bool senderWasActive =
          neighborIt != m_nwkNeighbors.end () &&
          neighborIt->second.wasActiveBeforeLastHello;

        std::cout << "[NWK " << m_nodeId
                  << "] Discover Chirp from " << helloSrc
                  << " listedNeighbors="
                  << unsigned (hh.GetChirpNeighborCount ())
                  << " selfListed=" << (selfListed ? 1 : 0)
                  << " senderWasActive="
                  << (senderWasActive ? 1 : 0)
                  << std::endl;

        if (!selfListed && senderWasActive)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Chirp from " << helloSrc
                      << " omitted local node; scheduling Verify"
                      << std::endl;

            Simulator::Schedule (
              MilliSeconds (20),
              &CsrNetLayer::SendNeighborCheck,
              this,
              helloSrc,
              CsrNeighborCheckType::Verify,
              CSR_BROADCAST_ID,
              0);
          }
        else if (!selfListed)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Chirp omitted local node, but sender was not"
                      << " previously considered active; no Verify"
                      << std::endl;
          }

        break;
      }

    case CsrDiscoverType::None:
    default:
      std::cout << "[NWK " << m_nodeId
                << "] Discover missing subtype from "
                << helloSrc
                << std::endl;
      break;
    }

  if (hh.GetAdvertisedRouteCount () > 0)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring Routes_PAYLOAD inside Discover"
                << std::endl;
    }

  // ProcessHello() already refreshed the direct neighbor route.
  (void) pathlossDb;
  (void) snrDb;
  (void) linkCost;
}

void
CsrNetLayer::SetDiscoveryResponseEnabled (bool enable)
{
  m_discoveryResponseEnabled = enable;

  std::cout << "[NWK " << m_nodeId
            << "] discovery_response_enabled="
            << (enable ? "true" : "false")
            << std::endl;
}

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
      {
        uint32_t responseSequence = hh.GetDiscoverySequence ();

        std::cout << "[NWK " << m_nodeId
                  << "] Discovery response from neighbor="
                  << helloSrc
                  << " responseSequence=" << responseSequence
                  << " localDiscoverySequence=" << m_discoverySequence
                  << std::endl;

        if (!m_discoveryActive)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Ignoring Discovery response from "
                      << helloSrc
                      << " because discovery is not active"
                      << std::endl;
            break;
          }

        if (responseSequence != m_discoverySequence)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Ignoring stale/mismatched Discovery response"
                      << " neighbor=" << helloSrc
                      << " responseSequence=" << responseSequence
                      << " expectedSequence=" << m_discoverySequence
                      << std::endl;
            break;
          }

        auto neighborIt = m_nwkNeighbors.find (helloSrc);

        if (neighborIt == m_nwkNeighbors.end ())
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Discovery response from unknown neighbor="
                      << helloSrc
                      << std::endl;
            break;
          }

        NwkNeighborEntry &neighbor = neighborIt->second;
        bool wasVerified = neighbor.discoveryVerified;

        neighbor.discoveryVerified = true;
        neighbor.stale = false;

        std::cout << "[NWK " << m_nodeId
                  << "] Discovery response accepted neighbor="
                  << helloSrc
                  << " verified="
                  << (wasVerified ? 1 : 0)
                  << "->1"
                  << " sequence=" << responseSequence
                  << " reportedActiveNodes="
                  << unsigned (hh.GetActiveNodes ())
                  << std::endl;

        break;
      }

    case CsrNeighborCheckType::Message:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck Message confirms link with "
                << helloSrc
                << std::endl;
      break;

    case CsrNeighborCheckType::Verify:
      {
        uint32_t verifySequence = hh.GetDiscoverySequence ();

        auto neighborIt = m_nwkNeighbors.find (helloSrc);

        if (neighborIt == m_nwkNeighbors.end ())
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Verify received from unknown neighbor="
                      << helloSrc
                      << std::endl;
            break;
          }

        NwkNeighborEntry &neighbor = neighborIt->second;

        neighbor.stale = false;
        neighbor.lastHeardSec = Simulator::Now ().GetSeconds ();

        std::cout << "[NWK " << m_nodeId
                  << "] Verify received from neighbor="
                  << helloSrc
                  << " discoverySequence=" << verifySequence
                  << " link confirmed"
                  << std::endl;

        break;
      }

    case CsrNeighborCheckType::Overheard:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck Overheard received from "
                << helloSrc
                << std::endl;
      break;

    /*case CsrNeighborCheckType::NoPath:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck NoPath received from "
                << helloSrc
                << " ; destination payload not implemented yet"
                << std::endl;
      break;*/

    case CsrNeighborCheckType::NoPath:
      {
        uint16_t unreachableDest = hh.GetNeighborCheckTarget ();

        std::cout << "[NWK " << m_nodeId
                  << "] NeighborCheck NoPath received from "
                  << helloSrc
                  << " unreachableDest=" << unreachableDest
                  << std::endl;

        if (unreachableDest == CSR_BROADCAST_ID)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Ignoring malformed NoPath with no destination"
                      << std::endl;
            break;
          }

        // ----------------------------------------------------------
        // Forward-route check:
        // Is the NoPath reporter currently our next hop to this
        // destination?
        // ----------------------------------------------------------
        bool matchingForwardRoute = false;

        for (const auto &re : m_routes)
          {
            if (re.valid &&
                re.nwkDst == unreachableDest &&
                re.nextHop == helloSrc)
              {
                matchingForwardRoute = true;

                std::cout << "[NWK " << m_nodeId
                          << "] NoPath reporter=" << helloSrc
                          << " is current nextHop for dst="
                          << unreachableDest
                          << "; preserving forward route while awaiting update"
                          << std::endl;

                break;
              }
          }

        if (!matchingForwardRoute)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] NoPath from " << helloSrc
                      << " does not match current forward route for dst="
                      << unreachableDest
                      << "; treating report as stale or informational"
                      << std::endl;
          }

        // ----------------------------------------------------------
        // Reverse-route check:
        // Remove the reverse path only when the NoPath reporter
        // matches the neighbor through which that source was learned.
        // ----------------------------------------------------------
        bool reverseRemoved =
          RemoveReverseRouteFromReporter (unreachableDest, helloSrc);

        if (!reverseRemoved)
          {
            std::cout << "[NWK " << m_nodeId
                      << "] No matching reverse path removed for dst="
                      << unreachableDest
                      << " reporter=" << helloSrc
                      << std::endl;
          }

        break;
      }

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

  ++m_discoverySequence;

  // Avoid using zero after uint32 wrap.
  if (m_discoverySequence == 0)
    {
      ++m_discoverySequence;
    }

  for (auto &kv : m_nwkNeighbors)
    {
      kv.second.discoveryVerified = false;
    }

  std::cout << "[NWK " << m_nodeId
            << "] DiscoveryStart Broadcast sequence="
            << m_discoverySequence
            << " resetVerifiedNeighbors="
            << m_nwkNeighbors.size ()
            << std::endl;

  SendHelloBroadcast (
    CsrArlRouteMsgType::Discover,
    CsrNeighborCheckType::None,
    CsrDiscoverType::Broadcast,
    m_discoverySequence);

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
            << "] DiscoveryStop sequence="
            << m_discoverySequence
            << std::endl;

  VerifyUnresponsiveDiscoveryNeighbors ();

  Simulator::Schedule (
    m_discoveryCooldown,
    &CsrNetLayer::DiscoveryCooldownOver,
    this);

  TryDrainQueueAfterDiscovery ();

  // Legacy discovery completion calls routesReroute():
  // recompute active-neighbor costs and request fresh
  // route state from every active neighbor.
  RefreshRoutesAfterDiscovery ();
}

void
CsrNetLayer::VerifyUnresponsiveDiscoveryNeighbors ()
{
  uint32_t verifiedCount = 0;
  uint32_t verifyScheduledCount = 0;
  uint32_t staleVerifyCount = 0;

  Time verifyDelay = MilliSeconds (20);

  for (auto &kv : m_nwkNeighbors)
    {
      NwkNeighborEntry &neighbor = kv.second;

      if (neighbor.discoveryVerified)
        {
          verifiedCount++;
          continue;
        }

      if (neighbor.stale)
        {
          staleVerifyCount++;

          std::cout << "[NWK " << m_nodeId
                    << "] Discovery completion includes stale neighbor="
                    << neighbor.nodeId
                    << " for Verify"
                    << std::endl;
        }

      uint16_t neighborId = neighbor.nodeId;
      Time thisDelay =
        verifyDelay * static_cast<int64_t> (verifyScheduledCount + 1);

      std::cout << "[NWK " << m_nodeId
                << "] Scheduling Verify for unresponsive neighbor="
                << neighborId
                << " sequence=" << m_discoverySequence
                << " delayMs=" << thisDelay.GetMilliSeconds ()
                << std::endl;

      Simulator::Schedule (
        thisDelay,
        &CsrNetLayer::SendNeighborCheck,
        this,
        neighborId,
        CsrNeighborCheckType::Verify,
        CSR_BROADCAST_ID,
        m_discoverySequence);

      verifyScheduledCount++;
    }

    std::cout << "[NWK " << m_nodeId
              << "] Discovery verification summary"
              << " sequence=" << m_discoverySequence
              << " known=" << m_nwkNeighbors.size ()
              << " verified=" << verifiedCount
              << " verifyScheduled=" << verifyScheduledCount
              << " staleIncluded=" << staleVerifyCount
              << std::endl;
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
            << "] Discovery Broadcast repeat sequence="
            << m_discoverySequence
            << std::endl;

  SendHelloBroadcast (
    CsrArlRouteMsgType::Discover,
    CsrNeighborCheckType::None,
    CsrDiscoverType::Broadcast,
    m_discoverySequence);

  ScheduleDiscoveryHello ();
}

void
CsrNetLayer::SendDiscoveryChirp ()
{
  std::cout << "[NWK " << m_nodeId
            << "] Sending ARL Discover Chirp"
            << std::endl;

  SendHelloBroadcast (
    CsrArlRouteMsgType::Discover,
    CsrNeighborCheckType::None,
    CsrDiscoverType::Chirp,
    0);
}

void
CsrNetLayer::SendHelloBroadcast (
  CsrArlRouteMsgType type,
  CsrNeighborCheckType checkType,
  CsrDiscoverType discoverType,
  uint32_t discoverySequence,
  uint32_t routingSequence)
{
  if (!m_hop) return;

  Ptr<Packet> p = Create<Packet> ();

  static uint16_t helloSeq = 0;

  CsrHelloHeader hh;
  hh.SetNodeId (m_nodeId);
  hh.SetHelloSeq (++helloSeq);

  hh.SetNodeType (m_nodeType);

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
  hh.SetDiscoverType (discoverType);
  hh.SetDiscoverySequence (discoverySequence);
  hh.SetRoutingSequence (routingSequence);

  hh.ClearChirpNeighbors ();

  if (type == CsrArlRouteMsgType::Discover &&
      discoverType == CsrDiscoverType::Chirp)
    {
      for (const auto &kv : m_nwkNeighbors)
        {
          const NwkNeighborEntry &neighbor = kv.second;

          // "Fresh" is the current NS-3 proxy for legacy NEIGHBOR_ACTIVE.
          if (neighbor.lastHeardSec < 0.0 || neighbor.stale)
            {
              continue;
            }

          if (hh.AddChirpNeighbor (neighbor.nodeId))
            {
              std::cout << "[NWK " << m_nodeId
                        << "] Chirp includes active neighbor="
                        << neighbor.nodeId
                        << std::endl;
            }
        }

      std::cout << "[NWK " << m_nodeId
                << "] Chirp activeNeighborCount="
                << unsigned (hh.GetChirpNeighborCount ())
                << std::endl;
    }

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
  uint32_t activeCount = 1; // local node

  for (const auto &kv : m_nwkNeighbors)
    {
      const NwkNeighborEntry &neighbor = kv.second;

      if (neighbor.lastHeardSec >= 0.0 &&
          !neighbor.stale)
        {
          activeCount++;
        }
    }

  return activeCount;
}

uint32_t
CsrNetLayer::GetNeighborCount () const
{
  return static_cast<uint32_t> (m_nwkNeighbors.size ());
}

void
CsrNetLayer::SendRoutingUpdate ()
{
  ++m_routingSequence;

  // Avoid zero after wrap so zero remains useful for
  // non-routing control packets.
  if (m_routingSequence == 0)
    {
      ++m_routingSequence;
    }

  std::cout << "[NWK " << m_nodeId
            << "] Sending ARL RoutingUpdate"
            << " routingSequence=" << m_routingSequence
            << std::endl;

  if (m_nodeType == CsrNodeType::Ordinary)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ordinary node sends no transit route advertisements"
                << std::endl;
    }

  SendHelloBroadcast (
    CsrArlRouteMsgType::RoutingUpdate,
    CsrNeighborCheckType::None,
    CsrDiscoverType::None,
    0,
    m_routingSequence);
}

void
CsrNetLayer::SendRoutingUpdateWithSequenceForTest (uint32_t sequence)
{
  std::cout << "[NWK " << m_nodeId
            << "] Sending test RoutingUpdate"
            << " routingSequence=" << sequence
            << std::endl;

  SendHelloBroadcast (
    CsrArlRouteMsgType::RoutingUpdate,
    CsrNeighborCheckType::None,
    CsrDiscoverType::None,
    0,
    sequence);
}

void
CsrNetLayer::SendNeighborCheck (
  uint16_t neighbor,
  CsrNeighborCheckType type,
  uint16_t target,
  uint32_t discoverySequence)
{
  if (m_hop == nullptr)
    {
      return;
    }

  Ptr<Packet> p = Create<Packet> ();

  CsrHelloHeader hh;
  hh.SetNodeId (m_nodeId);
  hh.SetHelloSeq (++m_neighborCheckSeq);
  hh.SetSpeedKey (m_minSpeedKey);

  hh.SetNodeType (m_nodeType);

  double s0PowerDbm = m_rxS0BaseLevelDbm + m_linkMarginDb;
  hh.SetRxPowerDbmX10 (
    static_cast<int16_t> (std::round (s0PowerDbm * 10.0)));

  hh.SetActiveNodes (
    static_cast<uint8_t> (GetActiveNodeCount ()));

  hh.SetArlRouteMsgType (
    CsrArlRouteMsgType::NeighborCheck);

  hh.SetNeighborCheckType (type);
  hh.SetNeighborCheckTarget (target);
  hh.SetDiscoverySequence (discoverySequence);
  hh.ClearAdvertisedRoutes ();

  p->AddHeader (hh);
  std::cout << "[NWK " << m_nodeId
            << "] Sending targeted ARL NeighborCheck"
            << " neighbor=" << neighbor
            << " subtype=" << NeighborCheckTypeName (type);

  if (type == CsrNeighborCheckType::NoPath)
    {
      std::cout << " unreachableDest=" << target;
    }

  if (type == CsrNeighborCheckType::Discovery ||
      type == CsrNeighborCheckType::Verify)
    {
      std::cout << " discoverySequence=" << discoverySequence;
    }

  std::cout << std::endl;

  m_hop->SendNeighborCheck (neighbor, p);
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

  if (m_nodeType == CsrNodeType::Ordinary)
    {
      return false;
    }

  if (!re.valid)
    {
      return false;
    }

  // Legacy ARL advertises only destinations with
  // routing capability. Ordinary nodes remain valid
  // local destinations but are not propagated as
  // multi-hop reachable destinations.
  if (re.capability == 0)
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

uint32_t
CsrNetLayer::
CountAdvertisableSelectedRoutes () const
{
  uint32_t count = 0;

  for (const auto &route : m_routes)
    {
      const RouteEntry *best =
        FindBestRoute (
          route.nwkDst);

      if (best != &route)
        {
          continue;
        }

      if (ShouldAdvertiseRoute (route))
        {
          count++;
        }
    }

  return count;
}

Ptr<Packet>
CsrNetLayer::BuildRoutingUpdatePayload (
  uint32_t routingSequence,
  uint8_t routingSection,
  uint8_t routingTotalSections)
{
  Ptr<Packet> packet =
    Create<Packet> ();

  CsrHelloHeader hh;

  hh.SetNodeId (m_nodeId);
  hh.SetHelloSeq (
    ++m_routingControlHeaderSeq);
  hh.SetNodeType (m_nodeType);
  hh.SetSpeedKey (m_minSpeedKey);

  double s0PowerDbm =
    m_rxS0BaseLevelDbm +
    m_linkMarginDb;

  hh.SetRxPowerDbmX10 (
    static_cast<int16_t> (
      std::round (
        s0PowerDbm * 10.0)));

  hh.SetActiveNodes (
    static_cast<uint8_t> (
      GetActiveNodeCount ()));

  hh.SetArlRouteMsgType (
    CsrArlRouteMsgType::
      RoutingUpdate);

  hh.SetNeighborCheckType (
    CsrNeighborCheckType::None);

  hh.SetDiscoverType (
    CsrDiscoverType::None);

  hh.SetDiscoverySequence (0);

  hh.SetRoutingSequence (
    routingSequence);

  hh.SetRoutingSection (
  routingSection);

  hh.SetRoutingTotalSections (
  routingTotalSections);

  hh.SetRoutingOperation (
    CsrRoutingOperation::Update);

  hh.ClearChirpNeighbors ();
  hh.ClearAdvertisedRoutes ();

  static constexpr uint32_t
    ROUTES_PER_SECTION = 8;

  uint8_t added = 0;
  uint32_t eligibleIndex = 0;

  uint32_t firstRouteIndex =
    static_cast<uint32_t> (
      routingSection) *
    ROUTES_PER_SECTION;

  for (const auto &route : m_routes)
    {
      const RouteEntry *best =
        FindBestRoute (
          route.nwkDst);

      if (best != &route)
        {
          continue;
        }

      if (!ShouldAdvertiseRoute (route))
        {
          continue;
        }

      // Count only routes that are actually eligible
      // to appear in the routing snapshot.
      if (eligibleIndex <
          firstRouteIndex)
        {
          eligibleIndex++;
          continue;
        }

      eligibleIndex++;

      int16_t pathlossX10 = 0;

      if (!std::isnan (
            route.pathlossDb))
        {
          pathlossX10 =
            static_cast<int16_t> (
              std::round (
                route.pathlossDb *
                10.0));
        }

      if (hh.AddAdvertisedRoute (
            route.nwkDst,
            route.numHop,
            route.cost,
            pathlossX10,
            route.capability,
            route.path))
        {
          added++;
        }

      if (added >= ROUTES_PER_SECTION)
        {
          break;
        }
    }

  std::cout << "[NWK " << m_nodeId
            << "] Built reliable RoutingUpdate"
            << " routingSequence="
            << routingSequence
            << " section="
            << unsigned (routingSection)
            << "/"
            << unsigned (
                routingTotalSections)
            << " advertisedRoutes="
            << unsigned (added)
            << std::endl;

  packet->AddHeader (hh);

  return packet;
}

Ptr<Packet>
CsrNetLayer::
BuildTargetedRoutingUpdatePayload (
  uint16_t destination,
  uint32_t routingSequence)
{
  const RouteEntry *route =
    FindBestRoute (
      destination);

  if (route == nullptr ||
      !ShouldAdvertiseRoute (*route))
    {
      return nullptr;
    }

  Ptr<Packet> packet =
    Create<Packet> ();

  CsrHelloHeader hh;

  hh.SetNodeId (m_nodeId);

  hh.SetHelloSeq (
    ++m_routingControlHeaderSeq);

  hh.SetNodeType (
    m_nodeType);

  hh.SetSpeedKey (
    m_minSpeedKey);

  double s0PowerDbm =
    m_rxS0BaseLevelDbm +
    m_linkMarginDb;

  hh.SetRxPowerDbmX10 (
    static_cast<int16_t> (
      std::round (
        s0PowerDbm * 10.0)));

  hh.SetActiveNodes (
    static_cast<uint8_t> (
      GetActiveNodeCount ()));

  hh.SetArlRouteMsgType (
    CsrArlRouteMsgType::
      RoutingUpdate);

  hh.SetNeighborCheckType (
    CsrNeighborCheckType::None);

  hh.SetDiscoverType (
    CsrDiscoverType::None);

  hh.SetDiscoverySequence (0);

  hh.SetRoutingSequence (
    routingSequence);

  hh.SetRoutingSection (0);
  hh.SetRoutingTotalSections (1);

  hh.SetRoutingOperation (
    CsrRoutingOperation::Update);

  hh.ClearChirpNeighbors ();
  hh.ClearAdvertisedRoutes ();

  int16_t pathlossX10 = 0;

  if (!std::isnan (
        route->pathlossDb))
    {
      pathlossX10 =
        static_cast<int16_t> (
          std::round (
            route->pathlossDb *
            10.0));
    }

  bool added =
    hh.AddAdvertisedRoute (
      route->nwkDst,
      route->numHop,
      route->cost,
      pathlossX10,
      route->capability,
      route->path);

  if (!added)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Failed to build targeted RoutingUpdate"
                << " destination="
                << destination
                << std::endl;

      return nullptr;
    }

  std::cout << "[NWK " << m_nodeId
            << "] Built targeted RoutingUpdate"
            << " destination="
            << destination
            << " nextHop="
            << route->nextHop
            << " cost="
            << route->cost
            << " hops="
            << unsigned (
                route->numHop)
            << " routingSequence="
            << routingSequence
            << std::endl;

  packet->AddHeader (hh);

  return packet;
}

Ptr<Packet>
CsrNetLayer::BuildRoutingRequestPayload (
  uint32_t routingSequence)
{
  Ptr<Packet> packet =
    Create<Packet> ();

  CsrHelloHeader hh;

  hh.SetNodeId (m_nodeId);
  hh.SetHelloSeq (
    ++m_routingControlHeaderSeq);
  hh.SetNodeType (m_nodeType);
  hh.SetSpeedKey (m_minSpeedKey);

  double s0PowerDbm =
    m_rxS0BaseLevelDbm +
    m_linkMarginDb;

  hh.SetRxPowerDbmX10 (
    static_cast<int16_t> (
      std::round (
        s0PowerDbm * 10.0)));

  hh.SetActiveNodes (
    static_cast<uint8_t> (
      GetActiveNodeCount ()));

  hh.SetArlRouteMsgType (
    CsrArlRouteMsgType::RoutingUpdate);

  hh.SetNeighborCheckType (
    CsrNeighborCheckType::None);

  hh.SetDiscoverType (
    CsrDiscoverType::None);

  hh.SetDiscoverySequence (0);
  hh.SetRoutingSequence (
    routingSequence);

  hh.SetRoutingOperation (
    CsrRoutingOperation::Request);

  hh.ClearChirpNeighbors ();
  hh.ClearAdvertisedRoutes ();

  packet->AddHeader (hh);

  return packet;
}

Ptr<Packet>
CsrNetLayer::BuildRoutingMarkerPayload (
  CsrRoutingOperation operation,
  uint32_t routingSequence,
  uint16_t routingTarget)
{
  Ptr<Packet> packet =
    Create<Packet> ();

  CsrHelloHeader hh;

  hh.SetNodeId (m_nodeId);

  hh.SetHelloSeq (
    ++m_routingControlHeaderSeq);

  hh.SetNodeType (m_nodeType);
  hh.SetSpeedKey (m_minSpeedKey);

  double s0PowerDbm =
    m_rxS0BaseLevelDbm +
    m_linkMarginDb;

  hh.SetRxPowerDbmX10 (
    static_cast<int16_t> (
      std::round (
        s0PowerDbm * 10.0)));

  hh.SetActiveNodes (
    static_cast<uint8_t> (
      GetActiveNodeCount ()));

  hh.SetArlRouteMsgType (
    CsrArlRouteMsgType::RoutingUpdate);

  hh.SetNeighborCheckType (
    CsrNeighborCheckType::None);

  hh.SetDiscoverType (
    CsrDiscoverType::None);

  hh.SetDiscoverySequence (0);

  hh.SetRoutingSequence (
    routingSequence);

  hh.SetRoutingOperation (
    operation);

  if (operation ==
      CsrRoutingOperation::Info)
    {
      CsrHelloHeader::RoutingInfo info;

      info.minSpeedKbps =
        static_cast<uint16_t> (
          std::clamp (
            m_minCfgSpeedKbps,
            0,
            65535));

      info.maxSpeedKbps =
        static_cast<uint16_t> (
          std::clamp (
            m_maxCfgSpeedKbps,
            0,
            65535));

      info.minPowerDbmX10 =
        static_cast<int16_t> (
          std::round (
            m_minTxPowerDbm * 10.0));

      info.maxPowerDbmX10 =
        static_cast<int16_t> (
          std::round (
            m_maxTxPowerDbm * 10.0));

      info.linkMarginDbX10 =
        static_cast<int16_t> (
          std::round (
            m_linkMarginDb * 10.0));

      // Closest current ns-3 equivalent to the
      // legacy RF low-power crossover.
      info.lowPowerDbmX10 =
        static_cast<int16_t> (
          std::round (
            m_txAmpBreakpointDbm * 10.0));

      // Legacy ROUTING_INFO includes the configured
      // temperature operating limits.
      info.tempLowCx10 =
        m_tempLowCx10;

      info.tempHighCx10 =
        m_tempHighCx10;

      hh.SetRoutingInfo (info);
    }

  hh.SetRoutingTarget (
    routingTarget);

  hh.ClearChirpNeighbors ();
  hh.ClearAdvertisedRoutes ();

  std::cout << "[NWK " << m_nodeId
            << "] Built RoutingSnapshot marker"
            << " operation="
            << RoutingOperationName (operation)
            << " routingSequence="
            << routingSequence
            << std::endl;

  packet->AddHeader (hh);

  return packet;
}

void
CsrNetLayer::StartReliableRoutingSnapshot (
  uint16_t neighbor)
{
  if (m_hop == nullptr)
    {
      return;
    }

  auto neighborIt =
    m_nwkNeighbors.find (neighbor);

  if (neighborIt == m_nwkNeighbors.end ())
    {
      std::cout << "[NWK " << m_nodeId
                << "] RoutingSnapshot rejected"
                << " unknownNeighbor="
                << neighbor
                << std::endl;
      return;
    }

  if (neighborIt->second.stale)
    {
      std::cout << "[NWK " << m_nodeId
                << "] RoutingSnapshot rejected"
                << " staleNeighbor="
                << neighbor
                << std::endl;
      return;
    }

  OutboundRoutingSnapshot &snapshot =
    m_outboundRoutingSnapshots[neighbor];

  if (snapshot.active)
    {
      std::cout << "[NWK " << m_nodeId
                << "] RoutingSnapshot already active"
                << " neighbor=" << neighbor
                << std::endl;
      return;
    }

  snapshot.active = true;

  static constexpr uint32_t
  ROUTES_PER_SECTION = 8;

  uint32_t advertisedRouteCount =
    CountAdvertisableSelectedRoutes ();

  uint32_t calculatedSections =
    std::max<uint32_t> (
      1,
      (advertisedRouteCount +
        ROUTES_PER_SECTION - 1) /
        ROUTES_PER_SECTION);

  snapshot.totalUpdateSections =
    static_cast<uint8_t> (
      std::min<uint32_t> (
        255,
        calculatedSections));

  snapshot.currentUpdateSection = 0;

  uint32_t snapshotSequence =
    NextRoutingSequence ();

  snapshot.infoSequence =
    snapshotSequence;

  snapshot.updateSequence =
    snapshotSequence;

  snapshot.flushSequence =
    snapshotSequence;

  std::cout << "[NWK " << m_nodeId
            << "] Starting reliable RoutingSnapshot"
            << " neighbor=" << neighbor
            << " infoSequence="
            << snapshot.infoSequence
            << " updateSequence="
            << snapshot.updateSequence
            << " flushSequence="
            << snapshot.flushSequence
            << " advertisedRoutes="
            << advertisedRouteCount
            << " updateSections="
            << unsigned (
                snapshot.totalUpdateSections)
            << std::endl;

  Ptr<Packet> infoPayload =
    BuildRoutingMarkerPayload (
      CsrRoutingOperation::Info,
      snapshot.infoSequence);

  m_hop->SendRoutingControl (
    neighbor,
    infoPayload);
}

void
CsrNetLayer::
ArmRoutingSnapshotWatchdog (
  uint16_t neighbor,
  const char *phase)
{
  auto snapshotIt =
    m_outboundRoutingSnapshots.find (
      neighbor);

  if (snapshotIt ==
      m_outboundRoutingSnapshots.end ())
    {
      return;
    }

  OutboundRoutingSnapshot &snapshot =
    snapshotIt->second;

  if (!snapshot.active)
    {
      return;
    }

  if (snapshot.watchdogEvent.IsPending ())
    {
      Simulator::Cancel (
        snapshot.watchdogEvent);
    }

  snapshot.watchdogGeneration++;

  snapshot.watchdogPhase =
    phase;

  uint32_t generation =
    snapshot.watchdogGeneration;

  snapshot.watchdogEvent =
    Simulator::Schedule (
      m_routingSnapshotWatchdogTimeout,
      &CsrNetLayer::
        RoutingSnapshotWatchdogExpired,
      this,
      neighbor,
      generation);

  std::cout << "[NWK " << m_nodeId
            << "] Armed RoutingSnapshot watchdog"
            << " neighbor=" << neighbor
            << " phase=" << phase
            << " generation="
            << generation
            << " timeoutSec="
            << m_routingSnapshotWatchdogTimeout
                 .GetSeconds ()
            << std::endl;
}

void
CsrNetLayer::
RoutingSnapshotWatchdogExpired (
  uint16_t neighbor,
  uint32_t expectedGeneration)
{
  auto snapshotIt =
    m_outboundRoutingSnapshots.find (
      neighbor);

  if (snapshotIt ==
      m_outboundRoutingSnapshots.end ())
    {
      return;
    }

  OutboundRoutingSnapshot &snapshot =
    snapshotIt->second;

  if (!snapshot.active)
    {
      return;
    }

  if (snapshot.watchdogGeneration !=
      expectedGeneration)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring stale RoutingSnapshot watchdog"
                << " neighbor=" << neighbor
                << " expectedGeneration="
                << expectedGeneration
                << " currentGeneration="
                << snapshot.watchdogGeneration
                << std::endl;

      return;
    }

  std::cout << "[NWK " << m_nodeId
            << "] RoutingSnapshot watchdog expired"
            << " neighbor=" << neighbor
            << " phase="
            << snapshot.watchdogPhase
            << " infoSequence="
            << snapshot.infoSequence
            << " updateSequence="
            << snapshot.updateSequence
            << " section="
            << unsigned (
                snapshot.currentUpdateSection)
            << "/"
            << unsigned (
                snapshot.totalUpdateSections)
            << " flushSequence="
            << snapshot.flushSequence
            << std::endl;

  snapshot.active = false;
  snapshot.watchdogPhase =
    "aborted";

  // A selected-route change may have been waiting
  // behind this snapshot.
  if (!m_pendingAutomaticRouteChanges.empty () &&
      !m_automaticRouteUpdateEvent.IsPending ())
    {
      m_automaticRouteUpdateEvent =
        Simulator::ScheduleNow (
          &CsrNetLayer::
            TrySendAutomaticRouteUpdates,
          this);
    }
}

void
CsrNetLayer::SendRoutingRequestAttempt (
  uint16_t neighbor,
  bool retry)
{
  if (m_hop == nullptr)
    {
      return;
    }

  auto neighborIt =
    m_nwkNeighbors.find (neighbor);

  if (neighborIt ==
      m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &entry =
    neighborIt->second;

  if (!entry.routingRequestPending)
    {
      return;
    }

  if (entry.stale)
    {
      entry.routingRequestPending = false;
      entry.routingRequestRetryCount = 0;

      std::cout << "[NWK " << m_nodeId
                << "] RoutingRequest aborted"
                << " neighbor became stale="
                << neighbor
                << std::endl;
      return;
    }

  uint32_t sequence =
    NextRoutingSequence ();

  entry.routingRequestSequence =
    sequence;

  Ptr<Packet> payload =
    BuildRoutingRequestPayload (
      sequence);

  std::cout << "[NWK " << m_nodeId
            << "] Sending reliable RoutingRequest"
            << " neighbor=" << neighbor
            << " routingSequence="
            << sequence
            << " attempt="
            << (entry.routingRequestRetryCount + 1)
            << " retry="
            << (retry ? 1 : 0)
            << std::endl;

  m_hop->SendRoutingControl (
    neighbor,
    payload);

  if (entry.routingRequestTimeoutEvent.IsPending ())
    {
      Simulator::Cancel (
        entry.routingRequestTimeoutEvent);
    }

  entry.routingRequestTimeoutEvent =
    Simulator::Schedule (
      m_routingRequestTimeout,
      &CsrNetLayer::RoutingRequestTimeout,
      this,
      neighbor,
      sequence);
}

void
CsrNetLayer::RoutingRequestTimeout (
  uint16_t neighbor,
  uint32_t expectedSequence)
{
  auto neighborIt =
    m_nwkNeighbors.find (neighbor);

  if (neighborIt ==
      m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &entry =
    neighborIt->second;

  if (!entry.routingRequestPending)
    {
      return;
    }

  // Ignore an old timer left behind by a newer attempt.
  if (entry.routingRequestSequence !=
      expectedSequence)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring stale RoutingRequest timeout"
                << " neighbor=" << neighbor
                << " expectedSequence="
                << expectedSequence
                << " currentSequence="
                << entry.routingRequestSequence
                << std::endl;
      return;
    }

  std::cout << "[NWK " << m_nodeId
            << "] RoutingRequest timeout"
            << " neighbor=" << neighbor
            << " requestSequence="
            << expectedSequence
            << " retryCount="
            << entry.routingRequestRetryCount
            << std::endl;

  // Discard any partial INFO/UPDATE transaction.
  entry.routingSnapshotActive = false;
  entry.routingSnapshotInfoSequence = 0;
  entry.routingSnapshotSeenDestinations.clear ();
  entry
    .routingSnapshotBufferedUpdates
    .clear ();

  if (entry.stale)
    {
      entry.routingRequestPending = false;
      entry.routingRequestRetryCount = 0;

      std::cout << "[NWK " << m_nodeId
                << "] RoutingRequest failed"
                << " neighbor is stale="
                << neighbor
                << std::endl;
      return;
    }

  if (entry.routingRequestRetryCount >=
      m_maxRoutingRequestRetries)
    {
      entry.routingRequestPending = false;

      std::cout << "[NWK " << m_nodeId
                << "] RoutingRequest failed"
                << " neighbor=" << neighbor
                << " retriesExhausted="
                << entry.routingRequestRetryCount
                << std::endl;

      entry.routingRequestRetryCount = 0;
      return;
    }

  entry.routingRequestRetryCount++;

  std::cout << "[NWK " << m_nodeId
            << "] Retrying RoutingRequest"
            << " neighbor=" << neighbor
            << " retry="
            << entry.routingRequestRetryCount
            << std::endl;

  SendRoutingRequestAttempt (
    neighbor,
    true);
}

const CsrNetLayer::RouteEntry*
CsrNetLayer::FindBestRoute (
  uint16_t destination) const
{
  const RouteEntry *best = nullptr;

  bool preferredValid = false;

  uint16_t preferredNextHop =
    CSR_BROADCAST_ID;

  auto preferredIt =
    m_selectedRoutePreferredNextHop.find (
      destination);

  if (preferredIt !=
      m_selectedRoutePreferredNextHop.end ())
    {
      preferredValid = true;

      preferredNextHop =
        preferredIt->second;
    }

  for (const auto &route :
       m_routes)
    {
      if (route.nwkDst != destination ||
          !route.valid)
        {
          continue;
        }

      if (best == nullptr)
        {
          best = &route;
          continue;
        }

      bool better = false;

      // Legacy primary criterion:
      // lowest total route cost.
      if (route.cost <
          best->cost)
        {
          better = true;
        }

      // Legacy secondary criterion:
      // lowest hop count.
      else if (route.cost ==
                 best->cost &&
               route.numHop <
                 best->numHop)
        {
          better = true;
        }

      // Legacy tie behavior:
      // when cost and hops are identical,
      // retain the currently selected neighbor.
      else if (route.cost ==
                 best->cost &&
               route.numHop ==
                 best->numHop &&
               preferredValid &&
               route.nextHop ==
                 preferredNextHop &&
               best->nextHop !=
                 preferredNextHop)
        {
          better = true;
        }

      // Otherwise leave the first equal candidate
      // alone. Do not choose based on node ID.
      if (better)
        {
          best = &route;
        }
    }

  return best;
}

CsrNetLayer::SelectedRouteState
CsrNetLayer::CaptureSelectedRouteState (
  uint16_t destination) const
{
  SelectedRouteState state;

  const RouteEntry *best =
    FindBestRoute (destination);

  if (best == nullptr)
    {
      return state;
    }

  state.available = true;
  state.nextHop = best->nextHop;
  state.cost = best->cost;
  state.numHop = best->numHop;
  state.immediate = best->immediate;
  state.capability = best->capability;
  state.path = best->path;

  return state;
}

bool
CsrNetLayer::SameSelectedRouteState (
  const SelectedRouteState &first,
  const SelectedRouteState &second)
{
  return
    first.available ==
      second.available &&
    first.nextHop ==
      second.nextHop &&
    first.cost ==
      second.cost &&
    first.numHop ==
      second.numHop &&
    first.immediate ==
      second.immediate &&
    first.capability ==
      second.capability &&
    first.path ==
      second.path;
}

void
CsrNetLayer::MarkSelectedRouteChanged (
  uint16_t destination,
  const char *reason)
  {
    const RouteEntry *selectedRoute =
    FindBestRoute (
      destination);

  if (selectedRoute != nullptr)
    {
      m_selectedRoutePreferredNextHop[
        destination] =
          selectedRoute->nextHop;

      std::cout << "[NWK " << m_nodeId
                << "] Updated selected-route preference"
                << " dst=" << destination
                << " preferredNextHop="
                << selectedRoute->nextHop
                << std::endl;
    }
  else
    {
      m_selectedRoutePreferredNextHop.erase (
        destination);

      std::cout << "[NWK " << m_nodeId
                << "] Cleared selected-route preference"
                << " dst=" << destination
                << " reason=no-valid-route"
                << std::endl;
    }

  m_pendingSelectedRouteChanges.insert (
    destination);

  std::cout << "[NWK " << m_nodeId
            << "] Selected route changed"
            << " dst=" << destination
            << " reason=" << reason
            << " pendingDestinations="
            << m_pendingSelectedRouteChanges.size ()
            << std::endl;

  if (!m_selectedRouteChangeEvent.IsPending ())
    {
      m_selectedRouteChangeEvent =
        Simulator::Schedule (
          m_selectedRouteChangeDelay,
          &CsrNetLayer::
            ReportPendingSelectedRouteChanges,
          this);
    }
}

void
CsrNetLayer::
ReportPendingSelectedRouteChanges ()
{
  std::set<uint16_t> destinations;

  destinations.swap (
    m_pendingSelectedRouteChanges);

  std::cout << "[NWK " << m_nodeId
            << "] Consolidated selected-route changes"
            << " count="
            << destinations.size ()
            << std::endl;

  for (uint16_t destination :
       destinations)
    {
      DumpBestRoute (destination);
    }

  if (!m_automaticRoutePropagationEnabled)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Automatic route propagation disabled;"
                << " reporting only"
                << std::endl;

      return;
    }

  uint32_t queuedNeighbors = 0;
  uint32_t queuedDestinationPairs = 0;

  for (const auto &entry :
      m_nwkNeighbors)
    {
      const NwkNeighborEntry &neighbor =
        entry.second;

      bool fresh =
        neighbor.lastHeardSec >= 0.0 &&
        !neighbor.stale;

      if (!fresh)
        {
          continue;
        }

      std::set<uint16_t>
        &pendingDestinations =
          m_pendingAutomaticRouteChanges[
            neighbor.nodeId];

      bool wasEmpty =
        pendingDestinations.empty ();

      for (uint16_t destination :
          destinations)
        {
          bool inserted =
            pendingDestinations.insert (
              destination)
              .second;

          if (inserted)
            {
              queuedDestinationPairs++;
            }
        }

      if (wasEmpty &&
          !pendingDestinations.empty ())
        {
          queuedNeighbors++;
        }
    }

  std::cout << "[NWK " << m_nodeId
            << "] Queued automatic destination-specific"
            << " route propagation"
            << " freshNeighborsAdded="
            << queuedNeighbors
            << " destinationPairsAdded="
            << queuedDestinationPairs
            << " pendingNeighbors="
            << m_pendingAutomaticRouteChanges.size ()
            << std::endl;

  if (!m_pendingAutomaticRouteChanges.empty () &&
      !m_automaticRouteUpdateEvent.IsPending ())
    {
      m_automaticRouteUpdateEvent =
        Simulator::ScheduleNow (
          &CsrNetLayer::
            TrySendAutomaticRouteUpdates,
          this);
    }
}

void
CsrNetLayer::
TrySendAutomaticRouteUpdates ()
{
  if (!m_automaticRoutePropagationEnabled ||
      m_hop == nullptr)
    {
      return;
    }

  bool sentAction = false;
  bool blockedBySnapshot = false;

  auto pendingIt =
    m_pendingAutomaticRouteChanges.begin ();

  while (pendingIt !=
         m_pendingAutomaticRouteChanges.end ())
    {
      uint16_t neighborId =
        pendingIt->first;

      auto neighborIt =
        m_nwkNeighbors.find (
          neighborId);

      if (neighborIt ==
            m_nwkNeighbors.end () ||
          neighborIt->second.stale)
        {
          std::cout << "[NWK " << m_nodeId
                    << "] Dropping automatic route propagation"
                    << " unavailableNeighbor="
                    << neighborId
                    << " destinations="
                    << pendingIt->second.size ()
                    << std::endl;

          pendingIt =
            m_pendingAutomaticRouteChanges.erase (
              pendingIt);

          continue;
        }

      auto snapshotIt =
        m_outboundRoutingSnapshots.find (
          neighborId);

      bool snapshotActive =
        snapshotIt !=
          m_outboundRoutingSnapshots.end () &&
        snapshotIt->second.active;

      if (snapshotActive)
        {
          blockedBySnapshot = true;
          ++pendingIt;
          continue;
        }

      std::set<uint16_t>
        &pendingDestinations =
          pendingIt->second;

      if (pendingDestinations.empty ())
        {
          pendingIt =
            m_pendingAutomaticRouteChanges.erase (
              pendingIt);

          continue;
        }

      uint16_t destination =
        *pendingDestinations.begin ();

      pendingDestinations.erase (
        pendingDestinations.begin ());

      bool neighborCompleted =
        pendingDestinations.empty ();

      if (neighborCompleted)
        {
          m_pendingAutomaticRouteChanges.erase (
            pendingIt);
        }

      const RouteEntry *selectedRoute =
        FindBestRoute (
          destination);

      bool sendUpdate =
        selectedRoute != nullptr &&
        ShouldAdvertiseRoute (
          *selectedRoute);

      if (sendUpdate)
        {
          uint32_t sequence =
            NextRoutingSequence ();

          Ptr<Packet> payload =
            BuildTargetedRoutingUpdatePayload (
              destination,
              sequence);

          if (payload != nullptr)
            {
              std::cout << "[NWK " << m_nodeId
                        << "] Sending automatic targeted RoutingUpdate"
                        << " neighbor="
                        << neighborId
                        << " destination="
                        << destination
                        << " routingSequence="
                        << sequence
                        << std::endl;

              m_hop->SendRoutingControl (
                neighborId,
                payload);
            }
          else
            {
              // The selected route changed again between
              // queueing and packet construction.
              std::cout << "[NWK " << m_nodeId
                        << "] Targeted Update became unavailable;"
                        << " sending Delete instead"
                        << " neighbor="
                        << neighborId
                        << " destination="
                        << destination
                        << std::endl;

              SendReliableRoutingDelete (
                neighborId,
                destination);
            }
        }
      else
        {
          std::cout << "[NWK " << m_nodeId
                    << "] Sending automatic targeted RoutingDelete"
                    << " neighbor="
                    << neighborId
                    << " destination="
                    << destination
                    << std::endl;

          SendReliableRoutingDelete (
            neighborId,
            destination);
        }

      sentAction = true;
      break;
    }

  if (m_pendingAutomaticRouteChanges.empty ())
    {
      std::cout << "[NWK " << m_nodeId
                << "] Automatic destination-specific"
                << " route propagation completed"
                << std::endl;

      return;
    }

  Time nextDelay =
    sentAction
      ? m_automaticRouteUpdateSpacing
      : m_automaticRouteUpdateRetryDelay;

  std::cout << "[NWK " << m_nodeId
            << "] Automatic route propagation rescheduled"
            << " pendingNeighbors="
            << m_pendingAutomaticRouteChanges.size ()
            << " blockedBySnapshot="
            << (blockedBySnapshot ? 1 : 0)
            << " delayMs="
            << nextDelay.GetMilliSeconds ()
            << std::endl;

  m_automaticRouteUpdateEvent =
    Simulator::Schedule (
      nextDelay,
      &CsrNetLayer::
        TrySendAutomaticRouteUpdates,
      this);
}

void
CsrNetLayer::DumpBestRoute (
  uint16_t destination) const
{
  uint32_t candidateCount = 0;
  uint32_t validCount = 0;

  for (const auto &route : m_routes)
    {
      if (route.nwkDst != destination)
        {
          continue;
        }

      candidateCount++;

      if (route.valid)
        {
          validCount++;
        }
    }

  const RouteEntry *best =
    FindBestRoute (destination);

  if (best == nullptr)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Best route"
                << " dst=" << destination
                << " unavailable"
                << " candidates="
                << candidateCount
                << " validCandidates="
                << validCount
                << std::endl;

      return;
    }

  std::cout << "[NWK " << m_nodeId
            << "] Best route"
            << " dst=" << destination
            << " nextHop="
            << best->nextHop
            << " cost=" << best->cost
            << " hops="
            << unsigned (best->numHop)
            << " learnedFrom="
            << best->learnedFrom
            << " candidates="
            << candidateCount
            << " validCandidates="
            << validCount
            << std::endl;
}

void
CsrNetLayer::
UpdateNegotiatedLinkProfile (
  NwkNeighborEntry &neighbor)
{
  neighbor.negotiatedLinkProfileValid =
    false;

  neighbor.negotiatedSpeedCompatible =
    false;

  neighbor.negotiatedPowerCompatible =
    false;

  if (!neighbor.routingInfoValid)
    {
      return;
    }

  uint16_t localMinSpeed =
    static_cast<uint16_t> (
      std::clamp (
        m_minCfgSpeedKbps,
        0,
        65535));

  uint16_t localMaxSpeed =
    static_cast<uint16_t> (
      std::clamp (
        m_maxCfgSpeedKbps,
        0,
        65535));

  neighbor.negotiatedMinSpeedKbps =
    std::max (
      localMinSpeed,
      neighbor.remoteMinSpeedKbps);

  neighbor.negotiatedMaxSpeedKbps =
    std::min (
      localMaxSpeed,
      neighbor.remoteMaxSpeedKbps);

  neighbor.negotiatedSpeedCompatible =
    neighbor.negotiatedMinSpeedKbps <=
      neighbor.negotiatedMaxSpeedKbps &&
    neighbor.negotiatedMaxSpeedKbps > 0;

  int16_t localMinPowerDbmX10 =
    static_cast<int16_t> (
      std::round (
        m_minTxPowerDbm * 10.0));

  int16_t localMaxPowerDbmX10 =
    static_cast<int16_t> (
      std::round (
        m_maxTxPowerDbm * 10.0));

  neighbor.negotiatedMinPowerDbmX10 =
    std::max (
      localMinPowerDbmX10,
      neighbor.remoteMinPowerDbmX10);

  neighbor.negotiatedMaxPowerDbmX10 =
    std::min (
      localMaxPowerDbmX10,
      neighbor.remoteMaxPowerDbmX10);

  neighbor.negotiatedPowerCompatible =
    neighbor.negotiatedMinPowerDbmX10 <=
      neighbor.negotiatedMaxPowerDbmX10;

  int16_t localLinkMarginDbX10 =
    static_cast<int16_t> (
      std::round (
        m_linkMarginDb * 10.0));

  neighbor.negotiatedLinkMarginDbX10 =
    std::max (
      localLinkMarginDbX10,
      neighbor.remoteLinkMarginDbX10);

  int16_t localLowPowerDbmX10 =
    static_cast<int16_t> (
      std::round (
        m_txAmpBreakpointDbm * 10.0));

  neighbor.negotiatedLowPowerDbmX10 =
    std::max (
      localLowPowerDbmX10,
      neighbor.remoteLowPowerDbmX10);

  // Keep the crossover inside the mutually
  // supported transmit-power range.
  if (neighbor.negotiatedPowerCompatible)
    {
      neighbor.negotiatedLowPowerDbmX10 =
        std::clamp (
          neighbor.negotiatedLowPowerDbmX10,
          neighbor.negotiatedMinPowerDbmX10,
          neighbor.negotiatedMaxPowerDbmX10);
    }
  else
    {
      // No valid power overlap exists, so there
      // is no meaningful crossover value.
      neighbor.negotiatedLowPowerDbmX10 = 0;
    }

  neighbor.negotiatedLinkProfileValid =
    neighbor.negotiatedSpeedCompatible &&
    neighbor.negotiatedPowerCompatible;

  std::cout << "[NWK " << m_nodeId
            << "] Negotiated link profile"
            << " neighbor=" << neighbor.nodeId
            << " valid="
            << (neighbor
                  .negotiatedLinkProfileValid
                  ? 1
                  : 0)
            << " speedCompatible="
            << (neighbor
                  .negotiatedSpeedCompatible
                  ? 1
                  : 0)
            << " speedKbps="
            << neighbor
                 .negotiatedMinSpeedKbps
            << "-"
            << neighbor
                 .negotiatedMaxSpeedKbps
            << " powerCompatible="
            << (neighbor
                  .negotiatedPowerCompatible
                  ? 1
                  : 0)
            << " powerDbmX10="
            << neighbor
                 .negotiatedMinPowerDbmX10
            << "-"
            << neighbor
                 .negotiatedMaxPowerDbmX10
            << " marginDbX10="
            << neighbor
                 .negotiatedLinkMarginDbX10
            << " lowPowerDbmX10="
            << neighbor
                 .negotiatedLowPowerDbmX10
            << std::endl;
}

void
CsrNetLayer::
RecomputeRoutesViaNextHop (
  uint16_t nextHop)
{
  auto neighborIt =
    m_nwkNeighbors.find (
      nextHop);

  if (neighborIt ==
      m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &neighbor =
    neighborIt->second;

  if (std::isnan (
        neighbor.lastPathlossDb))
    {
      return;
    }

  // Capture the selected state before modifying
  // any candidate costs through this neighbor.
  std::map<uint16_t, SelectedRouteState>
    selectedBefore;

  for (const auto &route :
       m_routes)
    {
      if (!route.valid ||
          route.nextHop != nextHop)
        {
          continue;
        }

      if (selectedBefore.find (
            route.nwkDst) ==
          selectedBefore.end ())
        {
          selectedBefore.emplace (
            route.nwkDst,
            CaptureSelectedRouteState (
              route.nwkDst));
        }
    }

  // ----------------------------------------------------------
  // Recompute the neighbor cost using the same bidirectional
  // rules used when ProcessHello() originally learns the link.
  // Legacy routesReroute() calls routesComputeHopCost(), so a
  // reroute must not fall back to a one-direction-only cost.
  // ----------------------------------------------------------

  double remoteS0PowerDbm =
    static_cast<double> (
      neighbor.rxPowerDbmX10) /
    10.0;

  if (remoteS0PowerDbm == 0.0)
    {
      remoteS0PowerDbm =
        m_rxS0BaseLevelDbm +
        m_linkMarginDb;
    }

  // Local direction:
  // this node transmits to the neighbor.
  int localSpeed = 0;
  double localTxPower = 0.0;
  int localDistance = 0;
  double localSpeedMargin = 0.0;
  double localTotalMargin = 0.0;

  uint32_t localDirectionalCost =
    ComputeLinkCost (
      remoteS0PowerDbm,
      neighbor.lastPathlossDb,
      neighbor.numFailures,
      &localSpeed,
      &localTxPower,
      &localDistance,
      &localSpeedMargin,
      &localTotalMargin,
      nullptr);

  uint32_t newLinkCost =
    localDirectionalCost;

  bool usingBidirectionalCost =
    false;

  uint32_t remoteDirectionalCost = 0;

  bool remoteLimitsValid =
    neighbor.routingInfoValid &&
    neighbor.remoteMinSpeedKbps > 0 &&
    neighbor.remoteMinSpeedKbps <=
      neighbor.remoteMaxSpeedKbps &&
    neighbor.remoteMinPowerDbmX10 <=
      neighbor.remoteMaxPowerDbmX10;

  if (remoteLimitsValid)
    {
      NwkNeighborEntry remoteTxProfile;

      remoteTxProfile
        .negotiatedLinkProfileValid =
          true;

      remoteTxProfile
        .negotiatedMinSpeedKbps =
          neighbor.remoteMinSpeedKbps;

      remoteTxProfile
        .negotiatedMaxSpeedKbps =
          neighbor.remoteMaxSpeedKbps;

      remoteTxProfile
        .negotiatedMinPowerDbmX10 =
          neighbor.remoteMinPowerDbmX10;

      remoteTxProfile
        .negotiatedMaxPowerDbmX10 =
          neighbor.remoteMaxPowerDbmX10;

      remoteTxProfile
        .negotiatedLowPowerDbmX10 =
          neighbor.remoteLowPowerDbmX10;

      // Reverse direction:
      // neighbor transmits to this node.
      double localS0PowerDbm =
        m_rxS0BaseLevelDbm +
        m_linkMarginDb;

      int remoteSpeed = 0;
      double remoteTxPower = 0.0;
      int remoteDistance = 0;
      double remoteSpeedMargin = 0.0;
      double remoteTotalMargin = 0.0;

      remoteDirectionalCost =
        ComputeLinkCost (
          localS0PowerDbm,
          neighbor.lastPathlossDb,
          neighbor.numFailures,
          &remoteSpeed,
          &remoteTxPower,
          &remoteDistance,
          &remoteSpeedMargin,
          &remoteTotalMargin,
          &remoteTxProfile);

      // Legacy routesComputeHopCost() takes the
      // pessimistic characteristic of both directions.
      int chosenSpeed =
        std::min (
          localSpeed,
          remoteSpeed);

      int estimatedDistance =
        std::max (
          localDistance,
          remoteDistance);

      double speedMargin =
        std::min (
          localSpeedMargin,
          remoteSpeedMargin);

      double totalMargin =
        std::min (
          localTotalMargin,
          remoteTotalMargin);

      // Apply the legacy 3-dB-per-failure adjustment
      // before converting distance to route cost.
      double failureMarginDb =
        3.0 *
        static_cast<double> (
          neighbor.numFailures);

      double adjustedTotalMarginDb =
        totalMargin -
        failureMarginDb;

      double adjustedSpeedMarginDb =
        speedMargin -
        failureMarginDb;

      uint64_t adjustedDistanceX100 =
        static_cast<uint64_t> (
          std::max (
            0,
            estimatedDistance)) *
        100ULL;

      if (adjustedTotalMarginDb < 0.0)
        {
          adjustedDistanceX100 *=
            2ULL;
        }
      else if (adjustedSpeedMarginDb >= 3.0)
        {
          adjustedDistanceX100 /=
            2ULL;
        }

      if (chosenSpeed > 0)
        {
          uint64_t rawCost =
            adjustedDistanceX100 /
            static_cast<uint64_t> (
              chosenSpeed);

          newLinkCost =
            static_cast<uint32_t> (
              std::min<uint64_t> (
                rawCost,
                std::numeric_limits<
                  uint32_t>::max ()));
        }
      else
        {
          newLinkCost = 1;
        }

      newLinkCost =
        std::max<uint32_t> (
          1,
          newLinkCost);

      usingBidirectionalCost =
        true;
    }

  std::cout << "[NWK " << m_nodeId
            << "] recompute hop-cost"
            << " neighbor="
            << nextHop
            << " failures="
            << neighbor.numFailures
            << " costMode="
            << (usingBidirectionalCost
                  ? "bidirectional"
                  : "local-only")
            << " localDirectionalCost="
            << localDirectionalCost
            << " remoteDirectionalCost="
            << remoteDirectionalCost
            << " newLinkCost="
            << newLinkCost
            << std::endl;

  for (auto &route :
       m_routes)
    {
      if (!route.valid ||
          route.nextHop != nextHop)
        {
          continue;
        }

      uint32_t oldCost =
        route.cost;

      route.linkCostToNextHop =
        newLinkCost;

      route.cost =
        route.linkCostToNextHop +
        route.advertisedCost;

      route.lastUpdated =
        Simulator::Now ();

      std::cout << "[NWK " << m_nodeId
                << "] Recomputed route after link change"
                << " dst=" << route.nwkDst
                << " nextHop=" << nextHop
                << " oldCost=" << oldCost
                << " newCost=" << route.cost
                << " linkCost="
                << route.linkCostToNextHop
                << " advertisedCost="
                << route.advertisedCost
                << " numFailures="
                << neighbor.numFailures
                << std::endl;
    }

  // Compare the selected route after all candidate
  // costs have been updated. A cost increase may cause
  // another next hop to become best.
  for (const auto &entry :
       selectedBefore)
    {
      uint16_t destination =
        entry.first;

      const SelectedRouteState &before =
        entry.second;

      SelectedRouteState after =
        CaptureSelectedRouteState (
          destination);

      if (!SameSelectedRouteState (
            before,
            after))
        {
          MarkSelectedRouteChanged (
            destination,
            "link cost recompute");

          std::cout << "[NWK " << m_nodeId
                    << "] Link-cost recompute changed selection"
                    << " dst=" << destination
                    << " oldAvailable="
                    << (before.available ? 1 : 0)
                    << " oldNextHop="
                    << before.nextHop
                    << " oldCost="
                    << before.cost
                    << " newAvailable="
                    << (after.available ? 1 : 0)
                    << " newNextHop="
                    << after.nextHop
                    << " newCost="
                    << after.cost
                    << std::endl;
        }
    }
}

void
CsrNetLayer::
RefreshRoutesAfterDiscovery ()
{
  std::vector<uint16_t>
    activeNeighbors;

  for (const auto &entry :
       m_nwkNeighbors)
    {
      const NwkNeighborEntry &neighbor =
        entry.second;

      bool active =
        neighbor.lastHeardSec >= 0.0 &&
        !neighbor.stale;

      if (!active)
        {
          continue;
        }

      activeNeighbors.push_back (
        neighbor.nodeId);
    }

  std::cout << "[NWK " << m_nodeId
            << "] Post-discovery routing refresh"
            << " activeNeighbors="
            << activeNeighbors.size ()
            << std::endl;

  // Legacy routesReroute() first recomputes
  // the cost to every active neighbor.
  for (uint16_t neighborId :
       activeNeighbors)
    {
      RecomputeRoutesViaNextHop (
        neighborId);
    }

  // Legacy then marks every active neighbor
  // as needing a fresh routing request.
  for (uint16_t neighborId :
       activeNeighbors)
    {
      auto it =
        m_nwkNeighbors.find (
          neighborId);

      if (it ==
          m_nwkNeighbors.end () ||
          it->second.stale)
        {
          continue;
        }

      if (it->second.routingRequestPending)
        {
          std::cout << "[NWK " << m_nodeId
                    << "] Post-discovery RoutingRequest"
                    << " already pending"
                    << " neighbor="
                    << neighborId
                    << std::endl;

          continue;
        }

      std::cout << "[NWK " << m_nodeId
                << "] Post-discovery requesting fresh routes"
                << " neighbor="
                << neighborId
                << std::endl;

      SendRoutingRequest (
        neighborId);
    }
}
// ------------------------------------------------------------
