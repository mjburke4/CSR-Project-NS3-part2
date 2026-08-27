#pragma once
#include "csr-common.h"
#include "csr-arl-routing-message.h"
#include "csr-hello-header.h"
#include "csr-hop-layer.h"
#include <cmath>
#include <set>
#include <algorithm>


class CsrNetLayer : public Object
{
public:

  enum class DiscoveryState { IDLE, SCHEDULED, ACTIVE };

  DiscoveryState m_discState { DiscoveryState::IDLE };
  EventId m_discoveryStartEvent, m_discoveryStopEvent;
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

  void SetNodeId (CsrNodeId id)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (id),
                     "CSR NWK node identifier exceeds 24 bits");
    m_nodeId = id;
  }

  /**
   * Select the legacy source-exact or owner-confirmed extended rate set.
   *
   * The setting propagates to HOP and MAC when those layers are attached.
   *
   * @param profile Runtime payload-rate profile.
   */
  void SetRateProfile (CsrRateProfile profile)
  {
    SetMaxSpeedKbps (CsrGetProfileMaxRateKbps (profile));
  }

  /** @return Active NWK/HOP/MAC payload-rate profile. */
  CsrRateProfile GetRateProfile () const
  {
    return m_maxCfgSpeedKbps > 128
      ? CsrRateProfile::EXTENDED_DQPSK
      : CsrRateProfile::LEGACY_SOURCE_EXACT;
  }

  /**
   * Set the configured legacy Max Speed attribute.
   *
   * @param rateKbps Maximum operational payload rate in kbit/s.
   */
  void SetMaxSpeedKbps (CsrRateKey rateKbps)
  {
    NS_ABORT_MSG_IF (!CsrIsOperationalRateKey (rateKbps),
                     "CSR NWK maximum speed is not operationally defined");
    m_maxCfgSpeedKbps = rateKbps;
    if (m_hop != nullptr)
      {
        m_hop->SetMaxSpeedKbps (rateKbps);
      }
  }

  /** @return Configured maximum route/link-control rate in kbit/s. */
  CsrRateKey GetMaxSpeedKbps () const
  {
    return static_cast<CsrRateKey> (m_maxCfgSpeedKbps);
  }

  /** Apply promoted route/link-control limits and propagate them to HOP/MAC. */
  void ConfigureLinkControl (CsrRateKey minSpeedKbps,
                             CsrRateKey maxSpeedKbps,
                             double minPowerDbm,
                             double maxPowerDbm,
                             double linkMarginDb)
  {
    NS_ABORT_MSG_IF (!CsrIsOperationalRateKey (minSpeedKbps) ||
                     !CsrIsOperationalRateKey (maxSpeedKbps) ||
                     minSpeedKbps > maxSpeedKbps,
                     "CSR NWK link-control speed range is invalid");
    NS_ABORT_MSG_IF (!std::isfinite (minPowerDbm) ||
                     !std::isfinite (maxPowerDbm) ||
                     !std::isfinite (linkMarginDb) ||
                     minPowerDbm > maxPowerDbm,
                     "CSR NWK link-control power range is invalid");
    m_minSpeedKey = minSpeedKbps;
    m_minCfgSpeedKbps = minSpeedKbps;
    m_maxCfgSpeedKbps = maxSpeedKbps;
    m_minTxPowerDbm = minPowerDbm;
    m_maxTxPowerDbm = maxPowerDbm;
    m_linkMarginDb = linkMarginDb;
    if (m_hop != nullptr)
      {
        m_hop->ConfigureLinkControl (minSpeedKbps,
                                     maxSpeedKbps,
                                     minPowerDbm,
                                     maxPowerDbm,
                                     linkMarginDb);
      }
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
    CsrNodeId neighbor,
    CsrNeighborCheckType type = CsrNeighborCheckType::Message,
    CsrNodeId target = CSR_BROADCAST_ID,
    uint32_t discoverySequence = 0);

  void StartReliableRoutingSnapshot (
    CsrNodeId neighbor);

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

  uint32_t GetDiscoveryStartCount () const
  {
    return m_discoveryStartCount;
  }

  uint32_t GetDiscoveryBroadcastCount () const
  {
    return m_discoveryBroadcastCount;
  }

  uint32_t GetSnmpStartSentCount () const
  {
    return m_snmpStartSentCount;
  }

  uint32_t GetSnmpStartReceivedCount () const
  {
    return m_snmpStartReceivedCount;
  }

  uint32_t GetSnmpDoneSentCount () const
  {
    return m_snmpDoneSentCount;
  }

  uint32_t GetSnmpDoneReceivedCount () const
  {
    return m_snmpDoneReceivedCount;
  }

  /**
   * Send the legacy one-hop relay-holdoff control to a neighbor.
   *
   * @param neighbor Final SNMP destination.
   * @return True when a route exists and the control was queued at HOP.
   */
  bool SendRelayHoldoff (CsrNodeId neighbor)
  {
    return SendSnmp (neighbor, CSR_SNMP_RELAY_HOLDOFF, 0);
  }

  /**
   * Send the legacy one-hop relay-clear control to a neighbor.
   *
   * @param neighbor Final SNMP destination.
   * @return True when a route exists and the control was queued at HOP.
   */
  bool SendRelayClear (CsrNodeId neighbor)
  {
    return SendSnmp (neighbor, CSR_SNMP_RELAY_CLEAR, 0);
  }

  /**
   * Read the last relay-control state received from a known neighbor.
   *
   * The supplied OPNET wrapper stores this flag but never consults it while
   * releasing the NWK queue.  It is therefore observable control state, not a
   * transit-forwarding gate.
   *
   * @param neighbor Neighbor whose received state should be queried.
   * @return True after RELAY_HOLDOFF and false after RELAY_CLEAR or when the
   *         neighbor is unknown.
   */
  bool IsRelayHoldoffSet (CsrNodeId neighbor) const
  {
    auto it = m_nwkNeighbors.find (neighbor);
    return it != m_nwkNeighbors.end () && it->second.relayHoldoff;
  }

  /** @return Number of relay-holdoff controls queued by this node. */
  uint32_t GetRelayHoldoffSentCount () const
  {
    return m_relayHoldoffSentCount;
  }

  /** @return Number of relay-clear controls queued by this node. */
  uint32_t GetRelayClearSentCount () const
  {
    return m_relayClearSentCount;
  }

  /** @return Number of relay-holdoff controls applied by this node. */
  uint32_t GetRelayHoldoffReceivedCount () const
  {
    return m_relayHoldoffReceivedCount;
  }

  /** @return Number of relay-clear controls applied by this node. */
  uint32_t GetRelayClearReceivedCount () const
  {
    return m_relayClearReceivedCount;
  }

  uint32_t GetPendingDiscoveryCount () const
  {
    return static_cast<uint32_t> (
      std::count_if (
        m_discoveryTable.begin (),
        m_discoveryTable.end (),
        [] (const DiscoveryEntry &entry) {
          return entry.discoveryNeeded;
        }));
  }

  CsrNodeId GetDiscoveryInitiatedBy () const
  {
    return m_discoveryInitiatedBy;
  }

  void ProcessHello (Ptr<Packet> helloPayload,
                   CsrNodeId hopSrc,
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

  uint32_t GetNwkQueueSize () const
  {
    return static_cast<uint32_t> (m_nwkQueue.size ());
  }

  uint32_t GetNsdpCount (CsrNodeId src, CsrNodeId dst) const
  {
    auto it = m_nsdp.find (std::make_pair (src, dst));
    return it == m_nsdp.end () ? 0 : it->second.count;
  }

  /**
   * Apply br_app's ENABLE_NWK_FLOW_CTRL admission test.
   *
   * The application suppresses packet creation/statistics while the local
   * source-destination pair has 16 outstanding packets.  NWK still retains
   * its source behavior of scanning other queued destinations independently.
   */
  bool CanAdmitApplicationPacket (CsrNodeId destination) const
  {
    return GetNsdpCount (m_nodeId, destination) < NSDP_DACK_THRESHOLD;
  }

  /**
   * Get the cost of the currently selected route.
   *
   * @param destination Network destination.
   * @param costOut Selected route cost when a route is available.
   * @return True when a selected route is available.
   */
  bool GetSelectedRouteCost (CsrNodeId destination,
                             uint32_t &costOut) const;

  /**
   * Read the last complete legacy ARL INFO record from a neighbor.
   *
   * @param neighbor Neighbor node identifier.
   * @param infoOut Decoded INFO fields when available.
   * @return True when the neighbor has valid routing information.
   */
  bool GetNeighborRoutingInfo (
    CsrNodeId neighbor,
    CsrHelloHeader::RoutingInfo &infoOut) const;

  /**
   * Count incomplete ARL routing sequences buffered for a neighbor.
   *
   * @param neighbor Neighbor node identifier.
   * @return Number of incomplete logical routing messages.
   */
  uint32_t GetPendingArlRoutingMessageCount (
    CsrNodeId neighbor) const;

  /**
   * Get the number of sections in the current outbound snapshot.
   *
   * @param neighbor Neighbor node identifier.
   * @return Total section count, or zero when no state exists.
   */
  uint8_t GetOutboundRoutingSnapshotTotalSections (
    CsrNodeId neighbor) const;

  /**
   * Count independently acknowledged sections in an outbound snapshot.
   *
   * @param neighbor Neighbor node identifier.
   * @return Number of acknowledged sections.
   */
  uint32_t GetOutboundRoutingSnapshotAckedSections (
    CsrNodeId neighbor) const;

  /**
   * Check whether an outbound snapshot is awaiting section acknowledgments.
   *
   * @param neighbor Neighbor node identifier.
   * @return True while the snapshot is active.
   */
  bool IsOutboundRoutingSnapshotActive (
    CsrNodeId neighbor) const;

  /** Enable the legacy two-sided-key plus NeighborCheck admission gate. */
  void SetArlNeighborAdmissionEnabled (bool enable);

  bool IsArlNeighborAdmissionEnabled () const
  {
    return m_arlNeighborAdmissionEnabled;
  }

  bool IsArlNeighborActive (CsrNodeId neighbor) const
  {
    auto it = m_nwkNeighbors.find (neighbor);
    return it != m_nwkNeighbors.end () && it->second.arlActive;
  }

  bool HasReceivedHopKey (CsrNodeId neighbor) const
  {
    auto it = m_nwkNeighbors.find (neighbor);
    return it != m_nwkNeighbors.end () && it->second.keyUpdateComplete;
  }

  bool HasSentHopKey (CsrNodeId neighbor) const
  {
    auto it = m_nwkNeighbors.find (neighbor);
    return it != m_nwkNeighbors.end () && it->second.keySendComplete;
  }

  uint32_t GetKeyRequestSentCount () const
  {
    return m_keyRequestSentCount;
  }

  uint32_t GetKeyUpdateSentCount () const
  {
    return m_keyUpdateSentCount;
  }

  uint32_t GetKeyUpdateReceivedCount () const
  {
    return m_keyUpdateReceivedCount;
  }

  uint32_t GetInactiveNeighborDropCount () const
  {
    return m_inactiveNeighborDropCount;
  }

  void
  SendNoPath (CsrNodeId neighbor, CsrNodeId unreachableDest)
  {
    SendNeighborCheck (neighbor,
                      CsrNeighborCheckType::NoPath,
                      unreachableDest);
  }

  void NoteLinkFailure (CsrNodeId nextHop)
  {
    auto &ne = m_nwkNeighbors[nextHop];

    ne.nodeId = nextHop;
    ne.numFailures++;

    if (m_hop != nullptr)
      {
        m_hop->SetNeighborFailureCount (nextHop,
                                        ne.numFailures);
      }

    std::cout << "[NWK " << m_nodeId
              << "] Link failure to nextHop=" << nextHop
              << " numFailures=" << ne.numFailures
              << std::endl;

    MakeNeighborInactive (nextHop, "link failure");
    RecomputeRoutesViaNextHop (nextHop);
  }

  void
  NoteNeighborCheckSuccess (
    CsrNodeId neighbor,
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

    if (m_hop != nullptr)
      {
        m_hop->SetNeighborFailureCount (neighbor,
                                        ne.numFailures);
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

      if (type == CsrNeighborCheckType::Discovery)
        {
          ne.admissionDiscoveryCheckActive = false;
          ne.discoverySequence = discoverySequence;
          ne.discoverySequenceValid = true;
        }

      ne.checkMessageActive = false;

      // routesHopSecSentPacket() makes an ACKed NeighborCheck an admission
      // proof.  routesMakeNeighborActive() still enforces both key directions.
      TryMakeNeighborActive (
        neighbor,
        "ACKed NeighborCheck");

      ScheduleCheckNwkQueue ();
  }

  void
  RecomputeRoutesViaNextHop (
    CsrNodeId nextHop);

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
          m_arlNeighborAdmissionEnabled
            ? neighbor.arlActive
            : (neighbor.lastHeardSec >= 0.0 &&
               !neighbor.stale);

        if (wasActive)
          {
            activeNeighborsCleared++;
            chirpNeeded = true;
          }

        neighbor.stale = true;
        neighbor.arlActive = false;
        neighbor.wasActiveBeforeLastHello = false;
        neighbor.arlRoutingReassemblies.clear ();

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
        m_hop->SetMaxSpeedKbps (
          static_cast<CsrRateKey> (m_maxCfgSpeedKbps));
        // Data/control payloads from HOP up to NWK
        m_hop->SetRxFromHopCallback (
          MakeCallback (&CsrNetLayer::ReceiveFromHop, this));

        // HELLO/control discovery path: OPNET proc_hello() equivalent
        m_hop->SetRxHelloFromHopCallback (
          MakeCallback (&CsrNetLayer::ProcessHello, this));

        // Legacy discovery orchestration uses a separate, non-reliable SNMP
        // payload path that does not update HOP neighbor state.
        m_hop->SetRxSnmpFromHopCallback (
          MakeCallback (&CsrNetLayer::ReceiveSnmpFromHop, this));

        // HOP ACK/DACK/resend completion releases NWK NSDP flow count
        m_hop->SetNsdpDecrementCallback (
          MakeCallback (&CsrNetLayer::DecrementNsdp, this));

        // HOP asks NWK whether this flow should use DACK-style delayed release
        m_hop->SetShouldDackCallback (
          MakeCallback (&CsrNetLayer::ShouldDack, this));

        m_hop->SetRelayRouteAvailableCallback (
          MakeCallback (&CsrNetLayer::HasRelayRoute, this));

        m_hop->SetLinkFailureCallback (
          MakeCallback (&CsrNetLayer::NoteLinkFailure, this));

        m_hop->SetNeighborCheckSuccessCallback (
          MakeCallback (&CsrNetLayer::NoteNeighborCheckSuccess, this));

        m_hop->SetNeighborCheckFailureCallback (
          MakeCallback (&CsrNetLayer::NoteNeighborCheckFailure, this));

        m_hop->SetKeyRequestReceivedCallback (
          MakeCallback (&CsrNetLayer::NoteKeyRequestReceived, this));

        m_hop->SetKeyUpdateReceivedCallback (
          MakeCallback (&CsrNetLayer::NoteKeyUpdateReceived, this));

        m_hop->SetKeyUpdateCompletionCallback (
          MakeCallback (&CsrNetLayer::NoteKeyUpdateCompletion, this));

        m_hop->SetSecurityCountChangeCallback (
          MakeCallback (&CsrNetLayer::NoteSecurityCountChange, this));

        m_hop->SetAuthenticatedGroupKeyNeededCallback (
          MakeCallback (
            &CsrNetLayer::NoteAuthenticatedGroupKeyNeeded,
            this));

        m_hop->SetLocalGroupKeyChangedCallback (
          MakeCallback (&CsrNetLayer::NoteLocalGroupKeyChanged, this));

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

        m_hop->SetNwkQueueWakeCallback (
          MakeCallback (
            &CsrNetLayer::
              ScheduleCheckNwkQueue,
            this));
      }
  }

  // Net -> App callback: payload + network source node ID
  void SetRxFromNetCallback (Callback<void, Ptr<Packet>, CsrNodeId> cb)
  {
    m_rxFromNetCb = cb;
  }

  bool HasRelayRoute (CsrNodeId nwkDst)
  {
    CsrNodeId nextHop;
    return LookupNextHop (nwkDst, nextHop);
  }

  bool ShouldDack (CsrNodeId src, CsrNodeId dst)
  {
    NsdpEntry &e = GetNsdpEntry (src, dst);
    return e.count >= e.limit;
  }

  // Called by App to send a payload to some destination
  void Send (CsrNodeId dst,
             uint8_t dscp,
             Ptr<Packet> payload,
             bool ackRequested)
  {
    // Wrap the app payload in a CsrNetHeader carrying nwk src/dst/DSCP.
    Ptr<Packet> framed = payload->Copy ();
    CsrNetHeader nh (m_nodeId, dst, dscp);
    framed->AddHeader (nh);

    NwkQueueEntry e;
    e.nwkSrc  = m_nodeId;
    e.nwkDst  = dst;
    e.dscp    = dscp;
    // This OPNET build has ENABLE_ACK_RESEND set globally.  Every
    // br_Hop DATA packet is therefore ACKable and Resendable; the
    // application cannot opt an individual DATA packet out of reliability.
    e.ack     = true;
    e.payload = framed;

    if (!ackRequested)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Ignoring non-ACKable DATA request"
                  << " dst=" << dst
                  << " to match legacy ENABLE_ACK_RESEND"
                  << std::endl;
      }

    // NSDP: increment count for (nwkSrc, nwkDst)
    NsdpEntry &nsdp = GetNsdpEntry (e.nwkSrc, e.nwkDst);
    nsdp.count++;

    std::cout << "[NWK " << m_nodeId << "] NSDP(" << e.nwkSrc
            << "->" << e.nwkDst << ") incremented to "
            << nsdp.count << std::endl;

    // OPNET does not numerically sort positive DSCP values here.  Every
    // positive-DSCP arrival is inserted at the head, while DSCP zero is
    // appended at the tail.  This intentionally makes positive traffic LIFO.
    if (dscp > 0)
      {
        m_nwkQueue.push_front (e);
      }
    else
      {
        m_nwkQueue.push_back (e);
      }

    ScheduleCheckNwkQueue ();
  }

  void DecrementNsdp (CsrNodeId src, CsrNodeId dst)
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
  void ReceiveFromHop (Ptr<Packet> packetFromHop, CsrNodeId hopSrc)
  {
    // Peek network header to inspect nwkSrc/nwkDst/DSCP
    CsrNetHeader nh;
    if (!packetFromHop->PeekHeader (nh))
      {
        NS_LOG_ERROR ("CsrNetLayer::ReceiveFromHop(): missing CsrNetHeader");
        return;
      }

    CsrNodeId nwkSrc = nh.GetSrc ();
    CsrNodeId nwkDst = nh.GetDst ();
    uint8_t  dscp   = nh.GetDscp ();

    if (!AcceptFromNeighbor (hopSrc))
      {
        m_inactiveNeighborDropCount++;

        std::cout << "[NWK " << m_nodeId
                  << "] Drop DATA from inactive ARL neighbor"
                  << " hopSrc=" << hopSrc
                  << " nwkSrc=" << nwkSrc
                  << " nwkDst=" << nwkDst
                  << " drops=" << m_inactiveNeighborDropCount
                  << std::endl;
        return;
      }

    UpdateReverseRoute (nwkSrc, hopSrc);

    if (nwkDst == m_nodeId)
      {
        // Final destination: retain the total br_Network size for trace
        // statistics, then strip our internal header before delivering the
        // application payload.  app_send uses the same total-size semantics.
        const uint32_t networkPacketBytes = packetFromHop->GetSize ();
        packetFromHop->RemoveHeader (nh);

        CsrDifferentialTraceEvent event;
        event.event = "nwk_delivery";
        event.node = CsrTraceInteger (m_nodeId);
        event.peer = CsrTraceInteger (hopSrc);
        event.packetType = "data";
        event.source = CsrTraceInteger (nwkSrc);
        event.destination = CsrTraceInteger (nwkDst);
        CsrDifferentialAppTag appTag;
        if (packetFromHop->PeekPacketTag (appTag))
          {
            event.sequence = CsrTraceInteger (appTag.GetSequence ());
          }
        event.sizeBytes = CsrTraceInteger (networkPacketBytes);
        event.success = "1";
        event.reason = "delivered";
        WriteDifferentialTrace (event);

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

        // Match proc_hop_pk(): all positive DSCP relay traffic goes to the
        // head, and best-effort traffic goes to the tail.
        if (dscp > 0)
          {
            m_nwkQueue.push_front (e);
          }
        else
          {
            m_nwkQueue.push_back (e);
          }

        ScheduleCheckNwkQueue ();
      }
  }

  bool
  AddOrUpdateRoute (
    CsrNodeId nwkDst,
    CsrNodeId nextHop,
    bool immediate,
    uint8_t numHop,
    double pathlossDb,
	    uint32_t linkCostToNextHop,
	    uint32_t advertisedCost,
	    CsrNodeId learnedFrom,
	    uint8_t capability = 0,
	    const std::vector<CsrNodeId> &path = {})
  {
    if (!CsrIsValidNodeId (nwkDst) ||
        !CsrIsValidNodeId (nextHop) ||
        !CsrIsValidNodeId (learnedFrom) ||
        std::any_of (path.begin (), path.end (), [] (CsrNodeId nodeId) {
          return !CsrIsValidNodeId (nodeId);
        }))
      {
        return false;
      }

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

    std::vector<CsrNodeId>
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

  void AddStaticRoute (CsrNodeId nwkDst, CsrNodeId nextHop)
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
  AddStaticRouteWithPathloss (CsrNodeId nwkDst,
                              CsrNodeId nextHop,
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
    CsrNodeId neighbor)
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

    if (entry.stale || !IsArlNeighborUsable (neighbor))
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingRequest rejected"
                  << " unavailableNeighbor="
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
    CsrNodeId neighbor,
    CsrNodeId destination)
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

    if (neighborIt->second.stale ||
        !IsArlNeighborUsable (neighbor))
      {
        std::cout << "[NWK " << m_nodeId
                  << "] RoutingDelete rejected"
                  << " unavailableNeighbor="
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
    CsrNodeId destination) const;

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
    CsrNodeId nwkDst {0};        // OPNET node_addr
    uint8_t  capability {0};    // OPNET capability
    bool     immediate {false}; // OPNET immediate neighbor flag
    CsrNodeId nextHop {0};       // OPNET next_hop

    double   pathlossDb {std::numeric_limits<double>::quiet_NaN ()};
    uint8_t  numHop {0};        // OPNET num_hop
    uint32_t cost {0};          // OPNET cost
    uint8_t  energyLevel {100}; // OPNET energy_level placeholder

    Time     lastUpdated {Seconds (0.0)};
    bool     valid {true};
    uint32_t linkCostToNextHop {0};
    uint32_t advertisedCost {0};
    CsrNodeId learnedFrom {
      CSR_BROADCAST_ID
    }; // node that taught us this route

    std::vector<CsrNodeId> path;

    // routes.c retains the originating routing-message sequence with every
    // neighbor route (including invalid DELETE/FLUSH tombstones).
    bool routingSequenceValid {false};
    uint32_t routingSequence {0};
  };

  struct SelectedRouteState
  {
    bool available {false};

    CsrNodeId nextHop {
      CSR_BROADCAST_ID
    };

    uint32_t cost {0};
    uint8_t numHop {0};
    bool immediate {false};
    uint8_t capability {0};

    std::vector<CsrNodeId> path;
  };

  const RouteEntry*
  FindBestRoute (
    CsrNodeId destination) const;

  SelectedRouteState
  CaptureSelectedRouteState (
    CsrNodeId destination) const;

  static bool
  SameSelectedRouteState (
    const SelectedRouteState &first,
    const SelectedRouteState &second);

  void
  MarkSelectedRouteChanged (
    CsrNodeId destination,
    const char *reason);

  void
  ReportPendingSelectedRouteChanges ();

  void
  TrySendAutomaticRouteUpdates ();

  struct ReverseRouteEntry
  {
    CsrNodeId netSrc {CSR_BROADCAST_ID};
    CsrNodeId reverseHop {CSR_BROADCAST_ID};
    Time lastUpdated {Seconds (0.0)};
    bool valid {false};
  };

  struct NwkQueueEntry
  {
    CsrNodeId   nwkSrc;
    CsrNodeId   nwkDst;
    uint8_t    dscp;
    bool       ack;
    Ptr<Packet> payload;
  };

  struct NwkNeighborEntry
  {
    struct ArlRoutingReassembly
    {
      uint8_t totalSections {0};
      Time firstReceived {Seconds (0.0)};
      std::map<uint8_t, std::vector<uint8_t>> sectionBodies;
    };

    CsrNodeId nodeId {0};
    double lastHeardSec {-1.0};
    double lastPathlossDb {std::numeric_limits<double>::quiet_NaN ()};
    double lastSnrDb {std::numeric_limits<double>::quiet_NaN ()};
    CsrRateKey speedKey {0};
    int16_t rxPowerDbmX10 {0};
    uint8_t activeNodes {0};
    bool wasActiveBeforeLastHello {false};

    // OPNET BrT_Neighbor_Entry::num_failures
    uint32_t numFailures {0};
    bool stale {false};

    // br_nwk stores SNMP_RELAY_HOLDOFF/CLEAR here, but its queue-release
    // routine never reads the field.  Preserve that observable no-op state.
    bool relayHoldoff {false};

    // Legacy routes.c admission is distinct from freshness.  A neighbor is
    // usable only after both group keys are exchanged and a NeighborCheck is
    // received/ACKed.
    bool arlActive {false};
    bool keySendActive {false};
    bool keySendComplete {false};
    bool keyUpdateComplete {false};

    bool keyRequestSentValid {false};
    Time keyRequestSentWhen {Seconds (0.0)};
    Time keyRequestDelay {MilliSeconds (5000)};

    bool keySendValid {false};
    Time keySendWhen {Seconds (0.0)};
    Time keySendDelay {MilliSeconds (5000)};

    bool overheardValid {false};
    Time overheardWhen {Seconds (0.0)};
    Time overheardDelay {MilliSeconds (5000)};

    bool checkMessageActive {false};
    bool admissionDiscoveryCheckPending {false};
    bool admissionDiscoveryCheckActive {false};
    uint32_t admissionDiscoverySequence {0};
    bool admissionNeedsRoutingRequest {false};
    EventId admissionRetryEvent;

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

    std::set<CsrNodeId>
      routingSnapshotSeenDestinations;

    // Complete routing UPDATE sections are buffered here
    // until the final section arrives. This mirrors the
    // legacy behavior of reconstructing the full routing
    // message before modifying the live route table.
    std::vector<CsrHelloHeader>
      routingSnapshotBufferedUpdates;

    // Legacy routesReceiveRouting() can hold several incomplete routing
    // sequences from one neighbor and orders their sections independently.
    std::map<uint32_t, ArlRoutingReassembly>
      arlRoutingReassemblies;

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

      };

  Ptr<Packet> BuildRoutingRequestPayload (
    uint32_t routingSequence);

  void SendRoutingRequestAttempt (
    CsrNodeId neighbor,
    bool retry);

  void RoutingRequestTimeout (
    CsrNodeId neighbor,
    uint32_t expectedSequence);

  // Legacy FLOW_CTRL_MAX_THRESHOLD. NSDP uses this only to choose
  // ACK versus DACK for relayed DATA; it does not gate NWK queue release.
  static constexpr uint32_t NSDP_DACK_THRESHOLD = 16;

  // NSDP: Network Source-Destination Pair entry
  struct NsdpEntry
  {
    CsrNodeId src;
    CsrNodeId dst;
    uint32_t count;
    uint32_t limit;  // DACK threshold, retained in metrics output
  };

  // Keyed by (src,dst)
  NsdpEntry& GetNsdpEntry (CsrNodeId src, CsrNodeId dst)
  {
    std::pair<CsrNodeId,CsrNodeId> key (src, dst);
    auto it = m_nsdp.find (key);
    if (it == m_nsdp.end ())
      {
        NsdpEntry e;
        e.src   = src;
        e.dst   = dst;
        e.count = 0;
        e.limit = NSDP_DACK_THRESHOLD;
        it = m_nsdp.insert (std::make_pair (key, e)).first;
      }
    return it->second;
  }

  void ScheduleCheckNwkQueue ()
  {
    if (!m_checkNwkQueueEvent.IsPending ())
      {
        m_checkNwkQueueEvent =
          Simulator::Schedule (
            Seconds (1.0 / 36.0e6),
            &CsrNetLayer::CheckNwkQueue,
            this);
      }
  }

  void CheckNwkQueue ()
  {
    if (m_hop == nullptr)
      {
        return;
      }

    std::cout << "[NWK " << m_nodeId
              << "] CheckNwkQueue: size="
              << m_nwkQueue.size ()
              << std::endl;

    // Legacy check_nwk_queue() scans the complete priority queue.
    // A packet with no route or a saturated next hop does not block
    // eligible traffic for another destination. NSDP is bookkeeping
    // for ACK/DACK selection, not a NWK-to-HOP admission gate.
    // BrT_Neighbor_Entry::relay_holdoff is likewise never consulted by the
    // supplied wrapper, so a stored relay control does not gate this scan.
    for (auto it = m_nwkQueue.begin ();
         it != m_nwkQueue.end ();)
      {
        if (!m_hop->CanAcceptDataGlobally ())
          {
            std::cout << "[NWK " << m_nodeId
                      << "] Global HOP DATA capacity exhausted;"
                      << " holding "
                      << m_nwkQueue.size ()
                      << " queued packets"
                      << std::endl;
            break;
          }

        CsrNodeId hopDest;
        if (!LookupNextHop (it->nwkDst, hopDest))
          {
            std::cout << "[NWK " << m_nodeId
                      << "] No route to nwkDst="
                      << it->nwkDst
                      << " -> holding packet without implicit discovery"
                      << std::endl;

            ++it;
            continue;
          }

        if (!m_hop->CanSendToHop (hopDest))
          {
            ++it;
            continue;
          }

        NwkQueueEntry entry = *it;
        it = m_nwkQueue.erase (it);

        m_hop->SendData (
          hopDest,
          entry.dscp,
          entry.payload,
          entry.ack);
      }
  }

  bool
  LookupNextHop (
    CsrNodeId nwkDst,
    CsrNodeId &nextHopOut)
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
        CsrNodeId reverseHop =
          reverseIt->second.reverseHop;

        auto neighborIt =
          m_nwkNeighbors.find (
            reverseHop);

        bool reverseNeighborActive =
          neighborIt !=
            m_nwkNeighbors.end () &&
          neighborIt->second.lastHeardSec >=
            0.0 &&
          !neighborIt->second.stale &&
          IsArlNeighborUsable (reverseHop);

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
                   double *totalMarginOut = nullptr) const
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
    CsrNodeId neighbor,
    uint32_t routingSequence,
    CsrRoutingOperation operation,
    uint8_t routingSection,
    uint8_t routingTotalSections,
    bool lastOfInfo)
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
	              << " lastOfInfo="
	              << (lastOfInfo ? 1 : 0)
	              << std::endl;

    // A partial destination ACK is progress for this section's grouped HOP
    // transaction.  Only lastOfInfo means every destination for the section
    // has ACKed.
    if (!lastOfInfo)
      {
        return;
      }

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

    if (!snapshot.active ||
        routingSequence != snapshot.routingSequence)
      {
        return;
      }

    if (routingTotalSections != snapshot.totalSections ||
        routingSection >= snapshot.totalSections)
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Ignoring mismatched ARL section ACK"
                  << " neighbor=" << neighbor
                  << " section="
                  << unsigned (routingSection)
                  << "/"
                  << unsigned (routingTotalSections)
                  << " expectedTotal="
                  << unsigned (snapshot.totalSections)
                  << std::endl;
        return;
      }

    snapshot.ackedSections.insert (routingSection);

    std::cout << "[NWK " << m_nodeId
              << "] ARL RoutingSnapshot section ACKed"
              << " neighbor=" << neighbor
              << " routingSequence=" << routingSequence
              << " section=" << unsigned (routingSection)
              << "/" << unsigned (snapshot.totalSections)
              << " ackedSections="
              << snapshot.ackedSections.size ()
              << std::endl;

    if (snapshot.ackedSections.size () ==
        snapshot.totalSections)
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
                  << "] Reliable ARL RoutingSnapshot completed"
                  << " neighbor=" << neighbor
                  << " routingSequence=" << routingSequence
                  << " sections="
                  << unsigned (snapshot.totalSections)
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
    std::vector<CsrNodeId> neighbors,
    uint32_t routingSequence,
    CsrRoutingOperation operation,
    uint8_t routingSection,
    uint8_t routingTotalSections,
    bool lastOfInfo)
  {
    std::cout << "[NWK " << m_nodeId
              << "] Reliable RoutingControl failure"
              << " neighbors=";
    for (uint32_t i = 0; i < neighbors.size (); ++i)
      {
        if (i > 0)
          {
            std::cout << ",";
          }
        std::cout << neighbors[i];
      }
    std::cout
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
              << " lastOfInfo="
              << (lastOfInfo ? 1 : 0)
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
    CsrNodeId neighbor)
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

    if (neighborIt->second.stale ||
        !IsArlNeighborUsable (neighbor))
      {
        std::cout << "[NWK " << m_nodeId
                  << "] Reliable RoutingUpdate rejected"
                  << " unavailableNeighbor="
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
    CsrNodeId neighbor,
    const char *phase);

  void
  RoutingSnapshotWatchdogExpired (
    CsrNodeId neighbor,
    uint32_t expectedGeneration);

private:
  CsrNodeId                              m_nodeId;
  Ptr<CsrHopLayer>                      m_hop;
  Callback<void, Ptr<Packet>, CsrNodeId> m_rxFromNetCb;

  std::deque<NwkQueueEntry>             m_nwkQueue;
  EventId                               m_checkNwkQueueEvent;
  std::vector<RouteEntry>               m_routes;
  std::map<CsrNodeId, ReverseRouteEntry> m_reverseRoutes;
  std::map<CsrNodeId, NwkNeighborEntry>  m_nwkNeighbors;
  std::map<std::pair<CsrNodeId,CsrNodeId>, NsdpEntry> m_nsdp;
  std::set<CsrNodeId>
    m_pendingSelectedRouteChanges;

  // Legacy routesFindBestRoute() prefers the
  // currently selected neighbor when cost and
  // hop count are exactly tied.
  std::map<CsrNodeId, CsrNodeId>
    m_selectedRoutePreferredNextHop;

  EventId m_selectedRouteChangeEvent;

  Time m_selectedRouteChangeDelay {
    MilliSeconds (25)
  };

  bool    m_discoveryActive { false };

  struct DiscoveryEntry
  {
    CsrNodeId nodeId {CSR_BROADCAST_ID};
    bool discoveryNeeded {false};
  };

  std::vector<DiscoveryEntry> m_discoveryTable;
  std::vector<CsrNodeId> m_discoveryCompletionRequesters;
  CsrNodeId m_discoveryInitiatedBy {CSR_BROADCAST_ID};

  EventId m_snmpReportEvent;
  Time m_snmpReportTimeout {Seconds (60.0)};

  uint8_t m_discoveryBroadcastsRemaining {0};
  bool m_repeatDiscoveryHello {true};
  bool m_discoveryCompletionQuietTickObserved {false};

  uint32_t m_discoveryStartCount {0};
  uint32_t m_discoveryBroadcastCount {0};
  uint32_t m_snmpStartSentCount {0};
  uint32_t m_snmpStartReceivedCount {0};
  uint32_t m_snmpDoneSentCount {0};
  uint32_t m_snmpDoneReceivedCount {0};
  uint32_t m_relayHoldoffSentCount {0};
  uint32_t m_relayHoldoffReceivedCount {0};
  uint32_t m_relayClearSentCount {0};
  uint32_t m_relayClearReceivedCount {0};

  bool m_discoveryResponseEnabled {true};

  uint32_t GetNeighborCount () const;
  uint32_t GetActiveNodeCount () const;

  CsrRateKey m_minSpeedKey {8};

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

  void ReceiveSnmpFromHop (
    Ptr<Packet> snmpPayload,
    CsrNodeId hopSource);

  bool SendSnmp (
    CsrNodeId destination,
    CsrSnmpCommand command,
    int32_t value,
    const std::vector<CsrNodeId> &nodes = {});

  void CompleteDiscoveryLifecycle ();
  void CheckDiscoveryTable ();
  void SnmpReportTimeout ();
  void EnsureDiscoveryEntry (CsrNodeId node, bool discoveryNeeded);
  void MarkDiscoveryNotNeeded (CsrNodeId node);
  std::vector<CsrNodeId> CollectKnownDiscoveryNodes () const;

  void SendHelloBroadcast (
    CsrArlRouteMsgType type = CsrArlRouteMsgType::Discover,
    CsrNeighborCheckType checkType = CsrNeighborCheckType::None,
    CsrDiscoverType discoverType = CsrDiscoverType::None,
    uint32_t discoverySequence = 0,
    uint32_t routingSequence = 0);

  std::set<CsrNodeId>
  ProcessRoutesPayload (
    const CsrHelloHeader &hh,
    CsrNodeId helloSrc,
    double pathlossDb,
    double snrDb,
    uint32_t linkCost);

  void ProcessArlRouteMessage (const CsrHelloHeader &hh,
                              Ptr<Packet> routingPayload,
                              CsrNodeId helloSrc,
                              double pathlossDb,
                              double snrDb,
                              uint32_t linkCost);

  void ProcessArlRoutingSection (
    const CsrHelloHeader &hh,
    Ptr<Packet> routingPayload,
    CsrNodeId helloSrc,
    double pathlossDb,
    double snrDb,
    uint32_t linkCost);

  void ApplyCompleteArlRoutingMessage (
    CsrNodeId helloSrc,
    uint32_t routingSequence,
    const std::vector<CsrArlRoutingMessage::Record> &records,
    double pathlossDb,
    double snrDb,
    uint32_t linkCost);

  void ProcessDiscover (const CsrHelloHeader &hh,
                        CsrNodeId helloSrc,
                        double pathlossDb,
                        double snrDb,
                        uint32_t linkCost);

  void ProcessRoutingUpdate (const CsrHelloHeader &hh,
                            CsrNodeId helloSrc,
                            double pathlossDb,
                            double snrDb,
                            uint32_t linkCost);

  void ProcessNeighborCheck (const CsrHelloHeader &hh,
                           CsrNodeId helloSrc,
                           double pathlossDb,
                           double snrDb,
                           uint32_t linkCost);

  bool IsArlNeighborUsable (CsrNodeId neighbor) const;
  bool AcceptFromNeighbor (CsrNodeId neighbor);
  void SyncNeighborKeyState (CsrNodeId neighbor);
  void EvaluateNeighborAdmission (
    CsrNodeId neighbor,
    bool receivedDiscovery = false);
  void AdmissionRetry (CsrNodeId neighbor);
  void ScheduleAdmissionRetry (CsrNodeId neighbor, Time delay);
  void SendKeyRequest (CsrNodeId neighbor, bool resetDelay);
  void SendKeyUpdate (CsrNodeId neighbor);
  void SendPendingDiscoveryCheck (CsrNodeId neighbor);
  void EnsureCheckMessage (CsrNodeId neighbor, const char *reason);
  void TryMakeNeighborActive (CsrNodeId neighbor, const char *reason);
  void MakeNeighborInactive (CsrNodeId neighbor, const char *reason);

  void NoteKeyRequestReceived (CsrNodeId neighbor);
  void NoteKeyUpdateReceived (CsrNodeId neighbor);
  void NoteKeyUpdateCompletion (CsrNodeId neighbor, bool acknowledged);
  void NoteSecurityCountChange (CsrNodeId neighbor);
  void NoteAuthenticatedGroupKeyNeeded (CsrNodeId neighbor,
                                        double pathlossDb,
                                        double snrDb);
  void NoteLocalGroupKeyChanged ();
  void NoteNeighborCheckFailure (
    CsrNodeId neighbor,
    CsrNeighborCheckType type,
    uint32_t discoverySequence);

  void UpdateReverseRoute (CsrNodeId netSrc, CsrNodeId hopSrc);
  bool RemoveReverseRouteFromReporter (CsrNodeId netSrc,
                                       CsrNodeId reporter);

  const char* ArlRouteMsgTypeName (CsrArlRouteMsgType t) const;

  bool ShouldAdvertiseRoute (const RouteEntry &re) const;

  void TryDrainQueueAfterDiscovery ();

  void ScheduleDiscoveryHello ();
  void DiscoveryHelloTick ();
  void VerifyUnresponsiveDiscoveryNeighbors ();

  void CheckNeighborFreshness ();
  void InvalidateRoutesViaNextHop (CsrNodeId nextHop, const char *reason);
  bool m_invalidateRoutesOnStaleNeighbor { false };

  bool m_arlNeighborAdmissionEnabled {true};
  uint32_t m_keyRequestSentCount {0};
  uint32_t m_keyUpdateSentCount {0};
  uint32_t m_keyUpdateReceivedCount {0};
  uint32_t m_inactiveNeighborDropCount {0};

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

  CsrNodeId m_gatewayNodeId {CSR_BROADCAST_ID};
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
    CsrNodeId destination,
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
    CsrNodeId routingTarget =
      CSR_BROADCAST_ID);

  std::vector<Ptr<Packet>>
  BuildArlRoutingSnapshotPayloads (
    uint32_t routingSequence);

  Ptr<Packet> BuildArlRoutingSectionPayload (
    const std::vector<uint8_t> &sectionBytes,
    uint32_t routingSequence,
    uint8_t routingSection,
    uint8_t routingTotalSections);

  struct OutboundRoutingSnapshot
  {
    bool active {false};

    uint32_t routingSequence {0};
    uint8_t totalSections {0};
    std::set<uint8_t> ackedSections;

    EventId watchdogEvent;

    uint32_t watchdogGeneration {0};

    const char *watchdogPhase {
      "inactive"
    };
  };

  std::map<CsrNodeId, OutboundRoutingSnapshot>
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

  // routesProcess() automatically sends each selected-route change to every
  // active neighbor.  Tests may disable that behavior when isolating a single
  // transaction, but the production default follows the ARL implementation.
  bool m_automaticRoutePropagationEnabled {
    true
  };

  // Changed destinations waiting to be advertised,
  // grouped by receiving neighbor.
  std::map<CsrNodeId, std::set<CsrNodeId>>
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
AppRxFromNet (Ptr<Packet> payload, CsrNodeId src)
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
        m_discState == DiscoveryState::ACTIVE)
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

  m_discoveryInitiatedBy = m_nodeId;

  StartDiscovery (
    Seconds (0.0),
    m_gatewayStartupDiscoveryDuration);
}

void
CsrNetLayer::SetArlNeighborAdmissionEnabled (bool enable)
{
  if (m_arlNeighborAdmissionEnabled == enable)
    {
      return;
    }

  m_arlNeighborAdmissionEnabled = enable;

  for (auto &entry : m_nwkNeighbors)
    {
      NwkNeighborEntry &neighbor = entry.second;

      if (!enable)
        {
          neighbor.arlActive =
            neighbor.lastHeardSec >= 0.0 && !neighbor.stale;
        }
      else
        {
          neighbor.arlActive = false;
          SyncNeighborKeyState (neighbor.nodeId);
          EvaluateNeighborAdmission (neighbor.nodeId, false);
        }
    }

  std::cout << "[NWK " << m_nodeId
            << "] ARL neighbor admission="
            << (enable ? "enabled" : "disabled")
            << std::endl;

  ScheduleCheckNwkQueue ();
}

bool
CsrNetLayer::IsArlNeighborUsable (CsrNodeId neighbor) const
{
  if (!m_arlNeighborAdmissionEnabled)
    {
      return true;
    }

  auto it = m_nwkNeighbors.find (neighbor);
  return it != m_nwkNeighbors.end () &&
         it->second.arlActive &&
         !it->second.stale;
}

void
CsrNetLayer::SyncNeighborKeyState (CsrNodeId neighbor)
{
  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end () || m_hop == nullptr)
    {
      return;
    }

  NwkNeighborEntry &entry = it->second;
  entry.keySendActive = m_hop->IsKeyUpdateSendActive (neighbor);
  entry.keySendComplete = m_hop->HasGroupKeySentTo (neighbor);
  entry.keyUpdateComplete = m_hop->HasGroupKeyReceivedFrom (neighbor);
}

void
CsrNetLayer::ScheduleAdmissionRetry (
  CsrNodeId neighbor,
  Time delay)
{
  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end () || it->second.arlActive)
    {
      return;
    }

  // There is one legacy-admission deadline per neighbor.  A new packet or a
  // transition to the next key/check phase replaces the obsolete deadline.
  if (it->second.admissionRetryEvent.IsPending ())
    {
      Simulator::Cancel (it->second.admissionRetryEvent);
    }

  it->second.admissionRetryEvent = Simulator::Schedule (
    delay,
    &CsrNetLayer::AdmissionRetry,
    this,
    neighbor);
}

void
CsrNetLayer::AdmissionRetry (CsrNodeId neighbor)
{
  EvaluateNeighborAdmission (neighbor, false);
}

void
CsrNetLayer::SendKeyRequest (
  CsrNodeId neighbor,
  bool resetDelay)
{
  if (m_hop == nullptr)
    {
      return;
    }

  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  entry.nodeId = neighbor;

  if (resetDelay)
    {
      entry.keyRequestDelay = MilliSeconds (5000);
      entry.keySendDelay = MilliSeconds (5000);
    }

  entry.keyRequestSentValid = true;
  entry.keyRequestSentWhen = Simulator::Now ();
  m_keyRequestSentCount++;

  std::cout << "[NWK " << m_nodeId
            << "] Sending source-faithful KeyRequest"
            << " neighbor=" << neighbor
            << " noAck=1"
            << " retryDelayMs="
            << entry.keyRequestDelay.GetMilliSeconds ()
            << std::endl;

  m_hop->SendKeyRequest (neighbor);
  ScheduleAdmissionRetry (neighbor, entry.keyRequestDelay);
}

void
CsrNetLayer::SendKeyUpdate (CsrNodeId neighbor)
{
  if (m_hop == nullptr)
    {
      return;
    }

  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  entry.nodeId = neighbor;
  SyncNeighborKeyState (neighbor);

  if (entry.keySendComplete || entry.keySendActive)
    {
      return;
    }

  entry.keySendValid = true;
  entry.keySendWhen = Simulator::Now ();

  if (m_hop->SendKeyUpdate (neighbor))
    {
      entry.keySendActive = true;
      m_keyUpdateSentCount++;

      std::cout << "[NWK " << m_nodeId
                << "] KeyUpdate send active"
                << " neighbor=" << neighbor
                << " reliable=1"
                << " securityBytes="
                << CsrKeyUpdateHeader::SERIALIZED_SIZE
                << std::endl;

      ScheduleAdmissionRetry (neighbor, entry.keySendDelay);
    }
}

void
CsrNetLayer::SendPendingDiscoveryCheck (CsrNodeId neighbor)
{
  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &entry = it->second;

  if (!entry.admissionDiscoveryCheckPending ||
      entry.admissionDiscoveryCheckActive ||
      !entry.keySendComplete ||
      !m_discoveryResponseEnabled)
    {
      return;
    }

  entry.admissionDiscoveryCheckPending = false;
  entry.admissionDiscoveryCheckActive = true;

  std::cout << "[NWK " << m_nodeId
            << "] Sending deferred Discovery NeighborCheck"
            << " neighbor=" << neighbor
            << " sequence=" << entry.admissionDiscoverySequence
            << " afterKeySend=1"
            << std::endl;

  SendNeighborCheck (neighbor,
                     CsrNeighborCheckType::Discovery,
                     CSR_BROADCAST_ID,
                     entry.admissionDiscoverySequence);
}

void
CsrNetLayer::EnsureCheckMessage (
  CsrNodeId neighbor,
  const char *reason)
{
  if (!m_arlNeighborAdmissionEnabled || m_hop == nullptr)
    {
      return;
    }

  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  entry.nodeId = neighbor;

  if (entry.arlActive ||
      entry.checkMessageActive ||
      entry.admissionDiscoveryCheckActive)
    {
      return;
    }

  entry.checkMessageActive = true;

  std::cout << "[NWK " << m_nodeId
            << "] Starting CHECK_MESSAGE admission proof"
            << " neighbor=" << neighbor
            << " reason=" << reason
            << std::endl;

  SendNeighborCheck (neighbor, CsrNeighborCheckType::Message);
}

void
CsrNetLayer::TryMakeNeighborActive (
  CsrNodeId neighbor,
  const char *reason)
{
  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &entry = it->second;
  SyncNeighborKeyState (neighbor);

  if (m_arlNeighborAdmissionEnabled &&
      (!entry.keyUpdateComplete || !entry.keySendComplete))
    {
      std::cout << "[NWK " << m_nodeId
                << "] Neighbor admission deferred"
                << " neighbor=" << neighbor
                << " reason=" << reason
                << " receivedKey="
                << (entry.keyUpdateComplete ? 1 : 0)
                << " sentKey="
                << (entry.keySendComplete ? 1 : 0)
                << std::endl;
      return;
    }

  if (entry.arlActive)
    {
      return;
    }

  std::map<CsrNodeId, SelectedRouteState> selectedBefore;
  for (const auto &route : m_routes)
    {
      if (route.nextHop == neighbor &&
          selectedBefore.find (route.nwkDst) == selectedBefore.end ())
        {
          selectedBefore.emplace (
            route.nwkDst,
            CaptureSelectedRouteState (route.nwkDst));
        }
    }

  entry.arlActive = true;
  entry.stale = false;
  entry.overheardDelay = MilliSeconds (5000);
  entry.checkMessageActive = false;
  entry.admissionDiscoveryCheckActive = false;

  if (entry.admissionRetryEvent.IsPending ())
    {
      Simulator::Cancel (entry.admissionRetryEvent);
    }

  std::cout << "[NWK " << m_nodeId
            << "] ARL neighbor ACTIVE"
            << " neighbor=" << neighbor
            << " reason=" << reason
            << " keys=two-sided"
            << std::endl;

  for (const auto &before : selectedBefore)
    {
      SelectedRouteState after =
        CaptureSelectedRouteState (before.first);

      if (!SameSelectedRouteState (before.second, after))
        {
          MarkSelectedRouteChanged (before.first, "neighbor admitted");
        }
    }

  // routesMakeNeighborActive() sets needsUpdate.  A complete reliable ARL
  // snapshot is this model's INFO/UPDATE/FLUSH equivalent of that flag.
  Simulator::Schedule (
    MilliSeconds (20),
    &CsrNetLayer::StartReliableRoutingSnapshot,
    this,
    neighbor);

  if (entry.admissionNeedsRoutingRequest)
    {
      entry.admissionNeedsRoutingRequest = false;
      Simulator::ScheduleNow (
        &CsrNetLayer::SendRoutingRequest,
        this,
        neighbor);
    }

  ScheduleCheckNwkQueue ();
}

void
CsrNetLayer::MakeNeighborInactive (
  CsrNodeId neighbor,
  const char *reason)
{
  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end () || !it->second.arlActive)
    {
      return;
    }

  std::map<CsrNodeId, SelectedRouteState> selectedBefore;
  for (const auto &route : m_routes)
    {
      if (route.nextHop == neighbor &&
          selectedBefore.find (route.nwkDst) == selectedBefore.end ())
        {
          selectedBefore.emplace (
            route.nwkDst,
            CaptureSelectedRouteState (route.nwkDst));
        }
    }

  NwkNeighborEntry &entry = it->second;
  entry.arlActive = false;
  entry.checkMessageActive = false;
  entry.admissionDiscoveryCheckActive = false;

  std::cout << "[NWK " << m_nodeId
            << "] ARL neighbor INACTIVE"
            << " neighbor=" << neighbor
            << " reason=" << reason
            << std::endl;

  for (const auto &before : selectedBefore)
    {
      SelectedRouteState after =
        CaptureSelectedRouteState (before.first);

      if (!SameSelectedRouteState (before.second, after))
        {
          MarkSelectedRouteChanged (before.first, "neighbor inactive");
        }
    }
}

void
CsrNetLayer::EvaluateNeighborAdmission (
  CsrNodeId neighbor,
  bool receivedDiscovery)
{
  if (!m_arlNeighborAdmissionEnabled || m_hop == nullptr)
    {
      return;
    }

  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end ())
    {
      return;
    }

  SyncNeighborKeyState (neighbor);
  NwkNeighborEntry &entry = it->second;

  if (entry.stale || entry.arlActive)
    {
      return;
    }

  Time now = Simulator::Now ();

  if (!entry.keyUpdateComplete)
    {
      if (receivedDiscovery)
        {
          // routesRcvUnAuthedMsg() resets both delays to 5000 ms and sends a
          // new no-ACK KeyRequest for the authenticated-but-not-decryptable
          // Discover.
          SendKeyRequest (neighbor, true);
        }
      else if (!entry.keyRequestSentValid ||
               now - entry.keyRequestSentWhen >= entry.keyRequestDelay)
        {
          // routesValidRx() doubles the next eligibility interval before it
          // emits a retry.  routesRcvUnAuthedMsg() is the special initial
          // Discover path above and keeps the reset 5000-ms interval.
          entry.keyRequestDelay = entry.keyRequestDelay * 2;
          SendKeyRequest (neighbor, false);
        }
      return;
    }

  if (!entry.keySendComplete)
    {
      if (!entry.keySendActive &&
          (!entry.keySendValid ||
           now - entry.keySendWhen >= entry.keySendDelay))
        {
          if (entry.keySendValid)
            {
              entry.keySendDelay = entry.keySendDelay * 2;
            }
          SendKeyUpdate (neighbor);
        }
      return;
    }

  if (entry.admissionDiscoveryCheckPending)
    {
      SendPendingDiscoveryCheck (neighbor);
      return;
    }

  if (!entry.overheardValid ||
      now - entry.overheardWhen >= entry.overheardDelay)
    {
      entry.overheardValid = true;
      entry.overheardWhen = now;
      entry.overheardDelay = entry.overheardDelay * 2;

      std::cout << "[NWK " << m_nodeId
                << "] Sending CHECK_OVERHEARD after two-sided key exchange"
                << " neighbor=" << neighbor
                << std::endl;

      SendNeighborCheck (neighbor, CsrNeighborCheckType::Overheard);
      ScheduleAdmissionRetry (neighbor, entry.overheardDelay);
    }
}

bool
CsrNetLayer::AcceptFromNeighbor (CsrNodeId neighbor)
{
  if (!m_arlNeighborAdmissionEnabled)
    {
      return true;
    }

  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  entry.nodeId = neighbor;
  SyncNeighborKeyState (neighbor);

  if (!entry.arlActive)
    {
      EnsureCheckMessage (neighbor, "DATA from inactive neighbor");
      EvaluateNeighborAdmission (neighbor, false);
    }

  return entry.arlActive;
}

void
CsrNetLayer::NoteKeyRequestReceived (CsrNodeId neighbor)
{
  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  entry.nodeId = neighbor;
  SyncNeighborKeyState (neighbor);

  Time sentWhen;
  if (m_hop != nullptr &&
      m_hop->GetGroupKeySentWhen (neighbor, sentWhen))
    {
      Time age = Simulator::Now () - sentWhen;

      std::cout << "[NWK " << m_nodeId
                << "] KeyRequest received after completed KeyUpdate"
                << " neighbor=" << neighbor
                << " ageMs=" << age.GetMilliSeconds ()
                << (age > MilliSeconds (5000)
                      ? " needs-mission-key-check"
                      : " recent-key-suppressed")
                << std::endl;
      return;
    }

  std::cout << "[NWK " << m_nodeId
            << "] KeyRequest accepted"
            << " neighbor=" << neighbor
            << std::endl;

  SendKeyUpdate (neighbor);
}

void
CsrNetLayer::NoteKeyUpdateReceived (CsrNodeId neighbor)
{
  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  entry.nodeId = neighbor;
  SyncNeighborKeyState (neighbor);
  m_keyUpdateReceivedCount++;

  std::cout << "[NWK " << m_nodeId
            << "] KeyUpdate complete"
            << " neighbor=" << neighbor
            << " receivedKey="
            << (entry.keyUpdateComplete ? 1 : 0)
            << std::endl;

  // routesKeyUpdateComplete() immediately sends our reciprocal group key.
  SendKeyUpdate (neighbor);
  EvaluateNeighborAdmission (neighbor, false);
}

void
CsrNetLayer::NoteKeyUpdateCompletion (
  CsrNodeId neighbor,
  bool acknowledged)
{
  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &entry = it->second;
  entry.keySendActive = false;
  SyncNeighborKeyState (neighbor);

  std::cout << "[NWK " << m_nodeId
            << "] KeyUpdate send completion"
            << " neighbor=" << neighbor
            << " acknowledged=" << (acknowledged ? 1 : 0)
            << " sentKey=" << (entry.keySendComplete ? 1 : 0)
            << std::endl;

  EvaluateNeighborAdmission (neighbor, false);
}

void
CsrNetLayer::NoteSecurityCountChange (CsrNodeId neighbor)
{
  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  entry.nodeId = neighbor;

  MakeNeighborInactive (neighbor, "security count changed");

  entry.discoverySequenceValid = false;
  entry.keyUpdateComplete = false;
  entry.keyRequestSentValid = false;
  entry.keySendComplete = false;
  entry.keySendActive = false;
  entry.keySendValid = false;
  entry.overheardValid = false;
  entry.keyRequestDelay = MilliSeconds (5000);
  entry.keySendDelay = MilliSeconds (5000);
  entry.overheardDelay = MilliSeconds (5000);
  entry.numFailures = 0;

  if (entry.admissionRetryEvent.IsPending ())
    {
      Simulator::Cancel (entry.admissionRetryEvent);
    }

  std::cout << "[NWK " << m_nodeId
            << "] Reset ARL admission after security-count change"
            << " neighbor=" << neighbor
            << std::endl;
}

void
CsrNetLayer::NoteAuthenticatedGroupKeyNeeded (
  CsrNodeId neighbor,
  double pathlossDb,
  double snrDb)
{
  NwkNeighborEntry &entry = m_nwkNeighbors[neighbor];
  bool isNew = entry.lastHeardSec < 0.0;
  entry.nodeId = neighbor;
  entry.lastHeardSec = Simulator::Now ().GetSeconds ();
  entry.lastPathlossDb = pathlossDb;
  entry.lastSnrDb = snrDb;
  entry.stale = false;
  SyncNeighborKeyState (neighbor);
  UpdateMacActiveNodes ();

  std::cout << "[NWK " << m_nodeId
            << "] Authenticated Discover awaits group key"
            << " neighbor=" << neighbor
            << " newNeighbor=" << (isNew ? 1 : 0)
            << std::endl;

  // routesRcvUnAuthedMsg() sends a fresh KeyRequest for every authenticated
  // Discover whose payload cannot yet be decrypted, including active peers
  // that crossed a new group-key epoch.
  SendKeyRequest (neighbor, true);
}

void
CsrNetLayer::NoteLocalGroupKeyChanged ()
{
  std::cout << "[NWK " << m_nodeId
            << "] Local group-key epoch changed"
            << std::endl;

  for (auto &item : m_nwkNeighbors)
    {
      NwkNeighborEntry &entry = item.second;
      SyncNeighborKeyState (entry.nodeId);
      entry.keySendComplete = false;
      entry.keySendValid = false;

      bool usable = m_arlNeighborAdmissionEnabled
        ? entry.arlActive && !entry.stale
        : entry.lastHeardSec >= 0.0 && !entry.stale;
      if (usable && !entry.keySendActive)
        {
          SendKeyUpdate (entry.nodeId);
        }
    }
}

void
CsrNetLayer::NoteNeighborCheckFailure (
  CsrNodeId neighbor,
  CsrNeighborCheckType type,
  uint32_t discoverySequence)
{
  auto it = m_nwkNeighbors.find (neighbor);
  if (it == m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &entry = it->second;
  entry.checkMessageActive = false;

  if (type == CsrNeighborCheckType::Discovery)
    {
      entry.admissionDiscoveryCheckActive = false;
      entry.admissionDiscoveryCheckPending = true;
      entry.admissionDiscoverySequence = discoverySequence;
    }

  std::cout << "[NWK " << m_nodeId
            << "] NeighborCheck admission proof failed"
            << " neighbor=" << neighbor
            << " subtype=" << NeighborCheckTypeName (type)
            << std::endl;

  ScheduleAdmissionRetry (neighbor, MilliSeconds (5000));
}

void
CsrNetLayer::ProcessHello (Ptr<Packet> helloPayload,
                            CsrNodeId hopSrc,
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

  CsrNodeId src = hh.GetNodeId ();

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
    m_arlNeighborAdmissionEnabled
      ? ne.arlActive
      : (!isNew && !wasStale);

  ne.nodeId = src;
  ne.lastHeardSec = now;
  ne.lastPathlossDb = pathlossDb;
  ne.lastSnrDb = snrDb;
  ne.speedKey = hh.GetSpeedKey ();
  ne.rxPowerDbmX10 = hh.GetRxPowerDbmX10 ();
  ne.activeNodes = hh.GetActiveNodes ();
  ne.stale = false;

  if (!m_arlNeighborAdmissionEnabled)
    {
      ne.arlActive = true;
    }

  SyncNeighborKeyState (src);

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
      &totalMargin);

  uint32_t linkCost =
    localDirectionalCost;

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
              << " costMode=local-only"
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
                  static_cast<uint8_t> (senderType));

    // ------------------------------------------------------------
    // 3) Advertised route from HELLO sender
    //    If src advertises dst=X, then we can reach X via src.
    // ------------------------------------------------------------

  ProcessArlRouteMessage (hh,
                        helloPayload,
                        src,
                        pathlossDb,
                        snrDb,
                        linkCost);

  EvaluateNeighborAdmission (
    src,
    hh.GetArlRouteMsgType () == CsrArlRouteMsgType::Discover &&
      !ne.keyUpdateComplete);

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
                                      Ptr<Packet> routingPayload,
                                      CsrNodeId helloSrc,
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
        if (m_arlNeighborAdmissionEnabled &&
            !IsArlNeighborUsable (helloSrc))
          {
            // routesRcvMsg() still processes and ACKs an update from an
            // inactive neighbor, but starts CHECK_MESSAGE in parallel.
            EnsureCheckMessage (
              helloSrc,
              "RoutingUpdate from inactive neighbor");
          }

        if (routingPayload != nullptr &&
            routingPayload->GetSize () >=
              CsrArlRoutingMessage::SECTION_PREFIX_SIZE)
          {
            ProcessArlRoutingSection (
              hh,
              routingPayload,
              helloSrc,
              pathlossDb,
              snrDb,
              linkCost);
          }
        else
          {
            // Retain the old CsrHelloHeader representation for focused
            // compatibility scenarios and targeted route changes while full
            // snapshots use the routes.c byte stream.
            ProcessRoutingUpdate (
              hh,
              helloSrc,
              pathlossDb,
              snrDb,
              linkCost);
          }
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
        // Compatibility path for older scenarios that embedded KeyRequest in
        // CsrHelloHeader.  New traffic uses the exact Pairwise32 HOP-security
        // record and reaches the same handler through CsrHopLayer.
        NoteKeyRequestReceived (helloSrc);
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
CsrNetLayer::ProcessArlRoutingSection (
  const CsrHelloHeader &hh,
  Ptr<Packet> routingPayload,
  CsrNodeId helloSrc,
  double pathlossDb,
  double snrDb,
  uint32_t linkCost)
{
  std::vector<uint8_t> bytes (routingPayload->GetSize ());
  routingPayload->CopyData (bytes.data (), bytes.size ());

  CsrArlRoutingMessage::Section section;
  std::string error;

  if (!CsrArlRoutingMessage::DecodeSection (
        bytes,
        section,
        &error))
    {
      std::cout << "[NWK " << m_nodeId
                << "] Rejecting malformed ARL routing section"
                << " from=" << helloSrc
                << " reason=" << error
                << std::endl;
      return;
    }

  // The CsrHelloHeader is an ns-3/HOP envelope.  The routes.c prefix remains
  // authoritative, but disagreement means the frame was assembled wrongly.
  if (hh.GetRoutingSequence () != section.sequence ||
      hh.GetRoutingSection () != section.section ||
      hh.GetRoutingTotalSections () != section.totalSections)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Rejecting ARL routing envelope/prefix mismatch"
                << " from=" << helloSrc
                << " prefixSequence=" << section.sequence
                << " envelopeSequence=" << hh.GetRoutingSequence ()
                << " prefixSection=" << unsigned (section.section)
                << "/" << unsigned (section.totalSections)
                << " envelopeSection="
                << unsigned (hh.GetRoutingSection ())
                << "/"
                << unsigned (hh.GetRoutingTotalSections ())
                << std::endl;
      return;
    }

  auto neighborIt = m_nwkNeighbors.find (helloSrc);
  if (neighborIt == m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &neighbor = neighborIt->second;
  NwkNeighborEntry::ArlRoutingReassembly &message =
    neighbor.arlRoutingReassemblies[section.sequence];

  if (message.totalSections == 0)
    {
      message.totalSections = section.totalSections;
      message.firstReceived = Simulator::Now ();
    }
  else if (message.totalSections != section.totalSections)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Rejecting ARL section with changed total"
                << " from=" << helloSrc
                << " routingSequence=" << section.sequence
                << " expectedTotal="
                << unsigned (message.totalSections)
                << " receivedTotal="
                << unsigned (section.totalSections)
                << std::endl;
      return;
    }

  auto existing = message.sectionBodies.find (section.section);
  if (existing != message.sectionBodies.end ())
    {
      bool identical = existing->second == section.body;
      std::cout << "[NWK " << m_nodeId
                << "] Ignoring "
                << (identical ? "duplicate" : "conflicting")
                << " ARL routing section"
                << " from=" << helloSrc
                << " routingSequence=" << section.sequence
                << " section=" << unsigned (section.section)
                << std::endl;
      return;
    }

  message.sectionBodies.emplace (
    section.section,
    std::move (section.body));

  std::cout << "[NWK " << m_nodeId
            << "] Buffered ARL routing section"
            << " from=" << helloSrc
            << " routingSequence=" << section.sequence
            << " section=" << unsigned (section.section)
            << "/" << unsigned (section.totalSections)
            << " buffered=" << message.sectionBodies.size ()
            << std::endl;

  if (message.sectionBodies.size () !=
      message.totalSections)
    {
      return;
    }

  std::vector<uint8_t> recordStream;
  for (uint32_t index = 0;
       index < message.totalSections;
       ++index)
    {
      auto bodyIt = message.sectionBodies.find (
        static_cast<uint8_t> (index));

      if (bodyIt == message.sectionBodies.end ())
        {
          return;
        }

      recordStream.insert (
        recordStream.end (),
        bodyIt->second.begin (),
        bodyIt->second.end ());
    }

  std::vector<CsrArlRoutingMessage::Record> records;
  if (!CsrArlRoutingMessage::ParseRecordStream (
        recordStream,
        records,
        &error))
    {
      std::cout << "[NWK " << m_nodeId
                << "] Rejecting complete malformed ARL routing message"
                << " from=" << helloSrc
                << " routingSequence=" << section.sequence
                << " reason=" << error
                << std::endl;
      neighbor.arlRoutingReassemblies.erase (
        section.sequence);
      return;
    }

  // Remove the reassembly state before applying records.  No route or INFO
  // state was touched while any section was missing.
  neighbor.arlRoutingReassemblies.erase (section.sequence);

  std::cout << "[NWK " << m_nodeId
            << "] Reassembled complete ARL routing message"
            << " from=" << helloSrc
            << " routingSequence=" << section.sequence
            << " sections=" << unsigned (section.totalSections)
            << " recordBytes=" << recordStream.size ()
            << " records=" << records.size ()
            << std::endl;

  ApplyCompleteArlRoutingMessage (
    helloSrc,
    section.sequence,
    records,
    pathlossDb,
    snrDb,
    linkCost);
}

void
CsrNetLayer::ApplyCompleteArlRoutingMessage (
  CsrNodeId helloSrc,
  uint32_t routingSequence,
  const std::vector<CsrArlRoutingMessage::Record> &records,
  double pathlossDb,
  double snrDb,
  uint32_t linkCost)
{
  (void) snrDb;

  auto neighborIt = m_nwkNeighbors.find (helloSrc);
  if (neighborIt == m_nwkNeighbors.end ())
    {
      return;
    }

  NwkNeighborEntry &neighbor = neighborIt->second;
  bool sawFlush = false;

  auto isIncomingNewer = [routingSequence] (
    bool currentValid,
    uint32_t currentSequence) {
      return !currentValid ||
        CompareRoutingSequence (
          currentSequence,
          routingSequence) < 0;
    };

  for (const auto &record : records)
    {
      switch (record.operation)
        {
        case CsrRoutingOperation::Request:
          std::cout << "[NWK " << m_nodeId
                    << "] Applying ARL REQUEST"
                    << " from=" << helloSrc
                    << " routingSequence=" << routingSequence
                    << std::endl;

          if (m_routingSnapshotResponseEnabled)
            {
              Simulator::Schedule (
                MilliSeconds (20),
                &CsrNetLayer::StartReliableRoutingSnapshot,
                this,
                helloSrc);
            }
          break;

        case CsrRoutingOperation::Info:
          if (isIncomingNewer (
                neighbor.routingInfoValid,
                neighbor.routingInfoSequence))
            {
              neighbor.routingInfoValid = true;
              neighbor.routingInfoSequence = routingSequence;
              neighbor.remoteMinSpeedKbps =
                record.info.minSpeedKbps;
              neighbor.remoteMaxSpeedKbps =
                record.info.maxSpeedKbps;
              neighbor.remoteMinPowerDbmX10 =
                record.info.minPowerDbmX10;
              neighbor.remoteMaxPowerDbmX10 =
                record.info.maxPowerDbmX10;
              neighbor.remoteLinkMarginDbX10 =
                record.info.linkMarginDbX10;
              neighbor.remoteLowPowerDbmX10 =
                record.info.lowPowerDbmX10;
              neighbor.remoteTempLowCx10 =
                record.info.tempLowCx10;
              neighbor.remoteTempHighCx10 =
                record.info.tempHighCx10;

              std::cout << "[NWK " << m_nodeId
                        << "] Applied ARL INFO"
                        << " from=" << helloSrc
                        << " routingSequence=" << routingSequence
                        << std::endl;
            }
          break;

        case CsrRoutingOperation::Update:
          {
            if (record.nodeId >= CSR_BROADCAST_ID ||
                record.nodeId == m_nodeId ||
                record.nodeId == helloSrc)
              {
                std::cout << "[NWK " << m_nodeId
                          << "] Ignoring unsupported ARL UPDATE node"
                          << " node24=" << record.nodeId
                          << " from=" << helloSrc
                          << std::endl;
                break;
              }

            if (record.hopCount >= CSR_MAX_ROUTE_PATH_HOPS)
              {
                std::cout << "[NWK " << m_nodeId
                          << "] Ignoring ARL UPDATE beyond model path limit"
                          << " dst=" << record.nodeId
                          << " hopCount=" << record.hopCount
                          << std::endl;
                break;
              }

            bool pathSupported = true;
            bool containsLocalNode = false;
            std::vector<CsrNodeId> advertisedPath;
            advertisedPath.reserve (record.path.size ());

            for (CsrNodeId pathNode : record.path)
              {
                if (pathNode >= CSR_BROADCAST_ID)
                  {
                    pathSupported = false;
                    break;
                  }

                CsrNodeId node = pathNode;
                advertisedPath.push_back (node);
                if (node == m_nodeId)
                  {
                    containsLocalNode = true;
                  }
              }

            if (!pathSupported)
              {
                std::cout << "[NWK " << m_nodeId
                          << "] Ignoring ARL UPDATE with unsupported 24-bit path"
                          << " dst=" << record.nodeId
                          << std::endl;
                break;
              }

            CsrNodeId destination = record.nodeId;

            RouteEntry *existingRoute = nullptr;
            for (auto &route : m_routes)
              {
                if (!route.immediate &&
                    route.nwkDst == destination &&
                    route.learnedFrom == helloSrc)
                  {
                    existingRoute = &route;
                    break;
                  }
              }

            if (existingRoute != nullptr &&
                !isIncomingNewer (
                  existingRoute->routingSequenceValid,
                  existingRoute->routingSequence))
              {
                break;
              }

            if (containsLocalNode)
              {
                SelectedRouteState before =
                  CaptureSelectedRouteState (destination);

                if (existingRoute == nullptr)
                  {
                    RouteEntry tombstone;
                    tombstone.nwkDst = destination;
                    tombstone.nextHop = helloSrc;
                    tombstone.learnedFrom = helloSrc;
                    tombstone.capability = record.capability;
                    tombstone.numHop = static_cast<uint8_t> (
                      record.hopCount + 1);
                    tombstone.advertisedCost = record.cost;
                    tombstone.path = advertisedPath;
                    tombstone.lastUpdated = Simulator::Now ();
                    tombstone.valid = false;
                    tombstone.routingSequenceValid = true;
                    tombstone.routingSequence = routingSequence;
                    m_routes.push_back (std::move (tombstone));
                  }
                else
                  {
                    existingRoute->valid = false;
                    existingRoute->lastUpdated = Simulator::Now ();
                    existingRoute->routingSequenceValid = true;
                    existingRoute->routingSequence = routingSequence;
                  }

                SelectedRouteState after =
                  CaptureSelectedRouteState (destination);
                if (!SameSelectedRouteState (before, after))
                  {
                    MarkSelectedRouteChanged (
                      destination,
                      "ARL looped UPDATE");
                  }
                break;
              }

            std::vector<CsrNodeId> candidatePath;
            candidatePath.reserve (advertisedPath.size () + 1);
            candidatePath.push_back (helloSrc);
            candidatePath.insert (
              candidatePath.end (),
              advertisedPath.begin (),
              advertisedPath.end ());

            AddOrUpdateRoute (
              destination,
              helloSrc,
              false,
              static_cast<uint8_t> (record.hopCount + 1),
              pathlossDb,
              linkCost,
              record.cost,
              helloSrc,
              record.capability,
              candidatePath);

            for (auto &route : m_routes)
              {
                if (!route.immediate &&
                    route.nwkDst == destination &&
                    route.learnedFrom == helloSrc)
                  {
                    route.routingSequenceValid = true;
                    route.routingSequence = routingSequence;
                    break;
                  }
              }

            std::cout << "[NWK " << m_nodeId
                      << "] Applied ARL UPDATE"
                      << " from=" << helloSrc
                      << " dst=" << destination
                      << " advertisedHops=" << record.hopCount
                      << " advertisedCost=" << record.cost
                      << " routingSequence=" << routingSequence
                      << std::endl;
            break;
          }

        case CsrRoutingOperation::Delete:
          {
            if (record.nodeId >= CSR_BROADCAST_ID ||
                record.nodeId == m_nodeId)
              {
                break;
              }

            CsrNodeId destination = record.nodeId;
            SelectedRouteState before =
              CaptureSelectedRouteState (destination);
            RouteEntry *matchingRoute = nullptr;

            for (auto &route : m_routes)
              {
                if (!route.immediate &&
                    route.nwkDst == destination &&
                    route.learnedFrom == helloSrc)
                  {
                    matchingRoute = &route;
                    break;
                  }
              }

            if (matchingRoute == nullptr)
              {
                RouteEntry tombstone;
                tombstone.nwkDst = destination;
                tombstone.nextHop = helloSrc;
                tombstone.learnedFrom = helloSrc;
                tombstone.lastUpdated = Simulator::Now ();
                tombstone.valid = false;
                tombstone.routingSequenceValid = true;
                tombstone.routingSequence = routingSequence;
                m_routes.push_back (std::move (tombstone));
              }
            else if (isIncomingNewer (
                       matchingRoute->routingSequenceValid,
                       matchingRoute->routingSequence))
              {
                matchingRoute->valid = false;
                matchingRoute->lastUpdated = Simulator::Now ();
                matchingRoute->routingSequenceValid = true;
                matchingRoute->routingSequence = routingSequence;
              }

            SelectedRouteState after =
              CaptureSelectedRouteState (destination);
            if (!SameSelectedRouteState (before, after))
              {
                MarkSelectedRouteChanged (
                  destination,
                  "ARL DELETE");
              }

            std::cout << "[NWK " << m_nodeId
                      << "] Applied ARL DELETE"
                      << " from=" << helloSrc
                      << " dst=" << destination
                      << " routingSequence=" << routingSequence
                      << std::endl;
            break;
          }

        case CsrRoutingOperation::Flush:
          {
            sawFlush = true;
            std::map<CsrNodeId, SelectedRouteState> beforeByDestination;
            uint32_t invalidated = 0;

            for (auto &route : m_routes)
              {
                if (route.immediate ||
                    route.learnedFrom != helloSrc ||
                    !isIncomingNewer (
                      route.routingSequenceValid,
                      route.routingSequence))
                  {
                    continue;
                  }

                if (beforeByDestination.find (route.nwkDst) ==
                    beforeByDestination.end ())
                  {
                    beforeByDestination.emplace (
                      route.nwkDst,
                      CaptureSelectedRouteState (route.nwkDst));
                  }

                if (route.valid)
                  {
                    invalidated++;
                  }
                route.valid = false;
                route.lastUpdated = Simulator::Now ();
                route.routingSequenceValid = true;
                route.routingSequence = routingSequence;
              }

            if (isIncomingNewer (
                  neighbor.routingInfoValid,
                  neighbor.routingInfoSequence))
              {
                neighbor.routingInfoValid = false;
                neighbor.routingInfoSequence = routingSequence;
              }

            for (const auto &entry : beforeByDestination)
              {
                SelectedRouteState after =
                  CaptureSelectedRouteState (entry.first);
                if (!SameSelectedRouteState (entry.second, after))
                  {
                    MarkSelectedRouteChanged (
                      entry.first,
                      "ARL FLUSH");
                  }
              }

            std::cout << "[NWK " << m_nodeId
                      << "] Applied ARL FLUSH"
                      << " from=" << helloSrc
                      << " routingSequence=" << routingSequence
                      << " routesInvalidated=" << invalidated
                      << std::endl;
            break;
          }

        case CsrRoutingOperation::None:
        default:
          break;
        }
    }

  if (!neighbor.routingSequenceValid ||
      CompareRoutingSequence (
        neighbor.routingSequence,
        routingSequence) < 0)
    {
      neighbor.routingSequenceValid = true;
      neighbor.routingSequence = routingSequence;
    }

  if (sawFlush)
    {
      neighbor.routingSnapshotActive = false;
      neighbor.routingSnapshotBufferedUpdates.clear ();
      neighbor.routingSnapshotSeenDestinations.clear ();

      if (neighbor.routingRequestPending)
        {
          if (neighbor.routingRequestTimeoutEvent.IsPending ())
            {
              Simulator::Cancel (
                neighbor.routingRequestTimeoutEvent);
            }
          neighbor.routingRequestPending = false;
          neighbor.routingRequestRetryCount = 0;
        }
    }

  ScheduleCheckNwkQueue ();
}

std::set<CsrNodeId>
CsrNetLayer::ProcessRoutesPayload (const CsrHelloHeader &hh,
                                    CsrNodeId helloSrc,
                                    double pathlossDb,
                                    double snrDb,
                                    uint32_t linkCost)
{
    std::set<CsrNodeId>
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

        for (CsrNodeId pathNode :
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

        std::vector<CsrNodeId> candidatePath;

        candidatePath.push_back (
          helloSrc);

        for (CsrNodeId pathNode :
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
  CsrNodeId helloSrc,
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

            std::set<CsrNodeId>
              acceptedDestinations;

            uint32_t bufferedRouteCount = 0;

            for (const CsrHelloHeader &bufferedHeader :
                neighbor.routingSnapshotBufferedUpdates)
              {
                bufferedRouteCount +=
                  bufferedHeader
                    .GetAdvertisedRouteCount ();

                std::set<CsrNodeId>
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

        std::set<CsrNodeId>
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
        CsrNodeId destination =
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
        std::map<CsrNodeId, SelectedRouteState>
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
            CsrNodeId destination =
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
CsrNetLayer::UpdateReverseRoute (CsrNodeId netSrc, CsrNodeId hopSrc)
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
  CsrNodeId oldHop = rr.reverseHop;

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
CsrNetLayer::RemoveReverseRouteFromReporter (CsrNodeId netSrc,
                                             CsrNodeId reporter)
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
          MakeNeighborInactive (
            ne.nodeId,
            "freshness timeout");
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

          ne.arlRoutingReassemblies.clear ();

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
  CsrNodeId nextHop,
  const char *reason)
{
  bool anyInvalidated = false;

  // Preserve the selected route for each affected
  // destination before invalidating candidates.
  std::map<CsrNodeId, SelectedRouteState>
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
      CsrNodeId destination =
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
                              CsrNodeId helloSrc,
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
                if (m_arlNeighborAdmissionEnabled)
                  {
                    // routesProcess() holds CHECK_DISCOVERY until our group
                    // key has been ACKed by this neighbor.
                    neighbor.admissionDiscoveryCheckPending = true;
                    neighbor.admissionDiscoverySequence = receivedSequence;

                    std::cout << "[NWK " << m_nodeId
                              << "] Discovery response pending key-send ACK"
                              << " neighbor=" << helloSrc
                              << " sequence=" << receivedSequence
                              << std::endl;

                    if (neighbor.keySendComplete)
                      {
                        Simulator::Schedule (
                          MilliSeconds (20),
                          &CsrNetLayer::SendPendingDiscoveryCheck,
                          this,
                          helloSrc);
                      }
                  }
                else
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
                                   CsrNodeId helloSrc,
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

        NwkNeighborEntry &admissionNeighbor =
          m_nwkNeighbors[helloSrc];
        admissionNeighbor.nodeId = helloSrc;

        bool remoteArlActive =
          hh.GetActiveNodes () == 3; // legacy NEIGHBOR_ACTIVE

        if (!admissionNeighbor.arlActive && remoteArlActive)
          {
            admissionNeighbor.admissionNeedsRoutingRequest = true;
          }

        if (!admissionNeighbor.arlActive || !remoteArlActive)
          {
            TryMakeNeighborActive (
              helloSrc,
              "received CHECK_DISCOVERY");
          }

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
      {
        std::cout << "[NWK " << m_nodeId
                  << "] NeighborCheck Message confirms link with "
                  << helloSrc
                  << std::endl;

        TryMakeNeighborActive (
          helloSrc,
          "received CHECK_MESSAGE");
        break;
      }

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

        TryMakeNeighborActive (
          helloSrc,
          "received CHECK_VERIFY");

        break;
      }

    case CsrNeighborCheckType::Overheard:
      {
        std::cout << "[NWK " << m_nodeId
                  << "] NeighborCheck Overheard received from "
                  << helloSrc
                  << std::endl;

        EnsureCheckMessage (
          helloSrc,
          "received CHECK_OVERHEARD");
        break;
      }

    /*case CsrNeighborCheckType::NoPath:
      std::cout << "[NWK " << m_nodeId
                << "] NeighborCheck NoPath received from "
                << helloSrc
                << " ; destination payload not implemented yet"
                << std::endl;
      break;*/

    case CsrNeighborCheckType::NoPath:
      {
        CsrNodeId unreachableDest = hh.GetNeighborCheckTarget ();

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

void CsrNetLayer::SetRepeatDiscoveryHello (bool enable)
{
  m_repeatDiscoveryHello = enable;
}

void
CsrNetLayer::DiscoveryStart ()
{
  m_discState = DiscoveryState::ACTIVE;
  m_discoveryActive = true;
  m_discoveryStartCount++;

  // routesDiscoveryInitLocalTC() uses repeatCount=3.  The existing boolean
  // remains as a test/demo escape hatch: false requests one broadcast, while
  // the production default reproduces all three legacy broadcasts.
  m_discoveryBroadcastsRemaining =
    m_repeatDiscoveryHello ? 3 : 1;
  m_discoveryCompletionQuietTickObserved = false;

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

  m_discoveryBroadcastCount++;
  m_discoveryBroadcastsRemaining--;

  // The legacy broadcaster runs once more after the third packet.  That
  // fourth timer invocation changes DiscoveryActive to DiscoveryInActive and
  // calls sensorAppEndedDiscovery().
  ScheduleDiscoveryHello ();
}

void
CsrNetLayer::DiscoveryStop ()
{
  if (m_discState != DiscoveryState::ACTIVE)
    {
      return;
    }

  if (m_discoveryHelloEvent.IsPending ())
    {
      Simulator::Cancel (m_discoveryHelloEvent);
    }

  if (m_discoveryStopEvent.IsPending ())
    {
      Simulator::Cancel (m_discoveryStopEvent);
    }

  // OPNET has no post-discovery cooldown.  The node is available for a new
  // SNMP_START_DISCOVERY immediately after sensorAppEndedDiscovery().
  m_discState = DiscoveryState::IDLE;
  m_discoveryActive = false;

  std::cout << "[NWK " << m_nodeId
            << "] DiscoveryStop sequence="
            << m_discoverySequence
            << std::endl;

  VerifyUnresponsiveDiscoveryNeighbors ();

  TryDrainQueueAfterDiscovery ();

  // Legacy discovery completion calls routesReroute():
  // recompute active-neighbor costs and request fresh
  // route state from every active neighbor.
  RefreshRoutesAfterDiscovery ();

  CompleteDiscoveryLifecycle ();
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

      CsrNodeId neighborId = neighbor.nodeId;
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

  if (m_discoveryBroadcastsRemaining == 0)
    {
      if (m_arlNeighborAdmissionEnabled)
        {
          uint32_t pendingAdmissions = 0;

          for (const auto &item : m_nwkNeighbors)
            {
              const NwkNeighborEntry &neighbor = item.second;

              if (neighbor.arlActive || neighbor.stale)
                {
                  continue;
                }

              // In OPNET the key/check exchange normally finishes during
              // the three-broadcast discovery window.  ns-3 can still have
              // those reliable frames in flight when the one-second
              // broadcaster expires.  Keep the lifecycle open so
              // sensorAppEndedDiscovery() sees the same admitted neighbor;
              // the separately scheduled DiscoveryStop remains the bound.
              if (neighbor.admissionDiscoveryCheckPending ||
                  neighbor.admissionDiscoveryCheckActive ||
                  neighbor.keyRequestSentValid ||
                  neighbor.keySendValid ||
                  neighbor.keySendActive ||
                  neighbor.keySendComplete ||
                  neighbor.keyUpdateComplete)
                {
                  pendingAdmissions++;
                }
            }

          if (pendingAdmissions > 0)
            {
              std::cout << "[NWK " << m_nodeId
                        << "] Discovery completion waiting for ARL admission"
                        << " pending=" << pendingAdmissions
                        << std::endl;
              ScheduleDiscoveryHello ();
              return;
            }
        }

      // The broadcaster timer can reach its fourth invocation while the
      // final long-preamble Discover is still in the MAC or while its peer's
      // response is about to start.  Wait for the local radio to clear and
      // then observe one complete quiet discovery interval.  This preserves
      // the legacy response/admission ordering without removing the explicit
      // DiscoveryStop bound.
      if (m_hop != nullptr && m_hop->IsMacBusyOrQueued ())
        {
          m_discoveryCompletionQuietTickObserved = false;
          std::cout << "[NWK " << m_nodeId
                    << "] Discovery completion waiting for MAC control"
                    << std::endl;
          ScheduleDiscoveryHello ();
          return;
        }

      if (!m_discoveryCompletionQuietTickObserved)
        {
          m_discoveryCompletionQuietTickObserved = true;
          std::cout << "[NWK " << m_nodeId
                    << "] Discovery completion observing response interval"
                    << std::endl;
          ScheduleDiscoveryHello ();
          return;
        }

      DiscoveryStop ();
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

  m_discoveryBroadcastCount++;
  m_discoveryBroadcastsRemaining--;

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

    // OPNET-ish "active" proxy: neighbor count (or 0 for now)
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

          if (neighbor.lastHeardSec < 0.0 ||
              neighbor.stale ||
              !IsArlNeighborUsable (neighbor.nodeId))
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

    if (type == CsrArlRouteMsgType::Discover)
      {
        m_hop->SendProtectedDiscovery (p);
      }
    else if (type == CsrArlRouteMsgType::RoutingUpdate)
      {
        m_hop->SendAuthenticatedRoutingHello (p);
      }
    else
      {
        m_hop->SendHello (p);
      }

}

uint32_t
CsrNetLayer::GetActiveNodeCount () const
{
  // OPNET raises active_nodes from neighbor-list size + 1 and never lowers
  // it when a known neighbor becomes inactive.  Neighbor records are kept
  // across ClearRoutes(), so their count is the legacy historical maximum.
  return static_cast<uint32_t> (m_nwkNeighbors.size ()) + 1;
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
  CsrNodeId neighbor,
  CsrNeighborCheckType type,
  CsrNodeId target,
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

  if (type == CsrNeighborCheckType::Discovery &&
      m_arlNeighborAdmissionEnabled)
    {
      // CHECK_DISCOVERY carries the sender's neighbor state byte.  The
      // legacy value is NEIGHBOR_ACTIVE == 3, not the global node count used
      // by ordinary HELLOs.
      hh.SetActiveNodes (
        IsArlNeighborActive (neighbor) ? 3 : 0);
    }
  else
    {
      hh.SetActiveNodes (
        static_cast<uint8_t> (GetActiveNodeCount ()));
    }

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
CsrNetLayer::EnsureDiscoveryEntry (
  CsrNodeId node,
  bool discoveryNeeded)
{
  if (node == m_nodeId || node == CSR_BROADCAST_ID)
    {
      return;
    }

  auto existing = std::find_if (
    m_discoveryTable.begin (),
    m_discoveryTable.end (),
    [node] (const DiscoveryEntry &entry) {
      return entry.nodeId == node;
    });

  // clear_discovery() and SNMP_DISCOVERY_DONE only insert missing entries;
  // they never turn a previously completed entry back on.
  if (existing != m_discoveryTable.end ())
    {
      return;
    }

  m_discoveryTable.push_back ({node, discoveryNeeded});

  std::cout << "[NWK " << m_nodeId
            << "] Discovery table append"
            << " node=" << node
            << " needed=" << (discoveryNeeded ? 1 : 0)
            << " position=" << (m_discoveryTable.size () - 1)
            << std::endl;
}

void
CsrNetLayer::MarkDiscoveryNotNeeded (CsrNodeId node)
{
  auto existing = std::find_if (
    m_discoveryTable.begin (),
    m_discoveryTable.end (),
    [node] (const DiscoveryEntry &entry) {
      return entry.nodeId == node;
    });

  if (existing == m_discoveryTable.end ())
    {
      EnsureDiscoveryEntry (node, false);
      return;
    }

  existing->discoveryNeeded = false;
}

std::vector<CsrNodeId>
CsrNetLayer::CollectKnownDiscoveryNodes () const
{
  std::vector<CsrNodeId> nodes;
  std::set<CsrNodeId> seen;

  // routesListWalkNext() walks logical destinations, not every alternate
  // path.  Preserve the first-seen route-table order while suppressing
  // duplicate next-hop alternatives.
  for (const RouteEntry &route : m_routes)
    {
      CsrNodeId destination = route.nwkDst;

      if (destination == m_nodeId ||
          destination == CSR_BROADCAST_ID ||
          seen.find (destination) != seen.end () ||
          FindBestRoute (destination) == nullptr)
        {
          continue;
        }

      seen.insert (destination);
      nodes.push_back (destination);

      if (nodes.size () == CsrSnmpHeader::MAX_NODES)
        {
          break;
        }
    }

  return nodes;
}

bool
CsrNetLayer::SendSnmp (
  CsrNodeId destination,
  CsrSnmpCommand command,
  int32_t value,
  const std::vector<CsrNodeId> &nodes)
{
  if (m_hop == nullptr)
    {
      return false;
    }

  CsrNodeId hopDestination = destination;
  if (destination != CSR_BROADCAST_ID)
    {
      // send_snmp_pk() uses lookup_route(), not the DATA forwarding policy.
      // Consequently a direct Ordinary/non-capable node is still a valid
      // discovery target even though it cannot be used as a transit route.
      const RouteEntry *route = FindBestRoute (destination);

      if (route == nullptr)
        {
          // The packet is destroyed immediately.  It is neither queued nor
          // allowed to trigger an implicit discovery.
          std::cout << "[NWK " << m_nodeId
                    << "] Drop legacy SNMP: no route"
                    << " destination=" << destination
                    << " command="
                    << unsigned (static_cast<uint8_t> (command))
                    << std::endl;
          return false;
        }

      hopDestination = route->nextHop;
    }

  CsrSnmpHeader header;
  header.SetSource (m_nodeId);
  header.SetDestination (destination);
  header.SetDestinationType (
    destination == CSR_BROADCAST_ID
      ? CSR_DEST_BROADCAST
      : CSR_DEST_UNICAST);
  header.SetCommand (command);
  header.SetValue (value);
  header.SetNodes (nodes);

  Ptr<Packet> payload = Create<Packet> ();
  payload->AddHeader (header);

  if (command == CSR_SNMP_START_DISCOVERY)
    {
      m_snmpStartSentCount++;
    }
  else if (command == CSR_SNMP_DISCOVERY_DONE)
    {
      m_snmpDoneSentCount++;
    }
  else if (command == CSR_SNMP_RELAY_HOLDOFF)
    {
      m_relayHoldoffSentCount++;
    }
  else if (command == CSR_SNMP_RELAY_CLEAR)
    {
      m_relayClearSentCount++;
    }

  std::cout << "[NWK " << m_nodeId
            << "] TX legacy SNMP"
            << " command="
            << unsigned (static_cast<uint8_t> (command))
            << " finalDestination=" << destination
            << " hopDestination=" << hopDestination
            << " nodes=" << header.GetNodes ().size ()
            << std::endl;

  m_hop->SendSnmp (hopDestination, payload);
  return true;
}

void
CsrNetLayer::ReceiveSnmpFromHop (
  Ptr<Packet> snmpPayload,
  CsrNodeId hopSource)
{
  CsrSnmpHeader header;
  if (!snmpPayload->RemoveHeader (header))
    {
      NS_LOG_ERROR ("CsrNetLayer::ReceiveSnmpFromHop(): missing CsrSnmpHeader");
      return;
    }

  bool forThisNode =
    header.GetDestinationType () == CSR_DEST_BROADCAST ||
    (header.GetDestinationType () == CSR_DEST_UNICAST &&
     (header.GetDestination () == m_nodeId ||
      header.GetDestination () == CSR_BROADCAST_ID));

  if (!forThisNode)
    {
      return;
    }

  CsrNodeId source = header.GetSource ();

  if (header.GetCommand () == CSR_SNMP_START_DISCOVERY)
    {
      m_snmpStartReceivedCount++;

      if (std::find (
            m_discoveryCompletionRequesters.begin (),
            m_discoveryCompletionRequesters.end (),
            source) == m_discoveryCompletionRequesters.end ())
        {
          if (m_discoveryCompletionRequesters.size () <
              CsrSnmpHeader::MAX_NODES)
            {
              m_discoveryCompletionRequesters.push_back (source);
            }
          else
            {
              std::cout << "[NWK " << m_nodeId
                        << "] Legacy discovery completion requester list full"
                        << " source=" << source
                        << std::endl;
            }
        }

      std::cout << "[NWK " << m_nodeId
                << "] RX SNMP_START_DISCOVERY"
                << " source=" << source
                << " hopSource=" << hopSource
                << " delay=" << header.GetValue ()
                << " state=" << static_cast<unsigned> (m_discState)
                << std::endl;

      if (m_discState == DiscoveryState::IDLE)
        {
          m_discoveryInitiatedBy = source;
          MarkDiscoveryNotNeeded (source);

          int32_t delaySeconds = std::max<int32_t> (0, header.GetValue ());
          StartDiscovery (
            Seconds (static_cast<double> (delaySeconds)),
            Seconds (30.0));
        }
      else
        {
          // The requester remains in the DCM list and receives DONE when the
          // discovery already in progress completes.
          std::cout << "[NWK " << m_nodeId
                    << "] Ignore duplicate SNMP discovery start while active"
                    << " requesterRetained=1"
                    << std::endl;
        }

      return;
    }

  if (header.GetCommand () == CSR_SNMP_DISCOVERY_DONE)
    {
      m_snmpDoneReceivedCount++;

      std::cout << "[NWK " << m_nodeId
                << "] RX SNMP_DISCOVERY_DONE"
                << " source=" << source
                << " hopSource=" << hopSource
                << " advertisedNodes=" << header.GetNodes ().size ()
                << std::endl;

      for (CsrNodeId node : header.GetNodes ())
        {
          EnsureDiscoveryEntry (node, true);
        }

      CheckDiscoveryTable ();
      return;
    }

  if (header.GetCommand () == CSR_SNMP_RELAY_HOLDOFF ||
      header.GetCommand () == CSR_SNMP_RELAY_CLEAR)
    {
      auto neighbor = m_nwkNeighbors.find (source);

      // br_SNMP bypasses update_neighbor() in the legacy HOP process.  A
      // relay-control source must consequently exist before this record is
      // received; the control itself must not create or refresh a neighbor.
      if (neighbor == m_nwkNeighbors.end ())
        {
          std::cout << "[NWK " << m_nodeId
                    << "] Ignore relay control from unknown neighbor="
                    << source
                    << " command="
                    << unsigned (static_cast<uint8_t> (header.GetCommand ()))
                    << std::endl;
          return;
        }

      bool holdoff =
        header.GetCommand () == CSR_SNMP_RELAY_HOLDOFF;

      neighbor->second.relayHoldoff = holdoff;

      if (holdoff)
        {
          m_relayHoldoffReceivedCount++;
        }
      else
        {
          m_relayClearReceivedCount++;
        }

      std::cout << "[NWK " << m_nodeId
                << "] RX "
                << (holdoff
                      ? "SNMP_RELAY_HOLDOFF"
                      : "SNMP_RELAY_CLEAR")
                << " source=" << source
                << " hopSource=" << hopSource
                << " storedState=" << (holdoff ? 1 : 0)
                << " transitGate=0"
                << std::endl;
      return;
    }

  std::cout << "[NWK " << m_nodeId
            << "] Ignore unsupported legacy SNMP command="
            << unsigned (static_cast<uint8_t> (header.GetCommand ()))
            << std::endl;
}

void
CsrNetLayer::CompleteDiscoveryLifecycle ()
{
  std::vector<CsrNodeId> knownNodes = CollectKnownDiscoveryNodes ();

  for (CsrNodeId node : knownNodes)
    {
      EnsureDiscoveryEntry (node, true);
    }

  std::vector<CsrNodeId> requesters = m_discoveryCompletionRequesters;

  for (CsrNodeId requester : requesters)
    {
      SendSnmp (
        requester,
        CSR_SNMP_DISCOVERY_DONE,
        0,
        knownNodes);
    }

  m_discoveryCompletionRequesters.clear ();

  std::cout << "[NWK " << m_nodeId
            << "] Legacy discovery lifecycle complete"
            << " initiator=" << m_discoveryInitiatedBy
            << " knownNodes=" << knownNodes.size ()
            << " completionReports=" << requesters.size ()
            << std::endl;

  CheckDiscoveryTable ();
}

void
CsrNetLayer::CheckDiscoveryTable ()
{
  if (m_nodeType != CsrNodeType::Gateway &&
      m_nodeType != CsrNodeType::Routable)
    {
      return;
    }

  if (m_snmpReportEvent.IsPending ())
    {
      Simulator::Cancel (m_snmpReportEvent);
    }

  for (DiscoveryEntry &entry : m_discoveryTable)
    {
      if (!entry.discoveryNeeded)
        {
          continue;
        }

      // check_discovery() marks the first tail-ordered entry complete before
      // attempting transmission.  A missing route is therefore not retried.
      entry.discoveryNeeded = false;

      SendSnmp (
        entry.nodeId,
        CSR_SNMP_START_DISCOVERY,
        0);

      m_snmpReportEvent = Simulator::Schedule (
        m_snmpReportTimeout,
        &CsrNetLayer::SnmpReportTimeout,
        this);

      std::cout << "[NWK " << m_nodeId
                << "] SNMP discovery handoff"
                << " target=" << entry.nodeId
                << " watchdog="
                << m_snmpReportTimeout.GetSeconds ()
                << "s"
                << std::endl;
      break;
    }
}

void
CsrNetLayer::SnmpReportTimeout ()
{
  std::cout << "[NWK " << m_nodeId
            << "] SNMP discovery report watchdog expired"
            << std::endl;
  CheckDiscoveryTable ();
}

bool
CsrNetLayer::ShouldAdvertiseRoute (const RouteEntry &re) const
{
  if (m_nodeType == CsrNodeType::Ordinary)
    {
      return false;
    }

  if (!re.valid)
    {
      return false;
    }

  if (!IsArlNeighborUsable (re.nextHop))
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
  CsrNodeId destination,
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
  CsrNodeId routingTarget)
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

Ptr<Packet>
CsrNetLayer::BuildArlRoutingSectionPayload (
  const std::vector<uint8_t> &sectionBytes,
  uint32_t routingSequence,
  uint8_t routingSection,
  uint8_t routingTotalSections)
{
  Ptr<Packet> packet = sectionBytes.empty ()
    ? Create<Packet> ()
    : Create<Packet> (sectionBytes.data (), sectionBytes.size ());

  CsrHelloHeader hh;
  hh.SetNodeId (m_nodeId);
  hh.SetHelloSeq (++m_routingControlHeaderSeq);
  hh.SetNodeType (m_nodeType);
  hh.SetSpeedKey (m_minSpeedKey);

  double s0PowerDbm =
    m_rxS0BaseLevelDbm + m_linkMarginDb;

  hh.SetRxPowerDbmX10 (
    static_cast<int16_t> (
      std::round (s0PowerDbm * 10.0)));
  hh.SetActiveNodes (
    static_cast<uint8_t> (GetActiveNodeCount ()));
  hh.SetArlRouteMsgType (
    CsrArlRouteMsgType::RoutingUpdate);
  hh.SetNeighborCheckType (CsrNeighborCheckType::None);
  hh.SetDiscoverType (CsrDiscoverType::None);
  hh.SetDiscoverySequence (0);

  // These envelope fields are redundant with the six-byte routes.c prefix.
  // HOP uses them only to report per-section reliable completion to NWK.
  hh.SetRoutingSequence (routingSequence);
  hh.SetRoutingSection (routingSection);
  hh.SetRoutingTotalSections (routingTotalSections);
  hh.SetRoutingOperation (CsrRoutingOperation::Update);
  hh.ClearChirpNeighbors ();
  hh.ClearAdvertisedRoutes ();

  packet->AddHeader (hh);
  return packet;
}

std::vector<Ptr<Packet>>
CsrNetLayer::BuildArlRoutingSnapshotPayloads (
  uint32_t routingSequence)
{
  CsrArlRoutingMessage::Builder builder;
  CsrHelloHeader::RoutingInfo info;

  info.minSpeedKbps =
    static_cast<uint16_t> (
      std::clamp (m_minCfgSpeedKbps, 0, 65535));
  info.maxSpeedKbps =
    static_cast<uint16_t> (
      std::clamp (m_maxCfgSpeedKbps, 0, 65535));
  info.minPowerDbmX10 =
    static_cast<int16_t> (
      std::round (m_minTxPowerDbm * 10.0));
  info.maxPowerDbmX10 =
    static_cast<int16_t> (
      std::round (m_maxTxPowerDbm * 10.0));
  info.linkMarginDbX10 =
    static_cast<int16_t> (
      std::round (m_linkMarginDb * 10.0));
  info.lowPowerDbmX10 =
    static_cast<int16_t> (
      std::round (m_txAmpBreakpointDbm * 10.0));
  info.tempLowCx10 = m_tempLowCx10;
  info.tempHighCx10 = m_tempHighCx10;

  // routesProcess() constructs one INFO + UPDATE* + FLUSH record stream.
  builder.AddInfo (info);

  uint32_t advertisedRoutes = 0;

  for (const auto &route : m_routes)
    {
      const RouteEntry *best = FindBestRoute (route.nwkDst);

      if (best != &route || !ShouldAdvertiseRoute (route))
        {
          continue;
        }

      std::vector<CsrNodeId> path;
      path.reserve (route.numHop);

      for (CsrNodeId pathNode : route.path)
        {
          if (path.size () >= route.numHop)
            {
              break;
            }
          path.push_back (pathNode);
        }

      if (path.empty () && route.numHop > 0)
        {
          path.push_back (route.nextHop);
        }

      // Old static/test route entries may not carry the full path list.  ARL
      // always emits exactly numHops 24-bit identifiers, ending at the
      // advertised destination.
      while (path.size () < route.numHop)
        {
          path.push_back (route.nwkDst);
        }

      std::string error;
      if (!builder.AddUpdate (
            route.nwkDst,
            route.capability,
            route.numHop,
            route.cost,
            path,
            &error))
        {
          std::cout << "[NWK " << m_nodeId
                    << "] Skipping ARL snapshot route"
                    << " dst=" << route.nwkDst
                    << " reason=" << error
                    << std::endl;
          continue;
        }

      advertisedRoutes++;
    }

  builder.AddFlush ();

  std::vector<std::vector<uint8_t>> sectionBytes;
  std::string error;

  if (!builder.BuildSections (
        routingSequence,
        sectionBytes,
        &error))
    {
      std::cout << "[NWK " << m_nodeId
                << "] Failed to build ARL routing snapshot"
                << " routingSequence=" << routingSequence
                << " reason=" << error
                << std::endl;
      return {};
    }

  std::vector<Ptr<Packet>> payloads;
  payloads.reserve (sectionBytes.size ());

  for (uint32_t index = 0;
       index < sectionBytes.size ();
       ++index)
    {
      payloads.push_back (
        BuildArlRoutingSectionPayload (
          sectionBytes[index],
          routingSequence,
          static_cast<uint8_t> (index),
          static_cast<uint8_t> (sectionBytes.size ())));
    }

  std::cout << "[NWK " << m_nodeId
            << "] Built ARL routing byte stream"
            << " routingSequence=" << routingSequence
            << " records=" << (advertisedRoutes + 2)
            << " advertisedRoutes=" << advertisedRoutes
            << " recordBytes="
            << builder.GetRecordStream ().size ()
            << " sections=" << payloads.size ()
            << " maxSectionBytes="
            << CsrArlRoutingMessage::MAX_SECTION_SIZE
            << std::endl;

  return payloads;
}

void
CsrNetLayer::StartReliableRoutingSnapshot (
  CsrNodeId neighbor)
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

  if (neighborIt->second.stale ||
      !IsArlNeighborUsable (neighbor))
    {
      std::cout << "[NWK " << m_nodeId
                << "] RoutingSnapshot rejected"
                << " unavailableNeighbor="
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

  uint32_t snapshotSequence =
    NextRoutingSequence ();

  std::vector<Ptr<Packet>> payloads =
    BuildArlRoutingSnapshotPayloads (
      snapshotSequence);

  if (payloads.empty ())
    {
      return;
    }

  snapshot.active = true;
  snapshot.routingSequence = snapshotSequence;
  snapshot.totalSections =
    static_cast<uint8_t> (payloads.size ());
  snapshot.ackedSections.clear ();

  std::cout << "[NWK " << m_nodeId
            << "] Starting reliable ARL RoutingSnapshot"
            << " neighbor=" << neighbor
            << " routingSequence="
            << snapshot.routingSequence
            << " sections="
            << unsigned (snapshot.totalSections)
            << std::endl;

  // All sections are independent HOP reliable transactions.  routes.c
  // queues every section immediately; it never waits for an INFO or prior
  // UPDATE ACK before releasing the next section.
  for (uint32_t section = 0;
       section < payloads.size ();
       ++section)
    {
      std::cout << "[NWK " << m_nodeId
                << "] Sending ARL RoutingSnapshot section="
                << section << "/" << payloads.size ()
                << " neighbor=" << neighbor
                << " routingSequence=" << snapshotSequence
                << std::endl;

      m_hop->SendRoutingControl (
        neighbor,
        payloads[section]);
    }

  ArmRoutingSnapshotWatchdog (
    neighbor,
    "awaiting-independent-section-acks");
}

void
CsrNetLayer::
ArmRoutingSnapshotWatchdog (
  CsrNodeId neighbor,
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
  CsrNodeId neighbor,
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
            << "] ARL RoutingSnapshot watchdog expired"
            << " neighbor=" << neighbor
            << " phase="
            << snapshot.watchdogPhase
            << " routingSequence="
            << snapshot.routingSequence
            << " ackedSections="
            << snapshot.ackedSections.size ()
            << "/"
            << unsigned (snapshot.totalSections)
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
  CsrNodeId neighbor,
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

  if (entry.stale || !IsArlNeighborUsable (neighbor))
    {
      entry.routingRequestPending = false;
      entry.routingRequestRetryCount = 0;

      std::cout << "[NWK " << m_nodeId
                << "] RoutingRequest aborted"
                << " neighbor became unavailable="
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
  CsrNodeId neighbor,
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
  CsrNodeId destination) const
{
  const RouteEntry *best = nullptr;

  bool preferredValid = false;

  CsrNodeId preferredNextHop =
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

      if (!IsArlNeighborUsable (route.nextHop))
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
  CsrNodeId destination) const
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
  CsrNodeId destination,
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

  CsrDifferentialTraceEvent traceEvent;
  traceEvent.event = "route_change";
  traceEvent.node = CsrTraceInteger (m_nodeId);
  traceEvent.destination = CsrTraceInteger (destination);
  traceEvent.success = selectedRoute != nullptr ? "1" : "0";
  traceEvent.reason = reason;
  if (selectedRoute != nullptr)
    {
      traceEvent.nextHop = CsrTraceInteger (selectedRoute->nextHop);
      traceEvent.routeCost = CsrTraceInteger (selectedRoute->cost);
    }
  WriteDifferentialTrace (traceEvent);

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
  std::set<CsrNodeId> destinations;

  destinations.swap (
    m_pendingSelectedRouteChanges);

  std::cout << "[NWK " << m_nodeId
            << "] Consolidated selected-route changes"
            << " count="
            << destinations.size ()
            << std::endl;

  for (CsrNodeId destination :
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
        !neighbor.stale &&
        IsArlNeighborUsable (neighbor.nodeId);

      if (!fresh)
        {
          continue;
        }

      std::set<CsrNodeId>
        &pendingDestinations =
          m_pendingAutomaticRouteChanges[
            neighbor.nodeId];

      bool wasEmpty =
        pendingDestinations.empty ();

      for (CsrNodeId destination :
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
      CsrNodeId neighborId =
        pendingIt->first;

      auto neighborIt =
        m_nwkNeighbors.find (
          neighborId);

      if (neighborIt ==
            m_nwkNeighbors.end () ||
          neighborIt->second.stale ||
          !IsArlNeighborUsable (neighborId))
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

      std::set<CsrNodeId>
        &pendingDestinations =
          pendingIt->second;

      if (pendingDestinations.empty ())
        {
          pendingIt =
            m_pendingAutomaticRouteChanges.erase (
              pendingIt);

          continue;
        }

      CsrNodeId destination =
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

bool
CsrNetLayer::GetSelectedRouteCost (
  CsrNodeId destination,
  uint32_t &costOut) const
{
  const RouteEntry *best =
    FindBestRoute (destination);

  if (best == nullptr)
    {
      return false;
    }

  costOut = best->cost;
  return true;
}

bool
CsrNetLayer::GetNeighborRoutingInfo (
  CsrNodeId neighbor,
  CsrHelloHeader::RoutingInfo &infoOut) const
{
  auto it = m_nwkNeighbors.find (neighbor);

  if (it == m_nwkNeighbors.end () ||
      !it->second.routingInfoValid)
    {
      return false;
    }

  const NwkNeighborEntry &entry = it->second;
  infoOut.minSpeedKbps = entry.remoteMinSpeedKbps;
  infoOut.maxSpeedKbps = entry.remoteMaxSpeedKbps;
  infoOut.minPowerDbmX10 = entry.remoteMinPowerDbmX10;
  infoOut.maxPowerDbmX10 = entry.remoteMaxPowerDbmX10;
  infoOut.linkMarginDbX10 = entry.remoteLinkMarginDbX10;
  infoOut.lowPowerDbmX10 = entry.remoteLowPowerDbmX10;
  infoOut.tempLowCx10 = entry.remoteTempLowCx10;
  infoOut.tempHighCx10 = entry.remoteTempHighCx10;
  return true;
}

uint32_t
CsrNetLayer::GetPendingArlRoutingMessageCount (
  CsrNodeId neighbor) const
{
  auto it = m_nwkNeighbors.find (neighbor);
  return it == m_nwkNeighbors.end ()
    ? 0
    : static_cast<uint32_t> (
        it->second.arlRoutingReassemblies.size ());
}

uint8_t
CsrNetLayer::GetOutboundRoutingSnapshotTotalSections (
  CsrNodeId neighbor) const
{
  auto it = m_outboundRoutingSnapshots.find (neighbor);
  return it == m_outboundRoutingSnapshots.end ()
    ? 0
    : it->second.totalSections;
}

uint32_t
CsrNetLayer::GetOutboundRoutingSnapshotAckedSections (
  CsrNodeId neighbor) const
{
  auto it = m_outboundRoutingSnapshots.find (neighbor);
  return it == m_outboundRoutingSnapshots.end ()
    ? 0
    : static_cast<uint32_t> (
        it->second.ackedSections.size ());
}

bool
CsrNetLayer::IsOutboundRoutingSnapshotActive (
  CsrNodeId neighbor) const
{
  auto it = m_outboundRoutingSnapshots.find (neighbor);
  return it != m_outboundRoutingSnapshots.end () &&
         it->second.active;
}

void
CsrNetLayer::DumpBestRoute (
  CsrNodeId destination) const
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
RecomputeRoutesViaNextHop (
  CsrNodeId nextHop)
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
  std::map<CsrNodeId, SelectedRouteState>
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

  // Legacy linkCharGetCost() reports only the locally measured
  // direction as valid, including during routesReroute().

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
      &localTotalMargin);

  uint32_t newLinkCost =
    localDirectionalCost;

  std::cout << "[NWK " << m_nodeId
            << "] recompute hop-cost"
            << " neighbor="
            << nextHop
            << " failures="
            << neighbor.numFailures
            << " costMode=local-only"
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
      CsrNodeId destination =
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
  std::vector<CsrNodeId>
    activeNeighbors;

  for (const auto &entry :
       m_nwkNeighbors)
    {
      const NwkNeighborEntry &neighbor =
        entry.second;

      bool active =
        neighbor.lastHeardSec >= 0.0 &&
        !neighbor.stale &&
        IsArlNeighborUsable (neighbor.nodeId);

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
  for (CsrNodeId neighborId :
       activeNeighbors)
    {
      RecomputeRoutesViaNextHop (
        neighborId);
    }

  // Legacy then marks every active neighbor
  // as needing a fresh routing request.
  for (CsrNodeId neighborId :
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
