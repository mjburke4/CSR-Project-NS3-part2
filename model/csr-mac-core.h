#pragma once
#include "csr-common.h"
#include <algorithm>

class CsrMacCore
{
  public:
  CsrMacCore ()
    : m_nodeId (0),
      m_dev (nullptr)
  {}

  static constexpr double TS_HOLDOFF_SECONDS = 0.3;
  static constexpr uint32_t TX_QUEUE_SIZE = 512;
  static constexpr uint32_t ACK_QUEUE_SIZE = 256;
  static constexpr uint32_t MAX_ACK_RESEND = 4;
  static constexpr uint32_t MAX_CONCAT_SEGMENTS = 16;

  /** Operational states used by the legacy br_mac process model. */
  enum class State
  {
    IDLE,
    SEARCH,
    TRACK,
    TX
  };

  static const char* StateName (State state)
  {
    switch (state)
      {
      case State::IDLE:   return "idle";
      case State::SEARCH: return "search";
      case State::TRACK:  return "track";
      case State::TX:     return "tx";
      }
    return "unknown";
  }

  void SetNodeId (CsrNodeId id)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (id),
                     "CSR MAC node identifier exceeds 24 bits");
    m_nodeId = id;
  }
  void SetDevice (CsrNetDevice *dev)  { m_dev = dev; }
  int SelectRateByPerTarget (CsrNodeId destId, uint32_t nBits, double targetPer) const;
  void PrintNeighbors () const;

  int GetLastTxRateKbps () const
  {
    return m_lastTxRateKbps;
  }

  /**
   * Select the legacy source-exact or owner-confirmed extended rate set.
   *
   * @param profile Runtime payload-rate profile.
   */
  void SetRateProfile (CsrRateProfile profile)
  {
    SetMaxRateKbps (CsrGetProfileMaxRateKbps (profile));
  }

  /** @return Active runtime payload-rate profile. */
  CsrRateProfile GetRateProfile () const
  {
    return m_maxRateKbps > 128
      ? CsrRateProfile::EXTENDED_DQPSK
      : CsrRateProfile::LEGACY_SOURCE_EXACT;
  }

  /**
   * Set the highest operational rate considered by automatic MAC selection.
   *
   * This accepts 500 as a scenario maximum even though the extended profile's
   * default maximum is 1000 kbit/s.
   *
   * @param rateKbps Maximum operational rate in kbit/s.
   */
  void SetMaxRateKbps (CsrRateKey rateKbps)
  {
    NS_ABORT_MSG_IF (!CsrIsOperationalRateKey (rateKbps),
                     "CSR MAC maximum rate is not operationally defined");
    m_maxRateKbps = rateKbps;
  }

  /** @return Highest rate considered by automatic MAC selection. */
  CsrRateKey GetMaxRateKbps () const
  {
    return static_cast<CsrRateKey> (m_maxRateKbps);
  }

  /**
   * Check whether the active profile permits an operational rate.
   *
   * @param rateKbps Candidate payload rate in kbit/s.
   * @return True when the rate is defined and does not exceed the ceiling.
   */
  bool IsRateEnabled (CsrRateKey rateKbps) const
  {
    return CsrIsOperationalRateKey (rateKbps) &&
           rateKbps <= m_maxRateKbps;
  }

  double GetLastTxPowerDbm () const
  {
    return m_lastTxPowerDbm;
  }

  /** Return the current OPNET-compatible MAC state. */
  State GetState () const
  {
    return m_state;
  }

  /** Return whether at least one acceptable preamble is currently present. */
  bool IsSyncPresent () const
  {
    return m_syncPresent;
  }

  /**
   * Update receiver state after an Idle/Search/Track transition.
   *
   * The device owns signal acquisition; the MAC owns Tx transitions.
   */
  void SetReceiveState (State state)
  {
    NS_ASSERT_MSG (state != State::TX,
                   "receiver transition cannot enter the Tx state");
    if (!m_txInProgress && m_state != state)
      {
        State previous = m_state;
        m_state = state;

        // br_mac.dsp_off() stops both slot and holdoff timers on entry to
        // Idle, while start_search() restarts the two timers independently.
        // A Track/Search transition does not restart either timer.
        if (state == State::IDLE)
          {
            m_txPreparationActive = false;
            m_txHoldoffOver = false;
            if (m_idleRtsEvent.IsPending ())
              {
                Simulator::Cancel (m_idleRtsEvent);
              }
            if (m_txEvent.IsPending ())
              {
                Simulator::Cancel (m_txEvent);
              }
            if (m_txHoldoffEvent.IsPending ())
              {
                Simulator::Cancel (m_txHoldoffEvent);
              }
            StopSlotTick ();
            if (HasPendingFrame ())
              {
                ScheduleIdleRts ();
              }
          }
        else if (previous == State::IDLE && state == State::SEARCH)
          {
            StartSearchTiming ();
          }
        else if (previous == State::TRACK &&
                 state == State::SEARCH &&
                 HasPendingFrame ())
          {
            // br_mac.back2search() resumes or starts PREP_TX immediately;
            // it does not wait for the next TSLOT after a tracked receive.
            ActivateTxPreparation ();
          }

        CsrDifferentialTraceEvent event;
        event.event = "mac_state";
        event.node = CsrTraceInteger (m_nodeId);
        event.detail = StateName (state);
        WriteDifferentialTrace (event);
      }
  }

  /** Update the legacy SYNC_PRES input used by slot processing. */
  void SetSyncPresent (bool present)
  {
    m_syncPresent = present;
  }

  /** Return whether either OPNET transmit queue contains a frame. */
  bool HasPendingFrames () const
  {
    return HasPendingFrame ();
  }

  /** Mark the radio busy for a complete over-the-air transmission. */
  void NotifyPhyTxStart (Time duration);

  void NoteReportedActiveNodes (uint32_t n)
  {
    uint32_t reported = std::max<uint32_t> (1, n);

    if (reported > m_maxReportedActiveNodes)
      {
        m_maxReportedActiveNodes = reported;

        std::cout << "[MAC " << m_nodeId
                  << "] max_reported_active_nodes updated to "
                  << m_maxReportedActiveNodes
                  << std::endl;
      }
  }

  uint32_t GetReportedActiveNodesForSlotting () const
  {
    // OPNET-style:
    // get_slot(max_reported_active_nodes > active_nodes ?
    //          max_reported_active_nodes : active_nodes, rn_range)
    return std::max<uint32_t> (m_activeNodesForPostTx,
                              m_maxReportedActiveNodes);
  }

  private:
  // DiscoveryStart, DiscoveryStop defined inline below
  // SendHelloInternal defined after CsrNetDevice
  void SendHelloInternal (bool reschedule);
  // --- Slot tick for reservation decay (OPNET rtslot_counter style) ---
  EventId m_slotTickEvent;
  //Time    m_slotTickPeriod { Seconds (1.0) };   // default; tune later
  Time    m_slotTickPeriod { Seconds (0.013) }; // OPNET TSLOT_CYCLE
  bool    m_slotTickEnabled { false };
  void SlotTick ();


  public:
  void StartSlotTick (Time period);
  void StopSlotTick ();

  void
  SendHello ()
  {
    // Periodic HELLO mode only
    if (!m_helloEnabled)
      {
        return;
      }
    SendHelloInternal (true);
  }

  void SetActiveNodesForPostTx (uint32_t n);

  // SendHelloInternal moved after CsrNetDevice definition (see below)

  // Discovery window (OPNET-style)
  void
  StartDiscovery (Time startDelay, Time duration)
  {
    // Cancel any previous discovery scheduling
    if (m_discoveryStartEvent.IsPending ()) { Simulator::Cancel (m_discoveryStartEvent); }
    if (m_discoveryStopEvent.IsPending ())  { Simulator::Cancel (m_discoveryStopEvent);  }

    m_discoveryActive = false;

    m_discoveryStartEvent = Simulator::Schedule (startDelay, &CsrMacCore::DiscoveryStart, this);
    m_discoveryStopEvent  = Simulator::Schedule (startDelay + duration, &CsrMacCore::DiscoveryStop, this);
  }

  void
  DiscoveryStart ()
  {
    m_discoveryActive = true;

    // Jitter so nodes do not all HELLO at the exact same instant.
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
    double jitter = rng->GetValue (0.05, 0.20);

    Simulator::Schedule (Seconds (jitter), &CsrMacCore::SendHelloInternal, this, false);
  }

  void
  DiscoveryStop ()
  {
    m_discoveryActive = false;
  }

  int
  GetOpnetSlotRange (uint32_t activeNodesForSlotting) const
  {
    switch (activeNodesForSlotting)
      {
      case 0:  return 15;
      case 1:  return 18;
      case 2:  return 21;
      case 3:  return 25;
      case 4:  return 31;
      case 5:  return 37;
      case 6:  return 44;
      case 7:  return 52;
      case 8:  return 63;
      case 9:  return 75;
      case 10: return 89;
      case 11: return 106;
      case 12: return 127;
      case 13: return 151;
      case 14: return 180;
      case 15: return 214;
      case 16: return 255;
      default: return 255;
      }
  }

  void SetRxCallback (Callback<void, Ptr<Packet>, double, double> cb)
  {
    m_rxCallback = cb;
  }

  void SetTxSentCallback (
    Callback<void, CsrNodeId, uint16_t, Time> cb)
  {
    m_txSentCallback = cb;
  }

  uint32_t GetQueuedFrameCount () const
  {
    return static_cast<uint32_t> (
      m_queue.size () + m_ackQueue.size ());
  }

  uint32_t GetDataQueuedFrameCount () const
  {
    return static_cast<uint32_t> (m_queue.size ());
  }

  uint64_t GetDataQueueDropCount () const
  {
    return m_dataQueueDropCount;
  }

  uint32_t GetAckQueuedFrameCount () const
  {
    return static_cast<uint32_t> (m_ackQueue.size ());
  }

  uint64_t GetTransmittedFrameCount () const
  {
    return m_transmittedFrameCount;
  }

  uint64_t GetLongPreambleTxCount () const
  {
    return m_longPreambleTxCount;
  }

  uint64_t GetShortPreambleTxCount () const
  {
    return m_shortPreambleTxCount;
  }

  int32_t GetNeighborReservationCounter (CsrNodeId neighbor) const
  {
    auto it = m_neighbors.find (neighbor);
    return (it == m_neighbors.end ()) ? -1 : it->second.rtCounter;
  }

  /** Return the sender's live OPNET txslot_counter value. */
  int32_t GetLocalReservationCounter () const
  {
    return m_txCountdownCounter;
  }

  /** Return the reservation carried in the most recent OTA frame. */
  int32_t GetLastAdvertisedReservationSlot () const
  {
    return m_lastAdvertisedReservationSlot;
  }

  /**
   * Force the reservation selection for a controlled OPNET differential run.
   *
   * A negative value disables the override. Production scenarios leave this
   * disabled; the reservation/collision fixture uses it to load a known slot
   * and counter and align the first countdown tick while retaining the source
   * holdoff, countdown-step, and transmit logic.
   *
   * @param slot Reservation slot in 1..255, or a negative value to disable.
   */
  void SetReservationSlotOverrideForDifferentialRun (int32_t slot)
  {
    NS_ABORT_MSG_IF (slot == 0 || slot > 255,
                     "differential reservation slot must be 1..255 or negative");
    m_forcedReservationSlot = slot;
  }

  /** Return the reservation consumed by the most recent transmission. */
  int32_t GetLastTxOpportunitySlot () const
  {
    return m_lastTxOpportunitySlot;
  }

  /** Return whether br_mac PREP_TX is active. */
  bool IsTxPreparationActive () const
  {
    return m_txPreparationActive;
  }

  // Called by Hop layer to enqueue a full over-the-air frame
  void EnqueueTxFrame (Ptr<Packet> frame,
                       CsrNodeId dest,
                       uint8_t dscp,
                       bool ackable);

  // Remove retransmissions that a cumulative HOP ACK/DACK has already
  // completed.  This mirrors the legacy br_Mac_Hop_Inst queue cleanup.
  uint32_t CancelAcknowledgedFrames (CsrNodeId neighbor,
                                     uint16_t baseSeq,
                                     uint64_t ackBitmap,
                                     uint64_t dackBitmap);

  // Legacy PacketTxInfo.immediateTag replaces an older queued packet of the
  // same type/tag.  KeyRequest uses the destination node as that tag.
  uint32_t CancelQueuedFramesByType (CsrNodeId neighbor, uint8_t type);

  // Called by CsrNetDevice when a frame is successfully received
  void DeliverRxFrameToUp (Ptr<Packet> frame, double pathlossDb, double snrDb)
  {
    if (!m_rxCallback.IsNull ())
      {
        m_rxCallback (frame, pathlossDb, snrDb);
      }
  }

  void NoteHeardFrom (CsrNodeId neighbor, double tRx)
  {
    m_lastHeardSec[neighbor] = tRx;
    auto neighborIt = m_neighbors.find (neighbor);
    bool alreadyKnown = neighborIt != m_neighbors.end ();
    NeighborInfo &info = m_neighbors[neighbor];
    info.lastHeardSec = tRx;
    if (alreadyKnown)
      {
        // br_mac.proc_rx_pk() tries to apply the advertised reservation before
        // its later ACKable-reception path adds an unknown sender.  A neighbor
        // that existed before this reception is therefore eligible; the first
        // reception that creates it is not.
        info.reservationEligible = true;
      }
    // pathloss may be updated separately
  }

  void SetNeighborPathloss (CsrNodeId neighbor, double pathlossDb)
  {
    NeighborInfo &info = m_neighbors[neighbor];
    info.lastPathlossDb = pathlossDb;
    m_lastPathlossDb[neighbor] = pathlossDb;
  }

  void NoteNeighborReservedSlot (CsrNodeId neighbor, int slot)
  {
    if (!m_slotTickEnabled)
      {
        StartSlotTick (m_slotTickPeriod);
      }

    auto neighborIt = m_neighbors.find (neighbor);
    if (neighborIt != m_neighbors.end () &&
        !neighborIt->second.reservationEligible)
      {
        std::cout << "[MAC " << m_nodeId
                  << "] Ignore first-contact reservation from unknown neighbor "
                  << neighbor
                  << " slot=" << slot
                  << std::endl;
        return;
      }

    NeighborInfo &info = m_neighbors[neighbor];

    info.reservationEligible = true;
    info.reserveSlot = slot;
    // OPNET copies the advertised "Reserve TSlot" into both fields.  The
    // counter then advances toward the neighbor's next transmission once per
    // 13-ms MAC slot; it is not a fixed reservation lifetime.
    info.rtCounter   = slot;

    std::cout << "[MAC " << m_nodeId << "] Learned neighbor " << neighbor
              << " uses slot " << slot
              << " (rtCounter=" << info.rtCounter << ")"
              << std::endl;
  }

  /*void NoteNeighborReservedSlot (CsrNodeId neighbor, int slot)
  {
    NeighborInfo &info = m_neighbors[neighbor];
    info.reserveSlot = slot;
    info.rtCounter = 20; // Simple hold window; tune later if needed.

    std::cout << "[MAC " << m_nodeId << "] Learned neighbor " << neighbor
          << " uses slot " << slot
          << " (rtCounter=" << m_neighbors[neighbor].rtCounter << ")"
          << std::endl;
  }*/

  void StartHello (Time interval)
  {
    m_helloInterval = interval;
    m_helloEnabled = true;
    m_neighborTimeout = std::max(Seconds(20.0), interval * 10);
    
    // small jitter so all nodes don't shout at t=0
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
    double jitter = rng->GetValue (0.05, 0.20);

    m_helloEvent = Simulator::Schedule (Seconds (jitter), &CsrMacCore::SendHello, this);

  }

  void
  StartNeighborAging (Time timeout, Time period)
  {
    m_neighborTimeout = timeout;
    m_neighborAgingEvent = Simulator::Schedule (period, &CsrMacCore::AgeNeighbors, this);
  }

  void AgeNeighbors ()
  {
    // Keep neighbor entries; OPNET uses last_rcvd_time as metadata,
    // not as a hard delete trigger in the supplied source files.
    PrintNeighbors ();
    m_neighborAgingEvent = Simulator::Schedule (Seconds (2.0),
                                              &CsrMacCore::AgeNeighbors, this);
  }


  // PrintNeighbors() moved after CsrNetDevice definition (see below)

  CsrNodeId GetNodeId () const { return m_nodeId; }

private:
  struct NeighborInfo
  {
    double lastHeardSec { -1.0 };
    double lastPathlossDb { NAN };
    double lastSnrDb { NAN };
    int32_t reserveSlot { -1 };   // neighbor's reserved timeslot (if tracked)
    int32_t rtCounter   { -1 };   // how long to honor that reservation
    bool reservationEligible {false}; // known before the current reception
  };

  struct TxQueueEntry
  {
    Ptr<Packet> frame;
    CsrNodeId    dest;
    uint8_t     dscp;
    bool        ackable;
  };

  struct AckQueueEntry
  {
    Ptr<Packet> frame;
    CsrNodeId    dest;
    uint16_t    seq;
    bool        hasWindow;
    uint32_t    txCount;
  };

  struct SelectedTxFrame
  {
    Ptr<Packet> frame;
    CsrNodeId    dest;
    bool        ackable;
    bool        fromAckQueue;
    uint32_t    queueIndex;
    int         rateKbps;
    double      txPowerDbm;
  };

  EventId   m_helloEvent;
  Time      m_helloInterval { Seconds (2.0) };
  uint16_t  m_helloSeq { 0 };
  bool      m_helloEnabled { false };
  bool      m_discoveryActive { false };
  //uint32_t  m_maxReportedActiveNodes { 0 };
  uint32_t  m_activeNodesForPostTx { 1 };      // local active_nodes
  uint32_t  m_maxReportedActiveNodes { 0 };    // max Active reported by neighbors
  EventId   m_discoveryStartEvent;
  EventId   m_discoveryStopEvent;


  void MaybeScheduleNextTx ();
  void EnqueueAckFrame (Ptr<Packet> frame, const CsrHeader &header);
  bool HasPendingFrame () const;
  void StartSearchTiming ();
  void ScheduleIdleRts ();
  void IdleRts ();
  void StartTxHoldoff ();
  void TxHoldoffExpired ();
  void ActivateTxPreparation (bool redrawZero = false);
  void FinishTx ();
  void DoTx ();
  int SelectQueuedFrameRate (Ptr<Packet> frame,
                             CsrNodeId dest,
                             bool ackable) const;
  double SelectQueuedFrameTxPower (Ptr<Packet> frame) const;
  uint32_t GetConcatByteLimit (int rateKbps) const;
  bool FitsConcatFrame (Ptr<Packet> frame,
                        int frameRateKbps,
                        uint32_t byteCount,
                        int currentRateKbps) const;
  PreambleType SelectAggregatePreamble (
    const std::vector<SelectedTxFrame> &selected,
    bool concatenating);

  int           ChooseRateForDest (CsrNodeId dest);
  PreambleType  ChoosePreambleForDest (CsrNodeId dest);

private:
  CsrNodeId                            m_nodeId;
  CsrNetDevice*                       m_dev;
  std::vector<TxQueueEntry>           m_queue;
  std::vector<AckQueueEntry>          m_ackQueue;
  EventId                             m_txEvent;
  Callback<void, Ptr<Packet>, double, double>  m_rxCallback;
  Callback<void, CsrNodeId, uint16_t, Time>    m_txSentCallback;
  uint64_t                            m_transmittedFrameCount {0};
  uint64_t                            m_longPreambleTxCount {0};
  uint64_t                            m_shortPreambleTxCount {0};
  uint64_t                            m_dataQueueDropCount {0};
  int                                 m_maxRateKbps {128}; ///< Transmit ceiling.
  int                                 m_lastTxRateKbps {0};
  double                              m_lastTxPowerDbm {0.0};
  int                                 m_scheduledTxSlot {-1};
  int                                 m_txCountdownCounter {-1};
  int                                 m_lastAdvertisedReservationSlot {-1};
  int                                 m_lastTxOpportunitySlot {-1};
  int                                 m_forcedReservationSlot {-1};
  bool                                m_txPreparationActive {false};
  bool                                m_txHoldoffOver {false};
  bool                                m_txInProgress {false};
  bool                                m_syncPresent {false};
  State                               m_state {State::SEARCH};
  EventId                             m_idleRtsEvent;
  EventId                             m_txHoldoffEvent;
  EventId                             m_finishTxEvent;
  std::map<CsrNodeId, NeighborInfo>    m_neighbors;
  Time                                m_neighborTimeout { Seconds (6.0) };   // default: 3x hello interval (if hello=2s)
  EventId                             m_neighborAgingEvent;
  std::map<CsrNodeId, double>          m_lastHeardSec;  // neighborId -> last heard time (seconds)
  std::map<CsrNodeId, double>          m_lastPathlossDb; // neighborId -> last pathloss (optional)
  //void DiscoveryStart ();
  //void DiscoveryStop ();
  //void SendHelloInternal (bool reschedule);

  /*int PickTxSlot (CsrNodeId dest)
  {
    const int SLOT_RANGE = 64;
    bool used[SLOT_RANGE];

    uint32_t activeForSlotting = GetReportedActiveNodesForSlotting ();

    std::cout << "[MAC " << m_nodeId
              << "] PickTxSlot active_nodes_for_slotting="
              << activeForSlotting
              << std::endl;
              
    for (int i = 0; i < SLOT_RANGE; ++i)
      {
        used[i] = false;
      }

    // Decay reservation counters and mark reserved slots as used
    for (auto &kv : m_neighbors)
      {
        NeighborInfo &ni = kv.second;
        if (ni.rtCounter > 0 && ni.reserveSlot >= 0 && ni.reserveSlot < SLOT_RANGE)
          {
            used[ni.reserveSlot] = true;
            //ni.rtCounter--;  // simple decay (we can move to slot-tick later)
            if (ni.rtCounter == 0)
              {
                ni.reserveSlot = -1;
              }
          }
      }

    // Debug: show avoided slots
    std::cout << "[MAC " << m_nodeId << "] PickTxSlot(dest=" << dest << ") avoid:";
    for (int s = 0; s < SLOT_RANGE; ++s)
      {
        if (used[s]) std::cout << " " << s;
      }
    std::cout << std::endl;

    // Build list of free slots
    std::vector<int> freeSlots;
    freeSlots.reserve (SLOT_RANGE);
    for (int s = 0; s < SLOT_RANGE; ++s)
      {
        if (!used[s])
          {
            freeSlots.push_back (s);
          }
      }

    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
    int chosenSlot;

    if (!freeSlots.empty ())
      {
        int idx = rng->GetInteger (0, static_cast<int>(freeSlots.size ()) - 1);
        chosenSlot = freeSlots[idx];
      }
    else
      {
        chosenSlot = rng->GetInteger (0, SLOT_RANGE - 1);
      }

    std::cout << "[MAC " << m_nodeId << "] PickTxSlot chose " << chosenSlot << std::endl;
    return chosenSlot;
  }

};*/

  int
  PickTxSlot (CsrNodeId dest)
  {
    uint32_t activeForSlotting = GetReportedActiveNodesForSlotting ();
    int slotRange = GetOpnetSlotRange (activeForSlotting);

    // OPNET get_slot() returns a slot in [1, slot_range].
    // Keep a fixed reservation table up to 255 for now.
    const int MAX_SLOTRESERVE_NS3 = 256;

    bool used[MAX_SLOTRESERVE_NS3];
    for (int i = 0; i < MAX_SLOTRESERVE_NS3; ++i)
      {
        used[i] = false;
      }

    std::cout << "[MAC " << m_nodeId
              << "] PickTxSlot(dest=" << dest
              << ") active_nodes_for_slotting=" << activeForSlotting
              << " opnet_slot_range=" << slotRange
              << std::endl;

    // OPNET get_slot() marks each neighbor's current reservation counter, not
    // the original advertised offset.  The counter moves every slot tick.
    for (auto &kv : m_neighbors)
      {
        NeighborInfo &ni = kv.second;

        if (ni.rtCounter >= 0 &&
            ni.rtCounter < MAX_SLOTRESERVE_NS3)
          {
            used[ni.rtCounter] = true;

            std::cout << "[MAC " << m_nodeId
                      << "] avoiding neighbor " << kv.first
                      << " currentReservedSlot=" << ni.rtCounter
                      << " rtslot_counter=" << ni.rtCounter
                      << std::endl;
          }
      }

    /*for (auto &kv : m_neighbors)
      {
        NeighborInfo &ni = kv.second;

        if (ni.rtCounter > 0 &&
            ni.reserveSlot >= 0 &&
            ni.reserveSlot < MAX_SLOTRESERVE_NS3)
          {
            used[ni.reserveSlot] = true;
          }

        // Keep existing behavior: reservation decay happens in SlotTick(),
        // not here. This is closer to the OPNET rtslot_counter idea.
        if (ni.rtCounter == 0)
          {
            ni.reserveSlot = -1;
          }
      }*/

    // Debug: show avoided slots inside the selectable OPNET range.
    std::cout << "[MAC " << m_nodeId << "] PickTxSlot(dest=" << dest << ") avoid:";
    for (int s = 1; s <= slotRange; ++s)
      {
        if (used[s])
          {
            std::cout << " " << s;
          }
      }
    std::cout << std::endl;

    if (m_forcedReservationSlot > 0)
      {
        std::cout << "[MAC " << m_nodeId
                  << "] controlled differential reservation slot="
                  << m_forcedReservationSlot
                  << std::endl;
        return m_forcedReservationSlot;
      }

    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();

    // The legacy code draws an ordinal in [1, slotRange], then scans the full
    // 255-entry reservation table beginning at slot zero.  Slot zero is
    // intentionally skipped by the countdown logic; occupied slots therefore
    // shift the selected physical offset beyond the nominal slot range.
    int remaining = rng->GetInteger (1, slotRange);
    int chosenSlot = -1;
    for (int slot = 0; slot < MAX_SLOTRESERVE_NS3 - 1; ++slot)
      {
        if (used[slot])
          {
            continue;
          }

        if (remaining == 0)
          {
            chosenSlot = slot;
            break;
          }
        remaining--;
      }

    if (chosenSlot < 0)
      {
        std::cout << "[MAC " << m_nodeId
                  << "] slot reservation table exhausted; using random fallback"
                  << std::endl;
        chosenSlot = rng->GetInteger (1, slotRange);
      }

    std::cout << "[MAC " << m_nodeId
              << "] PickTxSlot chose " << chosenSlot
              << " using OPNET free-slot ordinal range [1," << slotRange << "]"
              << std::endl;

    return chosenSlot;
  }

};

void
CsrMacCore::StartSlotTick (Time period)
{
  if (m_slotTickEnabled &&
      m_slotTickEvent.IsPending () &&
      period == m_slotTickPeriod)
    {
      return;
    }

  m_slotTickPeriod  = period;
  m_slotTickEnabled = true;

  if (m_slotTickEvent.IsPending ())
    {
      Simulator::Cancel (m_slotTickEvent);
    }

  NS_ABORT_MSG_IF (!m_slotTickPeriod.IsPositive (),
                   "CSR MAC slot period must be positive");

  // br_mac start_search()/prep_tx() establish the TSLOT phase relative to
  // the transition that wakes the receiver.  The phase remains stable until
  // dsp_off() cancels the timer; it is not snapped to a simulation-wide grid.
  m_slotTickEvent = Simulator::Schedule (m_slotTickPeriod,
                                         &CsrMacCore::SlotTick,
                                         this);
}

void
CsrMacCore::StopSlotTick ()
{
  m_slotTickEnabled = false;
  if (m_slotTickEvent.IsPending ())
    {
      Simulator::Cancel (m_slotTickEvent);
    }
}

void
CsrMacCore::SlotTick ()
{
  if (!m_slotTickEnabled)
    {
      return;
    }

  // br_mac reset_tslot_timer runs before any per-state work.  Re-arm first
  // so a TX completion exactly on a boundary cannot redefine the phase.
  m_slotTickEvent =
    Simulator::Schedule (m_slotTickPeriod,
                         &CsrMacCore::SlotTick,
                         this);

  // OPNET tslot_tasks() advances reservation counters only in Search_st and
  // only while no SYNC preamble is present.  Idle, Track, Tx, and Search with
  // SYNC all keep the 13-ms timer running without changing the counters.
  if (m_state == State::SEARCH)
    {
      if (!m_syncPresent)
        {
          for (auto &kv : m_neighbors)
            {
              CsrNodeId neighbor = kv.first;
              NeighborInfo &ni = kv.second;

              // The source's temporary `if (1)` decrements every neighbor on
              // every eligible Search tick, including already-expired values.
              ni.rtCounter--;

              if (ni.rtCounter == -1)
                {
                  std::cout << "[MAC " << m_nodeId
                            << "] rtslot_counter expired for neighbor "
                            << neighbor
                            << " reserveSlot=" << ni.reserveSlot
                            << std::endl;
                }
            }

          // The sender's txslot_counter advances on this same 13-ms phase.  In
          // br_mac this continues even with empty queues so a reservation carried
          // by the preceding OTA frame remains live for a later enqueue.
          if (m_txHoldoffOver && m_txCountdownCounter >= 0)
            {
              m_txCountdownCounter--;

              CsrDifferentialTraceEvent event;
              event.event = "reservation_tick";
              event.node = CsrTraceInteger (m_nodeId);
              event.reservationSlot = CsrTraceSignedInteger (
                m_scheduledTxSlot);
              event.reservationCounter = CsrTraceSignedInteger (
                m_txCountdownCounter);
              WriteDifferentialTrace (event);

              if (m_txCountdownCounter == -1 &&
                  !m_txPreparationActive)
                {
                  m_scheduledTxSlot = -1;
                  std::cout << "[MAC " << m_nodeId
                            << "] local advertised reservation expired"
                            << std::endl;
                }
            }

          if (m_txPreparationActive &&
              HasPendingFrame () &&
              m_txCountdownCounter == -1 &&
              !m_txEvent.IsPending ())
            {
              DoTx ();
              return;
            }
        }

      // OPNET keeps this PREP_TX activation outside its !SYNC_PRES block.
      // A Search-state arrival can therefore select/reuse a reservation while
      // synchronization is present, although the counter stays frozen.
      if (!m_txPreparationActive && HasPendingFrame ())
        {
          // Source ordering is deliberate: an empty reservation advances
          // before newly queued traffic activates.  In particular, counter
          // zero expires and selects a new slot rather than being reused.
          ActivateTxPreparation ();
        }
    }
}

// SendHello() moved after CsrNetDevice definition (see below)
