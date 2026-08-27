#pragma once
#include "csr-common.h"
#include "csr-mac-core.h"
#include "csr-opnet-envelope.h"
#include "csr-phy-model.h"

#include <limits>

class CsrNetDevice : public Object
{
public:
  CsrNetDevice () = default;
  CsrNetDevice (CsrNodeId id)
    : m_id (id)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (id),
                     "CSR device node identifier exceeds 24 bits");
    m_mac.SetNodeId (id);
    m_mac.SetDevice (this);
    m_rng = CreateObject<UniformRandomVariable> ();
  }

  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::CsrNetDevice")
      .SetParent<Object> ()
      .AddConstructor<CsrNetDevice> ();
    return tid;
  }

  void SetPeer (Ptr<CsrNetDevice> peer)
  {
    // backward-compatible: single peer
    m_peers.clear ();
    if (peer != nullptr)
      {
        m_peers.push_back (peer);
      }
  }

  void AddPeer (Ptr<CsrNetDevice> peer)
  {
    if (peer != nullptr)
      {
        m_peers.push_back (peer);
      }
  }

  void EnableDutyCycling (bool enable)
  {
    m_dutyCycleEnabled = enable;
    if (!enable && m_pendingTxWakeEvent.IsPending ())
      {
        Simulator::Cancel (m_pendingTxWakeEvent);
      }
    if (enable && m_rng)
      {
        // De-sync nodes so they don't all wake at same instant
        m_wakePhaseSec = m_rng->GetValue (0.0, m_wakeCycleSec);
      }
    RefreshDutyState ();
    if (enable &&
        m_mac.GetState () == CsrMacCore::State::IDLE &&
        m_mac.HasPendingFrames ())
      {
        SchedulePendingTxWake ();
      }
  }

  bool IsDutyCyclingEnabled () const { return m_dutyCycleEnabled; }

  /** Set a deterministic wake phase, primarily for repeatable scenarios. */
  void SetDutyCyclePhase (Time phase)
  {
    double seconds = std::fmod (phase.GetSeconds (), m_wakeCycleSec);
    m_wakePhaseSec = seconds < 0.0 ? seconds + m_wakeCycleSec : seconds;
    if (m_pendingTxWakeEvent.IsPending ())
      {
        Simulator::Cancel (m_pendingTxWakeEvent);
      }
    RefreshDutyState ();
    if (m_dutyCycleEnabled &&
        m_mac.GetState () == CsrMacCore::State::IDLE &&
        m_mac.HasPendingFrames ())
      {
        SchedulePendingTxWake ();
      }
  }

  void ForceAwakeFor (double seconds)
  {
    double now = Simulator::Now ().GetSeconds ();
    m_forceAwakeUntilSec = std::max (m_forceAwakeUntilSec, now + seconds);
    if (m_mac.GetState () == CsrMacCore::State::IDLE)
      {
        m_mac.SetReceiveState (CsrMacCore::State::SEARCH);
      }

    if (m_forceAwakeEndEvent.IsPending ())
      {
        Simulator::Cancel (m_forceAwakeEndEvent);
      }
    m_forceAwakeEndEvent = Simulator::Schedule (
      Seconds (std::max (0.0, m_forceAwakeUntilSec - now)),
      &CsrNetDevice::RefreshDutyState,
      this);
  }

  CsrMacCore& GetMac ()
  {
    return m_mac;
  }

  /**
   * Configure the MAC-only rate profile for direct device scenarios.
   *
   * Full NWK/HOP stacks should set the profile on CsrNetLayer so it propagates
   * consistently through link control and MAC selection.
   *
   * @param profile Runtime payload-rate profile.
   */
  void SetRateProfile (CsrRateProfile profile)
  {
    m_mac.SetRateProfile (profile);
  }

  /** @return Active MAC-only payload-rate profile. */
  CsrRateProfile GetRateProfile () const
  {
    return m_mac.GetRateProfile ();
  }

  CsrPhyModel& GetPhy() { return m_phy; }

  CsrNodeId GetId () const { return m_id; }

  /** Return the current OPNET Idle/Search/Track/Tx state. */
  CsrMacCore::State GetMacState () const
  {
    return m_mac.GetState ();
  }

  /** Return the number of overlapping acceptable preambles observed. */
  uint64_t GetRxCollisionCount () const
  {
    return m_rxCollisionCount;
  }

  /** Return the number of tracked frames that captured weaker interference. */
  uint64_t GetRxCaptureCount () const
  {
    return m_rxCaptureCount;
  }

  /** Return the number of frames missed while asleep or transmitting. */
  uint64_t GetRxMissCount () const
  {
    return m_rxMissCount;
  }

  /** Return the number of packets rejected specifically by br_ecc. */
  uint64_t GetRxEccDropCount () const
  {
    return m_rxEccDropCount;
  }

  /** Return whether a tracked signal has completed a PHY decision. */
  bool HasLastRxDecision () const
  {
    return m_hasLastRxDecision;
  }

  /** Return the most recent tracked receive decision and PHY diagnostics. */
  const CsrRxDecision &GetLastRxDecision () const
  {
    NS_ABORT_MSG_IF (!m_hasLastRxDecision,
                     "CSR receiver has no completed PHY decision");
    return m_lastRxDecision;
  }

  /** Configure the source-backed JSR boundary used before PHY calibration. */
  void SetCaptureMarginDb (double marginDb)
  {
    m_captureMarginDb = marginDb;
  }

  void SetActiveNodesForPostTx (uint32_t n)
  {
    m_activeNodesForPostTx = std::max<uint32_t> (1, n);
  }

  void NoteReportedActiveNodes (uint32_t n)
  {
    if (n > m_maxReportedActiveNodes)
      {
        m_maxReportedActiveNodes = n;

        std::cout << "[MAC " << m_id
                  << "] max_reported_active_nodes updated to "
                  << m_maxReportedActiveNodes
                  << std::endl;
      }
  }

  uint32_t GetReportedActiveNodesForSlotting () const
  {
    return std::max<uint32_t> (m_activeNodesForPostTx,
                              m_maxReportedActiveNodes);
  }

  Time SendToPeer (Ptr<Packet> frame,
                   CsrNodeId dest,
                   int rateKbps,
                   double txPowerDbm,
                   PreambleType preamble,
                   int slot,
                   bool ackable);

  Time SendFramesToPeers (const std::vector<Ptr<Packet>> &frames,
                          int rateKbps,
                          double txPowerDbm,
                          PreambleType preamble,
                          int slot,
                          bool ackable);

private:
  friend class CsrMacCore;

  struct RxSignal
  {
    uint64_t id {0};
    uint64_t arrivalOrder {0};
    CsrNodeId txId {0};
    uint16_t sequence {0};
    std::vector<Ptr<Packet>> frames;
    int rateKbps {8};
    double txPowerDbm {0.0};
    PreambleType preamble {PREAMBLE_LONG};
    int reservedSlot {-1};
    bool ackable {false};
    uint32_t payloadBytes {0};
    uint32_t packetBits {0};
    CsrTxRadioProfile txRadio;
    CsrPhyFrontEndResult frontEnd;
    double startSec {0.0};
    double endSec {0.0};
    double preambleEndSec {0.0};
    double pathlossDb {0.0};
    double rxPowerDbm {0.0};
    double snrDb {0.0};
    double currentInterferenceWatts {0.0};
    double peakNoiseWatts {0.0};
    double intervalStartSec {0.0};
    double jsrDb {-std::numeric_limits<double>::infinity ()};
    double timeOffsetSeconds {0.0};
    uint32_t collisionCount {0};
    std::vector<CsrBerInterval> berIntervals;
    CsrErrorAllocation errorAllocation;
    bool syncEligible {false};
    bool sameRateInterference {false};
    bool preambleActive {true};
    bool rejected {false};
    bool tracked {false};
    bool collided {false};
    bool missedByState {false};
  };

  void BeginReceiveSignal (RxSignal signal);
  void EndReceivePreamble (uint64_t signalId);
  void EndReceiveSignal (uint64_t signalId);
  void AcquireSignal ();
  void ScheduleAcquisition ();
  void ApplyTrackedInterference (RxSignal &interferer);
  void ApplyInterferencePair (RxSignal &first, RxSignal &second);
  void RemoveEndedInterference (const RxSignal &ended);
  void UpdateSignalNoise (RxSignal &signal);
  void CloseSignalInterval (RxSignal &signal, double endSec);
  void CloseAllSignalIntervals (double endSec);
  void StartAllSignalIntervals (double startSec);
  void RecordSameRateInterference (RxSignal &desired,
                                   const RxSignal &jammer);
  void RefreshHighRateInterference ();
  void DeliverTrackedSignal (const RxSignal &signal,
                             const CsrRxDecision &decision);
  void UpdateSyncPresence ();
  void RefreshDutyState ();
  void WakeForSignal ();
  void WakeForPendingTx ();
  void SleepReceiver ();
  void ScheduleSignalWake ();
  void SchedulePendingTxWake ();
  bool IsPeriodicAwakeAt (double timeSec) const;
  double GetPeriodicWindowEnd (double timeSec) const;
  double GetNextPeriodicWake (double timeSec) const;
  void OnMacTxFinished ();

  CsrNodeId                       m_id {0};
  CsrMacCore                     m_mac;
  std::vector< Ptr<CsrNetDevice> > m_peers;   // multiple peers on shared channel
  Ptr<UniformRandomVariable>     m_rng;
  CsrPhyModel                    m_phy;
  std::map<uint64_t, RxSignal>   m_rxSignals;
  uint64_t                       m_nextTxSignalId {0};
  uint64_t                       m_nextRxArrivalOrder {0};
  uint64_t                       m_trackedSignalId {0};
  uint64_t                       m_rxCollisionCount {0};
  uint64_t                       m_rxCaptureCount {0};
  uint64_t                       m_rxMissCount {0};
  uint64_t                       m_rxEccDropCount {0};
  bool                           m_hasLastRxDecision {false};
  CsrRxDecision                  m_lastRxDecision;
  EventId                        m_acquisitionEvent;
  EventId                        m_signalWakeEvent;
  EventId                        m_pendingTxWakeEvent;
  EventId                        m_sleepEvent;
  EventId                        m_forceAwakeEndEvent;

  // --- Duty cycling (OPNET-inspired) ---
  bool   m_dutyCycleEnabled { false };

  // OPNET constants:
  //   WAKE_CYCLE ~0.988 s
  //   DSP_RX_BOOTTIME 0.0011 s
  //   NO_SIG_SRH_TIME 0.0078 s
  double m_wakeCycleSec     { 0.988 };
  double m_awakeWindowSec   { 0.0011 + 0.0078 }; // ~0.0089 s
  double m_wakePhaseSec     { 0.0 };             // randomized per node
  double m_forceAwakeUntilSec { 0.0 };
  double m_syncToTrackSec { 0.00663 };
  // Retained for explicit compatibility BER hooks.  The production path uses
  // the source-derived JSR buckets, BER tables, interval errors, and ECC gate.
  double m_captureMarginDb {10.5};
  bool m_rxSameSpeed {false};
  double m_rxJsrDb {-1000.0};
  double m_rxTimeOffsetSeconds {0.0};
  // OPNET br_mac.pr.c:
  // POST_TX_WAIT_TIME = STAY_AWAKE_BASE_TIME + 1.5*active_nodes + STAY_AWAKE_GUARD_TIME
  double m_stayAwakeBaseSec  { 15.0 };
  double m_stayAwakeGuardSec { 0.5 };

  // Placeholder until active_nodes is plumbed from NWK/MAC state.
  // br_mac initializes active_nodes to 1.
  uint32_t m_activeNodesForPostTx { 1 };

  uint32_t m_maxReportedActiveNodes {0};


  /*void NoteReportedActiveNodes (uint32_t n)
  {
    if (n > m_maxReportedActiveNodes)
      {
        m_maxReportedActiveNodes = n;
      }
  }*/

  double GetPostTxWaitSeconds () const
  {
    return m_stayAwakeBaseSec
        + 1.5 * static_cast<double> (m_activeNodesForPostTx)
        + m_stayAwakeGuardSec;
  }

};

// Keep MAC-local active_nodes and device post-TX active_nodes synchronized.
// MAC uses this for OPNET get_slot() range;
// CsrNetDevice uses this for POST_TX_WAIT_TIME.
/*void
CsrMacCore::SetActiveNodesForPostTx (uint32_t n)
{
  uint32_t active = std::max<uint32_t> (1, n);

  // MAC-local reported active-node state used by PickTxSlot()
  NoteReportedActiveNodes (active);

  // Device copy used by POST_TX_WAIT_TIME
  if (m_dev != nullptr)
    {
      m_dev->SetActiveNodesForPostTx (active);
    }

  std::cout << "[MAC " << m_nodeId
            << "] active_nodes set to "
            << active
            << " for slotting and post-TX wait"
            << std::endl;
}*/

void
CsrMacCore::SetActiveNodesForPostTx (uint32_t n)
{
  uint32_t active = std::max<uint32_t> (1, n);

  // Local active_nodes used by MAC slotting.
  // Keep separate from max_reported_active_nodes.
  m_activeNodesForPostTx = active;

  // Device copy used by OPNET-style POST_TX_WAIT_TIME.
  if (m_dev != nullptr)
    {
      m_dev->SetActiveNodesForPostTx (active);
    }

  std::cout << "[MAC " << m_nodeId
            << "] active_nodes set to "
            << active
            << " for local slotting and post-TX wait"
            << std::endl;
}

void
CsrMacCore::PrintNeighbors () const
{
  double now = Simulator::Now ().GetSeconds ();
  std::cout << "[NEIGH " << m_dev->GetId () << " t=" << now << "] ";

  if (m_lastHeardSec.empty ())
    {
      std::cout << "(none)" << std::endl;
      return;
    }

  bool first = true;
  for (auto const& kv : m_lastHeardSec)
    {
      CsrNodeId n = kv.first;
      double age = now - kv.second;

      if (!first) std::cout << " | ";
      first = false;

      std::cout << "n=" << n << " age=" << age << "s";

      auto itPl = m_lastPathlossDb.find (n);
      if (itPl != m_lastPathlossDb.end ())
        std::cout << " pl=" << itPl->second << "dB";
    }
  std::cout << std::endl;
}

void
CsrMacCore::SendHelloInternal (bool reschedule)
{
  // Allow HELLO if either periodic is enabled OR discovery is active
  if ((m_dev == nullptr) || (!m_helloEnabled && !m_discoveryActive))
    {
      return;
    }

  CsrHeader h;
  h.SetSrc (m_dev->GetId ());
  h.SetDst (CSR_BROADCAST_ID);
  h.SetSeq (++m_helloSeq);
  h.SetDscp (7);
  h.SetAckable (false);
  h.SetIsAck (false);
  h.SetIsDack (false);

  h.SetType (CSR_PKT_HELLO);
  h.SetDestType (CSR_DEST_BROADCAST);
  h.SetSpeedKey (8);

  Ptr<Packet> p = Create<Packet> ();
  p->AddHeader (h);

  EnqueueTxFrame (p, CSR_BROADCAST_ID, /*dscp*/ 7, /*ackable*/ false);

  // Re-arm only if we're in periodic mode AND caller requested re-arm
  if (reschedule && m_helloEnabled)
    {
      m_helloEvent = Simulator::Schedule (m_helloInterval, &CsrMacCore::SendHello, this);
    }
}

int
CsrMacCore::SelectRateByPerTarget (CsrNodeId destId, uint32_t nBits, double targetPer) const
{
  static const std::vector<int> kRates = {
    8,
    16,
    32,
    64,
    128,
    500,
    1000
  };

  // Access PHY through the owning device (now safe because this definition is after CsrNetDevice is complete)
  const CsrPhyModel& phy = m_dev->GetPhy ();
  CsrNodeId txId = m_dev->GetId ();

  double snrDb = phy.PredictSnrDb (txId, destId, phy.profile.txPowerDbm);

  int chosen = kRates.front (); // most robust default
  for (int r : kRates)
    {
      if (r > m_maxRateKbps)
        {
          break;
        }
      double per = phy.EstimatePer (r, snrDb, nBits);
      if (per <= targetPer)
        chosen = r;
    }

  return chosen;
}

Time
CsrNetDevice::SendToPeer (Ptr<Packet> frame,
                          CsrNodeId dest,
                          int rateKbps,
                          double txPowerDbm,
                          PreambleType preamble,
                          int slot,
                          bool ackable)
{
  std::vector<Ptr<Packet>> frames;
  frames.push_back (frame);
  return SendFramesToPeers (frames,
                            rateKbps,
                            txPowerDbm,
                            preamble,
                            slot,
                            ackable);
}

Time
CsrNetDevice::SendFramesToPeers (
  const std::vector<Ptr<Packet>> &frames,
  int rateKbps,
  double txPowerDbm,
  PreambleType preamble,
  int slot,
  bool ackable)
{
  NS_ASSERT_MSG (!frames.empty (), "cannot transmit an empty CSR aggregate");
  NS_ABORT_MSG_IF (rateKbps < 0 ||
                   static_cast<uint64_t> (rateKbps) >
                     std::numeric_limits<CsrRateKey>::max () ||
                   !CsrIsOperationalRateKey (
                   static_cast<CsrRateKey> (rateKbps)),
                   "CSR transmission rate has no defined airtime or BER model");
  NS_ABORT_MSG_IF (!m_mac.IsRateEnabled (
                     static_cast<CsrRateKey> (rateKbps)),
                   "CSR transmission rate exceeds the active profile ceiling");

  uint32_t payloadBytes = 0;
  std::vector<Ptr<Packet>> frameCopies;
  frameCopies.reserve (frames.size ());
  for (const auto &segment : frames)
    {
      payloadBytes += CsrGetOpnetWireSize (segment);
      frameCopies.push_back (segment->Copy ());
    }

  // br_txdel.ps.c sends the preamble, SOF, speed, and length at S0, then
  // sends the inherited payload and 32-bit FCS at the selected payload rate.
  static constexpr double s0DurationSec = 0.00051;
  uint32_t preambleBits = preamble == PREAMBLE_LONG ? 7888 : 104;
  uint32_t s0HeaderBits = preambleBits + 16 + 16 + 16;
  uint32_t payloadAndFcsBits = payloadBytes * 8 + 32;
  uint32_t packetBits = s0HeaderBits + payloadAndFcsBits;
  double rbps = CsrRateKeyToBps (rateKbps);
  double preambleSec = preambleBits / 4.0 * s0DurationSec;
  double duration = s0HeaderBits / 4.0 * s0DurationSec
                  + payloadAndFcsBits / rbps;

  double txTime = Simulator::Now ().GetSeconds ();
  double maxPropagationDelaySec = 0.0;
  for (const auto &peer : m_peers)
    {
      if (peer == nullptr || peer->GetId () == m_id)
        {
          continue;
        }
      double distanceMeters = peer->m_phy.GetDistanceMeters (
        m_id,
        peer->GetId ());
      maxPropagationDelaySec = std::max (
        maxPropagationDelaySec,
        distanceMeters /
          CsrPhyModel::SPEED_OF_LIGHT_METERS_PER_SECOND);
    }

  // For log: peek header
  CsrHeader hdr;
  frameCopies.front ()->PeekHeader (hdr);

  std::cout   << "... type=" << unsigned(hdr.GetType())
              << " speedKey=" << unsigned(hdr.GetSpeedKey())
              << "[t=" << txTime << "] TX from node " << m_id
              << " to dest " << hdr.GetDst ()
              << " seq " << hdr.GetSeq ()
              << " size " << payloadBytes << " bytes"
              << " segments " << frameCopies.size ()
              << " at slot " << slot
              << " rate " << rateKbps << " kbps"
              << " txPower " << txPowerDbm << " dBm"
              << " (" << rbps << " bps)"
              << " preamble " << (preamble == PREAMBLE_LONG ? "LONG" : "SHORT")
              << " duration " << duration << " s"
              << std::endl;

  // OPNET br_mac.post_tx(): after the last transmission, MAC remains in
  // receive/search for POST_TX_WAIT_TIME. This is not limited to ACKable data;
  // HELLO/control transmissions also keep the node awake long enough to hear
  // follow-on traffic.
  double postTxWaitSec = GetPostTxWaitSeconds ();
  this->ForceAwakeFor (duration +
                       maxPropagationDelaySec +
                       postTxWaitSec);

  std::cout << "[MAC " << m_id
            << "] Post-TX force-awake for "
            << (duration + maxPropagationDelaySec + postTxWaitSec)
            << " s"
            << " (POST_TX_WAIT=" << postTxWaitSec << " s)"
            << std::endl;

  // Peek header once to know src/dst/seq
  CsrHeader hdrOnTx;
  frameCopies.front ()->PeekHeader (hdrOnTx);
  CsrNodeId txId = hdrOnTx.GetSrc ();
  uint16_t sequence = hdrOnTx.GetSeq ();
  uint64_t signalId = (static_cast<uint64_t> (m_id) << 32)
                    | (++m_nextTxSignalId & 0xffffffffULL);

  CsrDifferentialTraceEvent txEvent;
  txEvent.event = "tx_start";
  txEvent.node = CsrTraceInteger (m_id);
  txEvent.peer = CsrTraceInteger (hdrOnTx.GetDst ());
  txEvent.packetType = CsrPacketTypeName (hdrOnTx.GetType ());
  txEvent.source = CsrTraceInteger (hdrOnTx.GetSrc ());
  txEvent.destination = CsrTraceInteger (hdrOnTx.GetDst ());
  txEvent.sequence = CsrTraceInteger (sequence);
  txEvent.rateKbps = CsrTraceSignedInteger (rateKbps);
  txEvent.sizeBytes = CsrTraceInteger (payloadBytes);
  txEvent.reservationSlot = CsrTraceSignedInteger (slot);
  txEvent.reservationCounter = CsrTraceSignedInteger (slot);
  if (hdrOnTx.HasSecurityCount ())
    {
      txEvent.securityCount = CsrTraceInteger (
        hdrOnTx.GetSecurityCount ());
    }
  txEvent.detail = preamble == PREAMBLE_LONG ? "long" : "short";
  WriteDifferentialTrace (txEvent);

  // A direct SendToPeer call and a queued MAC transmission both occupy the
  // local half-duplex radio for the complete OTA duration.
  m_mac.NotifyPhyTxStart (Seconds (duration));

  for (const auto &peer : m_peers)
    {
      if (peer == nullptr || peer->GetId () == txId)
        {
          continue;
        }

      double distanceMeters = peer->m_phy.GetDistanceMeters (
        txId,
        peer->GetId ());
      CsrTxRadioProfile txRadio = m_phy.GetTxRadioProfile ();
      if (!peer->m_phy.HasClosure (distanceMeters, txRadio))
        {
          std::cout << "[t=" << txTime << "] RX OCCLUDED at node "
                    << peer->GetId () << " from node " << txId
                    << " seq " << sequence
                    << std::endl;
          CsrDifferentialTraceEvent dropEvent;
          dropEvent.event = "rx_drop";
          dropEvent.node = CsrTraceInteger (peer->GetId ());
          dropEvent.peer = CsrTraceInteger (txId);
          dropEvent.packetType = CsrPacketTypeName (hdrOnTx.GetType ());
          dropEvent.source = CsrTraceInteger (hdrOnTx.GetSrc ());
          dropEvent.destination = CsrTraceInteger (hdrOnTx.GetDst ());
          dropEvent.sequence = CsrTraceInteger (sequence);
          dropEvent.rateKbps = CsrTraceSignedInteger (rateKbps);
          dropEvent.sizeBytes = CsrTraceInteger (payloadBytes);
          dropEvent.success = "0";
          dropEvent.reason = "closure";
          dropEvent.detail = "distance_m=" + CsrTraceDouble (distanceMeters);
          WriteDifferentialTrace (dropEvent);
          continue;
        }
      double propagationDelaySec = distanceMeters /
        CsrPhyModel::SPEED_OF_LIGHT_METERS_PER_SECOND;

      RxSignal signal;
      signal.id = signalId;
      signal.txId = txId;
      signal.sequence = sequence;
      signal.rateKbps = rateKbps;
      signal.txPowerDbm = txPowerDbm;
      signal.preamble = preamble;
      signal.reservedSlot = slot;
      signal.ackable = ackable;
      signal.payloadBytes = payloadBytes;
      signal.packetBits = packetBits;
      signal.txRadio = txRadio;
      signal.startSec = txTime + propagationDelaySec;
      signal.endSec = signal.startSec + duration;
      signal.preambleEndSec = signal.startSec + preambleSec;
      signal.frames.reserve (frameCopies.size ());
      for (const auto &segment : frameCopies)
        {
          signal.frames.push_back (segment->Copy ());
        }

      Simulator::Schedule (Seconds (propagationDelaySec), [peer, signal] () {
        peer->BeginReceiveSignal (signal);
      });
    }

  return Seconds (duration);
}

bool
CsrNetDevice::IsPeriodicAwakeAt (double timeSec) const
{
  double offset = std::fmod (timeSec - m_wakePhaseSec, m_wakeCycleSec);
  if (offset < 0.0)
    {
      offset += m_wakeCycleSec;
    }
  return offset < m_awakeWindowSec;
}

double
CsrNetDevice::GetPeriodicWindowEnd (double timeSec) const
{
  double offset = std::fmod (timeSec - m_wakePhaseSec, m_wakeCycleSec);
  if (offset < 0.0)
    {
      offset += m_wakeCycleSec;
    }
  return IsPeriodicAwakeAt (timeSec)
    ? timeSec + m_awakeWindowSec - offset
    : timeSec;
}

double
CsrNetDevice::GetNextPeriodicWake (double timeSec) const
{
  double offset = std::fmod (timeSec - m_wakePhaseSec, m_wakeCycleSec);
  if (offset < 0.0)
    {
      offset += m_wakeCycleSec;
    }
  if (offset < m_awakeWindowSec)
    {
      return timeSec;
    }
  return timeSec + m_wakeCycleSec - offset;
}

void
CsrNetDevice::RefreshDutyState ()
{
  CsrMacCore::State state = m_mac.GetState ();
  if (state == CsrMacCore::State::TRACK ||
      state == CsrMacCore::State::TX)
    {
      return;
    }

  if (!m_dutyCycleEnabled)
    {
      m_mac.SetReceiveState (CsrMacCore::State::SEARCH);
      return;
    }

  double now = Simulator::Now ().GetSeconds ();
  // A queued frame alone does not extend a wake-only Search: the source's
  // 8.9-ms no-signal sleep can precede its 13-ms TSLOT.  Once Idle RTS or a
  // Search TSLOT has enabled PREP_TX, that preparation keeps Search active.
  bool preparationKeepsSearch =
    state == CsrMacCore::State::SEARCH &&
    m_mac.IsTxPreparationActive ();
  bool shouldSearch = preparationKeepsSearch ||
                      m_forceAwakeUntilSec > now ||
                      IsPeriodicAwakeAt (now);
  m_mac.SetReceiveState (shouldSearch
                           ? CsrMacCore::State::SEARCH
                           : CsrMacCore::State::IDLE);
}

void
CsrNetDevice::SleepReceiver ()
{
  if (m_mac.GetState () == CsrMacCore::State::TRACK ||
      m_mac.GetState () == CsrMacCore::State::TX)
    {
      return;
    }
  RefreshDutyState ();
}

void
CsrNetDevice::WakeForSignal ()
{
  m_mac.SetReceiveState (CsrMacCore::State::SEARCH);

  double now = Simulator::Now ().GetSeconds ();
  double windowEnd = GetPeriodicWindowEnd (now);
  if (windowEnd > now)
    {
      if (m_sleepEvent.IsPending ())
        {
          Simulator::Cancel (m_sleepEvent);
        }
      m_sleepEvent = Simulator::Schedule (Seconds (windowEnd - now),
                                          &CsrNetDevice::SleepReceiver,
                                          this);
    }
  ScheduleAcquisition ();
}

void
CsrNetDevice::WakeForPendingTx ()
{
  std::cout << "[MAC " << m_id
            << "] pending-TX periodic WAKE at "
            << Simulator::Now ().GetSeconds ()
            << std::endl;
  if (!m_dutyCycleEnabled || !m_mac.HasPendingFrames ())
    {
      RefreshDutyState ();
      return;
    }

  CsrMacCore::State state = m_mac.GetState ();
  if (state == CsrMacCore::State::TRACK ||
      state == CsrMacCore::State::TX)
    {
      return;
    }

  // This is the periodic WAKE -> start_search transition.  PREP_TX is not
  // activated until the first relative 13-ms TSLOT processes the queued
  // frame, matching the source state machine.
  m_mac.SetReceiveState (CsrMacCore::State::SEARCH);

  double now = Simulator::Now ().GetSeconds ();
  double windowEnd = GetPeriodicWindowEnd (now);
  if (windowEnd > now)
    {
      if (m_sleepEvent.IsPending ())
        {
          Simulator::Cancel (m_sleepEvent);
        }
      m_sleepEvent = Simulator::Schedule (
        Seconds (windowEnd - now),
        &CsrNetDevice::SleepReceiver,
        this);
    }
}

void
CsrNetDevice::ScheduleSignalWake ()
{
  if (!m_dutyCycleEnabled ||
      m_mac.GetState () != CsrMacCore::State::IDLE ||
      m_signalWakeEvent.IsPending ())
    {
      return;
    }

  double now = Simulator::Now ().GetSeconds ();
  double wakeTime = GetNextPeriodicWake (now);
  bool canAcquire = false;
  for (const auto &[id, signal] : m_rxSignals)
    {
      (void) id;
      if (signal.syncEligible &&
          signal.preambleActive &&
          !signal.rejected &&
          wakeTime + m_syncToTrackSec < signal.preambleEndSec)
        {
          canAcquire = true;
          break;
        }
    }

  if (canAcquire)
    {
      m_signalWakeEvent = Simulator::Schedule (
        Seconds (std::max (0.0, wakeTime - now)),
        &CsrNetDevice::WakeForSignal,
        this);
    }
}

void
CsrNetDevice::SchedulePendingTxWake ()
{
  if (!m_dutyCycleEnabled ||
      m_mac.GetState () != CsrMacCore::State::IDLE ||
      !m_mac.HasPendingFrames () ||
      m_pendingTxWakeEvent.IsPending ())
    {
      return;
    }

  double now = Simulator::Now ().GetSeconds ();
  double wakeTime = GetNextPeriodicWake (now);
  std::cout << "[MAC " << m_id
            << "] Idle RTS defers to nearby periodic WAKE at "
            << wakeTime
            << std::endl;
  m_pendingTxWakeEvent = Simulator::Schedule (
    Seconds (std::max (0.0, wakeTime - now)),
    &CsrNetDevice::WakeForPendingTx,
    this);
}

void
CsrNetDevice::UpdateSyncPresence ()
{
  bool present = false;
  for (const auto &[id, signal] : m_rxSignals)
    {
      (void) id;
      if (signal.syncEligible && signal.preambleActive)
        {
          present = true;
          break;
        }
    }
  m_mac.SetSyncPresent (present);
}

void
CsrNetDevice::ScheduleAcquisition ()
{
  if (m_mac.GetState () != CsrMacCore::State::SEARCH ||
      !m_mac.IsSyncPresent () ||
      m_acquisitionEvent.IsPending ())
    {
      return;
    }

  m_acquisitionEvent = Simulator::Schedule (Seconds (m_syncToTrackSec),
                                             &CsrNetDevice::AcquireSignal,
                                             this);
}

void
CsrNetDevice::UpdateSignalNoise (RxSignal &signal)
{
  double totalNoiseWatts = signal.frontEnd.backgroundNoiseWatts +
    signal.currentInterferenceWatts;
  signal.peakNoiseWatts = std::max (signal.peakNoiseWatts,
                                    totalNoiseWatts);
  signal.snrDb = CsrPhyModel::ComputeSnrDb (
    signal.frontEnd.receivedPowerWatts,
    totalNoiseWatts);
}

void
CsrNetDevice::CloseSignalInterval (RxSignal &signal, double endSec)
{
  double boundedEndSec = std::min (endSec, signal.endSec);
  if (boundedEndSec <= signal.intervalStartSec)
    {
      return;
    }

  CsrBerInterval interval;
  interval.startSec = signal.intervalStartSec;
  interval.endSec = boundedEndSec;
  interval.noisePowerWatts = signal.frontEnd.backgroundNoiseWatts +
    signal.currentInterferenceWatts;
  interval.collisionCount = signal.collisionCount;
  // Preserve br_ber's receiver-global last-collision state for the S0 header
  // and supplied spread-rate curves.  The high-rate payload fallback carries
  // a separate, signed per-signal jammer ratio so a stronger or shorter
  // jammer is represented only over the interval where it is active.
  interval.sameRateInterference = m_rxSameSpeed;
  interval.jsrDb = m_rxJsrDb;
  interval.timeOffsetSeconds = m_rxTimeOffsetSeconds;
  if (CsrIsHighRateDifferentialMode (signal.rateKbps))
    {
      interval.highRatePayloadJammer = signal.sameRateInterference;
      interval.highRatePayloadJsrDb = signal.jsrDb;
    }
  signal.berIntervals.push_back (interval);
  uint32_t preambleBits = signal.preamble == PREAMBLE_LONG ? 7888 : 104;
  CsrErrorAllocation allocated = m_phy.AllocateErrors (
    signal.frontEnd,
    signal.startSec,
    preambleBits,
    signal.packetBits,
    signal.rateKbps,
    {interval},
    m_rng);
  signal.errorAllocation.headerBits += allocated.headerBits;
  signal.errorAllocation.payloadBits += allocated.payloadBits;
  signal.errorAllocation.headerErrors += allocated.headerErrors;
  signal.errorAllocation.payloadErrors += allocated.payloadErrors;
  signal.errorAllocation.totalErrors += allocated.totalErrors;
  signal.errorAllocation.actualBer = allocated.actualBer;
  signal.errorAllocation.headerBer = allocated.headerBer;
  signal.errorAllocation.payloadBer = allocated.payloadBer;
  signal.errorAllocation.minimumSnrDb = std::min (
    signal.errorAllocation.minimumSnrDb,
    allocated.minimumSnrDb);
  signal.errorAllocation.peakNoisePowerWatts = std::max (
    signal.errorAllocation.peakNoisePowerWatts,
    allocated.peakNoisePowerWatts);
  signal.errorAllocation.packetErrorProbability = 1.0 -
    (1.0 - signal.errorAllocation.packetErrorProbability) *
    (1.0 - allocated.packetErrorProbability);
  signal.intervalStartSec = boundedEndSec;
}

void
CsrNetDevice::CloseAllSignalIntervals (double endSec)
{
  for (auto &[id, signal] : m_rxSignals)
    {
      (void) id;
      CloseSignalInterval (signal, endSec);
    }
}

void
CsrNetDevice::StartAllSignalIntervals (double startSec)
{
  for (auto &[id, signal] : m_rxSignals)
    {
      (void) id;
      signal.intervalStartSec = std::min (startSec, signal.endSec);
    }
}

void
CsrNetDevice::RecordSameRateInterference (RxSignal &desired,
                                           const RxSignal &jammer)
{
  if (!desired.frontEnd.channelMatched ||
      !jammer.frontEnd.channelMatched ||
      desired.frontEnd.receivedPowerWatts <= 0.0 ||
      jammer.frontEnd.receivedPowerWatts <= 0.0)
    {
      return;
    }

  double jsrDb = jammer.rxPowerDbm - desired.rxPowerDbm;
  if (!desired.sameRateInterference || jsrDb > desired.jsrDb)
    {
      desired.sameRateInterference = true;
      desired.jsrDb = jsrDb;
      desired.timeOffsetSeconds = jammer.startSec - desired.startSec;
    }
}

void
CsrNetDevice::RefreshHighRateInterference ()
{
  // The fallback is interval-local, unlike br_ber's receiver-global
  // diagnostic collision record.  Rebuild each active high-rate signal from
  // the currently overlapping signals whenever an interference boundary
  // removes a jammer.
  for (auto &[desiredId, desired] : m_rxSignals)
    {
      (void) desiredId;
      if (!CsrIsHighRateDifferentialMode (desired.rateKbps))
        {
          continue;
        }
      desired.sameRateInterference = false;
      desired.jsrDb = -std::numeric_limits<double>::infinity ();
      desired.timeOffsetSeconds = 0.0;
    }

  for (auto desiredIt = m_rxSignals.begin ();
       desiredIt != m_rxSignals.end ();
       ++desiredIt)
    {
      RxSignal &desired = desiredIt->second;
      if (!CsrIsHighRateDifferentialMode (desired.rateKbps))
        {
          continue;
        }
      for (auto jammerIt = m_rxSignals.begin ();
           jammerIt != m_rxSignals.end ();
           ++jammerIt)
        {
          if (jammerIt == desiredIt ||
              jammerIt->second.rateKbps != desired.rateKbps)
            {
              continue;
            }
          RecordSameRateInterference (desired, jammerIt->second);
        }
    }
}

void
CsrNetDevice::ApplyInterferencePair (RxSignal &first,
                                      RxSignal &second)
{
  double now = Simulator::Now ().GetSeconds ();
  if (!first.frontEnd.channelMatched ||
      !second.frontEnd.channelMatched ||
      first.endSec <= now ||
      second.endSec <= now)
    {
      return;
    }

  first.collisionCount++;
  second.collisionCount++;
  if (first.rateKbps == second.rateKbps)
    {
      m_rxSameSpeed = true;
      const RxSignal &stronger = first.rxPowerDbm > second.rxPowerDbm
        ? first
        : second;
      const RxSignal &weaker = first.rxPowerDbm > second.rxPowerDbm
        ? second
        : first;
      m_rxJsrDb = weaker.rxPowerDbm - stronger.rxPowerDbm;
      m_rxTimeOffsetSeconds = weaker.startSec - stronger.startSec;
      RecordSameRateInterference (first, second);
      RecordSameRateInterference (second, first);
      return;
    }

  m_rxSameSpeed = false;
  first.currentInterferenceWatts +=
    second.frontEnd.receivedPowerWatts;
  second.currentInterferenceWatts +=
    first.frontEnd.receivedPowerWatts;
  UpdateSignalNoise (first);
  UpdateSignalNoise (second);
}

void
CsrNetDevice::RemoveEndedInterference (const RxSignal &ended)
{
  for (auto &[signalId, signal] : m_rxSignals)
    {
      if (signalId == ended.id || signal.rateKbps == ended.rateKbps)
        {
          continue;
        }
      signal.currentInterferenceWatts = std::max (
        0.0,
        signal.currentInterferenceWatts -
          ended.frontEnd.receivedPowerWatts);
      UpdateSignalNoise (signal);
    }
}

void
CsrNetDevice::ApplyTrackedInterference (RxSignal &interferer)
{
  auto trackedIt = m_rxSignals.find (m_trackedSignalId);
  if (trackedIt == m_rxSignals.end ())
    {
      return;
    }

  RxSignal &tracked = trackedIt->second;
  interferer.rejected = true;
  if (!interferer.frontEnd.channelMatched ||
      tracked.rateKbps != interferer.rateKbps)
    {
      return;
    }

  double desiredMarginDb = tracked.rxPowerDbm - interferer.rxPowerDbm;
  if (desiredMarginDb >= m_captureMarginDb)
    {
      m_rxCaptureCount++;
    }
  else
    {
      tracked.collided = true;
    }
}

void
CsrNetDevice::BeginReceiveSignal (RxSignal signal)
{
  double now = Simulator::Now ().GetSeconds ();
  signal.arrivalOrder = ++m_nextRxArrivalOrder;
  double distanceMeters = m_phy.GetDistanceMeters (signal.txId, m_id);
  signal.frontEnd = m_phy.ComputeFrontEnd (
    signal.txPowerDbm,
    distanceMeters,
    signal.txRadio);
  signal.pathlossDb = signal.frontEnd.pathlossDb;
  signal.rxPowerDbm = signal.frontEnd.receivedPowerDbm;
  signal.intervalStartSec = now;
  UpdateSignalNoise (signal);

  bool hadSync = m_mac.IsSyncPresent ();
  CloseAllSignalIntervals (now);
  auto [it, inserted] = m_rxSignals.emplace (signal.id, std::move (signal));
  NS_ASSERT_MSG (inserted, "duplicate CSR receive signal identifier");
  RxSignal &incoming = it->second;

  for (auto &[otherId, other] : m_rxSignals)
    {
      if (otherId != incoming.id)
        {
          ApplyInterferencePair (incoming, other);
        }
    }
  StartAllSignalIntervals (now);
  incoming.syncEligible = incoming.frontEnd.channelMatched &&
    incoming.snrDb >= m_phy.profile.syncSnrThresholdDb;

  Simulator::Schedule (
    Seconds (std::max (0.0, incoming.preambleEndSec - now)),
    &CsrNetDevice::EndReceivePreamble,
    this,
    incoming.id);
  Simulator::Schedule (
    Seconds (std::max (0.0, incoming.endSec - now)),
    &CsrNetDevice::EndReceiveSignal,
    this,
    incoming.id);

  CsrMacCore::State state = m_mac.GetState ();
  if (state == CsrMacCore::State::TX)
    {
      incoming.missedByState = true;
      if (incoming.syncEligible)
        {
          m_rxCollisionCount++;
        }
    }
  else if (state == CsrMacCore::State::TRACK)
    {
      if (incoming.syncEligible)
        {
          m_rxCollisionCount++;
        }
      ApplyTrackedInterference (incoming);
    }
  else if (incoming.syncEligible && hadSync)
    {
      m_rxCollisionCount++;
    }

  UpdateSyncPresence ();
  RefreshDutyState ();

  if (m_mac.GetState () == CsrMacCore::State::SEARCH)
    {
      ScheduleAcquisition ();
    }
  else if (m_mac.GetState () == CsrMacCore::State::IDLE)
    {
      incoming.missedByState = true;
      ScheduleSignalWake ();
    }

  std::cout << "[t=" << now << "] RX SYNC at node " << m_id
            << " from node " << incoming.txId
            << " seq " << incoming.sequence
            << " power " << incoming.rxPowerDbm << " dBm"
            << " noise "
            << CsrPhyModel::WattsToDbm (
                 incoming.frontEnd.backgroundNoiseWatts +
                 incoming.currentInterferenceWatts)
            << " dBm"
            << " snr " << incoming.snrDb << " dB"
            << " state=" << static_cast<int> (m_mac.GetState ())
            << (incoming.frontEnd.channelMatched
                  ? (incoming.syncEligible ? "" : " below-threshold")
                  : " channel-mismatch")
            << std::endl;
}

void
CsrNetDevice::EndReceivePreamble (uint64_t signalId)
{
  auto it = m_rxSignals.find (signalId);
  if (it == m_rxSignals.end ())
    {
      return;
    }

  it->second.preambleActive = false;

  // clear_sync() cancels the outstanding RX_SIG_FOUND event whenever any
  // preamble expires.  Preserve that ordering, including the multi-signal
  // behavior of the supplied process model.
  if (m_acquisitionEvent.IsPending ())
    {
      Simulator::Cancel (m_acquisitionEvent);
    }
  UpdateSyncPresence ();
}

void
CsrNetDevice::AcquireSignal ()
{
  if (m_mac.GetState () != CsrMacCore::State::SEARCH)
    {
      return;
    }

  double now = Simulator::Now ().GetSeconds ();
  std::vector<RxSignal *> candidates;
  for (auto &[id, signal] : m_rxSignals)
    {
      (void) id;
      if (signal.syncEligible &&
          signal.preambleActive &&
          !signal.rejected)
        {
          candidates.push_back (&signal);
        }
    }

  if (candidates.empty ())
    {
      return;
    }

  std::sort (candidates.begin (),
             candidates.end (),
             [] (const RxSignal *left, const RxSignal *right) {
               if (left->startSec != right->startSec)
                 {
                   return left->startSec < right->startSec;
                 }
               return left->arrivalOrder < right->arrivalOrder;
             });

  RxSignal *selected = candidates.front ();
  for (std::size_t index = 1; index < candidates.size (); ++index)
    {
      RxSignal *candidate = candidates[index];
      if (now - candidate->startSec > m_syncToTrackSec &&
          candidate->rxPowerDbm > selected->rxPowerDbm)
        {
          selected = candidate;
        }
    }

  selected->tracked = true;
  selected->missedByState = false;
  m_trackedSignalId = selected->id;

  double strongestSameRateJammerDbm =
    -std::numeric_limits<double>::infinity ();
  for (auto &[id, other] : m_rxSignals)
    {
      if (id == selected->id || other.endSec <= now)
        {
          continue;
        }
      if (other.syncEligible && other.preambleActive)
        {
          other.rejected = true;
        }
      if (other.rateKbps == selected->rateKbps &&
          other.frontEnd.channelMatched)
        {
          strongestSameRateJammerDbm = std::max (
            strongestSameRateJammerDbm,
            other.rxPowerDbm);
        }
    }

  if (std::isfinite (strongestSameRateJammerDbm))
    {
      if (selected->rxPowerDbm - strongestSameRateJammerDbm >=
          m_captureMarginDb)
        {
          m_rxCaptureCount++;
        }
      else
        {
          selected->collided = true;
        }
    }

  if (m_sleepEvent.IsPending ())
    {
      Simulator::Cancel (m_sleepEvent);
    }
  m_mac.SetReceiveState (CsrMacCore::State::TRACK);

  std::cout << "[t=" << now << "] MAC " << m_id
            << " enters Track on node " << selected->txId
            << " seq " << selected->sequence
            << " power " << selected->rxPowerDbm << " dBm"
            << (selected->collided ? " collided" : "")
            << std::endl;
}

void
CsrNetDevice::DeliverTrackedSignal (const RxSignal &signal,
                                    const CsrRxDecision &decision)
{
  bool addressedFrame = false;
  bool ackableForPeer = false;
  for (const auto &segment : signal.frames)
    {
      CsrHeader header;
      segment->PeekHeader (header);
      if (header.IsForDestination (m_id))
        {
          addressedFrame = true;
          ackableForPeer = ackableForPeer || header.IsAckable ();
        }
    }

  double now = Simulator::Now ().GetSeconds ();
  std::cout << "[t=" << now << "] "
            << (addressedFrame ? "RX" : "RX OVERHEARD")
            << " at node " << m_id
            << " from node " << signal.txId
            << " seq " << signal.sequence
            << " rate " << signal.rateKbps << " kbps"
            << " txPower " << signal.txPowerDbm << " dBm"
            << " preamble "
            << (signal.preamble == PREAMBLE_LONG ? "LONG" : "SHORT")
            << " pathloss " << decision.pathlossDb << " dB"
            << " snr " << decision.snrDb << " dB"
            << " (slot " << signal.reservedSlot
            << ", len " << signal.payloadBytes << "B"
            << ", ackable=" << (signal.ackable ? 1 : 0) << ")"
            << std::endl;

  if (ackableForPeer)
    {
      ForceAwakeFor (0.35);
    }

  m_mac.NoteHeardFrom (signal.txId, now);
  m_mac.SetNeighborPathloss (signal.txId, decision.pathlossDb);
  if (signal.reservedSlot >= 0)
    {
      m_mac.NoteNeighborReservedSlot (signal.txId,
                                      signal.reservedSlot);
    }

  // OPNET MAC forwards every decoded segment to HOP and lets HOP perform
  // destination filtering after recording the source-neighbor observation.
  for (const auto &segment : signal.frames)
    {
      m_mac.DeliverRxFrameToUp (segment->Copy (),
                                decision.pathlossDb,
                                decision.snrDb);
    }
}

void
CsrNetDevice::EndReceiveSignal (uint64_t signalId)
{
  auto it = m_rxSignals.find (signalId);
  if (it == m_rxSignals.end ())
    {
      return;
    }

  double now = Simulator::Now ().GetSeconds ();
  CloseAllSignalIntervals (now);
  RxSignal signal = it->second;
  bool wasTracked = signalId == m_trackedSignalId;
  if (wasTracked)
    {
      uint32_t preambleBits = signal.preamble == PREAMBLE_LONG
        ? 7888
        : 104;
      bool stateAllowsDelivery =
        m_mac.GetState () == CsrMacCore::State::TRACK;
      bool priorAccepted = stateAllowsDelivery && !signal.rejected;
      CsrRxDecision decision = m_phy.EvaluateAllocatedRx (
        signal.frontEnd,
        preambleBits,
        signal.packetBits,
        signal.errorAllocation,
        signal.berIntervals,
        priorAccepted,
        false,
        false);
      decision.closure = true;
      if (m_phy.UsesCustomErrorModel ())
        {
          // Existing MAC-state tests use a zero-error hook to isolate the
          // pre-table collision gate from stochastic PHY behavior.
          decision.success = decision.success && !signal.collided;
        }
      m_lastRxDecision = decision;
      m_hasLastRxDecision = true;
      if (decision.eccDropped)
        {
          m_rxEccDropCount++;
        }

      if (g_rxCsv.is_open ())
        {
          g_rxCsv << now << ","
                  << signal.txId << ","
                  << m_id << ","
                  << signal.sequence << ","
                  << signal.rateKbps << ","
                  << signal.packetBits << ","
                  << signal.frontEnd.distanceMeters << ","
                  << static_cast<int> (decision.pathModel) << ","
                  << signal.frontEnd.bandOverlapHz << ","
                  << decision.pathlossDb << ","
                  << decision.receivedPowerDbm << ","
                  << decision.noisePowerDbm << ","
                  << decision.snrDb << ","
                  << (decision.sameRateInterference ? 1 : 0) << ","
                  << decision.jsrDb << ","
                  << decision.timeOffsetSeconds << ","
                  << decision.per << ","
                  << decision.headerBer << ","
                  << decision.payloadBer << ","
                  << decision.actualBer << ","
                  << decision.headerErrors << ","
                  << decision.payloadErrors << ","
                  << decision.totalErrors << ","
                  << decision.protectedBits << ","
                  << decision.correctableBits << ","
                  << (decision.eccDropped ? 1 : 0) << ","
                  << (decision.success ? 1 : 0)
                  << "\n";
        }

      CsrHeader traceHeader;
      signal.frames.front ()->PeekHeader (traceHeader);
      CsrDifferentialTraceEvent rxEvent;
      rxEvent.event = decision.success ? "rx_accept" : "rx_drop";
      rxEvent.node = CsrTraceInteger (m_id);
      rxEvent.peer = CsrTraceInteger (signal.txId);
      rxEvent.packetType = CsrPacketTypeName (traceHeader.GetType ());
      rxEvent.source = CsrTraceInteger (traceHeader.GetSrc ());
      rxEvent.destination = CsrTraceInteger (traceHeader.GetDst ());
      rxEvent.sequence = CsrTraceInteger (signal.sequence);
      rxEvent.rateKbps = CsrTraceSignedInteger (signal.rateKbps);
      rxEvent.sizeBytes = CsrTraceInteger (signal.payloadBytes);
      rxEvent.success = decision.success ? "1" : "0";
      rxEvent.reason = decision.success
        ? "accepted"
        : (decision.eccDropped
             ? "ecc"
             : (signal.collided
                  ? "collision"
                  : (stateAllowsDelivery ? "phy" : "state")));
      rxEvent.pathlossDb = CsrTraceDouble (decision.pathlossDb);
      rxEvent.rxPowerDbm = CsrTraceDouble (decision.receivedPowerDbm);
      rxEvent.noiseDbm = CsrTraceDouble (decision.noisePowerDbm);
      rxEvent.snrDb = CsrTraceDouble (decision.snrDb);
      rxEvent.jsrDb = CsrTraceDouble (decision.jsrDb);
      rxEvent.headerErrors = CsrTraceInteger (decision.headerErrors);
      rxEvent.payloadErrors = CsrTraceInteger (decision.payloadErrors);
      rxEvent.totalErrors = CsrTraceInteger (decision.totalErrors);
      rxEvent.detail = "collisions=" + CsrTraceInteger (
        signal.collisionCount);
      if (traceHeader.HasSecurityCount ())
        {
          rxEvent.securityCount = CsrTraceInteger (
            traceHeader.GetSecurityCount ());
        }
      WriteDifferentialTrace (rxEvent);

      if (decision.success)
        {
          DeliverTrackedSignal (signal, decision);
        }
      else
        {
          std::cout << "[t=" << Simulator::Now ().GetSeconds ()
                    << "] RX DROP at node " << m_id
                    << " from node " << signal.txId
                    << " seq " << signal.sequence
                    << (signal.collided ? " (collision)" : " (PHY/state)")
                    << std::endl;
        }

      m_trackedSignalId = 0;
    }
  else if (signal.missedByState)
    {
      m_rxMissCount++;
      CsrHeader traceHeader;
      signal.frames.front ()->PeekHeader (traceHeader);
      CsrDifferentialTraceEvent missEvent;
      missEvent.event = "rx_drop";
      missEvent.node = CsrTraceInteger (m_id);
      missEvent.peer = CsrTraceInteger (signal.txId);
      missEvent.packetType = CsrPacketTypeName (traceHeader.GetType ());
      missEvent.source = CsrTraceInteger (traceHeader.GetSrc ());
      missEvent.destination = CsrTraceInteger (traceHeader.GetDst ());
      missEvent.sequence = CsrTraceInteger (signal.sequence);
      missEvent.rateKbps = CsrTraceSignedInteger (signal.rateKbps);
      missEvent.sizeBytes = CsrTraceInteger (signal.payloadBytes);
      missEvent.success = "0";
      missEvent.reason = "state";
      missEvent.detail = "collisions=" + CsrTraceInteger (
        signal.collisionCount);
      WriteDifferentialTrace (missEvent);
      std::cout << "[t=" << Simulator::Now ().GetSeconds ()
                << "] RX MISS at node " << m_id
                << " from node " << signal.txId
                << " seq " << signal.sequence
                << std::endl;
    }
  else if (signal.collisionCount > 0)
    {
      // Modeler's receiver pipeline still produces a rejected ECC outcome for
      // the colliding signal that did not win acquisition. Record the same
      // prior-stage outcome without changing ns-3 delivery behavior.
      CsrHeader traceHeader;
      signal.frames.front ()->PeekHeader (traceHeader);
      CsrDifferentialTraceEvent collisionEvent;
      collisionEvent.event = "rx_drop";
      collisionEvent.node = CsrTraceInteger (m_id);
      collisionEvent.peer = CsrTraceInteger (signal.txId);
      collisionEvent.packetType = CsrPacketTypeName (traceHeader.GetType ());
      collisionEvent.source = CsrTraceInteger (traceHeader.GetSrc ());
      collisionEvent.destination = CsrTraceInteger (traceHeader.GetDst ());
      collisionEvent.sequence = CsrTraceInteger (signal.sequence);
      collisionEvent.rateKbps = CsrTraceSignedInteger (signal.rateKbps);
      collisionEvent.sizeBytes = CsrTraceInteger (signal.payloadBytes);
      collisionEvent.success = "0";
      collisionEvent.reason = "prior_stage";
      collisionEvent.pathlossDb = CsrTraceDouble (signal.pathlossDb);
      collisionEvent.rxPowerDbm = CsrTraceDouble (signal.rxPowerDbm);
      collisionEvent.snrDb = CsrTraceDouble (signal.snrDb);
      collisionEvent.jsrDb = CsrTraceDouble (signal.jsrDb);
      collisionEvent.detail = "collisions=" + CsrTraceInteger (
        signal.collisionCount);
      WriteDifferentialTrace (collisionEvent);
    }

  RemoveEndedInterference (signal);
  m_rxSignals.erase (it);
  RefreshHighRateInterference ();
  StartAllSignalIntervals (now);
  UpdateSyncPresence ();

  if (wasTracked && m_mac.GetState () != CsrMacCore::State::TX)
    {
      m_mac.SetReceiveState (CsrMacCore::State::SEARCH);
      ScheduleAcquisition ();
    }
  RefreshDutyState ();
}

void
CsrNetDevice::OnMacTxFinished ()
{
  UpdateSyncPresence ();
  ScheduleAcquisition ();
  RefreshDutyState ();
}

// ------------------------------------------------------------
// CsrMacCore implementation
// ------------------------------------------------------------

bool
CsrMacCore::HasPendingFrame () const
{
  return !m_ackQueue.empty () || !m_queue.empty ();
}

void
CsrMacCore::EnqueueAckFrame (Ptr<Packet> frame,
                             const CsrHeader &header)
{
  CsrNodeId dest = header.GetDst ();
  uint16_t seq = header.GetSeq ();

  if (header.HasAckWindow ())
    {
      // OPNET replaces the first cumulative ACK/DACK entry for a destination
      // and restarts its transmission count.  It does this even when the ACK
      // queue is already at its capacity.
      for (auto &entry : m_ackQueue)
        {
          if (entry.dest == dest)
            {
              entry.frame = frame;
              entry.seq = seq;
              entry.hasWindow = true;
              entry.txCount = 0;

              std::cout << "[MAC " << m_nodeId
                        << "] Update cumulative ACK queue entry"
                        << " dest=" << dest
                        << " baseSeq=" << seq
                        << " txCount=0"
                        << std::endl;

              MaybeScheduleNextTx ();
              return;
            }
        }
    }
  else
    {
      // Exact ACKs are retained only once for a destination/sequence pair.
      for (const auto &entry : m_ackQueue)
        {
          if (entry.dest == dest && entry.seq == seq)
            {
              std::cout << "[MAC " << m_nodeId
                        << "] Ignore duplicate exact ACK"
                        << " dest=" << dest
                        << " seq=" << seq
                        << std::endl;
              return;
            }
        }
    }

  if (m_ackQueue.size () >= ACK_QUEUE_SIZE)
    {
      std::cout << "[MAC " << m_nodeId
                << "] Drop ACK: OPNET ACK queue limit reached"
                << " dest=" << dest
                << " seq=" << seq
                << " limit=" << ACK_QUEUE_SIZE
                << std::endl;
      return;
    }

  AckQueueEntry entry;
  entry.frame = frame;
  entry.dest = dest;
  entry.seq = seq;
  entry.hasWindow = header.HasAckWindow ();
  entry.txCount = 0;
  m_ackQueue.push_back (entry);

  std::cout << "[MAC " << m_nodeId
            << "] Enqueue ACK"
            << " dest=" << dest
            << " seq=" << seq
            << " window=" << (entry.hasWindow ? 1 : 0)
            << " ack_queue_size=" << m_ackQueue.size ()
            << std::endl;

  MaybeScheduleNextTx ();
}

void
CsrMacCore::EnqueueTxFrame (Ptr<Packet> frame,
                            CsrNodeId dest,
                            uint8_t dscp,
                            bool ackable)
{
  CsrAnnotateOpnetEnvelope (frame);

  CsrHeader header;
  if (frame != nullptr && frame->PeekHeader (header) &&
      (header.IsAck () || header.IsDack () ||
       header.GetType () == CSR_PKT_ACK ||
       header.GetType () == CSR_PKT_DACK))
    {
      EnqueueAckFrame (frame, header);
      return;
    }

  // OPNET checks the non-ACK Tx queue limit before DSCP insertion.  A newly
  // arriving high-priority frame is dropped rather than displacing an older
  // lower-priority frame already occupying one of the 512 entries.
  if (m_queue.size () >= TX_QUEUE_SIZE)
    {
      m_dataQueueDropCount++;

      std::cout << "[MAC " << m_nodeId
                << "] Drop TX: OPNET data queue limit reached"
                << " dest=" << dest
                << " dscp=" << unsigned (dscp)
                << " limit=" << TX_QUEUE_SIZE
                << " drops=" << m_dataQueueDropCount
                << std::endl;
      return;
    }

  TxQueueEntry e;
  e.frame   = frame;
  e.dest    = dest;
  e.dscp    = dscp;
  e.ackable = ackable;

  // DSCP priority insert: higher dscp earlier
  auto it = m_queue.begin ();
  for (; it != m_queue.end (); ++it)
    {
      if (dscp > it->dscp)
        {
          break;
        }
    }

  std::cout << "[MAC] Enqueue TX pkt"
            << " dscp=" << int(dscp)
            << " insert_pos=" << std::distance (m_queue.begin (), it)
            << " queue_size_before=" << m_queue.size ()
            << std::endl;

  m_queue.insert (it, e);
  MaybeScheduleNextTx ();
}

uint32_t
CsrMacCore::CancelAcknowledgedFrames (CsrNodeId neighbor,
                                      uint16_t baseSeq,
                                      uint64_t ackBitmap,
                                      uint64_t dackBitmap)
{
  // DACK is also terminal at MAC: HOP moves that packet to its delayed DACK
  // list, so another queued over-the-air copy must not be transmitted.
  uint64_t completedBitmap = ackBitmap | dackBitmap;
  uint32_t removed = 0;

  for (auto it = m_queue.begin (); it != m_queue.end (); )
    {
      if (!it->ackable || it->dest != neighbor || it->frame == nullptr)
        {
          ++it;
          continue;
        }

      CsrHeader queuedHeader;
      if (!it->frame->PeekHeader (queuedHeader))
        {
          ++it;
          continue;
        }

      // OPNET's exact-ACK MAC cleanup compares the outer integer
      // destination.  A routing-control packet with a destination structure
      // is therefore left intact, even after one member of the group ACKs.
      if (queuedHeader.HasDestinationSequences ())
        {
          ++it;
          continue;
        }

      // Unsigned subtraction gives the sequence's age modulo 2^16, matching
      // the 16-bit legacy sequence space even across wraparound.
      uint16_t age = static_cast<uint16_t> (
        baseSeq - queuedHeader.GetSeq ());

      if (age < 64 && (completedBitmap & (1ULL << age)) != 0)
        {
          std::cout << "[MAC " << m_nodeId
                    << "] Cancel queued acknowledged frame"
                    << " neighbor=" << neighbor
                    << " seq=" << queuedHeader.GetSeq ()
                    << " baseSeq=" << baseSeq
                    << " windowIndex=" << age
                    << std::endl;

          it = m_queue.erase (it);
          removed++;
          continue;
        }

      ++it;
    }

  return removed;
}

uint32_t
CsrMacCore::CancelQueuedFramesByType (CsrNodeId neighbor, uint8_t type)
{
  uint32_t removed = 0;

  for (auto it = m_queue.begin (); it != m_queue.end (); )
    {
      CsrHeader header;
      if (it->dest == neighbor &&
          it->frame != nullptr &&
          it->frame->PeekHeader (header) &&
          header.GetType () == type)
        {
          it = m_queue.erase (it);
          removed++;
          continue;
        }

      ++it;
    }

  if (removed > 0)
    {
      std::cout << "[MAC " << m_nodeId
                << "] Replace immediate-tag frame"
                << " neighbor=" << neighbor
                << " type=" << unsigned (type)
                << " removed=" << removed
                << std::endl;
    }

  return removed;
}

void
CsrMacCore::MaybeScheduleNextTx ()
{
  if (m_dev == nullptr)
    {
      return;
    }

  if (!HasPendingFrame ())
    {
      return;
    }

  // A frame queued during Tx is picked up by FinishTx().  OPNET returns to
  // Search before resuming the already-cleared holdoff/countdown process.
  if (m_txInProgress)
    {
      return;
    }

  if (m_state == State::IDLE)
    {
      // Idle's enter executive schedules an untracked RTS TSLOT on the next
      // simulation-wide 13-ms boundary, unless that boundary is too close to
      // the periodic WAKE.  HOP_PK_RCVD itself only queues the frame.
      ScheduleIdleRts ();
      return;
    }
  if (!m_slotTickEnabled)
    {
      // Direct-device tests begin in Search rather than passing through the
      // OPNET Idle -> Search transition.  Initialize the same independent
      // slot and holdoff timers on their first transmit request.
      StartSearchTiming ();
    }

  if (!m_txHoldoffOver && !m_txHoldoffEvent.IsPending ())
    {
      StartTxHoldoff ();
    }

  // Search-state arrivals are activated by the next shared TSLOT, after its
  // reservation advancement, matching tslot_tasks() ordering.
}

void
CsrMacCore::StartSearchTiming ()
{
  if (!m_slotTickEnabled)
    {
      StartSlotTick (m_slotTickPeriod);
    }
  StartTxHoldoff ();
}

void
CsrMacCore::ScheduleIdleRts ()
{
  if (m_state != State::IDLE ||
      !HasPendingFrame () ||
      m_idleRtsEvent.IsPending ())
    {
      return;
    }

  int64_t nowSteps = Simulator::Now ().GetTimeStep ();
  int64_t periodSteps = m_slotTickPeriod.GetTimeStep ();
  NS_ABORT_MSG_IF (periodSteps <= 0,
                   "CSR MAC slot period must be positive");
  Time nextSlot = TimeStep (
    (nowSteps / periodSteps + 1) * periodSteps);

  bool deferToWake = false;
  if (m_dev != nullptr && m_dev->IsDutyCyclingEnabled ())
    {
      double nextWakeSec = m_dev->GetNextPeriodicWake (
        Simulator::Now ().GetSeconds ());
      deferToWake = std::fabs (nextWakeSec - nextSlot.GetSeconds ()) <=
        m_slotTickPeriod.GetSeconds () / 2.0;
    }

  if (deferToWake)
    {
      m_dev->SchedulePendingTxWake ();
      return;
    }

  std::cout << "[MAC " << m_nodeId
            << "] Idle queue schedules RTS TSLOT at "
            << nextSlot.GetSeconds ()
            << std::endl;
  m_idleRtsEvent = Simulator::Schedule (
    nextSlot - Simulator::Now (),
    &CsrMacCore::IdleRts,
    this);
}

void
CsrMacCore::IdleRts ()
{
  if (m_state != State::IDLE || !HasPendingFrame ())
    {
      return;
    }

  // Idle --RTS--> Search runs prep_tx(): restart the relative slot timer and
  // independent holdoff, enable PREP_TX immediately, and redraw <= 0.
  SetReceiveState (State::SEARCH);
  ActivateTxPreparation (true);
}

void
CsrMacCore::StartTxHoldoff ()
{
  if (m_txHoldoffEvent.IsPending ())
    {
      Simulator::Cancel (m_txHoldoffEvent);
    }

  m_txHoldoffOver = false;
  m_txHoldoffEvent = Simulator::Schedule (
    Seconds (TS_HOLDOFF_SECONDS),
    &CsrMacCore::TxHoldoffExpired,
    this);
}

void
CsrMacCore::TxHoldoffExpired ()
{
  m_txHoldoffOver = true;

  CsrDifferentialTraceEvent event;
  event.event = "reservation_holdoff";
  event.node = CsrTraceInteger (m_nodeId);
  if (m_scheduledTxSlot >= 0)
    {
      event.reservationSlot = CsrTraceSignedInteger (m_scheduledTxSlot);
    }
  if (m_txCountdownCounter >= -1)
    {
      event.reservationCounter = CsrTraceSignedInteger (
        m_txCountdownCounter);
    }
  WriteDifferentialTrace (event);

  std::cout << "[MAC " << m_nodeId
            << "] OPNET 300-ms transmit holdoff complete"
            << std::endl;
}

void
CsrMacCore::ActivateTxPreparation (bool redrawZero)
{
  if (m_txPreparationActive || !HasPendingFrame ())
    {
      return;
    }

  m_txPreparationActive = true;

  bool selectedNewSlot = m_txCountdownCounter < 0 ||
                         (redrawZero && m_txCountdownCounter == 0);
  if (selectedNewSlot)
    {
      CsrNodeId dest = !m_ackQueue.empty ()
        ? m_ackQueue.front ().dest
        : m_queue.front ().dest;
      m_scheduledTxSlot = PickTxSlot (dest);
      m_txCountdownCounter = m_scheduledTxSlot;

      std::cout << "[MAC " << m_nodeId
                << "] starts Pre-Tx with new slot="
                << m_scheduledTxSlot
                << " holdoff_over=" << (m_txHoldoffOver ? 1 : 0)
                << std::endl;
    }
  else
    {
      std::cout << "[MAC " << m_nodeId
                << "] resumes Pre-Tx with live slot="
                << m_scheduledTxSlot
                << " counter=" << m_txCountdownCounter
                << std::endl;
    }

  if (m_forcedReservationSlot > 0)
    {
      m_scheduledTxSlot = m_forcedReservationSlot;
      m_txCountdownCounter = m_forcedReservationSlot;
      selectedNewSlot = true;

      // Nodes in the recovered scenario have independent 13-ms timer phases.
      // Put controlled differential runs on a common future epoch so neither
      // sender hears the other's preamble before its final countdown event.
      Time epoch = Seconds (0.1);
      double holdoffGuard = m_txHoldoffOver ? 0.0 : TS_HOLDOFF_SECONDS;
      Time earliest = Simulator::Now () +
        Seconds (holdoffGuard) + m_slotTickPeriod;
      int64_t epochSteps = epoch.GetTimeStep ();
      int64_t earliestSteps = earliest.GetTimeStep ();
      int64_t alignedSteps =
        ((earliestSteps + epochSteps - 1) / epochSteps) * epochSteps;
      if (m_slotTickEvent.IsPending ())
        {
          Simulator::Cancel (m_slotTickEvent);
        }
      m_slotTickEnabled = true;
      m_slotTickEvent = Simulator::Schedule (
        TimeStep (alignedSteps - Simulator::Now ().GetTimeStep ()),
        &CsrMacCore::SlotTick,
        this);
    }

  CsrDifferentialTraceEvent event;
  event.event = "reservation_prepare";
  event.node = CsrTraceInteger (m_nodeId);
  event.reservationSlot = CsrTraceSignedInteger (m_scheduledTxSlot);
  event.reservationCounter = CsrTraceSignedInteger (
    m_txCountdownCounter);
  event.reason = m_forcedReservationSlot > 0
    ? (m_txHoldoffOver ? "controlled_ready" : "controlled_wait")
    : (selectedNewSlot ? "new" : "reuse");
  WriteDifferentialTrace (event);
}

void
CsrMacCore::NotifyPhyTxStart (Time duration)
{
  m_txInProgress = true;
  m_state = State::TX;

  CsrDifferentialTraceEvent event;
  event.event = "mac_state";
  event.node = CsrTraceInteger (m_nodeId);
  event.detail = StateName (State::TX);
  WriteDifferentialTrace (event);

  if (m_finishTxEvent.IsPending ())
    {
      Simulator::Cancel (m_finishTxEvent);
    }
  m_finishTxEvent = Simulator::Schedule (duration,
                                         &CsrMacCore::FinishTx,
                                         this);
}

void
CsrMacCore::FinishTx ()
{
  m_txInProgress = false;
  m_state = State::SEARCH;

  CsrDifferentialTraceEvent event;
  event.event = "mac_state";
  event.node = CsrTraceInteger (m_nodeId);
  event.detail = StateName (State::SEARCH);
  WriteDifferentialTrace (event);

  if (m_dev != nullptr)
    {
      m_dev->OnMacTxFinished ();
    }

  if (HasPendingFrame ())
    {
      // post_tx() enables PREP_TX immediately for frames that arrived or
      // remained during Tx, reusing the slot advertised by this frame.
      ActivateTxPreparation ();
    }
}

int
CsrMacCore::ChooseRateForDest (CsrNodeId dest)
{
  auto it = m_neighbors.find (dest);
  double pl = 90.0;
  if (it != m_neighbors.end ())
    {
      pl = it->second.lastPathlossDb;
    }

  if      (pl > 120.0) return 8;
  else if (pl > 100.0) return 16;
  else if (pl > 80.0)  return 32;
  else if (pl > 60.0)  return 64;
  else                 return 128;
}

PreambleType
CsrMacCore::ChoosePreambleForDest (CsrNodeId dest)
{
  auto it = m_neighbors.find (dest);

  if (it == m_neighbors.end ())
    {
      return PREAMBLE_LONG;
    }

  double lastHeard = it->second.lastHeardSec;
  double now       = Simulator::Now ().GetSeconds ();

  // OPNET chooses the preamble from neighbor freshness even when duty
  // cycling is enabled.  It uses the current local active_nodes value here,
  // not max_reported_active_nodes (the latter only expands slot selection).
  // This is the same expression as br_mac's POST_TX_WAIT_TIME macro.
  double threshold =
    15.0 + 1.5 * static_cast<double> (m_activeNodesForPostTx) + 0.5;

  if (lastHeard < 0.0 || (now - lastHeard) > threshold)
    {
      return PREAMBLE_LONG;
    }
  else
    {
      return PREAMBLE_SHORT;
    }
}

int
CsrMacCore::SelectQueuedFrameRate (Ptr<Packet> frame,
                                   CsrNodeId dest,
                                   bool ackable) const
{
  CsrHeader header;
  if (frame != nullptr && frame->PeekHeader (header))
    {
      if (header.HasLinkControl ())
        {
          switch (header.GetSpeedKey ())
            {
            case 8:
            case 16:
            case 32:
            case 64:
            case 128:
            case 500:
            case 1000:
              return header.GetSpeedKey ();
            default:
              return 8;
            }
        }

      if (!ackable)
        {
          return 8;
        }

      int selectedRate = m_maxRateKbps;
      for (CsrNodeId target : header.EnumerateDestinations ())
        {
          selectedRate = std::min (
            selectedRate,
            SelectRateByPerTarget (
              target,
              CsrGetOpnetWireSize (frame) * 8,
              0.50));
        }
      return selectedRate;
    }

  if (!ackable)
    {
      return 8;
    }

  return SelectRateByPerTarget (dest,
                                CsrGetOpnetWireSize (frame) * 8,
                                0.50);
}

double
CsrMacCore::SelectQueuedFrameTxPower (Ptr<Packet> frame) const
{
  CsrHeader header;
  if (frame != nullptr && frame->PeekHeader (header) &&
      header.HasLinkControl ())
    {
      return header.GetTxPowerDbm ();
    }

  return m_dev == nullptr
    ? 0.0
    : m_dev->GetPhy ().profile.txPowerDbm;
}

uint32_t
CsrMacCore::GetConcatByteLimit (int rateKbps) const
{
  switch (rateKbps)
    {
    case 8:   return 256;
    case 16:  return 512;
    case 32:  return 1024;
    case 64:  return 2048;
    case 128: return 4096;
    default:  return 0;
    }
}

bool
CsrMacCore::FitsConcatFrame (Ptr<Packet> frame,
                             int frameRateKbps,
                             uint32_t byteCount,
                             int currentRateKbps) const
{
  int aggregateRate = std::min (frameRateKbps, currentRateKbps);
  uint32_t limit = GetConcatByteLimit (aggregateRate);
  if (limit == 0)
    {
      // The source material does not provide high-rate concatenation limits.
      // Transmit the queue head alone instead of inventing a capacity or
      // repeatedly rescheduling an otherwise valid high-rate frame.
      return byteCount == 0;
    }
  return byteCount + CsrGetOpnetWireSize (frame) < limit;
}

PreambleType
CsrMacCore::SelectAggregatePreamble (
  const std::vector<SelectedTxFrame> &selected,
  bool concatenating)
{
  // Preserve a legacy limitation: outside the concatenation branch OPNET
  // reads only the packet's scalar/primary destination, even when the packet
  // also contains a destination structure.
  if (!concatenating)
    {
      return ChoosePreambleForDest (selected.front ().dest);
    }

  // OPNET keeps the earliest last_rcvd_time while it packs ACKs and then DATA.
  // Destination structures are walked member by member, so any stale or
  // unknown receiver represented anywhere in the aggregate requires LONG.
  for (const auto &entry : selected)
    {
      CsrHeader header;
      if (entry.frame != nullptr && entry.frame->PeekHeader (header))
        {
          for (CsrNodeId destination : header.EnumerateDestinations ())
            {
              if (ChoosePreambleForDest (destination) == PREAMBLE_LONG)
                {
                  return PREAMBLE_LONG;
                }
            }
        }
      else if (ChoosePreambleForDest (entry.dest) == PREAMBLE_LONG)
        {
          return PREAMBLE_LONG;
        }
    }

  return PREAMBLE_SHORT;
}

void
CsrMacCore::DoTx ()
{
  if (!HasPendingFrame () || m_dev == nullptr)
    {
      m_txPreparationActive = false;
      return;
    }

  // A delayed concatenation retry must re-enter through the same MAC gates
  // as an ordinary slot opportunity.  If state changed while the retry was
  // pending, retain PREP_TX/counter=-1 and let a later eligible SlotTick try
  // again instead of transmitting in Idle, Track, Tx, or under SYNC/holdoff.
  if (m_txInProgress ||
      m_state != State::SEARCH ||
      m_syncPresent ||
      !m_txHoldoffOver ||
      !m_txPreparationActive ||
      m_txCountdownCounter != -1)
    {
      return;
    }

  std::vector<SelectedTxFrame> selected;
  uint32_t byteCount = 0;
  int aggregateRateKbps = 1000;
  bool packingStopped = false;
  bool concatenate = m_ackQueue.size () + m_queue.size () > 1;

  if (concatenate)
    {
      // OPNET scans the ACK queue first and stops at the first entry that
      // cannot fit at the slowest rate selected so far.
      for (uint32_t i = 0;
           i < m_ackQueue.size () && !packingStopped;
           ++i)
        {
          const AckQueueEntry &entry = m_ackQueue[i];
          int rate = SelectQueuedFrameRate (entry.frame,
                                            entry.dest,
                                            false);
          if (!FitsConcatFrame (entry.frame,
                                rate,
                                byteCount,
                                aggregateRateKbps))
            {
              packingStopped = true;
              break;
            }

          aggregateRateKbps = std::min (aggregateRateKbps, rate);
          byteCount += CsrGetOpnetWireSize (entry.frame);
          selected.push_back ({entry.frame->Copy (),
                               entry.dest,
                               false,
                               true,
                               i,
                               rate,
                               SelectQueuedFrameTxPower (entry.frame)});
        }

      // Data is removed strictly from the priority queue head.  A non-fitting
      // head blocks every later entry, exactly as in the legacy segmentation
      // loop.  The 16-segment stop is checked only after adding data.
      for (uint32_t i = 0;
           i < m_queue.size () && !packingStopped;
           ++i)
        {
          const TxQueueEntry &entry = m_queue[i];
          int rate = SelectQueuedFrameRate (entry.frame,
                                            entry.dest,
                                            entry.ackable);
          if (!FitsConcatFrame (entry.frame,
                                rate,
                                byteCount,
                                aggregateRateKbps))
            {
              packingStopped = true;
              break;
            }

          aggregateRateKbps = std::min (aggregateRateKbps, rate);
          byteCount += CsrGetOpnetWireSize (entry.frame);
          selected.push_back ({entry.frame->Copy (),
                               entry.dest,
                               entry.ackable,
                               false,
                               i,
                               rate,
                               SelectQueuedFrameTxPower (entry.frame)});

          if (selected.size () >= MAX_CONCAT_SEGMENTS)
            {
              packingStopped = true;
            }
        }
    }
  else if (!m_ackQueue.empty ())
    {
      const AckQueueEntry &entry = m_ackQueue.front ();
      aggregateRateKbps = SelectQueuedFrameRate (entry.frame,
                                                  entry.dest,
                                                  false);
      byteCount = CsrGetOpnetWireSize (entry.frame);
      selected.push_back ({entry.frame->Copy (),
                           entry.dest,
                           false,
                           true,
                           0,
                           aggregateRateKbps,
                           SelectQueuedFrameTxPower (entry.frame)});
    }
  else
    {
      const TxQueueEntry &entry = m_queue.front ();
      aggregateRateKbps = SelectQueuedFrameRate (entry.frame,
                                                  entry.dest,
                                                  entry.ackable);
      byteCount = CsrGetOpnetWireSize (entry.frame);
      selected.push_back ({entry.frame->Copy (),
                           entry.dest,
                           entry.ackable,
                           false,
                           0,
                           aggregateRateKbps,
                           SelectQueuedFrameTxPower (entry.frame)});
    }

  if (selected.empty ())
    {
      std::cout << "[MAC " << m_nodeId
                << "] OPNET concatenation could not fit queue head"
                << std::endl;
      m_txEvent = Simulator::Schedule (Seconds (1.0),
                                       &CsrMacCore::MaybeScheduleNextTx,
                                       this);
      return;
    }

  CsrNodeId dest = selected.front ().dest;

  int powerSelectionRateKbps = selected.front ().rateKbps;
  double aggregateTxPowerDbm = selected.front ().txPowerDbm;

  for (uint32_t i = 1; i < selected.size (); ++i)
    {
      const SelectedTxFrame &entry = selected[i];
      if (entry.rateKbps < powerSelectionRateKbps)
        {
          powerSelectionRateKbps = entry.rateKbps;
          aggregateTxPowerDbm = entry.txPowerDbm;
        }
      else if (entry.rateKbps == powerSelectionRateKbps)
        {
          aggregateTxPowerDbm = std::max (aggregateTxPowerDbm,
                                          entry.txPowerDbm);
        }
    }

  std::vector<Ptr<Packet>> wireFrames;
  wireFrames.reserve (selected.size ());
  bool anyAckable = false;
  PreambleType pt = SelectAggregatePreamble (selected, concatenate);

  for (auto &entry : selected)
    {
      CsrHeader header;
      entry.frame->RemoveHeader (header);
      if (header.HasLinkControl ())
        {
          header.SetLinkControl (
            static_cast<CsrRateKey> (aggregateRateKbps),
            aggregateTxPowerDbm,
            header.GetRxPowerDbm ());
        }
      else
        {
          header.SetSpeedKey (aggregateRateKbps);
        }
      if (header.GetType () == CSR_PKT_DATA ||
          header.GetType () == CSR_PKT_ACK ||
          header.GetType () == CSR_PKT_DACK)
        {
          header.SetType (header.IsAck ()
                            ? CSR_PKT_ACK
                            : (header.IsDack ()
                                 ? CSR_PKT_DACK
                                 : CSR_PKT_DATA));
        }
      header.SetDestType (
        header.HasDestinationSequences ()
          ? CSR_DEST_MULTICAST
          : (entry.dest == CSR_BROADCAST_ID
               ? CSR_DEST_BROADCAST
               : CSR_DEST_UNICAST));
      entry.frame->AddHeader (header);
      wireFrames.push_back (entry.frame);
      anyAckable = anyAckable || entry.ackable;
    }

  if (pt == PREAMBLE_LONG)
    {
      m_longPreambleTxCount++;
    }
  else
    {
      m_shortPreambleTxCount++;
    }

  // OPNET includes a newly selected future reservation in every OTA frame.
  // The sender loads that same value into its persistent txslot_counter even
  // if this transmission drains both queues.
  m_lastTxOpportunitySlot = m_scheduledTxSlot;
  int nextReservedSlot = PickTxSlot (dest);
  m_lastAdvertisedReservationSlot = nextReservedSlot;
  m_scheduledTxSlot = nextReservedSlot;
  m_txCountdownCounter = nextReservedSlot;
  m_txPreparationActive = false;

  CsrDifferentialTraceEvent reservationEvent;
  reservationEvent.event = "reservation_advertise";
  reservationEvent.node = CsrTraceInteger (m_nodeId);
  reservationEvent.reservationSlot = CsrTraceSignedInteger (
    nextReservedSlot);
  reservationEvent.reservationCounter = CsrTraceSignedInteger (
    m_txCountdownCounter);
  WriteDifferentialTrace (reservationEvent);

  // Send via device
  std::cout << "[MAC " << m_nodeId
            << "] TX opportunity reached"
            << " currentSlot=" << m_lastTxOpportunitySlot
            << " reserveNextSlot=" << nextReservedSlot
            << std::endl;

  m_dev->SendFramesToPeers (wireFrames,
                            aggregateRateKbps,
                            aggregateTxPowerDbm,
                            pt,
                            nextReservedSlot,
                            anyAckable);

  m_lastTxRateKbps = aggregateRateKbps;
  m_lastTxPowerDbm = aggregateTxPowerDbm;

  m_transmittedFrameCount++;

  // OPNET br_mac sends br_Mac_Hop_Inst only after the frame actually enters
  // the transmitter.  HOP uses that sent instant, rather than its enqueue
  // time, as the origin for ACK timeout and retransmission processing.
  if (!m_txSentCallback.IsNull ())
    {
      for (const auto &entry : selected)
        {
          if (!entry.ackable)
            {
              continue;
            }
          CsrHeader header;
          entry.frame->PeekHeader (header);
          m_txSentCallback (entry.dest,
                            header.GetSeq (),
                            Simulator::Now ());
        }
    }

  uint32_t selectedAckCount = 0;
  uint32_t selectedDataCount = 0;
  for (const auto &entry : selected)
    {
      if (entry.fromAckQueue)
        {
          AckQueueEntry &ackEntry = m_ackQueue[entry.queueIndex];
          ackEntry.txCount++;
          selectedAckCount++;
          std::cout << "[MAC " << m_nodeId
                    << "] ACK transmitted in aggregate"
                    << " dest=" << ackEntry.dest
                    << " seq=" << ackEntry.seq
                    << " txCount=" << ackEntry.txCount
                    << "/" << (MAX_ACK_RESEND + 1)
                    << std::endl;
        }
      else
        {
          selectedDataCount++;
        }
    }

  m_ackQueue.erase (
    std::remove_if (
      m_ackQueue.begin (),
      m_ackQueue.end (),
      [this] (const AckQueueEntry &entry) {
        if (entry.txCount < MAX_ACK_RESEND + 1)
          {
            return false;
          }
        std::cout << "[MAC " << m_nodeId
                  << "] Remove ACK after OPNET resend limit"
                  << " dest=" << entry.dest
                  << " seq=" << entry.seq
                  << std::endl;
        return true;
      }),
    m_ackQueue.end ());

  if (selectedDataCount > 0)
    {
      m_queue.erase (m_queue.begin (),
                     m_queue.begin () + selectedDataCount);
    }

  std::cout << "[MAC " << m_nodeId
            << "] OPNET aggregate transmitted"
            << " segments=" << selected.size ()
            << " ackSegments=" << selectedAckCount
            << " dataSegments=" << selectedDataCount
            << " bytes=" << byteCount
            << " rate=" << aggregateRateKbps
            << std::endl;

  // FinishTx() re-enables PREP_TX if another frame remains.  Otherwise the
  // advertised counter continues to advance in Search with PREP_TX false,
  // exactly as br_mac.tslot_tasks() requires for reservation reuse.
}



// ------------------------------------------------------------
// CsrHopLayer: simplified hop layer
// ------------------------------------------------------------
