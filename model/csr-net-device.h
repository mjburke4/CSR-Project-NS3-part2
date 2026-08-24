#pragma once
#include "csr-common.h"
#include "csr-mac-core.h"
#include "csr-opnet-envelope.h"
#include "csr-phy-model.h"

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
    if (enable && m_rng)
      {
        // De-sync nodes so they don't all wake at same instant
        m_wakePhaseSec = m_rng->GetValue (0.0, m_wakeCycleSec);
      }
  }

  bool IsDutyCyclingEnabled () const { return m_dutyCycleEnabled; }

  double m_forceAwakeUntilSec { 0.0 };

  void ForceAwakeFor (double seconds)
  {
    double now = Simulator::Now().GetSeconds();
    m_forceAwakeUntilSec = std::max(m_forceAwakeUntilSec, now + seconds);
  }

  CsrMacCore& GetMac ()
  {
    return m_mac;
  }

  CsrPhyModel& GetPhy() { return m_phy; }

  CsrNodeId GetId () const { return m_id; }

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
  CsrNodeId                       m_id {0};
  CsrMacCore                     m_mac;
  std::vector< Ptr<CsrNetDevice> > m_peers;   // multiple peers on shared channel
  Ptr<UniformRandomVariable>     m_rng;
  CsrPhyModel                    m_phy;

  // --- Duty cycling (OPNET-inspired) ---
  bool   m_dutyCycleEnabled { false };

  // OPNET constants:
  //   WAKE_CYCLE ~0.988 s
  //   DSP_RX_BOOTTIME 0.0011 s
  //   NO_SIG_SRH_TIME 0.0078 s
  double m_wakeCycleSec     { 0.988 };
  double m_awakeWindowSec   { 0.0011 + 0.0078 }; // ~0.0089 s
  double m_wakePhaseSec     { 0.0 };             // randomized per node
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

  bool CanReceiveDuring (double tStart, double tEnd) const
  {
    if (!m_dutyCycleEnabled)
      {
        return true;
      }
      
    // If we are in an "active exchange" window, treat as awake
    if (m_forceAwakeUntilSec >= tStart)
      {
        return true;
      }

    // Periodic awake windows: [phase + k*cycle, phase + k*cycle + window]
    const double phase  = m_wakePhaseSec;
    const double cycle  = m_wakeCycleSec;
    const double win    = m_awakeWindowSec;

    // Find a nearby k to test; only a few candidates are needed
    double k0_real = std::floor ((tStart - phase) / cycle);
    int64_t k0 = static_cast<int64_t> (k0_real);

    for (int64_t dk = -2; dk <= 2; ++dk)
      {
        double ws = phase + (k0 + dk) * cycle;
        double we = ws + win;

        // overlap test: [ws,we] with [tStart,tEnd]
        if (we >= tStart && ws <= tEnd)
          {
            return true;
          }
      }

    return false;
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
  static const std::vector<int> kRates = { 8, 16, 32, 64, 128 }; // keep consistent with your supported set

  // Access PHY through the owning device (now safe because this definition is after CsrNetDevice is complete)
  const CsrPhyModel& phy = m_dev->GetPhy ();
  CsrNodeId txId = m_dev->GetId ();

  double snrDb = phy.PredictSnrDb (txId, destId, phy.profile.txPowerDbm);

  int chosen = kRates.front (); // most robust default
  for (int r : kRates)
    {
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

  uint32_t payloadBytes = 0;
  std::vector<Ptr<Packet>> frameCopies;
  frameCopies.reserve (frames.size ());
  for (const auto &segment : frames)
    {
      payloadBytes += CsrGetOpnetWireSize (segment);
      frameCopies.push_back (segment->Copy ());
    }

  double sizeBits  = payloadBytes * 8.0;
  double rbps      = CsrRateKeyToBps (rateKbps);   // exact CSNW rate from Table 1
  //double duration  = sizeBits / rbps;
  // Simple preamble timing model (we will refine later)
  double preambleSec = (preamble == PREAMBLE_LONG) ? 0.9 : 0.1;
  double duration    = preambleSec + (sizeBits / rbps);

  double txTime    = Simulator::Now ().GetSeconds ();
  double propDelay = 0.001;

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

  Ptr<CsrNetDevice> self = this;

  /*if (ackable)
  {
    // Stay awake through TX + propagation + some ACK turnaround margin.
    // Keep this conservative for now; we can tune later.
    double ackMarginSec = 0.35;   // small “transaction” window
    this->ForceAwakeFor(duration + propDelay + ackMarginSec);
  }*/
  // OPNET br_mac.post_tx(): after the last transmission, MAC remains in
  // receive/search for POST_TX_WAIT_TIME. This is not limited to ACKable data;
  // HELLO/control transmissions also keep the node awake long enough to hear
  // follow-on traffic.
  double postTxWaitSec = GetPostTxWaitSeconds ();
  this->ForceAwakeFor (duration + propDelay + postTxWaitSec);

std::cout << "[MAC " << m_id
          << "] Post-TX force-awake for "
          << (duration + propDelay + postTxWaitSec)
          << " s"
          << " (POST_TX_WAIT=" << postTxWaitSec << " s)"
          << std::endl;

  // Peek header once to know src/dst/seq
  CsrHeader hdrOnTx;
  frameCopies.front ()->PeekHeader (hdrOnTx);
  CsrNodeId txId   = hdrOnTx.GetSrc ();
  uint16_t seq    = hdrOnTx.GetSeq ();

  Simulator::Schedule (Seconds (propDelay + duration),
                     [self, frameCopies, rateKbps, txPowerDbm,
                      preamble, slot, ackable,
                      txId, seq, txTime, propDelay, duration, payloadBytes]() {
    if (self->m_peers.empty ())
      {
        return;
      }

    double tRx = Simulator::Now ().GetSeconds ();
    uint32_t packetBits = payloadBytes * 8;

    // A CSR transmission occupies the shared wireless medium.  Every awake
    // peer in range therefore attempts to decode the aggregate, even when no
    // segment names that peer as a destination.  OPNET forwards every decoded
    // MAC payload to HOP and lets HOP discard traffic addressed elsewhere.
    for (auto const &peer : self->m_peers)
      {
        if (peer == nullptr) { continue; }
        if (peer->GetId () == txId) { continue; }

        bool addressedFrame = false;
        bool ackableForPeer = false;
        for (const auto &segment : frameCopies)
          {
            CsrHeader segmentHeader;
            segment->PeekHeader (segmentHeader);
            bool matches =
              segmentHeader.IsForDestination (peer->GetId ());
            if (matches)
              {
                addressedFrame = true;
                ackableForPeer = ackableForPeer ||
                                 segmentHeader.IsAckable ();
              }
          }

        double rxStart = txTime + propDelay;
        double rxEnd   = rxStart + duration;

        if (!peer->CanReceiveDuring (rxStart, rxEnd))
          {
            std::cout << "[t=" << tRx << "] RX MISS (ASLEEP) at node " << peer->GetId ()
                      << " from node " << txId
                      << " seq " << seq
                      << " interval=[" << rxStart << "," << rxEnd << "]"
                      << std::endl;
            continue;
          }

        // PHY/pipeline decision
        CsrRxDecision d = peer->m_phy.EvaluateRx (txId,
                                                peer->GetId (),
                                                rateKbps,
                                                txPowerDbm,
                                                packetBits,
                                                peer->m_rng);


        if (g_rxCsv.is_open ())
          {
            double dist = peer->m_phy.GetDistanceMeters (txId, peer->GetId ());
            g_rxCsv << tRx << ","
                    << txId << ","
                    << peer->GetId () << ","
                    << seq << ","
                    << rateKbps << ","
                    << packetBits << ","
                    << dist << ","
                    << d.pathlossDb << ","
                    << d.snrDb << ","
                    << d.per << ","
                    << (d.success ? 1 : 0)
                    << "\n";
          }

        if (!d.success)
          {
            std::cout << "[t=" << tRx << "] RX DROP at node " << peer->GetId ()
                      << " from node " << txId
                      << " seq " << seq
                      << " (pathloss " << d.pathlossDb << " dB"
                      << ", snr " << d.snrDb << " dB"
                      << ", per=" << d.per << ")"
                      << std::endl;
            continue;
          }

        std::cout << "[t=" << tRx << "] "
                  << (addressedFrame ? "RX" : "RX OVERHEARD")
                  << " at node " << peer->GetId ()
                  << " from node " << txId
                  << " seq " << seq
                  << " rate " << rateKbps << " kbps"
                  << " txPower " << txPowerDbm << " dBm"
                  << " preamble " << (preamble == PREAMBLE_LONG ? "LONG" : "SHORT")
                  << " pathloss " << d.pathlossDb << " dB"
                  << " snr " << d.snrDb << " dB"
                  << " (slot " << slot
                  << ", len " << payloadBytes << "B"
                  << ", ackable=" << (ackable ? 1 : 0) << ")"
                  << std::endl;

        if (ackableForPeer)
          {
            // We just decoded a unicast that needs an ACK.
            // Stay awake long enough to send ACK + any immediate follow-on.
            peer->ForceAwakeFor(0.35);
          }

        // Update neighbor info (both ends) in MAC
        /*peer->m_mac.NoteHeardFrom (txId, tRx);
        peer->m_mac.SetNeighborPathloss (txId, d.pathlossDb);

        self->m_mac.NoteHeardFrom (destId, tRx);
        self->m_mac.SetNeighborPathloss (destId, d.pathlossDb);

        // Remember which slot this neighbor used
        peer->m_mac.NoteNeighborReservedSlot (txId, slot);
        self->m_mac.NoteNeighborReservedSlot (destId, slot);*/

        // Update neighbor info at the RECEIVER based on what it actually observed.
        // (OPNET update_neighbor() happens on RX.)
        peer->m_mac.NoteHeardFrom (txId, tRx);
        peer->m_mac.SetNeighborPathloss (txId, d.pathlossDb);
        peer->m_mac.NoteNeighborReservedSlot (txId, slot);

        // Forward every decoded segment to HOP.  HOP first updates its
        // source-neighbor observation and then applies destination filtering,
        // matching OPNET's MAC_TO_HOP processing order.
        for (const auto &segment : frameCopies)
          {
            peer->m_mac.DeliverRxFrameToUp (
              segment->Copy (), d.pathlossDb, d.snrDb);
          }
      }
  });

  return Seconds (duration);
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

  if (!m_slotTickEnabled)
    {
      StartSlotTick (m_slotTickPeriod);
    }

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

  if (m_txEvent.IsPending ())
    {
      return;
    }

  if (!HasPendingFrame ())
    {
      return;
    }

  // OPNET prep_tx(): a newly active transmitter waits through the 300-ms
  // radio holdoff, then decrements its selected counter once per 13-ms slot.
  // Transmission occurs only after the counter moves from zero to -1.
  CsrNodeId dest = !m_ackQueue.empty ()
    ? m_ackQueue.front ().dest
    : m_queue.front ().dest;
  int slot = PickTxSlot (dest);
  ScheduleTxOpportunity (slot, Seconds (0.0), true);
}

void
CsrMacCore::ScheduleTxOpportunity (int slot,
                                   Time notBefore,
                                   bool initialHoldoff)
{
  m_scheduledTxSlot = slot;

  Time holdoff = initialHoldoff
    ? Seconds (TS_HOLDOFF_SECONDS)
    : Seconds (0.0);
  Time countdown = m_slotTickPeriod * static_cast<uint64_t> (slot + 1);
  Time delay = notBefore + holdoff + countdown;

  std::cout << "[MAC " << m_nodeId
            << "] scheduled TX opportunity"
            << " slot=" << slot
            << " holdoff=" << holdoff.GetSeconds ()
            << " countdown=" << countdown.GetSeconds ()
            << " tx_in=" << delay.GetSeconds ()
            << "s"
            << std::endl;

  m_txEvent = Simulator::Schedule (delay, &CsrMacCore::DoTx, this);
}

void
CsrMacCore::FinishTx ()
{
  m_txInProgress = false;
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
              return header.GetSpeedKey ();
            default:
              return 8;
            }
        }

      if (!ackable)
        {
          return 8;
        }

      int selectedRate = 128;
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
  return limit > 0 &&
         byteCount + CsrGetOpnetWireSize (frame) < limit;
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
      m_scheduledTxSlot = -1;
      return;
    }

  std::vector<SelectedTxFrame> selected;
  uint32_t byteCount = 0;
  int aggregateRateKbps = 128;
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
                                       &CsrMacCore::DoTx,
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
            static_cast<uint8_t> (aggregateRateKbps),
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
  // If another frame is already queued, this is also its countdown slot.
  int nextReservedSlot = PickTxSlot (dest);

  // Send via device
  std::cout << "[MAC " << m_nodeId
            << "] TX opportunity reached"
            << " currentSlot=" << m_scheduledTxSlot
            << " reserveNextSlot=" << nextReservedSlot
            << std::endl;

  Time txDuration =
    m_dev->SendFramesToPeers (wireFrames,
                              aggregateRateKbps,
                              aggregateTxPowerDbm,
                              pt,
                              nextReservedSlot,
                              anyAckable);

  m_lastTxRateKbps = aggregateRateKbps;
  m_lastTxPowerDbm = aggregateTxPowerDbm;

  m_txInProgress = true;
  Simulator::Schedule (txDuration, &CsrMacCore::FinishTx, this);

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

  // Schedule next TX if any
  if (HasPendingFrame ())
    {
      // Time-slot counters pause while OPNET MAC is in Tx_st.  Resume the
      // advertised reservation countdown only after this frame finishes.
      ScheduleTxOpportunity (nextReservedSlot, txDuration, false);
    }
  else
    {
      m_scheduledTxSlot = -1;
    }
}



// ------------------------------------------------------------
// CsrHopLayer: simplified hop layer
// ------------------------------------------------------------
