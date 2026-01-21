#pragma once
#include "csr-common.h"
#include "csr-mac-core.h"
#include "csr-phy-model.h"

class CsrNetDevice : public Object
{
public:
  CsrNetDevice () = default;
  CsrNetDevice (uint16_t id)
    : m_id (id)
  {
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

  uint16_t GetId () const { return m_id; }


  void SendToPeer (Ptr<Packet> frame,
                   uint16_t dest,
                   int rateKbps,
                   PreambleType preamble,
                   int slot,
                   bool ackable);

private:
  uint16_t                       m_id {0};
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
      uint16_t n = kv.first;
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
CsrMacCore::SelectRateByPerTarget (uint16_t destId, uint32_t nBits, double targetPer) const
{
  static const std::vector<int> kRates = { 8, 16, 32, 64, 128 }; // keep consistent with your supported set

  // Access PHY through the owning device (now safe because this definition is after CsrNetDevice is complete)
  const CsrPhyModel& phy = m_dev->GetPhy ();
  uint16_t txId = m_dev->GetId ();

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

void
CsrNetDevice::SendToPeer (Ptr<Packet> frame,
                          uint16_t dest,
                          int rateKbps,
                          PreambleType preamble,
                          int slot,
                          bool ackable)
{

  double sizeBits  = frame->GetSize () * 8.0;
  double rbps      = CsrRateKeyToBps (rateKbps);   // exact CSNW rate from Table 1
  //double duration  = sizeBits / rbps;
  // Simple preamble timing model (we will refine later)
  double preambleSec = (preamble == PREAMBLE_LONG) ? 0.9 : 0.1;
  double duration    = preambleSec + (sizeBits / rbps);

  double txTime    = Simulator::Now ().GetSeconds ();
  double propDelay = 0.001;

  // For log: peek header
  CsrHeader hdr;
  frame->PeekHeader (hdr);

  std::cout   << "... type=" << unsigned(hdr.GetType())
              << " speedKey=" << unsigned(hdr.GetSpeedKey())
              << "[t=" << txTime << "] TX from node " << m_id
              << " to dest " << dest
              << " seq " << hdr.GetSeq ()
              << " size " << frame->GetSize () << " bytes"
              << " at slot " << slot
              << " rate " << rateKbps << " kbps"
              << " (" << rbps << " bps)"
              << " preamble " << (preamble == PREAMBLE_LONG ? "LONG" : "SHORT")
              << " duration " << duration << " s"
              << std::endl;

  Ptr<CsrNetDevice> self = this;
  Ptr<Packet> frameCopy = frame->Copy ();

  if (ackable)
  {
    // Stay awake through TX + propagation + some ACK turnaround margin.
    // Keep this conservative for now; we can tune later.
    double ackMarginSec = 0.35;   // small “transaction” window
    this->ForceAwakeFor(duration + propDelay + ackMarginSec);
  }

  // Peek header once to know src/dst/seq
  CsrHeader hdrOnTx;
  frameCopy->PeekHeader (hdrOnTx);
  uint16_t txId   = hdrOnTx.GetSrc ();
  uint16_t destId = hdrOnTx.GetDst ();
  uint16_t seq    = hdrOnTx.GetSeq ();

  Simulator::Schedule (Seconds (propDelay + duration),
                     [self, frameCopy, rateKbps, preamble, slot, ackable,
                      txId, destId, seq, txTime, propDelay, duration]() {
    if (self->m_peers.empty ())
      {
        return;
      }

    double tRx = Simulator::Now ().GetSeconds ();
    uint32_t packetBits = frameCopy->GetSize () * 8;

    // For each peer, deliver if it is the intended dest
    for (auto const &peer : self->m_peers)
      {
        if (peer == nullptr) { continue; }

        // Unicast: only the peer whose ID matches destId should receive
        /*if (peer->GetId () != destId)
          {
            continue;
          }*/
        
        bool isBcast = (destId == CSR_BROADCAST_ID);
        if (!isBcast && peer->GetId() != destId)
          {
            continue;
          }
        if (isBcast && peer->GetId() == txId)
          {
            continue; // don't deliver to self
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

        std::cout << "[t=" << tRx << "] RX at node " << peer->GetId ()
                  << " from node " << txId
                  << " seq " << seq
                  << " rate " << rateKbps << " kbps"
                  << " preamble " << (preamble == PREAMBLE_LONG ? "LONG" : "SHORT")
                  << " pathloss " << d.pathlossDb << " dB"
                  << " snr " << d.snrDb << " dB"
                  << " (slot " << slot
                  << ", len " << frameCopy->GetSize () << "B"
                  << ", ackable=" << (ackable ? 1 : 0) << ")"
                  << std::endl;

        if (ackable)
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

        // Deliver up into peer MAC (each peer needs its own copy)
        Ptr<Packet> peerCopy = frameCopy->Copy ();
        peer->m_mac.DeliverRxFrameToUp (peerCopy, d.pathlossDb, d.snrDb);
      }
  });
}

// ------------------------------------------------------------
// CsrMacCore implementation
// ------------------------------------------------------------

void
CsrMacCore::EnqueueTxFrame (Ptr<Packet> frame,
                            uint16_t dest,
                            uint8_t dscp,
                            bool ackable)
{
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

  if (m_queue.empty ())
    {
      return;
    }

  // Schedule immediate TX
  m_txEvent = Simulator::ScheduleNow (&CsrMacCore::DoTx, this);
}

int
CsrMacCore::ChooseRateForDest (uint16_t dest)
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
CsrMacCore::ChoosePreambleForDest (uint16_t dest)
{
  auto it = m_neighbors.find (dest);
  
  // If receivers are duty-cycling, SHORT is not reliable unless we have an
  // explicit "receiver is awake" handshake context. Be conservative.
  if (m_dev && m_dev->IsDutyCyclingEnabled ())
    {
      return PREAMBLE_LONG;
    }

  if (it == m_neighbors.end ())
    {
      return PREAMBLE_LONG;
    }

  double lastHeard = it->second.lastHeardSec;
  double now       = Simulator::Now ().GetSeconds ();

  // Rough analog of STAY_AWAKE_BASE_TIME + 1.5*active_nodes + GUARD
  double threshold = 15.0 + 1.5 * 8 + 0.5; // 27.5 s

  if (lastHeard < 0.0 || (now - lastHeard) > threshold)
    {
      return PREAMBLE_LONG;
    }
  else
    {
      return PREAMBLE_SHORT;
    }
}

void
CsrMacCore::DoTx ()
{
  if (m_queue.empty () || m_dev == nullptr)
    {
      return;
    }

  // Reference to front; do NOT pop/erase until we actually send
  TxQueueEntry &e = m_queue.front ();

  // Peek header for dest / ackable / type (informational)
  CsrHeader hdr;
  e.frame->PeekHeader (hdr);

  uint8_t  typePeek = hdr.GetType ();   // may be stale if type is inferred later
  bool     ackable  = hdr.IsAckable ();
  uint16_t dest     = hdr.GetDst ();

  // Bits in this frame
  uint32_t bits = e.frame->GetSize () * 8;

  // --- Guard: do not transmit if link is hopeless even at most robust rate ---
  const CsrPhyModel& phy = m_dev->GetPhy ();
  uint16_t txId = m_dev->GetId ();

  double snrDb = phy.PredictSnrDb (txId, dest, phy.profile.txPowerDbm);
  double perAtMinRate = phy.EstimatePer (8 /*kbps*/, snrDb, bits);

  // Processing gain (matches what you're printing elsewhere)
  double rb = CsrRateKeyToBps (8);
  double procGainDb = 10.0 * std::log10 (m_dev->GetPhy ().profile.rxBwHz / (2.0 * rb));
  double effSnrDb = snrDb + procGainDb;

  if (perAtMinRate >= 0.99)
    {
      std::cout << "[MAC] Deferring TX tx=" << txId
                << " dest=" << dest
                << " snr=" << snrDb
                << " effSnr=" << effSnrDb
                << " per@8=" << perAtMinRate
                << " (hopeless link)"
                << std::endl;

      // Coarse backoff; packet stays at the front of the queue
      m_txEvent = Simulator::Schedule (Seconds (1.0), &CsrMacCore::DoTx, this);
      return;
    }

  // Rate selection: keep control more robust
  double targetPer = 0.50;
  bool isAckOrControl = (!ackable); // in your model, ACK frames are not ackable

  int rateKbps = 0;
  if (isAckOrControl)
    {
      rateKbps = 8;  // robust control rate (tune later)
    }
  else
    {
      rateKbps = SelectRateByPerTarget (dest, bits, targetPer);
    }

  // Remove header so we can stamp final fields (type/speedKey/destType)
  CsrHeader th;
  e.frame->RemoveHeader (th);

  // Always stamp the chosen TX rate key
  th.SetSpeedKey (rateKbps);

  // Preserve control packet types already set (HELLO/DISCOVER/etc.)
  // Only infer DATA/ACK/DACK for "normal" traffic.
  if (th.GetType () == CSR_PKT_DATA || th.GetType () == CSR_PKT_ACK || th.GetType () == CSR_PKT_DACK)
    {
      th.SetType (th.IsAck () ? CSR_PKT_ACK : (th.IsDack () ? CSR_PKT_DACK : CSR_PKT_DATA));
    }

  // DestType should match whether we're unicasting or broadcasting
  th.SetDestType ((dest == CSR_BROADCAST_ID) ? CSR_DEST_BROADCAST : CSR_DEST_UNICAST);

  // --- PREAMBLE SELECTION (UPDATED) ---
  // In duty-cycled operation, SHORT preamble is only safe for ACK/DACK
  // because the peer is forced-awake during the exchange window.
  PreambleType pt = PREAMBLE_LONG;

  if (th.IsAck () || th.IsDack () || typePeek == CSR_PKT_ACK /* fallback */)
    {
      pt = PREAMBLE_SHORT;
    }
  else
    {
      pt = ChoosePreambleForDest (dest); // should return LONG when duty cycling enabled
    }

  // Slot selection (independent)
  int slot = PickTxSlot (dest);

  // Put header back
  e.frame->AddHeader (th);

  // Send via device
  m_dev->SendToPeer (e.frame, dest, rateKbps, pt, slot, ackable);

  // Now we can remove the queue entry
  m_queue.erase (m_queue.begin ());

  // Schedule next TX if any
  if (!m_queue.empty ())
    {
      m_txEvent = Simulator::ScheduleNow (&CsrMacCore::DoTx, this);
    }
}



// ------------------------------------------------------------
// CsrHopLayer: simplified hop layer
// ------------------------------------------------------------

