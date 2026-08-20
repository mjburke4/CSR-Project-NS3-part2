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

  void SetNodeId (uint16_t id)        { m_nodeId = id; }
  void SetDevice (CsrNetDevice *dev)  { m_dev = dev; }
  int SelectRateByPerTarget (uint16_t destId, uint32_t nBits, double targetPer) const;
  void PrintNeighbors () const;

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

    // Jitter so nodes don’t all HELLO at the exact same instant
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
    Callback<void, uint16_t, uint16_t, Time> cb)
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

  int32_t GetNeighborReservationCounter (uint16_t neighbor) const
  {
    auto it = m_neighbors.find (neighbor);
    return (it == m_neighbors.end ()) ? -1 : it->second.rtCounter;
  }

  // Called by Hop layer to enqueue a full over-the-air frame
  void EnqueueTxFrame (Ptr<Packet> frame,
                       uint16_t dest,
                       uint8_t dscp,
                       bool ackable);

  // Remove retransmissions that a cumulative HOP ACK/DACK has already
  // completed.  This mirrors the legacy br_Mac_Hop_Inst queue cleanup.
  uint32_t CancelAcknowledgedFrames (uint16_t neighbor,
                                     uint16_t baseSeq,
                                     uint64_t ackBitmap,
                                     uint64_t dackBitmap);

  // Called by CsrNetDevice when a frame is successfully received
  void DeliverRxFrameToUp (Ptr<Packet> frame, double pathlossDb, double snrDb)
  {
    if (!m_rxCallback.IsNull ())
      {
        m_rxCallback (frame, pathlossDb, snrDb);
      }
  }

  void NoteHeardFrom (uint16_t neighbor, double tRx)
  {
    m_lastHeardSec[neighbor] = tRx;
    NeighborInfo &info = m_neighbors[neighbor];
    info.lastHeardSec = tRx;
    // pathloss may be updated separately
  }

  void SetNeighborPathloss (uint16_t neighbor, double pathlossDb)
  {
    NeighborInfo &info = m_neighbors[neighbor];
    info.lastPathlossDb = pathlossDb;
    m_lastPathlossDb[neighbor] = pathlossDb;
  }

  void NoteNeighborReservedSlot (uint16_t neighbor, int slot)
  {
    if (!m_slotTickEnabled)
      {
        StartSlotTick (m_slotTickPeriod);
      }

    NeighborInfo &info = m_neighbors[neighbor];

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

  /*void NoteNeighborReservedSlot (uint16_t neighbor, int slot)
  {
    NeighborInfo &info = m_neighbors[neighbor];
    info.reserveSlot = slot;
    info.rtCounter   = 20; //4;   // simple “hold” window, tune later if you want

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
    // not as a hard delete trigger (in the files we’re porting).
    PrintNeighbors ();
    m_neighborAgingEvent = Simulator::Schedule (Seconds (2.0),
                                              &CsrMacCore::AgeNeighbors, this);
  }


  // PrintNeighbors() moved after CsrNetDevice definition (see below)

  uint16_t GetNodeId () const { return m_nodeId; }

private:
  struct NeighborInfo
  {
    double lastHeardSec { -1.0 };
    double lastPathlossDb { NAN };
    double lastSnrDb { NAN };
    int32_t reserveSlot { -1 };   // neighbor's reserved timeslot (if tracked)
    int32_t rtCounter   {  0 };   // how long to honor that reservation
  };

  struct TxQueueEntry
  {
    Ptr<Packet> frame;
    uint16_t    dest;
    uint8_t     dscp;
    bool        ackable;
  };

  struct AckQueueEntry
  {
    Ptr<Packet> frame;
    uint16_t    dest;
    uint16_t    seq;
    bool        hasWindow;
    uint32_t    txCount;
  };

  struct SelectedTxFrame
  {
    Ptr<Packet> frame;
    uint16_t    dest;
    bool        ackable;
    bool        fromAckQueue;
    uint32_t    queueIndex;
    int         rateKbps;
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
  void ScheduleTxOpportunity (int slot, Time notBefore, bool initialHoldoff);
  void FinishTx ();
  void DoTx ();
  int SelectQueuedFrameRate (Ptr<Packet> frame,
                             uint16_t dest,
                             bool ackable) const;
  uint32_t GetConcatByteLimit (int rateKbps) const;
  bool FitsConcatFrame (Ptr<Packet> frame,
                        int frameRateKbps,
                        uint32_t byteCount,
                        int currentRateKbps) const;
  PreambleType SelectAggregatePreamble (
    const std::vector<SelectedTxFrame> &selected,
    bool concatenating);

  int           ChooseRateForDest (uint16_t dest);
  PreambleType  ChoosePreambleForDest (uint16_t dest);

private:
  uint16_t                            m_nodeId;
  CsrNetDevice*                       m_dev;
  std::vector<TxQueueEntry>           m_queue;
  std::vector<AckQueueEntry>          m_ackQueue;
  EventId                             m_txEvent;
  Callback<void, Ptr<Packet>, double, double>  m_rxCallback;
  Callback<void, uint16_t, uint16_t, Time>     m_txSentCallback;
  uint64_t                            m_transmittedFrameCount {0};
  uint64_t                            m_longPreambleTxCount {0};
  uint64_t                            m_shortPreambleTxCount {0};
  uint64_t                            m_dataQueueDropCount {0};
  int                                 m_scheduledTxSlot {-1};
  bool                                m_txInProgress {false};
  std::map<uint16_t, NeighborInfo>    m_neighbors;
  Time                                m_neighborTimeout { Seconds (6.0) };   // default: 3x hello interval (if hello=2s)
  EventId                             m_neighborAgingEvent;
  std::map<uint16_t, double>          m_lastHeardSec;  // neighborId -> last heard time (seconds)
  std::map<uint16_t, double>          m_lastPathlossDb; // neighborId -> last pathloss (optional)
  //void DiscoveryStart ();
  //void DiscoveryStop ();
  //void SendHelloInternal (bool reschedule);

  /*int PickTxSlot (uint16_t dest)
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
  PickTxSlot (uint16_t dest)
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
  m_slotTickPeriod  = period;
  m_slotTickEnabled = true;

  if (m_slotTickEvent.IsPending ())
    {
      Simulator::Cancel (m_slotTickEvent);
    }

  m_slotTickEvent = Simulator::Schedule (m_slotTickPeriod, &CsrMacCore::SlotTick, this);
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

/*void
CsrMacCore::SlotTick ()
{
  if (!m_slotTickEnabled)
    {
      return;
    }

  // Decay neighbor reservations over time (not only on TX)
  for (auto &kv : m_neighbors)
  {
    auto &ni = kv.second;
    if (ni.rtCounter > 0)
    {
      ni.rtCounter--;
      if (ni.rtCounter == 0) ni.reserveSlot = -1;
        std::cout << "[MAC " << m_nodeId << "] SlotTick neighbor " << kv.first
                << " rtCounter=" << ni.rtCounter
                << " reserveSlot=" << ni.reserveSlot
                << std::endl;
    }
  }
  // Re-arm
  m_slotTickEvent = Simulator::Schedule (m_slotTickPeriod, &CsrMacCore::SlotTick, this);
}*/

void
CsrMacCore::SlotTick ()
{
  if (!m_slotTickEnabled)
    {
      return;
    }

  // OPNET tslot_tasks() keeps the timer running in Tx_st but does not advance
  // any reservation counters until the MAC returns to Search_st.
  if (!m_txInProgress)
    {
      for (auto &kv : m_neighbors)
        {
          uint16_t neighbor = kv.first;
          NeighborInfo &ni = kv.second;

          if (ni.rtCounter >= 0)
            {
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
        }
    }

  m_slotTickEvent =
    Simulator::Schedule (m_slotTickPeriod,
                         &CsrMacCore::SlotTick,
                         this);
}

// SendHello() moved after CsrNetDevice definition (see below)

// ------------------------------------------------------------
// PHY model for link quality
// ------------------------------------------------------------

struct CsrRxDecision
{
  bool   success;
  double pathlossDb;
  double snrDb;
  double per;   // packet error rate (0..1)
};

struct CsrPhyProfile
{
  double txPowerDbm      = 0.0;
  double noiseFloorDbm   = -100.0;
  double rxBwHz          = 1e6;   // start at 1 MHz (matches the report’s receiver BW usage in MATLAB appendix)

  // PL(d) = refLossDb + 10*n*log10(d/1m)
  double refLossDb       = 60.0;   // placeholder, tune later
  double pathlossExp     = 2.0;    // placeholder, tune later

  // Optional: lets you scale your toy distances to "km-ish" without changing topology
  double distanceScale   = 1.0;

  // Default DACK/ACK/perf-friendly behavior at short range.
};

// A BER/PER model hook: return PER in [0,1] given (rate, snr, bits).
using CsrPerModelFn = std::function<double(int /*rateKbps*/, double /*snrDb*/, uint32_t /*nBits*/)>;

// Default placeholder PER model (what you have today, just factored out)
static double
CsrPerModelPlaceholder (int rateKbps, double snrDb, uint32_t nBits)
{
  double snrThresh;
  if      (rateKbps <= 8)   snrThresh = -6.0;
  else if (rateKbps <= 16)  snrThresh = -3.0;
  else if (rateKbps <= 32)  snrThresh =  0.0;
  else if (rateKbps <= 64)  snrThresh =  3.0;
  else                      snrThresh =  6.0;

  double x = (snrDb - snrThresh);
  double per = 1.0 / (1.0 + std::exp (3.0 * x));

  // crude packet-size effect
  double sizeFactor = std::min (5.0, std::max (1.0, nBits / 800.0));
  per = 1.0 - std::pow (1.0 - per, sizeFactor);

  if (per < 0.0) per = 0.0;
  if (per > 1.0) per = 1.0;
  return per;
}

static double
CsrRateKeyToBps (int rateKbpsKey)
{
  switch (rateKbpsKey)
    {
    case 8:   return 7812.5;    // 64-chip
    case 16:  return 15625.0;   // 32-chip
    case 32:  return 31250.0;   // 16-chip
    case 64:  return 62500.0;   // 8-chip
    case 128: return 125000.0;  // 4-chip
    default:  return rateKbpsKey * 1000.0; // fallback
    }
}
