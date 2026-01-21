#pragma once
#include "csr-common.h"

class CsrMacCore
{
public:
  CsrMacCore ()
    : m_nodeId (0),
      m_dev (nullptr)
  {}

  void SetNodeId (uint16_t id)        { m_nodeId = id; }
  void SetDevice (CsrNetDevice *dev)  { m_dev = dev; }
  int SelectRateByPerTarget (uint16_t destId, uint32_t nBits, double targetPer) const;
  void PrintNeighbors () const;

  private:
  // DiscoveryStart, DiscoveryStop defined inline below
  // SendHelloInternal defined after CsrNetDevice
  void SendHelloInternal (bool reschedule);
  // --- Slot tick for reservation decay (OPNET rtslot_counter style) ---
  EventId m_slotTickEvent;
  Time    m_slotTickPeriod { Seconds (1.0) };   // default; tune later
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


  void SetRxCallback (Callback<void, Ptr<Packet>, double, double> cb)
  {
    m_rxCallback = cb;
  }

  // Called by Hop layer to enqueue a full over-the-air frame
  void EnqueueTxFrame (Ptr<Packet> frame,
                       uint16_t dest,
                       uint8_t dscp,
                       bool ackable);

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
    NeighborInfo &info = m_neighbors[neighbor];
    info.reserveSlot = slot;
    info.rtCounter   = 4;   // simple “hold” window, tune later if you want
    std::cout << "[MAC " << m_nodeId << "] Learned neighbor " << neighbor
          << " uses slot " << slot
          << " (rtCounter=" << m_neighbors[neighbor].rtCounter << ")"
          << std::endl;
  }

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

  EventId   m_helloEvent;
  Time      m_helloInterval { Seconds (2.0) };
  uint16_t  m_helloSeq { 0 };
  bool      m_helloEnabled { false };
  bool      m_discoveryActive { false };
  EventId   m_discoveryStartEvent;
  EventId   m_discoveryStopEvent;


  void MaybeScheduleNextTx ();
  void DoTx ();

  int           ChooseRateForDest (uint16_t dest);
  PreambleType  ChoosePreambleForDest (uint16_t dest);

private:
  uint16_t                            m_nodeId;
  CsrNetDevice*                       m_dev;
  std::vector<TxQueueEntry>           m_queue;
  EventId                             m_txEvent;
  Callback<void, Ptr<Packet>, double, double>  m_rxCallback;
  std::map<uint16_t, NeighborInfo>    m_neighbors;
  Time                                m_neighborTimeout { Seconds (6.0) };   // default: 3x hello interval (if hello=2s)
  EventId                             m_neighborAgingEvent;
  std::map<uint16_t, double>          m_lastHeardSec;  // neighborId -> last heard time (seconds)
  std::map<uint16_t, double>          m_lastPathlossDb; // neighborId -> last pathloss (optional)
  //void DiscoveryStart ();
  //void DiscoveryStop ();
  //void SendHelloInternal (bool reschedule);

  int PickTxSlot (uint16_t dest)
  {
    const int SLOT_RANGE = 64;
    bool used[SLOT_RANGE];
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

void
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

      /*std::cout << "[MAC " << m_nodeId << "] SlotTick neighbor " << kv.first
                << " rtCounter=" << ni.rtCounter
                << " reserveSlot=" << ni.reserveSlot
                << std::endl;*/
    }
  }
  // Re-arm
  m_slotTickEvent = Simulator::Schedule (m_slotTickPeriod, &CsrMacCore::SlotTick, this);
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

