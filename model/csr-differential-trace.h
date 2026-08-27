#pragma once

#include "csr-wire-format.h"
#include "ns3/simulator.h"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace ns3 {

/** Canonical ordered trace schema consumed by the OPNET/ns-3 comparator. */
struct CsrDifferentialTraceEvent
{
  std::string event;
  std::string node;
  std::string peer;
  std::string packetType;
  std::string source;
  std::string destination;
  std::string sequence;
  std::string rateKbps;
  std::string sizeBytes;
  std::string success;
  std::string reason;
  std::string pathlossDb;
  std::string rxPowerDbm;
  std::string noiseDbm;
  std::string snrDb;
  std::string jsrDb;
  std::string headerErrors;
  std::string payloadErrors;
  std::string totalErrors;
  std::string routeCost;
  std::string nextHop;
  std::string securityCount;
  std::string reservationSlot;
  std::string reservationCounter;
  std::string detail;
};

inline std::ofstream g_csrDifferentialTrace;
inline uint64_t g_csrDifferentialTraceIndex = 0;

inline std::string
CsrTraceCsvEscape (const std::string &value)
{
  if (value.find_first_of (",\"\r\n") == std::string::npos)
    {
      return value;
    }
  std::string escaped = "\"";
  for (char character : value)
    {
      if (character == '\"')
        {
          escaped += '\"';
        }
      escaped += character;
    }
  escaped += "\"";
  return escaped;
}

inline std::string
CsrTraceInteger (uint64_t value)
{
  return std::to_string (value);
}

inline std::string
CsrTraceSignedInteger (int64_t value)
{
  return std::to_string (value);
}

inline std::string
CsrTraceDouble (double value)
{
  if (value == std::numeric_limits<double>::infinity ())
    {
      return "inf";
    }
  if (value == -std::numeric_limits<double>::infinity ())
    {
      return "-inf";
    }
  std::ostringstream stream;
  stream << std::setprecision (17) << value;
  return stream.str ();
}

inline void
OpenDifferentialTraceCsv (const std::string &path)
{
  if (g_csrDifferentialTrace.is_open ())
    {
      g_csrDifferentialTrace.close ();
    }
  g_csrDifferentialTrace.open (path, std::ios::out | std::ios::trunc);
  NS_ABORT_MSG_IF (!g_csrDifferentialTrace.is_open (),
                   "cannot open CSR differential trace: " << path);
  g_csrDifferentialTraceIndex = 0;
  g_csrDifferentialTrace
    << "schema,event_index,time_s,event,node,peer,packet_type,src,dst,"
    << "sequence,rate_kbps,size_bytes,success,reason,pathloss_db,"
    << "rx_power_dbm,noise_dbm,snr_db,jsr_db,header_errors,"
    << "payload_errors,total_errors,route_cost,next_hop,security_count,"
    << "reservation_slot,reservation_counter,detail\n";
}

inline void
CloseDifferentialTraceCsv ()
{
  if (g_csrDifferentialTrace.is_open ())
    {
      g_csrDifferentialTrace.close ();
    }
}

inline bool
IsDifferentialTraceOpen ()
{
  return g_csrDifferentialTrace.is_open ();
}

inline void
WriteDifferentialTrace (const CsrDifferentialTraceEvent &event)
{
  if (!g_csrDifferentialTrace.is_open ())
    {
      return;
    }
  const std::string fields[] = {
    "csr-differential-trace-v1",
    CsrTraceInteger (g_csrDifferentialTraceIndex++),
    CsrTraceDouble (Simulator::Now ().GetSeconds ()),
    event.event,
    event.node,
    event.peer,
    event.packetType,
    event.source,
    event.destination,
    event.sequence,
    event.rateKbps,
    event.sizeBytes,
    event.success,
    event.reason,
    event.pathlossDb,
    event.rxPowerDbm,
    event.noiseDbm,
    event.snrDb,
    event.jsrDb,
    event.headerErrors,
    event.payloadErrors,
    event.totalErrors,
    event.routeCost,
    event.nextHop,
    event.securityCount,
    event.reservationSlot,
    event.reservationCounter,
    event.detail,
  };
  for (std::size_t index = 0; index < sizeof (fields) / sizeof (fields[0]); ++index)
    {
      if (index != 0)
        {
          g_csrDifferentialTrace << ',';
        }
      g_csrDifferentialTrace << CsrTraceCsvEscape (fields[index]);
    }
  g_csrDifferentialTrace << '\n';
}

} // namespace ns3
