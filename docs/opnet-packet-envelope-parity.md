# OPNET packet and control-envelope parity

Updated: 2026-08-26

## Scope

This increment ports the fixed-bit definitions from every supplied
`br_*.pk.m` packet model and applies their complete wrapper nesting to MAC
concatenation, PHY packet length, and payload-airtime calculations.

OPNET packet fields fall into two different categories:

- Fixed-bit fields contribute to `op_pk_total_size_get()` and appear in the
  direct serializer.
- Pointer, structure, annotation, and explicitly zero-bit fields carry
  simulator state but contribute no bits.  The ns-3 compatibility headers
  retain this state, while `CsrOpnetEnvelopeTag` prevents its storage bytes
  from changing modeled packet size.

This split is deliberate.  Removing the compatibility state would change
control behavior; charging it to the radio would change the OPNET packet
model.

## Exact supplied packet formats

| Format | Fixed modeled fields, in order | Fixed size |
| --- | --- | ---: |
| `br_Ack` | Source 24, Destination 24, Sequence 16 | 8 B |
| `br_Hello` | Sequence 8, Node ID 24, Capability Number 8, Capability 8, Cost 16, Time Offset 32 | 12 B |
| `br_Hop` | Source 24, destination count 8, Destination 24, Sequence 8, inherited Payload | 8 B + payload |
| `br_Mac` | Global Time 32, Global Address 24, Global Cost 8, Tx Power 8, Rx Power 8, Active 8, Source 24, Payload Length 16, Type 8, inherited Payload | 17 B + payload |
| `br_Mac_Hop_Inst` | Source 24, Destination 24, Sequence 16 | 8 B |
| `br_Network` | Source 24, Destination 24, inherited Payload, DSCP 8 | 7 B + payload |
| `br_OTA_LongPream` | Preamble 7,888, Start of Frame 16, Speed 16, Length 16, inherited Payload, FCS 32 | 996 B + payload |
| `br_OTA_ShortPream` | Preamble 104, Start of Frame 16, Speed 16, Length 16, inherited Payload, FCS 32 | 23 B + payload |
| `br_Routes` | Node ID 24, Capability Number 8, Capability 8, Cost 16, Time Offset 32 | 11 B |
| `br_SNMP` | Message 16, Value 32 | 6 B |
| `br_Sent_Info` | Source 24 | 3 B |

The direct serializer uses big-endian field encoding and preserves inherited
payload position.  In particular, `br_Network` places DSCP after its inherited
payload; this differs from the layout of the old seven-byte compatibility
header even though their fixed sizes are equal.

The preamble fields are never assigned by `br_mac.pr.c`.  The serializer emits
their unset bits deterministically as zero while retaining their exact 104- or
7,888-bit lengths.

## Complete envelope nesting

The executable wrapper, not format names alone, determines the modeled size:

| Logical traffic | OPNET envelope tree | Modeled fixed bytes before application or security content |
| --- | --- | ---: |
| DATA | `br_Mac -> br_Hop -> br_Network` | 32 B |
| ACK or DACK | `br_Mac -> br_Ack` | 25 B |
| Routed SNMP | `br_Mac -> br_Hop -> br_SNMP` | 31 B |
| HELLO | `br_Hello` | 12 B |
| ARL route control | `br_Routes` | 11 B |

`br_mac.pr.c` concatenates packet objects through an OPNET segmentation
buffer.  There is no supplied extra segment header, so an aggregate is the
strict sum of its member envelope sizes.  The 16-segment cap and per-rate byte
limits are then applied to that sum. The supplied code defines packing limits
only for 8-128 kbit/s. The owner-confirmed extended 500/1000 profile therefore
transmits one valid queue head at a time instead of inventing a capacity.

Enabled security records are charged in addition to their fixed OPNET control
wrapper:

- GroupEstablish HELLO/Discover adds seven bytes.
- Group16 HELLO adds five bytes.
- KeyRequest and KeyUpdate add their exact seven- and 51-byte records to
  `br_Routes`.
- Reliable Group16 routing control adds five security bytes plus actual ARL
  section bytes; the compatibility `CsrHelloHeader` storage is excluded.

## Known source-version discrepancy

The supplied `br_Hop.pk.m` declares an eight-bit `Sequence_Number`.  The
supplied OPNET HOP wrapper sets `MAX_SEQ_NUM` to 65,535, and the embedded
`hopLayer.c` implementation serializes a 16-bit sequence.  The direct packet
model serializer follows the `.pk.m` file and has an eight-bit HOP sequence;
the operational ns-3 HOP state remains 16-bit to preserve the independently
source-backed ACK/replay behavior.  This conflict is documented rather than
silently choosing one source for both purposes.

## Verification boundary

`csr-opnet-packet-envelope-smoke.cc` covers:

- exact fixed sizes for all eleven supplied formats;
- golden big-endian bytes for every non-OTA format;
- inherited payload placement and round trips;
- short- and long-preamble sizes and OTA field order;
- zero-bit metadata exclusion;
- packet-tag copying and aggregate summation;
- complete DATA, ACK/DACK, routed-SNMP, HELLO/Discover, NeighborCheck, ARL
  routing-section, KeyRequest, and KeyUpdate compatibility-envelope inference.

The exact serializer now also drives source-derived OTA duration: the preamble,
SOF, speed, and length fields use the S0 chip duration from `br_txdel.ps.c`,
while the inherited payload and 32-bit FCS use the selected payload rate. The
MAC receive model now covers acquisition, collision/capture, half duplex,
slot-counter freezing, and duty-cycle wake behavior. Received power, receiver
qualification, interference, exact table samples, interval errors, ECC, and
closure are now live. The default remains the source-exact 8-128-kbit/s rate
set; the actual radio's 500/1000-kbit/s modes are available through the
explicit owner-confirmed `EXTENDED_DQPSK` profile. Both automatic selection
and explicit high-rate transmissions must be within that active profile.
