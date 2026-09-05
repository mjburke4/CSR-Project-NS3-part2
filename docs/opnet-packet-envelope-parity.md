# OPNET packet and control-envelope parity

Updated: 2026-09-05

## Scope

This increment ports the fixed-bit definitions from every supplied
`br_*.pk.m` packet model and applies their complete wrapper nesting to MAC
concatenation, PHY packet length, and payload-airtime calculations.

OPNET packet fields fall into two different categories:

- Always-set fixed-bit fields contribute to `op_pk_total_size_get()` and
  appear in the direct serializer. Optional fixed-bit fields contribute only
  when set; the two `br_Ack` registers are carried by `CsrHeader` and charged
  conditionally by the envelope model rather than added to the eight-byte
  direct `br_Ack` base serializer.
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
| `br_Ack` | Source 24, Destination 24, Sequence 16; optional ACK and DACK registers, 64 bits each | 8 B base; 24 B with both registers set |
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
| Exact control ACK | `br_Mac -> br_Ack` | 25 B before security |
| Cumulative DATA ACK or DACK | `br_Mac -> br_Ack` with both 64-bit registers set | 41 B before security |
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
- Pairwise16 adds five bytes to `br_Mac -> br_Ack`: exact control ACKs are
  30 bytes, while cumulative DATA ACK/DACK feedback is 46 bytes.
- KeyRequest and KeyUpdate add their exact seven- and 51-byte records to
  `br_Routes`.
- Reliable Group16 routing control adds five security bytes plus actual ARL
  section bytes; the compatibility `CsrHelloHeader` storage is excluded.

### Archived OPNET versus production HOP security

The 30- and 46-byte Pairwise16 ACK sizes combine two independently
authoritative sources.  The supplied `br_Ack.pk.m` and `br_mac.pr.c` establish
25-byte exact-envelope arithmetic and a 41-byte envelope when both cumulative
registers are set. Production `packetTypes.c` maps the common `AckMsg` type to
Pairwise16, which adds five bytes.

The hash-bound campus-multihop executable
`adb97c54f7566439f1404e972d3d777a3bca613e2a965bf12f03353fb009d9af`
does not contain `packetTypes.c`, `hopSecLayer.c`, a `packetDecode` symbol, or
a project security-library dependency. Its compiled `send_ack()` directly
creates `br_Ack`, passes it through `br_Mac`, and MAC uses the resulting packet
size for concatenation and OTA duration. All five compiled callers pass a null
`PacketRxInfo`; because both cumulative fields exist, `send_ack()` sets both
64-bit registers on every ACK/DACK produced by this process. The archived
executable therefore directly establishes 41-byte cumulative ACK/DACK
emission. The 25-byte exact form remains valid packet-format arithmetic and a
compatibility receive case, but neither the supplied source nor the compiled
call graph shows that the archived executable emitted it. The 30/46-byte
accounting is source-exact production-security policy layered onto the two
OPNET envelope forms.

The same executable boundary applies to ordinary DATA. The archived
`check_nwk_queue()` path creates `br_Hop` around the raw `br_Network` packet,
and `proc_nwk_pk()` creates `br_Mac` around that result; neither compiled path
calls a security dispatcher. The supplied `br_nwk.pr.c` and `br_hop.pr.c`
sources show the same direct wrappers. A configured 200-byte application
packet becomes a 192-byte `br_Network` packet, so the archived wire envelope is
`192 + 8 + 17 = 217` bytes with no authentication record. Production
Pairwise16 adds five bytes and remains 222 bytes.
Historical aggregate comparisons must retain this source-version boundary;
they cannot use a 46-byte ns-3 result as packet-level proof that the archived
OPNET executable transmitted a 46-byte ACK. Both archived and production
sizes exceed the 8-kbit/s strict concatenation boundary beside their matching
DATA frame: `41 + 217 = 258` and `46 + 222 = 268`. The earlier 30-byte
cumulative approximation (`30 + 222 = 252`) incorrectly admitted that DATA
segment, so its apparently closer aggregate result was a compensating error.

The recovered hidden-node executable lineage has the same bare HOP envelope
boundary. Its 592-byte logical DATA body is wrapped by eight-byte `br_Hop` and
17-byte `br_Mac` envelopes, producing a 617-byte MAC payload. Its cumulative
ACK/DACK remains the bare 41-byte form. The two together sum to 658 bytes;
they cannot concatenate under the source's strict 8-kbit/s `< 256`-byte
packing rule. Both `hist-adb97c54-bare` and the separately hash-bound
`hist-dd3f38e8-bare` profile use this arithmetic; their distinct profile names
preserve executable provenance rather than implying different packet widths.

This boundary is now executable rather than documentary only. The canonical
atomic `hop_security_profile` value `production-pairwise16` remains the
default and retains Pairwise16 ordinary DATA plus 30/46-byte ACK accounting.
The narrowly scoped `hist-adb97c54-bare` value emits 217-byte ordinary DATA
and the source-evidenced bare 41-byte cumulative ACK/DACK, while retaining
25-byte exact-envelope receive compatibility. Impossible ordinary-DATA
metadata and profile-mismatched declared security are rejected before replay,
ACK, or delivery state. The archived value is rejected unless paired with that executable's
`legacy-send-only-no-dscp` application and
`hist-2014-next-tslot-modulo-probe` MAC profiles. The profile name, full
executable hash binding, and canonical scenario file hash are recorded in the
aggregate workflow manifest; topology and scenario names never select the
mode implicitly.

This envelope check is not a cryptographic format oracle. If a production
Pairwise16 frame's outer security-count-present flag is cleared, its remaining
record bytes can be parsed as an arbitrary bare payload; the recovered bare
`br_Network` format has no magic value or authenticated length discriminator
that safely proves otherwise. Canonical profile and executable-hash binding is
therefore the downgrade boundary. The model does not claim that body-shape
heuristics can distinguish every deliberately relabeled record.

The source boundary also applies to protected DATA retries. The legacy HOP
source directly proves that a DACK-marked sequence is not a duplicate and is
re-enqueued for a fresh decision. The production packet-type table proves the
Pairwise16 mode, but the recovered material does not include a production
dispatcher showing how an authenticated security replay is returned to HOP.
ns-3's authenticated-duplicate-to-DACK-retry bridge is therefore inferred
legacy integration, not source-exact dispatcher behavior. Because Pairwise16
does not authenticate the outer HOP sequence, cross-relabeling between two
simultaneously DACK-marked sequences remains unresolved.

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
- complete DATA, bare and Pairwise16 ACK/DACK, routed-SNMP, HELLO/Discover,
  NeighborCheck, ARL routing-section, KeyRequest, and KeyUpdate
  compatibility-envelope inference;
- the archived hidden-node 617-byte DATA and 41-byte cumulative ACK/DACK
  boundary; and
- literal 16-bit OTA Speed plus low-16-bit Length serialization semantics.

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
