# ARL neighbor admission and hop-security foundation

Updated: 2026-09-02

## Implemented scope

This implementation reproduces the legacy ARL neighbor-admission lifecycle and
the enabled target build's Pairwise16, Pairwise32, Pairwise32Encrypt,
KeyRequest, KeyUpdate, GroupEstablish, Group16, and Group32Encrypt
cryptography. The model uses the source's SHA-1, HMAC-SHA1, one-iteration
PBKDF2 derivations, AES-256-CBC group-key wrapping, AES-256-CTR pairwise/group
encryption, 128-entry pairwise and group replay windows, reliable completion,
admission decisions, and explicit production Pairwise16 versus hash-bound
historical bare ordinary-DATA and ACK/DACK wrappers.
Embedded flash and key-manager access are replaced by injectable synthetic
mission, pairwise, and group keys.

The admission invariant is:

> A fresh radio neighbor is not a usable ARL neighbor until this node has
> received its group key, this node's reciprocal KeyUpdate has been ACKed, and
> a NeighborCheck exchange proves the relationship.

Freshness and ARL activation are therefore separate states. Routes through an
inactive neighbor are retained as transaction state but cannot be selected or
advertised.

## Source-to-model mapping

| Legacy behavior | ns-3 location |
| --- | --- |
| `routesRcvUnAuthedMsg(Discover)` sends KeyRequest and resets request/send delays to 5000 ms | `CsrNetLayer::EvaluateNeighborAdmission()` / `SendKeyRequest()` |
| KeyRequest uses Pairwise32, unicast, `NoAck`, `NoResend`, and neighbor immediate tag | `CsrHopLayer::SendKeyRequest()` / `CsrMacCore::CancelQueuedFramesByType()` |
| Received KeyRequest sends KeyUpdate unless a previously ACKed key is already recorded | `CsrNetLayer::NoteKeyRequestReceived()` |
| KeyUpdate is reliable; receive ACKs it and starts a reciprocal KeyUpdate | `CsrHopLayer` KeyUpdate receive path / `CsrNetLayer::NoteKeyUpdateReceived()` |
| Pairwise protection keys use MNK + pairwise material + A5/5A pads and source/count/key-ID salt | `CsrLegacyCrypto::DeriveProtectionKeys()` |
| Production AppData/AppDataNoAck and common `AckMsg` use Pairwise16; archived executable `adb97c54...` directly wraps ordinary DATA and `br_Ack` without security | `CsrHopLayer::SendDataFrame()`, `ProtectAckFrame()`, and receive validation selected by the atomic HOP-security profile |
| KeyRequest authenticates its packed key-ID/sequence with a four-byte Pairwise32 HMAC | `CsrHopSecurityState::BuildKeyRequest()` / `ReceiveKeyRequest()` |
| KeyUpdate authenticates the plaintext group key, derives a pairwise wrapping key, and uses AES-256-CBC | `CsrHopSecurityState::BuildKeyUpdate()` / `ReceiveKeyUpdate()` |
| `getNextGroupKeyIdSeq()` advances the shared 12-bit group sequence and evolves the key at rollover | `CsrHopSecurityState::NextGroupKeySequence()` |
| GroupEstablish encrypts with AES-CTR and combines a mission-key tag half with a group-key tag half | `ProtectGroupMessage()` / `ReceiveGroupMessage()` with `GroupEstablish` |
| A mission-authenticated Discover without the sender's group key is retained until KeyUpdate completes | `HandleProtectedHello()` / `ReplayPendingGroupEstablish()` |
| Group16 authenticates plaintext routing records; Group32Encrypt authenticates ciphertext | `ProtectGroupMessage()` / `ReceiveGroupMessage()` with `Group16` or `Group32Encrypt` |
| Same-epoch future keys are derived; one previous key is retained; every 16th key ID starts a fresh epoch | `DeriveNextGroupKeyMaterial()` and group-key receive state |
| A fresh local group-key epoch invalidates distribution state and resends KeyUpdate to usable neighbors | `NotifyLocalGroupKeyChanged()` / `CsrNetLayer::NoteLocalGroupKeyChanged()` |
| `setGroupKeySentTrue()` occurs only after reliable ACK | `CsrHopSecurityState::MarkKeyUpdateAcked()` |
| Two key directions are required before NeighborCheck can activate the peer | `CsrNetLayer::TryMakeNeighborActive()` |
| CHECK_DISCOVERY is deferred until outbound KeyUpdate ACK | `SendPendingDiscoveryCheck()` |
| DATA from inactive peer is rejected and starts CHECK_MESSAGE | `AcceptFromNeighbor()` / `EnsureCheckMessage()` |
| RoutingUpdate from an inactive peer is retained while CHECK_MESSAGE starts, but its transit candidates remain selection-deferred | `ProcessArlRouteMessage()` / `CsrNetLayer::AddOrUpdateRoute()` |
| Activating a neighbor recomputes only that direct-neighbor destination; cached transit candidates do not become selectable merely because the peer was admitted | `CsrNetLayer::TryMakeNeighborActive()` / `FindBestRoute()` |
| A later source-owned recomputation of that destination reconsiders every cached candidate whose reporter is now active; a fresh accepted RoutingUpdate is one such trigger | `AddOrUpdateRoute()` / `RecomputeRoutesViaNextHop()` |
| An accepted looped UPDATE is retained as invalid but still performs the source-owned destination recomputation, so an eligible alternate reporter can become selected | `ApplyCompleteArlRoutingMessage()` / `ProcessRoutesPayload()` |
| Making a neighbor inactive deletes every transit candidate learned from that reporter even if it was not admitted yet; re-admission restores only the direct route until fresh route state arrives | `CsrNetLayer::MakeNeighborInactive()` / `TryMakeNeighborActive()` |
| Remote security-count change invalidates old key/admission state after authentication | `CsrHopSecurityState::CommitAuthenticatedPairwise()` / `NoteSecurityCountChange()` |

## Security records

| Packet/mode | Field layout | Security overhead | Current live use |
| --- | --- | ---: | --- |
| KeyRequest | packed 12-bit pairwise key ID + 12-bit sequence (3), auth tag (4) | 7 bytes | No ACK, no resend |
| KeyUpdate | packed key ID/sequence (3), reserved/group-key ID (2), data PRN wire field (6), wrapped group key (32), auth tag (8) | 51 bytes | ACK/resend |
| Pairwise16 | packed pairwise key ID/sequence (3), plaintext, auth tag (2) | 5 bytes | Default DATA and production common packet-type-0 ACK/DACK |
| Pairwise32 | packed pairwise key ID/sequence (3), plaintext, auth tag (4) | 7 bytes | Portable provider and golden tests |
| Pairwise32Encrypt | packed pairwise key ID/sequence (3), AES-CTR ciphertext, auth tag (4) | 7 bytes | Explicit encrypted one-hop DATA path |
| GroupEstablish | packed group key ID/sequence (3), AES-CTR ciphertext, mission/group auth tag (4) | 7 bytes | Discover broadcasts |
| Group16 | packed group key ID/sequence (3), plaintext, group auth tag (2) | 5 bytes | Routing broadcasts and reliable routing control |
| Group32Encrypt | packed group key ID/sequence (3), AES-CTR ciphertext, group auth tag (4) | 7 bytes | Non-ACKed AppNeighborcast broadcast |

`arlSecurity.h` defines `PSEUDORANDOM_NUMBER_SIZE_IN_BYTES` as five bytes, but
the legacy HOP-security packet construction reserves six bytes. The ns-3 record
preserves the six-byte observed wire layout. Crypto uses the first five bytes;
the deterministic ns-3 sender sets the sixth reserved byte to zero and the
receiver excludes it from authentication, matching the legacy call boundary.

The security count is a full 16-bit restart counter carried in a two-byte
optional field in the ns-3 HOP compatibility envelope. Key IDs and sequences
remain 12-bit fields. Existing packet sizes are unchanged when the security
count is absent.

## State and failure behavior

- Pairwise16, Pairwise32, Pairwise32Encrypt, KeyRequest, and KeyUpdate share one
  outbound pairwise key/sequence stream per neighbor.
- GroupEstablish, Group16, and Group32Encrypt share one outbound group
  key/sequence stream. Sequence rollover evolves the group key one way; every
  16th key ID generates a fresh group key and invalidates all outbound
  distribution-complete flags.
- Inbound pairwise key/sequence state uses the target `SLIDINGWINDOW`
  configuration's 128-entry window. Group traffic has an independent window
  with the same semantics. Authenticated out-of-order records inside a window
  are accepted; in-window authenticated duplicates are classified for
  packet-type-specific idempotent handling, while records before the window
  are rejected.
- GroupEstablish authenticates its first two tag bytes with the first 20 bytes
  of the mission key. This permits source authentication without the current
  group key. Only the newest deferred Discover per source is retained, and it
  is decrypted after the KeyUpdate callback installs the missing key.
- Group16 and Group32Encrypt require the sender's current key, the immediately
  previous key, or a derivable future key in the same 16-key epoch. Failed
  authentication does not advance replay or committed key state.
- Pairwise DATA authentication occurs before pairwise or HOP replay state,
  delivery, or ACK generation. An ACK-marked authenticated retransmission is
  ACKed again without a second delivery. A DACK-marked authenticated retry is
  re-enqueued and reassessed for fresh ACK/DACK selection, matching the legacy
  ACK-only duplicate test. Failed authentication cannot poison either replay
  window.
- ACK/DACK authenticates the complete logical ACK body before MAC cancellation,
  resend/custody mutation, or the packet-level delayed NWK wake. The mutable
  outer header carries only transport-visible security and link metadata.
- AppNeighborcast requires a broadcast, non-ACKable envelope and authenticates
  and decrypts before one-hop delivery. Tampered, replayed, and malformed
  envelopes are never delivered or ACKed.
- Authentication occurs before a KeyUpdate can consume HOP replay state,
  receive an ACK, install a group key, or affect ARL admission.
- A changed remote security count replaces the old neighbor security state,
  cancels an obsolete outbound KeyUpdate, and returns the NWK neighbor to
  inactive only after the first packet under the new count authenticates.
- A KeyUpdate enqueue does not complete the outbound-key state. Only its HOP
  ACK records `groupKeySent` and the send-completion time.
- Reliable KeyUpdate and NeighborCheck exhaustion are reported to NWK so their
  phases can retry.
- KeyRequest retries and key/check retry foundations use the source's
  millisecond units and 5000-ms starting delay.

## Verification

- `csr-hop-security-crypto-smoke.cc` checks standard SHA-1, HMAC-SHA1,
  PBKDF2, AES-256-CBC, and AES-CTR vectors; independently generated 7-byte and
  51-byte legacy security vectors; group-key recovery; tamper and wrong-key
  rejection; authentication-before-ACK; the 128-entry replay window; and
  16-bit security-count wrap.
- `csr-wire-format-smoke.cc` checks hand-built record layouts, header round
  trips, shared pairwise sequencing, ACK-only key-send completion,
  authenticated replay classification, and security-count reset.
- `csr-nwk-arl-admission-security-smoke.cc` checks pre-admission DATA rejection,
  reciprocal keys, NeighborCheck activation, admitted direct-route selection,
  a real ARL byte-stream UPDATE cached while its reporter is inactive,
  remaining unselected through admission, and becoming selectable when a fresh
  accepted UPDATE triggers the later destination recomputation. It also uses a
  real looped UPDATE from a different reporter to prove that the same
  recomputation releases an eligible cached alternate, then proves that
  both pre-admission failure and active-peer failure delete learned transit
  state. Re-admission cannot resurrect that state without a fresh UPDATE. The
  test also retains exactly-once post-admission delivery.
- `csr-hop-group-security-smoke.cc` checks independent golden records for all
  three group modes, both GroupEstablish tag halves, wrong-key and tamper
  rejection, deferred-record replacement/replay, callback ordering,
  authentication-before-ACK, same-epoch derivation, previous-key acceptance,
  and fresh 16-key epoch rollover.
- `csr-hop-remaining-security-smoke.cc` checks independent Pairwise16,
  Pairwise32, and Pairwise32Encrypt golden records; plaintext/ciphertext round
  trips; wrong-key, wrong-type, tamper, malformed, and replay rejection;
  authentication-before-ACK/delivery; plaintext/tampered/malformed ACK
  rejection; authenticated ACK/DACK state and wake ordering; live 46-byte
  production Pairwise16 and 41-byte hash-bound historical cumulative DACKs,
  222-byte production and 217-byte historical ordinary DATA with exact PHY bit
  accounting and matching live receive, impossible-metadata and declared
  cross-profile rejection before replay state, and a synthetic 25-byte bare exact-envelope
  compatibility ACK and its custody/wake path (not an archived transmit
  claim); exact-once AppNeighborcast; and live over-air
  sends for Pairwise16, Pairwise32Encrypt, and Group32Encrypt.
- `csr-hop-neighbor-check-security-smoke.cc` checks the source-exact
  packet-type-9 Pairwise16 NeighborCheck wrapper policy, authentication before
  ACK/delivery, all five operational subtypes, authenticated duplicate re-ACK
  with exact-once delivery, an exact 30-byte protected ACK, byte-identical
  protected retries, and preserved success/final-failure metadata.
- `csr-nwk-residual-routing-retry-smoke.cc` checks that NWK retains the exact
  grouped plaintext owner while HOP retries the original multicast, removes
  one destination per ACK, defers final-NACK recovery to a later same-time
  event, prunes inactive residuals, reuses unchanged routing bytes/metadata,
  and assigns fresh HOP sequences only to the ordered residual group.
- `csr-nwk-same-pass-info-coupling-smoke.cc` checks exact INFO-first mixed
  INFO/UPDATE/DELETE bytes, INFO-only final-value coalescing, 10+1 neighbor
  grouping, no-recipient consumption without replay or sequence advancement,
  and exact 694-/83-byte body slicing for a 777-byte logical stream.
- The complete build and regression pass: 36 focused smoke targets,
  `csr-mac-demo-split`, and the imported-scenario runner (38/38).

## Deliberate boundary

All enabled HOP-security transforms now have portable provider coverage.
Pairwise16 is connected to production ordinary DATA, the common ACK/DACK packet type, and
packet-type-9 NeighborCheck; Pairwise32Encrypt has an explicit one-hop DATA
path; and Group32Encrypt is connected to AppNeighborcast. The ACK/DACK
Pairwise16 policy is source-exact. The adb97 profile atomically selects direct
bare ordinary DATA and ACK/DACK, and its full executable digest is required in
canonical certifying scenarios. The ns-3 logical-body byte mapping remains
unverified without a production golden protected record. Clearing a
Pairwise16 frame's outer security-count flag can leave bytes indistinguishable
from an arbitrary bare payload because `br_Network` has no authenticated magic
or length discriminator; no stronger body-heuristic rejection is claimed.
Reassessing a
Pairwise16 authenticated duplicate when its HOP sequence is DACK-marked is an
inferred cross-source integration: `br_hop.pr.c` proves the HOP retry rule, but
the archived OPNET executable has no production security dispatcher proving
how an authenticated duplicate reaches that rule. The outer HOP sequence is
not authenticated, so relabeling between two simultaneously DACK-marked
sequences remains an unresolved production-security binding uncertainty. For NeighborCheck,
the packet type, Pairwise16 mode, reliable-unicast ownership, retry record, and
five-byte security overhead are source-exact. The byte-for-byte mapping of the
current ns-3 `CsrHelloHeader` compatibility body to the production
NeighborCheck body is unresolved. Its bounds check is an intentional defensive
extension that runs only after HOP authentication. Exact wrapping for other
remaining control messages, the distinct network-security modes, and
operational key-store behavior still require packet-type-table or
differential-trace confirmation before they can be claimed as equivalent.

This route-admission increment covers the inactive-peer selection boundary and
source-owned destination recomputation after normal or looped UPDATEs,
selected-route DELETE/FLUSH/stale fallback, inactive-reporter route deletion,
and eligible link-cost reroutes.
Source-owned self-route capability advertisement is now covered, including
zero-hop UPDATE serialization, HELLO-independent receive authority, and
DELETE-to-ordinary behavior that preserves direct reachability. Grouped
same-time propagation is also source-exact through the NWK-to-HOP handoff,
including an optional INFO record before combined UPDATE/DELETE records,
INFO-only sends, destination/neighbor creation order, section splitting,
ten-neighbor grouping, sequence advancement, no-recipient consumption, and
same-pass snapshot exclusion. The INFO power and margin fields also preserve
the wrapper's signed `int16` conversion, including truncation toward zero.
NWK-owned residual retry is now source-backed for those grouped automatic
updates: partial ACKs mutate only NWK's retained destination set, HOP keeps its
whole-frame retry ownership, and a final NACK schedules an exact-payload retry
after HOP releases custody. The routes-library all-or-none buffer check is
preserved behind a default-not-full seam because the supplied operational
OPNET wrapper hardcodes `hopSecCheckBufferFull()` to return zero.

The changed-value INFO path is source-exact. Whether a legacy
`routesNotifyChange()` call whose newly polled fields are byte-identical still
sets the INFO dirty flag cannot be rechecked from the currently materialized
source set. ns-3 suppresses that wire-identical setter case as inferred legacy
behavior; it does not affect the source-confirmed changed-value path.

The default ns-3 key arrays are deterministic and non-secret so existing
scenarios remain runnable; parity or security experiments should explicitly
inject synthetic per-scenario mission, pairwise, and group keys. The model does
not reproduce flash storage, operational key loading, or key-manager hardware,
and no operational keys are stored in the repository. The group-security flag
is carried in the existing ns-3 HOP compatibility envelope; complete `br_Hop`
packet-model equivalence is still part of the remaining packet-wrapper audit.
