# ARL neighbor admission and hop-security foundation

Updated: 2026-08-24

## Implemented scope

This implementation reproduces the legacy ARL neighbor-admission lifecycle and
the enabled target build's KeyRequest, KeyUpdate, GroupEstablish, Group16, and
Group32Encrypt cryptography. The model uses the source's SHA-1, HMAC-SHA1,
one-iteration PBKDF2 derivations, AES-256-CBC group-key wrapping, AES-256-CTR
group encryption, 128-entry pairwise and group replay windows, reliable
completion, and admission decisions. Embedded flash and key-manager access are
replaced by injectable synthetic mission, pairwise, and group keys.

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
| RoutingUpdate from inactive peer is processed while CHECK_MESSAGE starts | `ProcessArlRouteMessage()` |
| Remote security-count change invalidates old key/admission state after authentication | `CsrHopSecurityState::CommitAuthenticatedPairwise()` / `NoteSecurityCountChange()` |

## Security records

| Packet/mode | Field layout | Security overhead | Current live use |
| --- | --- | ---: | --- |
| KeyRequest | packed 12-bit pairwise key ID + 12-bit sequence (3), auth tag (4) | 7 bytes | No ACK, no resend |
| KeyUpdate | packed key ID/sequence (3), reserved/group-key ID (2), data PRN wire field (6), wrapped group key (32), auth tag (8) | 51 bytes | ACK/resend |
| GroupEstablish | packed group key ID/sequence (3), AES-CTR ciphertext, mission/group auth tag (4) | 7 bytes | Discover broadcasts |
| Group16 | packed group key ID/sequence (3), plaintext, group auth tag (2) | 5 bytes | Routing broadcasts and reliable routing control |
| Group32Encrypt | packed group key ID/sequence (3), AES-CTR ciphertext, group auth tag (4) | 7 bytes | Portable provider and golden tests; live app-neighborcast wrapper remains |

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

- KeyRequest and KeyUpdate share one outbound pairwise key/sequence stream per
  neighbor.
- GroupEstablish, Group16, and Group32Encrypt share one outbound group
  key/sequence stream. Sequence rollover evolves the group key one way; every
  16th key ID generates a fresh group key and invalidates all outbound
  distribution-complete flags.
- Inbound pairwise key/sequence state uses the target `SLIDINGWINDOW`
  configuration's 128-entry window. Group traffic has an independent window
  with the same semantics. Authenticated out-of-order records inside a window
  are accepted; authenticated duplicates and records before the window are
  rejected.
- GroupEstablish authenticates its first two tag bytes with the first 20 bytes
  of the mission key. This permits source authentication without the current
  group key. Only the newest deferred Discover per source is retained, and it
  is decrypted after the KeyUpdate callback installs the missing key.
- Group16 and Group32Encrypt require the sender's current key, the immediately
  previous key, or a derivable future key in the same 16-key epoch. Failed
  authentication does not advance replay or committed key state.
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
  reciprocal keys, NeighborCheck activation, admitted route selection, and
  exactly-once post-admission delivery.
- `csr-hop-group-security-smoke.cc` checks independent golden records for all
  three group modes, both GroupEstablish tag halves, wrong-key and tamper
  rejection, deferred-record replacement/replay, callback ordering,
  authentication-before-ACK, same-epoch derivation, previous-key acceptance,
  and fresh 16-key epoch rollover.
- The complete build and regression pass: 21 focused smoke targets plus
  `csr-mac-demo-split` (22/22).

## Deliberate boundary

The enabled GroupEstablish and Group16 paths are connected end to end for
Discover and routing traffic. Group32Encrypt is implemented and verified in
the portable provider, but its legacy AppNeighborcast consumer does not yet
have an exact live ns-3 packet wrapper. Pairwise16, Pairwise32Encrypt, and the
network-security modes also remain outside this step.

The default ns-3 key arrays are deterministic and non-secret so existing
scenarios remain runnable; parity or security experiments should explicitly
inject synthetic per-scenario mission, pairwise, and group keys. The model does
not reproduce flash storage, operational key loading, or key-manager hardware,
and no operational keys are stored in the repository. The group-security flag
is carried in the existing ns-3 HOP compatibility envelope; complete `br_Hop`
packet-model equivalence is still part of the remaining packet-wrapper audit.
