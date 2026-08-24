# ARL neighbor admission and hop-security foundation

Updated: 2026-08-24

## Implemented scope

This implementation reproduces the legacy ARL neighbor-admission lifecycle and
the enabled target build's KeyRequest/KeyUpdate cryptography. The model now
uses the source's SHA-1, HMAC-SHA1, one-iteration PBKDF2 derivations,
AES-256-CBC group-key wrapping, 128-entry pairwise replay window, reliable
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
| `setGroupKeySentTrue()` occurs only after reliable ACK | `CsrHopSecurityState::MarkKeyUpdateAcked()` |
| Two key directions are required before NeighborCheck can activate the peer | `CsrNetLayer::TryMakeNeighborActive()` |
| CHECK_DISCOVERY is deferred until outbound KeyUpdate ACK | `SendPendingDiscoveryCheck()` |
| DATA from inactive peer is rejected and starts CHECK_MESSAGE | `AcceptFromNeighbor()` / `EnsureCheckMessage()` |
| RoutingUpdate from inactive peer is processed while CHECK_MESSAGE starts | `ProcessArlRouteMessage()` |
| Remote security-count change invalidates old key/admission state after authentication | `CsrHopSecurityState::CommitAuthenticatedPairwise()` / `NoteSecurityCountChange()` |

## Security records

| Packet | Field layout | Size | Reliability |
| --- | --- | ---: | --- |
| KeyRequest | packed 12-bit pairwise key ID + 12-bit sequence (3), auth tag (4) | 7 bytes | No ACK, no resend |
| KeyUpdate | packed key ID/sequence (3), reserved/group-key ID (2), data PRN wire field (6), wrapped group key (32), auth tag (8) | 51 bytes | ACK/resend |

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
- Inbound pairwise key/sequence state uses the target `SLIDINGWINDOW`
  configuration's 128-entry window. Authenticated out-of-order records inside
  the window are accepted; authenticated duplicates and records before the
  window are rejected.
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
- The complete build and regression pass: 20 focused smoke targets plus
  `csr-mac-demo-split` (21/21).

## Deliberate boundary

This step completes enabled crypto for KeyRequest and KeyUpdate, not every HOP
security mode. `GroupEstablish`, `Group16`, and `Group32Encrypt`, including
queued unauthenticated group traffic, remain to be connected to the portable
provider. The default ns-3 key arrays are deterministic and non-secret so old
scenarios remain runnable; parity or security experiments should explicitly
inject synthetic per-scenario mission, pairwise, and group keys. The model does
not reproduce flash storage, operational key loading, or key-manager hardware,
and no operational keys are stored in the repository.
