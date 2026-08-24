# ARL neighbor admission and hop-security foundation

Updated: 2026-08-21

## Implemented scope

This implementation reproduces the legacy ARL neighbor-admission lifecycle and
establishes final-size KeyRequest/KeyUpdate records. Counters, replay state,
reliable completion, and admission are live, while cryptographic byte arrays
remain zero-filled using the legacy source's compile-time non-`SECURITY`
fallback. The subsequently supplied target `parameters.h` defines `SECURITY`,
so this is an integration foundation rather than final target-build crypto
parity.

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
| `setGroupKeySentTrue()` occurs only after reliable ACK | `CsrHopSecurityState::MarkKeyUpdateAcked()` |
| Two key directions are required before NeighborCheck can activate the peer | `CsrNetLayer::TryMakeNeighborActive()` |
| CHECK_DISCOVERY is deferred until outbound KeyUpdate ACK | `SendPendingDiscoveryCheck()` |
| DATA from inactive peer is rejected and starts CHECK_MESSAGE | `AcceptFromNeighbor()` / `EnsureCheckMessage()` |
| RoutingUpdate from inactive peer is processed while CHECK_MESSAGE starts | `ProcessArlRouteMessage()` |
| Remote security-count change invalidates old key/admission state | `CsrHopSecurityState::AcceptPairwiseSequence()` / `NoteSecurityCountChange()` |

## Security records

| Packet | Field layout | Size | Reliability |
| --- | --- | ---: | --- |
| KeyRequest | packed 12-bit pairwise key ID + 12-bit sequence (3), auth tag (4) | 7 bytes | No ACK, no resend |
| KeyUpdate | packed key ID/sequence (3), reserved/group-key ID (2), data PRN wire field (6), wrapped group key (32), auth tag (8) | 51 bytes | ACK/resend |

`arlSecurity.h` defines `PSEUDORANDOM_NUMBER_SIZE_IN_BYTES` as five bytes, but
the legacy HOP-security packet construction reserves six bytes. The ns-3 record
preserves the six-byte observed wire layout; real crypto should populate the
five API bytes and explicitly define the remaining wire byte.

The security count is a 12-bit value carried in a two-byte optional field in
the ns-3 HOP compatibility envelope. Existing packet sizes are unchanged when
the field is absent.

## State and failure behavior

- KeyRequest and KeyUpdate share one outbound pairwise key/sequence stream per
  neighbor.
- Inbound pairwise key/sequence state rejects duplicate or stale records.
- A changed remote security count replaces the old neighbor security state,
  cancels an obsolete outbound KeyUpdate, and returns the NWK neighbor to
  inactive.
- A KeyUpdate enqueue does not complete the outbound-key state. Only its HOP
  ACK records `groupKeySent` and the send-completion time.
- Reliable KeyUpdate and NeighborCheck exhaustion are reported to NWK so their
  phases can retry.
- KeyRequest retries and key/check retry foundations use the source's
  millisecond units and 5000-ms starting delay.

## Verification

- `csr-wire-format-smoke.cc` checks golden 7-byte and 51-byte vectors, header
  round trips, shared pairwise sequencing, ACK-only key-send completion,
  replay rejection, and security-count reset.
- `csr-nwk-arl-admission-security-smoke.cc` checks pre-admission DATA rejection,
  reciprocal keys, NeighborCheck activation, admitted route selection, and
  exactly-once post-admission delivery.
- The entire current regression baseline passes: 19 smoke targets plus
  `csr-mac-demo-split`.

## Deliberate boundary

No packet is cryptographically authenticated or encrypted yet. Completing that
work requires the internal derivation implementation, AES and SHA-160
implementations, and a portable replacement or model for flash-backed mission
and pairwise-key provisioning. The supplied follow-on bundle provides
`hmacBasedOnSha160.*`, `arlKeyLookup.*`, `parameters.h`, `slidingw.*`, `aes.h`,
and `arlSecurityInternal.h`. However, its internal header omits
security-count arguments that are present at the corresponding call sites in
the supplied `arlSecurity.c`; a matching `arlSecurityInternal.c/.h` pair or
known-good crypto vectors is therefore needed before exact crypto integration.
The new `CsrHopSecurityState` is the seam for that provider; the ARL admission
state machine should not need to be redesigned.
