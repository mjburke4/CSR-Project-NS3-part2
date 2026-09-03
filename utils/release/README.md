# CSR release-certification harness

`certify_release.py` is a fail-closed release harness. It requires a clean CSR
checkout whose `HEAD` and `origin/main` equal the full tested commit, plus a
separate clean ns-3 checkout
at `6b5cd24ea80713ce16d88575869aedd6f432bdae`. It archives both Git objects into
the new output directory, makes the staged CSR tree read-only, and binds all 38
scratch targets and `contrib/csr` to that one tree.

The debug build keeps assertions and logging enabled but explicitly disables
warnings-as-errors. The supplied source contains three pre-existing unused
header-local helpers; their warnings are retained in the build log and are not
treated as release failures. The harness verifies
`NS3_WARNINGS_AS_ERRORS:BOOL=OFF` in the generated CMake cache.

Do not point `--ns3-repo` at a worktree already used for a build: untracked
build output is deliberately treated as a release failure.

Example invocation after the final CSR merge and fetch:

```bash
utils/release/certify_release.py \
  --rc-repo /absolute/path/to/clean/final-csr-checkout \
  --rc-commit FULL_40_DIGIT_FINAL_COMMIT \
  --ns3-repo /absolute/path/to/clean/ns-3-checkout \
  --source-archive /absolute/path/to/CSR-project-examples.zip \
  --supplied-source-dir /absolute/path/to/freeze-source-inputs \
  --evidence-root /workspace/scratch/c0223674fc50/opnet_evidence \
  --opnet-source-dir /workspace/scratch/c0223674fc50/project_sources \
  --cmake-bin-dir /workspace/scratch/c0223674fc50/cmake-3.31.8-linux-x86_64/bin \
  --output-dir /absolute/new/path/csr-release-certification \
  --jobs 4
```

The output path must not exist and must not be inside either repository or the
evidence input tree. On success, the root contains `release-manifest.json`,
`SHA256SUMS`, and `SHA256SUMS.sha256`. `artifacts/` is the frozen evidence set;
the ephemeral build/staging workspace defaults to the disjoint sibling
`OUTPUT_DIR.work` (or an explicit new `--work-dir`) and is not part of the
frozen deliverable. The harness, both Git source archives, every built binary,
every dynamically loaded shared library, build configuration, isolated run
logs, and results are copied into and hashed within the output directory.
The classifier probe is also frozen, executed from that frozen copy, logged,
and rehashed before sealing. Every directly invoked or pinned tool executable
is recorded by resolved path, SHA-256, and version and is rehashed after use.
Repository admission uses
a minimal recorded Git environment; build and test commands inherit no host
environment variables. A private command directory binds `cmake`, `make`,
`gcc`, `g++`, and both Python command names to the recorded executables. The
ns-3 wrapper, top-level RC Python utilities, and nested Python differential
stages are invoked with the one stored, hash-bound interpreter path in
isolated, no-bytecode mode under the minimal inherited environment.
Each top-level Python entry script is hashed immediately before and after its
command and recorded beside the pinned interpreter digest.
The protocol comparator is additionally copied into the frozen tool set and
its input aggregates, report, normalized CSV, and log are path-, size-, and
SHA-256-bound before the recursive checksum seal.

The executing harness and its sibling classifier probe must themselves resolve
to `utils/release/certify_release.py` and
`utils/release/test_certify_release_classifier.py` inside `--rc-repo`. Their
working-file Git blob IDs must exactly equal those paths at `--rc-commit`; the
binding is repeated at the final TOCTOU gate. An out-of-tree copy, symlink, or
modified classifier therefore cannot certify an otherwise clean release tree.

The canonical scenario explicitly selects the application, MAC, and
`hist-adb97c54-bare` HOP-security profiles bound to the archived multihop
executable. The canonical selector projects both ordinary DATA and ACK/DACK as
bare (ordinary DATA is 217 bytes); the deprecated ACK-only CSV alias must be
blank. The aggregate manifest must independently record the same two-part
behavior for both the canonical identity and the runner, with explicit profile
origin and the exact `adb97c54…` executable SHA-256. The canonical application
aggregate comparator may exit 0 for exact numeric parity or 1 for a measured
numeric residual. In both cases its exact eight statistics and all 800
identities must align, 780 numeric points must be compared, the 20
authoritative OPNET missing values must be the only skipped points,
delivery/size integrity must be exact, and every non-numeric failure count must
be zero. `Generator.Packet Size (bits)` is also an independent hard gate: all
95 authoritative values must match exactly, with only the five OPNET no-sample
buckets skipped.

A second zero-tolerance comparison gates the source-comparable protocol queue
surface over the 95 steady bucket ends from 360 through 6,000 seconds:
`HOP.Resend Queue Size (packets)`, `MAC.ACK Queue Size (packets)`,
`MAC.Tx Queue Size (packets)`, and `MAC.Tx Queuing Delay (sec)`. All 380
identities must be numeric and aligned with no missing, extra, skipped, or
noncomparable point. Its numeric residual is recorded, and the top-level
release status depends on both aggregate comparator exit codes. The two NWK
queue statistics remain ns-3-only diagnostics because the recovered OPNET
vector set contains no corresponding series. The startup bucket ends at 60,
120, 180, 240, and 300 seconds are intentionally outside this comparator; this
gate is a steady-state queue-surface check, not a claim of full-duration HOP or
MAC parity.

A successful harness exit means that the release evidence checkpoint passed
its structural, provenance, execution, and sealing gates. It does not relabel
an observed aggregate residual as numeric parity or certify whole-model parity;
the top-level manifest status must preserve both the authorized scoped
exemptions and any observed aggregate residual.

The Python suite count is discovered from the immutable staged RC tree in an
isolated interpreter, then the complete suite is run in a second interpreter.
The discovered and executed counts must be positive and exactly equal. Every
test source path and SHA-256 is retained in the manifest; there is no stale
hardcoded test count.

All 16 directly supplied OPNET source files are required by exact filename and
SHA-256, copied into the evidence set, rechecked at the end, and listed in a
dedicated scope manifest. `br_supervisor.pr.c` and `br_battery.pr.c` remain
hash-bound inputs but are explicit project-authorized exemptions. No
supervisory-layer, battery, or energy parity claim is made.
The hash-bound `adb97` executable contains those OPNET components; the
exemption means present but out of scope, not absent. The canonical zero-DSCP
application leaves its supervisor hook inert. Likewise, a fixed ns-3
`energy=100` value is not evidence of battery depletion or battery metrics.
These are the only project-authorized exemptions. The canonical routing target
is the supplied `__ARL_ROUTING__` configuration; alternative BBN routing is an
unimplemented parity gap/non-claim rather than an exemption.

`--supplied-source-dir` is also required and must contain exactly six entries,
including hidden names in the comparison: `CSR project examples.zip` plus the
five supplemental artifacts `more missing files.zip`, `routing.lib`,
`routes(1).c`, `Modulation_tables.zip`, and
`Project files for chatgpt(1).zip`. Every identity is fixed by SHA-256; the
canonical ZIP is cross-bound to `--source-archive`, and immutable copies of the
other five are staged, manifested, and rechecked after all execution. These
artifacts are provenance references, not a blanket assertion that every member
of an archive is implemented or in parity scope, and they do not close gaps for
historical source files that were cited but not separately supplied to this
certification (`arlSecurity.*`, `packetTypes.*`, `msectime.*`, and the split
`arlAuthTag`/`arlContext`/`arlEncrypt`/`arlKey` sources). The two explicit
battery/supervisor exemptions are unchanged.

The admission-enriched run is separate from the compact baseline. Its semantic
aggregate rows and application diagnostics must reproduce the compact run. An
admission analyzer exit 1 is accepted only when it has zero invalid legs, its
only global findings are duplicate deliveries, every duplicate key matches the
trace, and every extra delivery is covered by the immutable aggregate tool's
validated source-ordered lost-DACK lineage budget. Any other admission finding
is a hard release failure. The analyzer summary, full leg CSV, and application
diagnostics CSV are all path-, hash-, and row-count-bound. The diagnostics must
contain exactly the six canonical flows, partition every attempt, and reproduce
the summary totals with no reconciliation mismatch.

Packet-path report schema `csr-packet-path-analysis-v2` is accepted only when
the analyzer exits 0 in strict mode, every lifecycle row is valid, lifecycle
copy indexes are unique and contiguous per packet key, every extra lifecycle
equals a provenance proof budget, delivery multiplicity exactly matches the
trace, and every directed-HOP identity in each reconstructed path exists in and
exactly covers the tagged HOP-admission evidence in the trace. Distinct
delivered lifecycles must have distinct trace-grounded HOP lineages.
Valid undelivered terminal prefixes are the only allowed issue class. This is
an internal ns-3 packet-path integrity claim, not OPNET packet-event parity.

Legacy schema `csr-packet-path-analysis-v1` remains supported only through the
older exact, trace-bound repeated-DACK exception: its strict failure is retained
as a documented non-claim and any unexpected key, issue, terminal state, or
budget blocks release. A schema-v1 clean pass is also accepted only after every
CSV row and summary count is independently shown to be one valid, complete,
issue-free trace delivery. Schema v2 support is pinned to the current exact
directed-HOP, queue-delay-constrained, unique bipartite queue-copy matching
contract. It also requires the exact queue-delay statistic, evidence-adjacency
rule, numeric tolerance, and zero ambiguous repeated-DACK branch keys. The
earlier existence-only bipartite contract fails closed.
