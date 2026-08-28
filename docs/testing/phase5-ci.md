# Phase 5 CI gate

Phase 5 separates local CI equivalence from remote GitHub evidence. Neither
path replaces the live Phase 1/2 simulation checks, the Phase 3 positive
control or 15-run campaign, or the privileged Phase 4 `--apply` acceptance
run.

## Local and hosted checks

Run the local gate from an Ubuntu 24.04 / ROS 2 Jazzy environment:

```bash
scripts/verify_phase5.sh --local
```

The public workflow runs the same implementation with the explicit internal
CI mode:

```bash
scripts/verify_phase5.sh --ci
```

Before entering that gate, the hosted job refreshes the Jazzy rosdep cache and
installs only dependencies declared by packages under `src/`.

Both modes are limited to L0-L2 evidence:

- workflow policy, complete direct-dependency/license inventory, published
  claim-to-evidence bindings, Bash, ShellCheck, and Ruff checks;
- pure Python unit tests and Go race/vet tests;
- a fresh temporary colcon build and package-test pass with at most four
  workers; and
- installed CLI `--help` smoke checks that do not start a ROS graph.

The fresh build uses temporary build, install, and log bases, then removes
only that owned temporary directory. The evidence directory retains bounded
command logs and a summary that explicitly records that Gazebo, hardware,
systemd, Phase 3 campaign execution, and Phase 4 apply execution were not
started.

Every local or hosted run records the exact argument vector and starting
working directory, Git state, a first-party source snapshot, platform and tool
versions, a JSON summary, and a matching one-row CSV. Finalization writes an
exact `SHA256SUMS` over every run file except the manifest and its validation
sidecar, then persists strict `sha256sum -c` output in
`checksum-validation.txt`. Failed gates use the same finalization path. Local
retention operates only on resolved, timestamp-named direct children and keeps
the current run plus at most four prior runs.

The source snapshot covers the workflow and repository policies, root release
documents, all normative documentation/results, configuration, packaging,
scenarios, scripts, ROS packages, the supervisor, and tests. Generated Python
caches and `benchmarks/raw/` are excluded. The audited direct dependency
surface is recorded in `config/dependency-license-inventory.json`; it includes
the Phase 0 apt manifest, external `package.xml` dependencies, pinned Ruff and
GitHub Actions, and direct Go modules. It does not claim that resolver-added
operating-system dependencies are vendored. `config/release-claims.json`
binds each published quantitative README claim and evidence-bounded portfolio
bullet to checked-in result text.

## Workflow supply-chain policy

The workflow uses a standard `ubuntu-24.04` GitHub-hosted runner, grants only
`contents: read`, disables persisted checkout credentials, and accepts only
the reviewed full-length action commits below:

| Action | Release | Full commit |
| --- | --- | --- |
| `actions/checkout` | `v6.0.2` | `de0fac2e4500dabe0009e67214ff5f5447ce83dd` |
| `ros-tooling/setup-ros` | `0.7.19` | `649ef6bcd696da05bc27ceb3fab69d810c0daeab` |
| `actions/upload-artifact` | `v7.0.1` | `043fb46d1a93c77aae656e7c1c64a875d1fc6a0a` |

Review sources:

- <https://github.com/actions/checkout/releases/tag/v6.0.2> and
  <https://github.com/actions/checkout/commit/de0fac2e4500dabe0009e67214ff5f5447ce83dd>
- <https://github.com/ros-tooling/setup-ros/releases/tag/0.7.19> and
  <https://github.com/ros-tooling/setup-ros/commit/649ef6bcd696da05bc27ceb3fab69d810c0daeab>
- <https://github.com/actions/upload-artifact/releases/tag/v7.0.1> and
  <https://github.com/actions/upload-artifact/commit/043fb46d1a93c77aae656e7c1c64a875d1fc6a0a>
- <https://docs.github.com/en/code-security/tutorials/secure-your-organization/protect-against-threats>
- <https://github.com/ros-infrastructure/rosdep/blob/master/doc/overview.rst>

The Phase 5 workflow-contract test rejects movable action tags, unreviewed
actions, write permissions, non-standard runners, worker limits above four,
and commands that launch Gazebo, invoke Phase 1/2, authorize a Phase 3
campaign, apply Phase 4, mutate systemd/package lifecycle state, or publish
repository changes. A second fail-closed scan rejects package-test
registrations for launch tests, Gazebo/ROS launch commands, systemd, the Phase
3 runner, or privileged Phase 4 before `colcon test` is allowed to run.

## Documentation replay and portfolio media

P5-04 and P5-05 are evidence lanes of their own. A local or hosted Phase 5 CI
PASS does not close them.

### README command inventory and replay

Every README fence whose info string is exactly `bash` is one executable replay
unit. Candidate source keeps each such fence to
`cd /home/hasan/robotest-lab` plus one workflow command, with no placeholder,
comment, or interactive prompt. `config/phase5-readme-replay.json` binds each
unit's exact LF-normalized body and order to its expected exit, wall timeout,
log cap, authorization class, and exact tracked-state postcondition. Most units
must finish clean. The six Phase 0 units instead allow only their exact
matrix-declared generated subset below `artifacts/evidence/phase0/`:
`ros-apt-source.json`; the preinstall quartet `apt-planned-packages.txt`,
`apt-resolution.tsv`, `apt-simulation.txt`, and `phase0-preinstall.json`;
`install-transcript.txt`; and the refresh quartet `installed-packages.tsv`,
`phase0-versions.json`, `ros2-doctor.txt`, and `tool-versions.tsv`. Each unit
records the exact observed subset, sizes, and hashes. A blanket clean-end rule
must not erase or conceal those expected generated deltas.

Capture extracts those units from `git show <C>:README.md` and compares them
with the matrix. It validates the first-line `cd` as the frozen documented-root
assertion but does not execute that line: the second workflow line runs with
its working directory mapped to a detached clean checkout at exact candidate
`C`. Capture records both the documented and replay roots and does not inherit
the current workspace overlay. Read-only units need no additional authority.
Repository/dependency apply units remain inert unless the operator adds the
literal `--authorize-mutating-commands` option after separately authorizing
those system changes. Bare `verify_all.sh` is a successful documentation
replay only when it reaches its frozen exit `3`/`INCOMPLETE`; that outcome is
never converted into release PASS.

### Genuine chart and Mermaid renders

P5-05 deterministically selects the Phase 3 Scenario 5 trial at `runs/12`; the
operator cannot choose another passing trial. The validator independently
joins its `localization-error.png` to the exact Scenario 5 PASS result, ordered
candidate suite, recomputed aggregate, candidate SHA, run artifact manifest
and validation sidecar, dimensions, bytes, and SHA-256. The projected PNG must
be byte-identical to that accepted run artifact. It is a run-derived metric
chart, not a screenshot.

The checked-in Phase 1 RViz PNG is a genuine screenshot from a separate
bounded development run. It remains useful visual evidence for that scope, but
it cannot substitute for or be relabeled as the Scenario 5 chart.

README contains exactly two marked portfolio Mermaid fences: `architecture`
and `release-flow`. Stage 1 extracts their source from exact `C`, records each
fence SHA-256, and emits the exact render request. The operator renders those
two sources, and capture validates the supplied bounded SVG bytes while a human
inspects the same hashes. Stage 2 consumes the canonical review bound to those
exact bytes and may finalize only when both verdicts are PASS. A review cannot
predate the render, and any rerender requires a new review. The diagrams
explain the system; they are not runtime evidence.

The finalized CLI uses exact caller-selected candidate and Phase 3 roots,
derives the only permitted Scenario 5 result at
`<exact-phase3-candidate-root>/runs/12/result/run-result.json`, and never
discovers a latest run. Prepare the immutable attempt only after separately
authorizing the two documented Phase 0 mutations:

```bash
scripts/capture_phase5_portfolio.sh --documentation-replay \
  --repository . \
  --candidate-sha <candidate-sha> \
  --phase3-candidate-root <exact-phase3-candidate-root> \
  --attempt-id <YYYYMMDDTHHMMSSZ-serial> \
  --authorize-mutating-commands
```

`serial` is a positive decimal integer of at most ten digits. The complete
attempt ID must match `YYYYMMDDTHHMMSSZ-serial` exactly.

A successful prepare intentionally exits `3` with `status=REVIEW_REQUIRED`.
It preserves the exact Mermaid sources and `render-request.json` below that
attempt; it does not treat an unreviewed render as PASS. Render both sources to
new files outside the attempt. Then create one canonical LF-terminated review
JSON with exact top-level keys `attempt_id`, `candidate_git_sha`, `diagrams`,
`reviewed_utc`, `reviewer`, and `schema_version`. `diagrams` is ordered
`architecture`, `release-flow`; each item has exactly `diagram_id`,
`render_sha256`, `rendered_utc`, `renderer`, `source_sha256`, and
`verdict=PASS`. `renderer` has exactly `identity`, `mode`, `reference`, and
`version`; `mode` is `argv` with the exact bounded argument list or `url` with
the immutable GitHub `blob/<C>/README.md` URL. The source/render hashes must
match the request and supplied SVG bytes, and
`prepared_utc <= rendered_utc <= reviewed_utc`.

Finalize that one explicit attempt:

```bash
scripts/capture_phase5_portfolio.sh --finalize \
  --repository . \
  --candidate-sha <candidate-sha> \
  --phase3-candidate-root <exact-phase3-candidate-root> \
  --attempt-id <YYYYMMDDTHHMMSSZ-serial> \
  --architecture-svg <reviewed-architecture.svg> \
  --release-flow-svg <reviewed-release-flow.svg> \
  --visual-review <canonical-visual-review.json>
```

Mutating replay remains inert unless the operator supplies the explicit
authorization option. An attempt ID is never reused, even after failure.

Every capture creates a new immutable attempt child below
`artifacts/evidence/phase5/portfolio/<C>/`. A failure, rerender, or recapture
never overwrites an earlier child. Projection selects one exact finalized PASS
attempt, and adds exactly these six regular mode-`100644` files to evidence
commit `E`:

- `docs/results/phase-5/portfolio-<C>.json`;
- `docs/results/phase-5/portfolio-<C>.SHA256SUMS`;
- `docs/results/phase-5/portfolio-<C>.validation.txt`;
- `docs/results/phase-5/architecture-<C>.svg`;
- `docs/results/phase-5/release-flow-<C>.svg`; and
- `docs/results/phase-5/scenario5-localization-error-<C>.png`.

Projection and final validation compare the tracked files byte-for-byte with
that explicitly selected checksummed PASS attempt. Missing, extra, renamed,
rebound, mode-changed, source-inconsistent, or overwritten-attempt files fail
closed. Failed and superseded attempts stay immutable evidence.

## Canonical release order

Use this order exactly; later evidence-producing steps intentionally dirty the
worktree and must not precede any gate that requires a clean candidate:

1. Create and push the clean candidate commit C. Keep the checkout at C with no
   tracked, untracked, assume-unchanged, or skip-worktree source delta, and wait
   for C's hosted `RoboTest CI` workflow to complete successfully.
2. While C is still clean, run the separately authorized Phase 3 campaign and
   Phase 4 acceptance workflow. Retain their exact caller-selected local
   evidence roots; do not discover a `latest` result.
3. Capture P5-04/P5-05 from exact C while the primary checkout is still clean:
   validate and replay the README units in detached C worktrees, bind the
   deterministic Scenario 5 `runs/12` chart, render and review both Mermaid
   fences, and finalize one explicit immutable PASS attempt. Candidate CI and
   Phase 4 must already have completed before the attempt's `prepared_utc`.
4. Still starting from clean C, run bare `scripts/verify_all.sh`. Its expected
   exit is 3/`incomplete`; it records the exact allowed Phase 0 refresh delta
   and checksummed prior-local aggregate for the later evidence commit. The
   portfolio attempt must have finalized before this aggregate's `checked_at`.
5. Capture its candidate remote proof
   with `scripts/verify_phase5.sh --remote <candidate-sha>`, then generate the
   deterministic Phase 3/4 result documents and exact six-file portfolio
   projection from the evidence selected in steps 2 and 3. Do not rerun a
   clean-start live or bare gate after this point.
6. Review and create the exact evidence-only child commit E containing only the
   allowed Phase 0 delta, both proof/result surfaces, bounded README regions,
   claims extension, and six portfolio files described below. Push E and wait
   for E's own hosted workflow.
7. Capture E's successful workflow with the ignored
   `--remote-evidence-commit <evidence-commit-sha>` proof. Do not commit that
   second proof.
8. Run `scripts/verify_all.sh --release-evidence` with every exact path. This is
   the final read-only, zero-verifier release decision.

## Remote proof

After a commit is pushed to a public GitHub repository and its workflow is
complete, verify the exact lowercase 40-character SHA:

```bash
scripts/verify_phase5.sh --remote <sha>
```

Remote mode requires the native Ubuntu `gh` package declared in
`config/apt-packages.txt` and an authenticated per-user session (`gh auth
status`). It reads repository,
commit, and workflow-run metadata; it never creates a repository, pushes a
commit, dispatches a workflow, or reruns a job. It accepts only a completed
successful `RoboTest CI` run whose `headSha` exactly equals the requested SHA
and records the public repository and run URLs. The remote JSON receives its
own one-file `SHA256SUMS` sidecar and persisted strict checksum-validation
output. The compact three-file proof is intentionally written below
`docs/results/phase-5/`, where Git does not ignore it. Review those exact files
and create the separate evidence-only commit manually; remote mode never
commits or pushes. Raw local logs remain ignored below
`artifacts/evidence/phase5/`. Missing authentication, missing `origin`, a
private repository, a pending/failed run, or a SHA mismatch leaves remote
verification incomplete.

After the evidence-only commit has itself passed public CI, capture its exact
SHA without creating another tracked proof:

```bash
scripts/verify_phase5.sh --remote-evidence-commit <evidence-commit-sha>
```

This performs the same read-only GitHub checks but writes below ignored
`artifacts/evidence/phase5/remote-evidence-commit/`. It proves the second CI
run without creating an infinite proof/commit/CI chain.

`scripts/verify_all.sh` invokes Phase 5 only as `--local` and never injects the
Phase 3 campaign authorization or Phase 4 `--apply`. Even when all six routed
checks pass, bare mode exits 3 with `status=incomplete` and writes the
checksummed prior-local aggregate at
`artifacts/evidence/phase0/verify-all.json`. Phase 0 refreshes four tracked
provenance files, so the aggregate records the exact resulting subset, sizes,
and hashes. Start state must be clean, HEAD must remain unchanged, and every
end-state change must be an unstaged modification of one of those four files;
any source or configuration delta fails the gate.

After the separately authorized Phase 3 campaign and Phase 4 acceptance run,
generate the deterministic tracked result projection from the two exact
caller-selected canonical inputs:

```bash
python3 -B tests/phase5_release_docs.py \
  --repository . \
  --phase3-candidate-root <exact-phase3-candidate-root> \
  --phase3-aggregate <exact-phase3-candidate-root>/aggregate/aggregate-result.json \
  --phase4-scenario6 <exact-phase4-run-directory>/scenario6-result.json \
  --phase5-portfolio-root artifacts/evidence/phase5/portfolio/<candidate-sha> \
  --phase5-portfolio-raw-proof artifacts/evidence/phase5/portfolio/<candidate-sha>/attempts/<YYYYMMDDTHHMMSSZ-serial>/projection/portfolio-<candidate-sha>.json
```

The producer writes six Phase 3/4 result files:
`docs/results/phase-3/<candidate-id>.{json,csv,md}` and
`docs/results/phase-4/<run-id>.{json,csv,md}`. JSON and CSV are byte-identical
to the selected canonical inputs. Markdown is a bounded, deterministic render
with source hashes, sorted Phase 3 scenario/metric tables, and sorted Phase 4
target/measurement/check tables. The same call projects the exact six
portfolio files from the explicitly selected raw PASS attempt, replaces only
the two uniquely delimited README release regions, and appends the exact
three-record final extension to `config/release-claims.json`.

Create evidence commit E containing the eight non-portfolio documentation
paths (the six Phase 3/4 result files, README, and release claims), the six
portfolio projection paths, the exact Phase 0 refresh subset recorded by the
prior-local aggregate, that aggregate's three-file proof, and the candidate
remote-proof trio. No other path is allowed, and every changed path must be a
regular mode-`100644` blob. After E passes CI and its raw second proof is
generated, run the evidence-only final gate with every explicit path:

```bash
scripts/verify_all.sh --release-evidence \
  --local-aggregate <exact-prior-verify-all.json> \
  --phase3-candidate-root <exact-phase3-candidate-root> \
  --phase3-aggregate <exact-phase3-candidate-root>/aggregate/aggregate-result.json \
  --phase4-run-directory <exact-phase4-run-directory> \
  --phase4-scenario6 <exact-phase4-run-directory>/scenario6-result.json \
  --phase5-portfolio-root artifacts/evidence/phase5/portfolio/<candidate-sha> \
  --phase5-portfolio-proof docs/results/phase-5/portfolio-<candidate-sha>.json \
  --phase5-remote-proof docs/results/phase-5/remote-<candidate-sha>.json \
  --phase5-evidence-commit-remote-proof artifacts/evidence/phase5/remote-evidence-commit/remote-<evidence-commit-sha>.json
```

This mode invokes no phase verifier and performs no campaign, privileged,
network, publication, or latest-evidence operation. It revalidates the prior
local summary and checksums, the complete ordered Phase 3 suite and recomputed
aggregate, the exact Phase 4 check/hash/CSV surface, the tracked candidate
proof, and the ignored exact-SHA proof for E's own successful CI. All candidate
lanes must bind the same clean candidate SHA. The checked-out HEAD must be its
single-parent evidence-only child. Its changed paths and modes must exactly
equal the two trackable trios, the recorded Phase 0 refresh subset, the six
result documents, README, the release-claims extension, and the six portfolio
files. README, replay inventory, and claims are reconstructed from
`git show <candidate>:...`; changes outside the two bounded README regions or
exact claim extension fail. The explicitly selected immutable raw portfolio
PASS attempt, deterministic Scenario 5 `runs/12` lineage, and tracked
media/validation bytes are independently rejoined. The final result is raw
local proof written separately below
`artifacts/evidence/phase5/release/`; the supplied local aggregate is never
overwritten and no `latest` directory is discovered.

The Phase 3 and Phase 4 artifacts are local, self-hashed evidence. Their
schemas, identities, complete manifests, and producer invariants detect
accidental or partial tampering, but the files are not signed and do not resist
an adversary who can rewrite the complete evidence set and every checksum.
Likewise, the remote proof JSON is an exact producer-shaped record with
self-checksums, not a signed GitHub attestation; the final gate assumes an
honest operator captured it through the documented authenticated `gh` command.

Local PASS evidence is therefore not a claim of public CI, simulation,
hardware, packaging lifecycle, or release completion.
