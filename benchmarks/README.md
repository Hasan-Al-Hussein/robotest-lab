# RoboTest Lab benchmark artifacts

This directory is the tracked publication surface for small, reviewable
benchmark derivatives. It is not the live candidate workspace, and it does
not currently claim an authoritative Phase 3 benchmark result.

## Canonical candidate evidence

The staged Phase 3 runner accepts only this ignored output root:

```text
artifacts/evidence/phase3-benchmarks/<candidate-id>/
```

One candidate directory contains the immutable suite plan and build binding,
the collision positive control, the single-scenario smoke, exactly 15 ordered
cold-stack trial directories, and the aggregate result. Canonical per-trial
verdicts are the `runs/<suite-index>/result/run-result.json` files; the final
suite verdict is `aggregate/aggregate-result.json`. Their CSV files, manifests,
SHA-256 sidecars, component evidence, process records, and bounded logs remain
with the candidate. The whole root is ignored by Git because it can contain
machine-specific and comparatively large runtime evidence.

## Raw outputs

Raw observations include ROS/Gazebo component artifacts, traces, bounded logs,
resource samples, and optional bounded bags. They stay with their canonical
candidate under `artifacts/evidence/phase3-benchmarks/`. The ignored
`benchmarks/raw/` path is available only for a deliberate export or working
copy; content placed there is not canonical evidence and must retain its source
candidate ID, relative path, size, and SHA-256.

Raw output is bounded by the capacities and byte limits in the
[metrics contract](../docs/architecture/metrics-contract.md). A truncated,
overflowed, missing, or hash-mismatched source fails its affected result rather
than being repaired during reporting.

## Derived outputs

Small canonical JSON/CSV results, generated charts, and generated summary
reports may be promoted here after the candidate is complete. Every derivative
must be generated exclusively from canonical `run-result.json` or
`aggregate-result.json` data, retain source candidate/run IDs and hashes, and
remain within the same documented caps. Hand-entered benchmark values are not
accepted.

Human-facing reports may also be published under `docs/results/`. Large bags,
logs, and other raw captures remain ignored. Promotion copies evidence for
review; it never changes which files own the runtime verdict.
