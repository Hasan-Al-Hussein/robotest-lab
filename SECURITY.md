# Security policy

## Intended boundary

RoboTest Lab is a local, CPU-only engineering and portfolio project. It is not
an internet-facing robotics controller and must not be used to command physical
hardware without a separate safety analysis and supervised validation.

All supervisor HTTP endpoints must bind to `127.0.0.1`. No remote restart
endpoint is planned. Generated examples and tests must use synthetic data and
must not contain credentials, private maps, or personal rosbag content.

## Supported versions

Before the first stable release, security fixes target the current `main`
branch only. Release-specific support will be documented when versioned
releases exist.

## Reporting a vulnerability

After the public repository is created, report vulnerabilities through its
private GitHub Security Advisory interface. Until that interface exists, do
not publish exploit details or credentials in an issue, benchmark artifact,
bag, or log. Provide the affected revision, reproduction conditions, impact,
and the smallest safe supporting artifact.

## Project security requirements

- Keep services loopback-only unless a reviewed design explicitly changes the
  trust boundary.
- Use least-privilege GitHub Actions permissions and no repository secrets for
  ordinary builds.
- Treat mission YAML, supervisor configuration, process commands, and package
  scripts as untrusted input boundaries and validate them before use.
- Use bounded process restarts, timeouts, message queues, logs, and artifact
  retention.
- Never run project setup in `Ubuntu-20.04`; scripts must verify Noble before
  invoking privileged operations.
- Download the official ROS apt-source package over HTTPS and verify its pinned
  SHA-256 before installation.
- Do not claim physical safety from simulation, unit tests, or static analysis.

Third-party vulnerabilities are evaluated against the exact versions recorded
in Phase 0 evidence. Updating a dependency requires rerunning the affected
verification gates.
