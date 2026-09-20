#!/usr/bin/env bash
# Run the logic tests on the host.
#
# .cargo/config.toml pins the build to riscv32imc-esp-espidf, and the lib's
# esp-idf dependencies can't build for the host -- so tests run against a
# throwaway host-only manifest containing just the pure-logic modules. Those
# modules import nothing from esp-idf by design, which is what makes this work.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

mkdir -p "$work/src"
cp "$here/src/lib.rs" "$here/src/machine.rs" "$here/src/parse.rs" "$here/src/tdoa.rs" "$work/src/"

cat > "$work/Cargo.toml" <<'EOF'
[package]
name = "cyclevroom"
version = "0.1.0"
edition = "2021"

[lib]
path = "src/lib.rs"
EOF

cd "$work"
cargo test "$@"
