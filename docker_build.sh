#!/bin/bash
# Build the moteus firmware inside an x86_64 Ubuntu 22.04 container.
# The repo is mounted at /moteus.
set -ex

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq build-essential git curl ca-certificates \
    python3 file unzip zip libtinfo5 > /dev/null

cd /moteus
git config --global --add safe.directory /moteus

echo "=== STEP 1: host unit tests (includes SafeSpi frame tests) ==="
tools/bazel test --config=host //fw:test

echo "=== STEP 2: firmware target build ==="
tools/bazel build --config=target //:target

echo "=== STEP 3: copy artifacts to the repo (visible on the host) ==="
GITREV=$(git rev-parse --short HEAD)
mkdir -p /moteus/build-output
cp -L bazel-out/stm32g4-opt/bin/fw/moteus.elf \
      "/moteus/build-output/moteus-fw-safespi-g${GITREV}.elf"
cp -L bazel-out/stm32g4-opt/bin/fw/can_bootloader.elf \
      "/moteus/build-output/moteus-bl-safespi-g${GITREV}.elf"
ls -la /moteus/build-output/
file "/moteus/build-output/moteus-fw-safespi-g${GITREV}.elf"

echo "BUILD_COMPLETE"
