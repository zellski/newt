#!/usr/bin/env bash
#
# get-hqp.sh -- fetch and build NewT's solver dependencies at known-good
# revisions, installed into a single prefix, for a reproducible local build.
# The repo has no dependency pinning otherwise; this is the one place the
# versions are written down.
#
# Pinned revisions:
#   ADOL-C   coin-or/ADOL-C   releases/2.7.2   (commit 0e8f8a6)
#   HQP      omuses/hqp       703dbc0          (version string 1.9.9.26.03.30)
#
# Tuned for Debian/Ubuntu (tested on ubuntu:24.04 / gcc-13). Install the
# prerequisites first:
#
#   apt-get install -y build-essential g++ gfortran make autoconf automake \
#       libtool pkg-config tcl8.6-dev libsqlite3-dev libopenblas-dev git
#
# Usage:
#   scripts/get-hqp.sh [--prefix DIR] [--build DIR] [--without-adolc] [--jobs N]
#
# Everything is overridable by environment variable (see the defaults below).
# The seed builds against ADOL-C, so ADOL-C is built by default; pass
# --without-adolc (or ADOLC=0) to build an ADOL-C-free HQP instead -- the
# resulting libomu/libhqp link no ADOL-C at all.
#
set -euo pipefail

# --- pins (override via env) ------------------------------------------------
ADOLC_REPO=${ADOLC_REPO:-https://github.com/coin-or/ADOL-C.git}
ADOLC_REF=${ADOLC_REF:-releases/2.7.2}            # commit 0e8f8a6
HQP_REPO=${HQP_REPO:-https://github.com/omuses/hqp.git}
HQP_REF=${HQP_REF:-703dbc0}                       # master @ 1.9.9.26.03.30

# --- knobs (override via env or flags) --------------------------------------
PREFIX=${PREFIX:-$HOME/newtdeps}
BUILD_DIR=${BUILD_DIR:-$PWD/.deps-build}
ADOLC=${ADOLC:-1}            # 1: --enable-adolc, 0: --disable-adolc
JOBS=${JOBS:-1}             # HQP's recursive make is not parallel-safe

while [ $# -gt 0 ]; do
  case "$1" in
    --prefix) PREFIX=$2; shift 2 ;;
    --build)  BUILD_DIR=$2; shift 2 ;;
    --jobs)   JOBS=$2; shift 2 ;;
    --without-adolc) ADOLC=0; shift ;;
    --with-adolc)    ADOLC=1; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# Tcl: Debian/Ubuntu multiarch layout. --with-tcl points at the directory
# holding tclConfig.sh and is required (not just --with-tclinclude).
TRIPLET=$(gcc -dumpmachine)
TCL_CONFIG_DIR=${TCL_CONFIG_DIR:-/usr/lib/$TRIPLET/tcl8.6}
TCL_INCLUDE_DIR=${TCL_INCLUDE_DIR:-/usr/include/tcl8.6}
SYS_LIBDIR=${SYS_LIBDIR:-/usr/lib/$TRIPLET}

PREFIX=$(mkdir -p "$PREFIX" && cd "$PREFIX" && pwd)   # absolutize
mkdir -p "$BUILD_DIR"
BUILD_DIR=$(cd "$BUILD_DIR" && pwd)

say() { printf '\n=== %s ===\n' "$*"; }

require() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing tool: $1 -- see the apt-get line at the top of this script" >&2
    exit 1
  }
}
for t in git gcc g++ make gfortran autoreconf; do require "$t"; done

# clone (once) and pin to the recorded ref
fetch() {
  repo=$1; ref=$2; dir=$3
  if [ ! -d "$dir/.git" ]; then
    say "cloning $repo"
    git clone "$repo" "$dir"
  fi
  say "checking out $ref in $(basename "$dir")"
  git -C "$dir" checkout -q "$ref"
}

build_adolc() {
  src=$BUILD_DIR/ADOL-C
  fetch "$ADOLC_REPO" "$ADOLC_REF" "$src"
  say "building ADOL-C (lands in $PREFIX/lib64)"
  ( cd "$src"
    autoreconf -fi
    ./configure --prefix="$PREFIX" --disable-sparse --enable-static --enable-shared
    make -j"$JOBS"
    make install )
}

build_hqp() {
  src=$BUILD_DIR/hqp
  fetch "$HQP_REPO" "$HQP_REF" "$src"
  if [ "$ADOLC" = 1 ]; then
    adolc_flags="--enable-adolc --with-adolc=$PREFIX"
    say "building HQP with ADOL-C"
  else
    adolc_flags="--disable-adolc"
    say "building HQP without ADOL-C"
  fi
  # serial make: HQP's recursive build is not parallel-safe
  ( cd "$src"
    ./configure $adolc_flags --enable-fortran --prefix="$PREFIX" \
      --with-tcl="$TCL_CONFIG_DIR" --with-tclinclude="$TCL_INCLUDE_DIR" \
      LDFLAGS=-L"$SYS_LIBDIR"
    make
    make install )
}

[ "$ADOLC" = 1 ] && build_adolc
build_hqp

say "done -- dependencies installed under $PREFIX"
if [ "$ADOLC" = 1 ]; then
  cat <<EOF
Build NewT against this prefix:

  make -C src \\
    ADOLC_INC=$PREFIX/include ADOLC_LIB=$PREFIX/lib64 \\
    HQP_INC=$PREFIX/include/hqp HQP_LIB=$PREFIX/lib
  make -C demo \\
    ADOLC_INC=$PREFIX/include ADOLC_LIB=$PREFIX/lib64 \\
    HQP_INC=$PREFIX/include/hqp HQP_LIB=$PREFIX/lib \\
    TCL_INC=$TCL_INCLUDE_DIR
EOF
else
  cat <<EOF
This built an ADOL-C-free HQP/Omuses under $PREFIX.

NewT itself currently needs ADOL-C: src/Makefile and demo/Makefile force
-DOMU_WITH_ADOLC=1 and -ladolc, so they will not link against this prefix
as-is. Re-run without --without-adolc to build NewT (the default), or edit
those Makefiles first if you are deliberately building an ADOL-C-free NewT.
EOF
fi
