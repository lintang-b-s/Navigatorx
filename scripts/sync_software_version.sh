#!/usr/bin/env bash
set -euo pipefail

repo_root=$(git rev-parse --show-toplevel)
version_file="$repo_root/pkg/util/binary_io.go"
tag=$(
  git tag --list 'v[0-9]*.[0-9]*.[0-9]*' --sort=-version:refname |
    grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' |
    head -n 1 || true
)

if [[ -z "$tag" ]]; then
  echo "no semantic version tag matching vMAJOR.MINOR.PATCH was found" >&2
  exit 1
fi

IFS=. read -r major minor patch <<<"${tag#v}"

sed -Ei \
  -e "s/(SoftwareVersionMajor[[:space:]]+uint16[[:space:]]*=[[:space:]]*)[0-9]+/\1$major/" \
  -e "s/(SoftwareVersionMinor[[:space:]]+uint16[[:space:]]*=[[:space:]]*)[0-9]+/\1$minor/" \
  -e "s/(SoftwareVersionPatch[[:space:]]+uint16[[:space:]]*=[[:space:]]*)[0-9]+/\1$patch/" \
  "$version_file"

gofmt -w "$version_file"
