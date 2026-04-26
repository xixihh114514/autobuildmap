#!/usr/bin/env bash

set -euo pipefail

launch_args=()
for arg in "$@"; do
  case "$arg" in
    __name:=*|__log:=*|__ns:=*|__master:=*|__ip:=*|__hostname:=*)
      ;;
    *)
      launch_args+=("$arg")
      ;;
  esac
done

if [ "${#launch_args[@]}" -lt 2 ]; then
  echo "Usage: delayed_roslaunch.sh <package> <launch_file> [name:=value ...]" >&2
  exit 64
fi

exec roslaunch "${launch_args[@]}"
