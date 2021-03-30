#!/usr/bin/env sh
#
# Copyright (c) 2021 Auterion AG
#

set -eu

DEF_BUILDDIR="build"

err()
{
	>&2 echo "ERROR: ${*}"
}

usage()
{
	echo "Usage: ${0} [OPTIONS]"
	echo "Helper script that calls CMake and make"
	echo "    -b  build directory (default: '${DEF_BUILDDIR}')"
	echo
}

main()
{
	while getopts ":hb:" _options; do
		case "${_options}" in
		h)
			usage
			exit 0
			;;
		b)
			_build="${OPTARG}"
			;;
		:)
			err "Option -${OPTARG} requires an argument"
			exit 1
			;;
		?)
			err "Invalid option: -${OPTARG}"
			exit 1
			;;
		esac
	done
	shift "$((OPTIND - 1))"

	builddir="${_build:-${DEF_BUILDDIR}}"

	cmake -B"${builddir}" -S"."
	make -j$(nproc --all) -C"${builddir}"
}

main "${@}"
