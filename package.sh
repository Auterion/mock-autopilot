#!/usr/bin/env sh
#
# Copyright (c) 2021 Auterion AG
#

set -eu

DEF_OUTPUT="output"

err()
{
	>&2 echo "ERROR: ${*}"
}

# note:
# container build requires 'output' directory to be defined within
# working directory. Artifacts won't be accessible otherwise.
usage()
{
	echo "Usage: ${0} [OPTIONS]"
	echo "Helper script that generate Debian package"
	echo "    -o  output directory (defaut: '${DEF_OUTPUT}')"
	echo
}

main()
{
	while getopts ":ho:" _options; do
		case "${_options}" in
		h)
			usage
			exit 0
			;;
		o)
			_output="${OPTARG}"
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

	output="${_output:-${DEF_OUTPUT}}"

	dpkg-buildpackage -us -uc -nc -b
	mkdir -p "${output}"
	mv ../mock-autopilot* "${output}"
}

main "${@}"
