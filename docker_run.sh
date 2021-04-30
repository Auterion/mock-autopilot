#!/usr/bin/env sh
#
# Copyright (c) 2021 Auterion AG
#

set -eu

DEF_IMAGE="auterion/mock-autopilot:latest"

usage()
{
	echo "Usage: ${0} [OPTIONS] command arg1 arg2.."
	echo "Run local command in container environment"
	echo "    -i  container image to be used (default: '${DEF_IMAGE}')"
	echo "    -f  custom container flags"
	echo
	echo "Example:"
	echo "    ./docker_run.sh ./build.sh -h"
}

prepare_image()
{
	_image="${1:?Missing image name}"
	docker build \
		--cache-from="${_image}" \
		--rm \
		--tag "${_image}" \
		"."
}

docker_run()
{
	image="${1:?Missing image name}"
	cmd="${2:?Missing arguments}"
	flags="${3:-}"
# shellcheck disable=SC2086
	docker run \
		--rm \
		--volume "${PWD}:/workdir" \
		--workdir="/workdir" \
		${flags} \
		"${image}" \
		/bin/bash -c "${cmd}"
}

main()
{
	while getopts ":hb:i:f:" _options; do
		case "${_options}" in
		h)
			usage
			exit 0
			;;
		i)
			_image="${OPTARG}"
			;;
		f)
			_flags="${OPTARG}"
			;;
		:)
			e_err "Option -${OPTARG} requires an argument"
			exit 1
			;;
		?)
			e_err "Invalid option: -${OPTARG}"
			exit 1
			;;
		esac
	done
	shift "$((OPTIND - 1))"

	# Options can be set in 3 ways:
	# command line argument - highest priority
	# global variable
	# default value
	image="${_image:-${IMAGE:-${DEF_IMAGE}}}"
	flags="${_flags:-}"
	cmd="${*}"

	prepare_image "${image}"
	docker_run "${image}" "${cmd}" "${flags}"
}

main "${@}"
