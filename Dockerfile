#
# Copyright (C) 2021 Auterion AG
#

FROM "ubuntu:20.04"

LABEL maintainer="Artur Mądrzak <artur@auterion.com>"

WORKDIR /workdir

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install mavsdk dependency.
# hadolint ignore=DL3020
# ADD "https://github.com/mavlink/MAVSDK/releases/download/v0.39.0/mavsdk_0.39.0_ubuntu20.04_amd64.deb" "."
COPY mavsdk_0.39.0_ubuntu20.04_amd64.deb .
RUN dpkg -i mavsdk_0.39.0_ubuntu20.04_amd64.deb \
    && rm mavsdk_0.39.0_ubuntu20.04_amd64.deb

RUN ldconfig

# Install requirements to build image
# hadolint ignore=DL3008,DL3015
RUN apt-get --yes --quiet update \
    && DEBIAN_FRONTEND=noninteractive apt-get --yes --quiet --fix-broken install \
        build-essential \
        cmake \
        debhelper \
        pkg-config \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
