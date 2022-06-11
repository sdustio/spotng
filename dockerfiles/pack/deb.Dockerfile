FROM debian

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install build-essential tar curl zip unzip cmake git \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

ENV VCPKG_ROOT=/opt/vcpkg
ENV PATH="${PATH}:${VCPKG_ROOT}"

COPY vcpkg-install.sh /tmp/
RUN /tmp/vcpkg-install.sh && rm -f /tmp/vcpkg-install.sh
