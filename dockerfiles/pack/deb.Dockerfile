FROM debian

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install build-essential tar curl zip unzip cmake git pkg-config \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

ENV VCPKG_ROOT=/opt/vcpkg
ENV PATH="${PATH}:${VCPKG_ROOT}"

COPY vcpkg-install.sh /tmp/
RUN /tmp/vcpkg-install.sh && rm -f /tmp/vcpkg-install.sh

ENV GH_VERSION=2.12.1
RUN curl -LO https://github.com/cli/cli/releases/download/v${GH_VERSION}/gh_${GH_VERSION}_linux_amd64.deb \
    && dpkg -i gh_${GH_VERSION}_linux_amd64.deb && rm gh_${GH_VERSION}_linux_amd64.deb

RUN vcpkg install eigen3 spdlog
