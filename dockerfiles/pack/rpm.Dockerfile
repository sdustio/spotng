FROM redhat/ubi9

RUN yum install -y cmake rpm-build gcc-c++ git

ENV VCPKG_ROOT=/opt/vcpkg
ENV PATH="${PATH}:${VCPKG_ROOT}"

COPY vcpkg-install.sh /tmp/
RUN /tmp/vcpkg-install.sh && rm -f /tmp/vcpkg-install.sh

ENV GH_VERSION=2.12.1
RUN curl -LO https://github.com/cli/cli/releases/download/v${GH_VERSION}/gh_${GH_VERSION}_linux_amd64.rpm \
    && rpm -i gh_${GH_VERSION}_linux_amd64.rpm && rm gh_${GH_VERSION}_linux_amd64.rpm

RUN vcpkg install eigen3 spdlog
