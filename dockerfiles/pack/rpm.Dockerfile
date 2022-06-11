FROM redhat/ubi9

RUN yum install -y cmake rpm-build gcc-c++ git

ENV VCPKG_ROOT=/opt/vcpkg
ENV PATH="${PATH}:${VCPKG_ROOT}"

COPY vcpkg-install.sh /tmp/
RUN /tmp/vcpkg-install.sh && rm -f /tmp/vcpkg-install.sh
