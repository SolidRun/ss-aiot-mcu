# syntax=docker/dockerfile:1

# STM32CubeIDE, for headless builds of this project.
#
# ST offers no anonymous download of the IDE, so the installer is not fetched
# here. Place the .sh.zip ST publishes next to this file before building.
#
# Bump CUBEIDE_VERSION_FULL on an upgrade. It must match the installer filename
# "st-stm32cubeide_${CUBEIDE_VERSION_FULL}.sh.zip".
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

# unzip and git for the installer and the .cproject pre-build step; the
# libraries are what the IDE launcher links against, headless or not.
# For sbom generation: bear, jq. For packaging build artifacts: zip.
RUN set -e; \
        apt-get update; \
        apt-get install -y --no-install-recommends bear ca-certificates git jq libusb-1.0-0 libgtk-3-0 libncurses5 libwebkit2gtk-4.0-37 locales unzip zip; \
        rm -rf /var/lib/apt/lists/*; \
        :

# The installer is read from the bind-mounted build context rather than
# COPYed, so its 1.3 GB never becomes a layer of the image.
# Unpacked installer script is deleted after use to avid inclusion in layer.
ARG CUBEIDE_VERSION_FULL=1.19.0_25607_20250703_0907_amd64
RUN --mount=type=bind,target=/ctx set -e -x; \
    unzip -d /tmp -j /ctx/st-stm32cubeide_${CUBEIDE_VERSION_FULL}.sh.zip st-stm32cubeide_${CUBEIDE_VERSION_FULL}.sh; \
    mkdir -p /opt/stm32cubeide; \
    sh /tmp/st-stm32cubeide_${CUBEIDE_VERSION_FULL}.sh --tar -xOf ./st-stm32cubeide_${CUBEIDE_VERSION_FULL}.tar.gz | tar -C /opt/stm32cubeide -xzvf -; \
    rm -f /tmp/st-stm32cubeide_${CUBEIDE_VERSION_FULL}.sh; \
    test -x /opt/stm32cubeide/stm32cubeide; \
    :

RUN locale-gen en_US.UTF-8
RUN update-ca-certificates

# Set environment variables
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    PATH="/opt/stm32cubeide:${PATH}"

# Arguments to pass host user's UID and GID
ARG USER_ID=1000
ARG GROUP_ID=1000

# Ensure there is a user with the same UID/GID as the host user, create 'developer' when missing
RUN getent group ${GROUP_ID} || groupadd -g ${GROUP_ID} developer
RUN getent passwd ${USER_ID} || useradd -m -u ${USER_ID} -g ${GROUP_ID} -d /home/developer developer

# Create and set the working directory
WORKDIR /work
RUN chown -R ${USER_ID}:${GROUP_ID} /work

# drop privileges, numerically so it holds whichever account owns the id
USER ${USER_ID}:${GROUP_ID}

# The entry point is set to bash. This means that when the container starts, it will drop the user into bash
ENTRYPOINT ["/bin/bash"]
