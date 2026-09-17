# syntax=docker/dockerfile:1

# STM32CubeIDE, for headless builds of this project.
#
# ST offers no anonymous download of the IDE, so nothing is fetched here. Place
# the archives ST publishes next to this file before building:
#
#   st-stm32cubeide_${CUBEIDE_VERSION_FULL}.sh.zip  the IDE installer
#   ${CUBE_REPO_PKGS}                               the STM32Cube code generation packages
#
# Both arguments are filenames, and are bumped on an upgrade. CUBE_REPO_PKGS
# is a space separated list, so a build can be given more than one package.
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

# unzip and git for the installer and the .cproject pre-build step; the
# libraries are what the IDE launcher links against, headless or not.
# For sbom generation: bear, jq. For packaging build artifacts: zip.
# For STM32CubeMX: imagemagick, libXtst, xvfb.
RUN set -e; \
        apt-get update; \
        apt-get install -y --no-install-recommends bear ca-certificates git imagemagick jq libusb-1.0-0 libgtk-3-0 libncurses5 libwebkit2gtk-4.0-37 libxtst6 locales unzip xvfb zip; \
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

# STM32CubeMX reads the STM32Cube firmware package from a repository below the
# JVM's user.home, and asks for my.st.com credentials for authenticated
# download. Provide the package in a root-owned repository at /opt/cubemx-repo.
# The archives are read from the bind-mounted build context rather than
# COPYed, so they never become a layer of the image.
# Additional packages / versions can be added here as needed.
ARG CUBE_REPO_PKGS=stm32cube_fw_u0_v130.zip
RUN --mount=type=bind,target=/ctx set -e; \
    for pkg in ${CUBE_REPO_PKGS}; do \
        unzip -q -d /opt/cubemx-repo /ctx/$pkg; \
    done; \
    :

# runs a STM32CubeMX command script against a project, see its header
COPY headless-cubemx.sh /usr/local/bin/headless-cubemx.sh
RUN chmod 0755 /usr/local/bin/headless-cubemx.sh

RUN update-ca-certificates

# Set environment variables
ENV LANG=C.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=C.UTF-8 \
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
