################################################################################
# Custom Extension
################################################################################

ARG BASE_IMAGE=focal

################################################################################
# Base Image
################################################################################

FROM mcr.microsoft.com/devcontainers/base:${BASE_IMAGE}

################################################################################
# Configuration
################################################################################

ARG BAZEL_VERSION=6.4.0
ARG RUST_VERSION=1.75.0

################################################################################
# User 'zen'
################################################################################

ARG USERNAME=zen

# Automatically creates a UID, group ${USERNAME}, adds ${USERNAME} to the group
#
# NB: If using this image as a base for a devcontainer, it is critical to have
# just one user, since dynamic UID mapping will not work otherwise, refer to:
#   https://github.com/microsoft/vscode-remote-release/issues/1155
#
RUN useradd -o --uid 1000 -s "/bin/bash" -m ${USERNAME} && \
    apt-get install -y sudo && \
    echo "${USERNAME} ALL=NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

COPY rust-zen/zen.bashrc /home/${USERNAME}/.bashrc.d/zen.bashrc
RUN echo "source /home/${USERNAME}/.bashrc.d/zen.bashrc" >> /home/${USERNAME}/.bashrc

RUN mkdir -p /home/${USERNAME}/.local/share/bash-completion/completions/ && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.local

# Debugging with root.

COPY rust-zen/zen.bashrc ${HOME}/.bashrc.d/zen.bashrc
RUN echo "source ${HOME}/.bashrc.d/zen.bashrc" >> ${HOME}/.bashrc

################################################################################
# Unset gcr configured ubuntu user
################################################################################

USER root

################################################################################
# Tools & Utilities
################################################################################

RUN apt-get update && apt-get install -y --no-install-recommends \
    # development
    curl \
    bash \
    bash-completion \
    g++ \
    gcc \
    git \
    graphviz \
    make \
    pkg-config \
    ssh \
    wget \
    vim \
    less \
    zip \
    # python
    python3-dev \
    python3-pip \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

RUN pip install \
  pre-commit==2.20.0

################################################################################
# Bazelisk
################################################################################

ARG BAZELISK_VERSION=v1.10.1
ARG BAZELISK_DOWNLOAD_SHA=dev-mode
RUN curl -fSsL -o /usr/local/bin/bazelisk https://github.com/bazelbuild/bazelisk/releases/download/${BAZELISK_VERSION}/bazelisk-linux-amd64  && \
    ([ "${BAZELISK_DOWNLOAD_SHA}" = "dev-mode" ] || echo "${BAZELISK_DOWNLOAD_SHA} */usr/local/bin/bazelisk" | sha256sum --check - ) && \
    chmod 0755 /usr/local/bin/bazelisk && \
    # The only way to get root (CI), vscode (codespaces) & zen
    # (local devcontainer) on the same page without a
    # three-way .bazelrc config
    sudo ln -s /usr/local/bin/bazelisk /usr/local/bin/bazel

ENV USE_BAZEL_VERSION "${BAZEL_VERSION}"

################################################################################
# Installs Rust toolchain for specified rust version (RUST_VERSION).
# Installs Rust autocompletion for bash.
################################################################################

ENV RUSTUP_HOME="/opt/rustup"
ENV CARGO_HOME="/opt/cargo"
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y  --default-toolchain ${RUST_VERSION} \
    && . "${CARGO_HOME}/env" \
    && rustup component add clippy \
    && rustup component add rustfmt \
    && cargo install cargo-deb \
    && cargo install cargo-expand \
    && cargo install cargo-rdme \
    && rustup target add x86_64-unknown-linux-gnu \
    && rustup completions bash cargo >> /home/${USERNAME}/.local/share/bash-completion/completions/cargo \
    && echo "export RUSTUP_HOME=${RUSTUP_HOME}" >> /etc/profile \
    && echo "export PATH=${CARGO_HOME}/bin:\${PATH}" >> /etc/profile
ENV PATH="$CARGO_HOME/bin/:$PATH"

# The previous command roots permissions in ${CARGO_HOME} and ${RUSTUP_HOME}.
# Establish wide permissions for user 'zen'. This step takes several minutes.

RUN find ${CARGO_HOME} -type d -exec chmod 777 {} + && \
    find ${CARGO_HOME} -type f -exec chmod a+rw {} + && \
    find ${RUSTUP_HOME} -type d -exec chmod 777 {} + && \
    find ${RUSTUP_HOME} -type f -exec chmod a+rw {} +

################################################################################
# Login Shell
################################################################################

ENV TERM xterm-256color
ENTRYPOINT ["/bin/bash", "--login", "-i"]

ENV BLESSING="May you be blessed by a tickle from his noodly appendages..."
