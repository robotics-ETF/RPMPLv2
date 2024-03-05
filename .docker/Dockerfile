FROM ubuntu:22.04 as rpmplv2-dev-container

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    curl \
    wget \
    gnupg \
    make \
    cmake \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libeigen3-dev libkdl-parser-dev libgflags-dev libgoogle-glog-dev liborocos-kdl-dev \
    libyaml-cpp-dev liburdf-dev python3-pip libfcl-dev libnanoflann-dev \
    && pip3 install trimesh urdfpy \
    && rm -rf /var/lib/apt/lists/*

ENV CLANG_VERSION=18
RUN wget https://apt.llvm.org/llvm.sh && \
    sed -i "s/add-apt-repository \"${REPO_NAME}\"/add-apt-repository \"${REPO_NAME}\" -y/g" llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh $CLANG_VERSION -y && \
    apt-get install -y --no-install-recommends clang-$CLANG_VERSION clang-tidy-$CLANG_VERSION clang-format-$CLANG_VERSION \
    llvm-$CLANG_VERSION-dev libc++-$CLANG_VERSION-dev libomp-$CLANG_VERSION-dev libc++abi-$CLANG_VERSION-dev libunwind-$CLANG_VERSION-dev && \
    rm -rf /var/lib/apt/lists/*

RUN  update-alternatives --install /usr/bin/clang clang /usr/bin/clang-$CLANG_VERSION $CLANG_VERSION \
    --slave /usr/bin/clang++ clang++ /usr/bin/clang++-$CLANG_VERSION \
    --slave /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-$CLANG_VERSION \
    --slave /usr/bin/clang-format clang-format /usr/bin/clang-format-$CLANG_VERSION

RUN ln -s /usr/bin/clangd-${CLANG_VERSION} /usr/bin/clangd

ARG USERNAME=roboticsETF
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
