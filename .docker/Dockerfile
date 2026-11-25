FROM ubuntu:22.04 AS rpmplv2-base

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
    cmake ccache \
    lsb-release \
    software-properties-common \
    unzip \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libeigen3-dev libkdl-parser-dev libgflags-dev libgoogle-glog-dev liborocos-kdl-dev \
    libgtest-dev libyaml-cpp-dev liburdf-dev libfcl-dev libnanoflann-dev \
    && rm -rf /var/lib/apt/lists/*

# ONNX Runtime (CPU) used by SL_DRGBT. The microsoft/onnxruntime GitHub repository (and its release tarballs) no longer
# exists, so headers and library are taken from Microsoft's official NuGet package. When changing the version, update
# the checksum as well (sha256sum of the downloaded .nupkg). cmake/OnnxRuntime.cmake picks up $ONNXRUNTIME_ROOT.
ARG ONNXRUNTIME_VERSION=1.23.2
ARG ONNXRUNTIME_SHA256=25fe172f7fcdf34f2b5b02c8b997f6b88abc282956849d4af4335af23d0c4e4a
RUN curl -fsSL -o /tmp/ort.nupkg https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/${ONNXRUNTIME_VERSION} \
    && echo "${ONNXRUNTIME_SHA256}  /tmp/ort.nupkg" | sha256sum -c - \
    && unzip -q /tmp/ort.nupkg 'build/native/include/*' 'runtimes/linux-x64/native/*' -d /tmp/ort \
    && mkdir -p /opt/onnxruntime \
    && mv /tmp/ort/build/native/include /opt/onnxruntime/include \
    && mv /tmp/ort/runtimes/linux-x64/native /opt/onnxruntime/lib \
    && ln -s libonnxruntime.so /opt/onnxruntime/lib/libonnxruntime.so.1 \
    && rm -rf /tmp/ort /tmp/ort.nupkg
ENV ONNXRUNTIME_ROOT=/opt/onnxruntime

# Headless image for building and running the tests.
FROM rpmplv2-base AS rpmplv2-test

RUN apt-get update \
    && apt-get install -y --no-install-recommends g++ valgrind \
    && rm -rf /var/lib/apt/lists/*

FROM rpmplv2-base AS rpmplv2-dev-container

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
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
