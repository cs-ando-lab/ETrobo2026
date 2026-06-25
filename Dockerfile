# ETロボコン プラットフォーム (C/C++ / TOPPERSベース) をビルドするための環境
FROM ubuntu:22.04

# 対話プロンプトを無効化
ENV DEBIAN_FRONTEND=noninteractive

# 必要なパッケージ（Ruby, CMake, クロスコンパイラ、clang-formatなど）をインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    ruby \
    git \
    cmake \
    clang-format \
    wget \
    tar \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# GNU Arm Embedded Toolchain のインストール 
WORKDIR /opt
RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 \
    && tar xjf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 \
    && rm gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2

ENV PATH="/opt/gcc-arm-none-eabi-10.3-2021.10/bin:${PATH}"

# spike-rt 環境のクローン
WORKDIR /etrobo
RUN git clone --depth=1 https://github.com/ETrobocon/spike-rt-RasPike-ART.git spike-rt -b main

# 作業ディレクトリをソースコードの場所に設定
WORKDIR /etrobo/workspace/ETrobo2026
