FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxfixes3 \
    libxi6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libegl1 \
    libglfw3 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages open3d

WORKDIR /workspace

COPY open3d_loop_closure_matching.py /workspace/open3d_loop_closure_matching.py

CMD ["python3", "open3d_loop_closure_matching.py"]