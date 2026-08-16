#!/bin/bash

set -e

IMAGE_NAME="open3d_docker"

CLONE_DIR="$HOME/hdmapping-benchmark-loop-closure"
OPEN3D_REPO="$CLONE_DIR/benchmark-HDMapping-AILoopClosure-Open3D"
DATA_DIR="$CLONE_DIR/data"

echo "==========================================="
echo "OPEN3D LOOP CLOSURE"
echo "==========================================="

echo "Repository:"
echo "$OPEN3D_REPO"

echo "Dataset:"
echo "$DATA_DIR"

if [[ $# -ne 1 ]]; then
    echo
    echo "Usage:"
    echo "  $0 <folder>"
    echo
    echo "Example:"
    echo "  $0 ~/hdmapping-benchmark-loop-closure/data/2011_09_30/2011_09_30_drive_0016_extract/velodyne_points/data"
    exit 1
fi

FOLDER="$(realpath "$1")"

echo "Folder:"
echo "$FOLDER"


if [[ ! -d "$FOLDER" ]]; then
    echo
    echo "ERROR: Folder does not exist:"
    echo "$FOLDER"
    exit 1
fi

SOURCE_FILE="$FOLDER/0000000000.txt"
TARGET_FILE="$FOLDER/0000000001.txt"

if [[ ! -f "$SOURCE_FILE" ]]; then
    echo
    echo "ERROR: Source file does not exist:"
    echo "$SOURCE_FILE"
    exit 1
fi

if [[ ! -f "$TARGET_FILE" ]]; then
    echo
    echo "ERROR: Target file does not exist:"
    echo "$TARGET_FILE"
    exit 1
fi

echo
echo "Source:"
echo "$SOURCE_FILE"

echo "Target:"
echo "$TARGET_FILE"

DOCKER_FOLDER="${FOLDER#$DATA_DIR}"

if [[ "$DOCKER_FOLDER" == "$FOLDER" ]]; then
    echo
    echo "ERROR: Folder must be inside:"
    echo "$DATA_DIR"
    exit 1
fi

DOCKER_FOLDER="/data$DOCKER_FOLDER"

echo
echo "Docker folder:"
echo "$DOCKER_FOLDER"

echo
echo "==========================================="
echo "RUNNING OPEN3D"
echo "==========================================="

docker run --rm -it \
    --network host \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$DATA_DIR:/data" \
    "$IMAGE_NAME" \
    python3 /workspace/open3d_loop_closure_matching.py \
    "$DOCKER_FOLDER"

echo
echo "==========================================="
echo "OPEN3D DONE"
echo "==========================================="