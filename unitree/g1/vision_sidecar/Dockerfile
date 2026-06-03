# syntax=docker/dockerfile:1
#
# robotics-connect/vision-sidecar — GPU DINOv2 ViT-S/14 encoder over TCP.
#
# Base: l4t-pytorch for JetPack 5.1 (L4T R35.2.1). PyTorch 2.0 is built
# against CUDA 11.4 which is what the host already has at
# /usr/local/cuda-11.4 — nvidia-container-toolkit mounts the driver libs
# from the host into the container at `docker run --runtime=nvidia` time.
#
# We do NOT pin a specific PyTorch version in this image — the base image
# tag is the version pin. Bumping L4T/PyTorch means bumping the FROM line.

FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

# Stop apt from trying to interact with the user during any RUN steps.
ENV DEBIAN_FRONTEND=noninteractive

# Pin DINOv2 to a Python 3.8-compatible commit.
#
# The base image ships Python 3.8 (L4T R35.2.1 was cut before 3.10 was
# standard). DINOv2 `main` now uses PEP 604 `float | None` annotations
# which raise TypeError under 3.8 at class-definition time. Commit
# `e1277af2ba` (2024-02-22 "[bug] Fix interpolation of positional
# embeddings") is the last pre-PEP-604 commit we verified clean — see
# https://github.com/facebookresearch/dinov2 commit history.
#
# torch.hub.load("user/repo:ref") only accepts branches/tags, not raw
# commit hashes, so we clone the repo ourselves and load with
# `source="local"`. When the base image bumps to Python 3.10+, we can
# drop the clone and point torch.hub.load at the canonical repo.
ARG DINOV2_REF=e1277af2ba
ENV DINOV2_REF=${DINOV2_REF}
ENV DINOV2_LOCAL=/opt/dinov2

# Install git (not in the base image), clone the repo at the pinned
# commit, then drop .git + git itself to keep the image small.
RUN apt-get update && apt-get install -y --no-install-recommends git \
    && git clone https://github.com/facebookresearch/dinov2.git ${DINOV2_LOCAL} \
    && cd ${DINOV2_LOCAL} && git checkout ${DINOV2_REF} && rm -rf .git \
    && apt-get purge -y git && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Bake the DINOv2 weights into an image layer so cold starts do not need
# network access and the first encode() is hot within a second or two
# of container boot. Weights cache under /root/.cache/torch/hub/checkpoints.
RUN python3 -c "import torch, torch.hub; \
    m = torch.hub.load('${DINOV2_LOCAL}', 'dinov2_vits14', \
                       source='local', trust_repo=True, verbose=False); \
    print('weights cached:', sum(p.numel() for p in m.parameters()), 'params')"

# The sidecar only needs a handful of Python bytes — copy it into /app.
WORKDIR /app
COPY vision_sidecar.py /app/vision_sidecar.py

# socketserver listens on 0.0.0.0 inside the container; the systemd unit
# runs us with `--net=host` so 127.0.0.1:9878 on the host maps directly
# to the server. Keeping EXPOSE for documentation only.
EXPOSE 9878

# Unbuffered stdout so `journalctl -u robotics-connect-vision-sidecar -f` is live.
ENV PYTHONUNBUFFERED=1

ENTRYPOINT ["python3", "/app/vision_sidecar.py"]
