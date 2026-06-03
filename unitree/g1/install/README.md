# Robotics Connect — Deploy to a Unitree G1 EDU

Installer for the Robotics Connect stack. Targets a stock **Unitree G1 EDU
Type F** (Jetson Orin, L4T Ubuntu 20.04). Idempotent and reversible.

**Default install** is lean (~5 MB of code deployed, no GPU sidecar).
For GPU embedding workloads that need DINOv2 image embeddings, add
`WITH_SIDECAR=1` and the installer pulls in the ~12 GB GPU
vision-sidecar Docker image too. Workloads that don't need GPU
embeddings get no benefit from the sidecar.

---

## Why not Docker?

The [vision sidecar](../vision_sidecar/) runs in Docker because it needs
a pinned CUDA userland that would clash with the factory image. The rest
of Robotics Connect doesn't — it lives in a dedicated conda env and a single
top-level directory, so uninstall is `rm -rf + conda env remove`. Docker
for the mode scripts would mean fighting CycloneDDS multicast and losing
interactive `ipython`-style iteration, for no isolation gain we don't
already get from the env.

## What gets installed

| Path | Owner | Always? |
|---|---|---|
| `/home/unitree/robotics-connect/` | this installer | yes |
| `~/miniconda3/envs/robotics-connect/` | this installer (cloned from `unitree_deploy`) | yes |
| `/etc/profile.d/robotics-connect.sh` | this installer | yes |
| `/etc/systemd/system/robotics-connect-vision-sidecar.service` | vision_sidecar/install.sh | only with `WITH_SIDECAR=1` |
| `docker robotics-connect/vision-sidecar:0.1` image | vision_sidecar/install.sh | only with `WITH_SIDECAR=1` |

## What is preserved (factory)

Nothing under `/home/unitree/cyclonedds_ws/`, `/home/unitree/unitree_ros2/`,
`/home/unitree/unitree_sdk2*`, `/home/unitree/miniconda3/envs/unitree_deploy/`,
`/home/unitree/wifi-bt-deb/`, `/home/unitree/librealsense/`, `/opt/ros/foxy/`,
`/etc/systemd/system/nv*.service`, or `~/.bashrc` is touched. The installer
refuses to run if any of those factory paths are missing (sign of a wrong
or already-modified target).

## First-time deployment to a new robot (one command on the robot)

```
# 1. Connect laptop → robot over ethernet (USB-C→Ethernet dongle, cable to
#    the G1's side port). Robot defaults to 192.168.123.164.

# 2. Copy the package onto the robot (git archive keeps only tracked
#    files — no .git, no build artifacts). HEAD:unitree/g1 packs just the
#    G1 package subtree so it extracts flat (arm_fk/, install/, ...).
git archive --format=tar HEAD:unitree/g1 | gzip > /tmp/robotics-connect.tgz
scp /tmp/robotics-connect.tgz unitree@192.168.123.164:/tmp/
ssh unitree@192.168.123.164 'mkdir -p /tmp/robotics-connect-src && tar xzf /tmp/robotics-connect.tgz -C /tmp/robotics-connect-src'

# 3. SSH in and run the one-shot bootstrap.
ssh unitree@192.168.123.164
  cd /tmp/robotics-connect-src/install
  SSID="YourNetwork" WIFI_PASS="..." ./first-boot.sh

# 4. From now on, future updates go over WiFi — no more cable needed.
```

`first-boot.sh` chains three idempotent steps:
1. `setup-wifi.sh` — enables the Realtek rtl8852bu driver persistently
   (factory driver search falls back through repo bundle → offline
   bundle → `$WIFI_BT_DEB_DIR`, so missing factory dirs are handled).
2. `nmcli device wifi connect` against the provided `$SSID` /
   `$WIFI_PASS` (prompts interactively if missing; skip via
   `SKIP_WIFI_JOIN=1`).
3. `install.sh` — deploys the Robotics Connect stack.

Safe to re-run. On an already-bootstrapped robot each step becomes a
no-op, and the final summary reports both `eth0` and `wlan0` IPs.

You can also run the three scripts individually if you prefer.

## Re-deploying after a code change

The installer is safe to re-run. It refreshes the code tree, refreshes the
activation hook, and restarts the vision sidecar. It does NOT recreate
the conda env if one already exists.

## Uninstall

install.sh deploys the full `install/` subtree, so `uninstall.sh` is
available on the robot for rollback without needing the source bundle:

```
bash /home/unitree/robotics-connect/install/uninstall.sh
```

Leaves the robot byte-for-byte factory, modulo the conda package cache.

## Offline install (no internet on the robot)

For a fully air-gapped install, produce a bundle on a build machine that
has internet + docker, then copy the bundle onto a USB drive or scp it
to the robot. `install.sh` auto-detects the bundle and skips every step
that would otherwise hit the network.

```
# ── On your build machine (laptop/DGX/etc.) ──────────────────────────
bash install/build-offline-bundle.sh
# produces dist/robotics-connect-offline-<sha>.tar (~12 GB — dominated by the
# pre-built vision-sidecar image)

# ── On the robot (over USB or scp) ───────────────────────────────────
# Extract the bundle somewhere writable
tar xf /tmp/robotics-connect-offline-*.tar -C /tmp/
cd /tmp/robotics-connect-offline-*/source

# install.sh looks for ../offline/ automatically
bash install/install.sh
```

The bundle layout:

```
robotics-connect-offline-<sha>/
├── source/                              (git-tracked source)
└── offline/
    ├── robotics-connect-vision-sidecar.tar      (docker save — 11.8 GB)
    └── wheels/                          (pip wheels for the delta)
```

## What is still TODO

- [ ] `setup-wifi.sh` assumes the factory WiFi driver package at
      `~/wifi-bt-deb/`. If Unitree changes the default image, we need
      to bundle our own copy.
- [ ] Wrap the whole first-boot flow (ethernet → setup-wifi → install
      → connect to home SSID) in a single operator-facing script so a
      non-technical user never has to type more than one command.
