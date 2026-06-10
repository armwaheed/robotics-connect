---
name: unitree-g1-connect
description: >-
  Connect a control host (a workstation or a DGX Spark) to a Unitree G1 EDU over the network — route the
  host onto the robot's 192.168.123.0/24 subnet over WiFi and configure CycloneDDS for unicast. Use when
  you need to talk to the robot's DDS topics (LiDAR, video, lowstate) from a host. Reversible via the
  revert_*.sh counterparts. (This is host↔robot networking; for the host's Isaac Sim stack see
  setup-dgx-spark.)
metadata:
  tags: [unitree-g1, networking, dds, cyclonedds, subnet, connectivity, host]
---

# Unitree G1 — connect a control host

Route a control host to the robot's subnet and configure CycloneDDS so DDS topics (LiDAR, video,
lowstate) are reachable. The scripts live at the `unitree/g1/` root; this skill wraps them.

## When to use

- Connect a workstation or DGX Spark to a G1 EDU to read its DDS topics (the `*_sight` capability skills
  subscribe to these), then revert when done.

## The scripts (at `unitree/g1/`)

| Script | Role |
|---|---|
| [`../configure_robot.sh`](../configure_robot.sh) | Configure the robot side of the host↔robot route. |
| [`../configure_spark.sh`](../configure_spark.sh) | Route a control host (DGX Spark / workstation) onto the robot's `192.168.123.0/24` subnet over WiFi. |
| [`../revert_robot.sh`](../revert_robot.sh) · [`../revert_spark.sh`](../revert_spark.sh) | Undo each side. |
| [`../cyclonedds.xml`](../cyclonedds.xml) | CycloneDDS unicast config (the robot's services expect unicast, not multicast discovery). |

## Use it

Run order and host-specific details are in each script's header. Point `CYCLONEDDS_URI` at the deployed
config so DDS uses the robot's unicast routing:

```bash
bash configure_spark.sh                                              # route the host onto the subnet
export CYCLONEDDS_URI=file:///home/unitree/robotics-connect/cyclonedds.xml
# ... use the *_sight skills / DDS topics ...
bash revert_spark.sh                                                 # restore host networking
```

> The robot's `eth0` lives on its own `192.168.123.0/24` subnet; these scripts bridge a WiFi-connected
> host to it. For the host's Isaac Sim / Isaac Lab setup (a separate concern), see
> [`setup-dgx-spark`](../../../skills/setup-dgx-spark/SKILL.md).
