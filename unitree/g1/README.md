# Unitree G1 EDU — control stack

Control and perception stack for the **Unitree G1 EDU** humanoid, part of
the `robotics-connect` toolkit. Each subdirectory is a self-contained,
import-and-call module; together they cover arm kinematics, hand touch
sensing, depth and LiDAR perception, a GPU inference sidecar, and the
deployment + networking glue needed to run them on the robot.

## Modules

| Module | What it does |
|---|---|
| [`arm_fk/`](arm_fk/) | Pure-numpy URDF forward kinematics for the G1 arms. Turns a 14-DOF `arm_q` into body-frame palm / elbow / wrist XYZ in the same frame the depth camera reports — numpy + stdlib only, ~1 kHz on the Jetson. |
| [`brainco_touch/`](brainco_touch/) | Direct-Modbus bridge for the Brainco Revo2 **Touch** hands: per-finger normal / tangential force published over a local TCP JSON protocol, plus an installer and smoke test. |
| [`depth_camera_sight/`](depth_camera_sight/) | Intel RealSense depth-camera perception: table-plane estimation and on-table object localisation in the robot body frame, plus a depth encoder. |
| [`lidar_sight/`](lidar_sight/) | Mid-field LiDAR perception singleton and scene mapping: frame accumulation across waist yaw, floor / obstacle estimation, an A\* planner, and goal-table selection. |
| [`vision_sidecar/`](vision_sidecar/) | Containerised GPU inference sidecar (DINOv2 ViT-S/14) exposing a local TCP RPC on `127.0.0.1:9878` for accelerated image embeddings, so host processes need no GPU torch wheel. |
| [`install/`](install/) | Deploy / uninstall the stack onto a fresh G1 EDU: WiFi driver, dedicated conda env, shell activation hook, optional GPU sidecar, and an offline (no-internet) bundle builder. |

The top-level `configure_*.sh` / `revert_*.sh` scripts plus `cyclonedds.xml`
wire networking between a control host and the robot — see below.

---

# Networking: control host ↔ G1 private ethernet

This directory also contains four helper scripts to configure network communication between your control host (e.g. an NVIDIA DGX Spark) and your Unitree G1 robot, allowing you to interact with the robot's `eth0` network adapter (`192.168.123.x` subnet) over your existing Wi-Fi connection.

**Update Note:** The `configure_spark.sh` and `revert_spark.sh` scripts have been updated to fix a Netplan error (`No access points defined`). The new versions now safely modify your existing Netplan configuration file instead of creating a new one.

## Why These Scripts?

You may not be able to directly communicate with the robot's `eth0` interface because it's on a different subnet (`192.168.123.0/24`) than your home Wi-Fi (`192.168.1.0/24`), which the control host is connected to. Direct cabling is inconvenient, and these scripts provide a robust solution using network routing and a firewall.

The solution involves:
1.  **Robot as a Router:** Configuring the G1 robot to act as a router, forwarding traffic between its Wi-Fi (`wlan0`) and Ethernet (`eth0`) interfaces.
2.  **Static Route on the Host:** Telling the control host that to reach the robot's `eth0` subnet, it needs to send the traffic via the robot's `wlan0` IP address.
3.  **Firewall for Security:** Implementing a firewall on the robot to restrict forwarded traffic **only** to and from the control host, enhancing security.

## Prerequisites

*   `sudo` access on both the control host and the Unitree G1 robot.
*   `scp` available on the control host to copy files to the robot.
*   The control host's Wi-Fi IP address (e.g. `192.168.1.110`).
*   The robot's Wi-Fi IP address (e.g. `192.168.1.119`, `wlan0` interface).
*   The robot's Ethernet IP address (e.g. `192.168.123.164`, `eth0` interface).

## The Scripts

*   `configure_robot.sh`: Configures IP forwarding and firewall rules on your **G1 robot**.
*   `revert_robot.sh`: Reverts the network configuration changes on your **G1 robot**.
*   `configure_spark.sh`: **Safely modifies** the existing network configuration on your **control host** to add a persistent static route. It creates a backup before making changes.
*   `revert_spark.sh`: **Safely restores** the original configuration from the backup on your **control host**.

## How to Use

Follow these steps carefully:

### Step 1: Configure the G1 Robot

1.  **Copy scripts to the robot:**
    From the control host, copy the robot configuration scripts to your G1 robot. Replace `unitree@192.168.1.119` with your robot's actual SSH user and IP if it differs.
    ```bash
    scp configure_robot.sh revert_robot.sh unitree@192.168.1.119:~/
    ```

2.  **Execute configuration on the robot:**
    SSH into your robot and run the `configure_robot.sh` script. This will enable IP forwarding, install `iptables-persistent` if needed, set up specific firewall rules, and save them.
    ```bash
    ssh unitree@192.168.1.119
    # Once connected to the robot:
    sudo ./configure_robot.sh
    exit
    ```

### Step 2: Configure the Control Host

1.  **Execute configuration on the host:**
    From the control host, run the `configure_spark.sh` script. It will automatically find and safely edit the correct network configuration to establish the route.
    ```bash
    sudo ./configure_spark.sh
    ```

    NOTE: `cyclonedds.xml` tells the DDS service to send messages to a specific IP address rather than multicast them. Point `CYCLONEDDS_URI` at the deployed copy, e.g.:

       export CYCLONEDDS_URI=file:///home/unitree/robotics-connect/cyclonedds.xml

### Step 3: Verify the Connection

After both configuration scripts have been run, you should be able to ping the robot's `eth0` IP address from the control host:

```bash
ping 192.168.123.164
```

You should see successful replies. If not, double-check the IP addresses, interface names, and ensure both configuration scripts executed without errors.

### Step 4: Reverting Changes (If Needed)

If you need to undo these network configurations:

1.  **Revert host changes:**
    On the control host, the revert script will restore the backup.
    ```bash
    sudo ./revert_spark.sh
    ```

2.  **Revert Robot changes:**
    SSH into your robot and run the `revert_robot.sh` script:
    ```bash
    ssh unitree@192.168.1.119
    # Once connected to the robot:
    sudo ./revert_robot.sh
    exit
    ```

## Security Considerations

The `configure_robot.sh` script includes `iptables` firewall rules that only permit traffic from the control host's IP to the robot's `eth0` network. All other forwarding attempts from the Wi-Fi network are dropped. This significantly mitigates the security risks associated with enabling IP forwarding on the robot. However, always ensure your Wi-Fi network itself is secure.
