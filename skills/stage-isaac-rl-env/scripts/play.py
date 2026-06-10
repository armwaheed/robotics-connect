"""Play the reach policy and record an mp4 for verify-by-eye (the durable project rule: judge behavior
by what the human sees, never by telemetry).

Renders free-base humanoids tracking sampled hand targets (the goal-pose marker is drawn) so you can
SEE whether the robot balances while reaching the deep target or topples. Frames are grabbed from an
in-scene Camera sensor and ffmpeg-encoded — gymnasium.RecordVideo does NOT capture Isaac vec envs
headless on this build (IsaacLab#875), so we read the in-scene camera + ffmpeg, the proven path.

Run (from the IsaacLab dir; LD_PRELOAD mandatory on aarch64):
  cd ~/workspaces/git/IsaacLab && export LD_PRELOAD=$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1 \
    && ./isaaclab.sh -p play.py --headless --enable_cameras --video --checkpoint <path/to/model_xxxx.pt>

Lifted + generalized from armwaheed/robots#2 `rl/play.py`.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play the whole-body reach policy.")
parser.add_argument("--num_envs", type=int, default=4, help="Number of envs to simulate.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to model_*.pt.")
parser.add_argument("--video", action="store_true", help="Record an mp4.")
parser.add_argument("--video_length", type=int, default=400, help="Frames to record (50 Hz control).")
parser.add_argument("--zero", action="store_true", help="Run a zero policy (pre-train spawn sanity).")
parser.add_argument("--tag", type=str, default="eval", help="Filename tag for the mp4.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.video:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import glob  # noqa: E402
import importlib.metadata as metadata  # noqa: E402
import os  # noqa: E402
import shutil  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from PIL import Image  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from agents import ReachPPORunnerCfg  # noqa: E402
from reach_env_cfg import ReachEnvCfg_PLAY  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def latest_checkpoint() -> str | None:
    runs = sorted(glob.glob(os.path.join(HERE, "logs", "reach_wbc", "*")))
    for run in reversed(runs):
        ckpts = sorted(glob.glob(os.path.join(run, "model_*.pt")), key=lambda p: int(p.split("_")[-1].split(".")[0]))
        if ckpts:
            return ckpts[-1]
    return None


def encode(frames_dir: str, out_mp4: str, n: int) -> None:
    if shutil.which("ffmpeg"):
        subprocess.run(
            ["ffmpeg", "-y", "-framerate", "25", "-pattern_type", "glob",
             "-i", os.path.join(frames_dir, "frame_*.png"),
             "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-pix_fmt", "yuv420p", out_mp4],
            capture_output=True,
        )
        print(f"\n[reach] >>> EVAL MP4 READY: {out_mp4}  ({n} frames)\n")
    else:
        print(f"[reach] {n} frames in {frames_dir} (install ffmpeg for mp4)")


def main():
    env_cfg = ReachEnvCfg_PLAY()
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    policy = None
    if not args_cli.zero:
        agent_cfg = ReachPPORunnerCfg()
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))
        agent_cfg.device = str(env.unwrapped.device)
        resume = args_cli.checkpoint or latest_checkpoint()
        if resume is None:
            print("[reach] no checkpoint found; falling back to a zero policy.")
        else:
            runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
            runner.load(resume)
            policy = runner.get_inference_policy(device=env.unwrapped.device)
            print(f"[reach] loaded checkpoint: {resume}")

    frames_dir = out_mp4 = cam = None
    if args_cli.video:
        out_dir = os.path.join(HERE, "eval")
        frames_dir = os.path.join(out_dir, f"frames_{args_cli.tag}")
        shutil.rmtree(frames_dir, ignore_errors=True)
        os.makedirs(frames_dir, exist_ok=True)
        out_mp4 = os.path.join(out_dir, f"reach_{args_cli.tag}.mp4")
        cam = env.unwrapped.scene["eval_cam"]
        origins = env.unwrapped.scene.env_origins
        eyes = origins + torch.tensor([2.4, 2.4, 1.7], device=origins.device)
        targets = origins + torch.tensor([0.25, 0.0, 0.7], device=origins.device)
        cam.set_world_poses_from_view(eyes, targets)

    obs = env.get_observations()  # rsl-rl 5.x returns a TensorDict (single value, not a tuple)
    n_act = env.unwrapped.action_manager.total_action_dim
    steps = args_cli.video_length if args_cli.video else 1000
    saved = 0
    with torch.inference_mode():
        for _ in range(steps):
            actions = policy(obs) if policy is not None else torch.zeros((env.unwrapped.num_envs, n_act), device=env.unwrapped.device)
            obs, _, _, _ = env.step(actions)
            if cam is not None:
                rgb = cam.data.output["rgb"]
                if rgb is not None and rgb.shape[0] > 0:
                    img = rgb[0, :, :, :3].detach().cpu().numpy().astype(np.uint8)
                    Image.fromarray(img).save(os.path.join(frames_dir, f"frame_{saved:04d}.png"))
                    saved += 1

    env.close()
    if args_cli.video and saved > 0:
        encode(frames_dir, out_mp4, saved)
    elif args_cli.video:
        print("[reach] WARNING: no frames captured (camera produced no rgb).")


if __name__ == "__main__":
    main()
    simulation_app.close()
    os._exit(0)
