"""Train the whole-body reach policy (free-base humanoid) with rsl_rl PPO.

Self-contained: builds the manager-based env directly and drives rsl-rl-lib's OnPolicyRunner. Calls
Isaac Lab's deprecation shim so the agent cfg works with the bundled rsl-rl-lib 5.x (no
`KeyError: 'class_name'`). On finish, exports the policy to JIT + ONNX for deployment.

The robot is selected by the env vars read in reach_env_cfg.py:
  ROBOTICS_CONNECT_DESCRIPTOR=<descriptor.json>   ROBOTICS_CONNECT_MOBILE_USD=<free-base.usd>

Run (headless on the Spark GPU, from the IsaacLab dir; LD_PRELOAD mandatory on aarch64):
  cd ~/workspaces/git/IsaacLab && export LD_PRELOAD=$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1 \
    && ROBOTICS_CONNECT_DESCRIPTOR=.../unitree_g1_29dof.json \
       ROBOTICS_CONNECT_MOBILE_USD=.../g1_inspire_mobile.usd \
       ./isaaclab.sh -p train.py --headless --num_envs 2048 --max_iterations 1000

Lifted + generalized from armwaheed/robots#2 `rl/train.py`.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train the whole-body reach policy.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of parallel envs.")
parser.add_argument("--max_iterations", type=int, default=None, help="PPO iterations.")
parser.add_argument("--seed", type=int, default=42, help="Random seed.")
parser.add_argument("--run_name", type=str, default="", help="Suffix for the run directory.")
parser.add_argument("--resume_from", type=str, default=None,
                    help="Warm-start: load this model_*.pt checkpoint before training (continue/fine-tune).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import importlib.metadata as metadata  # noqa: E402
import os  # noqa: E402
import sys  # noqa: E402
from datetime import datetime  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab.utils.dict import print_dict  # noqa: E402
from isaaclab.utils.io import dump_yaml  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from agents import ReachPPORunnerCfg  # noqa: E402
from reach_env_cfg import ReachEnvCfg  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    env_cfg = ReachEnvCfg()
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    agent_cfg = ReachPPORunnerCfg()
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    agent_cfg.seed = args_cli.seed
    # Convert the deprecated `policy` cfg -> new actor/critic schema for rsl-rl-lib >= 4.0.
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run = stamp if not args_cli.run_name else f"{stamp}_{args_cli.run_name}"
    log_dir = os.path.join(HERE, "logs", agent_cfg.experiment_name, run)
    os.makedirs(log_dir, exist_ok=True)
    print(f"[reach] logging to: {log_dir}")
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)

    env = ManagerBasedRLEnv(cfg=env_cfg)
    agent_cfg.device = str(env.unwrapped.device)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print_dict(agent_cfg.to_dict(), nesting=0)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    if args_cli.resume_from is not None:
        print(f"[reach] warm-starting from checkpoint: {args_cli.resume_from}")
        runner.load(args_cli.resume_from)
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    export_dir = os.path.join(log_dir, "exported")
    runner.export_policy_to_jit(path=export_dir, filename="policy.pt")
    runner.export_policy_to_onnx(path=export_dir, filename="policy.onnx")
    print(f"[reach] exported policy to: {export_dir}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    # Isaac's replicator orchestrator can hang on a clean close; force exit after artifacts land.
    os._exit(0)
