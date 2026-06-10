"""rsl_rl PPO runner config for the whole-body reach policy.

Mirrors Isaac Lab's own G1 flat-locomotion PPO cfg (the config schema that trains cleanly with the
bundled rsl-rl-lib 5.x). `actor_obs_normalization` / `critic_obs_normalization` are set explicitly so
the deprecation shim (`handle_deprecated_rsl_rl_cfg`, called in train.py) converts the deprecated
`policy` cfg into the new actor/critic schema without a `KeyError: 'class_name'`. Lifted verbatim from
armwaheed/robots#2 `rl/agents.py`.
"""

from __future__ import annotations

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class ReachPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 3000
    save_interval = 100
    experiment_name = "reach_wbc"
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.008,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
