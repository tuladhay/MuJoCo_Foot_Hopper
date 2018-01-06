from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from footHopper_env import footHopperEnv
from rllab.envs.normalized_env import normalize
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.misc.instrument import run_experiment_lite
import gym


def run_task(*_):
    env = normalize(footHopperEnv())
    policy = GaussianMLPPolicy(
        env_spec=env.spec,
        hidden_sizes=(16, 16)
    )
    baseline = LinearFeatureBaseline(env_spec=env.spec)

    algo = TRPO(
        env=env,
        policy=policy,
        baseline=baseline,
        n_itr=2000,
        discount=0.99,
        batch_size=4000,
        max_path_length=500,
        render=True,
    )
    algo.train()


run_experiment_lite(
    run_task,
    # Number of parallel workers for sampling
    n_parallel=1,
    # Only keep the snapshot parameters for the last iteration
    snapshot_mode="last",
    # Specifies the seed for the experiment. If this is not provided, a random seed
    # will be used
    seed=1,
    log_dir='log',
)
