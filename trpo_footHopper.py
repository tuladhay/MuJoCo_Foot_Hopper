# from rllab.algos.trpo import TRPO
# from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
# from footHopper_env import footHopperEnv
# from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
# import pickle
# from rllab.misc.instrument import run_experiment_lite
# from rllab.envs.normalized_env import normalize
#
#
# def run_task(*_):
#     env = normalize(footHopperEnv())
#     policy = GaussianMLPPolicy(
#         env_spec=env.spec,
#         hidden_sizes=(32, 32)
#     )
#     baseline = LinearFeatureBaseline(env_spec=env.spec)
#     algo = TRPO(
#         env=env,
#         policy=policy,
#         baseline=baseline,
#         batch_size=4000,
#         max_path_length=20,
#         n_itr=1000,
#         discount=0.99,
#         step_size=0.01,
#     )
#     algo.train()
#
# # rollout(env, policy, max_path_length=10000, animated=True, speedup=1000)
#
#
# run_experiment_lite(
#     run_task,
#     # Number of parallel workers for sampling
#     n_parallel=1,
#     # Only keep the snapshot parameters for the last iteration
#     snapshot_mode="last",
#     # Specifies the seed for the experiment. If this is not provided, a random seed
#     # will be used
#     seed=1,
#     # plot=True,
#     log_dir='log',
# )
#




from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from footHopper_env import footHopperEnv
from rllab.envs.normalized_env import normalize
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy

env = normalize(footHopperEnv())
policy = GaussianMLPPolicy(
    env_spec=env.spec,
)
baseline = LinearFeatureBaseline(env_spec=env.spec)
algo = TRPO(
    env=env,
    policy=policy,
    baseline=baseline,
    n_itr=40,
    discount=0.99,
    batch_size=4000,
    max_path_length=500,
)
algo.train()
