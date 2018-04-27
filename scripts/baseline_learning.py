#!/usr/bin/env python
"""Script for finetuning DIAYN-learned skills.

"""
import rospy
import numpy as np
from aquacore.srv import SetGait

from rllab.misc.instrument import VariantGenerator

from sac.algos import SAC
from aqua_sac_env import AquaEnv
from sac.envs.meta_env import FixedOptionEnv
from sac.misc.instrument import run_sac_experiment
from sac.misc.sampler import rollouts
from sac.misc.utils import timestamp
from sac.policies.hierarchical_policy import FixedOptionPolicy
from sac.policies.gmm import GMMPolicy
from sac.replay_buffers import SimpleReplayBuffer
from sac.value_functions import NNQFunction, NNVFunction
from rllab.envs.normalized_env import normalize

from rllab.envs.env_spec import EnvSpec
from rllab import spaces


import argparse
# import joblib
import numpy as np
import os
import tensorflow as tf


SHARED_PARAMS = {
    'seed': 1,
    'lr': 3E-4,
    'discount': 0.99,
    'tau': 0.01,
    'K': 4,
    'layer_size': 200,
    'batch_size': 128,
    'max_pool_size': 1E6,
    'n_train_repeat': 1,
    'epoch_length': 25,
    'snapshot_mode': 'all',
    'snapshot_gap': 10,
    'sync_pkl': True,
    'use_pretrained_values': False,  # Whether to use qf and vf from pretraining
}

TAG_KEYS = ['n_epochs', 'layer_size']

ENV_PARAMS = {
    'aqua': {
        'prefix': 'aqua',
        'env_name': 'Aqua-v0',
        'max_path_length': 25,
        'n_epochs': 700,
        'scale_reward': 1,
    },
}
DEFAULT_ENV = 'aqua'
AVAILABLE_ENVS = list(ENV_PARAMS.keys())

def parse_args(rospy_args):
    parser = argparse.ArgumentParser()
    parser.add_argument('--env',
                        type=str,
                        choices=AVAILABLE_ENVS,
                        default='aqua')
    parser.add_argument('--exp_name', type=str, default=timestamp())
    parser.add_argument('--mode', type=str, default='local')
    parser.add_argument('--log_dir', type=str, default=None)
    args = parser.parse_args(rospy_args)

    return args

def get_variants(args):
    env_params = ENV_PARAMS[args.env]
    params = SHARED_PARAMS
    params.update(env_params)

    vg = VariantGenerator()
    for key, val in params.items():
        if isinstance(val, list):
            vg.add(key, val)
        else:
            vg.add(key, [val])
    return vg

def run_experiment(variant):
    tf.logging.set_verbosity(tf.logging.INFO)
    with tf.Session() as sess:

        env = normalize(AquaEnv(variant['env_name']))
        # obs_space = env.spec.observation_space
        #
        # low = np.hstack([obs_space.low, np.full(variant['num_skills'], 0)])
        # high = np.hstack([obs_space.high, np.full(variant['num_skills'], 1)])
        # aug_obs_space = spaces.Box(low=low, high=high)
        # aug_env_spec = EnvSpec(aug_obs_space, env.spec.action_space)
        aug_env_spec = env.spec
        pool = SimpleReplayBuffer(
            env_spec=aug_env_spec,
            max_replay_buffer_size=variant['max_pool_size'],
        )

        base_kwargs = dict(
            min_pool_size=variant['max_path_length'],
            min_pool_size=variant['batch_size'], #variant['max_path_length'],
            epoch_length=variant['epoch_length'],
            n_epochs=variant['n_epochs'],
            max_path_length=variant['max_path_length'],
            batch_size=variant['batch_size'],
            n_train_repeat=variant['n_train_repeat'],
            eval_render=False,
            eval_n_episodes=1,
            eval_deterministic=True,
        )

        M = variant['layer_size']
        network_arch = [M, M, M, M]

        qf = NNQFunction(
            env_spec=aug_env_spec,
            hidden_layer_sizes=network_arch,
            var_scope='qf-baseline',
        )

        vf = NNVFunction(
            env_spec=aug_env_spec,
            hidden_layer_sizes=network_arch,
            var_scope='vf-baseline',
        )

        policy = GMMPolicy(
            env_spec=aug_env_spec,
            K=variant['K'],
            hidden_layer_sizes=network_arch,
            qf=qf,
            reg=0.001,
        )

        algorithm = SAC(
            base_kwargs=base_kwargs,
            env=env,
            policy=policy,
            pool=pool,
            qf=qf,
            vf=vf,
            lr=variant['lr'],
            scale_reward=variant['scale_reward'],
            discount=variant['discount'],
            tau=variant['tau'],
            save_full_state=False,
        )

        algorithm.train()


def launch_experiments(variant_generator):
    variants = variant_generator.variants()

    for i, variant in enumerate(variants):
        tag = 'baseline__'
        tag += '__'.join(['%s_%s' % (key, variant[key]) for key in TAG_KEYS])
        log_dir = os.path.join(args.log_dir, tag)
        print('Launching {} experiments.'.format(len(variants)))
        run_sac_experiment(
            run_experiment,
            mode=args.mode,
            variant=variant,
            exp_prefix=variant['prefix'] + '/' + args.exp_name,
            exp_name=variant['prefix'] + '-' + args.exp_name + '-' + str(i).zfill(2),
            n_parallel=1,  # Increasing this barely effects performance,
                           # but breaks learning of hierarchical policy.
            seed=variant['seed'],
            terminate_machine=True,
            log_dir=log_dir,
            snapshot_mode=variant['snapshot_mode'],
            snapshot_gap=variant['snapshot_gap'],
            sync_s3_pkl=variant['sync_pkl'],
        )


def set_gait_flex_sine():
    rospy.wait_for_service('/aqua/set_gait')
    try:
        resp = rospy.ServiceProxy('/aqua/set_gait', SetGait)
        resp('flexible-sine')
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    set_gait_flex_sine()
    args = parse_args(rospy.myargv()[1:])
    variant_generator = get_variants(args)
    launch_experiments(variant_generator)
