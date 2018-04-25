#!/usr/bin/env python
import gym
import numpy as np
import rospy

from gym import spaces
from Queue import Queue

from std_srvs.srv import Empty as EmptySrv
from robot_learning.msg import ExperienceData
# from robot_learning.srv import T2VInfo

from aqua_diayn.srv import EnvSpaceBounds, EnvSpaceUnits
from aqua_diayn.msg import TargetState

import csv
import os
import json


class ExponentialReward():
    def __init__(self, c=10.0):
        self.c = c
        self.scale_factor = - 1./(2. * self.c)

    def __call__(self, curr_state, target_state=None):
        reward = -1.

        if target_state is not None:

            if isinstance(curr_state, list):
                curr_state = np.array(curr_state)
            if isinstance(target_state, list):
                target_state = np.array(target_state)

            curr_state.reshape(-1, 1)
            target_state.reshape(-1, 1)

            Q = np.eye(len(curr_state))
            for ii, tt in enumerate(target_state):
                if np.isnan(tt):
                    Q[ii, ii] = 0.0
                    target_state[ii] = 0.0  # NOTE: This value is irrelevant.

            err = curr_state - target_state
            reward = np.matmul(np.matmul(err.T, Q), err)
            reward = np.exp(self.scale_factor * reward) - 1

        return reward


def angles2vector(state, units):
    aug_state = []

    if state is None:
        return None

    for idx, unit in enumerate(units):
        if unit == 'radians':
            angle_sin = np.sin(state[idx])
            angle_cos = np.cos(state[idx])
            aug_state.append(angle_sin)
            aug_state.append(angle_cos)
        else:
            aug_state.append(state[idx])

    return aug_state

class CSVLogger():
    def __init__(self):#, filepath, fieldnames):
        # self.filepath = filepath
        # self.fieldnames = fieldnames
        # self.wrote_header = False
        pass

    def dump(self, data):
        # if not self.wrote_header:
        #     with open(self.filepath, 'wb') as csvfile:
        #         self.exp_keys = self.expand_fields(data)
        #         self.exp_fields = reduce(lambda x, y: x + y,
        #                                  self.exp_keys.values())
        #         writer = csv.DictWriter(csvfile, fieldnames=self.exp_fields)
        #         writer.writeheader()
        #         self.wrote_header = True

        # with open(self.filepath, 'ab') as csvfile:
        self.exp_keys = self.expand_fields(data)
        expanded_data = {}
        for key, fields in self.exp_keys.iteritems():
            key_data = data[key]
            try:
                for field, val in zip(fields, key_data):
                    expanded_data[field] = val
            except TypeError:
                expanded_data[key] = key_data

        log_str = json.dumps(expanded_data)
        rospy.logdebug(log_str)
        # writer = csv.DictWriter(csvfile, fieldnames=self.exp_fields)
        # writer.writerow(expanded_data)
            # outlist = [str(kk) + ': ' + str(vv) for kk, vv in data.iteritems()]
            # outstr = ', '.join(outlist) + '\n'
            # logfile.write(outstr)

    def expand_fields(self, data):
        expanded_keys = {}
        for key, value in data.iteritems():
            try:
                expanded = [key + '_' + str(ii) for ii in range(len(value))]
                expanded_keys[key] = expanded
            except TypeError:
                # value is not iterable
                expanded_keys[key] = [key]
        return expanded_keys

    def log_reset(self):
        # if self.wrote_header:
            # with open(self.filepath, 'ab') as csvfile:
                # writer = csv.DictWriter(csvfile, fieldnames=self.exp_fields)
                # row = {kk: 'RESET' for kk in self.exp_fields}
                # writer.writerow(row)
        rospy.logdebug('RESET')


class ROSPlant(gym.Env):
    '''
    Class for collecting msg and executing policies on a ROS-enabled robot
    '''

    reset_srv_name = '/rl/reset_robot'
    stop_srv_name = '/rl/stop_robot'

    command_dims_srv_name = '/rl/command_dims'
    state_dims_srv_name = '/rl/state_dims'

    state_bounds_srv_name = '/rl/state_bounds'
    command_bounds_srv_name = '/rl/command_bounds'

    state_units_srv_name = '/rl/state_units'

    def __init__(self, state0_dist=None, loss_func=None, dt=0.5,
                 noise_dist=None, angle_dims=[], name='ROSPlant',
                 init_ros_node=False, max_experience_queue_size=1000,
                 experience_topic='/rl/command_data',
                 command_topic='/rl/experience_data',
                 *args, **kwargs):
        # init queue. This is to ensure that we collect experience at every dt
        # seconds
        self.experience_queue = Queue(maxsize=max_experience_queue_size)
        self.t = 0

        # initalize internal plant parameteers
        self.init_params(state0_dist, loss_func, dt, noise_dist, angle_dims,
                         name, *args, **kwargs)

        # initialize ros node, publishers and subscribers
        self.ros_init(init_ros_node)

        # Observation and action spaces will be populated by reading from the
        # appropriate topics
        self.init_obs_act_spaces()

        # initialize publishers and subscribers
        self.command_pub = rospy.Publisher(
            '/rl/command_data', ExperienceData, queue_size=-1)
        self.experience_sub = rospy.Subscriber(
            '/rl/experience_data', ExperienceData, self.experience_callback,
            queue_size=-1)
        self.target_state_sub = rospy.Subscriber(
            '/rl/target_state', TargetState, self.target_state_callback,
            queue_size=-1)

        # get initial state
        self.state = self.wait_for_state(dt=0.1*self.dt)[1]
        rospy.loginfo(
            '[%s] waiting for first experience data msg...' % (self.name))

        self.t, self.state, self.cmd = self.wait_for_state(self.dt)
        rospy.loginfo('[%s] Ready.' % (self.name))

        #
        # logfile_path = '/localdata/nikhil/data/comp767project_diayn/diversity_learning/rosplant_log.csv'
        # log_dict = {'curr_state': None,
        #             'action': None,
        #             'reward': None,
        #             'next_state': None
        #             }
        self.logger = CSVLogger()

    def init_params(self, state0_dist=None, loss_func=None, dt=0.5,
                    noise_dist=None, angle_dims=[], name='ROSPlant',
                    *args, **kwargs):
        self.name = name
        self.dt = dt
        self.noise_dist = noise_dist
        self.angle_dims = angle_dims

        # initial state. only needed for gazebo environments, or in cases
        # where we know how to reset to an initial state, (e.g. a robotic arm)
        self.state0_dist = state0_dist

        # Target state representing a particular task
        self.target_state = None

        # user specified reward/loss function. takes as input state vector,
        # produces as output scalar reward/cost. If not specified, the step
        # function will return None for the reward/loss function
        if loss_func is None:
            self.loss_func = ExponentialReward()
        else:
            self.loss_func = loss_func

    def ros_init(self, init_ros_node=False):
        # init plant ros node
        if init_ros_node:
            rospy.init_node(self.name, anonymous=True, disable_signals=True)
        # start service proxies
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name, ROSPlant.reset_srv_name))
        rospy.wait_for_service(ROSPlant.reset_srv_name)
        self.reset_srv = rospy.ServiceProxy(ROSPlant.reset_srv_name, EmptySrv)
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name, ROSPlant.stop_srv_name))
        rospy.wait_for_service(ROSPlant.stop_srv_name)
        self.stop_srv = rospy.ServiceProxy(ROSPlant.stop_srv_name, EmptySrv)
        # init time
        self.t = rospy.get_time()

    def init_obs_act_spaces(self):

        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name,
                                        ROSPlant.state_units_srv_name))
        rospy.wait_for_service(ROSPlant.state_units_srv_name)
        self.state_units = rospy.ServiceProxy(ROSPlant.state_units_srv_name, EnvSpaceUnits)().units

        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name,
                                        ROSPlant.state_bounds_srv_name))
        rospy.wait_for_service(ROSPlant.state_bounds_srv_name)
        s_bounds = rospy.ServiceProxy(ROSPlant.state_bounds_srv_name, EnvSpaceBounds)

        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name,
                                        ROSPlant.command_bounds_srv_name))
        rospy.wait_for_service(ROSPlant.command_bounds_srv_name)
        c_bounds = rospy.ServiceProxy(ROSPlant.command_bounds_srv_name, EnvSpaceBounds)

        o_lbound = []
        o_ubound = []
        for dim_bounds in s_bounds().bounds:
            o_lbound.append(dim_bounds.LowBound)
            o_ubound.append(dim_bounds.UpBound)
        o_lbound = np.array(o_lbound)
        o_ubound = np.array(o_ubound)

        a_lbound = []
        a_ubound = []
        for dim_bounds in c_bounds().bounds:
            a_lbound.append(dim_bounds.LowBound)
            a_ubound.append(dim_bounds.UpBound)
        a_lbound = np.array(a_lbound)
        a_ubound = np.array(a_ubound)

        self.observation_space = spaces.Box(o_lbound, o_ubound)
        self.action_space = spaces.Box(a_lbound, a_ubound)

    def target_state_callback(self, msg):
        try:
            assert np.array(msg.target_state).shape == self.observation_space.shape
            self.target_state = np.array(msg.target_state)
        except AssertionError:
            print('AssertionError: target_state does not match observation '
                  'space dimension')

    def experience_callback(self, msg):
        # put incoming messages into experience queue
        q = self.experience_queue
        if q.full():
            q.get()
        t = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
        state = np.array(msg.state_data)
        cmd = np.array(msg.command_data)
        q.put((t, state, cmd))

    def wait_for_state(self, dt=None):
        if dt is None:
            dt = self.dt
        t1 = self.t + dt
        t = self.t
        state = []
        cmd = []

        while t < t1:
            if self.experience_queue.empty():
                # sleep for a short time
                rospy.sleep(0.1*dt)
            else:
                t, state, cmd = self.experience_queue.get(
                    timeout=0.01*dt)
        return t, state, cmd

    def apply_control(self, u):
        '''
            publish control message. We send the
        '''
        self.u = u
        msg = ExperienceData()
        msg.header.stamp = rospy.Time()
        # we fill the state msg for logging purposes, the topics to vector
        # node ignores this information.
        msg.state_data = self.state.tolist()
        msg.command_data = u.tolist()
        self.command_pub.publish(msg)

    def _step(self, action):
        # apply action and return state dt seconds after sending command
        # For control tasks, the robot driver should be responsible to decide
        # whether to use zero-order hold (constant command during dt) or
        # any other scheme.
        info = {}
        curr_state = self.state

        # first apply control
        self.apply_control(action)

        # step for dt seconds
        t, state, cmd = self.wait_for_state(self.dt)

        # save latest measurement info
        self.state = state
        self.t = t
        info['t'] = self.t
        info['action'] = action

        # evaluate cost, if given
        # CHANGED: from cost = None to reward = 0.0
        #          for compatibility with SAC codebase
        #          Hence the default reward will be 0.0
        reward = 0.0
        if self.loss_func is not None:
            reward = self.loss_func(
                                    angles2vector(self.state, self.state_units),
                                    angles2vector(self.target_state, self.state_units)
                                    )

        self.logger.dump({'curr_state': curr_state,
                          'action': action,
                          'reward': reward,
                          'next_state': self.state
                          })
        # return output following the openai gym convention
        return self.state, reward, False, info

    def _reset(self):
        '''
            calls the registered reset service with the desired state.
        '''
        self.reset_srv()
        # init time
        self.t = rospy.get_time()
        self.t, self.state, cmd = self.wait_for_state(dt=0.1*self.dt)

        self.logger.log_reset()

        return self.state

    def _close(self):
        '''
            class any registered service with empty messages
        '''
        pass

    def stop(self):
        '''
            calls the registered reset service with the desired state.
        '''
        self.stop_srv()
