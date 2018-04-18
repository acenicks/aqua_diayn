#!/usr/bin/env python

import rospy
import roslib
import message_filters
from aqua_diayn.msg import EnvDimBounds
from aqua_diayn.srv import *

import sys
from numbers import Number
from collections import OrderedDict

state_bounds_srv_name = '/rl/state_bounds'
command_bounds_srv_name = '/rl/command_bounds'


# load_package and load message are shamelessly copied from the
# rosserial_python SerialClient code
def load_package(package_name, directory):
    # check if its in the python path
    in_path = False
    path = sys.path
    # check for the source directory which
    # is added to path by roslib boostrapping
    pkg_src = package_name+'/src'
    for entry in sys.path:
        if pkg_src in entry:
            in_path = True
    if not in_path:
        roslib.load_manifest(package_name)
    try:
        package_module = __import__(package_name + '.' + directory)
    except:
        rospy.logerr("Cannot import package : %s" % package_name)
        rospy.logerr("sys.path was " + str(path))
        return None
    return package_module


def load_message(package_name, message_name):
    package_module = load_package(package_name, 'msg')
    msg_submodule = getattr(package_module, 'msg')
    return getattr(msg_submodule, message_name)
# end of stolen code


def recursive_getattr(obj, attr):
    if '.' not in attr and '[' not in attr:
        return getattr(obj, attr)
    elif '.' not in attr and '[' in attr:
        attr_data = attr.split('[', 1)
        index = int(attr_data[1].split(']', 1)[0])
        attr_list = getattr(obj, attr_data[0])
        # return eval('attr_list['+index+']')
        return attr_list[index]
    else:
        attr_data = attr.split('.', 1)
        return recursive_getattr(getattr(obj, attr_data[0]), attr_data[1])


def get_all_numeric_fields(msg, ignore_header=True):
    # if we have a number type, return a List with a single element
    if isinstance(msg, Number):
        return [msg]
    # else if it is already a list or tuple, return it
    elif (type(msg) is tuple) or (type(msg) is list):
        return msg
    # else, recurse, append to List, and returnd that List
    else:
        return_val = []
        for field_name in msg.__slots__:
            return_val.extend(
                get_all_numeric_fields(getattr(msg, field_name)))
        return return_val


# TODO: reduce code replication in get_state_bounds() and get_command_bounds()
def get_state_bounds(req):
    state_topics_list = rospy.get_param("~experience_state_topics", [])

    topic_types = OrderedDict()
    state_subscribers = OrderedDict()
    filtered_fields = OrderedDict()
    topic_bounds = OrderedDict()

    for topic in state_topics_list:
        topic_name = topic['topic_name']
        message_type = topic['type']
        m = load_message(message_type['package'], message_type['name'])

        state_subscribers[topic_name] = message_filters.Subscriber(
            topic_name, m)

        topic_types[topic_name] = m
        filtered_fields[topic_name] = topic.get('filter', m.__slots__)
        topic_bounds[topic_name] = topic.get('bounds')

    state_bounds = []
    for topic in state_subscribers:
        msg = topic_types[topic]()
        for field, field_bounds in zip(filtered_fields[topic], topic_bounds[topic]):
            field_value = recursive_getattr(msg, field)
            dim_bounds = EnvDimBounds()
            dim_bounds.LowBound = field_bounds[0]
            dim_bounds.UpBound = field_bounds[1]
            if not (hasattr(field_value, '_type')
                    and field_value._type == 'std_msgs/Header'):
                numeric_fields = get_all_numeric_fields(field_value)
                for dim in range(len(numeric_fields)):
                    state_bounds.append(dim_bounds)

    return EnvSpaceBoundsResponse(state_bounds)


def get_command_bounds(req):
    command_topics_list = rospy.get_param("~experience_command_topics", [])

    topic_types = OrderedDict()
    command_subscribers = OrderedDict()
    filtered_fields = OrderedDict()
    topic_bounds = OrderedDict()

    for topic in command_topics_list:
        topic_name = topic['topic_name']
        message_type = topic['type']
        m = load_message(message_type['package'], message_type['name'])

        command_subscribers[topic_name] = message_filters.Subscriber(
            topic_name, m)

        topic_types[topic_name] = m
        filtered_fields[topic_name] = topic.get('filter', m.__slots__)
        topic_bounds[topic_name] = topic.get('bounds')

    command_bounds = []
    for topic in command_subscribers:
        msg = topic_types[topic]()
        for field, field_bounds in zip(filtered_fields[topic], topic_bounds[topic]):
            field_value = recursive_getattr(msg, field)
            dim_bounds = EnvDimBounds()
            dim_bounds.LowBound = field_bounds[0]
            dim_bounds.UpBound = field_bounds[1]
            if not (hasattr(field_value, '_type')
                    and field_value._type == 'std_msgs/Header'):
                numeric_fields = get_all_numeric_fields(field_value)
                for dim in range(len(numeric_fields)):
                    command_bounds.append(dim_bounds)

    return EnvSpaceBoundsResponse(command_bounds)


if __name__ == "__main__":
    rospy.init_node('env_spaces')

    state_bounds_srv = rospy.Service(state_bounds_srv_name, EnvSpaceBounds, get_state_bounds)
    command_bounds_srv = rospy.Service(command_bounds_srv_name, EnvSpaceBounds, get_command_bounds)
    rospy.spin()
