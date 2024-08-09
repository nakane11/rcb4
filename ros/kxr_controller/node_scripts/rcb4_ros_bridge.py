#!/usr/bin/env python3

import os
import os.path as osp
import shlex
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import actionlib
import geometry_msgs.msg
from kxr_controller.msg import ServoOnOffAction
from kxr_controller.msg import ServoOnOffResult
from kxr_controller.msg import Stretch
from kxr_controller.msg import StretchAction
from kxr_controller.msg import StretchResult
from hand_v3.msg import RawAngle
import numpy as np
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import JointState
import serial
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
import std_msgs.msg
import yaml

from rcb4.armh7interface import ARMH7Interface
from rcb4.rcb4interface import RCB4Interface, deg_to_servovector


np.set_printoptions(precision=0, suppress=True)


def load_yaml(file_path, Loader=yaml.SafeLoader):
    """Load a YAML file into a Python dict.

    Parameters
    ----------
    file_path : str or pathlib.PosixPath
        The path to the YAML file.

    Returns
    -------
    data : dict
        A dict with the loaded yaml data.
    """
    if not osp.exists(str(file_path)):
        raise OSError('{} not exists'.format(str(file_path)))
    with open(osp.expanduser(file_path), 'r') as f:
        data = yaml.load(f, Loader=Loader)
    data = data['joint_name_to_servo_id']
    joint_name_to_id = {}
    for name in data:
        if isinstance(data[name], int):
            joint_name_to_id[name] = data[name]
        else:
            joint_name_to_id[name] = data[name]['id']
    return joint_name_to_id, data


def make_urdf_file(joint_name_to_id):
    robot = ET.Element('robot', {
        'name': 'dummy_robot',
        'xmlns:xi': 'http://www.w3.org/2001/XInclude'
    })

    ET.SubElement(robot, 'link', name='base_link')
    previous_link_name = 'base_link'

    for joint_name in joint_name_to_id.keys():
        link_name = f"{joint_name}_link"
        ET.SubElement(robot, 'link', name=link_name)

        joint = ET.SubElement(robot, 'joint', name=joint_name,
                              type='continuous')
        ET.SubElement(joint, 'parent', link=previous_link_name)
        ET.SubElement(joint, 'child', link=link_name)

        previous_link_name = link_name

    tmp_urdf_file = tempfile.mktemp(suffix='.urdf')
    tree = ET.ElementTree(robot)
    tree.write(tmp_urdf_file, encoding='utf-8', xml_declaration=True)
    return tmp_urdf_file


def run_robot_state_publisher(namespace=None):
    command = f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun'
    command += " robot_state_publisher robot_state_publisher"
    if namespace is not None:
        command += f" _tf_prefix:={namespace}"
    command = shlex.split(command)
    process = subprocess.Popen(command)
    return process


def run_kxr_controller(namespace=None):
    command = f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun'
    command += " kxr_controller kxr_controller"
    command += ' __name=:kxr_controller'
    command = shlex.split(command)
    process = subprocess.Popen(command)
    return process


def set_initial_position(positions, namespace=None):
    rospy.set_param(namespace + '/initial_position',
                    positions)


def set_fullbody_controller(joint_names):
    controller_yaml_dict = {
        'type': 'position_controllers/JointTrajectoryController',
        'joints': joint_names,
    }
    rospy.set_param('kxr_fullbody_controller', controller_yaml_dict)


def set_joint_state_controler():
    rospy.set_param('joint_state_controller', {
        'type': 'joint_state_controller/JointStateController',
        'publish_rate': 10
    })


def set_robot_description(urdf_path,
                          param_name='robot_description'):
    with open(urdf_path) as f:
        rospy.set_param(param_name, f.read())


class RCB4ROSBridge(object):

    def __init__(self):
        self.proc_controller_spawner = None
        self.proc_robot_state_publisher = None
        self.proc_kxr_controller = None

        servo_config_path = rospy.get_param('~servo_config_path')
        self.joint_name_to_id, servo_infos = load_yaml(servo_config_path)

        r = RobotModel()
        urdf_path = rospy.get_param('~urdf_path', None)
        tmp_urdf = False
        if urdf_path is None:
            urdf_path = make_urdf_file(self.joint_name_to_id)
            tmp_urdf = True
            rospy.loginfo("Use temporary URDF")
        with open(urdf_path) as f:
            with no_mesh_load_mode():
                r.load_urdf_file(f)

        joint_list = [j for j in r.joint_list
                      if j.__class__.__name__ != 'FixedJoint']
        self.joint_names = [j.name for j in joint_list]

        full_namespace = rospy.get_namespace()
        last_slash_pos = full_namespace.rfind('/')
        clean_namespace = full_namespace[:last_slash_pos] \
            if last_slash_pos != 0 else ''
        self.clean_namespace = clean_namespace

        set_robot_description(
            urdf_path,
            param_name=clean_namespace + '/robot_description')
        if tmp_urdf:
            os.remove(urdf_path)

        set_joint_state_controler()
        self.current_joint_states_pub = rospy.Publisher(
            clean_namespace + '/current_joint_states',
            JointState,
            queue_size=1)

        self.interface = None
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('~device', None):
                    self.interface = ARMH7Interface.from_port(
                        rospy.get_param('~device', None))
                elif rospy.get_param('~use_rcb4'):
                    self.interface = RCB4Interface()
                    self.interface.auto_open()
                else:
                    # self.interface = ARMH7Interface.from_port()
                    self.interface = ARMH7Interface()
                break
            except serial.SerialException as e:
                rospy.logerr('Waiting for the port to become available. {}'
                             .format(e))
            rospy.sleep(1.0)
        if self.interface is None:
            rospy.logerr('Could not open port!')
            sys.exit(1)
        self._prev_velocity_command = None

        # set servo ids to rosparam
        # rospy.set_param(clean_namespace + '/servo_ids',
        #                 self.interface.search_servo_ids().tolist())

        wheel_servo_sorted_ids = []
        trim_vector_servo_ids = []
        trim_vector_offset = []
        self.futaba_joint_names = []
        for name, info in servo_infos.items():
            if isinstance(info, int):
                continue
            servo_id = info['id']
            direction = info.get('direction', 1)
            offset = info.get('offset', 0)
            if 'type' in info and info['type'] == 'continuous':
                wheel_servo_sorted_ids.append(servo_id)
            if 'product' in info and info['product'] == 'futaba':
                self.futaba_joint_names.append(name)
        #     idx = self.interface.servo_id_to_index(servo_id)
        #     if idx is None:
        #         continue
        #     self.interface._joint_to_actuator_matrix[idx, idx] = \
        #         direction * self.interface._joint_to_actuator_matrix[idx, idx]
        #     trim_vector_servo_ids.append(servo_id)
        #     trim_vector_offset.append(direction * offset)
        # if self.interface.__class__.__name__ != 'RCB4Interface':
        #     self.interface.trim_vector(
        #         trim_vector_offset, trim_vector_servo_ids)
        # if self.interface.wheel_servo_sorted_ids is None:
        #     self.interface.wheel_servo_sorted_ids = wheel_servo_sorted_ids

        self.set_fullbody_controller(clean_namespace)
        self.set_initial_positions(clean_namespace)
        # self.check_servo_states()

        rospy.loginfo('run kxr_controller')
        self.proc_kxr_controller = run_kxr_controller(
            namespace=clean_namespace)

        self.command_joint_state_sub = rospy.Subscriber(
            clean_namespace + '/command_joint_state',
            JointState, queue_size=1,
            callback=self.command_joint_state_callback)

        # self.velocity_command_joint_state_sub = rospy.Subscriber(
        #     clean_namespace + '/velocity_command_joint_state',
        #     JointState, queue_size=1,
        #     callback=self.velocity_command_joint_state_callback)

        self.servo_on_off_server = actionlib.SimpleActionServer(
            clean_namespace
            + '/kxr_fullbody_controller/servo_on_off_real_interface',
            ServoOnOffAction,
            execute_cb=self.servo_on_off_callback,
            auto_start=False)
        self.servo_on_off_server.start()

        # TODO(someone) support rcb-4 miniboard
        if not rospy.get_param('~use_rcb4'):
            self.stretch_server = actionlib.SimpleActionServer(
                clean_namespace
                + '/kxr_fullbody_controller/stretch_interface',
                StretchAction,
                execute_cb=self.stretch_callback,
                auto_start=False)
            self.stretch_server.start()
            self.stretch_publisher = rospy.Publisher(
                clean_namespace
                + '/kxr_fullbody_controller/stretch',
                Stretch,
                queue_size=1,
                latch=True)
            rospy.sleep(0.1)
            # self.publish_stretch()

        self.proc_controller_spawner = subprocess.Popen(
            [f'/opt/ros/{os.environ["ROS_DISTRO"]}/bin/rosrun',
             'controller_manager', 'spawner']
            + ['joint_state_controller', 'kxr_fullbody_controller'])
        self.proc_robot_state_publisher = run_robot_state_publisher(
            clean_namespace)

        self.publish_imu = rospy.get_param('~publish_imu', True)
        self.publish_sensor = rospy.get_param('~publish_sensor', False)
        self.publish_battery_voltage = rospy.get_param(
            '~publish_battery_voltage', True)
        if self.interface.__class__.__name__ == 'RCB4Interface':
            self.publish_imu = False
            self.publish_sensor = False
        if self.publish_imu:
            self.imu_frame_id = rospy.get_param(
                '~imu_frame_id', clean_namespace + '/' + r.root_link.name)
            self.imu_publisher = rospy.Publisher(
                clean_namespace + '/imu',
                sensor_msgs.msg.Imu,
                queue_size=1)
        if self.publish_sensor:
            self._sensor_publisher_dict = {}
        if self.publish_battery_voltage:
            self.battery_voltage_publisher = rospy.Publisher(
                clean_namespace + '/battery_voltage',
                std_msgs.msg.Float32,
                queue_size=1)
        self.kondo_pub = rospy.Publisher("/kondo/command_angle", RawAngle, queue_size=1)
        self.futaba_pub = rospy.Publisher("/futaba/command_angle", RawAngle, queue_size=1)

    def __del__(self):
        if self.proc_controller_spawner:
            self.proc_controller_spawner.kill()
        if self.proc_robot_state_publisher:
            self.proc_robot_state_publisher.kill()
        if self.proc_kxr_controller:
            self.proc_kxr_controller.kill()

    def unsubscribe(self):
        self.command_joint_state_sub.unregister()
        self.velocity_command_joint_state_sub.unregister()

    def check_servo_states(self):
        self.joint_servo_on = {jn: False for jn in self.joint_names}
        servo_on_states = self.interface.servo_states()
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            idx = self.joint_name_to_id[jn]
            if idx in servo_on_states:
                self.joint_servo_on[jn] = True
            else:
                self.joint_servo_on[jn] = False

    def set_fullbody_controller(self, clean_namespace):
        self.fullbody_jointnames = []
        for jn in self.joint_names:
            if jn not in self.joint_name_to_id:
                continue
            servo_id = self.joint_name_to_id[jn]
            # if servo_id in self.interface.wheel_servo_sorted_ids:
            #     continue
            self.fullbody_jointnames.append(jn)
        set_fullbody_controller(self.fullbody_jointnames)

    def set_initial_positions(self, clean_namespace):
        initial_positions = {}
        init_av = [-60,-60,-60,60,-50,60,67,0,0,0]
        joint_names = ['THUMB_JOINT0',
                'THUMB_JOINT1',
                'INDEX_JOINT',
                'MIDDLE_JOINT',
                'RING_JOINT',
                'LITTLE_JOINT',
                'ADDUCTION_JOINT',
                'RARM_JOINT0',
                'RARM_JOINT1',
                'RARM_JOINT2']
        for av, jn in zip(init_av, joint_names):
            initial_positions[jn] = float(np.deg2rad(av))
        # # init_av = self.interface.angle_vector()
        # for jn in self.joint_names:
        #     if jn not in self.joint_name_to_id:
        #         continue        #     servo_id = self.joint_name_to_id[jn]
        #     # if servo_id in self.interface.wheel_servo_sorted_ids:
        #     #     continue
        #     idx = self.interface.servo_id_to_index(servo_id)
        #     if idx is None:
        #         continue
        #     initial_positions[jn] = float(
        #         np.deg2rad(init_av[idx]))
        set_initial_position(initial_positions, namespace=clean_namespace)

    def _msg_to_angle_vector_and_servo_ids(
            self, msg,
            velocity_control=False):
        used_servo_id = {}
        kondo_servo_ids = []
        kondo_angle_vector = []
        futaba_servo_ids = []
        futaba_angle_vector = []
        for name, angle in zip(msg.name, msg.position):
            if name not in self.joint_name_to_id: # or \
               # (name in self.joint_servo_on and not self.joint_servo_on[name]):
                continue
            idx = self.joint_name_to_id[name]
            if name in self.futaba_joint_names:
                futaba_angle_vector.append(np.rad2deg(angle))
                futaba_servo_ids.append(idx)
                continue
            # if velocity_control:
            #     if idx not in self.interface.wheel_servo_sorted_ids:
            #         continue
            # else:
            #     if idx in self.interface.wheel_servo_sorted_ids:
            #         continue
            # should ignore duplicated index.
            if idx in used_servo_id:
                continue
            used_servo_id[idx] = True
            kondo_angle_vector.append(7500+deg_to_servovector*np.rad2deg(angle))
            kondo_servo_ids.append(idx)
        futaba_angle_vector = np.array(futaba_angle_vector, dtype=np.int32)
        futaba_servo_ids = np.array(futaba_servo_ids, dtype=np.int32)
        kondo_angle_vector = np.array(kondo_angle_vector, dtype=np.int32)
        kondo_servo_ids = np.array(kondo_servo_ids, dtype=np.int32)
        # valid_indices = self.interface.valid_servo_ids(kondo_servo_ids)
        return kondo_angle_vector, kondo_servo_ids,\
            futaba_angle_vector, futaba_servo_ids

    def velocity_command_joint_state_callback(self, msg):
        av, servo_ids = self._msg_to_angle_vector_and_servo_ids(
            msg, velocity_control=True)
        if len(av) == 0:
            return
        if self._prev_velocity_command is not None and np.allclose(
                self._prev_velocity_command, av):
            return
        try:
            self.interface.angle_vector(av, servo_ids, velocity=0)
            self._prev_velocity_command = av
        except serial.serialutil.SerialException as e:
            rospy.logerr('[velocity_command_joint] {}'.format(str(e)))

    def command_joint_state_callback(self, msg):
        kondo_av, kondo_servo_ids, futaba_av, futaba_servo_ids\
            = self._msg_to_angle_vector_and_servo_ids(
                msg, velocity_control=False)
        if len(kondo_av) != 0:
            kondo_msg = RawAngle()
            kondo_msg.angle = kondo_av
            kondo_msg.id = list(kondo_servo_ids)
            kondo_msg.length = len(kondo_av)
            self.kondo_pub.publish(kondo_msg)
            # try:
            #     self.interface.angle_vector(kondo_av, kondo_servo_ids, velocity=1)
            # except serial.serialutil.SerialException as e:
            #     rospy.logerr('[command_joint] {}'.format(str(e)))
        if len(futaba_av) != 0:
            futaba_msg = RawAngle()
            futaba_msg.angle = futaba_av
            futaba_msg.id = list(futaba_servo_ids)
            futaba_msg.length = len(futaba_av)
            self.futaba_pub.publish(futaba_msg)

    def servo_on_off_callback(self, goal):
        servo_vector = []
        servo_ids = []
        for joint_name, servo_on in zip(
                goal.joint_names, goal.servo_on_states):
            if joint_name not in self.joint_name_to_id:
                continue
            servo_ids.append(self.joint_name_to_id[joint_name])
            if servo_on:
                servo_vector.append(32767)
                self.joint_servo_on[joint_name] = True
            else:
                servo_vector.append(32768)
                self.joint_servo_on[joint_name] = False
        try:
            self.interface.servo_angle_vector(
                servo_ids, servo_vector, velocity=1)
        except RuntimeError as e:
            self.unsubscribe()
            rospy.signal_shutdown('Disconnected {}.'.format(e))
        except serial.serialutil.SerialException as e:
            rospy.logerr('[servo_on_off] {}'.format(str(e)))
        return self.servo_on_off_server.set_succeeded(ServoOnOffResult())

    def publish_stretch(self):
        # Get current stretch of all servo motors and publish them
        joint_names = []
        servo_ids = []
        for joint_name in self.joint_names:
            if joint_name not in self.joint_name_to_id:
                continue
            joint_names.append(joint_name)
            servo_ids.append(self.joint_name_to_id[joint_name])
        try:
            stretch = self.interface.read_stretch(servo_ids=servo_ids)
        except serial.serialutil.SerialException as e:
            rospy.logerr('[read_stretch] {}'.format(str(e)))
        stretch_msg = Stretch(
            joint_names=joint_names,
            stretch=stretch
        )
        self.stretch_publisher.publish(stretch_msg)

    def stretch_callback(self, goal):
        if len(goal.joint_names) == 0:
            goal.joint_names = self.joint_names
        joint_names = []
        servo_ids = []
        for joint_name in goal.joint_names:
            if joint_name not in self.joint_name_to_id:
                continue
            joint_names.append(joint_name)
            servo_ids.append(self.joint_name_to_id[joint_name])
        # Send new stretch
        stretch = goal.stretch
        try:
            self.interface.send_stretch(
                value=stretch, servo_ids=servo_ids)
        except serial.serialutil.SerialException as e:
            rospy.logerr('[send_stretch] {}'.format(str(e)))
        # Return result
        self.publish_stretch()
        rospy.loginfo(
            "Update {} stretch to {}".format(
                joint_names, stretch))
        return self.stretch_server.set_succeeded(StretchResult())

    def publish_imu_message(self):
        msg = sensor_msgs.msg.Imu()
        msg.header.frame_id = self.imu_frame_id
        msg.header.stamp = rospy.Time.now()
        try:
            q_wxyz, acc, gyro = self.interface.read_imu_data()
        except serial.serialutil.SerialException as e:
            rospy.logerr('[publish_imu] {}'.format(str(e)))
            return
        (msg.orientation.w, msg.orientation.x,
         msg.orientation.y, msg.orientation.z) = q_wxyz
        (msg.angular_velocity.x, msg.angular_velocity.y,
         msg.angular_velocity.z) = gyro
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = acc
        self.imu_publisher.publish(msg)

    def publish_sensor_values(self):
        stamp = rospy.Time.now()
        msg = geometry_msgs.msg.WrenchStamped()
        msg.header.stamp = stamp
        try:
            sensors = self.interface.all_jointbase_sensors()
        except serial.serialutil.SerialException as e:
            rospy.logerr('[publish_sensor_values] {}'.format(str(e)))
            return
        for sensor in sensors:
            for i in range(4):
                for typ in ['proximity', 'force']:
                    key = 'kjs_{}_{}_{}'.format(sensor.id, typ, i)
                    if typ == 'proximity':
                        msg.wrench.force.x = sensor.ps[i]
                    elif typ == 'force':
                        msg.wrench.force.x = sensor.adc[i]
                    if key not in self._sensor_publisher_dict:
                        self._sensor_publisher_dict[key] = rospy.Publisher(
                            self.clean_namespace
                            + '/kjs/{}/{}/{}'.format(sensor.id, typ, i),
                            geometry_msgs.msg.WrenchStamped,
                            queue_size=1)
                    msg.header.frame_id = 'kjs_{}_{}_frame'.format(
                        sensor.id, i)
                    self._sensor_publisher_dict[key].publish(msg)

    def publish_battery_voltage_value(self):
        try:
            volt = self.interface.battery_voltage()
        except serial.serialutil.SerialException as e:
            rospy.logerr('[publish_battery_voltage] {}'.format(str(e)))
            return
        self.battery_voltage_publisher.publish(
            std_msgs.msg.Float32(
                data=volt))

    def publish_joint_states(self):
        try:
            av = self.interface.angle_vector()
            torque_vector = self.interface.servo_error()
        except IndexError as e:
            rospy.logerr('[publish_joint_states] {}'.format(str(e)))
            return
        except serial.serialutil.SerialException as e:
            rospy.logerr('[publish_joint_states] {}'.format(str(e)))
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for name in self.joint_names:
            if name in self.joint_name_to_id:
                servo_id = self.joint_name_to_id[name]
                idx = self.interface.servo_id_to_index(servo_id)
                if idx is None:
                    continue
                msg.position.append(np.deg2rad(av[idx]))
                msg.effort.append(torque_vector[idx])
                msg.name.append(name)
        self.current_joint_states_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(rospy.get_param(
            self.clean_namespace + '/control_loop_rate', 20))
        while not rospy.is_shutdown():
            # if self.interface.is_opened() is False:
            #     self.unsubscribe()
            #     rospy.signal_shutdown('Disconnected.')
            #     break
            # self.publish_joint_states()

            if self.publish_imu and self.imu_publisher.get_num_connections():
                self.publish_imu_message()
            if self.publish_sensor:
                self.publish_sensor_values()
            if self.publish_battery_voltage:
                self.publish_battery_voltage_value()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('rcb4_ros_bridge')
    ros_bridge = RCB4ROSBridge()  # NOQA
    ros_bridge.run()
