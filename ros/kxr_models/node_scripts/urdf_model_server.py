#!/usr/bin/env python3

import os
import tempfile
import xml.etree.ElementTree as ET

from filelock import FileLock
from kxr_models.md5sum_utils import checksum_md5
from kxr_models.urdf import aggregate_urdf_mesh_files
import rospkg
import rospy


class URDFModelServer(object):

    def __init__(self):
        full_namespace = rospy.get_namespace()
        last_slash_pos = full_namespace.rfind('/')
        self.clean_namespace = full_namespace[:last_slash_pos] \
            if last_slash_pos != 0 else ''

    def run(self):
        rate = rospy.Rate(1)

        rospack = rospkg.RosPack()
        kxr_models_path = rospack.get_path('kxr_models')

        while not rospy.is_shutdown():
            rate.sleep()
            urdf = rospy.get_param(
                self.clean_namespace + '/robot_description',
                None)
            if urdf is not None:
                urdf_path = tempfile.mktemp()
                rospy.loginfo('Create tmp urdf file {}'.format(urdf_path))
                with open(urdf_path, "w") as f:
                    f.write(urdf)
                md5sum = checksum_md5(urdf_path)
                compressed_urdf_path = os.path.join(
                    kxr_models_path, 'models', 'urdf',
                    '{}.tar.gz'.format(md5sum))
                if os.path.exists(compressed_urdf_path):
                    parser = ET.XMLParser(
                        target=ET.TreeBuilder(insert_comments=True))
                    tree = ET.parse(urdf_path, parser)
                    root = tree.getroot()
                    robot_name = root.get('name')

                    rospy.set_param(self.clean_namespace + '/urdf_hash',
                                    md5sum)
                    with open(os.path.join(
                            kxr_models_path, 'models', 'urdf',
                            md5sum, '{}.urdf'.format(robot_name))) as f:
                        rospy.set_param(
                            self.clean_namespace + '/robot_description_viz',
                            f.read())
                    rospy.loginfo('Delete created urdf {}'.format(urdf_path))
                    if os.path.exists(urdf_path):
                        os.remove(urdf_path)
                    continue

                lock_path = compressed_urdf_path + ".lock"
                lock = FileLock(lock_path, timeout=10)
                try:
                    with lock:
                        aggregate_urdf_mesh_files(
                            urdf_path,
                            os.path.join(
                                kxr_models_path, 'models', 'urdf'),
                            compress=True)
                        rospy.loginfo(
                            'Compressed urdf model is saved to {}'
                            .format(compressed_urdf_path))
                    os.remove(lock_path)
                finally:
                    rospy.loginfo('Delete created urdf {}'.format(urdf_path))
                    if os.path.exists(urdf_path):
                        os.remove(urdf_path)


if __name__ == '__main__':
    rospy.init_node('urdf_model_server')
    server = URDFModelServer()
    server.run()
