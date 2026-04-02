#!/usr/bin/env python3
from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rostest
import unittest

from geometry_msgs.msg import PoseStamped

from cse478 import utils
from cse478.collector import SynchronizedMessageCollector


class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        pass

    def _test_particle_filter(self):
        reference_pose_topic = "reference_pose"
        plot = bool(rospy.get_param("~plot", False))
        bag_name = rospy.get_param("~bag_name")

        collector = SynchronizedMessageCollector(
            [reference_pose_topic],
            [PoseStamped],
        )
        try:
            rospy.sleep(5)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # We expect time to jump back because the test restarts the bag
            pass
        msgs = collector.start(duration=20)
        self.assertGreaterEqual(
            len(msgs),
            50,
            msg="The test didn't receive enough messages to be able to compare "
            "the particle filter estimate with the ground truth.",
        )

        references = [utils.pose_to_particle(m[0].pose) for m in msgs]
        references = np.array(references)

        # Drop (0, 0, 0) estimates from pre-mature initialization
        premature = (np.abs(references) < 2e-1).all(axis=1)
        references = references[~premature, :]
        if plot:
            plt.xlabel("x")
            plt.ylabel("y")
            plt.plot(references[:, 0], references[:, 1], c="g", label="Car")
            plt.legend()
            plt.gca().set_aspect(aspect=1.0)
            plt.show()

        if bag_name != "full":
            return

if __name__ == "__main__":
    rospy.init_node("test_particle_filter")
    bag_name = rospy.get_param("~bag_name")
    # The xml report will use the method's name, so we have to manually
    # mangle it with the bag name to ensure all entries appear
    setattr(
        TestParticleFilter,
        "test_particle_filter_{}".format(bag_name),
        lambda self: self._test_particle_filter(),
    )
    rostest.rosrun(
        "localization", "test_particle_filter_{}".format(bag_name), TestParticleFilter
    )
