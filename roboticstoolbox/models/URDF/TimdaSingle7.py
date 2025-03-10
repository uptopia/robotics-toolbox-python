#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3


class TimdaSingle7(Robot):
    """
    Class that imports a TimdaSingle7 URDF model

    ``TimdaSingle7()`` is a class which imports a ICLAB TimdaSingle7 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.TimdaSingle7()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "timda_single7_description/urdf/single_arm.xacro"
        )

        print("links", links)
        print("name", name)
        print("urdf_string", urdf_string)
        print("urdf_filepath", urdf_filepath)

        super().__init__(
            links,
            name="TimdaSingle7",
            manufacturer="ICLAB",
            # gripper_links=links[9],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        # self.grippers[0].tool = SE3(0, 0, 0.1034)

        # self.qdlim = np.array(
        #     [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0]
        # )

        # self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])
        # self.qz = np.zeros(7)

        # self.addconfiguration("qr", self.qr)
        # self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    r = TimdaSingle7()

    # r.qz

    # for link in r.grippers[0].links:
    #     print(link)
