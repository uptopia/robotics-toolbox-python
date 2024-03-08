"""
@author: Gautam Sinha, Indian Institute of Technology, Kanpur
  (original MATLAB version)
@author: Peter Corke
@author: Samuel Drew
@author: Sherry Wong
"""

from roboticstoolbox import DHRobot, RevoluteDH
from math import pi
import numpy as np


class TimdaSingle7(DHRobot):
    """
    Class that models a ICLAB TIMDA 7-DoF (7-DoF+no Slide) manipulator

    ``TimdaSingle7()`` is a class which models a ICLAB TIMDA robot and
    describes its kinematic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.TimdaSingle7()
        >>> print(robot)

    Defined joint configurations are:

      - qk1, nominal working position 1 (TODO)
      - qz, zero joint angle configuration, 'L' shaped configuration
      - qr, vertical 'READY' configuration
      - qs, arm is stretched out in the x-direction
      - qn, arm is at a nominal non-singular configuration

    .. note::
      - SI units of metres are used.

    :references:
      - https://github.com/tku-iarc/timda_dual_arm

    .. codeauthor:: 
    """  # noqa

    def __init__(self):
        deg = pi / 180

        L1 = RevoluteDH(
            a=0.0,          # link length (Dennavit-Hartenberg notation)
            d=0.2463,       # link offset (Dennavit-Hartenberg notation)
            alpha=pi / 2,   # link twist (Dennavit-Hartenberg notation)
            qlim=[-180 * deg, 180 * deg]    # minimum and maximum joint angle
        )
        L2 = RevoluteDH(
            a=0.0, 
            d=0.0, 
            alpha=pi / 2,
            qlim=[0 * deg, 180 * deg]
        )
        L3 = RevoluteDH(
            a=-0.03,
            d=0.29,
            alpha=-pi / 2,
            qlim=[-180 * deg, 180 * deg],
        )
        L4 = RevoluteDH(
            a=0.03,
            d=0.0,
            alpha=-pi / 2,
            qlim=[-180 * deg, 0 * deg],
        )
        L5 = RevoluteDH(
            a=0.0, 
            d=0.27, 
            alpha=-pi / 2, 
            qlim=[-180 * deg, 180 * deg]
        )
        L6 = RevoluteDH(
            a=0.0, 
            d=0.0, 
            alpha=pi / 2,
            qlim=[-100 * deg, 100 * deg]
        )
        L7 = RevoluteDH(
            a=0.0, 
            d=0.24, 
            alpha=0.0, 
            qlim=[-180 * deg, 180 * deg]
        )

        L = [L1, L2, L3, L4, L5, L6, L7]

        # Create SerialLink object
        super().__init__(
            L,
            name="TimdaSingle7",
            manufacturer="ICLAB",
            # meshdir="meshes/ICLAB/TimdaSingle7",
        )

        self.qr = np.array([pi / 2, pi / 2, pi, pi, 0.0, 0.0, 0.0])

        self.addconfiguration("qr", self.qr)


if __name__ == "__main__":  # pragma nocover
    robot = TimdaSingle7()
    print(robot)
