# -*- coding: utf-8 -*-
"""Joint module for URDF generation from Fusion 360.

This module provides classes and functions to extract joint information from
Fusion 360 components and convert them to URDF joint representations.

Created on Sun May 12 20:17:17 2019
@author: syuntoku
@modified by: haoyan.li
"""

from enum import Enum
from typing import Any, Optional
from xml.etree.ElementTree import Element, SubElement

# pyright: reportMissingImports=false
import adsk
import adsk.core
import adsk.fusion

from ..utils import convert_occ_name, utils
from ..utils.math_utils import Transform


class Joint:
    """Represents a URDF joint with all necessary properties.

    This class encapsulates joint information including type, origin transformation,
    axis direction, parent/child links, and joint limits. It provides methods to
    generate URDF XML representations for both joints and transmissions.

    Attributes:
        name: Name of the joint
        type: Type of joint (revolute, continuous, prismatic, fixed)
        origin: Transformation from parent link to child joint origin
        axis: Axis of rotation/translation for the joint
        parent: Name of the parent link
        child: Name of the child link
        upper_limit: Upper limit for joint motion (radians for revolute, meters for prismatic)
        lower_limit: Lower limit for joint motion (radians for revolute, meters for prismatic)
        xyz: Translation component of origin transformation
        rpy: Rotation component of origin transformation (roll, pitch, yaw)
        joint_xml: Generated URDF XML for the joint (set by make_joint_xml)
        tran_xml: Generated URDF XML for the transmission (set by make_transmission_xml)
    """

    def __init__(
        self,
        name: str,
        origin: Transform,
        axis: list[float],
        parent: str,
        child: str,
        joint_type: str,
        upper_limit: float,
        lower_limit: float,
    ) -> None:
        """Initialize a Joint instance.

        Args:
            name: Name of the joint
            origin: Transformation from parent link to child joint origin
            axis: Direction vector [x, y, z] for joint axis
            parent: Name of the parent link
            child: Name of the child link
            joint_type: Type of joint (revolute, continuous, prismatic, fixed)
            upper_limit: Upper motion limit (radians/meters depending on joint type)
            lower_limit: Lower motion limit (radians/meters depending on joint type)
        """
        self.name: str = name
        self.type: str = joint_type
        self.origin: Transform = origin
        self.parent: str = parent
        self.child: str = child
        self.joint_xml: Optional[str] = None
        self.tran_xml: Optional[str] = None
        self.axis: list[float] = axis  # for 'revolute' and 'continuous'
        self.upper_limit: float = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit: float = lower_limit  # for 'revolute' and 'prismatic'

        self.xyz: list[float] = origin.translation
        self.rpy: list[float] = origin.rotation

    def make_joint_xml(self) -> list[str]:
        """Generate URDF XML representation of the joint.

        Creates a complete URDF joint element with origin, parent/child links,
        axis (for non-fixed joints), and limits (for revolute/prismatic joints).
        The generated XML is stored in self.joint_xml and returned.

        Returns:
            list[str]: Complete URDF XML string for the joint

        Note:
            The XML includes effort and velocity limits set to default values of 100.
        """
        joint = Element("joint")
        joint.attrib = {"name": self.name, "type": self.type}

        origin = SubElement(joint, "origin")
        origin.attrib = {
            "xyz": " ".join([str(round(el, 6)) for el in self.xyz]),
            "rpy": " ".join([str(round(el, 6)) for el in self.rpy]),
        }
        parent = SubElement(joint, "parent")
        parent.attrib = {"link": self.parent}
        child = SubElement(joint, "child")
        child.attrib = {"link": self.child}
        if (
            self.type == "revolute"
            or self.type == "continuous"
            or self.type == "prismatic"
        ):
            axis = SubElement(joint, "axis")
            axis.attrib = {"xyz": " ".join([str(round(el, 6)) for el in self.axis])}
        if self.type == "revolute" or self.type == "prismatic":
            limit = SubElement(joint, "limit")
            limit.attrib = {
                "upper": str(round(self.upper_limit, 6)),
                "lower": str(round(self.lower_limit, 6)),
                "effort": "100",
                "velocity": "100",
            }
        if self.type == "fixed":
            # fixed joint does not have axis and limit
            pass

        lines = utils.prettify(joint).split("\n")[1:]
        self.joint_xml = "\n".join(lines)

        return lines

    def make_transmission_xml(self) -> list[str]:
        """Generate URDF XML representation of the joint transmission.

        Creates a transmission element for use with ros_control, defining the
        interface between the joint and its actuator. Uses SimpleTransmission
        with EffortJointInterface and a mechanical reduction of 1:1.

        Returns:
            list[str]: Complete URDF XML string for the transmission

        Note:
            - Transmission type: transmission_interface/SimpleTransmission
            - Hardware interface: hardware_interface/EffortJointInterface
            - Mechanical reduction: 1 (no reduction)
        """

        tran = Element("transmission")
        tran.attrib = {"name": self.name + "_tran"}

        joint_type = SubElement(tran, "type")
        joint_type.text = "transmission_interface/SimpleTransmission"

        joint = SubElement(tran, "joint")
        joint.attrib = {"name": self.name}
        hardwareInterface_joint = SubElement(joint, "hardwareInterface")
        hardwareInterface_joint.text = "hardware_interface/EffortJointInterface"

        actuator = SubElement(tran, "actuator")
        actuator.attrib = {"name": self.name + "_actr"}
        hardwareInterface_actr = SubElement(actuator, "hardwareInterface")
        hardwareInterface_actr.text = "hardware_interface/EffortJointInterface"
        mechanicalReduction = SubElement(actuator, "mechanicalReduction")
        mechanicalReduction.text = "1"

        lines = utils.prettify(tran).split("\n")[1:]
        self.tran_xml = "\n".join(lines)

        return lines


class JointTypes(Enum):
    """Enumeration of Fusion 360 joint types.

    Maps Fusion 360 joint type constants to their corresponding integer values.
    Used to identify and process different joint types during URDF conversion.

    Note:
        Only RIGID, REVOLUTE, and SLIDER joints are currently supported
        for URDF export. Other joint types will raise an error.
    """

    RIGID = 0
    REVOLUTE = 1
    SLIDER = 2
    CYLINDRICAL = 3
    PIN_SLOT = 4
    PLANAR = 5
    BALL = 6
    INFERRED = 7


def make_joints(
    root: adsk.fusion.Component,
) -> dict[str, Joint]:
    """
    Extracts joint information from a Fusion 360 component and constructs a dictionary of Joint objects suitable for URDF export.
    Args:
        root (adsk.fusion.Component): The root Fusion 360 component containing all joints.
    Returns:
        dict[str, Joint]: A dictionary mapping joint names to Joint objects, each containing origin, axis, parent/child links, joint type, and limits.
    Raises:
        ValueError: If joint origins are not properly set, if joint axis/direction is not set, if angle limits are incomplete, or if an unsupported joint type is encountered.
    Notes:
        - Only revolute, prismatic (slider), and rigid joints are processed. Rigid joints are skipped.
        - Joint origins and axes are extracted and transformed to the parent link frame.
        - Joint limits are rounded to six decimal places.
    """

    def get_axis_from_direction(
        direction: int, joint_name: str, axis_type: str
    ) -> list[float]:
        """Convert direction index to axis vector."""
        if direction == 0:  # X axis
            return [1, 0, 0]
        elif direction == 1:  # Y axis
            return [0, 1, 0]
        elif direction == 2:  # Z axis
            return [0, 0, 1]
        else:
            raise ValueError(
                f"{joint_name} has no {axis_type} axis set. Please set it and try again."
            )

    def process_revolute_joint(
        motion: Any, fusion_joint_name: str
    ) -> tuple[str, list[float], float, float]:
        """Process revolute joint and return type, axis, and limits."""
        axis = get_axis_from_direction(
            motion.rotationAxis, fusion_joint_name, "rotation"
        )

        if (
            motion.rotationLimits.isMaximumValueEnabled
            and motion.rotationLimits.isMinimumValueEnabled
        ):
            joint_type = "revolute"
            upper_limit = round(motion.rotationLimits.maximumValue, 6)
            lower_limit = round(motion.rotationLimits.minimumValue, 6)
        elif (
            not motion.rotationLimits.isMaximumValueEnabled
            and not motion.rotationLimits.isMinimumValueEnabled
        ):
            joint_type = "continuous"
            upper_limit = 0.0
            lower_limit = 0.0
        else:
            raise ValueError(
                f"{fusion_joint_name} has incomplete angle limits. Please set both or neither."
            )

        return joint_type, axis, upper_limit, lower_limit

    def process_prismatic_joint(
        motion: Any, fusion_joint_name: str
    ) -> tuple[str, list[float], float, float]:
        """Process prismatic joint and return type, axis, and limits."""
        axis = get_axis_from_direction(
            motion.slideDirection, fusion_joint_name, "slide direction"
        )

        upper_limit = 0.0
        lower_limit = 0.0
        if (
            motion.slideLimits.isMaximumValueEnabled
            and motion.slideLimits.isMinimumValueEnabled
        ):
            upper_limit = round(motion.slideLimits.maximumValue, 6)
            lower_limit = round(motion.slideLimits.minimumValue, 6)

        return "prismatic", axis, upper_limit, lower_limit

    all_joints: list[adsk.fusion.Joint] = root.allJoints
    joints: dict[str, Joint] = {}

    for fusion_joint in all_joints:
        child_occ = fusion_joint.occurrenceOne
        parent_occ = fusion_joint.occurrenceTwo

        child_joint_origin = fusion_joint.geometryOrOriginOne
        parent_joint_origin = fusion_joint.geometryOrOriginTwo

        if not isinstance(
            child_joint_origin, adsk.fusion.JointOrigin
        ) or not isinstance(parent_joint_origin, adsk.fusion.JointOrigin):
            raise ValueError(
                f"{fusion_joint.name} joint origins are not properly set. Please set them and try again."
            )

        motion: Any = fusion_joint.jointMotion
        fusion_joint_type = motion.jointType

        # Process rigid joints to generate fixed joints in URDF
        if fusion_joint_type == JointTypes.RIGID.value:
            joint_type = "fixed"
            axis = [0.0] * 3
            joint_upper_limit = 0.0
            joint_lower_limit = 0.0
        elif fusion_joint_type == JointTypes.REVOLUTE.value:
            # Process joint based on type
            joint_type, axis, joint_upper_limit, joint_lower_limit = (
                process_revolute_joint(motion, fusion_joint.name)
            )
        elif fusion_joint_type == JointTypes.SLIDER.value:
            joint_type, axis, joint_upper_limit, joint_lower_limit = (
                process_prismatic_joint(motion, fusion_joint.name)
            )
        else:
            raise ValueError(
                f"{fusion_joint.name} has unsupported joint type. Only Revolute, Rigid and Slider are supported."
            )

        # calculate the origin of the joint
        # wMpl: world to parent link
        wMpl = Transform.from_Matrix3D(parent_occ.transform2)
        # wMcl: world to child link
        wMcl = Transform.from_Matrix3D(child_occ.transform2)
        # clMcj: child link to child joint origin
        clMcj = Transform.from_Matrix3D(child_joint_origin.transform)
        # wMcj: world to child joint origin
        wMcj = wMcl * clMcj
        # origin = plMcj: parent link to child joint origin
        origin = wMpl.inverse() * wMcj

        parent_occ_name = convert_occ_name(parent_occ.name)
        child_occ_name = convert_occ_name(child_occ.name)

        joint = Joint(
            name=fusion_joint.name,
            origin=origin,
            axis=axis,
            parent=parent_occ_name,
            child=child_occ_name,
            joint_type=joint_type,
            upper_limit=joint_upper_limit,
            lower_limit=joint_lower_limit,
        )
        joints[joint.name] = joint

    return joints
