# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

from enum import Enum
from typing import Tuple
from ..utils.math_utils import Transform
import adsk
import re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils, convert_occ_name


class Joint:
    def __init__(
        self, name, origin, axis, parent, child, joint_type, upper_limit, lower_limit
    ):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        transform: Transform
            transformation of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name: str = name
        self.type: str = joint_type
        self.origin: Transform = origin
        self.parent: str = parent
        self.child: str = child
        self.joint_xml: str = None
        self.tran_xml: str = None
        self.axis: list[float] = axis  # for 'revolute' and 'continuous'
        self.upper_limit: float = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit: float = lower_limit  # for 'revolute' and 'prismatic'

        self.xyz: list[float] = origin.translation
        self.rpy: list[float] = origin.rotation

    def make_joint_xml(self) -> str:
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element("joint")
        joint.attrib = {"name": self.name, "type": self.type}

        origin = SubElement(joint, "origin")
        origin.attrib = {
            "xyz": " ".join([str(_) for _ in self.xyz]),
            "rpy": " ".join([str(_) for _ in self.rpy]),
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
            axis.attrib = {"xyz": " ".join([str(_) for _ in self.axis])}
        if self.type == "revolute" or self.type == "prismatic":
            limit = SubElement(joint, "limit")
            limit.attrib = {
                "upper": str(self.upper_limit),
                "lower": str(self.lower_limit),
                "effort": "100",
                "velocity": "100",
            }

        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

        return self.joint_xml

    def make_transmission_xml(self) -> str:
        """
        Generate the tran_xml and hold it by self.tran_xml


        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface
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

        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])

        return self.tran_xml


class JointTypes(Enum):
    RIGID = 0
    REVOLUTE = 1
    SLIDER = 2
    CYLINDRICAL = 3
    PIN_SLOT = 4
    PLANAR = 5
    BALL = 6
    INFERRED = 7


def make_joints(root: adsk.fusion.Component) -> Tuple[list[Joint], bool, str]:
    """
    Collect all the joints in the design and make Joint instances for each joint
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status
    Returns
    ----------
    joints: [Joint, ...]
        List of Joint instances
    msg: str
        Tell the status
    Notes
    ----------
    Only Revolute, Rigid and Slider joints are supported for now.
    The axis of the joint must be set.
    The joint origin must be set for the second component of the joint.
    The first component of the joint must be JointGeometry and the second component must be JointOrigin.
    The unit of the angle limit is radian.
    The unit of the slide limit is cm.
    The unit of the translation of the joint origin is cm.
    The unit of the rotation of the joint origin is radian.
    The name of the parent and child link is the name of the component of the joint.
    The name of the component is replaced ":", " ", "(", ")" with "_".
    The name of the base component is "base_link".
    Only one joint is allowed to connect to the base component.
    The name of the joint is the name of the joint in Fusion 360.
    The name of the joint must be unique.
    The name of the joint is used as the name of the transmission.
    The name of the transmission is the name of the joint + "_tran".
    """

    joints = []

    for fusion_joint in root.joints:
        child_occ = fusion_joint.occurrenceOne
        parent_occ = fusion_joint.occurrenceTwo
        # child_geom = (
        #     fusion_joint.geometryOrOriginOne
        #     if type(fusion_joint.geometryOrOriginOne) == adsk.fusion.JointGeometry
        #     else fusion_joint.geometryOrOriginTwo.geometry
        # )
        # parent_geom = (
        #     fusion_joint.geometryOrOriginTwo
        #     if type(fusion_joint.geometryOrOriginTwo) == adsk.fusion.JointGeometry
        #     else fusion_joint.geometryOrOriginOne.geometry
        # )
        motion = fusion_joint.jointMotion
        fusion_joint_type = motion.jointType

        axis = [0, 0, 0]
        joint_upper_limit = 0.0
        joint_lower_limit = 0.0

        if fusion_joint_type == JointTypes.RIGID.value:
            joint_type = "fixed"
            continue  # ignore rigid joint
        elif fusion_joint_type == JointTypes.REVOLUTE.value:
            if (
                motion.rotationLimits.isMaximumValueEnabled
                and motion.rotationLimits.isMinimumValueEnabled
            ):
                joint_upper_limit = round(motion.rotationLimits.maximumValue, 6)
                joint_lower_limit = round(motion.rotationLimits.minimumValue, 6)

                joint_type = "revolute"
                if motion.rotationAxis == 0:  # X axis
                    axis = [1, 0, 0]
                elif motion.rotationAxis == 1:  # Y axis
                    axis = [0, 1, 0]
                elif motion.rotationAxis == 2:  # Z axis
                    axis = [0, 0, 1]
                else:
                    msg = (
                        fusion_joint.name
                        + " is not set its rotation axis. Please set it and try again."
                    )
                    return None, False, msg
            elif (
                not motion.rotationLimits.isMaximumValueEnabled
                and not motion.rotationLimits.isMinimumValueEnabled
            ):
                joint_type = "continuous"
                if motion.rotationAxis == 0:  # X axis
                    axis = [1, 0, 0]
                elif motion.rotationAxis == 1:  # Y axis
                    axis = [0, 1, 0]
                elif motion.rotationAxis == 2:  # Z axis
                    axis = [0, 0, 1]
                else:
                    msg = (
                        fusion_joint.name
                        + " is not set its rotation axis. Please set it and try again."
                    )
                    return None, False, msg
            else:
                msg = (
                    fusion_joint.name
                    + " is not set its angle limit. Please set it and try again."
                )
                return None, False, msg

        elif fusion_joint_type == JointTypes.SLIDER.value:
            joint_type = "prismatic"

        else:
            msg = (
                fusion_joint.name
                + " is not supported joint type. Only Revolute, Rigid and Slider are supported for now."
            )
            return None, msg

        # calculate the origin of the joint
        parent_tf = Transform.from_matrix(parent_occ.transform2.asArray())
        child_tf = Transform.from_matrix(child_occ.transform2.asArray())

        origin = parent_tf.inverse() * child_tf

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
        joints.append(joint)

        msg = "All the joints are successfully collected."

    return joints, True, msg


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints


    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    joints_dict:
        {name: {type, axis, upper_limit, lower_limit, parent, child, transform}, ...}
    msg: str
        Tell the status
    """

    joint_type_list = [
        "fixed",
        "revolute",
        "prismatic",
        "Cylinderical",
        "PinSlot",
        "Planner",
        "Ball",
    ]  # these are the names in urdf

    joints_dict = {}

    for joint in root.joints:
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict["type"] = joint_type

        # swhich by the type of the joint
        joint_dict["axis"] = [0, 0, 0]
        joint_dict["upper_limit"] = 0.0
        joint_dict["lower_limit"] = 0.0

        # support  "Revolute", "Rigid" and "Slider"
        if joint_type == "revolute":
            joint_motion_axis = joint.jointMotion.rotationAxis
            if joint_motion_axis == 0:  # X axis
                joint_dict["axis"] = [1, 0, 0]
            elif joint_motion_axis == 1:  # Y axis
                joint_dict["axis"] = [0, 1, 0]
            elif joint_motion_axis == 2:  # Z axis
                joint_dict["axis"] = [0, 0, 1]
            else:
                raise Exception("Unknown rotation axis")

            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled

            if max_enabled and min_enabled:
                joint_dict["upper_limit"] = round(
                    joint.jointMotion.rotationLimits.maximumValue, 6
                )
                joint_dict["lower_limit"] = round(
                    joint.jointMotion.rotationLimits.minimumValue, 6
                )
            elif max_enabled and not min_enabled:
                msg = (
                    joint.name
                    + "is not set its lower limit. Please set it and try again."
                )
                break
            elif not max_enabled and min_enabled:
                msg = (
                    joint.name
                    + "is not set its upper limit. Please set it and try again."
                )
                break
            else:  # if there is no angle limit
                joint_dict["type"] = "continuous"

        elif joint_type == "prismatic":
            joint_dict["axis"] = [
                round(i, 6) for i in joint.jointMotion.slideDirectionVector.asArray()
            ]  # Also normalized
            max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                joint_dict["upper_limit"] = round(
                    joint.jointMotion.slideLimits.maximumValue / 100, 6
                )
                joint_dict["lower_limit"] = round(
                    joint.jointMotion.slideLimits.minimumValue / 100, 6
                )
            elif max_enabled and not min_enabled:
                msg = (
                    joint.name
                    + "is not set its lower limit. Please set it and try again."
                )
                break
            elif not max_enabled and min_enabled:
                msg = (
                    joint.name
                    + "is not set its upper limit. Please set it and try again."
                )
                break
        elif joint_type == "fixed":
            pass

        if joint.occurrenceTwo.component.name == "base_link":
            joint_dict["parent"] = "base_link"
        else:
            joint_dict["parent"] = re.sub("[ :()]", "_", joint.occurrenceTwo.name)
        joint_dict["child"] = re.sub("[ :()]", "_", joint.occurrenceOne.name)

        # check joint one and two, only allow JointOrigin for two and JointGeometry for one
        if (
            type(joint.geometryOrOriginOne) != adsk.fusion.JointGeometry
            or type(joint.geometryOrOriginTwo) != adsk.fusion.JointOrigin
        ):
            msg = (
                joint.name + " doesn't have joint origin. Please set it and run again."
            )
            break

        oMj = Transform.from_matrix(joint.geometryOrOriginTwo.transform.asArray())

        joint_dict["oMj"] = oMj
        joints_dict[joint.name] = joint_dict

    return joints_dict, msg
