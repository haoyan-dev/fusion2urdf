# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

from enum import Enum
from typing import Tuple, Any, Optional
from ..utils.math_utils import Transform
from ..core.Link import Link
import adsk
import adsk.fusion
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
        self.joint_xml: Optional[str] = None
        self.tran_xml: Optional[str] = None
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


def make_joints(
    root: adsk.fusion.Component,
    urdf_infos: dict[str, Any],
) -> Tuple[dict[str, Joint], dict[str, Link], bool, str]:
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
    all_occurrences: adsk.fusion.OccurrenceList = root.allOccurrences
    all_joints: list[adsk.fusion.Joint] = root.allJoints

    joints: dict[str, Joint] = {}
    links: dict[str, Link] = {}

    repo = urdf_infos["repo"]

    def make_link(occ: adsk.fusion.Occurrence, repo: str) -> Link:
        prop = occ.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy  # type: ignore
        )
        component = occ.component
        all_joint_origins = component.jointOrigins

        # find the joint origin whose name has the prefix "j_"
        j = [jo for jo in all_joint_origins if jo.name.startswith("j_")]
        if len(j) != 1:
            print("Invalid number of joint origins in " + occ.name)
        joint_origin = j[0] if len(j) == 1 else None

        name = convert_occ_name(occ.name)

        mass = prop.mass  # kg
        center_of_mass = [_ / 100.0 for _ in prop.centerOfMass.asArray()]  ## cm to m

        # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
        (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
        moment_inertia_world = [
            _ / 10000.0 for _ in [xx, yy, zz, xy, yz, xz]
        ]  ## kg / cm^2 -> kg/m^2
        inertia_tensor = utils.origin2center_of_mass(
            moment_inertia_world, center_of_mass, mass
        )

        link = Link(name, center_of_mass, repo, mass, inertia_tensor, joint_origin.transform if joint_origin else None)

        return link

    for occ in all_occurrences:
        link = make_link(occ, repo)
        links[occ.name] = link

    print(f"Number of links: {len(links)}")
    print(f"Names of links: {[link.name for link in links.values()]}")

    for fusion_joint in all_joints:
        child_occ = fusion_joint.occurrenceOne
        parent_occ = fusion_joint.occurrenceTwo

        child_joint_origin = fusion_joint.geometryOrOriginOne
        parent_joint_origin = fusion_joint.geometryOrOriginTwo

        if not isinstance(
            child_joint_origin, adsk.fusion.JointOrigin
        ) or not isinstance(parent_joint_origin, adsk.fusion.JointOrigin):
            msg = (
                fusion_joint.name
                + " is not set its joint origin. Please set it and try again."
            )
            return {}, {}, False, msg

        motion: Any = fusion_joint.jointMotion
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
                    return {}, {}, False, msg
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
                    return {}, {}, False, msg
            else:
                msg = (
                    fusion_joint.name
                    + " is not set its angle limit. Please set it and try again."
                )
                return {}, {}, False, msg

        elif fusion_joint_type == JointTypes.SLIDER.value:
            joint_type = "prismatic"

        else:
            msg = (
                fusion_joint.name
                + " is not supported joint type. Only Revolute, Rigid and Slider are supported for now."
            )
            return {}, {}, False, msg

        # calculate the origin of the joint
        wMpo = Transform.from_Matrix3D(parent_occ.transform2)
        wMco = Transform.from_Matrix3D(child_occ.transform2)
        poMpj = Transform.from_Matrix3D(parent_joint_origin.transform)
        coMcj = Transform.from_Matrix3D(child_joint_origin.transform)

        wMcj = wMco * coMcj

        origin = wMpo.inverse() * wMcj

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

        msg = "All the joints are successfully collected."

    return joints, links, True, msg
