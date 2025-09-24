# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:11:28 2019

@author: syuntoku
"""

from typing import Optional, Tuple
from ..utils.math_utils import Transform
import adsk
import adsk.fusion
import adsk.core
import re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils, convert_occ_name


class Link:
    """
    Class for a link.
    Unit: m, kg, radian
    """

    def __init__(self, name, center_of_mass, repo, mass, inertia_tensor, joint_origin_tf: Optional[adsk.core.Matrix3D] = None):
        """
        Parameters
        ----------
        name: str
            name of the link
        center_of_mass: [x, y, z]
            coordinate for the center of mass
        link_xml: str
            generated xml describing about the link
        repo: str
            the name of the repository to save the xml file
        mass: float
            mass of the link
        inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
            tensor of the inertia
        """
        self.name = name
        self.center_of_mass = center_of_mass
        self.link_xml = None
        self.repo = repo
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        self.lMjo = Transform.from_Matrix3D(joint_origin_tf) if joint_origin_tf else Transform()
        self.joMl = self.lMjo.inverse()

    def make_link_xml(self) -> str:
        """
        Generate the link_xml and hold it by self.link_xml
        """

        link = Element("link")
        link.attrib = {"name": self.name}

        # inertial
        inertial = SubElement(link, "inertial")
        origin_i = SubElement(inertial, "origin")
        origin_i.attrib = {
            "xyz": " ".join([str(_) for _ in self.center_of_mass]),
            "rpy": "0 0 0",
        }
        mass = SubElement(inertial, "mass")
        mass.attrib = {"value": str(self.mass)}
        inertia = SubElement(inertial, "inertia")
        inertia.attrib = {
            "ixx": str(self.inertia_tensor[0]),
            "iyy": str(self.inertia_tensor[1]),
            "izz": str(self.inertia_tensor[2]),
            "ixy": str(self.inertia_tensor[3]),
            "iyz": str(self.inertia_tensor[4]),
            "ixz": str(self.inertia_tensor[5]),
        }

        # STL file has no unit. We assume the export from Fusion 360 is in meters.
        scale = 0.001
        # visual
        visual = SubElement(link, "visual")
        origin_v = SubElement(visual, "origin")
        # The urdf origin is at the joint origin, so use self.joMl to fix the visual and collision offset when there is an offset between the joint origin and the link origin.
        origin_v.attrib = {
            "xyz": " ".join([str(round(el, 6)) for el in self.joMl.translation]),
            "rpy": " ".join([str(round(el, 6)) for el in self.joMl.rotation]),
        }
        geometry_v = SubElement(visual, "geometry")
        mesh_v = SubElement(geometry_v, "mesh")
        mesh_v.attrib = {
            "filename": f"{self.repo}meshes/{self.name}.stl",
            "scale": " ".join([str(scale)] * 3),
        }
        material = SubElement(visual, "material")
        material.attrib = {"name": "silver"}

        # collision
        collision = SubElement(link, "collision")
        origin_c = SubElement(collision, "origin")
        origin_c.attrib = {
            "xyz": " ".join([str(round(el, 6)) for el in self.joMl.translation]),
            "rpy": " ".join([str(round(el, 6)) for el in self.joMl.rotation]),
        }
        # origin_c.attrib = {"xyz": " ".join([str(_) for _ in self.xyz]), "rpy": "0 0 0"}
        geometry_c = SubElement(collision, "geometry")
        mesh_c = SubElement(geometry_c, "mesh")
        mesh_c.attrib = {
            "filename": f"{self.repo}meshes/{self.name}.stl",
            "scale": " ".join([str(scale)] * 3),
        }

        # print("\n".join(utils.prettify(link).split("\n")[1:]))
        self.link_xml = "\n".join(utils.prettify(link).split("\n")[1:])

        return self.link_xml


def make_links(
    all_occurrences: adsk.fusion.OccurrenceList, repo: str
) -> Tuple[list, bool, str]:
    """
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    links: list[Link]

    msg: str
        Tell the status
    """
    # Get component properties.
    # Design: Lengths (cm), Angles (radians), Mass (kg)
    allOccs = all_occurrences
    links = []

    for occ in allOccs:

        prop = occ.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy  # type: ignore
        )

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

        link = Link(name, center_of_mass, repo, mass, inertia_tensor)
        links.append(link)

    msg = "Successfully create links"
    return links, True, msg

