# -*- coding: utf-8 -*-
"""Link module for URDF generation from Fusion 360.

This module provides classes and functions to extract link information from
Fusion 360 components and convert them to URDF link representations with
mass properties, visual and collision geometries.

Created on Sun May 12 20:11:28 2019
@author: syuntoku
@modified by: haoyan.li
"""

from typing import Optional
from xml.etree.ElementTree import Element, SubElement

# pyright: reportMissingImports=false
import adsk
import adsk.core
import adsk.fusion

from ..utils import convert_occ_name, prettify, origin2center_of_mass, UrdfInfo
from ..utils.math_utils import Transform


class Link:
    """Represents a URDF link with mass properties and geometry information.

    This class encapsulates all link properties including mass, center of mass,
    inertia tensor, and geometry references. It handles coordinate transformations
    between joint origins and link frames, and generates URDF XML representations.

    Units: meters for length, kilograms for mass, radians for angles

    Attributes:
        name: Name of the link
        center_of_mass: Center of mass coordinates [x, y, z] in meters
        repo: Repository path for mesh file references
        mass: Mass of the link in kilograms
        inertia_tensor: Inertia tensor [ixx, iyy, izz, ixy, iyz, ixz] in kg⋅m²
        lMjo: Transformation from link frame to joint origin
        joMl: Transformation from joint origin to link frame
        link_xml: Generated URDF XML for the link (set by make_link_xml)
    """

    def __init__(
        self,
        name: str,
        center_of_mass: list[float],
        repo: str,
        mass: float,
        inertia_tensor: list[float],
        joint_origin_tf: Optional[adsk.core.Matrix3D] = None,
    ) -> None:
        """Initialize a Link instance.

        Args:
            name: Name of the link
            center_of_mass: Center of mass coordinates [x, y, z] in meters
            repo: Repository path for mesh file references
            mass: Mass of the link in kilograms
            inertia_tensor: Inertia tensor [ixx, iyy, izz, ixy, iyz, ixz] in kg⋅m²
            joint_origin_tf: Optional transformation matrix from link to joint origin
        """
        self.name: str = name
        self.center_of_mass: list[float] = center_of_mass
        self.link_xml: Optional[str] = None
        self.repo: str = repo
        self.mass: float = mass
        self.inertia_tensor: list[float] = inertia_tensor
        self.lMjo: Transform = (
            Transform.from_Matrix3D(joint_origin_tf) if joint_origin_tf else Transform()
        )
        self.joMl: Transform = self.lMjo.inverse()

    def make_link_xml(self) -> list[str]:
        """Generate URDF XML representation of the link.

        Creates a complete URDF link element including:
        - Inertial properties (mass, center of mass, inertia tensor)
        - Visual geometry (STL mesh with silver material)
        - Collision geometry (same STL mesh)

        The visual and collision geometries are adjusted for any offset between
        the joint origin and link origin using the joMl transformation.

        Returns:
            list[str]: Complete URDF XML string for the link

        Note:
            STL meshes are scaled by 0.001 (assuming Fusion 360 export is in mm,
            converting to meters for URDF).
        """

        link = Element("link")
        link.attrib = {"name": self.name}

        # inertial
        inertial = SubElement(link, "inertial")
        origin_i = SubElement(inertial, "origin")
        origin_i.attrib = {
            "xyz": " ".join([f"{round(el, 6)}" for el in self.center_of_mass]),
            "rpy": "0 0 0",
        }
        mass = SubElement(inertial, "mass")
        mass.attrib = {"value": f"{round(self.mass, 6)}"}
        inertia = SubElement(inertial, "inertia")
        inertia.attrib = {
            "ixx": f"{round(self.inertia_tensor[0], 6)}",
            "iyy": f"{round(self.inertia_tensor[1], 6)}",
            "izz": f"{round(self.inertia_tensor[2], 6)}",
            "ixy": f"{round(self.inertia_tensor[3], 6)}",
            "iyz": f"{round(self.inertia_tensor[4], 6)}",
            "ixz": f"{round(self.inertia_tensor[5], 6)}",
        }

        # STL file has no unit. We assume the export from Fusion 360 is in meters.
        scale = 0.001
        # visual
        visual = SubElement(link, "visual")
        origin_v = SubElement(visual, "origin")
        # The urdf origin is at the joint origin, so use self.joMl to fix the visual and collision offset when there is an offset between the joint origin and the link origin.
        origin_v.attrib = {
            "xyz": " ".join([f"{round(el, 6)}" for el in self.joMl.translation]),
            "rpy": " ".join([f"{round(el, 6)}" for el in self.joMl.rotation]),
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
            "xyz": " ".join([f"{round(el, 6)}" for el in self.joMl.translation]),
            "rpy": " ".join([f"{round(el, 6)}" for el in self.joMl.rotation]),
        }
        # origin_c.attrib = {"xyz": " ".join([str(_) for _ in self.xyz]), "rpy": "0 0 0"}
        geometry_c = SubElement(collision, "geometry")
        mesh_c = SubElement(geometry_c, "mesh")
        mesh_c.attrib = {
            "filename": f"{self.repo}meshes/{self.name}.stl",
            "scale": " ".join([str(scale)] * 3),
        }

        # print("\n".join(utils.prettify(link).split("\n")[1:]))
        lines = prettify(link).split("\n")[1:]
        self.link_xml = "\n".join(lines)
        return lines


def make_links(root: adsk.fusion.Component, urdf_infos: UrdfInfo) -> dict[str, Link]:
    """Create Link objects for all occurrences in a Fusion 360 component.

    Processes all occurrences in the root component and creates corresponding
    Link objects with extracted mass properties and geometry information.

    Args:
        root: Root Fusion 360 component containing all occurrences
        urdf_infos: Dictionary containing URDF generation information including repo path

    Returns:
        dict[str, Link]: Dictionary mapping occurrence names to Link objects
    """
    all_occurrences = root.allOccurrences
    repo = urdf_infos["repo"]

    links = {occ.name: make_link(occ, repo) for occ in all_occurrences}

    return links


def make_link(occ: adsk.fusion.Occurrence, repo: str) -> Link:
    """Create a Link instance from a Fusion 360 occurrence.

    Extracts physical properties from the occurrence including mass, center of mass,
    and inertia tensor. Identifies joint origins with names starting with "j_" and
    uses them for coordinate transformations.

    Args:
        occ: Fusion 360 occurrence to extract link data from
        repo: Repository path for mesh file references

    Returns:
        Link: Link instance with extracted properties

    Raises:
        Exception: If occurrence has invalid number of joint origins (not exactly 1)

    Note:
        - Uses VeryHighCalculationAccuracy for physical property calculations
        - Converts units from cm to m for distances and kg⋅cm² to kg⋅m² for inertia
        - Transforms inertia tensor from world frame to center of mass frame
    """
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
    inertia_tensor = origin2center_of_mass(moment_inertia_world, center_of_mass, mass)

    link = Link(
        name,
        center_of_mass,
        repo,
        mass,
        inertia_tensor,
        joint_origin.transform if joint_origin else None,
    )

    return link
