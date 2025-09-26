"""Utilities package for URDF generation from Fusion 360.

This package provides mathematical transformations, file operations, and utility
functions needed for converting Fusion 360 designs to URDF format.

Modules:
    math_utils: Mathematical transformation utilities and matrix operations
    utils: File operations, XML processing, and URDF generation utilities
"""

from .math_utils import Transform
from .utils import (
    convert_occ_name,
    export_stl,
    file_dialog,
    make_package_structure,
    create_package,
    prettify,
    prettify_xml_str,
    origin2center_of_mass,
    UrdfInfo,
)

__all__ = [
    "Transform",
    "convert_occ_name",
    "export_stl",
    "file_dialog",
    "make_package_structure",
    "create_package",
    "prettify",
    "prettify_xml_str",
    "origin2center_of_mass",
    "UrdfInfo",
]
