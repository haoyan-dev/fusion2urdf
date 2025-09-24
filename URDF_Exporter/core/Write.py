# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:46:26 2019

@author: syuntoku
"""

import os
from typing import Any
from xml.etree.ElementTree import Element, SubElement

import adsk

from ..core import Joint, Link
from ..utils import utils


def write_link_urdf(file_name: str, links: dict[str, Link]):
    """
    Write links information into urdf "repo/file_name"


    Parameters
    ----------
    file_name: str
        urdf full path
    links: list[Link]
    """
    with open(file_name, mode="a") as f:
        for link_name, link in links.items():
            link_xml = link.make_link_xml()
            f.write(link_xml)
            f.write("\n")


def write_joint_urdf(file_name: str, joints: dict[str, Joint]):
    """
    Write joints information into urdf "repo/file_name"

    Parameters
    ----------
    file_name: str
        urdf full path
    joints: list[Joint]
    """
    with open(file_name, mode="a") as f:
        for joint_name, joint in joints.items():
            transmission_xml = joint.make_transmission_xml()
            joint_xml = joint.make_joint_xml()
            f.write(transmission_xml)
            f.write("\n")
            f.write(joint_xml)
            f.write("\n")


def write_gazebo_endtag(file_name):
    """
    Write about gazebo_plugin and the </robot> tag at the end of the urdf


    Parameters
    ----------
    file_name: str
        urdf full path
    """
    with open(file_name, mode="a") as f:
        f.write("</robot>\n")


def write_urdf(
    urdf_infos: dict[str, Any],
):
    package_name = urdf_infos["package_name"]
    robot_name = urdf_infos["robot_name"]
    urdf_dir = urdf_infos["urdf_dir"]
    joints: dict[str, Joint] = urdf_infos["joints"]
    links: dict[str, Link] = urdf_infos["links"]

    file_name = os.path.join(urdf_dir, f"{robot_name}.xacro")

    with open(file_name, mode="w") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(
            '<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(
                robot_name
            )
        )
        f.write("\n")
        f.write(
            f'<xacro:include filename="$(find {package_name})/urdf/materials.xacro" />'
        )
        f.write("\n")
        f.write(
            f'<xacro:include filename="$(find {package_name})/urdf/{robot_name}.trans" />'
        )
        f.write("\n")
        f.write(
            f'<xacro:include filename="$(find {package_name})/urdf/{robot_name}.gazebo" />'
        )
        f.write("\n")

    write_link_urdf(file_name, links)
    write_joint_urdf(file_name, joints)

    write_gazebo_endtag(file_name)


def write_materials_xacro(urdf_infos: dict[str, str]):
    robot_name = urdf_infos["robot_name"]
    urdf_dir = urdf_infos["urdf_dir"]
    file_name = os.path.join(urdf_dir, "materials.xacro")  # the name of urdf file

    with open(file_name, mode="w") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(
            '<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(
                robot_name
            )
        )
        f.write("\n")
        f.write('<material name="silver">\n')
        f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
        f.write("</material>\n")
        f.write("\n")
        f.write("</robot>\n")


def write_transmissions_xacro(urdf_infos: dict[str, Any]):
    """
    Write joints and transmission information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """
    robot_name = urdf_infos["robot_name"]
    urdf_dir = urdf_infos["urdf_dir"]
    joints: dict[str, Joint] = urdf_infos["joints"]
    file_name = os.path.join(urdf_dir, f"{robot_name}.trans")  # the name of urdf file

    with open(file_name, mode="w") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(
            '<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(
                robot_name
            )
        )
        f.write("\n")

        for j_name, j in joints.items():
            if j.type != "fixed":
                tran_xml = j.make_transmission_xml()
                f.write(tran_xml)
                f.write("\n")

        f.write("</robot>\n")


def write_gazebo_xacro(urdf_infos: dict[str, Any]):
    robot_name = urdf_infos["robot_name"]
    urdf_dir = urdf_infos["urdf_dir"]
    joints: dict[str, Joint] = urdf_infos["joints"]
    file_name = os.path.join(urdf_dir, f"{robot_name}.gazebo")  # the name of urdf file

    with open(file_name, mode="w") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(
            '<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(
                robot_name
            )
        )
        f.write("\n")
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write("\n")

        gazebo = Element("gazebo")
        plugin = SubElement(gazebo, "plugin")
        plugin.attrib = {"name": "control", "filename": "libgazebo_ros_control.so"}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write("  <material>${body_color}</material>\n")
        f.write("  <mu1>0.2</mu1>\n")
        f.write("  <mu2>0.2</mu2>\n")
        f.write("  <selfCollide>true</selfCollide>\n")
        f.write("  <gravity>true</gravity>\n")
        f.write("</gazebo>\n")
        f.write("\n")

        # others
        for joint_name, joint in joints.items():
            f.write('<gazebo reference="{}">\n'.format(joint.child))
            f.write("  <material>${body_color}</material>\n")
            f.write("  <mu1>0.2</mu1>\n")
            f.write("  <mu2>0.2</mu2>\n")
            f.write("  <selfCollide>true</selfCollide>\n")
            f.write("</gazebo>\n")
            f.write("\n")

        f.write("</robot>\n")


def write_display_launch(urdf_infos: dict[str, Any]):
    """
    write display launch file "save_dir/launch/display.launch"


    Parameter
    ---------
    robot_name: str
    name of the robot
    save_dir: str
    path of the repository to save
    """
    package_name = urdf_infos["package_name"]
    robot_name = urdf_infos["robot_name"]
    launch_dir = urdf_infos["launch_dir"]
    
    launch = Element("launch")

    arg1 = SubElement(launch, "arg")
    arg1.attrib = {
        "name": "model",
        "default": f"$(find {package_name})/urdf/{robot_name}.xacro",
    }

    arg2 = SubElement(launch, "arg")
    arg2.attrib = {"name": "gui", "default": "true"}

    arg3 = SubElement(launch, "arg")
    arg3.attrib = {
        "name": "rvizconfig",
        "default": f"$(find {package_name})/launch/urdf.rviz",
    }

    param1 = SubElement(launch, "param")
    param1.attrib = {
        "name": "robot_description",
        "command": "$(find xacro)/xacro $(arg model)",
    }

    param2 = SubElement(launch, "param")
    param2.attrib = {"name": "use_gui", "value": "$(arg gui)"}

    node1 = SubElement(launch, "node")
    node1.attrib = {
        "name": "joint_state_publisher_gui",
        "pkg": "joint_state_publisher_gui",
        "type": "joint_state_publisher_gui",
    }

    node2 = SubElement(launch, "node")
    node2.attrib = {
        "name": "robot_state_publisher",
        "pkg": "robot_state_publisher",
        "type": "robot_state_publisher",
    }

    node3 = SubElement(launch, "node")
    node3.attrib = {
        "name": "rviz",
        "pkg": "rviz",
        "args": "-d $(arg rvizconfig)",
        "type": "rviz",
        "required": "true",
    }

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])

    file_name = f"{launch_dir}/display.launch"
    with open(file_name, mode="w") as f:
        f.write(launch_xml)


def write_gazebo_launch(urdf_infos: dict[str, Any]):
    """
    write gazebo launch file "save_dir/launch/gazebo.launch"


    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """
    package_name = urdf_infos["package_name"]
    robot_name = urdf_infos["robot_name"]
    launch_dir = urdf_infos["launch_dir"]

    launch = Element("launch")
    param = SubElement(launch, "param")
    param.attrib = {
        "name": "robot_description",
        "command": f"$(find xacro)/xacro $(find {package_name})/urdf/{robot_name}.xacro",
    }

    node = SubElement(launch, "node")
    node.attrib = {
        "name": "spawn_urdf",
        "pkg": "gazebo_ros",
        "type": "spawn_model",
        "args": f"-param robot_description -urdf -model {robot_name}",
    }

    include_ = SubElement(launch, "include")
    include_.attrib = {"file": "$(find gazebo_ros)/launch/empty_world.launch"}

    number_of_args = 5
    args = [None for i in range(number_of_args)]
    args_name_value_pairs = [
        ["paused", "true"],
        ["use_sim_time", "true"],
        ["gui", "true"],
        ["headless", "false"],
        ["debug", "false"],
    ]

    for i, arg in enumerate(args):
        arg = SubElement(include_, "arg")
        arg.attrib = {
            "name": args_name_value_pairs[i][0],
            "value": args_name_value_pairs[i][1],
        }

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])

    file_name = f"{launch_dir}/gazebo.launch"
    with open(file_name, mode="w") as f:
        f.write(launch_xml)


def write_control_launch(urdf_infos: dict[str, Any]):
    """
    write control launch file "save_dir/launch/controller.launch"


    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    package_name = urdf_infos["package_name"]
    robot_name = urdf_infos["robot_name"]
    joints: dict[str, Joint] = urdf_infos["joints"]
    launch_dir = urdf_infos["launch_dir"]

    controller_name = f"{robot_name}_controller"
    # rosparam = SubElement(launch, 'rosparam')
    # rosparam.attrib = {'file':'$(find {})/launch/controller.yaml'.format(package_name),
    #                   'command':'load'}

    controller_args_str = ""
    for j_name, j in joints.items():
        joint_type = j.type
        if joint_type != "fixed":
            controller_args_str += f"{j.name}_position_controller "
    controller_args_str += "joint_state_controller "

    node_controller = Element("node")
    node_controller.attrib = {  
        "name": "controller_spawner",
        "pkg": "controller_manager",
        "type": "spawner",
        "respawn": "false",
        "output": "screen",
        "ns": robot_name,
        "args": f"{controller_args_str}",
    }

    node_publisher = Element("node")
    node_publisher.attrib = {
        "name": "robot_state_publisher",
        "pkg": "robot_state_publisher",
        "type": "robot_state_publisher",
        "respawn": "false",
        "output": "screen",
    }
    remap = SubElement(node_publisher, "remap")
    remap.attrib = {"from": "/joint_states", "to": f"/{robot_name}/joint_states"}

    # launch_xml  = "\n".join(utils.prettify(launch).split("\n")[1:])
    launch_xml = "\n".join(utils.prettify(node_controller).split("\n")[1:])
    launch_xml += "\n".join(utils.prettify(node_publisher).split("\n")[1:])

    file_name = f"{launch_dir}/controller.launch"
    with open(file_name, mode="w") as f:
        f.write("<launch>\n")
        f.write("\n")
        # for some reason ROS is very picky about the attribute ordering, so we'll bitbang this element
        f.write(
            f'<rosparam file="$(find {package_name})/launch/controller.yaml" command="load"/>'
        )
        f.write("\n")
        f.write(launch_xml)
        f.write("\n")
        f.write("</launch>")


def write_yaml(urdf_infos: dict[str, Any]):
    """
    write yaml file "save_dir/launch/controller.yaml"


    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    robot_name = urdf_infos["robot_name"]
    joints: dict[str, Joint] = urdf_infos["joints"]
    launch_dir = urdf_infos["launch_dir"]

    controller_name = f"{robot_name}_controller"
    file_name = f"{launch_dir}/controller.yaml"
    with open(file_name, "w") as f:
        f.write(f"{controller_name}:\n")
        # joint_state_controller
        f.write("  # Publish all joint states -----------------------------------\n")
        f.write("  joint_state_controller:\n")
        f.write("    type: joint_state_controller/JointStateController\n")
        f.write("    publish_rate: 50\n\n")
        # position_controllers
        f.write("  # Position Controllers --------------------------------------\n")
        for joint_name, joint in joints.items():
            joint_type = joint.type
            if joint_type != "fixed":
                f.write(f"  {joint.name}_position_controller:\n")
                f.write(f"    type: effort_controllers/JointPositionController\n")
                f.write(f"    joint: {joint.name}\n")
                f.write(f"    pid: {{p: 100.0, i: 0.01, d: 10.0}}\n")
