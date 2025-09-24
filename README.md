# Fusion2URDF - URDF Exporter for Fusion 360

A Fusion 360 add-in that exports 3D models to URDF (Unified Robot Description Format) for robotics applications and ROS (Robot Operating System) integration.

## 🔗 Fork Information

This project is forked and enhanced from the original [fusion2urdf by syuntoku14](https://github.com/syuntoku14/fusion2urdf). The original work provided the foundation for exporting Fusion 360 models to URDF format. This version includes improvements, bug fixes, and additional features for better ROS integration.

## ✨ Features

- **Complete URDF Export**: Generate comprehensive URDF files with links, joints, and material properties
- **ROS Package Generation**: Automatically creates ROS package structure with all necessary files
- **Multiple Joint Types**: Supports Revolute, Rigid (Fixed), and Slider (Prismatic) joints
- **STL Mesh Export**: Exports visual and collision meshes in STL format
- **Launch Files**: Generates RViz visualization and Gazebo simulation launch files
- **Xacro Support**: Creates modular xacro files for materials, transmissions, and Gazebo properties
- **Mass Properties**: Automatically calculates inertial properties from Fusion 360 physical properties
- **Coordinate Transformation**: Handles proper coordinate system conversions between Fusion 360 and ROS
- **Package Template**: Includes CMakeLists.txt and package.xml for ROS workspace integration

### Generated Files Structure
```
robot_name_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robot_name.urdf
│   ├── materials.xacro
│   ├── transmissions.xacro
│   └── gazebo.xacro
├── meshes/
│   └── *.stl (visual and collision meshes)
├── launch/
│   ├── display.launch
│   ├── gazebo.launch
│   └── control.launch
└── config/
    └── joint_names.yaml
```

## 🚀 Installation

### Prerequisites
- Autodesk Fusion 360
- Windows or macOS (as specified in manifest)

### Installation Steps

1. **Download the Add-in**
   - Clone or download this repository
   - Extract to a local directory

2. **Install in Fusion 360**
   - Open Fusion 360
   - Go to **Tools** → **Add-Ins** → **Scripts and Add-Ins**
   - Click the **+** button next to "My Scripts"
   - Navigate to and select the `URDF_Exporter` folder
   - The script should appear in the list as "Export stl and URDF file"

3. **Run the Exporter**
   - Select your script from the list
   - Click **Run**
   - Choose your export directory when prompted

## 🎯 Fusion 360 Design Guidelines

To ensure successful URDF export, follow these design rules when creating your Fusion 360 model:

### 🔧 Component Structure
- **Root Component**: Must contain all robot parts as a single assembly
- **Named Components**: Give meaningful names to all components (these become link names)
- **Joint Hierarchy**: Organize joints in a tree structure from base to end-effector

### ⚙️ Joint Configuration
- **Supported Joint Types**:
  - **Revolute**: Rotational joints with defined axis
  - **Rigid**: Fixed connections between components  
  - **Slider**: Linear/prismatic joints with defined axis
- **Joint Limits**: Define proper motion limits for revolute and slider joints
- **Joint Origins**: Verify that joint coordinate systems are correctly aligned for each connection.
    - **Naming Convention**:
        - `j_*`: Represents the actual joint origin associated with the current link.
        - `jref_*`: Serves as the reference joint origin for alignment with child links.

### 📐 Design Best Practices
- **Units**: Use consistent units (length in cm, mass properties in kg/cm²)
- **Mass Properties**: Assign realistic material properties for accurate inertial calculations
- **Coordinate System**: Design with ROS coordinate conventions in mind (X-forward, Z-up)
- **Component Names**: Use descriptive names without special characters or spaces
- **Assembly Structure**: Maintain clear parent-child relationships between components

### ⚠️ Important Notes
- **Length Unit**: Internal calculations use centimeters (cm)
- **Inertial Unit**: Mass properties calculated in kg/cm²
- **Joint Effort/Velocity**: Default values set to 100 (modify in generated URDF if needed)
- **Prismatic Joints**: May require manual limit verification if not defined in Fusion model
- **Root Body**: If no 'body' exists in root component, coordinate system may be incorrect

## 🛠️ Development Information

### Project Structure
```
URDF_Exporter/
├── URDF_Exporter.py          # Main entry point and orchestrator
├── URDF_Exporter.manifest    # Fusion 360 add-in manifest
├── core/                     # Core URDF generation modules
│   ├── __init__.py
│   ├── Link.py              # Link class and link generation
│   ├── Joint.py             # Joint class and joint generation  
│   └── Write.py             # File writing utilities
├── utils/                   # Utility functions and helpers
│   ├── __init__.py
│   ├── utils.py             # General utilities and file operations
│   └── math_utils.py        # Mathematical transformations
├── package/                 # ROS package template files
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── launch/
└── ruff.toml               # Code formatting configuration
```

### Key Classes
- **`Link`**: Represents URDF links with mass properties and geometry
- **`Joint`**: Represents URDF joints with transformations and limits  
- **`UrdfInfo`**: Data structure containing all export information
- **`Transform`**: Handles coordinate transformations between Fusion 360 and ROS

### Technical Details
- **Language**: Python 3.12+
- **Dependencies**: Fusion 360 API (adsk.core, adsk.fusion)
- **Code Quality**: Configured with Ruff for linting and formatting
- **Export Format**: URDF 1.0 specification compliant
- **Coordinate Systems**: Automatic conversion between Fusion 360 and ROS conventions

### Contributing
1. Follow the existing code style (enforced by Ruff configuration)
2. Add type hints for new functions and classes
3. Include docstrings for public methods
4. Test with various Fusion 360 model configurations
5. Update this README for any new features or changes

### Known Limitations
- Limited to Revolute, Rigid, and Slider joint types
- Requires manual verification of prismatic joint limits
- STL export quality depends on Fusion 360 mesh settings
- Complex assemblies may require manual URDF adjustments

## 📄 License

This project maintains the same license as the original fusion2urdf project. Please refer to the original repository for licensing information.

## 🙏 Acknowledgments

- Original fusion2urdf project by [syuntoku14](https://github.com/syuntoku14)
- Fusion 360 API documentation and community
- ROS/URDF specification maintainers