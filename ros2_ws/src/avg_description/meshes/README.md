# Meshes Directory

This directory is reserved for 3D mesh files (STL, DAE, or OBJ formats) that can be used to enhance the visual representation of the AGV robot.

## Usage

Place your mesh files here and reference them in the URDF/Xacro files using:

```xml
<mesh filename="package://avg_description/meshes/your_mesh_file.stl" scale="1 1 1"/>
```

## Recommended Structure

```plain
meshes/
├── chassis/
│   └── agv_body.stl
├── wheels/
│   └── wheel.stl
└── sensors/
    ├── lidar.dae
    └── camera.stl
```

## Notes

- STL files are good for collision geometry
- DAE (COLLADA) files support colors and textures for visual representation
- Keep mesh complexity reasonable for real-time simulation performance
- Consider using simplified meshes for collision geometry
