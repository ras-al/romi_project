# Romi Robot Mesh Files

This directory contains 3D mesh files for the Pololu Romi robot.

## Directory Structure

```
meshes/
├── visual/          # High-detail meshes for visualization
│   ├── chassis.stl
│   ├── wheel.stl
│   └── ball_caster.stl
└── collision/       # Simplified meshes for collision detection
    ├── chassis.stl
    ├── wheel.stl
    └── ball_caster.stl
```

## Getting Mesh Files

### Option 1: Download from Pololu

Pololu provides official 3D models:

1. **Romi Chassis STEP File:**
   - URL: https://www.pololu.com/product/3502
   - Download the STEP file (under "Resources" tab)
   - Convert STEP to STL using:
     - FreeCAD (free, open source)
     - Fusion 360 (free for hobbyists)
     - Online converters

2. **Ball Caster STEP File:**
   - URL: https://www.pololu.com/product/3539
   - Follow same conversion process

### Option 2: Use Existing CAD Models

If you have existing CAD files:
1. Export/Convert to STL format
2. Make sure units are in **millimeters** (scale factor 0.001 in URDF)
3. Orient correctly:
   - Chassis: Z-up, X-forward
   - Wheels: Z-axis along rotation axis
   - Casters: Centered at ball center

### Option 3: Create Simple Meshes

For testing, you can create simple meshes:

**Using FreeCAD (Python Console):**

```python
import FreeCAD
import Part
import Mesh

# Chassis (160mm x 140mm x 40mm box)
chassis = Part.makeBox(160, 140, 40)
chassis.translate(FreeCAD.Vector(-80, -70, 0))
Mesh.export([chassis], 'chassis.stl')

# Wheel (70mm diameter, 15mm width cylinder)
wheel = Part.makeCylinder(35, 15)
Mesh.export([wheel], 'wheel.stl')

# Ball Caster (25.4mm diameter sphere)
ball = Part.makeSphere(12.7)
Mesh.export([ball], 'ball_caster.stl')
```

### Option 4: Use Primitive Geometries (Current Setup)

If you don't have mesh files, the current `romi.urdf` uses primitive shapes (boxes, cylinders, spheres) which work fine for simulation and basic visualization.

## File Requirements

- **Format:** STL (ASCII or Binary)
- **Units:** Millimeters (scale: 0.001 in URDF)
- **Origin:** Center of part for symmetric parts
- **Orientation:** Match coordinate conventions:
  - X: Forward
  - Y: Left
  - Z: Up

## Converting STEP to STL

### Using FreeCAD (Free):

```bash
# Install FreeCAD
sudo apt install freecad  # Linux
# Or download from https://www.freecad.org/

# Convert via GUI:
# 1. Open STEP file
# 2. Select part in tree
# 3. File → Export → Select STL
# 4. Choose binary STL for smaller files
```

### Using MeshLab (Free):

```bash
sudo apt install meshlab
meshlab input.step
# File → Export Mesh As → STL
```

### Online Converters:

- https://www.convertcad.com/ (STEP to STL)
- https://imagetostl.com/convert/file/stp/to/stl

## Using the Mesh URDF

Once you have the mesh files in place:

```bash
# Copy mesh files to the visual folder
cp chassis.stl wheel.stl ball_caster.stl meshes/visual/

# Use the mesh version
cp romi_meshes.urdf romi.urdf

# Reload in RViz2
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat romi.urdf)"
```

## Notes

- **Visual meshes** can be detailed (for appearance)
- **Collision meshes** should be simplified (for performance)
- Keep file sizes reasonable (< 1MB per mesh)
- Binary STL files are more compact than ASCII

## Current Status

⚠️ **Mesh files not included** - You need to obtain/create them using the options above.

For immediate use, stick with `romi.urdf` (geometric primitives) which works without mesh files.

