# 3D Models for Satellite Constellation Simulation

Place 3D models (.obj files) for different satellite types in this directory. Each model should have a [model].obj and a [model].mtl and any textures in a textures subdirectory.

## Directory Structure

Each model directory should follow this structure:

```
model_name/
├── model_name.obj      # The 3D mesh file
├── model_name.mtl      # Material definitions (colors, textures)
└── textures/           # Texture images referenced by .mtl file
    └── *.jpg/png       # Texture files
```

## How to Add a New Model

### 1. Create Model Directory

Create a new folder in the `models/` directory with your satellite name:

```bash
cd models
mkdir my_model
```

### 2. Add Your Model Files

Place your `.obj` file and `.mtl` file in the directory:

```
models/my_satellite/
├── my_model.obj
└── my_model.mtl
```

**Important**: The `.obj` and `.mtl` files must have the **same base name** as the directory.

### 3. Add Textures (if any)

If your model uses textures, create a `textures/` subdirectory:

```bash
cd my_satellite
mkdir textures
```

Place texture files referenced by your `.mtl` file in this directory.

## Where to Find Satellite 3D Models

### Free Sources

#### 1. NASA 3D Resources (Recommended)

- **URL**: https://nasa3d.arc.nasa.gov/models
- **Content**: Official NASA spacecraft models
- **Format**: Multiple formats including OBJ
- **License**: Public domain (free for any use)
- **Examples**: ISS, James Webb, Mars rovers, various satellites

#### 2. Sketchfab

- **URL**: https://sketchfab.com/
- **Search**: "satellite", "spacecraft", "cubesat"
- **Filter**: Check "Downloadable" and look for CC licenses
- **Format**: Usually includes OBJ export
- **License**: Varies (check each model - look for CC0 or CC-BY)
- **Quality**: Very high quality, many realistic models

#### 3. TurboSquid Free Section

- **URL**: https://www.turbosquid.com/Search/3D-Models/free/satellite
- **Content**: Professional-quality models
- **Format**: OBJ available
- **License**: Free models with attribution
- **Note**: Check license for each model

#### 4. Free3D

- **URL**: https://free3d.com/3d-models/satellite
- **Content**: Various satellite models
- **Format**: OBJ, FBX
- **License**: Free for personal/educational use (check each model)

#### 5. CGTrader Free Section

- **URL**: https://www.cgtrader.com/free-3d-models/space/satellite
- **Content**: Satellites, spacecraft, space stations
- **Format**: Multiple formats
- **License**: Varies (filter by "Free")

#### 6. OpenGameArt.org

- **URL**: https://opengameart.org/
- **Search**: "satellite" or "spacecraft"
- **Format**: Various, often OBJ available
- **License**: CC0 / Public Domain options available

## Model Requirements

### Performance

- **Polygon count**: <50,000 triangles recommended
- **Textures**: 2K resolution or less (2048×2048)
- **Complexity**: Simple geometry is better for performance

### Orientation

- **Forward**: +X axis (direction satellite points)
- **Right**: +Y axis
- **Up**: +Z axis (antenna/solar panel normal)
- **Note**: Orientation can be adjusted in code if needed
