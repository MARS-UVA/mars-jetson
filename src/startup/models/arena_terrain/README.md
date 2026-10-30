# Separate arena terrain models

Run from the repository root:

```bash
python3 src/startup/models/arena_terrain/generate_heightmap.py \
  --layout /path/to/layout.json \
  --output-dir src/startup/models/arena_terrain_alternate
```

The layout is a JSON array of `[world_x, world_y, radius, depth]`, all in metres:

```json
[
  [1.0, 1.0, 0.2, 0.15],
  [5.0, 0.5, 0.225, 0.25]
]
```

The arena spans x=0..6.88 and y=-2.5..2.5. Craters must fit inside it.
Protect starting/construction zones and the central column when choosing positions;
these exclusions are not enforced. Overlapping bowls add their depths.
Use `[]` for a flat floor. Omit `--layout` to use the original ten-crater layout.

The output directory contains a complete Gazebo model: `model.config`, `model.sdf`,
`meshes/terrain.obj`, its material, the PNG, and a saved `layout.json`.
Both collision and visual mesh URIs use the output folder's name. Use a unique
folder name for each terrain. Reusing an output directory overwrites its generated files.
The PNG rescales for overlapping bowls; the OBJ always stores actual metre depths.

Copy `src/startup/world/arena_nasa_craters.world` to a new world and change its
terrain include to:

```xml
<include>
  <uri>model://arena_terrain_alternate</uri>
  <pose>3.44 0 0 0 0 0</pose>
</include>
```

Rebuild the `startup` package and source the workspace to install the new model
and world. Select the new world in your launch configuration; this generator
only creates models. For models outside this package, add their parent directory
to `GZ_SIM_RESOURCE_PATH`.

Running the generator without arguments regenerates the original model in place.
