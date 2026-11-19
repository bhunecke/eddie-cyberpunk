# Getting Eddie into Cyberpunk 2077

Goal: Eddie as a spawnable prop (AMM "Decor" tab), first visual-only, then with
collision. Based on the RED Modding wiki's custom-prop workflow
(<https://wiki.redmodding.org/cyberpunk-2077-modding/for-mod-creators-theory/3d-modelling/custom-props>);
menu names can shift between WolvenKit versions.

Input files (from `build_glb.py`, in `out/`):

- `eddie_static.glb`: 14 submeshes `submesh_00_LOD_1` … `submesh_13_LOD_1`
- `textures/*.png`: 3 solid textures per material (`_d01` colour, `_r01` roughness, `_m01` metalness)
- `eddie_physics.json`: which submesh uses which material, mass, centre of mass, hulls

## 0. Install

| Tool | Version (per wiki) |
|---|---|
| WolvenKit | 8.15 stable, or ≥ 9.1 / ≥ 10.0 |
| Blender + Cyberpunk Blender add-on (Wolvenkit Blender IO Suite) | Blender 5.3, add-on 2.5.0 |
| Cyber Engine Tweaks + Appearance Menu Mod (AMM) | current, plus AMM's listed requirements |

Start the game once with CET + AMM to confirm they load.

## 1. WolvenKit project and prop files

1. New project, e.g. `eddie_robot` (this becomes the mod name).
2. In the Asset Browser, add a vanilla mesh to the project to use as a container,
   e.g. `base\environment\decoration\construction\concrete_debris\concrete_debris_piece_h.mesh`.
3. Rename it (keep "Update in project files" ticked) to something unique:
   `yourname\props\eddie\eddie.mesh`.
4. `File` → `Generate files` → `Prop (Decoration)`: parent folder `yourname\props\eddie`,
   appearance `default`, first mesh = `eddie.mesh`, tick `1` under "Write to meshes".
   This creates `eddie.ent` and the AMM registration `.lua`.

## 2. Put the robot mesh into `eddie.mesh`

1. WolvenKit: Export tool → export `eddie.mesh`. Note the path of the exported
   `eddie.glb` under the project's `raw` folder.
2. Blender (new empty scene): `File` → `Import` → `glTF 2.0` → `out/eddie_static.glb`.
   - The scene should hold 14 objects at top level named `submesh_XX_LOD_1`. If
     Blender appended `.001` style suffixes, that's fine (WolvenKit ignores them);
     any other name change is not.
   - The mesh is already triangulated, in metres, and under the game's
     65k-per-submesh limit. Don't join or separate anything.
3. Select all → `File` → `Export` → `Cyberpunk GLB`, overwrite the `raw\...\eddie.glb`
   from step 1. Options: **Skinned Mesh off**, **Fix meshes on**, no armature.
4. WolvenKit: Import tool → import `eddie.glb` back onto `eddie.mesh`.

## 3. Textures and materials

1. Copy `out/textures/*.png` into the project's `raw\yourname\props\eddie\textures\`
   and import them with the Import tool (they become `.xbm`).
2. Open `eddie.mesh`. Run `Clean Up` → `Adjust Submesh Counts`
   (optionally convert preload materials to local first).
3. Make one material entry per material name below (12). For each:
   `baseMaterial` = `engine\materials\metal_base.remt`, clear `values`, add three
   Texture parameters with keys `BaseColor`, `Roughness`, `Metalness` pointing to
   `eddie_<material>_d01.xbm`, `_r01.xbm`, `_m01.xbm`.
4. `appearances` → `default` → `chunkMaterials`: assign per submesh:

| Submesh | Material | Submesh | Material |
|---|---|---|---|
| 00 | casing_grey | 07 | drive_grey |
| 01 | cover_light | 08 | wheel_light |
| 02 | cover_light | 09 | tyre_dark |
| 03 | lidar_grey | 10 | arm_grey |
| 04 | mount_grey | 11 | arm_accent |
| 05 | sensor_black | 12 | gripper_black |
| 06 | drive_grey | 13 | gripper_grey |

(This is the one-rigid-mesh build, `MOVING_PARTS = False`. With the default moving
parts, drives and wheels are separate meshes and the body has fewer submeshes; the
current tables are `submeshes` and `moving_parts.meshes` in `eddie_physics.json`, and
`install_wolvenkit.py` does all of this for you.) Later you can swap these flat materials for the game's
multilayered metals/plastics; the UVs are set up for tiling materials.

## 4. Test in game

1. WolvenKit toolbar: green **Install and Launch**.
2. AMM → `Decor` tab → search for the name from the generated `.lua` → spawn.
3. Check: right size (0.71 × 0.69 m, 0.90 m tall), sitting on the floor, front
   facing the way you placed it, colours per part.

At this point Eddie is visible but has no collision (you walk through it).

## 5. Collision and physics (the hard part)

This is the least documented area of Cyberpunk modding; expect trial and error
and use the RED Modding Discord.

- **Static collision (V and NPCs bump into it):** the prop's mesh component needs
  collision data. The wiki's "Enable embedded collisions" guide covers switching
  it on (`filterData`, `navigationImpact` = `Blocking` on the mesh component in
  the `.ent`), but it requires a mesh that already contains a
  `meshMeshParamPhysics` entry, which our imported mesh doesn't. Workarounds the
  wiki suggests: put a hidden vanilla mesh with collision (e.g. a crate or box,
  roughly 0.7 × 0.7 × 0.9 m) inside the prop as a second component.
- **Custom collider shapes:** the Blender add-on's **Collision Tools** can generate
  box, capsule and convex colliders from a mesh (use the hulls in
  `eddie_collision.glb`, or boxes around chassis/torso/arms) and write them into a
  `.phys` file. That export needs an existing `.phys.json` from WolvenKit as a base,
  so start from a vanilla prop that has a `.phys`.
- **Dynamic (falls over, gets pushed or shot):** not documented on the wiki. The
  practical route is to find a vanilla movable prop (one that tips over when shot,
  e.g. a trash bin or crate), copy its `.ent`, swap its mesh for `eddie.mesh` and
  resize its collider to Eddie's shape. Set the mass to 113 kg and the centre of
  mass to `(0, 0, 0.307)` in Z-up (from `eddie_physics.json`). Note the game's prop
  physics is fairly crude, and a 113 kg object may barely move.

In this manual route the arms are part of the one rigid body. The scripted route
(`install_wolvenkit.py` + `eddie_spawner`) moves them without rigging: every arm link
is its own mesh component, placed by the CET mod from the joint values (see README.md).
