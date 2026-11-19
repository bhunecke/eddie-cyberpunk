"""Push the build output into a WolvenKit prop project, pack it, and install it.

Expects a project created with WolvenKit's `File > Generate files > Prop (Decoration)`
(it provides the .ent, a container .mesh and the AMM .lua). This script:

  1. imports out/eddie_static.glb onto the project's .mesh (keeps the CR2W, swaps geometry),
     and the part meshes (eddie_drive, eddie_wheel_r/l, eddie_arm_*, eddie_grip_*)
  2. imports out/textures/*.png as .xbm (colour maps set to sRGB)
  3. writes one metal_base.remt material per build material and assigns them per submesh
  4. adds a kinematic entColliderComponent to the .ent: one box per hull in eddie_collision.glb
     (sets the AMM spawn offset to 0 too; the mesh origin is on the floor), and one mesh
     component per drive housing, wheel, arm link and gripper part (eddie_part_*), plus
     eddie_parts.lua (layout, arm chain and poses) for eddie_spawner
  5. builds the project and copies into the game: the .archive, the project's own CET mods
     (eddie_spawner), and the AMM / entSpawner registration files if those mods are installed
  6. with --package: writes dist/eddie_robot-<version>.zip, laid out like the game folder

Run:  .venv/Scripts/python install_wolvenkit.py [--game DIR] [--wolvenkit CLI] [--no-install]
                                                [--skip-import] [--package]
The WolvenKit CLI and the game folder can also come from the WOLVENKIT_CLI and
CYBERPUNK_DIR environment variables; the project defaults to ../wolvenkit.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import shutil
import subprocess
import os
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"

PROP_DIR = r"etarak\props\eddie"  # depot folder of the prop inside the project
MESH_NAME = "eddie"
MOD_NAME = "eddie_spawner"
CET_MODS_REL = Path("bin") / "x64" / "plugins" / "cyber_engine_tweaks" / "mods"
DIST = HERE.parent / "dist"
# the usual install folders, tried when neither --game nor CYBERPUNK_DIR is given
GAME_GUESSES = [rf"{d}:\{p}" for d in "CDEFG" for p in (
    r"SteamLibrary\steamapps\common\Cyberpunk 2077",
    r"Program Files (x86)\Steam\steamapps\common\Cyberpunk 2077",
    r"Program Files (x86)\GOG Galaxy\Games\Cyberpunk 2077",
    r"GOG Games\Cyberpunk 2077",
    r"Program Files\Epic Games\Cyberpunk2077")]

# set by configure()
WKIT = PROJECT = GAME = ARCHIVE = RAW = CET_MODS = SPAWNER_MOD = None


def configure(project: Path, wolvenkit: str | None, game: str | None, need_game: bool) -> None:
    global WKIT, PROJECT, GAME, ARCHIVE, RAW, CET_MODS, SPAWNER_MOD
    PROJECT = project.resolve()
    if not (PROJECT / f"{PROJECT.name}.cpmodproj").exists() and not list(PROJECT.glob("*.cpmodproj")):
        raise SystemExit(f"no WolvenKit project (*.cpmodproj) in {PROJECT}")
    ARCHIVE = PROJECT / "source" / "archive"
    RAW = PROJECT / "source" / "raw"
    SPAWNER_MOD = PROJECT / "source" / "resources" / CET_MODS_REL / MOD_NAME
    cli = wolvenkit or os.environ.get("WOLVENKIT_CLI") or shutil.which("WolvenKit.CLI")
    if not cli or not Path(cli).exists():
        raise SystemExit("WolvenKit CLI not found: pass --wolvenkit path\\to\\WolvenKit.CLI.exe "
                         "or set WOLVENKIT_CLI")
    WKIT = Path(cli)
    game = game or os.environ.get("CYBERPUNK_DIR") or next(
        (g for g in GAME_GUESSES if (Path(g) / "bin" / "x64" / "Cyberpunk2077.exe").exists()), None)
    if need_game and not game:
        raise SystemExit("Cyberpunk 2077 not found: pass --game \"path\\to\\Cyberpunk 2077\" "
                         "or set CYBERPUNK_DIR (or use --no-install)")
    GAME = Path(game) if game else None
    CET_MODS = GAME / CET_MODS_REL if GAME else None


def wkit(*args: str) -> str:
    proc = subprocess.run([str(WKIT), *args], capture_output=True, text=True)
    out = proc.stdout + proc.stderr
    # the exit code isn't reliable (import returns non-zero on success), so go by the log
    if re.search(r"\]\s*-\s*(Failed|Error)|\[\s*\d+:\s*Error", out) or (proc.returncode != 0 and not out):
        raise RuntimeError(f"WolvenKit {' '.join(args[:1])} failed:\n{out}")
    return out


def cname(value: str) -> dict:
    return {"$type": "CName", "$storage": "string", "$value": value}


def depot(path: str) -> dict:
    return {"DepotPath": {"$type": "ResourcePath", "$storage": "string", "$value": path}, "Flags": "Default"}


def edit_cr2w(path: Path, edit) -> None:
    """Serialize a CR2W file to JSON, let `edit` change the root chunk, write it back."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        wkit("convert", "serialize", str(path), "-o", str(tmp))
        js = tmp / f"{path.name}.json"
        doc = json.loads(js.read_text(encoding="utf-8"))
        edit(doc["Data"]["RootChunk"])
        js.write_text(json.dumps(doc, indent=2), encoding="utf-8")
        built = tmp / "built"
        built.mkdir()
        wkit("convert", "deserialize", str(js), "-o", str(built))
        shutil.copyfile(built / path.name, path)


def moving_parts() -> dict | None:
    return json.loads((OUT / "eddie_physics.json").read_text()).get("moving_parts")


def mesh_tables() -> dict[str, list[dict]]:
    """.mesh name -> its submesh/material table (the body, plus the drive and wheel parts)."""
    meta = json.loads((OUT / "eddie_physics.json").read_text())
    tables = {MESH_NAME: meta["submeshes"]}
    tables.update((meta.get("moving_parts") or {}).get("meshes", {}))
    return tables


def import_mesh(name: str, glb: Path) -> None:
    raw_glb = RAW / PROP_DIR / f"{name}.glb"
    raw_glb.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(glb, raw_glb)
    target = ARCHIVE / PROP_DIR / f"{name}.mesh"
    if not target.exists():  # the import rebuilds an existing .mesh: start from the body's
        shutil.copyfile(ARCHIVE / PROP_DIR / f"{MESH_NAME}.mesh", target)
    out = wkit("import", str(raw_glb), "-k", "-o", str(ARCHIVE / PROP_DIR))
    if "Rebuilt with buffers" not in out:
        raise RuntimeError(out)
    print(f"{name}.mesh imported")


def import_meshes() -> None:
    import_mesh(MESH_NAME, OUT / "eddie_static.glb")
    for name in mesh_tables():
        if name != MESH_NAME:
            import_mesh(name, OUT / f"{name}.glb")


def import_textures() -> list[str]:
    raw_tex = RAW / PROP_DIR / "textures"
    arc_tex = ARCHIVE / PROP_DIR / "textures"
    raw_tex.mkdir(parents=True, exist_ok=True)
    arc_tex.mkdir(parents=True, exist_ok=True)
    pngs = sorted((OUT / "textures").glob("*.png"))
    for png in pngs:
        shutil.copyfile(png, raw_tex / png.name)
    wkit("import", str(raw_tex), "-o", str(arc_tex))

    def srgb(root):
        root["setup"]["isGamma"] = 1

    for png in pngs:
        if png.stem.endswith("_d01"):
            edit_cr2w(arc_tex / f"{png.stem}.xbm", srgb)
    print(f"{len(pngs)} textures imported")
    return [p.stem for p in pngs]


def setup_materials(mesh_name: str, submeshes: list[dict]) -> None:
    meta = json.loads((OUT / "eddie_physics.json").read_text())
    textures = {row["material"]: row for row in meta["material_textures"]}
    chunk_materials = [row["material"] for row in submeshes]
    order = list(dict.fromkeys(chunk_materials))  # material slot order
    tex_dir = PROP_DIR + r"\textures"

    def edit(root):
        n_chunks = len(root["renderResourceBlob"]["Data"]["header"]["renderChunkInfos"])
        if n_chunks != len(chunk_materials):
            raise RuntimeError(f"mesh has {n_chunks} submeshes, build table has {len(chunk_materials)}")
        root["materialEntries"] = [
            {"$type": "CMeshMaterialEntry", "index": i, "isLocalInstance": 1, "name": cname(m)}
            for i, m in enumerate(order)
        ]
        root["externalMaterials"] = []
        root["preloadExternalMaterials"] = []
        root["localMaterialInstances"] = []
        root["localMaterialBuffer"] = {"$type": "meshMeshMaterialBuffer", "materials": [],
                                       "rawData": None, "rawDataHeaders": []}
        # handles are numbered in document order: appearances, materials, render blob
        root["appearances"] = [{"HandleId": "0", "Data": {
            "$type": "meshMeshAppearance",
            "chunkMaterials": [cname(m) for m in chunk_materials],
            "name": cname("default"), "tags": []}}]
        root["preloadLocalMaterialInstances"] = [
            {"HandleId": str(1 + i), "Data": {
                "$type": "CMaterialInstance",
                "audioTag": cname("None"),
                "baseMaterial": depot(r"engine\materials\metal_base.remt"),
                "cookingPlatform": "PLATFORM_None",
                "enableMask": 0,
                "metadata": None,
                "resourceVersion": 4,
                "values": [
                    {"$type": "rRef:ITexture", key: depot(rf"{tex_dir}\{textures[m][key]}.xbm")}
                    for key in ("BaseColor", "Roughness", "Metalness")
                ],
            }}
            for i, m in enumerate(order)
        ]
        root["renderResourceBlob"]["HandleId"] = str(1 + len(order))

    edit_cr2w(ARCHIVE / PROP_DIR / f"{mesh_name}.mesh", edit)
    print(f"{mesh_name}: {len(order)} materials on {len(chunk_materials)} submeshes")


COLLIDER_NAME = "eddie_collider"
# same filter as the vanilla devices\custom\falling_crate.ent collider: blocks V, NPCs, bullets
COLLIDER_FILTER = {
    "$type": "physicsFilterData",
    "customFilterData": None,
    "preset": cname("Device solid visible"),
    "queryFilter": {"$type": "physicsQueryFilter", "mask1": "0", "mask2": "139837704"},
    "simulationFilter": {"$type": "physicsSimulationFilter", "mask1": "114696", "mask2": "23627"},
}


def _identity_world_transform() -> dict:
    fp = {"$type": "FixedPoint", "Bits": 0}
    return {"$type": "WorldTransform",
            "Orientation": {"$type": "Quaternion", "i": 0, "j": 0, "k": 0, "r": 1},
            "Position": {"$type": "WorldPosition", "x": dict(fp), "y": dict(fp), "z": dict(fp)}}


def _box(b: dict) -> dict:
    i, j, k, r = b["quat_ijkr"]
    x, y, z = b["center"]
    hx, hy, hz = b["half_extents"]
    return {
        "$type": "physicsColliderBox",
        "filterData": None,
        "halfExtents": {"$type": "Vector3", "X": hx, "Y": hy, "Z": hz},
        "isImported": 0, "isObstacle": 0, "isQueryShapeOnly": 0,
        "localToBody": {"$type": "Transform",
                        "orientation": {"$type": "Quaternion", "i": i, "j": j, "k": k, "r": r},
                        "position": {"$type": "Vector4", "W": 0, "X": x, "Y": y, "Z": z}},
        "material": cname("None"),
        "materialApperanceOverrides": [],
        "tag": cname("None"),
        "volumeModifier": 1,
    }


def remove_components(root: dict, match) -> None:
    """Drop the components whose name matches from `components` and `compiledData`,
    keeping the compiled chunk index -> component id table (CruidDict) in step."""
    compiled = root["compiledData"]["Data"]
    chunks = compiled["Chunks"]
    name = lambda c: (c.get("name") or {}).get("$value")
    keep = [i for i, c in enumerate(chunks) if not match(name(c))]
    new_index = {old: new for new, old in enumerate(keep)}
    compiled["CruidDict"] = {str(new_index[int(k)]): v for k, v in
                             sorted(compiled["CruidDict"].items(), key=lambda kv: int(kv[0]))
                             if int(k) in new_index}
    chunks[:] = [chunks[i] for i in keep]
    root["components"] = [c for c in root["components"] if not match(name(c))]


def component_id(name: str) -> str:
    return str(int.from_bytes(hashlib.sha1(name.encode()).digest()[:8], "little") >> 1)


def add_colliders() -> int:
    """One entColliderComponent with a box per collision hull, kinematic (moves only when
    spawned/placed), written to both `components` and `compiledData` like vanilla entities."""
    meta = json.loads((OUT / "eddie_physics.json").read_text())
    boxes = [h["box_zup"] for h in meta["collision_hulls"]]
    comp_id = component_id(COLLIDER_NAME)

    def edit(root):
        compiled = root["compiledData"]["Data"]
        chunks = compiled["Chunks"]
        remove_components(root, lambda n: n == COLLIDER_NAME)  # idempotent: drop an earlier run's

        used = [int(h) for h in re.findall(r'"Handle(?:Ref)?Id": "(\d+)"', json.dumps(root))]
        next_id = max(used, default=-1) + 1
        box_ids = [str(next_id + n) for n in range(len(boxes))]
        filter_id = str(next_id + len(boxes))

        def component(defined: bool) -> dict:
            return {
                "$type": "entColliderComponent",
                "colliders": [{"HandleId": h, "Data": _box(b)} if defined else {"HandleRefId": h}
                              for h, b in zip(box_ids, boxes)],
                "comOffset": {"$type": "Transform",
                              "orientation": {"$type": "Quaternion", "i": 0, "j": 0, "k": 0, "r": 1},
                              "position": {"$type": "Vector4", "W": 0, "X": 0, "Y": 0,
                                           "Z": round(meta["center_of_mass"][1], 4)}},
                "dynamicTrafficSetting": {"$type": "TrafficGenDynamicTrafficSetting", "impact": "Ignored"},
                "filterData": {"HandleId": filter_id, "Data": COLLIDER_FILTER} if defined
                else {"HandleRefId": filter_id},
                "id": comp_id,
                "inertia": {"$type": "Vector3", "X": 1, "Y": 1, "Z": 1},
                "isEnabled": 1,
                "isReplicable": 0,
                "localTransform": _identity_world_transform(),
                "mass": round(meta["mass_kg"], 2),
                "massOverride": -1,
                "name": cname(COLLIDER_NAME),
                "parentTransform": None,
                "simulationType": "Kinematic",
                "startInactive": 0,
                "useCCD": 0,
                "volume": 0,
            }

        chunks.append(component(defined=True))  # compiledData comes first in the file
        compiled["CruidDict"][str(len(chunks) - 1)] = comp_id
        root["components"].append(component(defined=False))

    edit_cr2w(ARCHIVE / PROP_DIR / f"{MESH_NAME}.ent", edit)
    print(f"collider: {len(boxes)} boxes")
    return len(boxes)


PART_PREFIX = "eddie_part_"  # component names: eddie_part_<unit>, eddie_part_<unit>_<r|l>
UNIT_KEYS = {"front_left": "fl", "rear_left": "rl", "rear_right": "rr", "front_right": "fr"}
# keys of the body's entPhysicalMeshComponent that a plain entMeshComponent doesn't have
PHYSICAL_ONLY = ("filterData", "filterDataSource", "simulationType", "startInactive",
                 "useResourceSimulationType", "visibilityAnimationParam")


def _game_pos(p):
    """URDF base frame (x forward, y left) -> entity space (front +Y, Z up)."""
    x, y, z = p
    return -y, x, z


def _place(p, yaw: float) -> dict:
    fixed = lambda v: {"$type": "FixedPoint", "Bits": round(v * 131072)}  # 17 fractional bits
    x, y, z = _game_pos(p)
    return {"$type": "WorldTransform",
            "Orientation": {"$type": "Quaternion", "i": 0, "j": 0, "k": round(math.sin(yaw / 2), 7),
                            "r": round(math.cos(yaw / 2), 7)},
            "Position": {"$type": "WorldPosition", "x": fixed(x), "y": fixed(y), "z": fixed(z)}}


def _place_q(p, q) -> dict:
    """Like _place, for a full orientation (quaternion i j k r in the URDF base frame)."""
    t = _place(p, 0.0)
    s = math.sqrt(0.5)  # entity space = URDF base frame turned +90 deg about z
    i, j, k, r = q
    t["Orientation"].update(i=round(s * (i - j), 7), j=round(s * (i + j), 7),
                            k=round(s * (k + r), 7), r=round(s * (r - k), 7))
    return t


def add_parts() -> int:
    """One entMeshComponent per drive housing, wheel, arm link and gripper part, in its
    rest pose. The CET mod moves them with SetLocalTransform; without it they just sit
    where they belong."""
    parts = moving_parts()

    def edit(root):
        remove_components(root, lambda n: bool(n) and n.startswith(PART_PREFIX))
        if not parts:
            return
        compiled = root["compiledData"]["Data"]
        chunks = compiled["Chunks"]
        body = next(c for c in chunks if c.get("mesh", {}).get("DepotPath", {}).get("$value", "")
                    .endswith(f"\\{MESH_NAME}.mesh"))
        template = {k: v for k, v in body.items() if k not in PHYSICAL_ONLY}
        template["$type"] = "entMeshComponent"
        template["parentTransform"] = None

        def component(name, mesh, pos, yaw=0.0, quat=None):
            c = json.loads(json.dumps(template))
            place = _place(pos, yaw) if quat is None else _place_q(pos, quat)
            c.update(name=cname(name), id=component_id(name), localTransform=place,
                     mesh=depot(rf"{PROP_DIR}\{mesh}.mesh"))
            return c

        new = []
        for d in parts["drives"]:
            key, pivot, steer = UNIT_KEYS[d["name"]], d["pivot"], d["steer0"]
            yaw = math.pi / 2 + steer  # mesh space is the drive frame; entity space is it turned +90 deg
            new.append(component(f"{PART_PREFIX}{key}", "eddie_drive", pivot, yaw))
            for side, (ox, oy, oz) in parts["wheel_offsets"].items():
                c, s = math.cos(steer), math.sin(steer)
                pos = (pivot[0] + c * ox - s * oy, pivot[1] + s * ox + c * oy, pivot[2] + oz)
                new.append(component(f"{PART_PREFIX}{key}_{side}", f"eddie_wheel_{side}", pos, yaw))
        for e in (parts.get("arms") or {}).get("links", []):
            new.append(component(f"{PART_PREFIX}{e['name']}", e["mesh"], e["rest"][:3], quat=e["rest"][3:]))
        for c in new:
            chunks.append(c)
            compiled["CruidDict"][str(len(chunks) - 1)] = c["id"]
            root["components"].append(json.loads(json.dumps(c)))

    edit_cr2w(ARCHIVE / PROP_DIR / f"{MESH_NAME}.ent", edit)
    write_parts_lua(parts)
    n = 0 if not parts else (len(parts["drives"]) * (1 + len(parts["wheel_offsets"]))
                             + len((parts.get("arms") or {}).get("links", [])))
    print(f"moving parts: {n} mesh components")
    return n


def write_parts_lua(parts: dict | None) -> None:
    """The layout for eddie_spawner (component names, pivots, wheel offsets, radius)."""
    (SPAWNER_MOD / "parts.lua").unlink(missing_ok=True)  # earlier name
    path = SPAWNER_MOD / "eddie_parts.lua"
    if not parts:
        path.unlink(missing_ok=True)
        return
    v = lambda p: "{ %s }" % ", ".join(f"{c:.6f}" for c in p)
    lines = ["-- Generated by install_wolvenkit.py from out/eddie_physics.json; don't edit.",
             "-- URDF base frame: x forward, y left, z up, metres; angles in radians.",
             "return {",
             f'    prefix = "{PART_PREFIX}",',
             f"    wheel_radius = {parts['wheel_radius']:.6f},",
             "    wheel_offsets = { " + ", ".join(f"{s} = {v(o)}" for s, o in parts["wheel_offsets"].items()) + " },",
             "    drives = {"]
    for d in parts["drives"]:
        lines.append(f'        {{ key = "{UNIT_KEYS[d["name"]]}", pivot = {v(d["pivot"])}, steer0 = {d["steer0"]:.6f} }},')
    lines.append("    },")
    arms = parts.get("arms")
    if arms:
        # parts in chain order; parent = index into this list (0: Eddie's base frame);
        # origin = x y z qi qj qk qr from the parent to the joint; the part is then turned
        # about axis by scale * vars[var]
        tab = lambda d: "{ " + ", ".join(f"{k} = {x:.9f}" for k, x in d.items()) + " }"
        v9 = lambda p: "{ %s }" % ", ".join(f"{c:.9f}" for c in p)
        lines += ["    arms = {",
                  f"        tool = {v(arms['tool'])},",
                  f"        grip_closed = {arms['grip_closed']:.4f},",
                  f"        vars = {tab(arms['vars'])},",
                  f'        wave = {{ var = "{arms["wave"]["var"]}", amplitude = {arms["wave"]["amplitude"]:.4f}, '
                  f'hz = {arms["wave"]["hz"]:.4f} }},',
                  "        poses = {"]
        for name, pose in arms["poses"].items():
            lines.append(f"            {name} = {tab(pose)},")
        lines += ["        },", "        links = {"]
        for e in arms["links"]:
            joint = (f', axis = {v(e["axis"])}, var = "{e["var"]}", scale = {e["scale"]:.1f}'
                     if e["var"] else "")
            lines.append(f'            {{ name = "{e["name"]}", parent = {e["parent"] + 1}, '
                         f'origin = {v9(e["origin"])}{joint} }},')
        lines += ["        },", "    },"]
    lines += ["}", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def fix_amm_lua() -> list[Path]:
    luas = list((PROJECT / "source" / "resources").rglob("*.lua"))
    for lua in luas:
        text = lua.read_text(encoding="utf-8")
        lua.write_text(re.sub(r"distanceFromGround\s*=\s*[\d.]+", "distanceFromGround = 0", text),
                       encoding="utf-8")
    return luas


def build_and_install(install: bool) -> None:
    packed = PROJECT / "packed"
    if packed.exists():
        shutil.rmtree(packed)
    wkit("build", str(PROJECT))
    archives = list(packed.rglob("*.archive"))
    if not archives:
        raise RuntimeError(f"build produced no .archive in {packed}")
    print("built", ", ".join(str(a.relative_to(PROJECT)) for a in archives))
    if not install:
        return
    mod_dir = GAME / "archive" / "pc" / "mod"
    mod_dir.mkdir(parents=True, exist_ok=True)
    for a in archives:
        shutil.copyfile(a, mod_dir / a.name)
        print("installed", mod_dir / a.name)
    # CET-side files only when the mod they belong to is installed
    for f in (packed / "bin").rglob("*") if (packed / "bin").exists() else []:
        if not f.is_file():
            continue
        rel = f.relative_to(packed)
        owner = rel.parts[5] if len(rel.parts) > 5 else ""  # bin/x64/plugins/cyber_engine_tweaks/mods/<owner>
        ours = (packed / "bin/x64/plugins/cyber_engine_tweaks/mods" / owner / "init.lua").exists()
        if not ours and not (CET_MODS / owner / "init.lua").exists():
            print(f"skipped {rel} ({owner} is not installed)")
            continue
        dest = GAME / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(f, dest)
        print("installed", dest)


def package() -> Path:
    """dist/eddie_robot-<version>.zip: the .archive and the eddie_spawner CET mod (with the
    licence files), laid out like the game folder so it can be extracted straight into it."""
    proj = next(PROJECT.glob("*.cpmodproj"))
    version = re.search(r"<Version>([^<]+)</Version>", proj.read_text(encoding="utf-8")).group(1)
    packed = PROJECT / "packed"
    archive = packed / "archive" / "pc" / "mod" / f"{proj.stem}.archive"
    mod = packed / CET_MODS_REL / MOD_NAME
    DIST.mkdir(exist_ok=True)
    out = DIST / f"{proj.stem}-{version}.zip"
    root = HERE.parent
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(archive, (Path("archive") / "pc" / "mod" / archive.name).as_posix())
        for f in sorted(mod.glob("*.lua")):
            z.write(f, (CET_MODS_REL / MOD_NAME / f.name).as_posix())
        for name in ("LICENSE", "THIRD_PARTY_NOTICES.md"):
            z.write(root / name, (CET_MODS_REL / MOD_NAME / name).as_posix())
    print("packaged", out, f"({out.stat().st_size / 1e6:.1f} MB):")
    with zipfile.ZipFile(out) as z:
        for n in z.namelist():
            print("  ", n)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--project", type=Path, default=HERE.parent / "wolvenkit", help="WolvenKit project folder")
    ap.add_argument("--wolvenkit", help="path to WolvenKit.CLI.exe (default: WOLVENKIT_CLI, or on PATH)")
    ap.add_argument("--game", help="Cyberpunk 2077 folder (default: CYBERPUNK_DIR, or the usual places)")
    ap.add_argument("--no-install", action="store_true", help="build the project but don't touch the game")
    ap.add_argument("--skip-import", action="store_true",
                    help="build the project as it is (e.g. after editing the mod's Lua) without re-importing out/")
    ap.add_argument("--package", action="store_true", help="also write the release zip to ../dist")
    args = ap.parse_args()
    configure(args.project, args.wolvenkit, args.game, need_game=not args.no_install)
    if not args.skip_import:
        import_meshes()
        import_textures()
        for name, table in mesh_tables().items():
            setup_materials(name, table)
        add_colliders()
        add_parts()
        fix_amm_lua()
    build_and_install(install=not args.no_install)
    if args.package:
        package()


if __name__ == "__main__":
    main()
