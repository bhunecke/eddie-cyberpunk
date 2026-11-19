# Eddie in Cyberpunk 2077

A Cyberpunk 2077 mod that adds Eddie to the game. Eddie is a mobile manipulator: a
holonomic base on four KELO drive units, with two Kinova Gen3 arms and Robotiq 2F-85
grippers. Spawn it next to V, have it follow you or drive it around, ride on it, and
move its arms. The model is converted from the robot's own description packages
([eddie_description](https://github.com/secorolab/eddie_description)), so the
dimensions, wheels and arm kinematics match the real robot.

- **Driving:** follow V, drive to where you look, or drive it by hand. It can strafe and
  turn on the spot, like the real base.
- **Wheels:** the drive units swivel like castors and the wheels roll at ground speed.
- **Riding:** stand on top of the torso and drive with W/A/S/D and the mouse.
- **Arms:** poses (rest, reach, arms up, wave), and grippers that open and close.
- **Collision:** V, NPCs and bullets collide with Eddie.

## Requirements

- Cyberpunk 2077 on PC, at a game version supported by the current Cyber Engine Tweaks
- [Cyber Engine Tweaks](https://www.nexusmods.com/cyberpunk2077/mods/107) (CET)

## Install

1. Download `eddie_robot-<version>.zip` from the
   [Releases](https://github.com/bhunecke/eddie-cyberpunk/releases) page.
2. Extract it into your Cyberpunk 2077 folder (the one that contains `archive` and
   `bin`), so that you end up with:
   ```
   Cyberpunk 2077/archive/pc/mod/eddie_robot.archive
   Cyberpunk 2077/bin/x64/plugins/cyber_engine_tweaks/mods/eddie_spawner/init.lua
   ```
   Vortex and Mod Organizer 2 can install the zip as it is.
3. Start the game and load a save.

To uninstall, delete `archive/pc/mod/eddie_robot.archive` and the
`bin/x64/plugins/cyber_engine_tweaks/mods/eddie_spawner` folder.

## Use

Open the CET overlay (the key you chose when you first started CET; `~` by default)
and use the **Eddie** window:

| Button | What it does |
|---|---|
| Spawn in front of V / Despawn all | The newest Eddie is the one you control |
| Follow V | Keeps about 2 m from V and faces V |
| Go where I look | Drives to the spot under the crosshair (up to 40 m) |
| Manual | Drive with the keys bound to the `eddie_*` inputs (see below) |
| Stop | Stops driving |
| Ride / Get off | V stands on top of the torso (walk within 10 m first). **W/A/S/D** drive relative to where you look, the **mouse** steers, **Jump** gets off |
| Speed | 0.2–2 m/s |
| Rest, Reach, Arms up, Wave | Arm poses |
| Close / Open grippers | Grippers |

Under **Bindings → eddie_spawner** in the CET overlay you can bind:

- **Hotkeys:** spawn, despawn, follow, go where I look, stop, ride / get off, each arm
  pose, and grippers.
- **Inputs** (hold to drive): forward, back, strafe left / right, turn left / right.
  Pressing one switches to Manual.

### Known limitations

- Eddie moves kinematically: it slides along the floor with ground and wall probes and
  doesn't get pushed around by physics. It stops at walls, drops and steps over 4 cm.
- The arm collision boxes stay in the rest pose.
- While riding, only Eddie's base is checked against walls, so V's head can pass through
  low ceilings.
- If you save while riding, V can't walk after that save is loaded. To fix it, press the
  ride hotkey or button with no Eddie spawned.

## Building it yourself

The model is generated from the robot description packages by
[`cyberpunk/build_glb.py`](cyberpunk/build_glb.py). It's then imported into the WolvenKit
project in [`wolvenkit/`](wolvenkit) and packed by
[`cyberpunk/install_wolvenkit.py`](cyberpunk/install_wolvenkit.py). See
[cyberpunk/README.md](cyberpunk/README.md) for prerequisites, the build steps, how to
tune poses and materials, and the tests.

If you only change the mod's Lua (`wolvenkit/source/resources/.../eddie_spawner/init.lua`),
you don't need the Python build: copy the file into your game's CET mods folder, or run
`install_wolvenkit.py --skip-import` to rebuild the project as it is.

## Licence

The code in this repository is MIT licensed (see [LICENSE](LICENSE)). The robot meshes
come from the packages listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md), under
their own licences. This is an unofficial fan mod. Cyberpunk 2077 is a trademark of
CD PROJEKT S.A., which doesn't endorse this mod.
