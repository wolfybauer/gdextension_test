# GDExtension development

- Native code lives in `src/`. `src/register_types.cpp` registers the Godot classes.
- `src/fade/` implements `FadeWall3D` and `FadeFloor3D`. `src/fire.cpp` implements `FireComponent3D`. `src/destronoi.cpp` implements `DestronoiNode` for runtime fracturing.
- The pinned `godot-cpp` submodule is `10.0.0-stable`. The build uses the API and interface dumped from Godot 4.8 beta 4.

## Build and test

From the repo root, run `scons -j4 compiledb=yes`. The default target is `template_debug`. SCons updates `compile_commands.json` and installs the shared library into `test_project/gde_test/bin/linux/` as part of the default build. Use `scons -j4 target=template_release` when preparing a Linux release export.

The local toolchain is Python 3.14 and SCons 4.11.1. Activate it with `source .venv/bin/activate`. In a fresh checkout, initialize the submodule, recreate the environment, and generate the API files:

```sh
git submodule update --init --recursive
python3 -m venv --without-pip .venv
python3 -m pip --python .venv install SCons==4.11.1
mkdir -p .godot-api
(
    cd .godot-api
    ~/00_apps/Godot_v4.8-beta4_linux.x86_64 --headless \
        --dump-extension-api --dump-gdextension-interface-json
)
```

Open the editor with `~/00_apps/Godot_v4.8-beta4_linux.x86_64 --editor --path test_project`. F6 runs the open scene. F5 runs `test_fire_cpp.tscn`.

| Scene | What to check |
| --- | --- |
| `test_fade_cpp.tscn` | Move with WASD and toggle between orthographic and perspective cameras. Check walls and floors as they pass between the player and camera. |
| `test_fire_cpp.tscn` | Press Enter or Space to ignite the first crate. Press Up to fracture the three sample bodies. Press R to reset. |
| `test_interact_cpp.tscn` | Move with WASD and check wall fading around the player and camera pivot. The last fade-wall refactor already called out odd behavior in this scene. |
| `humanoid_demo/humanoid_demo.tscn` | WASD moves and sets facing when there is no arrow-key aim. Shift jumps, Space attacks (hold for the SMG), E reloads firearms and Tab swaps models. Aiming equips the weapon, and releasing aim stows it. Keys 1–7 select pistol, revolver, shotgun, SMG, knife, wooden bat, and grenade. Hold G for a fixed-angle throw preview and release G to throw the held item. F picks up a tossed weapon or uses a nearby interaction prop. Use F at the orange switch to light the nearby crate pile. H releases the yard hazards. The imported `meleeguy0` layout has upper walkways and native wall and floor fades. R spawns another randomized enemy. Shoot or strike the enemies to test their hit reaction and defeat animation. |

For headless checks, import with `--headless --editor --import --path test_project` and run each scene with `--headless --path test_project --quit-after 90 SCENE.tscn`.

Use the target Godot executable to regenerate both files when changing engine builds. The files are local build inputs and are ignored by Git. The manual cross-platform GitHub Actions workflow still needs a step to supply the API files for its builds.

## Humanoid rig

Instance `res://gde_test/assets/humanoid/humanoid.tscn` under the character body and assign a model scene. The bundled libraries target the humanoid-retargeted `%GeneralSkeleton` with separate torso and leg tracks. Skullguy uses the negative-Z profile. The generic humanoid uses the positive-Z profile. For a different rig, supply matching animation libraries and set the chest, hand, and stow bone names in the Inspector.

`HumanoidRig3D` provides independent `play_torso` and `play_legs` calls, directional `play_locomotion`, chest aiming through `set_aim_target`, and `mount_weapon` for manual attachment. Use `configure_aim_weapon` with the hand and stow transforms, then pass the actor’s aim intent to `set_weapon_aiming` for automatic attachment. The `torso_animation_finished` signal reports the qualified clip name when a torso animation finishes. `HumanoidController3D` owns movement, aim profiles, animation selection, recoil, knockback, and action transitions. Connect it to a `RigidCharacterBody3D` and its rig with `configure`. The controller also accepts `CharacterBody3D` for existing scenes. From the player's input script or an NPC's decision loop, submit world-XZ movement and aim with `set_input`, call `step(delta)` once per physics tick, and use `attack`, `reload`, `jump`, or `play_reaction` for actions. The demo uses `fire` and `melee_attack` signals to perform scene hit checks, damage, and effects. Call `apply_knockback(direction, speed)` for a raw impulse or `react_to_hit(direction, speed, recovery_seconds)` for a stagger and an impulse with repeat-hit protection. Weapon setup accepts optional horizontal spread in degrees, and `set_aim_error` adds per-actor spread. Pass `false` as the second argument to `step` to continue physics while a defeat animation plays. The bat's aim profile uses its authored chest pose, with a separate grip modifier in the shared rig scene.

The controller provides `throw_item(item)` and emits `item_released(item, direction)` at the throw clip’s release frame. The game handles the released item’s physics and effects. Use `interact(clip, direction)` for pickup, button, door, push, and talk, and connect `interaction_contact(clip)` to the game’s interaction handler. The demo uses `carry` while holding its grenade.

The old animation sources are in the attached-drive `77_blender` and `77_new_blender` backups. `lib2/raw2` and `lib3/raw` contain the source FBXs, and the neighboring Blender and GLB files hold the assembled libraries. `scripts/armature_rotate.py` contains the rotated-armature bake. In `meleeguy0`, `tools/anim_lib_splitter.gd` derives the torso from the Spine hierarchy and the lower body from Hips and both leg hierarchies, then saves separate Godot libraries with the other layer's tracks disabled.

Off-aim attacks use the held weapon. Handguns bash with the right-jab clip, and long guns use `smash_twohand`, retargeted from `Smash.fbx` for both facing profiles. The controller exposes `weapon_equipped_changed` for pose modifiers that follow the hand mount.

Press K in the humanoid demo to open the grip tuner. It previews the current pose at the scrubber time and saves per-model, per-weapon fits to `user://humanoid_grips.cfg`. The demo loads the saved fit when equipping the weapon.

The bat uses `aim_bat` and `swing_bat` in both facing profiles, retargeted from `Baseball Idle.fbx` and `Great Sword Slash.fbx`. The bat follows the right-hand socket, with a left-hand IK target on the grip.

The demo rolls a model from `humanoid_demo/psx_characters/` and a weapon profile independently at each spawn. `demo_enemy.gd` sets attack range, spacing, and cadence for each profile. The demo root exposes `enemy_aim_error_scale`, `enemy_attack_interval_scale`, and `player_hit_recovery` for pacing and difficulty. `demo_combat.gd` defines the shared melee arcs and cover check. The recovered `palette_test2` models and palette randomizer are still available under `humanoid_demo/enemies/`.

Set `impact_energy_threshold` and `safe_landing_speed` on a `RigidCharacterBody3D` to opt into native prop and fall damage. Connect `impact_damage(damage, direction, landing)` to the actor’s health handler. The physics solver already supplies the impulse for prop hits.

## Blood effects

Instance `res://gde_test/assets/blood/blood_controller.tscn` under the actor and call `splatter(direction)` from the damage handler. Particle bursts and stains are added to the current scene and expire independently of the actor. The stain is a horizontal textured mesh for the Compatibility renderer. Use `stain_y_offset` to align it with the ground when the actor's origin is raised.

## Using the extension in games

- `../fire2` already uses `res://gde_test/`. For C++-only edits, close the game and editor, then copy the matching build from `test_project/gde_test/bin/linux/` to `../fire2/gde_test/bin/linux/`. On the first handoff of the 4.8 build, also update `compatibility_minimum` in the game's descriptor to `"4.8"`. Preserve its customized fire particle resources and `wooden_crate.tscn`.
- `../meleeguy0` has no installed `gde_test` or references to these native mechanic classes in the current copy. For a first integration, copy the extension folder to the game root so its `res://gde_test/` resource paths resolve, then wire up the nodes in a game scene.
