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

For headless checks, import with `--headless --editor --import --path test_project` and run each scene with `--headless --path test_project --quit-after 90 SCENE.tscn`.

Use the target Godot executable to regenerate both files when changing engine builds. The files are local build inputs and are ignored by Git. The manual cross-platform GitHub Actions workflow still needs a step to supply the API files for its builds.

## Using the extension in games

- `../fire2` already uses `res://gde_test/`. For C++-only edits, close the game and editor, then copy the matching build from `test_project/gde_test/bin/linux/` to `../fire2/gde_test/bin/linux/`. On the first handoff of the 4.8 build, also update `compatibility_minimum` in the game's descriptor to `"4.8"`. Preserve its customized fire particle resources and `wooden_crate.tscn`.
- `../meleeguy0` has no installed `gde_test` or references to these native mechanic classes in the current copy. For a first integration, copy the extension folder to the game root so its `res://gde_test/` resource paths resolve, then wire up the nodes in a game scene.
