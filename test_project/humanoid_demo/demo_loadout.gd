extends RefCounted

const MODELS := [
	{"scene": preload("res://humanoid_demo/psx_characters/extras/male/Character_Sheriff_N.fbx"), "name": "Sheriff", "positive_z": true},
	{"scene": preload("res://humanoid_demo/generic_guy.glb"), "name": "Generic", "positive_z": true},
	{"scene": preload("res://humanoid_demo/skullguy1.glb"), "name": "Skullguy", "positive_z": false},
]
const GRIP_FILE := "user://humanoid_grips.cfg"

const WEAPONS := [
	{"scene": preload("res://humanoid_demo/pistol.glb"), "muzzle": Vector3(0, 0.082, -0.217), "name": "Pistol", "stance": &"pistol", "cooldown": 0.2, "automatic": false, "recoil": 0.32, "spread": 1.0},
	{"scene": preload("res://humanoid_demo/revolver.glb"), "muzzle": Vector3(0, 0.085, -0.313), "name": "Revolver", "stance": &"pistol", "cooldown": 0.3, "automatic": false, "recoil": 0.42, "spread": 1.5},
	{"scene": preload("res://humanoid_demo/shotgun.glb"), "muzzle": Vector3(0, 0.044, -0.785), "name": "Shotgun", "stance": &"twohand", "cooldown": 0.35, "automatic": false, "recoil": 0.52, "spread": 2.0},
	{"scene": preload("res://humanoid_demo/smg.glb"), "muzzle": Vector3(0, 0.087, -0.371), "name": "SMG", "stance": &"pistol", "cooldown": 0.1, "automatic": true, "recoil": 0.24, "spread": 2.5},
	{"scene": preload("res://humanoid_demo/knife.glb"), "name": "Knife", "stance": &"melee", "cooldown": 0.2, "automatic": false, "recoil": 0.0},
	{"scene": preload("res://humanoid_demo/bat/wooden_bat.tscn"), "name": "Wooden Bat", "stance": &"bat", "cooldown": 0.0, "automatic": false, "recoil": 0.0},
	{"scene": preload("res://humanoid_demo/grenade.tscn"), "name": "Grenade", "stance": &"carry", "cooldown": 0.0, "automatic": false, "recoil": 0.0},
]
# Mounts are Skullguy grip/stow, then generic grip/stow.
const WEAPON_MOUNTS := {
	&"carry": [
		[Vector3(0.01, -0.09, 0.025), Vector3.ZERO],
		[Vector3.ZERO, Vector3.ZERO],
		[Vector3(-0.01, 0.09, 0.025), Vector3.ZERO],
		[Vector3.ZERO, Vector3.ZERO],
	],
	&"bat": [
		[Vector3(0.003, -0.058, 0.002), Vector3(2.18, 0, 1.02)],
		[Vector3(0.13, -0.30, 0.15), Vector3(0, 0, -0.5)],
		[Vector3(-0.005, 0.058, -0.002), Vector3(0.96, 0, -1.02)],
		[Vector3(-0.13, -0.30, -0.12), Vector3(0.23, 0, 0.38)],
	],
	&"melee": [
		[Vector3(0.0180969, -0.0501575, 0.024302), Vector3(-0.732574, 2.0172, -0.605067)],
		[Vector3(-0.186177, -0.092466, -0.0394792), Vector3(-0.931017, 2.99635, 0.242755)],
		[Vector3(-0.0192128, 0.0666303, 0.0332389), Vector3(-0.0461587, -1.45172, 2.09839)],
		[Vector3(0.190635, -0.0629065, 0.0637274), Vector3(-0.521793, -0.0928114, 0.00531627)],
	],
	&"pistol": [
		[Vector3(-0.0134877, -0.0914844, 0.0357493), Vector3(-1.29335, -2.36534, -2.24928)],
		[Vector3(0.19507, -0.101801, 0.0315023), Vector3(-0.933135, 2.96802, 2.95033)],
		[Vector3(-0.00806117, 0.0938622, 0.0486137), Vector3(1.35223, -2.99427, 1.73925)],
		[Vector3(-0.210638, -0.194035, -0.0325694), Vector3(-1.1832, 1.5865, 1.45052)],
	],
	&"twohand": [
		[Vector3(-0.0710149, -0.0351743, 0.00805993), Vector3(-1.41816, -1.71328, -2.89253)],
		[Vector3(0.0919104, 0.304443, 0.140105), Vector3(-1.1174, -1.52794, 0.149945)],
		[Vector3(0.0766877, 0.0590649, 0.0290658), Vector3(1.54858, 0.329305, -1.156)],
		[Vector3(-0.144882, 0.25231, -0.148223), Vector3(-0.997982, -1.39961, -0.054284)],
	],
}
const WEAPON_STYLES := {
	&"carry": HumanoidController3D.UNARMED,
	&"pistol": HumanoidController3D.PISTOL,
	&"twohand": HumanoidController3D.LONG_GUN,
	&"melee": HumanoidController3D.KNIFE,
	&"bat": HumanoidController3D.BAT,
}

static func default_grip_profile(setup: Dictionary, positive_z: bool) -> Dictionary:
	var index := 2 if positive_z else 0
	var mounts: Array = WEAPON_MOUNTS[setup.stance]
	var grip: Array = mounts[index]
	var stow: Array = mounts[index + 1]
	var shotgun: bool = setup.stance == &"twohand"
	return {
		"hand_position": grip[0], "hand_rotation": grip[1] * 180.0 / PI,
		"stow_position": stow[0], "stow_rotation": stow[1] * 180.0 / PI,
		"support_position": Vector3(0, -0.04, -0.30) if shotgun else Vector3(0, -0.06, 0),
		"support_rotation": Vector3(-86, 134, -178) if positive_z else Vector3(-83.2, 36.9, 96.5),
		"support_fixed": shotgun, "elbow_offset": Vector3.ZERO,
	}

static func read_grip_profile(model: PackedScene, setup: Dictionary, positive_z: bool) -> Dictionary:
	var profile := default_grip_profile(setup, positive_z)
	var file := ConfigFile.new()
	if model and file.load(GRIP_FILE) == OK:
		var section := model.resource_path + ":" + String(setup.name)
		for key in profile:
			var saved = file.get_value(section, key, profile[key])
			if typeof(saved) == typeof(profile[key]):
				profile[key] = saved
	return profile

static func save_grip_profile(model: PackedScene, setup: Dictionary, profile: Dictionary) -> Error:
	var file := ConfigFile.new()
	var result := file.load(GRIP_FILE)
	if result != OK and result != ERR_FILE_NOT_FOUND:
		return result
	var section := model.resource_path + ":" + String(setup.name)
	for key in profile:
		file.set_value(section, key, profile[key])
	return file.save(GRIP_FILE)

static func apply_support_grip(rig: HumanoidRig3D, weapon: Node3D, profile: Dictionary, stance: StringName) -> void:
	var wrist = Basis.from_euler(profile.support_rotation * PI / 180.0) if profile.support_fixed else null
	var axis := Vector3.FORWARD if stance == &"twohand" else Vector3.UP
	rig.set_support_grip(weapon, profile.support_position, axis, wrist, profile.elbow_offset)

static func apply_mounts(rig: HumanoidRig3D, weapon: Node3D, profile: Dictionary) -> void:
	rig.configure_aim_weapon(weapon,
		Transform3D(Basis.from_euler(profile.hand_rotation * PI / 180.0), profile.hand_position),
		Transform3D(Basis.from_euler(profile.stow_rotation * PI / 180.0), profile.stow_position))

static func mount_weapon(rig: HumanoidRig3D, controller: HumanoidController3D,
		weapon: Node3D, setup: Dictionary, positive_z: bool, profile := {}) -> void:
	if profile.is_empty():
		profile = read_grip_profile(rig.model_scene, setup, positive_z)
	apply_mounts(rig, weapon, profile)
	controller.configure_weapon(weapon, WEAPON_STYLES[setup.stance],
		setup.cooldown, setup.automatic, setup.recoil, not positive_z, setup.get("spread", 0.0))
