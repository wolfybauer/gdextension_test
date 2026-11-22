extends RefCounted

const WEAPONS := [
	{"scene": preload("res://humanoid_demo/pistol.glb"), "muzzle": Vector3(0, 0.082, -0.217), "name": "Pistol", "stance": &"pistol", "cooldown": 0.2, "automatic": false, "recoil": 0.32, "spread": 1.0},
	{"scene": preload("res://humanoid_demo/revolver.glb"), "muzzle": Vector3(0, 0.085, -0.313), "name": "Revolver", "stance": &"pistol", "cooldown": 0.3, "automatic": false, "recoil": 0.42, "spread": 1.5},
	{"scene": preload("res://humanoid_demo/shotgun.glb"), "muzzle": Vector3(0, 0.044, -0.785), "name": "Shotgun", "stance": &"twohand", "cooldown": 0.35, "automatic": false, "recoil": 0.52, "spread": 2.0},
	{"scene": preload("res://humanoid_demo/smg.glb"), "muzzle": Vector3(0, 0.087, -0.371), "name": "SMG", "stance": &"pistol", "cooldown": 0.1, "automatic": true, "recoil": 0.24, "spread": 2.5},
	{"scene": preload("res://humanoid_demo/knife.glb"), "name": "Knife", "stance": &"melee", "cooldown": 0.2, "automatic": false, "recoil": 0.0},
	{"scene": preload("res://humanoid_demo/bat/wooden_bat.tscn"), "name": "Wooden Bat", "stance": &"bat", "cooldown": 0.0, "automatic": false, "recoil": 0.0},
]
# Mounts are Skullguy grip/stow, then generic grip/stow.
const WEAPON_MOUNTS := {
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
	&"pistol": HumanoidController3D.PISTOL,
	&"twohand": HumanoidController3D.LONG_GUN,
	&"melee": HumanoidController3D.KNIFE,
	&"bat": HumanoidController3D.BAT,
}

static func mount_weapon(rig: HumanoidRig3D, controller: HumanoidController3D,
		weapon: Node3D, setup: Dictionary, positive_z: bool) -> void:
	var index := 2 if positive_z else 0
	var mounts: Array = WEAPON_MOUNTS[setup.stance]
	var grip: Array = mounts[index]
	var stow: Array = mounts[index + 1]
	rig.configure_aim_weapon(weapon,
		Transform3D(Basis.from_euler(grip[1]), grip[0]),
		Transform3D(Basis.from_euler(stow[1]), stow[0]))
	controller.configure_weapon(weapon, WEAPON_STYLES[setup.stance],
		setup.cooldown, setup.automatic, setup.recoil, not positive_z, setup.get("spread", 0.0))
