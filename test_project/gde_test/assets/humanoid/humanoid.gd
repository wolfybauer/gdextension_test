extends HumanoidRig3D

const JabHipCompensation = preload("jab_hip_compensation.gd")
const BatGrip = preload("bat_grip.gd")

const PLUS_Z_TORSO = preload("animations/lib3_all_torso.res")
const PLUS_Z_LEGS = preload("animations/lib3_all_legs.res")
const MINUS_Z_TORSO = preload("animations/lib3r_all_torso.res")
const MINUS_Z_LEGS = preload("animations/lib3r_all_legs.res")

@export var model_scene: PackedScene
@export var faces_positive_z := false
@export var torso_library: AnimationLibrary
@export var legs_library: AnimationLibrary
@export var chest_bone := &"Chest"
@export var hand_bone := &"RightHand"
@export var stow_bone := &"Spine"

var bat_grip: BatGrip

func _ready() -> void:
	if model_scene:
		load_model(model_scene, faces_positive_z)

func load_model(scene: PackedScene, positive_z: bool) -> bool:
	var upper := torso_library if torso_library else (PLUS_Z_TORSO if positive_z else MINUS_Z_TORSO)
	var lower := legs_library if legs_library else (PLUS_Z_LEGS if positive_z else MINUS_Z_LEGS)
	if not configure(scene, upper, lower, positive_z, chest_bone, hand_bone, stow_bone):
		return false
	model_scene = scene
	faces_positive_z = positive_z
	bat_grip = null
	var skeleton := get_skeleton()
	if lower.has_animation(&"punch_right_jab") and skeleton.find_bone(&"Hips") >= 0 and skeleton.find_bone(&"Spine") >= 0:
		JabHipCompensation.new().setup(self)
	return true

func set_bat_grip(weapon: Node3D) -> void:
	set_support_grip(weapon, Vector3(0, -0.06, 0), Vector3.UP)

func set_support_grip(weapon: Node3D, point: Vector3, axis: Vector3, wrist: Variant = null, elbow := Vector3.ZERO) -> void:
	if weapon and not is_instance_valid(bat_grip):
		bat_grip = BatGrip.new()
		bat_grip.setup(get_skeleton())
	if is_instance_valid(bat_grip):
		bat_grip.set_grip(weapon, point, axis, wrist, elbow)
