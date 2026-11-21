extends HumanoidRig3D

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
	return true
