extends Node3D

const Loadout = preload("demo_loadout.gd")
const VECTOR_FIELDS := {
	"hand_position": "Right-hand offset",
	"hand_rotation": "Right-hand rotation (°)",
	"stow_position": "Stow offset",
	"stow_rotation": "Stow rotation (°)",
	"support_position": "Left palm on weapon",
	"support_rotation": "Left wrist on weapon (°)",
	"elbow_offset": "Left elbow IK offset",
}

@onready var rig = $Humanoid
@onready var panel: VBoxContainer = $Interface/Panel/Margin/Fields
var model_picker := OptionButton.new()
var weapon_picker := OptionButton.new()
var pose_picker := OptionButton.new()
var scrub := HSlider.new()
var fixed_wrist := CheckBox.new()
var status := Label.new()
var fields: Dictionary = {}
var profile: Dictionary
var setup: Dictionary
var selected_model: Dictionary
var weapon: Node3D
var contact_marker: MeshInstance3D
var clips: Array[StringName] = []
var rebuilding := false

func _ready() -> void:
	_row("Model", model_picker)
	for model in Loadout.MODELS:
		model_picker.add_item(model.name)
	model_picker.item_selected.connect(func(_index: int) -> void: _load_selection())
	_row("Weapon", weapon_picker)
	for item in Loadout.WEAPONS:
		weapon_picker.add_item(item.name)
	weapon_picker.select(2)
	weapon_picker.item_selected.connect(func(_index: int) -> void: _load_selection())
	_row("Pose", pose_picker)
	pose_picker.item_selected.connect(func(_index: int) -> void: _pose())
	scrub.step = 0.005
	_row("Time (s)", scrub)
	scrub.value_changed.connect(_sample)
	var turn := HSlider.new()
	turn.min_value = -180.0
	turn.max_value = 180.0
	turn.step = 1.0
	turn.set_value_no_signal(rig.rotation_degrees.y)
	_row("Turn model (°)", turn)
	turn.value_changed.connect(func(value: float) -> void: rig.rotation_degrees.y = value)
	for key in VECTOR_FIELDS:
		var box := HBoxContainer.new()
		var components: Array[SpinBox] = []
		for axis in 3:
			var spin := SpinBox.new()
			spin.min_value = -360.0 if key.ends_with("rotation") else -1.0
			spin.max_value = 360.0 if key.ends_with("rotation") else 1.0
			spin.step = 0.5 if key.ends_with("rotation") else 0.005
			spin.prefix = ["X", "Y", "Z"][axis]
			spin.custom_minimum_size.x = 81
			spin.value_changed.connect(_change_component.bind(key, axis))
			box.add_child(spin)
			components.append(spin)
		fields[key] = components
		_row(VECTOR_FIELDS[key], box)
	fixed_wrist.text = "Lock left wrist to weapon"
	fixed_wrist.toggled.connect(func(on: bool) -> void:
		if not rebuilding:
			profile.support_fixed = on
			_apply_fit())
	panel.add_child(fixed_wrist)
	status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	panel.add_child(status)
	var buttons := HBoxContainer.new()
	panel.add_child(buttons)
	_button(buttons, "Save fit", _save)
	_button(buttons, "Defaults", _defaults)
	_button(buttons, "Back to demo", _return_to_demo)
	_load_selection()

func _row(label: String, control: Control) -> void:
	var row := HBoxContainer.new()
	var caption := Label.new()
	caption.text = label
	caption.custom_minimum_size.x = 173
	control.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(caption)
	row.add_child(control)
	panel.add_child(row)

func _button(parent: Container, title: String, callback: Callable) -> void:
	var button := Button.new()
	button.text = title
	button.pressed.connect(callback)
	parent.add_child(button)

func _load_selection() -> void:
	if weapon:
		rig.set_bat_grip(null)
		weapon.free()
	selected_model = Loadout.MODELS[model_picker.selected]
	setup = Loadout.WEAPONS[weapon_picker.selected]
	rig.load_model(selected_model.scene, selected_model.positive_z)
	weapon = setup.scene.instantiate()
	profile = Loadout.read_grip_profile(selected_model.scene, setup, selected_model.positive_z)
	contact_marker = MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = 0.018
	sphere.height = 0.036
	var material := StandardMaterial3D.new()
	material.albedo_color = Color(0.22, 0.9, 0.95)
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.no_depth_test = true
	sphere.material = material
	contact_marker.mesh = sphere
	weapon.add_child(contact_marker)
	pose_picker.clear()
	clips.clear()
	_add_pose("Stowed", &"idle_unarmed")
	var aim := &"carry" if setup.stance == &"carry" else StringName("aim_" + String(setup.stance))
	_add_pose("In hand", aim)
	match setup.stance:
		&"twohand":
			_add_pose("Smash", &"smash_twohand")
		&"bat":
			_add_pose("Swing", &"swing_bat")
		&"pistol", &"melee":
			_add_pose("Bash", &"punch_right_jab")
	pose_picker.select(clips.size() - 1)
	_rebuild_fields()
	status.text = ""
	_pose()

func _add_pose(label: String, clip: StringName) -> void:
	clips.append(clip)
	pose_picker.add_item(label)

func _pose() -> void:
	var clip := clips[pose_picker.selected]
	rig.play_torso(clip, 0)
	var legs: AnimationPlayer = rig.get_legs_player()
	if not rig.play_legs(clip, 0):
		rig.play_legs(&"idle_unarmed", 0)
	var torso: AnimationPlayer = rig.get_torso_player()
	torso.advance(0.0)
	legs.advance(0.0)
	torso.pause()
	legs.pause()
	scrub.max_value = torso.current_animation_length
	scrub.set_value_no_signal(0.0)
	_sample(0.0)
	_apply_fit()

func _sample(time: float) -> void:
	var torso: AnimationPlayer = rig.get_torso_player()
	var legs: AnimationPlayer = rig.get_legs_player()
	torso.seek(time, true)
	legs.seek(minf(time, legs.current_animation_length), true)

func _apply_fit() -> void:
	var equipped := pose_picker.selected != 0
	Loadout.apply_mounts(rig, weapon, profile)
	rig.set_weapon_aiming(equipped)
	var support: bool = equipped and (setup.stance == &"bat" or setup.stance == &"twohand")
	if support:
		Loadout.apply_support_grip(rig, weapon, profile, setup.stance)
	else:
		rig.set_bat_grip(null)
	contact_marker.position = profile.support_position
	contact_marker.visible = support

func _rebuild_fields() -> void:
	rebuilding = true
	for key in fields:
		for axis in 3:
			fields[key][axis].set_value_no_signal(profile[key][axis])
	fixed_wrist.set_pressed_no_signal(profile.support_fixed)
	rebuilding = false

func _change_component(value: float, key: String, axis: int) -> void:
	if rebuilding:
		return
	var vector: Vector3 = profile[key]
	vector[axis] = value
	profile[key] = vector
	_apply_fit()

func _defaults() -> void:
	profile = Loadout.default_grip_profile(setup, selected_model.positive_z)
	_rebuild_fields()
	_apply_fit()
	status.text = "Defaults previewed. Save to use them in the demo."

func _save() -> void:
	var result := Loadout.save_grip_profile(selected_model.scene, setup, profile)
	status.text = "Saved for %s · %s" % [selected_model.name, setup.name] if result == OK else error_string(result)

func _return_to_demo() -> void:
	get_tree().change_scene_to_file("res://humanoid_demo/humanoid_demo.tscn")
