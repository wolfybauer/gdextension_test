extends SkeletonModifier3D

class PreserveWrist extends SkeletonModifier3D:
	var bone: int
	var authored_basis := Basis.IDENTITY

	func _process_modification() -> void:
		var skeleton := get_skeleton()
		var forearm := skeleton.get_bone_global_pose(skeleton.get_bone_parent(bone)).basis
		var local := (forearm.inverse() * authored_basis).orthonormalized()
		skeleton.set_bone_pose_rotation(bone, local.get_rotation_quaternion())

var weapon: Node3D
var ik: TwoBoneIK3D
var wrists: Array[PreserveWrist] = []
var grips: Array[Node3D] = []
var poles: Array[Node3D] = []
var shoulders: Array[int] = []
var elbows: Array[int] = []
var arm_reach: Array[float] = []
var hands: Array[int] = []
var support_palm_offset := Vector3.ZERO

func setup(skeleton: Skeleton3D) -> void:
	skeleton.add_child(self)
	ik = TwoBoneIK3D.new()
	ik.active = false
	skeleton.add_child(ik)
	ik.setting_count = 2
	for index in 2:
		var side := "Right" if index == 0 else "Left"
		var shoulder := skeleton.find_bone(side + "UpperArm")
		var elbow := skeleton.find_bone(side + "LowerArm")
		var hand := skeleton.find_bone(side + "Hand")
		hands.append(hand)
		shoulders.append(shoulder)
		elbows.append(elbow)
		arm_reach.append(skeleton.get_bone_rest(elbow).origin.length() + skeleton.get_bone_rest(hand).origin.length() - 0.015)
		var grip := Node3D.new()
		skeleton.add_child(grip)
		grips.append(grip)
		var pole := Node3D.new()
		skeleton.add_child(pole)
		poles.append(pole)
		ik.set_root_bone(index, shoulder)
		ik.set_middle_bone(index, elbow)
		ik.set_end_bone(index, hand)
		ik.set_target_node(index, ik.get_path_to(grip))
		ik.set_pole_node(index, ik.get_path_to(pole))
	var knuckle := skeleton.find_bone("LeftIndexProximal")
	if knuckle >= 0:
		var hand_rest := skeleton.get_bone_global_rest(hands[1])
		support_palm_offset = hand_rest.affine_inverse() * skeleton.get_bone_global_rest(knuckle).origin * 0.65
	for hand in hands:
		var wrist := PreserveWrist.new()
		wrist.bone = hand
		skeleton.add_child(wrist)
		wrists.append(wrist)
	set_bat(null)

func set_bat(bat: Node3D) -> void:
	weapon = bat
	active = weapon != null
	ik.active = active
	for wrist in wrists:
		wrist.active = active

func _process_modification() -> void:
	if not is_instance_valid(weapon):
		return
	var skeleton := get_skeleton()
	var hand_pose := skeleton.get_bone_global_pose(hands[0])
	wrists[0].authored_basis = hand_pose.basis
	var bat_pose := hand_pose * weapon.transform
	var support_basis := skeleton.get_bone_global_pose(hands[1]).basis.orthonormalized()
	var palm_direction := (support_basis * support_palm_offset).normalized()
	var across_handle := palm_direction.slide(bat_pose.basis.y.normalized()).normalized()
	if not palm_direction.is_zero_approx() and not across_handle.is_zero_approx():
		support_basis = Basis(Quaternion(palm_direction, across_handle)) * support_basis
	wrists[1].authored_basis = support_basis
	# Place the palm on the handle, leaving room between it and the lead hand.
	var support_palm := bat_pose * Vector3(0, -0.06, 0)
	var spacing := support_palm - support_basis * support_palm_offset - hand_pose.origin
	var grip_position := hand_pose.origin
	var right := skeleton.get_bone_global_pose(shoulders[0]).origin
	var left := skeleton.get_bone_global_pose(shoulders[1]).origin
	# Shift both grips together into reach while retaining the authored swing rotation.
	for pass_index in 3:
		grip_position = left + (grip_position + spacing - left).limit_length(arm_reach[1]) - spacing
		grip_position = right + (grip_position - right).limit_length(arm_reach[0])
	grips[0].position = grip_position
	grips[1].position = grip_position + spacing
	for index in 2:
		poles[index].position = skeleton.get_bone_global_pose(elbows[index]).origin + Vector3.DOWN * 0.2
