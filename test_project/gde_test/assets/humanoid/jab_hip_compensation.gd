extends SkeletonModifier3D

var torso: AnimationPlayer
var jab_legs: Animation
var hips_track := -1
var hips := -1
var spine := -1
var hips_parent := -1
var hips_parent_track := -1

func setup(rig: HumanoidRig3D) -> void:
	var skeleton := rig.get_skeleton()
	torso = rig.get_torso_player()
	jab_legs = rig.get_legs_player().get_animation(&"main/punch_right_jab")
	hips = skeleton.find_bone("Hips")
	spine = skeleton.find_bone("Spine")
	hips_parent = skeleton.get_bone_parent(hips)
	for track in jab_legs.get_track_count():
		var path := jab_legs.track_get_path(track)
		if jab_legs.track_get_type(track) != Animation.TYPE_ROTATION_3D:
			continue
		var bone := path.get_concatenated_subnames()
		if bone == &"Hips":
			hips_track = track
		elif hips_parent >= 0 and bone == skeleton.get_bone_name(hips_parent):
			hips_parent_track = track
	skeleton.add_child(self)

func _process_modification() -> void:
	if hips_track < 0 or torso.current_animation != &"main/punch_right_jab":
		return
	var skeleton := get_skeleton()
	# Preserve the jab's authored hip-to-spine orientation over the live leg pose.
	var time := torso.current_animation_position
	var authored_hips := jab_legs.rotation_track_interpolate(hips_track, time)
	if hips_parent >= 0:
		var authored_parent := skeleton.get_bone_rest(hips_parent).basis.get_rotation_quaternion()
		if hips_parent_track >= 0:
			authored_parent = jab_legs.rotation_track_interpolate(hips_parent_track, time)
		authored_hips = authored_parent * authored_hips
	var live_hips := skeleton.get_bone_global_pose(hips).basis.orthonormalized().get_rotation_quaternion()
	var live_spine := skeleton.get_bone_pose_rotation(spine)
	skeleton.set_bone_pose_rotation(spine, (live_hips.inverse() * authored_hips * live_spine).normalized())
