extends RefCounted

const SHOTGUN_SPREAD := [-27.0, -13.0, -3.0, 3.0, 13.0, 27.0]

static func trace(attacker: PhysicsBody3D, origin: Vector3, endpoint: Vector3) -> Dictionary:
	var query := PhysicsRayQueryParameters3D.create(origin, endpoint, 3, [attacker.get_rid()])
	return attacker.get_world_3d().direct_space_state.intersect_ray(query)

static func melee_reach(style: int) -> float:
	match style:
		HumanoidController3D.BAT, HumanoidController3D.LONG_GUN:
			return 2.1
		HumanoidController3D.KNIFE, HumanoidController3D.PISTOL:
			return 1.65
		_:
			return 1.45

static func melee_damage(style: int) -> int:
	match style:
		HumanoidController3D.UNARMED:
			return 1
		HumanoidController3D.LONG_GUN:
			return 3
		_:
			return 2

static func melee_targets(attacker: PhysicsBody3D, direction: Vector3, style: int,
		target_mask: int) -> Dictionary[PhysicsBody3D, Vector3]:
	var reach := melee_reach(style)
	var half_angle := 55.0 if style in [HumanoidController3D.BAT, HumanoidController3D.LONG_GUN] else 40.0
	var origin := attacker.global_position + Vector3.UP * 1.05
	var volume := SphereShape3D.new()
	volume.radius = reach
	var query := PhysicsShapeQueryParameters3D.new()
	query.shape = volume
	query.transform.origin = origin
	query.collision_mask = target_mask
	query.exclude = [attacker.get_rid()]

	var space := attacker.get_world_3d().direct_space_state
	var targets: Dictionary[PhysicsBody3D, Vector3] = {}
	for overlap in space.intersect_shape(query):
		var target := overlap.collider as PhysicsBody3D
		if not target or targets.has(target):
			continue
		var center := target.global_position + Vector3.UP * 1.05
		var offset := center - origin
		var outward := Vector3(offset.x, 0, offset.z).normalized()
		if absf(offset.y) > 1.1 or direction.dot(outward) < cos(deg_to_rad(half_angle)):
			continue
		var cover := PhysicsRayQueryParameters3D.create(origin, center, 1,
			[attacker.get_rid(), target.get_rid()])
		if space.intersect_ray(cover).is_empty():
			targets[target] = outward
	return targets
