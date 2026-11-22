extends Node3D

signal exploded(center: Vector3)

const SPEED := 5.4
const LIFT := 3.4
const RADIUS := 0.13
const DOT_SPACING := 5
const DOT_COUNT := 24
const PREVIEW_COLOR := Color(1.0, 0.76, 0.32)

var path_dots: MultiMeshInstance3D
var marker: MeshInstance3D

func _ready() -> void:
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = PREVIEW_COLOR
	var dot := SphereMesh.new()
	dot.radius = 0.035
	dot.height = 0.07
	dot.radial_segments = 8
	dot.rings = 4
	dot.material = material
	path_dots = MultiMeshInstance3D.new()
	var dots := MultiMesh.new()
	dots.transform_format = MultiMesh.TRANSFORM_3D
	dots.mesh = dot
	dots.instance_count = DOT_COUNT
	dots.visible_instance_count = 0
	path_dots.multimesh = dots
	add_child(path_dots)
	var ring := TorusMesh.new()
	ring.inner_radius = 0.16
	ring.outer_radius = 0.20
	ring.rings = 20
	ring.ring_segments = 6
	ring.material = material
	marker = MeshInstance3D.new()
	marker.mesh = ring
	marker.hide()
	add_child(marker)

func _origin(actor: CharacterBody3D, direction: Vector3) -> Vector3:
	var chest := actor.global_position + Vector3.UP * 1.55
	var hand := chest + direction * 0.60
	var hit := _trace(actor, chest, hand)
	return hand if hit.is_empty() else hit.position - direction * RADIUS

func _velocity(direction: Vector3) -> Vector3:
	return direction * SPEED + Vector3.UP * LIFT

func _trace(actor: CharacterBody3D, start: Vector3, end: Vector3) -> Dictionary:
	return get_world_3d().direct_space_state.intersect_ray(
		PhysicsRayQueryParameters3D.create(start, end, 3, [actor.get_rid()]))

func preview(actor: CharacterBody3D, direction: Vector3) -> void:
	var dt := 1.0 / Engine.physics_ticks_per_second
	var gravity := actor.get_gravity()
	var origin := _origin(actor, direction)
	var velocity := _velocity(direction)
	var dots := 0
	marker.hide()
	for step in DOT_COUNT * DOT_SPACING:
		var next := origin + velocity * dt + gravity * (0.5 * dt * dt)
		var hit := _trace(actor, origin, next)
		if not hit.is_empty():
			marker.global_position = hit.position + hit.normal * 0.018
			marker.global_basis = Basis(Quaternion(Vector3.UP, hit.normal))
			marker.show()
			break
		if step % DOT_SPACING == 0:
			path_dots.multimesh.set_instance_transform(dots, Transform3D(Basis.IDENTITY, to_local(next)))
			dots += 1
		origin = next
		velocity += gravity * dt
	path_dots.multimesh.visible_instance_count = dots

func clear_preview() -> void:
	path_dots.multimesh.visible_instance_count = 0
	marker.hide()

func launch(actor: CharacterBody3D, item: Node3D, direction: Vector3, slot: int,
		explosive: bool) -> void:
	var projectile := RigidBody3D.new()
	projectile.collision_layer = 0
	projectile.collision_mask = 3
	projectile.continuous_cd = true
	projectile.contact_monitor = true
	projectile.max_contacts_reported = 1
	projectile.body_entered.connect(
		func(_body: Node) -> void:
			projectile.set_deferred("linear_damp", 5.0)
			projectile.set_deferred("angular_damp", 5.0),
		CONNECT_ONE_SHOT)
	projectile.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	projectile.linear_damp = 0.0
	projectile.set_meta(&"slot", slot)
	projectile.set_meta(&"explosive", explosive)
	var collision := CollisionShape3D.new()
	var sphere := SphereShape3D.new()
	sphere.radius = RADIUS
	collision.shape = sphere
	projectile.add_child(collision)
	add_child(projectile)
	projectile.global_transform = Transform3D(item.global_basis.orthonormalized(), _origin(actor, direction))
	item.reparent(projectile, false)
	item.transform = Transform3D.IDENTITY
	projectile.add_collision_exception_with(actor)
	projectile.linear_velocity = _velocity(direction)
	projectile.angular_velocity = direction.cross(Vector3.UP) * 7.5
	get_tree().create_timer(1.55 if explosive else 20.0, false, true).timeout.connect(
		_expire.bind(weakref(projectile), explosive))

func nearest_item(actor: CharacterBody3D) -> RigidBody3D:
	var nearest: RigidBody3D
	var distance := 1.5
	for node in get_children():
		var item := node as RigidBody3D
		if item and not item.is_queued_for_deletion() and not item.get_meta(&"explosive"):
			var delta := actor.global_position.distance_to(item.global_position)
			if delta < distance and (item.sleeping or item.linear_velocity.length() < 1.0):
				distance = delta
				nearest = item
	return nearest

func _expire(reference: WeakRef, explosive: bool) -> void:
	var projectile := reference.get_ref() as RigidBody3D
	if not projectile:
		return
	if explosive:
		var center := projectile.global_position
		exploded.emit(center)
		var flash := MeshInstance3D.new()
		var sphere := SphereMesh.new()
		sphere.radial_segments = 12
		sphere.rings = 6
		var material := StandardMaterial3D.new()
		material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		material.albedo_color = Color(1.0, 0.53, 0.17, 0.8)
		flash.material_override = material
		flash.mesh = sphere
		add_child(flash)
		flash.global_position = center
		var tween := create_tween().set_parallel()
		tween.tween_property(flash, "scale", Vector3.ONE * 4.0, 0.24).from(Vector3.ONE * 0.25)
		tween.tween_property(material, "albedo_color:a", 0.0, 0.24)
		tween.chain().tween_callback(flash.queue_free)
	projectile.queue_free()
