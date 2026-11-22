extends Node3D

const STEEL := Color(0.22, 0.33, 0.40)
const EDGE := Color(0.93, 0.60, 0.17)
const CRATE := Color(0.63, 0.38, 0.20)
var hazards: Array[RigidBody3D] = []

func _ready() -> void:
	_box(Vector3(-2.3, 0.35, -0.8), Vector3.ONE * 0.7, CRATE, 0.65)
	_box(Vector3(2.4, 0.48, -1.3), Vector3.ONE * 0.96, Color(0.36, 0.45, 0.52), 16.0)
	_pipe(Vector3(-3.8, 0.41, -2.4))
	_boulder(Vector3(3.6, 0.55, -3.8))
	for i in 3:
		var height := float(i + 1) * 0.38
		_box(Vector3(-4.3, height / 2.0, 0.25 - i * 0.85),
			Vector3(1.5, height, 0.85), STEEL)
	for i in 3:
		_box(Vector3(-5.9 - i * 0.83, 0.72, -5.9),
			Vector3(0.45, 1.44, 0.45), STEEL)
	var ramp := _box(Vector3(6.5, 2.0, -2.7), Vector3(2.6, 0.35, 9.2), STEEL)
	ramp.rotation.x = 0.46
	var deck := _box(Vector3(6.5, 3.99, -8.6), Vector3(2.6, 0.35, 3.6), STEEL)
	_paint_edge(deck, Vector3(2.6, 0.35, 3.6))

func release_hazards() -> void:
	for hazard in hazards:
		if is_instance_valid(hazard):
			hazard.queue_free()
	hazards.clear()
	var pipe := _pipe(Vector3(-2.8, 0.44, -8.0))
	pipe.linear_velocity = Vector3(0.0, 0.0, 7.0)
	pipe.angular_velocity = Vector3(16.0, 0.0, 0.0)
	hazards.append(pipe)
	var rock := _boulder(Vector3(0.4, 0.63, -9.0))
	rock.linear_velocity = Vector3(0.0, 0.0, 8.0)
	hazards.append(rock)
	var crate := _box(Vector3(3.1, 4.7, -3.6), Vector3.ONE * 0.85, CRATE, 10.0) as RigidBody3D
	crate.angular_velocity = Vector3(1.5, 0.4, 1.0)
	hazards.append(crate)

func _box(at: Vector3, size: Vector3, color: Color, mass: float = 0.0) -> PhysicsBody3D:
	var shape := BoxShape3D.new()
	shape.size = size
	var mesh := BoxMesh.new()
	mesh.size = size
	var body := _body(shape, mesh, at, color, mass)
	_paint_edge(body, size)
	return body

func _paint_edge(body: Node3D, size: Vector3) -> void:
	var stripe := BoxMesh.new()
	stripe.size = Vector3(size.x * 0.94, 0.012, minf(size.z * 0.16, 0.26))
	_visual(body, stripe, EDGE, Vector3(0, size.y / 2.0 + 0.006, -size.z * 0.39))
	_visual(body, stripe, EDGE, Vector3(0, size.y / 2.0 + 0.006, size.z * 0.39))

func _pipe(at: Vector3) -> RigidBody3D:
	var section := CylinderShape3D.new()
	section.radius = 0.40
	section.height = 1.65
	var mesh := CylinderMesh.new()
	mesh.top_radius = section.radius
	mesh.bottom_radius = section.radius
	mesh.height = section.height
	mesh.radial_segments = 12
	var pipe := _body(section, mesh, at, Color(0.29, 0.52, 0.51), 9.0) as RigidBody3D
	pipe.rotation.z = PI / 2.0
	var lip := TorusMesh.new()
	lip.inner_radius = 0.32
	lip.outer_radius = 0.41
	lip.rings = 12
	lip.ring_segments = 6
	for side in [-1, 1]:
		_visual(pipe, lip, STEEL, Vector3(0, side * 0.80, 0))
	return pipe

func _boulder(at: Vector3) -> RigidBody3D:
	var shape := SphereShape3D.new()
	shape.radius = 0.55
	var mesh := SphereMesh.new()
	mesh.radius = shape.radius
	mesh.height = 2.0 * shape.radius
	mesh.radial_segments = 9
	mesh.rings = 4
	return _body(shape, mesh, at, Color(0.53, 0.49, 0.44), 13.0) as RigidBody3D

func _body(shape: Shape3D, mesh: Mesh, at: Vector3, color: Color, mass: float) -> PhysicsBody3D:
	var body: PhysicsBody3D
	if mass > 0.0:
		var rigid := RigidBody3D.new()
		rigid.mass = mass
		rigid.collision_mask = 3
		rigid.continuous_cd = true
		body = rigid
	else:
		body = StaticBody3D.new()
	body.position = at
	var collision := CollisionShape3D.new()
	collision.shape = shape
	body.add_child(collision)
	_visual(body, mesh, color, Vector3.ZERO)
	add_child(body)
	return body

func _visual(parent: Node3D, mesh: Mesh, color: Color, at: Vector3) -> void:
	var view := MeshInstance3D.new()
	view.mesh = mesh
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 0.82
	view.material_override = material
	view.position = at
	parent.add_child(view)
