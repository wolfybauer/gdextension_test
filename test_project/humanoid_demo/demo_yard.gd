extends Node3D

const BURNABLE_CRATE = preload("demo_burnable_crate.tscn")
const STEEL := Color(0.22, 0.33, 0.40)
const EDGE := Color(0.93, 0.60, 0.17)
const CRATE := Color(0.63, 0.38, 0.20)
var hazards: Array[RigidBody3D] = []

var ignition_crate: RigidBody3D

func _ready() -> void:
	_box(Vector3(0.1, 0.35, 4.6), Vector3.ONE * 0.7, CRATE, 0.65)
	_box(Vector3(7.5, 0.48, 4.4), Vector3.ONE * 0.96, Color(0.36, 0.45, 0.52), 16.0)
	_pipe(Vector3(-4.5, 0.41, 4.0))
	_boulder(Vector3(10.5, 0.55, 2.5))
	ignition_crate = _burnable(Vector3(-3.4, 0.61, 3.75))
	_burnable(Vector3(-2.35, 0.61, 3.8))
	_burnable(Vector3(-3.35, 0.61, 4.85))
	_burnable(Vector3(9.6, 0.61, -4.4))
	_burnable(Vector3(17.0, 4.61, -27.0))

func _burnable(at: Vector3) -> RigidBody3D:
	var crate: RigidBody3D = BURNABLE_CRATE.instantiate()
	crate.position = at
	add_child(crate)
	return crate

func ignite_crates() -> void:
	if not is_instance_valid(ignition_crate):
		ignition_crate = _burnable(Vector3(-3.4, 0.61, 3.75))
	ignition_crate.ignite()

func release_hazards() -> void:
	for hazard in hazards:
		if is_instance_valid(hazard):
			hazard.queue_free()
	hazards.clear()
	var pipe := _pipe(Vector3(4.65, 0.44, -7.7))
	pipe.linear_velocity = Vector3(0.0, 0.0, 7.0)
	pipe.angular_velocity = Vector3(16.0, 0.0, 0.0)
	hazards.append(pipe)
	var rock := _boulder(Vector3(1.35, 0.63, -7.7))
	rock.linear_velocity = Vector3(0.0, 0.0, 8.0)
	hazards.append(rock)
	var crate := _box(Vector3(5.2, 4.7, 3.7), Vector3.ONE * 0.85, CRATE, 10.0) as RigidBody3D
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
		rigid.add_to_group("fade_obj", true)
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
