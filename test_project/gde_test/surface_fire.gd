class_name SurfaceFireComponent
extends Node

@export var grid_size:Vector3i = Vector3i(3, 3, 3)

@export var col_instance:CollisionShape3D = null
var shape:Shape3D = null
var grid:Dictionary[Vector3i,Dictionary] = {}
var mesh:ArrayMesh

func _get_convex_planes() -> Array[Plane]:
	var planes: Array[Plane] = []

	await get_tree().process_frame
	if not mesh:
		push_error("debug mesh not available yet")
		return planes

	for s in range(mesh.get_surface_count()):
		var arrays := mesh.surface_get_arrays(s)
		if arrays.is_empty():
			continue

		var verts: PackedVector3Array = arrays[ArrayMesh.ARRAY_VERTEX]
		var indices_var = arrays[ArrayMesh.ARRAY_INDEX]
		var indices: PackedInt32Array = PackedInt32Array()

		if indices_var != null:
			indices = indices_var


		if indices.is_empty():
			# fallback: extremely rare, but be defensive
			for i in range(0, verts.size() - 2, 3):
				planes.append(Plane(verts[i], verts[i + 1], verts[i + 2]))
			continue

		for i in range(0, indices.size(), 3):
			var a := verts[indices[i]]
			var b := verts[indices[i + 1]]
			var c := verts[indices[i + 2]]

			planes.append(Plane(a, b, c))

	return planes


func _project_to_nearest_plane(p: Vector3, planes: Array[Plane]) -> Vector3:
	var best_out_dist := INF
	var best_out_plane = null

	var best_in_dist := -INF
	var best_in_plane = null

	for plane in planes:
		var d := plane.distance_to(p)

		if d > 0.0:
			# outside this face
			if d < best_out_dist:
				best_out_dist = d
				best_out_plane = plane
		else:
			# inside / behind this face
			if d > best_in_dist:
				best_in_dist = d
				best_in_plane = plane

	# prefer snapping from outside
	if best_out_plane != null:
		return p - best_out_plane.normal * best_out_dist

	# otherwise snap from inside to nearest exit face
	if best_in_plane != null:
		return p - best_in_plane.normal * best_in_dist

	# should never happen
	return p




func _setup_grid() -> void:
	var planes:Array[Plane] = await _get_convex_planes()

	# shape local AABB
	#var aabb := shape.get_debug_mesh().get_aabb()
	var aabb:AABB = mesh.get_aabb()
	var step := Vector3(
		aabb.size.x / grid_size.x,
		aabb.size.y / grid_size.y,
		aabb.size.z / grid_size.z
	)

	grid.clear()

	for x in range(grid_size.x):
		for y in range(grid_size.y):
			for z in range(grid_size.z):
				var cell := Vector3i(x, y, z)

				var p := aabb.position + Vector3(
					(x + 0.5) * step.x,
					(y + 0.5) * step.y,
					(z + 0.5) * step.z
				)

				var projected := _project_to_nearest_plane(p, planes)

				grid[cell] = {
					"local_pos": projected
				}
	# print('planes:',planes.size())

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	if col_instance and col_instance.shape is ConvexPolygonShape3D:
		shape = col_instance.shape
	else:
		# FIND MESH
		for n in get_parent().get_children(true):
			#print(n)
			if n is CollisionShape3D and n.shape is ConvexPolygonShape3D:
				shape = n.shape
				break
		if not shape:
			push_error('NO CONVEX COLLISION SHAPE FOUND')
			get_tree().quit()
			return
	
	mesh = shape.get_debug_mesh()
	await _setup_grid()
	
	for cell in grid:
		var m = MeshInstance3D.new()
		m.mesh = BoxMesh.new()
		(m.mesh as BoxMesh).size = Vector3(0.05,0.05,0.05)
		m.cast_shadow = false
		add_child(m)
		m.global_position = col_instance.to_global(grid[cell]['local_pos'])
		# print(cell, ':', grid[cell]['local_pos'])
