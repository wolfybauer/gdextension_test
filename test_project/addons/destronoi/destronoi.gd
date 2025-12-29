#@tool
@icon("res://addons/destronoi/destronoi_icon.svg")
extends Node
class_name DestronoiGD
"""
Author: George Power <george@georgepower.dev>
"""
## Subdivides a convex [ArrayMesh] belonging to a [RigidBody3D] by generating a Voronoi Subdivision Tree (VST).
##
## A [DestronoiNode] must be a child of a [RigidBody3D] with a single
## [MeshInstance3D] as a sibling. The [MeshInstance3D] must have the default name
## "MeshInstance3D". The mesh data [b]must[/b] be an [ArrayMesh]. Using an imported
## mesh as an [code].obj[/code] file should suffice. When the Destronoi node is loaded
## it will create a VST which is accessible through the [param _root].
## [br]See the demo scene for an example.

## The root node of the VST. Contains a copy of the sibling [MeshInstance3D]. 
var _root: VSTNodeGD = null
var _parent:Node3D
var base_material:Material

## The [DestronoiNode] generates [code]2^n[/code] fragments, where [code]n[/code] is the [param tree_height] of the VST.
@export_range(1, 8) var tree_height: int = 1
@export var persistence: float = 2.0
@export var inside_material: Material


## Initializes the [param _root] with a copy of the sibling [MeshInstance3D].
## The mesh is subdivided according to the [param tree_height].
func _ready():
	# Set root geometry to sibling MeshInstance3D
	_parent = get_parent()
	var mesh_instance = null
	for child in _parent.get_children():
		if (child is MeshInstance3D):
			mesh_instance = child
			break

	if (mesh_instance == null):
		print("[Destronoi] No MeshInstance3D sibling found")
		return # no mesh; return early
	
	base_material = mesh_instance.get_active_material(0)
	if base_material == null:
		base_material = StandardMaterial3D.new()

	var am:ArrayMesh = ensure_array_mesh(mesh_instance.mesh)

	_root = VSTNodeGD.new(am, base_material)

	# Plot 2 sites for the subdivision
	var plot_mdt:MeshDataTool = MeshDataTool.new()
	var sta = SurfaceTool.new()
	var stb = SurfaceTool.new()
	plot_sites_random(plot_mdt, _root)
	# Generate 2 children from the root
	bisect(sta, stb, plot_mdt, _root)
	# Perform additional subdivisions depending on tree height
	for i in range(tree_height - 1):
		var leaves = []
		_root.get_leaf_nodes(_root, leaves);
		for leaf in range(leaves.size()):
			plot_sites_random(plot_mdt, leaves[leaf])
			bisect(sta, stb, plot_mdt, leaves[leaf])

func _emit_vertex(st:SurfaceTool, data:MeshDataTool, vid:int) -> void:
	var pos = data.get_vertex(vid)
	if not pos:
		return

	st.set_uv(data.get_vertex_uv(vid))
	st.set_normal(data.get_vertex_normal(vid))
	st.set_tangent(data.get_vertex_tangent(vid))
	st.set_color(data.get_vertex_color(vid))
	st.add_vertex(pos)


func _emit_intersection(st:SurfaceTool, data:MeshDataTool, a_id:int, b_id:int, p:Vector3) -> void:
	var a = data.get_vertex(a_id)
	var b = data.get_vertex(b_id)

	var t = a.distance_to(p) / max(a.distance_to(b), 0.000001)

	# uv
	var uv = data.get_vertex_uv(a_id).lerp(data.get_vertex_uv(b_id), t)

	# normal
	var nrm = data.get_vertex_normal(a_id).lerp(
		data.get_vertex_normal(b_id), t
	).normalized()

	# color
	var col = data.get_vertex_color(a_id).lerp(
		data.get_vertex_color(b_id), t
	)

	# tangent
	var ta = data.get_vertex_tangent(a_id)
	var tb = data.get_vertex_tangent(b_id)
	var tan = Plane(
		ta.x + (tb.x - ta.x) * t,
		ta.y + (tb.y - ta.y) * t,
		ta.z + (tb.z - ta.z) * t,
		ta.d + (tb.d - ta.d) * t
	)

	st.set_uv(uv)
	st.set_normal(nrm)
	st.set_tangent(tan)
	st.set_color(col)
	st.add_vertex(p)

static func ensure_array_mesh(mesh: Mesh) -> ArrayMesh:
	if mesh is ArrayMesh:
		# print('it array!')
		return mesh
	# print('not array!')

	# Convert PrimitiveMesh → ArrayMesh
	var arrays:Array = mesh.get_mesh_arrays()
	PrimitiveMesh
	var am := ArrayMesh.new()
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return am



## Randomly plots a pair of valid sites using rejection sampling. A site is
## considered valid if it falls within the volume of the [member VSTNodeGD._mesh_instance].
## [br][color=yellow]Note:[/color] Site coordinates are relative to the centre of [member VSTNodeGD._mesh_instance].
## [br][color=yellow]Note:[/color] Reusing this method will overwrite any existing sites.
func plot_sites_random(mdt:MeshDataTool, vst_node: VSTNodeGD):
	vst_node._sites = [] # clear existing sites

	var site: Vector3

	# MeshDataTool used to parse the mesh faces
	mdt.clear()
	mdt.create_from_surface(vst_node._mesh, 0)

	# Bounding box used to get range limits for random points
	var aabb: AABB = vst_node._mesh.get_aabb()
	var min_vec: Vector3 = aabb.position
	var max_vec: Vector3 = aabb.end

	# Centers of each axis
	var avg_x = (max_vec.x + min_vec.x) / 2.0
	var avg_y = (max_vec.y + min_vec.y) / 2.0
	var avg_z = (max_vec.z + min_vec.z) / 2.0

	var dev = 0.1 # deviation from the mean
	var num_intersections = 0
	var face_v_ids = []
	var verts = []
	var intersection_point
	# keep generating sites until they are within the mesh
	# a valid pair of sites are both inside the mesh boundary
	while vst_node._sites.size() < 2:
		# normally distributed about AABB center
		site = Vector3(randfn(avg_x, dev), randfn(avg_y, dev), randfn(avg_z, dev))

		num_intersections = 0
		for tri in range(mdt.get_face_count()):
			face_v_ids = [mdt.get_face_vertex(tri, 0), mdt.get_face_vertex(tri, 1), mdt.get_face_vertex(tri, 2)]
			verts = [mdt.get_vertex(face_v_ids[0]), mdt.get_vertex(face_v_ids[1]), mdt.get_vertex(face_v_ids[2])]
			intersection_point = Geometry3D.ray_intersects_triangle(site, Vector3.UP, verts[0], verts[1], verts[2])

			if (intersection_point != null):
				num_intersections += 1

		if (num_intersections == 1): # must be inside; add
			vst_node._sites.append(site)

func bisect(sta:SurfaceTool, stb:SurfaceTool, data_tool:MeshDataTool, vst_node: VSTNodeGD) -> bool:
	# Bisection aborted! Must have exactly 2 sites
	if vst_node.get_site_count() != 2:
		return false

	# Create the plane
	# Equidistant from both sites; normal vector towards to site B
	var site_a := vst_node._sites[0]
	var site_b := vst_node._sites[1]
	var plane_normal := (site_b - site_a).normalized() # a to b
	var plane_position := site_a + 0.5 * (site_b - site_a) # halfway between a,b
	var plane = Plane(plane_normal, plane_position)

	# Create MeshDataTool to parse mesh data of current VSTNodeGD
	data_tool.clear()
	data_tool.create_from_surface(vst_node._mesh, 0)
	
	# Create SurfaceTool to construct the ABOVE mesh
	sta.clear()
	sta.begin(Mesh.PRIMITIVE_TRIANGLES)
	sta.set_material(vst_node._mat)
	sta.set_smooth_group(-1)

	# Create SurfaceTool to construct the BELOW mesh
	stb.clear()
	stb.begin(Mesh.PRIMITIVE_TRIANGLES)
	stb.set_material(vst_node._mat)
	stb.set_smooth_group(-1)
	
	var tools:Array[SurfaceTool] = [sta, stb]

	## GENERATE SUB MESHES
	# ITERATE OVER EACH FACE OF THE BASE MESH
	# 2 iterations for 2 sub meshes (above/below)
	for side in range(2):
		# Intermediate surface tool to construct the new mesh
		var surface_tool := tools[side]

		# invert normal for other side (i.e. treat below as above and repeat the process)
		if (side == 1):
			plane.normal = - plane.normal
			plane.d = - plane.d

		var coplanar_vertices := [] # new vertices which intersect the plane

		for face in range(data_tool.get_face_count()):
			var face_vertices := []
			var vertices_above_plane := []
			var intersection_points := []

			# ITERATE OVER EACH VERTEX AND DETERMINE "ABOVENESS"
			for vertex_index in range(3):
				var vertex_id := data_tool.get_face_vertex(face, vertex_index)
				face_vertices.append(vertex_id)
				if plane.is_point_over(data_tool.get_vertex(vertex_id)):
					vertices_above_plane.append(vertex_id)

			# INTERSECTION CASE 0/0.5: ALL or NOTHING above the plane
			if (vertices_above_plane.size() == 0):
				continue
			if (vertices_above_plane.size() == 3):
				for v_id in face_vertices:
					_emit_vertex(surface_tool, data_tool, v_id)
				continue

			# INTERSECTION CASE 1: ONE point above the plane
			# Find intersection points and append them in cw winding order
			if (vertices_above_plane.size() == 1):
				var index_before: int = -1
				var index_after: int = -1
				for index in range(3):
					if vertices_above_plane[0] == face_vertices[index]:
						index_after = (index + 1) % 3
						index_before = (index + 2) % 3
						break
				var p_after = plane.intersects_segment(
					data_tool.get_vertex(vertices_above_plane[0]),
					data_tool.get_vertex(face_vertices[index_after])
				)

				intersection_points.append({
					"a_id": vertices_above_plane[0],
					"b_id": face_vertices[index_after],
					"p": p_after
				})

				coplanar_vertices.append(p_after)

				var p_before = plane.intersects_segment(
					data_tool.get_vertex(vertices_above_plane[0]),
					data_tool.get_vertex(face_vertices[index_before])
				)

				intersection_points.append({
					"a_id": vertices_above_plane[0],
					"b_id": face_vertices[index_before],
					"p": p_before
				})

				coplanar_vertices.append(p_before)

 
				# TRIANGLE CREATION
				_emit_vertex(surface_tool, data_tool, vertices_above_plane[0])
				
				var A = intersection_points[0]
				_emit_intersection(surface_tool, data_tool, A.a_id, A.b_id, A.p)

				var B = intersection_points[1]
				_emit_intersection(surface_tool, data_tool, B.a_id, B.b_id, B.p)

				continue

			# INTERSECTION CASE 2: TWO points above the plane
			if (vertices_above_plane.size() == 2):
				var index_remaining: int = -1 # index of the point below the plane

				# Ensure vertices are in cyclic CW order
				if (vertices_above_plane[0] != face_vertices[1]
				and vertices_above_plane[1] != face_vertices[1]):
					vertices_above_plane.reverse()
					index_remaining = 1
				elif (vertices_above_plane[0] != face_vertices[0]
				and vertices_above_plane[1] != face_vertices[0]):
					index_remaining = 0
				else:
					index_remaining = 2

				# compute intersections (FIXED: now actually defined here)
				var intersection_after = plane.intersects_segment(
					data_tool.get_vertex(vertices_above_plane[1]),
					data_tool.get_vertex(face_vertices[index_remaining])
				)

				var intersection_before = plane.intersects_segment(
					data_tool.get_vertex(vertices_above_plane[0]),
					data_tool.get_vertex(face_vertices[index_remaining])
				)

				# save structured intersection data
				var ip_after = {
					"a_id": vertices_above_plane[1],
					"b_id": face_vertices[index_remaining],
					"p": intersection_after
				}

				var ip_before = {
					"a_id": vertices_above_plane[0],
					"b_id": face_vertices[index_remaining],
					"p": intersection_before
				}

				intersection_points.append(ip_after)
				intersection_points.append(ip_before)

				# preserve your original coplanar geometry
				coplanar_vertices.append(intersection_after)
				coplanar_vertices.append(intersection_before)

				# choose diagonal
				var dist_0 = data_tool.get_vertex(vertices_above_plane[0]).distance_to(intersection_after)
				var dist_1 = data_tool.get_vertex(vertices_above_plane[1]).distance_to(intersection_before)

				var index_shortest := 0
				if dist_1 > dist_0:
					index_shortest = 1

				#
				# TRIANGLE 1
				_emit_vertex(surface_tool, data_tool, vertices_above_plane[0])
				_emit_vertex(surface_tool, data_tool, vertices_above_plane[1])

				# use structured intersection instead of raw vector
				var ip_short = intersection_points[index_shortest]
				_emit_intersection(surface_tool, data_tool, ip_short.a_id, ip_short.b_id, ip_short.p)

				#
				# TRIANGLE 2
				#
				var ip0 = intersection_points[0]
				var ip1 = intersection_points[1]

				_emit_intersection(surface_tool, data_tool, ip0.a_id, ip0.b_id, ip0.p)
				_emit_intersection(surface_tool, data_tool, ip1.a_id, ip1.b_id, ip1.p)

				_emit_vertex(surface_tool, data_tool, vertices_above_plane[index_shortest])

				continue

		# END for face in range(data_tool.get_face_count())

		var centroid := Vector3(0, 0, 0)
		for vertices in coplanar_vertices:
			centroid += vertices
		centroid /= coplanar_vertices.size()

		# DEFINE NEW FACE; FIND CENTROID; APPEND TRIANGLES
		#surface_tool.set_color(inner_color)
		for i in range(coplanar_vertices.size() - 1):
			if (i % 2 != 0): continue ;
			surface_tool.add_vertex(coplanar_vertices[i + 1])
			surface_tool.add_vertex(coplanar_vertices[i])
			surface_tool.add_vertex(centroid)

	sta.index()
	sta.generate_normals()
	stb.index()
	stb.generate_normals()

	# Assign new meshes to left and right nodes
	# Left is above, right is below; this decision was arbitrary
	var m:ArrayMesh = sta.commit()
	vst_node._left = VSTNodeGD.new(m, base_material, vst_node._level + 1, VSTNodeGD.Laterality.LEFT)

	m = stb.commit()
	vst_node._right = VSTNodeGD.new(m, base_material, vst_node._level + 1, VSTNodeGD.Laterality.RIGHT)
	
	return true
	

func _free_vst_tree(vroot:VSTNodeGD):
	if vroot == null: return
	#if vroot._mesh_instance:
		#vroot._mesh_instance.queue_free()
	vroot._mesh = null
	vroot._mat = null
	_free_vst_tree(vroot._left)
	_free_vst_tree(vroot._right)
	
	vroot._left = null
	vroot._right = null
	vroot._sites = []

## Replaces the [RigidBody3D] with a specified number of fragments.
## [br]If [param radial_velocity][code] > 0.0[/code] fragments will accelerate 
## outward from the centre of the object with a velocity equal to [param radial_velocity].
## [br]If [param radial_velocity][code] == 0.0[/code] fragments will not be accelerated.
## [br][param left_val] and [param right_val] specify the depth level of the
## fragments. E.g. if both values are 1, only fragments from the 1st level will
## be used, resulting in 2 fragments being placed.
## [br][color=yellow]Warning:[/color] You must have left and right values of 1 or greater.
func destroy(radial_velocity: float = 5.0, linear_velocity: Vector3 = Vector3.ZERO):
	
	var depth = pow(2, tree_height-1)
	var vst_leaves := []
	var current_node: VSTNodeGD = _root
	current_node.get_left_leaf_nodes(current_node, vst_leaves, depth)
	current_node.get_right_leaf_nodes(current_node, vst_leaves, depth)

	var new_rigid_bodies:Array[RigidBody3D] = []
	var sum_mass = 0

	for vst_leaf in range(vst_leaves.size()):
		var new_body: RigidBody3D = RigidBody3D.new()
		new_body.name = "VFragment_{id}".format({"id": vst_leaf})

		new_body.position = _parent.transform.origin

		var new_mesh_instance:MeshInstance3D = MeshInstance3D.new()
		new_mesh_instance.mesh = vst_leaves[vst_leaf]._mesh
		new_mesh_instance.name = "MeshInstance3D"
		new_body.add_child(new_mesh_instance)

		# Create collision geometry
		#var new_body_mesh_instance: MeshInstance3D = new_body.get_child(0)
		var new_collision_shape: CollisionShape3D = CollisionShape3D.new()
		new_collision_shape.name = "CollisionShape3D"

		var velocity_dir = new_mesh_instance.mesh.get_aabb().get_center() - _parent.position;
		velocity_dir = velocity_dir.normalized()

		new_body.mass = max(new_mesh_instance.mesh.get_aabb().get_volume(), 0.1)
		sum_mass += new_body.mass

		# Combustion calculations:
		# find the vector pointing from the base object's center, to the new fragment's center
		# fragments should go outward, as if the object has combusted

		var estim_dir = Vector3(0, 0, 0)
		if (!is_zero_approx(radial_velocity)):
			var endpoints = []
			for i in range(8):
				var current_endpoint = new_body.get_node("MeshInstance3D").mesh.get_aabb().get_endpoint(i)
				current_endpoint = current_endpoint.normalized()
				var current_dot = current_endpoint.dot(_parent.transform.origin.normalized())
				if abs(current_dot) > 0.0:
					endpoints.append(current_endpoint)

			for x in range(endpoints.size()):
				estim_dir += endpoints[x]

			if (endpoints.size() > 0):
				estim_dir /= endpoints.size()

			estim_dir = estim_dir.normalized() * radial_velocity

		new_body.set_axis_velocity(estim_dir + linear_velocity)

		new_collision_shape.shape = new_mesh_instance.mesh.create_convex_shape(false, false)

		new_body.add_child(new_collision_shape)
		new_rigid_bodies.append(new_body)

	# scale masses to match the base object
	var base_mass = _parent.mass if _parent is RigidBody3D else 1.0
	var pgroups:Array[StringName] = _parent.get_groups()
	var _pp:Node3D = _parent.get_parent_node_3d()
	for body in new_rigid_bodies:
		body.mass = body.mass * (base_mass / sum_mass)
		for g in pgroups:
			body.add_to_group(g)
		_pp.add_child(body)
		#body.visible  = not (body.global_position > GAME.player.global_position)

	reparent(_pp)
	_parent.queue_free()
	_free_vst_tree(_root)
	if not (persistence > 0.0):
		return

	await get_tree().create_timer(persistence).timeout
	for n in new_rigid_bodies:
		n.call_deferred('queue_free')
	queue_free()
