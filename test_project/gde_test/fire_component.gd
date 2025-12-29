class_name FireComponentGD
extends Node3D

signal on_fire_catch(world_pos:Vector3)
signal on_fire_damage(amount:int)

@export var torch:bool = false
@export var grid_resolution: Vector3i = Vector3i(3, 3, 3) # Grid density
@export var debug_visualize:bool = false:
	set(b):
		for cell in fire_grid:
			if fire_grid[cell].visual:
				(fire_grid[cell].visual as Node3D).visible = b
				debug_visualize = b
@export var interior_cells: bool = false  # If false, only outermost cells are kept
@export var max_hitpoints: int = 45 # How resistant cells are to fire
@export var burn_time: float = 4.0 # How long a cell burns
@export var s_spread_budget: int = 20 # Prevents infinite fire
@export var spread_cost: int = 1 # Spread budget per burning cell
@export var spread_damage:int = 10  # Damage per second from adjacent burning cells
@export var spread_margin:float = 0.7
@export var spread_interval:float = 0.5
@export var col_inst:CollisionShape3D = null
var is_convex:bool = false
var convex_planes:Array[Plane] = []

@onready var debug_mat:StandardMaterial3D = StandardMaterial3D.new()
@onready var debug_mesh:SphereMesh = SphereMesh.new()

const FLAMMABLE_LAYER:int = 31
const FIRE_LAYER:int = 32

var fire_grid:Dictionary = {} # Stores fire cells (key: grid position, value: {hitpoints, burning, time_left})
var emitters:Dictionary = {} # store particle emitters
var fire_visuals:Dictionary = {} # Fire visualization (grid position -> sphere reference)
var cell_size: Vector3 # Computed cell size
var collision_aabb: AABB # Object's collision bounding box
var burning_aabb:AABB
var inter_object_timer:float = 0.0

@onready var burning_area:Area3D = Area3D.new()
@onready var burn_box:BoxShape3D = BoxShape3D.new()
@onready var burn_col:CollisionShape3D = CollisionShape3D.new()

var parent:Node3D

const FIRE_PARTICLES = preload("res://gde_test/assets/fire/fire_particles.tscn")

const SURROUNDING:Array = [
	Vector3i(1, 0, 0),
	Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0),
	Vector3i(0, -1, 0),
	Vector3i(0, 0, 1),
	Vector3i(0, 0, -1)
]



func get_shape_aabb(collision_shape: CollisionShape3D, to_world:bool=false) -> AABB:
	var shape = collision_shape.shape
	var aabb:AABB
	is_convex = false
	if shape is BoxShape3D:
		aabb = AABB(-shape.size / 2, shape.size)  # Centered AABB
	elif shape is SphereShape3D:
		var r = shape.radius
		aabb = AABB(Vector3(-r, -r, -r), Vector3(r * 2, r * 2, r * 2))
	elif shape is CapsuleShape3D:
		var r = shape.radius
		var h = shape.height
		aabb = AABB(Vector3(-r, -h / 2, -r), Vector3(r * 2, h, r * 2))
	else:
		# inefficient probably
		var m:ArrayMesh = collision_shape.shape.get_debug_mesh()
		aabb = m.get_aabb()
		convex_planes = build_convex_planes(shape)
		is_convex = true
	if to_world:
		aabb.position = collision_shape.global_transform.origin + aabb.position # Convert to world space
	return aabb

func build_convex_planes(shape: ConvexPolygonShape3D) -> Array[Plane]:
	var planes: Array[Plane] = []

	var points: PackedVector3Array = shape.points
	if points.size() < 4:
		return planes

	# compute center
	var center := Vector3.ZERO
	for p in points:
		center += p
	center /= points.size()

	# brute-force face discovery (acceptable for grid setup)
	for i in range(points.size()):
		for j in range(i + 1, points.size()):
			for k in range(j + 1, points.size()):
				var a := points[i]
				var b := points[j]
				var c := points[k]

				if (b - a).cross(c - a).length_squared() < 1e-6:
					continue # degenerate

				var plane := Plane(a, b, c)

				# ensure outward normal
				if plane.distance_to(center) > 0.0:
					plane = Plane(-plane.normal, -plane.d)

				var valid := true
				for p in points:
					if plane.distance_to(p) > 0.001:
						valid = false
						break

				if valid:
					# dedupe
					var dup:bool = false
					for q in planes:
						if plane.normal.dot(q.normal) > 0.999 and abs(plane.d - q.d) < 0.001:
							dup = true
							break
					if not dup:
						planes.append(plane)

	return planes

func is_inside_object(world_pos: Vector3) -> bool:
	if not collision_aabb.has_point(world_pos):
		return false

	if not is_convex:
		return true

	var local := col_inst.global_transform.affine_inverse() * world_pos

	for plane in convex_planes:
		if plane.distance_to(local) > 0.001:
			return false

	return true



func apply_fire_damage(world_pos: Vector3, damage: int):

	var closest_cell = get_closest_fire_cell(world_pos)
	if not closest_cell:
		return
	
	var cell_data = fire_grid[closest_cell]

	cell_data["hitpoints"] -= damage
	
	if cell_data["hitpoints"] <= 0:
		ignite_cell(closest_cell)
		on_fire_catch.emit(world_pos)

func get_closest_fire_cell(world_pos: Vector3):
	var min_dist = INF
	var closest = null

	for cell in fire_grid.keys():
		var dist = world_pos.distance_squared_to(to_global(fire_grid[cell]['local_pos']))
		if dist < min_dist:
			min_dist = dist
			closest = cell

	return closest

func ignite_cell(cell: Vector3i):
	var data = fire_grid[cell]
	if s_spread_budget <= 0 or data["burning"]:
		return

	data["burning"] = true
	data["time_left"] = burn_time
	s_spread_budget -= spread_cost # Consume fire spread points

	emitters[cell] = FIRE_PARTICLES.instantiate()
	add_child(emitters[cell])
	emitters[cell].transform.origin = data['local_pos']
	update_burning_area()
	

func extinguish_cell(cell: Vector3i):
	var data = fire_grid[cell]
	if not data["burning"]:
		return
	remove_child(emitters[cell])
	emitters[cell].queue_free()
	emitters[cell] = null
	data["hitpoints"] = max_hitpoints
	data["burning"] = false
	s_spread_budget += spread_cost
	data["cooldown"] = burn_time / 2.0
	update_burning_area()

func get_surrounding_cells(cell:Vector3i) -> Array[Vector3i]:
	var ret:Array[Vector3i] = []
	for offset in SURROUNDING:
		if fire_grid.has(cell + offset):
			ret.append(cell + offset)
	return ret

func setup_grid() -> void:
	if col_inst == null:
		for ch in parent.get_children():
			if ch is CollisionShape3D:
				col_inst = ch
				break
	if col_inst == null:
		push_error("No CollisionShape3D found for FireComponentGD!")
		return

	# Compute combined collision AABB
	collision_aabb = get_shape_aabb(col_inst, true)
	cell_size = collision_aabb.size / (grid_resolution as Vector3)

	#prints("Collision AABB:", collision_aabb, "Cell Size:", cell_size)

	# Generate fire grid using collision AABB
	fire_grid.clear()
	for x in range(grid_resolution.x):
		for y in range(grid_resolution.y):
			for z in range(grid_resolution.z):
				var grid_coord = Vector3i(x, y, z)
				var world_pos = collision_aabb.position + Vector3(x, y, z) * cell_size + cell_size / 2.0

				if not is_inside_object(world_pos):
					continue
				var sphere:MeshInstance3D = MeshInstance3D.new()
				var sphere_mesh:SphereMesh = debug_mesh.duplicate()
				sphere_mesh.radius = cell_size.length() * 0.2
				sphere_mesh.height = sphere_mesh.radius * 2
				sphere.mesh = sphere_mesh
				sphere.material_override = debug_mat.duplicate()  # Default material
				add_child(sphere)
				fire_grid[grid_coord] = {
					"hitpoints": -1 if torch else max_hitpoints,
					"burning": false,
					"cooldown": 0.0,
					"time_left": burn_time,
					"local_pos": to_local(world_pos),
					"visual": sphere
				}
				sphere.transform.origin = fire_grid[grid_coord]['local_pos']
				
	if not interior_cells:
		prune_interior_cells()
	
	print(get_parent().name, 'grid size:', fire_grid.size())

func is_on_boundary(cell: Vector3i) -> bool:
	# If any neighbor is missing, this cell is on the boundary.
	for offset in SURROUNDING:
		if not fire_grid.has(cell + offset):
			return true
	return false

func prune_interior_cells() -> void:
	var to_del:Array = []
	for cell in fire_grid.keys():
		if not is_on_boundary(cell):
			to_del.append(cell)
	for d in to_del:
		fire_grid.erase(d)

func update_debug_visuals():
	for cell in fire_grid.keys():
		var cell_data = fire_grid.get(cell, null)
		if not cell_data or not cell_data['visual']:
			print('uh oh')
			continue
		var sphere:MeshInstance3D = cell_data['visual']

		var material = sphere.get_active_material(0) as StandardMaterial3D

		if cell_data["burning"]:
			material.albedo_color = Color(1, 0.4, 0)  # Bright Orange
		elif cell_data["hitpoints"] < max_hitpoints:
			var fire_intensity = 1.0 - float(cell_data["hitpoints"]) / max_hitpoints
			material.albedo_color = Color(1.0, 0.5 * fire_intensity, 0)  # Red-Orange Scaling
		else:
			material.albedo_color = Color(0.5, 0.5, 0.5)  # Default: Grayish

func get_burning_aabb() -> AABB:
	var burn_aabb = null
	for cell in fire_grid.keys():
		if fire_grid[cell]["burning"]:
			var cell_world_pos = to_global(fire_grid[cell]["local_pos"])
			var cell_box = AABB(cell_world_pos, cell_size)
			if burn_aabb == null:
				burn_aabb = cell_box
			else:
				burn_aabb = burn_aabb.merge(cell_box)
	if burn_aabb:
		# Expand by margin on each side.
		burn_aabb.position -= Vector3.ONE * spread_margin
		burn_aabb.size += Vector3.ONE * (spread_margin * 2)
		return burn_aabb
	return burn_aabb if burn_aabb != null else AABB()

func update_burning_area() -> void:
	var b_aabb = get_burning_aabb()
	# If no burning cells exist (or the computed AABB is empty), disable monitoring
	if b_aabb.size == Vector3.ZERO:
		burning_area.monitoring = false
		return
	burning_area.monitoring = true
	
	burn_box.extents = b_aabb.size / 2.0
	
	# Make sure the burning_area has a child collision shape.
	#burning_area.get_child(0).shape = burn_box
	burning_area.global_transform.origin = b_aabb.position + b_aabb.size / 2.0


func check_inter_object_spread() -> void:
	# Update our burning_area before checking overlaps.
	# If the area is not active, skip.
	if not burning_area.monitoring:
		return
	
	on_fire_damage.emit(int(spread_damage*emitters.size()/2.0))
	
	# Get overlapping areas.
	var overlaps = burning_area.get_overlapping_bodies()
	for body in overlaps:
		# Check if the overlapping area belongs to another FireComponentGD.
		# This might be done by checking for a specific group or by casting.
		if body != parent:
			var fc = body.find_child('FireComponentGD')
			if fc:
				fc.apply_fire_damage(burning_area.global_transform.origin, spread_damage)
				
			# For example, apply fire damage to the other object.
			# You may wish to calculate damage based on the overlap or use a fixed value.

func spread_fire(delta: float) -> void:
	for cell in fire_grid.keys():
		var cell_data = fire_grid[cell]
		if cell_data["burning"]:
			var neighbors = get_surrounding_cells(cell)
			for neighbor in neighbors:
				var neighbor_data = fire_grid.get(neighbor, null)
				# Only affect neighbors that are not burning and have no cooldown.
				if neighbor_data and not neighbor_data["burning"] and neighbor_data["cooldown"] <= 0:
					neighbor_data["hitpoints"] -= spread_damage * delta
					if neighbor_data["hitpoints"] <= 0:
						ignite_cell(neighbor)

func _ready():
	parent = get_parent()
	parent.add_to_group('flammable')
	parent.add_to_group('fade_obj')
	burn_col.debug_color = Color.RED
	burn_col.shape = burn_box
	burning_area.add_child(burn_col)
	burning_area.collision_layer = FIRE_LAYER
	burning_area.collision_mask = FLAMMABLE_LAYER
	burning_area.monitorable = false
	burning_area.monitoring = false
	add_child(burning_area)
	burning_area.global_position = parent.global_position
	
	setup_grid()
	if torch:
		for cell in fire_grid.keys():
			ignite_cell(cell)
	print(get_parent().name,' is_convex=', is_convex)

func _process(delta: float) -> void:
	var on_fire:bool = false
	#var is_hot:bool = false
	for cell in fire_grid.keys():
		var data = fire_grid[cell]
		#if data['hitpoints'] < max_hitpoints:
			#is_hot = true
		if data['burning']:
			on_fire = true
		if data["burning"]:
			burning_area.monitoring = true
			burning_area.monitorable = true
			data["time_left"] -= delta
			if data["time_left"] <= 0 and not torch:
				extinguish_cell(cell)
		else:
			# If the cell is on cooldown, decrement it.
			if data["cooldown"] > 0:
				data["cooldown"] = max(0, data["cooldown"] - delta)
			else:
				# Regenerate hitpoints only when not on cooldown.
				if data["hitpoints"] < max_hitpoints:
					data["hitpoints"] = min(max_hitpoints, data["hitpoints"] + spread_damage / 4.0 * delta)
	
	if debug_visualize:
		update_debug_visuals()
	
	if on_fire:
		spread_fire(delta)
		inter_object_timer -= delta
		if inter_object_timer <= 0.0:
			check_inter_object_spread()
			inter_object_timer = spread_interval
	#if not is_hot:
		#clear_grid()
