class_name DDraw
extends Node

@onready var _parent:Node3D
@onready var mesh:ImmediateMesh = ImmediateMesh.new()
@onready var instance:MeshInstance3D = MeshInstance3D.new()

var xform:Transform3D:
	get:
		if not _parent: return Transform3D()
		return _parent.global_transform

const EPSILON:float = 0.00001
const strip_order:Array = [4,5,0,1,2,5,6,4,7,0,3,2,7,6]

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	_parent = get_parent() as Node3D
	if !_parent:
		push_error('no parent Node3D found. abort')
		return
	instance.mesh = mesh
	await _parent.ready
	_parent.add_child(instance)
	
func _physics_process(_delta: float) -> void:
	mesh.clear_surfaces()

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	pass

func draw_line(from:Vector3, to:Vector3, thickness:float=2.0, color:Color=Color.BLUE):
	var local_b = to
	to = from + to
	
	if from.is_equal_approx(to):
		return

	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLE_STRIP)
	mesh.surface_set_color(color)
	var dir:Vector3 = from.direction_to(to)
	
	# draw cube line
	var normal:Vector3
	if (abs(dir.x) + abs(dir.y) > EPSILON):
		normal = Vector3(-dir.y, dir.x, 0).normalized()
	else:
		normal = Vector3(0, -dir.z, dir.y).normalized()
	normal *= thickness / 100.0
	
	var vertex:Vector3
	for v in range(len(strip_order)):
		#if mesh.get_surface_count() == RenderingServer.MAX_MESH_SURFACES:
			#break
		if strip_order[v] < 4: vertex = normal
		else: vertex = normal + local_b
		vertex = vertex.rotated(dir, PI * (0.5 * (strip_order[v] % 4) + 0.25))
		vertex += from
		mesh.surface_add_vertex(vertex)
		#mesh.surface_set_normal(vertex)
		
	mesh.surface_end()
	#print('draw from % to %', [from, to])

func draw_line_relative(to:Vector3, thickness:float=2.0, color:Color=Color.BLUE):
	draw_line(xform.origin, xform.origin+to, thickness, color)
