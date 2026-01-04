class_name ThrowPoint
extends Node3D

@export var marker_tex:Texture = null
#@export var throw_scene:PackedScene = null

@export var thickness:float = 10.0
@export var colors:Array[Color] = [Color.NAVAJO_WHITE, Color.INDIAN_RED]

@onready var marker:Decal = null
@onready var ddraw:DDraw = DDraw.new()

var current_impulse:float = 5.0
var current_pitch:float = 0.0
var grav_scale:float = 1.0
var g:float = -ProjectSettings.get_setting('physics/3d/default_gravity', 9.8)
var drag:float = ProjectSettings.get_setting('physics/3d/default_linear_damp', 0.0)
var tstep:float = 0.05

# const GRENADE = preload("res://elements/grenade/grenade.tscn")

# const MARKER = preload("res://elements/explosion/textures/T_vfx_hit122.jpg")
func _ready() -> void:
	add_child(ddraw)
	if not marker_tex:
		return
	marker = Decal.new()
	add_child(marker)
	marker.visible = false
	marker.texture_albedo = marker_tex

func get_forward(pitch_deg:float=0):
	return -global_basis.z.rotated(global_basis.x.normalized(), deg_to_rad(pitch_deg))

func draw_aim(impulse:float=5.0, pitch_deg=0.0):
	current_impulse = impulse
	current_pitch = pitch_deg
	
	var vel:Vector3 = get_forward(pitch_deg)
	#DebugDraw.set_highlight_direction(vel)
	vel *= impulse
	
	var start_pos:Vector3 = global_position
	g *= grav_scale
	
	var line_start := start_pos
	var line_end := start_pos
	
	for i in range(1,255):
		vel.y += g * tstep
		vel *= clampf(1.0 - drag * tstep, 0, 1)
		line_end = line_start
		line_end += vel*tstep
		
		var ray:Dictionary = raycast_query(line_start, line_end, colors[i%len(colors)])
		if not ray.is_empty():
			break
		ddraw.draw_line(line_start, line_end-line_start, thickness, colors[i%len(colors)])
		line_start = line_end


func raycast_query(from:Vector3, to:Vector3, color:Color) -> Dictionary:
	var space_state := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.create(from, to, 1)
	query.hit_from_inside = false
	var result := space_state.intersect_ray(query)
	if result:
		#prints(result.collider.name, result)
		ddraw.draw_line(from, result.position-from, thickness, color)
		if marker:
			marker.reparent(result.collider)
			marker.global_position = result.position + result.normal * 0.1
			#marker.global_position = Vector3.ZERO
			#var up := Vector3.FORWARD if abs(result.normal.dot(Vector3.UP)) > 0.95 else Vector3.UP
			#marker.global_transform = Transform3D(
				#Basis.looking_at(-result.normal, up),
				#result.position + result.normal * 0.01
			#)

			marker.visible = true

		return result
	else:
		if marker:
			marker.visible = false
		return {}

#func throw():
	#marker.visible = false
	#var obj:RigidBody3D = GRENADE.instantiate()
	#get_tree().current_scene.add_child(obj)
	#obj.global_position = global_position
	#var dir:Vector3 = get_forward(current_pitch)
	#obj.linear_velocity = dir * current_impulse
	#obj.apply_torque(Vector3.RIGHT * randi_range(-5,5))
