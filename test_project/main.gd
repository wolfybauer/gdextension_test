extends Node3D

var popped_once:bool = false

func _unhandled_input(_event: InputEvent) -> void:
	if Input.is_action_just_pressed('ui_cancel'):
		get_tree().quit()
	if Input.is_action_just_pressed('ui_accept'):
		pop_boxes()

func pop_some(filter:String):
	for n in get_children():
		if not (filter in n.name and n.visible):
			continue
		var b = n.find_child('Destronoi*')
		if not b: continue
		if b is DestronoiNode:
			b.destroy(5.0, Vector3(5,0,0))
		else:
			(b as DestronoiGD).destroy(5,5,6.0)
func pop_boxes():
	if not popped_once:
		pop_some('Box')
		popped_once = true
	else:
		pop_some('Glass')
