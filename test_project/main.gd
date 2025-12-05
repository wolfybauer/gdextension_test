extends Node3D

var popped_once:bool = false

func _unhandled_input(_event: InputEvent) -> void:
	if Input.is_action_just_pressed('ui_cancel'):
		get_tree().quit()
	if Input.is_action_just_pressed('ui_accept'):
		pop_boxes()

func _ready() -> void:
	for n in get_children(true):
		for g in n.get_children():
			print(g.name)
			if g is DestronoiNode:
				print(n.name, 'generated', g.name)
				g.generate()

func pop_boxes():
	if not popped_once:
		for n in get_children():
			if n.visible and 'Box' in n.name:
				var b = n.find_child('Destronoi*')
				if b: b.destroy(5, 5, 6.0)
		popped_once = true
	else:
		for n in get_children():
			if n.visible and 'Glass' in n.name:
				var b = n.find_child('Destronoi*')
				if b: b.destroy(5, 5, 6.0)
