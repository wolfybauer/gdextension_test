class_name NameLabel
extends Label3D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	text = get_parent().name
	billboard = BaseMaterial3D.BILLBOARD_ENABLED
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	pass
