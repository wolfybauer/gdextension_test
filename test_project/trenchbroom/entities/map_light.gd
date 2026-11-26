@tool
extends OmniLight3D

func _func_godot_apply_properties(properties: Dictionary) -> void:
	var strength := float(properties.get("light", 180.0))
	omni_range = clampf(sqrt(maxf(1.0, strength)) * 0.45, 2.0, 16.0)
	light_energy = clampf(strength / 400.0, 0.10, 2.0)
	shadow_enabled = false
