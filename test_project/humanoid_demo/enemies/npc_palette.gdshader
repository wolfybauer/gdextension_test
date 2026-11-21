shader_type spatial;

uniform sampler2D palette : source_color, filter_nearest;
uniform vec3 skin : source_color;
uniform vec3 coat : source_color;
uniform vec3 clothes : source_color;

float hue(vec3 color) {
	float maximum = max(color.r, max(color.g, color.b));
	float chroma = maximum - min(color.r, min(color.g, color.b));
	if (chroma < 0.01) {
		return -1.0;
	}
	float sector = maximum == color.r ? (color.g - color.b) / chroma :
		(maximum == color.g ? 2.0 + (color.b - color.r) / chroma : 4.0 + (color.r - color.g) / chroma);
	return fract(sector / 6.0);
}

void fragment() {
	vec3 original = texture(palette, UV).rgb;
	float tint = hue(original);
	float shade = mix(0.62, 1.0, max(original.r, max(original.g, original.b)));
	vec3 color = original;
	// The recovered palette uses magenta, green, and violet as channel markers.
	if (tint > 0.82 && tint < 0.96) {
		color = skin * shade;
	} else if (tint > 0.28 && tint < 0.41) {
		color = coat * shade;
	} else if (tint > 0.68 && tint < 0.82) {
		color = clothes * shade;
	}
	ALBEDO = color;
	ROUGHNESS = 0.88;
}
