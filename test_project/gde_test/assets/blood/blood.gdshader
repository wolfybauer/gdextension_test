shader_type spatial;
render_mode blend_mix, cull_disabled, shadows_disabled;

uniform vec4 blood_color : source_color = vec4(0.38, 0.018, 0.029, 1.0);
uniform sampler2D shape_noise : repeat_enable, filter_linear;

varying flat float particle_seed;
varying float particle_opacity;

void vertex() {
	particle_seed = float(INSTANCE_ID);
	particle_opacity = COLOR.a;
	vec3 scale = vec3(length(MODEL_MATRIX[0].xyz), length(MODEL_MATRIX[1].xyz), length(MODEL_MATRIX[2].xyz));
	mat4 billboard = mat4(INV_VIEW_MATRIX[0] * scale.x, INV_VIEW_MATRIX[1] * scale.y,
		INV_VIEW_MATRIX[2] * scale.z, MODEL_MATRIX[3]);
	MODELVIEW_MATRIX = VIEW_MATRIX * billboard;
}

void fragment() {
	vec2 point = UV * 2.0 - 1.0;
	vec2 noise_offset = vec2(particle_seed * 0.173, particle_seed * 0.317);
	float noise = texture(shape_noise, UV + noise_offset).r;
	float edge = mix(0.35, 0.78, noise);
	float radius = length(point);
	float coverage = 1.0 - smoothstep(edge - 0.13, edge, radius);
	float opacity = particle_opacity * coverage;
	if (opacity < 0.01) {
		discard;
	}
	ALPHA = opacity;

	float dome = sqrt(max(0.0, 1.0 - dot(point, point)));
	NORMAL = normalize(vec3(point.x, -point.y, dome));
	ALBEDO = blood_color.rgb * mix(0.55, 1.0, dome);
	ROUGHNESS = 0.22;
	SPECULAR = 0.65;

	// Keep some red in shadow and a small wet highlight at sprite scale.
	float glint = 1.0 - smoothstep(0.06, 0.30, length(point - vec2(-0.24, -0.29)));
	EMISSION = blood_color.rgb * 0.08 + vec3(0.16, 0.052, 0.044) * glint * glint;
}
