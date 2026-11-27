shader_type spatial;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap, repeat_enable;
uniform vec4 albedo_color : source_color = vec4(1.0);
uniform bool camera_cutaway = true;
uniform bool use_texture = false;
uniform float roughness : hint_range(0.0, 1.0) = 0.85;

varying float camera_depth;

void vertex() {
	vec3 world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	vec3 camera_position = MAIN_CAM_INV_VIEW_MATRIX[3].xyz;
	camera_depth = dot(camera_position - world_position, MAIN_CAM_INV_VIEW_MATRIX[2].xyz);
}

void fragment() {
	float coverage = smoothstep(0.30, 1.65, camera_depth);
	// Screen-space dither keeps the geometry in the opaque depth pass.
	float threshold = fract(52.9829189 * fract(dot(FRAGCOORD.xy, vec2(0.06711056, 0.00583715))));
	if (camera_cutaway && coverage <= threshold) {
		discard;
	}
	ALBEDO = albedo_color.rgb * (use_texture ? texture(albedo_texture, UV).rgb : vec3(1.0));
	ROUGHNESS = roughness;
}
