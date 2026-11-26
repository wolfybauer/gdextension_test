shader_type spatial;
// render_mode blend_mix, depth_draw_alpha_prepass;

uniform float opacity : hint_range(0.0, 1.0) = 1.0;

uniform bool camera_cutaway = false;

uniform bool use_texture = false;
uniform sampler2D albedo_tex : source_color;
uniform vec4 albedo_color : source_color = vec4(1.0);

void fragment() {
    vec4 base = use_texture
        ? texture(albedo_tex, UV)
        : albedo_color;

    ALBEDO = base.rgb;
    ALPHA = opacity * (camera_cutaway ? smoothstep(0.40, 1.40, -VERTEX.z) : 1.0);
}