#include "fade/fade_wall.hpp"
#include "godot_cpp/classes/engine.hpp"
// #include "godot_cpp/classes/resource_loader.hpp"
#include "godot_cpp/classes/camera3d.hpp"
#include "godot_cpp/classes/mesh.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include <cmath>

using namespace godot;

static inline bool _point_in_aabb(Vector2 p, Vector2 aabbmin, Vector2 aabbmax) {
    return (
        p.x >= aabbmin.x && p.x <= aabbmax.x &&
        p.y >= aabbmin.y && p.y <= aabbmax.y
    );
}

static bool _seg_seg(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4) {
    Vector2 d1 = p2 - p1;
    Vector2 d2 = p4 - p3;

    float denom = d1.cross(d2);
    if(Math::abs<float>(denom) < 0.000001f) {
        return false;
    }

    float t = (p3 - p1).cross(d2) / denom;
    float u = (p3 - p1).cross(d1) / denom;

    return (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f);
}

static bool _segment_hits_aabb(Vector2 cam2, Vector2 ply2, Vector2 aabbmin, Vector2 aabbmax) {
    if(_point_in_aabb(cam2, aabbmin, aabbmax) || _point_in_aabb(ply2, aabbmin, aabbmax)) {
        return true;
    }
    if (_seg_seg(cam2, ply2, Vector2(aabbmin.x, aabbmin.y), Vector2(aabbmin.x, aabbmax.y))) { return true; }
	if (_seg_seg(cam2, ply2, Vector2(aabbmax.x, aabbmin.y), Vector2(aabbmax.x, aabbmax.y))) { return true; }
	if (_seg_seg(cam2, ply2, Vector2(aabbmin.x, aabbmin.y), Vector2(aabbmax.x, aabbmin.y))) { return true; }
	if (_seg_seg(cam2, ply2, Vector2(aabbmin.x, aabbmax.y), Vector2(aabbmax.x, aabbmax.y))) { return true; }
    return false;
}

static bool _projected_outside(Vector2 cam2, Vector2 ply2, Vector2 center2d) {
    Vector2 v = ply2 - cam2;
    Vector2 w = center2d - cam2;
	// if dot signs differ or wall lies further than target -> no occlusion
    return (v.dot(w) <= 0.0f || w.length() > v.length());
}

void FadeWall3D::_precompute_bounds(MeshInstance3D * mesh_inst) {
    Ref<Mesh> mesh = mesh_inst->get_mesh();
    if(!mesh.is_valid()) {
        UtilityFunctions::push_error("[FadeWall] ", get_name(), " _precompute_bounds : Mesh is invalid. abort");
        return;
    }
    
    float minx = INFINITY;
    float maxx = -INFINITY;

    float minz = INFINITY;
    float maxz = -INFINITY;

    float maxy = -INFINITY;

    Transform3D xf = mesh_inst->get_global_transform();
    Array arr = mesh->surface_get_arrays(0)[Mesh::ARRAY_VERTEX];

    for(int i=0; i<arr.size(); i++) {
        Vector3 v = arr[i];
        Vector3 w = xf.xform(v);
        minx = MIN(minx, w.x);
		maxx = MAX(maxx, w.x);

		minz = MIN(minz, w.z);
		maxz = MAX(maxz, w.z);

		maxy = MAX(maxy, w.y);
    }

    _aabb_min = Vector2(minx, minz);
    _aabb_max = Vector2(maxx, maxz);
    _center2d = (_aabb_min + _aabb_max) * 0.5f;

    float dx = Math::abs<float>(_aabb_max.x - _aabb_min.x);
    float dz = Math::abs<float>(_aabb_max.y - _aabb_min.y);

    // pillar / square
    if(Math::abs<float>(dx - dz) < 0.1f) {
        _wall_normal = Vector2(0,0);
    // z-facing
    } else if(dx > dz) {
        _wall_normal = Vector2(0,1);
    // x-facing
    } else {
        _wall_normal = Vector2(1,0);
    }
}

void FadeWall3D::check_fade(Node3D * target, Camera3D * camera, float max_dist, float fade_speed, float min_alpha)
{
    // // TODO: this is fade floor logic. can prob skip by just adding to fade_obj group
    // if(target->get_global_position().y + TOP_MARGIN < get_global_position().y) {
    //     _fade_target = min_alpha;
    //     set_visible(false);
    //     return;
    // }
    // set_visible(true);
    // // TODO: unset fade if doing this

    Vector3 tp = target->get_global_position();
    Vector2 ply2 = Vector2(tp.x, tp.z);

    if(max_dist > 0.0f && !_wall_normal.is_zero_approx()) {
        _target_dist = (ply2 - _center2d).dot(_wall_normal);
        if(Math::abs<float>(_target_dist) > max_dist) {
            _fade_target = 1.0f;
            return;
        }
    }


    Transform3D ct = camera->get_global_transform();
    enum Camera3D::ProjectionType p = camera->get_projection();
    if(p == Camera3D::PROJECTION_PERSPECTIVE) {
        Vector2 cam2 = Vector2(ct.origin.x, ct.origin.z);
        if(_projected_outside(cam2, ply2, _center2d) ||
            !_segment_hits_aabb(cam2, ply2, _aabb_min, _aabb_max)) {
            _fade_target = 1.0f;
        } else {
            _fade_target = min_alpha;
        }
    } else if(p == Camera3D::PROJECTION_ORTHOGONAL) {
        Vector3 fwd = -ct.basis.get_column(2);
        fwd.y = 0.0f;
        float yaw = Math::atan2(fwd.x, fwd.z);

        // pillar? skip back/front test
        if(!_wall_normal.is_zero_approx()) {
            Vector2 rel_norm = _wall_normal.rotated(-yaw);
            if((ply2 - _center2d).dot(rel_norm.rotated(camera->get_rotation().y)) <= 0.0f) {
                _fade_target = 1.0f;
                return;
            }
        }

        Vector2 proj_dir = Vector2(fwd.x, fwd.z).normalized();
        Vector2 a = ply2 - proj_dir * 9999.0f;
        Vector2 b = ply2 + proj_dir * 9999.0f;
        
        if(_segment_hits_aabb(a, b, _aabb_min, _aabb_max)) {
            _fade_target = min_alpha;
        } else {
            _fade_target = 1.0f;
        }
    } else {
        UtilityFunctions::push_error("[FadeWall] ", get_name(), " check_fade : unsupported camera type");
    }
}

void FadeWall3D::_on_ready() {
    MeshInstance3D * mesh_inst = nullptr;
    for(auto & v : get_children()) {
        mesh_inst = Object::cast_to<MeshInstance3D>(v);
        if(mesh_inst != nullptr) {
            break;
        }
    }
    if(mesh_inst == nullptr) {
        UtilityFunctions::push_error("[FadeWall] ", get_name(), " _on_ready : MeshInstance3D not found. abort");
        return;
    }

    _fade_mat.instantiate();
    _fade_shader.instantiate();
    _fade_shader->set_code(R"(
        shader_type spatial;
        // render_mode blend_mix, depth_draw_alpha_prepass;

        uniform float opacity : hint_range(0.0, 1.0) = 1.0;
        uniform sampler2D _albedo : source_color;

        void fragment() {
            ALBEDO = texture(_albedo, UV).rgb;
            ALPHA = opacity;
        }
    )");
    _fade_mat->set_shader(_fade_shader);
    if(!_fade_mat.is_valid()) {
        UtilityFunctions::push_error("[FadeWall] ", get_name(), "_on_ready : _fade_mat invalid. abort");
        return;
    }

    // TODO set render priority
    Ref<StandardMaterial3D> old = mesh_inst->get_mesh()->surface_get_material(0);
    if(old.is_valid()) {
        // TODO rest of shader stuff
        _fade_mat->set_shader_parameter("_albedo", old->get("albedo_texture"));
    }
    mesh_inst->set_surface_override_material(0, _fade_mat);

    _precompute_bounds(mesh_inst); // TODO add error code

    add_to_group("fade_wall");
    set_physics_process(true);
}

void FadeWall3D::_on_physics_process(float delta) {
    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }
    if(Math::is_equal_approx(_fade_amount, _fade_target)) {
        return;
    }
    if(!_fade_mat.is_valid()) {
        return;
    }
    _fade_amount = Math::lerp(_fade_amount, _fade_target, _fade_speed * delta);
    _fade_mat->set_shader_parameter("opacity", _fade_amount);
}

void FadeWall3D::_notification(int p_what) {
    switch (p_what) {
		case NOTIFICATION_READY: {
            _on_ready();
            break;
        }

        case NOTIFICATION_PHYSICS_PROCESS: {
            _on_physics_process((float)get_physics_process_delta_time());
            break;
        }
        default:
            break;
    }
}

void FadeWall3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("check_fade", "target", "camera", "max_dist", "fade_speed", "min_alpha"), &FadeWall3D::check_fade);
}
