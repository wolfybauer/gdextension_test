#include "fade/fade_wall.hpp"
#include "fade/geometry.hpp"

#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/camera3d.hpp"
#include "godot_cpp/classes/mesh.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/resource_loader.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/classes/texture.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include <cmath>

using namespace godot;
using namespace fade_geometry;

int FadeWall3D::s_global_render_priority = 0;
float FadeWall3D::s_global_y_margin = DEFAULT_FADE_Y_MARGIN;

void FadeWall3D::set_opacity(float op) {
    op = Math::clamp<float>(op, 0.0f,1.0f);
    _fade_target = op;
}


void FadeWall3D::check_fade(Node3D *target, Camera3D *camera, float max_dist, float fade_speed, float min_alpha) {
    if (!_fade_mat.is_valid() || !target || !camera) {
        return;
    }

    _fade_speed = MAX(fade_speed, 0.0f);
    const Vector3 target_position = target->get_global_position();
    if (camera->is_position_behind(target_position)) {
        set_opacity(1.0f);
        return;
    }

    const Vector3 ray_origin = camera->project_ray_origin(camera->unproject_position(target_position));
    const Transform3D world_to_wall = get_global_transform().affine_inverse();
    if (!_local_aabb.intersects_segment(world_to_wall.xform(ray_origin), world_to_wall.xform(target_position))) {
        set_opacity(1.0f);
        return;
    }

    const float height_difference = target_position.y - get_global_position().y;
    if (height_difference < -*_y_margin) {
        const bool nearby = max_dist <= 0.0f || get_xz_dist(target_position, _aabb_min, _aabb_max) <= max_dist;
        set_opacity(nearby ? 0.0f : 1.0f);
        return;
    }

    const Vector2 target_xz(target_position.x, target_position.z);
    const float wall_distance = Math::abs((target_xz - _center2d).dot(_wall_normal));
    if (height_difference > *_y_margin || (max_dist > 0.0f && wall_distance > max_dist)) {
        set_opacity(1.0f);
        return;
    }

    set_opacity(min_alpha);
}

void FadeWall3D::check_fade_walls(SceneTree * tree, Node3D * target, Camera3D * camera, float max_dist, float fade_speed, float min_alpha) {
    if(!tree || !target || !camera) {
        UtilityFunctions::push_error("[FadeWall3D] check_fade_walls : tree or target or camera is null. abort");
        return;
    }
    tree->call_group("fade_wall", "check_fade", target, camera, max_dist, fade_speed, min_alpha);
}

void FadeWall3D::_on_ready() {
    if(Engine::get_singleton()->is_editor_hint()) {
		return;
	}

    MeshInstance3D * mesh_inst = nullptr;
    for(auto & v : get_children()) {
        mesh_inst = Object::cast_to<MeshInstance3D>(v);
        if(mesh_inst != nullptr) {
            break;
        }
    }
    if(mesh_inst == nullptr) {
        UtilityFunctions::push_error("[FadeWall3D] ", get_name(), " _on_ready : MeshInstance3D not found. abort");
        return;
    }

    const Ref<Mesh> mesh = mesh_inst->get_mesh();
    if (mesh.is_null() || mesh->get_surface_count() == 0) {
        UtilityFunctions::push_error("[FadeWall3D] ", get_name(), " has no mesh surfaces");
        return;
    }
    _local_aabb = mesh_inst->get_transform().xform(mesh_inst->get_aabb());

    _fade_shader = ResourceLoader::get_singleton()->load(FADE_SHADER_PATH);
    if(!_fade_shader.is_valid()) {
        UtilityFunctions::push_error("[FadeWall3D] ", get_name(), " _on_ready : error loading shader. abort");
        return;
    }
    _fade_mat.instantiate();
    _fade_mat->set_shader(_fade_shader);

    // assign render prio / y margin pointers
    _render_prio = render_prio_override_enable ? &render_prio_override : &s_global_render_priority;
    _y_margin = y_margin_override_enable ? &y_margin_override : &s_global_y_margin;
    _apply_render_priority();

    // get / set up the texture/color we'll be fading
    Ref<StandardMaterial3D> old = mesh->surface_get_material(0);
    if(old.is_valid()) {
        Ref<Texture> tex = old->get("albedo_texture");
        if(tex.is_valid()) {
            _fade_mat->set_shader_parameter("albedo_tex", tex);
            _fade_mat->set_shader_parameter("use_texture", true);
            _fade_mat->set_shader_parameter("albedo_color", Color(1,1,1,1));
        } else {
            _fade_mat->set_shader_parameter("albedo_color", old->get_albedo());
            _fade_mat->set_shader_parameter("use_texture", false);
        }
    }
    mesh_inst->set_surface_override_material(0, _fade_mat);

    precompute_bounds(mesh_inst, _aabb_min, _aabb_max, _center2d, _wall_normal); // TODO add error code

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
    const float weight = Math::clamp(_fade_speed * delta, 0.0f, 1.0f);
    _fade_amount = Math::lerp(_fade_amount, _fade_target, weight);
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
    ClassDB::bind_static_method("FadeWall3D", D_METHOD("check_fade_walls", "tree", "target", "camera", "max_dist", "fade_speed", "min_alpha"), &FadeWall3D::check_fade_walls);
    
    ClassDB::bind_method(D_METHOD("set_global_render_priority", "lvl"), &FadeWall3D::set_global_render_priority);
    ClassDB::bind_method(D_METHOD("get_global_render_priority"), &FadeWall3D::get_global_render_priority);

    ClassDB::bind_method(D_METHOD("set_render_priority_override", "lvl"), &FadeWall3D::set_render_priority_override);
    ClassDB::bind_method(D_METHOD("get_render_priority_override"), &FadeWall3D::get_render_priority_override);
    ClassDB::bind_method(D_METHOD("set_priority_override_enable", "en"), &FadeWall3D::set_priority_override_enable);
    ClassDB::bind_method(D_METHOD("get_priority_override_enable"), &FadeWall3D::get_priority_override_enable);

    ClassDB::bind_method(D_METHOD("set_global_y_margin", "height"), &FadeWall3D::set_global_y_margin);
    ClassDB::bind_method(D_METHOD("get_global_y_margin"), &FadeWall3D::get_global_y_margin);
    
    ClassDB::bind_method(D_METHOD("set_y_margin_override", "height"), &FadeWall3D::set_y_margin_override);
    ClassDB::bind_method(D_METHOD("get_y_margin_override"), &FadeWall3D::get_y_margin_override);
    ClassDB::bind_method(D_METHOD("set_margin_override_enable", "en"), &FadeWall3D::set_margin_override_enable);
    ClassDB::bind_method(D_METHOD("get_margin_override_enable"), &FadeWall3D::get_margin_override_enable);

    ADD_PROPERTY(
        PropertyInfo(
            Variant::INT,
            "rendering/global_render_priority"
        ),
        "set_global_render_priority",
        "get_global_render_priority"
    );
    
    ADD_PROPERTY(
        PropertyInfo(
            Variant::INT,
            "rendering/render_priority_override"
        ),
        "set_render_priority_override",
        "get_render_priority_override"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::BOOL,
            "rendering/priority_override_enable"
        ),
        "set_priority_override_enable",
        "get_priority_override_enable"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::FLOAT,
            "rendering/global_y_margin",
            PROPERTY_HINT_RANGE,
            "0.0,10.0,0.1"
        ),
        "set_global_y_margin",
        "get_global_y_margin"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::FLOAT,
            "rendering/y_margin_override",
            PROPERTY_HINT_RANGE,
            "0.0,10.0,0.1"
        ),
        "set_y_margin_override",
        "get_y_margin_override"
    );

    ADD_PROPERTY(
        PropertyInfo(
            Variant::BOOL,
            "rendering/margin_override_enable"
        ),
        "set_margin_override_enable",
        "get_margin_override_enable"
    );
}

void FadeWall3D::_apply_render_priority() {
    if (!_fade_mat.is_valid() || !_render_prio) {
        return;
    }
    _fade_mat->set_render_priority(*_render_prio);
}


void FadeWall3D::set_global_render_priority(int lvl) {
    s_global_render_priority = lvl;

    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    Array walls = get_tree()->get_nodes_in_group("fade_wall");
    for (int i = 0; i < walls.size(); i++) {
        FadeWall3D *fw = Object::cast_to<FadeWall3D>(walls[i]);
        if (!fw) {
            continue;
        }

        if (!fw->render_prio_override_enable) {
            fw->_render_prio = &s_global_render_priority;
            fw->_apply_render_priority();
        }
    }
}


int FadeWall3D::get_global_render_priority() const {
    return s_global_render_priority;
}

void FadeWall3D::set_render_priority_override(int lvl) {
    render_prio_override = lvl;
    if (render_prio_override_enable) {
        _apply_render_priority();
    }
}

int FadeWall3D::get_render_priority_override() const {
    return render_prio_override;
}

void FadeWall3D::set_priority_override_enable(bool e) {
    render_prio_override_enable = e;
    _render_prio = e ? &render_prio_override : &s_global_render_priority;
    _apply_render_priority();
}

bool FadeWall3D::get_priority_override_enable() const {
    return render_prio_override_enable;
}

void FadeWall3D::set_global_y_margin(float m) {
    s_global_y_margin = m;

    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    Array walls = get_tree()->get_nodes_in_group("fade_wall");
    for (int i = 0; i < walls.size(); i++) {
        FadeWall3D *fw = Object::cast_to<FadeWall3D>(walls[i]);
        if (!fw) {
            continue;
        }

        if (!fw->y_margin_override_enable) {
            fw->_y_margin = &s_global_y_margin;
        }
    }
}

float FadeWall3D::get_global_y_margin() const {
    return s_global_y_margin;
}

void FadeWall3D::set_y_margin_override(float m) {
    y_margin_override = m;
}

float FadeWall3D::get_y_margin_override() const {
    return y_margin_override;
}

void FadeWall3D::set_margin_override_enable(bool e) {
    y_margin_override_enable = e;
    _y_margin = e ? &y_margin_override : &s_global_y_margin;
}

bool FadeWall3D::get_margin_override_enable() const {
    return y_margin_override_enable;
}
