#include "fade/fade_floor.hpp"
#include "fade/geometry.hpp"


#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/variant/vector3.hpp"

#include <vector>


using namespace godot;
using namespace fade_geometry;

float FadeFloor3D::s_global_y_margin = DEFAULT_FADE_Y_MARGIN;

float FadeFloor3D::distance_to_footprint(const Vector3 &position) const {
    return get_xz_dist(position, _aabb_min, _aabb_max);
}

bool FadeFloor3D::should_hide(const Vector3 &target, float max_dist) const {
    const float margin = y_margin_override_enable ? y_margin_override : s_global_y_margin;
    return target.y + margin < get_global_position().y &&
            (max_dist <= 0.0f || distance_to_footprint(target) <= max_dist);
}

void FadeFloor3D::check_fade(Node3D *target, float max_dist) {
    if (target && is_in_group("fade_floor")) {
        set_visible(!should_hide(target->get_global_position(), max_dist));
    }
}

void FadeFloor3D::check_fade_floors(SceneTree * tree, Node3D * target, float max_dist) {
    if(!tree || !target) {
        UtilityFunctions::push_error("[FadeFloor3D] check_fade_floors : tree or target is null. abort");
        return;
    }
    tree->call_group("fade_floor", "check_fade", target, max_dist);
}


void FadeFloor3D::check_fade_objects(SceneTree * tree, Node3D * target, float max_dist) {
    if(!tree || !target) {
        UtilityFunctions::push_error("[FadeFloor3D] check_fade_objects : tree or target is null. abort");
        return;
    }

    const Vector3 target_position = target->get_global_position();
    std::vector<FadeFloor3D *> overhead_floors;
    for (const Variant &node : tree->get_nodes_in_group("fade_floor")) {
        FadeFloor3D *floor = Object::cast_to<FadeFloor3D>(node);
        if (floor && floor->should_hide(target_position, max_dist)) {
            overhead_floors.push_back(floor);
        }
    }

    for (const Variant &node : tree->get_nodes_in_group("fade_obj")) {
        Node3D *object = Object::cast_to<Node3D>(node);
        if (!object || object == target) {
            continue;
        }

        const Vector3 object_position = object->get_global_position();
        bool covered = false;
        for (const FadeFloor3D *floor : overhead_floors) {
            if (object_position.y >= floor->get_global_position().y &&
                    floor->distance_to_footprint(object_position) == 0.0f) {
                covered = true;
                break;
            }
        }
        object->set_visible(!covered);
    }
}

void FadeFloor3D::_on_ready() {
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
        UtilityFunctions::push_error("[FadeFloor3D] ", get_name(), " _on_ready : MeshInstance3D not found. abort");
        return;
    }

    if (mesh_inst->get_mesh().is_null()) {
        UtilityFunctions::push_error("[FadeFloor3D] ", get_name(), " has no floor mesh");
        return;
    }

    const AABB bounds = mesh_inst->get_global_transform().xform(mesh_inst->get_aabb());
    const Vector3 end = bounds.get_end();
    _aabb_min = Vector2(bounds.position.x, bounds.position.z);
    _aabb_max = Vector2(end.x, end.z);
    add_to_group("fade_floor");
}

void FadeFloor3D::_notification(int p_what) {
    switch (p_what) {
		case NOTIFICATION_READY: {
            _on_ready();
            break;
        }

        default:
            break;
    }
}

void FadeFloor3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("check_fade", "target", "max_dist"), &FadeFloor3D::check_fade);
    ClassDB::bind_static_method("FadeFloor3D", D_METHOD("check_fade_objects", "tree", "target", "max_dist"), &FadeFloor3D::check_fade_objects);
    ClassDB::bind_static_method("FadeFloor3D", D_METHOD("check_fade_floors", "tree", "target", "max_dist"), &FadeFloor3D::check_fade_floors);
    
    ClassDB::bind_method(D_METHOD("set_global_y_margin", "height"), &FadeFloor3D::set_global_y_margin);
    ClassDB::bind_method(D_METHOD("get_global_y_margin"), &FadeFloor3D::get_global_y_margin);
    
    ClassDB::bind_method(D_METHOD("set_y_margin_override", "height"), &FadeFloor3D::set_y_margin_override);
    ClassDB::bind_method(D_METHOD("get_y_margin_override"), &FadeFloor3D::get_y_margin_override);
    ClassDB::bind_method(D_METHOD("set_margin_override_enable", "en"), &FadeFloor3D::set_margin_override_enable);
    ClassDB::bind_method(D_METHOD("get_margin_override_enable"), &FadeFloor3D::get_margin_override_enable);

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

void FadeFloor3D::set_global_y_margin(float m) {
    s_global_y_margin = m;
}

float FadeFloor3D::get_global_y_margin() const {
    return s_global_y_margin;
}

void FadeFloor3D::set_y_margin_override(float m) {
    y_margin_override = m;
}

float FadeFloor3D::get_y_margin_override() const {
    return y_margin_override;
}

void FadeFloor3D::set_margin_override_enable(bool e) {
    y_margin_override_enable = e;
}

bool FadeFloor3D::get_margin_override_enable() const {
    return y_margin_override_enable;
}
