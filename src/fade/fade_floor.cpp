#include "fade/fade_floor.hpp"
#include "fade/geometry.hpp"


#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/variant/vector3.hpp"


using namespace godot;
using namespace fade_geometry;

float FadeFloor3D::s_global_y_margin = DEFAULT_FADE_Y_MARGIN;


void FadeFloor3D::check_fade(Node3D * target, float max_dist) {
    Vector3 tp = target->get_global_position();
    Vector3 myp = get_global_position();

    // early out: target not below
    if (tp.y + *_y_margin >= myp.y) {
        set_visible(true);
        return;
    }

    if (max_dist > 0.0f) {
        Vector2 t2(tp.x, tp.z);

        // clamp target to floor AABB in XZ
        Vector2 closest(
            Math::clamp(t2.x, _aabb_min.x, _aabb_max.x),
            Math::clamp(t2.y, _aabb_min.y, _aabb_max.y)
        );

        float dist = t2.distance_to(closest);
        if (dist > max_dist) {
            set_visible(true);
            return;
        }
    }

    set_visible(false);
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

    _y_margin = &s_global_y_margin;

    precompute_bounds(mesh_inst, _aabb_min, _aabb_max, _center2d, _wall_normal); // TODO add error code
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

    if(Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    Array floors = get_tree()->get_nodes_in_group("fade_floor");
    for (int i = 0; i < floors.size(); i++) {
        FadeFloor3D *fw = Object::cast_to<FadeFloor3D>(floors[i]);
        if (!fw) {
            continue;
        }

        if (!fw->y_margin_override_enable) {
            fw->_y_margin = &s_global_y_margin;
        }
    }
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
    _y_margin = e ? &y_margin_override : &s_global_y_margin;
}

bool FadeFloor3D::get_margin_override_enable() const {
    return y_margin_override_enable;
}
