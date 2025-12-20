#include "fire.hpp"
#include "godot_cpp/classes/array_mesh.hpp"
#include "godot_cpp/classes/box_shape3d.hpp"
#include "godot_cpp/classes/sphere_shape3d.hpp"
#include "godot_cpp/classes/capsule_shape3d.hpp"
#include "godot_cpp/classes/collision_shape3d.hpp"
#include "godot_cpp/classes/convex_polygon_shape3d.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/shape3d.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/variant/aabb.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"
#include "godot_cpp/variant/plane.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include <cmath>
#include <vector>

using namespace godot;

const Vector3i SURROUNDING[6] = {
    Vector3i(1, 0, 0),
    Vector3i(-1, 0, 0),

    Vector3i(0, 1, 0),
    Vector3i(0, -1, 0),

    Vector3i(0, 0, 1),
    Vector3i(0, 0, -1),
};


static AABB _get_shape_aabb(CollisionShape3D * collision_shape, bool * is_convex) {
    *is_convex = false;
    Ref<Shape3D> shape = collision_shape->get_shape();

    // optimize for boxshape
    Ref<BoxShape3D> box = shape;
    if(box.is_valid()) {
        return AABB(-box->get_size() / 2.0f, box->get_size());
    }
    
    // optimize for sphere
    Ref<SphereShape3D> sphere = shape;
    if(sphere.is_valid()) {
        float r = sphere->get_radius();
        return AABB(Vector3(-r,-r,-r), Vector3(2*r,2*r,2*r));
    }
    
    // optimize for capsule
    Ref<CapsuleShape3D> caps = shape;
    if(caps.is_valid()) {
        return AABB();
    }
    
    // convex polygon
    Ref<ConvexPolygonShape3D> cvx = shape;
    if(cvx.is_valid()) {
        *is_convex = true;
        return AABB();
    }

    // warn or error
    UtilityFunctions::print("[Enflame] _get_shape_aabb : CollisionShape3D has no shape");
    return AABB();
}

static void _build_convex_planes(CollisionShape3D * inst, std::vector<Plane> &planes) {
    Ref<ConvexPolygonShape3D> shape = inst->get_shape();
    if(shape.is_null()) {
        UtilityFunctions::print("[Enflame] _get_shape_aabb : shape type not supported");

    }
    planes.clear();
    PackedVector3Array points = shape->get_points();
    if(points.size() < 4) {
        return;
    }

    // computer center
    Vector3 center = Vector3(0,0,0);
    for(Vector3 p : points) {
        center += p;
    }
    center /= points.size();
    int sz = points.size();
    for(int i=0; i<sz; i++) {
        for(int j=i+1; j<sz; j++) {
            for(int k=j+1; k<sz; k++) {
                Vector3 a = points[i];
                Vector3 b = points[j];
                Vector3 c = points[k];

                if((b - a).cross(c - a).length_squared() < 1e-6f) {
                    continue;
                }

                Plane plane = Plane(a, b, b);

                if(plane.distance_to(center) > 0.0f) {
                    plane = Plane(-plane.normal, -plane.d);
                }

                bool valid = true;
                for(Vector3 p : points) {
                    if(plane.distance_to(p) > 0.001f) {
                        valid = false;
                        break;
                    }
                }

                if(!valid) {
                    continue;
                }

                // remove duplicates
                bool dup = false;
                for(Plane q : planes) {
                    if((plane.normal.dot(q.normal) > 0.999f) && (Math::abs(plane.d - q.d) < 0.001f)) {
                        dup = true;
                        break;
                    }
                }
                if(!dup) {
                    planes.push_back(plane);
                }
            }
        }
    }
}

bool FireComponent3D::_is_inside_object(Vector3 world_pos) {
    if(!_collision_aabb.has_point(world_pos)) {
        return false;
    }

    if(!_is_convex) {
        return true;
    }

    Vector3 loc = _col_inst->get_global_transform().affine_inverse().get_origin() * world_pos;
    for(Plane plane : _convex_planes) {
        if(plane.distance_to(loc) > 0.001f) {
            return false;
        }
    }
    return true;
}



fire_cell_t * FireComponent3D::_get_closest_cell(Vector3 world_pos, Vector3i * cell) {
    float min_dist = INFINITY;
    fire_cell_t * closest = nullptr;

    for(auto &kv : _grid) {
        fire_cell_t &data = kv.second;
        float dist = world_pos.distance_squared_to(to_global(data.local_pos));
        if(dist > min_dist) {
            continue;
        }
        min_dist = dist;
        closest = &data;
        *cell = kv.first;
    }
    return closest;
}

void FireComponent3D::apply_fire(Vector3 world_pos, int damage) {
    if(damage < 1) {
        UtilityFunctions::print("[Enflame] apply_fire : invalid damage=", damage);
        return;    
    }
    Vector3i pos = Vector3i();
    fire_cell_t * cell = _get_closest_cell(world_pos, &pos);
    if(cell == nullptr) {
        UtilityFunctions::print("[Enflame] apply_fire : closest cell is null");
        return;    
    }

    cell->hitpoints -= damage;
    if(cell->hitpoints <= 0) {
        _ignite_cell(pos);
        // TODO: EMIT CATCH FIRE SIGNAL HERE
        // (check if NOT on fire first?)
    }
}

void FireComponent3D::_ignite_cell(Vector3i cell) {
    fire_cell_t data = _grid[cell];
    if(/*[TODO check (static?) spread budget] || */ data.burning) {
        return;
    }

    data.burning = true;
    // data.time_left = burn_time // TODO
    // TODO decrease (static?) spread budget

    // TODO setup particle emitter for cell

    // TODO update burning area
    
}
void FireComponent3D::_extinguish_cell(Vector3i cell) {
    fire_cell_t data = _grid[cell];
    if(!data.burning) {
        return;
    }

    data.burning = false;

    // TODO teardown particle emitter for cell

    // data.hitpoints = max_hitpoints // TODO
    // TODO increase (static?) spread budget
    // data.cooldown = burn_time / 2.0 // TODO


}

void FireComponent3D::_build_grid() {
}

void FireComponent3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_torch", "t"), &FireComponent3D::set_torch);
	ClassDB::bind_method(D_METHOD("get_torch"), &FireComponent3D::get_torch);

    ClassDB::bind_method(D_METHOD("set_resolution", "r"), &FireComponent3D::set_resolution);
	ClassDB::bind_method(D_METHOD("get_resolution"), &FireComponent3D::get_resolution);

    ADD_PROPERTY(
        PropertyInfo(Variant::BOOL, "torch", PROPERTY_HINT_RESOURCE_TYPE, "bool"),
        "set_torch",
        "get_torch"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::VECTOR3I, "resolution"),
        "set_resolution",
        "get_resolution"
    );
}

void FireComponent3D::set_torch(bool t) {
    is_torch = t;
}
bool FireComponent3D::get_torch() const {
    return is_torch;
}

void FireComponent3D::set_resolution(Vector3i r) {
    grid_resolution = r;
}
Vector3i FireComponent3D::get_resolution() const {
    return grid_resolution;
}

void FireComponent3D::_ready() {
    Node *parent = get_parent();
	if (!parent) {
		return;
    }

    for(Object * child : parent->get_children()) {
		if (CollisionShape3D *col = Object::cast_to<CollisionShape3D>(child)) {
			_col_inst = col;
			break;
		}
	}

    if(!_col_inst) {
        UtilityFunctions::print("[Enflame] No CollisionShape3D sibling found");
		return;
    }

    Ref<Shape3D> shape = _col_inst->get_shape();
    if(shape.is_null()) {
        UtilityFunctions::print("[Enflame] CollisionShape3D has no shape");
		return;
    }

    _collision_aabb = _get_shape_aabb(_col_inst, &_is_convex);
    _cell_size = _collision_aabb.get_size() / grid_resolution;
    if(_is_convex) {
        _build_convex_planes(_col_inst, _convex_planes);
    }

    UtilityFunctions::prints("[Enflame]", parent->get_name(), "_is_convex=", _is_convex);
    UtilityFunctions::prints("[Enflame]", parent->get_name(), "_collision_aabb=", _collision_aabb);
    UtilityFunctions::prints("[Enflame]", parent->get_name(), "_cell_size=", _cell_size);

}

void FireComponent3D::_process(double p_delta) {

}
