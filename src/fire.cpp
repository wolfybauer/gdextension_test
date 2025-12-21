#include "fire.hpp"
#include "godot_cpp/classes/array_mesh.hpp"
#include "godot_cpp/classes/base_material3d.hpp"
#include "godot_cpp/classes/box_shape3d.hpp"
#include "godot_cpp/classes/rendering_server.hpp"
#include "godot_cpp/classes/sphere_shape3d.hpp"
#include "godot_cpp/classes/capsule_shape3d.hpp"
#include "godot_cpp/classes/collision_shape3d.hpp"
#include "godot_cpp/classes/convex_polygon_shape3d.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/shape3d.hpp"
#include "godot_cpp/classes/world3d.hpp"
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

static bool _is_on_boundary(std::unordered_map<Vector3i, fire_cell_t, Vector3iHash, Vector3iEq> grid, Vector3i pos) {
    for(Vector3 offset : SURROUNDING) {
        // key not exist, so IS on boundary
        if (grid.find(pos+offset) == grid.end()) {
            return true;
        }
    }
    return false;
}

void FireComponent3D::_clear_grid() {
    for (auto it = _grid.begin(); it != _grid.end(); ) {
        fire_cell_t cell = it->second;
        if(cell.dbg_mesh_rid.is_valid()) {
            RenderingServer::get_singleton()->free_rid(cell.dbg_mesh_rid);
        }
        it = _grid.erase(it);
    }
}


void FireComponent3D::_build_grid() {
    // preclean grid
    _clear_grid();

    // make debug sphere
    if(_dbg_sphere.is_null()) {
        _dbg_sphere.instantiate();
    }

    // determine size every (re)build of grid
    _dbg_sphere->set_radius(_cell_size.length() * 0.2f);
    _dbg_sphere->set_height(_dbg_sphere->get_radius()*2.0f);

    // built it
    for(int x=0; x<grid_resolution.x; x++) {
        for(int y=0; y<grid_resolution.y; y++) {
            for(int z=0; z<grid_resolution.z; z++) {
                Vector3i coord = Vector3i(x,y,z);
                Vector3 world_pos = _collision_aabb.position + Vector3(x,y,z) * _cell_size + _cell_size / 2.0f;
                if(!_is_inside_object(world_pos)) {
                    continue;
                }

                // TODO setup debug visuals

                _grid[coord] = {
                    .hitpoints = (is_torch) ? -1.0f : (float)max_hitpoints,
                    .burning = false,
                    .cooldown = 0.0,
                    .time_left = 0.0,
                    .local_pos = to_local(world_pos),
                    .emitter = nullptr
                };
                if(_grid[coord].dbg_mat_ref.is_null()) {
                    _grid[coord].dbg_mat_ref.instantiate();
                }

                if(_grid[coord].dbg_mesh_ref.is_null()) {
                    _grid[coord].dbg_mesh_ref = _dbg_sphere->duplicate();
                    _grid[coord].dbg_mesh_ref->set_material(_grid[coord].dbg_mat_ref);
                }

                if(!_grid[coord].dbg_mesh_rid.is_valid()) {
                    _grid[coord].dbg_mesh_rid = RenderingServer::get_singleton()->instance_create2(_grid[coord].dbg_mesh_ref->get_rid(), get_world_3d()->get_scenario());
                    RenderingServer::get_singleton()->instance_geometry_set_cast_shadows_setting(_grid[coord].dbg_mesh_rid, RenderingServer::SHADOW_CASTING_SETTING_OFF);
                    RenderingServer::get_singleton()->instance_set_transform(_grid[coord].dbg_mesh_rid, Transform3D(Basis(), to_global(_grid[coord].local_pos)) * get_global_transform());
                }
            }
        }
    }

    // prune interior cells
    for (auto it = _grid.begin(); it != _grid.end(); ) {
        if (!_is_on_boundary(_grid, it->first)) {
            fire_cell_t cell = it->second;
            if(cell.dbg_mesh_rid.is_valid()) {
                RenderingServer::get_singleton()->free_rid(cell.dbg_mesh_rid);
            }
            it = _grid.erase(it);
        } else {
            ++it;
        }
    }

}

void FireComponent3D::_intra_spread(double delta) {
    for (auto it = _grid.begin(); it != _grid.end(); ) {
        Vector3i pos = it->first;
        fire_cell_t data = it->second;

        if(!data.burning) {
            continue;
        }

        // spread to neighboring cells
        for(Vector3 offset : SURROUNDING) {
            // key not exist, skip
            if (_grid.find(pos+offset) == _grid.end()) {
                continue;
            }

            // jgkjgkgkhgkjgkjghhhgdgfdg
        }
    }
}


void FireComponent3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_torch", "t"), &FireComponent3D::set_torch);
	ClassDB::bind_method(D_METHOD("get_torch"), &FireComponent3D::get_torch);

    ClassDB::bind_method(D_METHOD("set_resolution", "r"), &FireComponent3D::set_resolution);
	ClassDB::bind_method(D_METHOD("get_resolution"), &FireComponent3D::get_resolution);

    ClassDB::bind_method(D_METHOD("set_max_hp", "r"), &FireComponent3D::set_max_hp);
	ClassDB::bind_method(D_METHOD("get_max_hp"), &FireComponent3D::get_max_hp);

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

    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "max_hp"),
        "set_max_hp",
        "get_max_hp"
    );
}

void FireComponent3D::set_torch(bool t) {
    if(t) {
        _last_max_hp = max_hitpoints;
        max_hitpoints = -1;
    } else {
        max_hitpoints = _last_max_hp;
    }
    is_torch = t;
}
bool FireComponent3D::get_torch() const {
    return is_torch;
}

void FireComponent3D::set_max_hp(int h) {
    max_hitpoints = h;
}
int FireComponent3D::get_max_hp() const {
    return max_hitpoints;
}

void FireComponent3D::set_resolution(Vector3i r) {
    grid_resolution = r;
    _build_grid();
}
Vector3i FireComponent3D::get_resolution() const {
    return grid_resolution;
}

void FireComponent3D::_on_ready() {
    Node *parent = get_parent();
	if (!parent) {
		return;
    }

    // find the CollisionShape3D
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

    // make sure it has a shape
    Ref<Shape3D> shape = _col_inst->get_shape();
    if(!shape.is_valid()) {
        UtilityFunctions::print("[Enflame] CollisionShape3D has no shape");
		return;
    }

    // get bunrable object's aabb (and determine if convex), get cell size(radius in physical space)
    _collision_aabb = _get_shape_aabb(_col_inst, &_is_convex);
    _cell_size = _collision_aabb.get_size() / grid_resolution;
    
    // get planes for is_inside checking
    if(_is_convex) {
        _build_convex_planes(_col_inst, _convex_planes);
    }

    _build_grid();

    UtilityFunctions::prints("[Enflame]", parent->get_name(), "_is_convex=", _is_convex);
    UtilityFunctions::prints("[Enflame]", parent->get_name(), "_collision_aabb=", _collision_aabb);
    UtilityFunctions::prints("[Enflame]", parent->get_name(), "_cell_size=", _cell_size);
    UtilityFunctions::prints("[Enflame]", parent->get_name(), "_grid.size()=", _grid.size());

}

void FireComponent3D::_on_xform_changed() {

}

void FireComponent3D::_process(double p_delta) {

}

void FireComponent3D::_notification(int p_what) {
    // UtilityFunctions::prints("[Enflame]", "_notification=", p_what);
    switch (p_what) {
		case NOTIFICATION_READY: {
            _on_ready();
        } break;

        case NOTIFICATION_PREDELETE: {
            _clear_grid();
        }

        case NOTIFICATION_TRANSFORM_CHANGED: {
            _on_xform_changed();
        }

        default:
            break;
    }
}
