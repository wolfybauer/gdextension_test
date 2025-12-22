#include "fire.hpp"
#include "godot_cpp/classes/area3d.hpp"
#include "godot_cpp/classes/array_mesh.hpp"
// #include "godot_cpp/classes/base_material3d.hpp"
#include "godot_cpp/classes/box_shape3d.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/physics_body3d.hpp"
// #include "godot_cpp/classes/random_number_generator.hpp"
#include "godot_cpp/classes/rendering_server.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/sphere_shape3d.hpp"
#include "godot_cpp/classes/capsule_shape3d.hpp"
#include "godot_cpp/classes/collision_shape3d.hpp"
#include "godot_cpp/classes/convex_polygon_shape3d.hpp"
#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/shape3d.hpp"
#include "godot_cpp/classes/world3d.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/aabb.hpp"
#include "godot_cpp/variant/color.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"
#include "godot_cpp/variant/plane.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace godot;

int FireComponent3D::spread_budget = DEFAULT_SPREAD_BUDGET;
int FireComponent3D::max_spread = DEFAULT_SPREAD_BUDGET;
int FireComponent3D::fire_collision_layer = DEFAULT_COL_LAYER;
int FireComponent3D::flammable_collision_layer = DEFAULT_MASK_LAYER;

const Color DBG_COLD_COLOR = Color(0.5f, 0.5f, 0.5f);
const Color DBG_BURN_COLOR = Color(1.0f, 1.0f, 1.0f);

const Vector3i SURROUNDING[6] = {
    Vector3i(1, 0, 0),
    Vector3i(-1, 0, 0),

    Vector3i(0, 1, 0),
    Vector3i(0, -1, 0),

    Vector3i(0, 0, 1),
    Vector3i(0, 0, -1),
};

// optimize for case when convex polygon is actually just a box
static bool _convex_is_box(const Ref<ConvexPolygonShape3D> &cvx, Vector3 &out_size) {
    PackedVector3Array pts = cvx->get_points();
    if (pts.size() != 8) {
        return false;
    }

    float xs[8], ys[8], zs[8];
    for (int i = 0; i < 8; i++) {
        xs[i] = pts[i].x;
        ys[i] = pts[i].y;
        zs[i] = pts[i].z;
    }

    auto unique_count = [](float *v) {
        std::sort(v, v + 8);
        int c = 1;
        for (int i = 1; i < 8; i++) {
            if (!Math::is_equal_approx(v[i], v[i - 1])) {
                c++;
            }
        }
        return c;
    };

    if (unique_count(xs) != 2 ||
        unique_count(ys) != 2 ||
        unique_count(zs) != 2) {
        return false;
    }

    float minx = xs[0], maxx = xs[7];
    float miny = ys[0], maxy = ys[7];
    float minz = zs[0], maxz = zs[7];

    out_size = Vector3(
        maxx - minx,
        maxy - miny,
        maxz - minz
    );

    return true;
}



static AABB _get_shape_aabb(CollisionShape3D * collision_shape, bool * is_convex) {
    *is_convex = false;
    Ref<Shape3D> shape = collision_shape->get_shape();
    AABB aabb;

    // Transform3D glob_xform = collision_shape->get_global_transform();

    // optimize for boxshape
    Ref<BoxShape3D> box = shape;
    if(box.is_valid()) {
        aabb = AABB(-box->get_size() * 0.5f, box->get_size());
        // aabb.position += glob_xform.origin; // convert to world space
        return aabb;
    }
    
    // optimize for sphere
    Ref<SphereShape3D> sphere = shape;
    if(sphere.is_valid()) {
        float r = sphere->get_radius();
        aabb = AABB(Vector3(-r,-r,-r), Vector3(2*r,2*r,2*r));
        // aabb.position += glob_xform.origin; // convert to world space
        return aabb;
    }
    
    // optimize for capsule
    Ref<CapsuleShape3D> caps = shape;
    if(caps.is_valid()) {
        float r = caps->get_radius();
        float h = caps->get_height();
        aabb = AABB(Vector3(-r, -h * 0.5f, -r), Vector3(r * 2.0f, h, r * 2.0f));
        // aabb.position += glob_xform.origin; // convert to world space
        return aabb;
    }
    
    // convex polygon
    Ref<ConvexPolygonShape3D> cvx = shape;
    if (cvx.is_valid()) {
        Vector3 box_size;
        if (_convex_is_box(cvx, box_size)) {
            // treat as BoxShape3D
            *is_convex = false;

            aabb = AABB(-box_size * 0.5f, box_size);
        } else {
            *is_convex = true;
            // real convex
            aabb = cvx->get_debug_mesh()->get_aabb();
        }

        // aabb.position += glob_xform.origin;
        return aabb;
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

                Plane plane = Plane(a, b, c);

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

bool FireComponent3D::_is_inside_object(Vector3 local_pos) {
    // local AABB test
    if (!_local_aabb.has_point(local_pos)) {
        return false;
    }

    if (!_is_convex) {
        return true;
    }

    // local convex plane test
    for (const Plane &plane : _convex_planes) {
        if (plane.distance_to(local_pos) > 0.001f) {
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

void FireComponent3D::_update_burn_area() {

    // get combined aabb of all burn points
    AABB burn_aabb = AABB();
    for(auto &kv : _grid) {
        fire_cell_t &data = kv.second;
        if(!data.burning) {
            continue;
        }

        Vector3 world_pos = to_global(data.local_pos);
        AABB new_box = AABB(world_pos, _cell_size);
        burn_aabb = burn_aabb.merge(new_box);
    }

    // nothing burning
    if(burn_aabb.size.is_zero_approx()) {
        _burn_area->set_monitoring(false);
        return;
    }

    burn_aabb.position -= (Vector3(1,1,1) * spread_margin);
    burn_aabb.size += (Vector3(1,1,1) * spread_margin * 2.0f);

    _burn_box_ref->set_size(burn_aabb.size);
    Transform3D t = _burn_area->get_global_transform();
    t.origin = burn_aabb.position + burn_aabb.size * 0.5f;
    _burn_area->set_global_transform(t);

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
    fire_cell_t & data = _grid[cell];
    if(!spread_budget || data.burning) {
        return;
    }

    data.burning = true;
    data.time_left = BURN_TIME_MS;
    spread_budget = MAX(spread_budget-1, 0);

    // TODO setup particle emitter for cell

    _update_burn_area();
    
}
void FireComponent3D::_extinguish_cell(Vector3i cell) {
    fire_cell_t & data = _grid[cell];
    if(!data.burning) {
        return;
    }

    data.burning = false;

    // TODO teardown particle emitter for cell

    data.hitpoints = max_hitpoints;
    spread_budget = MIN(spread_budget+1, max_spread);
    data.cooldown = BURN_TIME_MS / 2;

    // TODO update burning area
    _update_burn_area();

}

static bool _is_on_boundary(std::unordered_map<Vector3i, fire_cell_t, Vector3iHash, Vector3iEq> &grid, Vector3i pos) {
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
        fire_cell_t & cell = it->second;
        if(cell.dbg_mesh_rid.is_valid()) {
            RenderingServer::get_singleton()->free_rid(cell.dbg_mesh_rid);
        }
        it = _grid.erase(it);
    }
}

void FireComponent3D::_cleanup() {
    _clear_grid();
    if(_burn_area != nullptr) {
        _burn_area->queue_free();
    }
}


void FireComponent3D::_build_grid() {
    if(!is_inside_tree()) {
        return;
    }

    // reset cell size every rebuild
    _cell_size = _local_aabb.get_size() / grid_resolution;

    // preclean grid
    _clear_grid();

    // make debug sphere
    if(_dbg_sphere_ref.is_null()) {
        _dbg_sphere_ref.instantiate();
    }

    // determine size every (re)build of grid
    _dbg_sphere_ref->set_radius(_cell_size.length() * 0.2f);
    _dbg_sphere_ref->set_height(_dbg_sphere_ref->get_radius()*2.0f);

    Transform3D glob_xform = get_global_transform();
    RenderingServer * rs = RenderingServer::get_singleton();

    // built it
    for(int x=0; x<grid_resolution.x; x++) {
        for(int y=0; y<grid_resolution.y; y++) {
            for(int z=0; z<grid_resolution.z; z++) {
                Vector3i coord = Vector3i(x,y,z);
                Vector3 loc = _local_aabb.position + Vector3(x,y,z) * _cell_size + _cell_size * 0.5f;
                if(!_is_inside_object(loc)) {
                    continue;
                }

                // TODO setup debug visuals

                _grid[coord] = {
                    .hitpoints = (is_torch) ? -1 : max_hitpoints,
                    .burning = false,
                    .cooldown = 0,
                    .time_left = 0,
                    // .local_pos = to_local(world_pos),
                    .local_pos = loc,
                    .emitter = nullptr
                };
                if(_grid[coord].dbg_mat_ref.is_null()) {
                    _grid[coord].dbg_mat_ref.instantiate();
                }
                
                _grid[coord].dbg_mat_ref->set_albedo(DBG_COLD_COLOR);

                if(_grid[coord].dbg_mesh_ref.is_null()) {
                    _grid[coord].dbg_mesh_ref = _dbg_sphere_ref->duplicate();
                    _grid[coord].dbg_mesh_ref->set_material(_grid[coord].dbg_mat_ref);
                }

                if(!_grid[coord].dbg_mesh_rid.is_valid()) {
                    _grid[coord].dbg_mesh_rid = rs->instance_create2(_grid[coord].dbg_mesh_ref->get_rid(), get_world_3d()->get_scenario());
                    rs->instance_geometry_set_cast_shadows_setting(_grid[coord].dbg_mesh_rid, RenderingServer::SHADOW_CASTING_SETTING_OFF);
                    rs->instance_set_transform(
                        _grid[coord].dbg_mesh_rid,
                        glob_xform * Transform3D(Basis(), _grid[coord].local_pos)
                    );
                    rs->instance_set_visible(_grid[coord].dbg_mesh_rid, visible_debug);

                }
            }
        }
    }

    // prune interior cells
    for (auto it = _grid.begin(); it != _grid.end(); ) {
        if (!_is_on_boundary(_grid, it->first)) {
            fire_cell_t & cell = it->second;
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

    for (auto it : _grid ) {
        Vector3i pos = it.first;
        fire_cell_t & data = it.second;

        if(!data.burning) {
            continue;
        }

        // spread to neighboring cells
        for(Vector3i offset : SURROUNDING) {
            // key not exist, skip
            if (_grid.find(pos+offset) == _grid.end()) {
                continue;
            }

            fire_cell_t & neighb = _grid[pos+offset];
            if(!neighb.burning && neighb.cooldown <= 0) {

            }
        }
    }
}


void FireComponent3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_torch", "t"), &FireComponent3D::set_torch);
	ClassDB::bind_method(D_METHOD("get_torch"), &FireComponent3D::get_torch);

    ClassDB::bind_method(D_METHOD("set_visible_debug", "v"), &FireComponent3D::set_visible_debug);
	ClassDB::bind_method(D_METHOD("get_visible_debug"), &FireComponent3D::get_visible_debug);

    ClassDB::bind_method(D_METHOD("set_resolution", "r"), &FireComponent3D::set_resolution);
	ClassDB::bind_method(D_METHOD("get_resolution"), &FireComponent3D::get_resolution);

    ClassDB::bind_method(D_METHOD("set_max_hp", "h"), &FireComponent3D::set_max_hp);
	ClassDB::bind_method(D_METHOD("get_max_hp"), &FireComponent3D::get_max_hp);

    ClassDB::bind_method(D_METHOD("get_spread_budget"), &FireComponent3D::get_spread_budget);
    ClassDB::bind_method(D_METHOD("set_spread_budget", "v"), &FireComponent3D::set_spread_budget);

    ClassDB::bind_method(D_METHOD("set_fire_collision_layer", "l"), &FireComponent3D::set_fire_collision_layer);
	ClassDB::bind_method(D_METHOD("get_fire_collision_layer"), &FireComponent3D::get_fire_collision_layer);

    ClassDB::bind_method(D_METHOD("set_flammable_collision_layer", "l"), &FireComponent3D::set_flammable_collision_layer);
	ClassDB::bind_method(D_METHOD("get_flammable_collision_layer"), &FireComponent3D::get_flammable_collision_layer);

    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "spread_budget", PROPERTY_HINT_RANGE, "0,1000,1"),
        "set_spread_budget",
        "get_spread_budget"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::BOOL, "torch", PROPERTY_HINT_RESOURCE_TYPE, "bool"),
        "set_torch",
        "get_torch"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::BOOL, "visible_debug", PROPERTY_HINT_RESOURCE_TYPE, "bool"),
        "set_visible_debug",
        "get_visible_debug"
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

    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "fire_collision_layer", PROPERTY_HINT_RANGE, "0,31,1"),
        "set_fire_collision_layer",
        "get_fire_collision_layer"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "flammable_collision_layer", PROPERTY_HINT_RANGE, "0,31,1"),
        "set_flammable_collision_layer",
        "get_flammable_collision_layer"
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

void FireComponent3D::set_visible_debug(bool v) {
    if(visible_debug == v) {
        return;
    }
    visible_debug = v;

    if(!is_inside_tree()) {
        return;
    }
    
    RenderingServer * rs = RenderingServer::get_singleton();
    for (auto it = _grid.begin(); it != _grid.end(); it++) {
        fire_cell_t & cell = it->second;
        if(!cell.dbg_mesh_rid.is_valid()) {
            continue;
        }
        rs->instance_set_visible(cell.dbg_mesh_rid, visible_debug);
    }

    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "visible_debug=", visible_debug);

}
bool FireComponent3D::get_visible_debug() const {
    return visible_debug;
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

int FireComponent3D::get_spread_budget() const {
    return spread_budget;
}

void FireComponent3D::set_spread_budget(int v) {
    max_spread = MAX(v, 0);
}

void FireComponent3D::set_fire_collision_layer(int l) {
    if(fire_collision_layer == l) {
        return;
    }
    l = MAX(0, l);
    l = MIN(31, l);
    // unset old collision layer
    if(!is_inside_tree()) {
        fire_collision_layer = l;
        return;
    }
    SceneTree * tree = get_tree();
    for(auto n : tree->get_nodes_in_group("flammable")) {
        PhysicsBody3D * body = Object::cast_to<PhysicsBody3D>(n);
        if(body == nullptr) {
            continue;
        }
        body->set_collision_layer_value(fire_collision_layer, false);
        body->set_collision_layer_value(l, true);
    }
    fire_collision_layer = l;
}
int FireComponent3D::get_fire_collision_layer() const {
    return fire_collision_layer;
}

void FireComponent3D::set_flammable_collision_layer(int l) {
    if(flammable_collision_layer == l) {
        return;
    }
    l = MAX(0, l);
    l = MIN(31, l);
    // unset old collision layer
    if(!is_inside_tree()) {
        flammable_collision_layer = l;
        return;
    }
    SceneTree * tree = get_tree();
    for(auto n : tree->get_nodes_in_group("flammable")) {
        PhysicsBody3D * body = Object::cast_to<PhysicsBody3D>(n);
        if(body == nullptr) {
            continue;
        }
        body->set_collision_mask_value(flammable_collision_layer, false);
        body->set_collision_mask_value(l, true);
    }
    flammable_collision_layer = l;
}
int FireComponent3D::get_flammable_collision_layer() const {
    return flammable_collision_layer;
}

void FireComponent3D::_on_ready() {
    _parent = get_parent_node_3d();
	if (!_parent) {
		return;
    }

    // find the CollisionShape3D
    for(Object * child : _parent->get_children()) {
		if (CollisionShape3D *col = Object::cast_to<CollisionShape3D>(child)) {
			_base_col_inst = col;
			break;
		}
	}
    if(!_base_col_inst) {
        UtilityFunctions::print("[Enflame] No CollisionShape3D sibling found");
		return;
    }

    // make sure it has a shape
    Ref<Shape3D> shape = _base_col_inst->get_shape();
    if(!shape.is_valid()) {
        UtilityFunctions::print("[Enflame] CollisionShape3D has no shape");
		return;
    }

    // get bunrable object's aabb (and determine if convex), get cell size(radius in physical space)
    _local_aabb = _get_shape_aabb(_base_col_inst, &_is_convex);
    
    // get planes for is_inside checking
    if(_is_convex) {
        _build_convex_planes(_base_col_inst, _convex_planes);
    }

    _build_grid();
    _parent->add_to_group("flammable");
    set_notify_transform(true);
    set_process(true);

    // the area3d that monitors for other burnable things
    _burn_area = memnew(Area3D);
    _burn_area_col_inst = memnew(CollisionShape3D);
    _burn_box_ref.instantiate();
    _burn_area_col_inst->set_shape(_burn_box_ref);
    _burn_area->add_child(_burn_area_col_inst);
    _burn_area->set_monitoring(false);
    _burn_area->set_monitorable(false);
    add_child(_burn_area);
    _burn_area->set_global_transform(_parent->get_global_transform());

    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_is_convex=", _is_convex);
    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_local_aabb=", _local_aabb);
    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_cell_size=", _cell_size);
    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_grid.size()=", _grid.size());

}

void FireComponent3D::_process(double p_delta) {

}

void FireComponent3D::_on_process(double delta) {
    if(Engine::get_singleton()->is_editor_hint()) {
		return;
	}
    
    // check if should update
    _dbg_visuals_timer -= delta * 1000.0;
    bool update_dbg = _dbg_visuals_timer < 0.0;
    if(update_dbg) {
        _dbg_visuals_timer = DBG_VISUAL_UPDATE_MS;
    }

    for (auto it = _grid.begin(); it != _grid.end(); it++) {
        fire_cell_t & cell = it->second;

        // update debug visuals
        if(visible_debug && update_dbg) {
            if(cell.burning) {
                cell.dbg_mat_ref->set_albedo(DBG_BURN_COLOR);
            } else if(cell.hitpoints < max_hitpoints) {
                // float intensity = 1.0f - cell.hitpoints / max_hitpoints;
                float intensity = (float)cell.hitpoints / max_hitpoints;
                cell.dbg_mat_ref->set_albedo(Color(1.0f, 0.5f * intensity, 0.0f));
            } else {
                cell.dbg_mat_ref->set_albedo(DBG_COLD_COLOR);
            }

            // TEST COLORS
            // apply_fire(Vector3(6.0,0,0), 5);
            // cell.hitpoints -= 5.0f;
            // cell.hitpoints = Math::clamp<float>(cell.hitpoints, 0.0f, (float)max_hitpoints);
        }

        // do other stuff

        
    }
}

void FireComponent3D::_on_xform_changed() {
    if(!visible_debug) {
        return;
    }
    Transform3D glob_xform = get_global_transform();
    for (auto it = _grid.begin(); it != _grid.end(); it++) {
        fire_cell_t & cell = it->second;
        if(!cell.dbg_mesh_rid.is_valid()) {
            continue;
        }
        RenderingServer::get_singleton()->instance_set_transform(
            cell.dbg_mesh_rid,
            glob_xform * Transform3D(Basis(), cell.local_pos)
        );
    }
}

void FireComponent3D::_notification(int p_what) {

    // UtilityFunctions::prints("[Enflame]", "_notification=", p_what, "processing=", is_processing());
    switch (p_what) {
		case NOTIFICATION_READY: {
            _on_ready();
            break;
        }

        case NOTIFICATION_PREDELETE: {
            _clear_grid();
            break;
        }

        case NOTIFICATION_TRANSFORM_CHANGED: {
            _on_xform_changed();
            break;
        }

        case NOTIFICATION_PROCESS: {
            // UtilityFunctions::prints("[Enflame]", "_notification=", "NOTIFICATION_PROCESS", "processing=", is_processing());
            _on_process(get_process_delta_time());
            break;
        }

        default:
            break;
    }
}
