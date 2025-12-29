#include "fire.hpp"
#include "godot_cpp/classes/area3d.hpp"
#include "godot_cpp/classes/array_mesh.hpp"
// #include "godot_cpp/classes/base_material3d.hpp"
#include "godot_cpp/classes/box_shape3d.hpp"
#include "godot_cpp/classes/engine.hpp"
// #include "godot_cpp/classes/fast_noise_lite.hpp"
#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/packed_scene.hpp"
#include "godot_cpp/classes/physics_body3d.hpp"
// #include "godot_cpp/classes/random_number_generator.hpp"
#include "godot_cpp/classes/rendering_server.hpp"
#include "godot_cpp/classes/resource_loader.hpp"
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
#include "godot_cpp/core/property_info.hpp"
#include "godot_cpp/variant/aabb.hpp"
#include "godot_cpp/variant/array.hpp"
#include "godot_cpp/variant/color.hpp"
#include "godot_cpp/variant/dictionary.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"
#include "godot_cpp/variant/plane.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/variant.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace godot;

int FireComponent3D::s_spread_budget = DEFAULT_SPREAD_BUDGET;
int FireComponent3D::s_max_spread = DEFAULT_SPREAD_BUDGET;
int FireComponent3D::s_fire_collision_layer = DEFAULT_COL_LAYER;
int FireComponent3D::s_flammable_collision_layer = DEFAULT_MASK_LAYER;

const Color DBG_COLD_COLOR = Color(0.5f, 0.5f, 0.5f);
const Color DBG_BURN_COLOR = Color(0.1f, 0.0f, 0.0f);

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

    // nothing burning
    if(_burn_count == 0) {
        _burn_area->set_monitoring(false);
        return;
    }

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
    if(cell->hitpoints <= 0.0f) {
        _ignite_cell(pos);
    }
}

void FireComponent3D::_ignite_cell(Vector3i cell) {
    fire_cell_t & data = _grid[cell];
    if((s_spread_budget < 1) || data.burning) {
        return;
    }

    data.burning = true;
    data.time_left = BURN_TIME_S;
    s_spread_budget = MAX(s_spread_budget-1, 0);

    _burn_count++;

    if(_burn_count == 1) {
        emit_signal("ignite", to_global(data.local_pos));
    }

    // --- particle emitter ---
    if (data.emitter == nullptr) {
        Ref<PackedScene> scene = _get_emitter_scene();
        if (scene.is_valid()) {
            // Node *n = scene->instantiate();
            Node3D *em = Object::cast_to<Node3D>(scene->instantiate());
            if (em) {
                em->set_transform(
                    Transform3D(Basis(), data.local_pos)
                );
                add_child(em);
                data.emitter = em;
            } else {
                // n->queue_free();
            }
        }
    }

    _update_burn_area();
    
}
void FireComponent3D::_extinguish_cell(Vector3i cell) {
    fire_cell_t & data = _grid[cell];
    if(!data.burning) {
        return;
    }

    // --- particle emitter ---
    if (data.emitter) {
        data.emitter->queue_free();
        data.emitter = nullptr;
    }
    
    data.burning = false;
    data.hitpoints = max_hitpoints;
    s_spread_budget = MIN(s_spread_budget+1, s_max_spread);
    data.cooldown = BURN_TIME_S * 0.5f;

    _burn_count--;
    if(_burn_count == 0) {
        emit_signal("extinguish");
    }

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
            cell.dbg_mesh_rid = RID();
        }
        cell.dbg_mesh_ref.unref();
        cell.dbg_mat_ref.unref();
        // if(!is_inside_tree()) {
        //     ++it;
        //     continue;
        // }
        if(cell.emitter != nullptr) {
            cell.emitter->queue_free();
            cell.emitter = nullptr;
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

                _grid[coord] = {
                    .hitpoints = (is_torch) ? -1 : (float)max_hitpoints,
                    .burning = false,
                    .cooldown = 0.0f,
                    .time_left = BURN_TIME_S,
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
                cell.dbg_mesh_rid = RID();
            }
            it = _grid.erase(it);
        } else {
            ++it;
        }
    }
}

void FireComponent3D::_intra_spread(Vector3i pos, fire_cell_t & data, float dt) {
    // spread to neighboring cells
    for(Vector3i offset : SURROUNDING) {
        // key not exist, skip
        if (_grid.find(pos+offset) == _grid.end()) {
            continue;
        }

        fire_cell_t & neighb = _grid[pos+offset];
        if(!neighb.burning && neighb.cooldown <= 0.0f) {
            // UtilityFunctions::prints(
            //     "[NEIGH]",
            //     "src=", pos,
            //     "dst=", pos + offset,
            //     "burning=", neighb.burning,
            //     "cd=", neighb.cooldown,
            //     "hp=", neighb.hitpoints
            // );

            neighb.hitpoints -= spread_damage * dt;
            if(neighb.hitpoints <= 0.0f) {
                _ignite_cell(pos+offset);
            }
        }
    }
}

void FireComponent3D::_check_inter_spread() {
    if(!is_on_fire()) {
        return;
    }

    Vector3 pos = _burn_area->get_global_transform().origin;
    int dmg = (int)(spread_damage * _burn_count * 0.5f);


    emit_signal("damage", dmg);
    // UtilityFunctions::prints("[Enflame] DMG!", _parent->get_name(), dmg);

    for(auto &n : _burn_area->get_overlapping_bodies()) {
        Node3D * body = Object::cast_to<Node3D>(n);
        if(body ==nullptr || !body->is_in_group("flammable")) {
            continue;
        }
        if(!body->has_user_signal("spread_fire")) {
            UtilityFunctions::print("[Enflame] ERROR no spread signal", get_name(), "->", body->get_name());
            continue;
        }
        body->emit_signal("spread_fire", pos, dmg);
    }
}



void FireComponent3D::_bind_methods() {
    ADD_SIGNAL(MethodInfo("ignite", PropertyInfo(Variant::VECTOR3, "world_pos")));
    ADD_SIGNAL(MethodInfo("damage", PropertyInfo(Variant::INT, "amount")));
    ADD_SIGNAL(MethodInfo("extinguish"));

    ClassDB::bind_method(D_METHOD("set_torch", "t"), &FireComponent3D::set_torch);
	ClassDB::bind_method(D_METHOD("get_torch"), &FireComponent3D::get_torch);

    ClassDB::bind_method(D_METHOD("set_disabled", "d"), &FireComponent3D::set_disabled);
	ClassDB::bind_method(D_METHOD("get_disabled"), &FireComponent3D::get_disabled);

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

	ClassDB::bind_method(D_METHOD("is_on_fire"), &FireComponent3D::is_on_fire);
	
    ClassDB::bind_method(D_METHOD("apply_fire"), &FireComponent3D::apply_fire);

    // ClassDB::bind_method(D_METHOD("set_default_emitter", "scene"), &FireComponent3D::set_default_emitter);
    // ClassDB::bind_method(D_METHOD("get_default_emitter"), &FireComponent3D::get_default_emitter);

    ClassDB::bind_method(D_METHOD("set_override_emitter", "scene"), &FireComponent3D::set_override_emitter);
    ClassDB::bind_method(D_METHOD("get_override_emitter"), &FireComponent3D::get_override_emitter);

    // ADD_PROPERTY(
    //     PropertyInfo(Variant::OBJECT, "static/default_emitter", PROPERTY_HINT_RESOURCE_TYPE, "PackedScene"),
    //     "set_default_emitter",
    //     "get_default_emitter"
    // );

    ADD_PROPERTY(
        PropertyInfo(Variant::OBJECT, "override_emitter", PROPERTY_HINT_RESOURCE_TYPE, "PackedScene"),
        "set_override_emitter",
        "get_override_emitter"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "static/s_spread_budget", PROPERTY_HINT_RANGE, "0,1000,1"),
        "set_spread_budget",
        "get_spread_budget"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::BOOL, "disabled", PROPERTY_HINT_RESOURCE_TYPE, "bool"),
        "set_disabled",
        "get_disabled"
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
        PropertyInfo(Variant::INT, "static/s_fire_collision_layer", PROPERTY_HINT_RANGE, "0,31,1"),
        "set_fire_collision_layer",
        "get_fire_collision_layer"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "static/s_flammable_collision_layer", PROPERTY_HINT_RANGE, "0,31,1"),
        "set_flammable_collision_layer",
        "get_flammable_collision_layer"
    );
}

Ref<PackedScene> FireComponent3D::_get_emitter_scene() {
    if (_override_emitter_scene.is_valid()) {
        return _override_emitter_scene;
    }
    if(!_emitter_scene.is_valid()) {
        _emitter_scene = ResourceLoader::get_singleton()->load(
            DEFAULT_FIRE_EMITTER_PATH);
    }
    return _emitter_scene;
}

void FireComponent3D::set_override_emitter(const Ref<PackedScene> &scene) {
    _override_emitter_scene = scene;
}

Ref<PackedScene> FireComponent3D::get_override_emitter() const {
    return _override_emitter_scene;
}


void FireComponent3D::set_torch(bool t) {
    if(t) {
        _last_max_hp = max_hitpoints;
        max_hitpoints = -1;
    } else {
        max_hitpoints = _last_max_hp;
    }
    is_torch = t;
    if(Engine::get_singleton()->is_editor_hint() || !is_torch) {
        return;
    }

    for(auto & it : _grid) {
        _ignite_cell(it.first);
    }
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
    for (auto & it : _grid) {
        fire_cell_t & cell = it.second;
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

void FireComponent3D::set_disabled(bool d) {
    disabled = d;

}
bool FireComponent3D::get_disabled() const {
    return disabled;
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
    if(Engine::get_singleton()->is_editor_hint()) {
        return s_max_spread;
    }
    return s_spread_budget;
}

void FireComponent3D::set_spread_budget(int v) {
    s_max_spread = MAX(v, 0);
}

void FireComponent3D::set_fire_collision_layer(int l) {
    if(s_fire_collision_layer == l) {
        return;
    }
    l = MAX(0, l);
    l = MIN(31, l);
    // unset old collision layer
    if(!is_inside_tree()) {
        s_fire_collision_layer = l;
        return;
    }
    SceneTree * tree = get_tree();
    for(auto & n : tree->get_nodes_in_group("flammable")) {
        PhysicsBody3D * body = Object::cast_to<PhysicsBody3D>(n);
        if(body == nullptr) {
            continue;
        }
        body->set_collision_layer_value(s_fire_collision_layer, false);
        body->set_collision_layer_value(l, true);
    }
    s_fire_collision_layer = l;
}
int FireComponent3D::get_fire_collision_layer() const {
    return s_fire_collision_layer;
}

void FireComponent3D::set_flammable_collision_layer(int l) {
    if(s_flammable_collision_layer == l) {
        return;
    }
    l = MAX(0, l);
    l = MIN(31, l);
    // unset old collision layer
    if(!is_inside_tree()) {
        s_flammable_collision_layer = l;
        return;
    }
    SceneTree * tree = get_tree();
    for(auto & n : tree->get_nodes_in_group("flammable")) {
        PhysicsBody3D * body = Object::cast_to<PhysicsBody3D>(n);
        if(body == nullptr) {
            continue;
        }
        body->set_collision_mask_value(s_flammable_collision_layer, false);
        body->set_collision_mask_value(l, true);
    }
    s_flammable_collision_layer = l;
}
int FireComponent3D::get_flammable_collision_layer() const {
    return s_flammable_collision_layer;
}

bool FireComponent3D::is_on_fire() const {
    return _burn_count > 0;
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
    _burn_area_col_inst->set_debug_color(Color(1.0f,0.0f,0.0f));
    _burn_area->add_child(_burn_area_col_inst);
    _burn_area->set_monitoring(false);
    _burn_area->set_monitorable(false);
    add_child(_burn_area);
    _burn_area->set_global_transform(get_global_transform());

    // add user signal to parent for inter-object spread
    Array usrargs;
    Dictionary dpos;
    dpos["name"] = "world_pos";
    dpos["type"] = Variant::VECTOR3;
    Dictionary ddam;
    ddam["name"] = "damage";
    ddam["type"] = Variant::INT;
    usrargs.push_back(dpos);
    usrargs.push_back(ddam);
    _parent->add_user_signal("spread_fire", usrargs);
    _parent->connect("spread_fire", Callable(this, "apply_fire"));

    // verbose print
    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_is_convex=", _is_convex);
    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_local_aabb=", _local_aabb);
    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_cell_size=", _cell_size);
    UtilityFunctions::prints("[Enflame]", _parent->get_name(), "_grid.size()=", _grid.size());

}

void FireComponent3D::_on_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint() || disabled) {
        return;
    }

    const float dt = static_cast<float>(delta);

    bool any_burning = false;

    // -------------------------
    // PHASE 1: update cell state
    // -------------------------
    for (auto &kv : _grid) {
        Vector3i pos = kv.first;
        fire_cell_t &cell = kv.second;

        if (cell.burning) {
            any_burning = true;

            if(!_burn_area->is_monitoring()) {
                _burn_area->set_monitoring(true);
                _burn_area->set_monitorable(true);
            }

            cell.time_left -= dt;

            if (cell.time_left <= 0.0f) {
                if (is_torch) {
                    cell.time_left = BURN_TIME_S;
                } else {
                    _extinguish_cell(pos);
                }
            }
        } else {
            if (cell.cooldown > 0.0f) {
                cell.cooldown = MAX(0.0f, cell.cooldown - dt);
            } else {
                if (cell.hitpoints < max_hitpoints) {
                    cell.hitpoints = MIN(
                        (float)max_hitpoints,
                        cell.hitpoints + (spread_damage * 0.25f) * dt
                    );
                }
            }
        }
    }

    // -------------------------
    // PHASE 2: debug visuals
    // -------------------------
    _dbg_visuals_timer -= dt;
    const bool update_dbg = _dbg_visuals_timer <= 0.0f;
    if (update_dbg) {
        _dbg_visuals_timer = DBG_VISUAL_UPDATE_S;

        if (visible_debug) {
            for (auto &kv : _grid) {
                fire_cell_t &cell = kv.second;

                if (cell.burning) {
                    cell.dbg_mat_ref->set_albedo(DBG_BURN_COLOR);
                } else if (cell.hitpoints < max_hitpoints) {
                    float intensity = 1.0f - cell.hitpoints / max_hitpoints;
                    cell.dbg_mat_ref->set_albedo(Color(1.0f, 0.5f * intensity, 0.0f));
                } else {
                    cell.dbg_mat_ref->set_albedo(DBG_COLD_COLOR);
                }
            }
        }
    }

    // -------------------------
    // PHASE 3: intra-object spread
    // -------------------------
    if (any_burning) {
        for (auto &kv : _grid) {
            if (!kv.second.burning) {
                continue;
            }
            _intra_spread(kv.first, kv.second, dt);
        }
    }

    // -------------------------
    // PHASE 4: inter-object spread (timer-gated)
    // -------------------------
    if (any_burning) {
        _spread_timer -= dt;
        if (_spread_timer <= 0.0f) {
            _spread_timer = spread_interval;
            _check_inter_spread();
        }
    }
}


void FireComponent3D::_on_xform_changed() {
    if(!is_inside_tree()) {
        return;
    }
    Transform3D glob_xform = get_global_transform();
    for (auto & it : _grid) {
        fire_cell_t & cell = it.second;
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
        
        case NOTIFICATION_EXIT_WORLD: {
            break;
        }

        case NOTIFICATION_PREDELETE: {
            set_process(false);
            set_notify_transform(false);

            // hard-disable first
            disabled = true;

            // disconnect safely
            if (_parent && _parent->is_connected("spread_fire", Callable(this, "apply_fire"))) {
                _parent->disconnect("spread_fire", Callable(this, "apply_fire"));
            }

            _cleanup();
            break;
        }


        case NOTIFICATION_TRANSFORM_CHANGED: {
            _on_xform_changed();
            break;
        }

        case NOTIFICATION_PROCESS: {
            _on_process(get_process_delta_time());
            break;
        }

        default:
            break;
    }
}
