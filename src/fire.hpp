#pragma once

#include "godot_cpp/classes/area3d.hpp"
#include "godot_cpp/classes/box_shape3d.hpp"
#include "godot_cpp/classes/collision_shape3d.hpp"
#include "godot_cpp/classes/packed_scene.hpp"
#include "godot_cpp/classes/sphere_mesh.hpp"
// #include "godot_cpp/classes/sphere_shape3d.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/variant/plane.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <unordered_map>
#include <vector>

namespace godot {

#define DEFAULT_SPREAD_BUDGET 20

#define DBG_VISUAL_UPDATE_S 1.0f
#define SPREAD_UPDATE_S 0.5f
#define BURN_TIME_S 4.0f

#define DEFAULT_COL_LAYER   30
#define DEFAULT_MASK_LAYER  31


static inline uint64_t hash_combine(uint64_t h, uint64_t k) {
    k *= 0xff51afd7ed558ccdULL;
    k ^= k >> 33;
    k *= 0xc4ceb9fe1a85ec53ULL;
    k ^= k >> 33;
    return h ^ k;
}

struct Vector3iHash {
    size_t operator()(const Vector3i &v) const noexcept {
        uint64_t h = 0;
        h = hash_combine(h, (uint64_t)v.x);
        h = hash_combine(h, (uint64_t)v.y);
        h = hash_combine(h, (uint64_t)v.z);
        return (size_t)h;
    }
};

struct Vector3iEq {
    bool operator()(const Vector3i &a, const Vector3i &b) const noexcept {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

typedef struct {
    float hitpoints;
    bool burning;
    float cooldown;
    float time_left;
    Vector3 local_pos;
    Node3D * emitter;
    RID dbg_mesh_rid;
    Ref<StandardMaterial3D> dbg_mat_ref;
    Ref<SphereMesh> dbg_mesh_ref;
} fire_cell_t;

class FireComponent3D : public Node3D {
    GDCLASS(FireComponent3D, Node3D);

public:
    FireComponent3D() = default;
    ~FireComponent3D() override = default;
    // ~FireComponent3D() {
    //     _grid.clear();
    // }

    void _notification(int p_what);

    void set_torch(bool t);
    bool get_torch() const;

    void set_visible_debug(bool v);
    bool get_visible_debug() const;

    void set_disabled(bool d);
    bool get_disabled() const;

    void set_max_hp(int h);
    int get_max_hp() const;

    void set_fire_collision_layer(int l);
    int get_fire_collision_layer() const;

    void set_flammable_collision_layer(int l);
    int get_flammable_collision_layer() const;

    void set_resolution(Vector3i t);
    Vector3i get_resolution() const;

    int get_spread_budget() const;
    void set_spread_budget(int v);

    bool is_on_fire() const;

    void set_default_emitter(const Ref<PackedScene> &scene);
    Ref<PackedScene> get_default_emitter() const;
    void set_override_emitter(const Ref<PackedScene> &scene);
    Ref<PackedScene> get_override_emitter() const;


protected:
    static void _bind_methods();

    void apply_fire(Vector3 world_pos, int damage);

private:

    // globals
    static int s_spread_budget;
    static int s_max_spread;
    static int s_fire_collision_layer;
    static int s_flammable_collision_layer;
    static Ref<PackedScene> s_default_emitter_scene;

    // per-instance override
    Ref<PackedScene> _override_emitter_scene;

    
    // exports
    Vector3i grid_resolution = Vector3i(3, 3, 3);
    bool disabled = false;
    bool is_torch = false;
    bool visible_debug = false;
    int max_hitpoints = 45;


    int spread_damage = 10;
    float spread_margin = 0.7f;
    float spread_interval = SPREAD_UPDATE_S;

    // internals
    int _last_max_hp = 45;
    float _dbg_visuals_timer = DBG_VISUAL_UPDATE_S;

    Node3D * _parent = nullptr;
    uint32_t _burn_count = 0;
    float _spread_timer = SPREAD_UPDATE_S;

    // grid setup
    std::unordered_map<Vector3i, fire_cell_t, Vector3iHash, Vector3iEq> _grid;
    AABB _local_aabb;
    Vector3 _cell_size;
    std::vector<Plane> _convex_planes;
    bool _is_convex = false;

    // colliding burn area
    CollisionShape3D * _base_col_inst = nullptr;
    CollisionShape3D * _burn_area_col_inst = nullptr;
    Area3D * _burn_area = nullptr;
    Ref<BoxShape3D> _burn_box_ref;

    Ref<SphereMesh> _dbg_sphere_ref;

    // funcs

    bool _is_inside_object(Vector3 local_pos);
    fire_cell_t * _get_closest_cell(Vector3 world_pos, Vector3i * cell);
    void _ignite_cell(Vector3i cell);
    void _extinguish_cell(Vector3i cell);
    
    void _on_ready();
    void _on_process(double delta);
    void _on_xform_changed();

    void _build_grid();
    void _clear_grid();
    void _cleanup();
    void _intra_spread(Vector3i pos, fire_cell_t & data, float dt);
    void _check_inter_spread();
    void _update_burn_area();
    Ref<PackedScene> _get_emitter_scene() const;

    static void _shutdown_static_resources();

};

}
