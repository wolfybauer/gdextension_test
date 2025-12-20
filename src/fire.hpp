#pragma once

#include "godot_cpp/classes/collision_shape3d.hpp"
#include "godot_cpp/variant/plane.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <unordered_map>
#include <vector>

namespace godot {

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
    int hitpoints;
    bool burning;
    float cooldown;
    float time_left;
    Vector3 local_pos;
    Node3D * emitter;
} fire_cell_t;

class FireComponent3D : public Node3D {
    GDCLASS(FireComponent3D, Node3D);

public:
    FireComponent3D() = default;
    ~FireComponent3D() override = default;

    void _ready() override;
    void _process(double p_delta) override;

    void set_torch(bool t);
    bool get_torch() const;

    void set_resolution(Vector3i t);
    Vector3i get_resolution() const;

protected:
    static void _bind_methods();

    void apply_fire(Vector3 world_pos, int damage);

private:
    bool is_torch = false;
    bool enable_interior_cells = false;
    int spread_damage = 10;
    float spread_margin = 0.7f;
    float spread_interval = 0.5f;
    Vector3i grid_resolution = Vector3i(3, 3, 3);
    
    std::unordered_map<Vector3i, fire_cell_t, Vector3iHash, Vector3iEq> _grid;
    AABB _collision_aabb;
    Vector3 _cell_size;
    bool _is_convex = false;
    CollisionShape3D * _col_inst;
    std::vector<Plane> _convex_planes;

    bool _is_inside_object(Vector3 world_pos);
    fire_cell_t * _get_closest_cell(Vector3 world_pos, Vector3i * cell);
    void _ignite_cell(Vector3i cell);
    void _extinguish_cell(Vector3i cell);
    
    void _build_grid();
};

}
