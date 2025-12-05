#pragma once

#include <vector>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/core/class_db.hpp>

namespace godot {

class VSTNode;
class RigidBody3D;

class DestronoiNode : public Node {
    GDCLASS(DestronoiNode, Node);

public:
    DestronoiNode() = default;
    ~DestronoiNode() override = default;

    void set_tree_height(int h);
    int get_tree_height() const;

    void set_visible_seconds(float s);
    float get_visible_seconds() const;

    void set_inner_material(const Ref<Material> &m);
    Ref<Material> get_inner_material() const;

    void generate();                     // GDScript `_ready()` equivalent
    void destroy(int left_val = 1,
                 int right_val = 1,
                 float combust_velocity = 0.0f);
    void _cleanup();

protected:
    static void _bind_methods();

private:
    int tree_height = 5;                 // @export_range(1,8)
    float visible_seconds = 1.0f;
    VSTNode *_root = nullptr;            // root of the VST
    std::vector<RigidBody3D *> new_bodies;
    Ref<Material> inner_material;
};

}
