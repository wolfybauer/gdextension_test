#pragma once

#include "godot_cpp/classes/mesh_data_tool.hpp"
#include "godot_cpp/classes/surface_tool.hpp"
#include "vst_node.hpp"
#include <godot_cpp/classes/ref.hpp>
#include "godot_cpp/variant/vector3.hpp"
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/core/class_db.hpp>

namespace godot {

class RigidBody3D;
class CollisionShape3D;

class DestronoiNode : public Node {
    GDCLASS(DestronoiNode, Node);

public:
    DestronoiNode() = default;
    ~DestronoiNode() override = default;

    void set_granularity(int h);
    int get_granularity() const;

    void set_persistence(float s);
    float get_persistence() const;

    void set_inner_material(const Ref<Material> &m);
    Ref<Material> get_inner_material() const;

    void destroy(float radial_velocity = 5.0f,
                Vector3 linear_velocity = Vector3());
    void _cleanup();

    void _ready() override;

protected:
    static void _bind_methods();

private:
    bool _bisect(Ref<SurfaceTool> sta, Ref<SurfaceTool> stb,
        Ref<MeshDataTool> data_tool, VSTNode * vst_node);
    int granularity = 5;                 // @export_range(1,8)
    float persistence = 1.0f;
    VSTNode * _root;            // root of the VST
    RigidBody3D * new_bodies;
    MeshInstance3D * new_meshes;
    CollisionShape3D * new_cols;
    Ref<Material> inner_material;
};

}
