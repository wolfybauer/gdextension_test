#pragma once

#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/packed_scene.hpp"
#include "godot_cpp/classes/texture2d.hpp"
#include "godot_cpp/variant/typed_array.hpp"

namespace godot {

class BloodController3D : public Node3D {
    GDCLASS(BloodController3D, Node3D)

public:
    void splatter(const Vector3 &direction);

    void set_splatter_scene(const Ref<PackedScene> &scene) { splatter_scene = scene; }
    Ref<PackedScene> get_splatter_scene() const { return splatter_scene; }
    void set_stain_textures(const TypedArray<Texture2D> &textures) { stain_textures = textures; }
    TypedArray<Texture2D> get_stain_textures() const { return stain_textures; }
    void set_stain_timeout(float seconds);
    float get_stain_timeout() const { return stain_timeout; }
    void set_stain_scale(float scale);
    float get_stain_scale() const { return stain_scale; }
    void set_stain_y_offset(float offset) { stain_y_offset = offset; }
    float get_stain_y_offset() const { return stain_y_offset; }
    void set_splat_y_offset(float offset) { splat_y_offset = offset; }
    float get_splat_y_offset() const { return splat_y_offset; }

protected:
    static void _bind_methods();

private:
    Ref<PackedScene> splatter_scene;
    TypedArray<Texture2D> stain_textures;
    float stain_timeout = 10.0f;
    float stain_scale = 1.0f;
    float stain_y_offset = 0.0f;
    float splat_y_offset = 1.0f;

    void spawn_particles(Node *world, const Vector3 &origin, const Vector3 &direction) const;
    void spawn_stain(Node *world, const Vector3 &position, float angle, const Vector2 &size) const;
};

} // namespace godot
