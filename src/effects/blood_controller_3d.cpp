#include "effects/blood_controller_3d.hpp"

#include "godot_cpp/classes/gpu_particles3d.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/particle_process_material.hpp"
#include "godot_cpp/classes/plane_mesh.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/scene_tree_timer.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

namespace godot {
namespace {

void expire(Node *effect, double seconds) {
    const Ref<SceneTreeTimer> timer = effect->get_tree()->create_timer(seconds, false);
    timer->connect("timeout", Callable(effect, "queue_free"), Object::CONNECT_ONE_SHOT);
}

} // namespace

void BloodController3D::splatter(const Vector3 &direction) {
    if (!is_inside_tree() || !get_tree()->get_current_scene() || !direction.is_finite()) {
        return;
    }

    Node *world = get_tree()->get_current_scene();
    const Vector3 origin = get_global_position();
    spawn_particles(world, origin, direction.is_zero_approx() ? Vector3(0, 1, 0) : direction.normalized());
    if (stain_textures.is_empty() || stain_scale <= 0.0f || stain_timeout <= 0.0f) {
        return;
    }

    const Vector3 horizontal = Vector3(direction.x, 0, direction.z).limit_length(1.0f);
    const Vector3 surface_origin = origin + Vector3(0, stain_y_offset + 0.008f, 0);
    const float size = stain_scale * UtilityFunctions::randf_range(0.7, 1.1);
    if (horizontal.is_zero_approx()) {
        spawn_stain(world, surface_origin, UtilityFunctions::randf_range(0.0, 6.2831853), Vector2(size, size));
        return;
    }

    const float bearing = Math::atan2(horizontal.x, horizontal.z);
    spawn_stain(world, surface_origin + horizontal * 0.6f, bearing + UtilityFunctions::randf_range(-0.25, 0.25),
            Vector2(size, size * (1.0f + horizontal.length() * 0.65f)));
    const float satellite_size = size * UtilityFunctions::randf_range(0.25, 0.45);
    const Vector3 side(horizontal.z, 0, -horizontal.x);
    spawn_stain(world, surface_origin + horizontal * 1.1f + side * UtilityFunctions::randf_range(-0.25, 0.25) +
                Vector3(0, 0.001f, 0), bearing, Vector2(satellite_size, satellite_size * 1.4f));
}

void BloodController3D::spawn_particles(Node *world, const Vector3 &origin, const Vector3 &direction) const {
    if (splatter_scene.is_null()) {
        return;
    }
    Node *instance = splatter_scene->instantiate();
    auto *particles = Object::cast_to<GPUParticles3D>(instance);
    if (!particles) {
        if (instance) {
            memdelete(instance);
        }
        UtilityFunctions::push_warning("BloodController3D splatter scene needs a GPUParticles3D root.");
        return;
    }

    particles->set_emitting(false);
    particles->set_one_shot(true);
    const Ref<ParticleProcessMaterial> material = particles->get_process_material();
    if (material.is_valid()) {
        Ref<ParticleProcessMaterial> local_material = material->duplicate();
        local_material->set_direction(direction);
        particles->set_process_material(local_material);
    }
    world->add_child(particles);
    particles->set_global_position(origin + Vector3(0, splat_y_offset, 0));
    particles->set_emitting(true);
    expire(particles, particles->get_lifetime() * 2.0);
}

void BloodController3D::spawn_stain(Node *world, const Vector3 &position, float angle, const Vector2 &size) const {
    const Ref<Texture2D> texture = stain_textures[UtilityFunctions::randi_range(0, stain_textures.size() - 1)];
    if (texture.is_null()) {
        return;
    }

    Ref<StandardMaterial3D> material;
    material.instantiate();
    material->set_texture(BaseMaterial3D::TEXTURE_ALBEDO, texture);
    material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
    material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);

    Ref<PlaneMesh> mesh;
    mesh.instantiate();
    mesh->set_size(size);
    mesh->set_material(material);

    auto *stain = memnew(MeshInstance3D);
    stain->set_name("BloodStain");
    stain->set_mesh(mesh);
    stain->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
    world->add_child(stain);
    stain->set_global_position(position);
    stain->set_global_rotation(Vector3(0, angle, 0));
    expire(stain, stain_timeout);
}

void BloodController3D::set_stain_timeout(float seconds) {
    stain_timeout = MAX(0.0f, seconds);
}

void BloodController3D::set_stain_scale(float scale) {
    stain_scale = MAX(0.0f, scale);
}

void BloodController3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("splatter", "direction"), &BloodController3D::splatter, DEFVAL(Vector3()));
    ClassDB::bind_method(D_METHOD("set_splatter_scene", "scene"), &BloodController3D::set_splatter_scene);
    ClassDB::bind_method(D_METHOD("get_splatter_scene"), &BloodController3D::get_splatter_scene);
    ClassDB::bind_method(D_METHOD("set_stain_textures", "textures"), &BloodController3D::set_stain_textures);
    ClassDB::bind_method(D_METHOD("get_stain_textures"), &BloodController3D::get_stain_textures);
    ClassDB::bind_method(D_METHOD("set_stain_timeout", "seconds"), &BloodController3D::set_stain_timeout);
    ClassDB::bind_method(D_METHOD("get_stain_timeout"), &BloodController3D::get_stain_timeout);
    ClassDB::bind_method(D_METHOD("set_stain_scale", "scale"), &BloodController3D::set_stain_scale);
    ClassDB::bind_method(D_METHOD("get_stain_scale"), &BloodController3D::get_stain_scale);
    ClassDB::bind_method(D_METHOD("set_stain_y_offset", "offset"), &BloodController3D::set_stain_y_offset);
    ClassDB::bind_method(D_METHOD("get_stain_y_offset"), &BloodController3D::get_stain_y_offset);
    ClassDB::bind_method(D_METHOD("set_splat_y_offset", "offset"), &BloodController3D::set_splat_y_offset);
    ClassDB::bind_method(D_METHOD("get_splat_y_offset"), &BloodController3D::get_splat_y_offset);

    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "splatter_scene", PROPERTY_HINT_RESOURCE_TYPE, "PackedScene"),
            "set_splatter_scene", "get_splatter_scene");
    ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "stain_textures", PROPERTY_HINT_ARRAY_TYPE, "Texture2D"),
            "set_stain_textures", "get_stain_textures");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stain_timeout"), "set_stain_timeout", "get_stain_timeout");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stain_scale"), "set_stain_scale", "get_stain_scale");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stain_y_offset"), "set_stain_y_offset", "get_stain_y_offset");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "splat_y_offset"), "set_splat_y_offset", "get_splat_y_offset");
}

} // namespace godot
