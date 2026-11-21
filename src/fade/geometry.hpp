#pragma once

#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include <godot_cpp/classes/ref.hpp>

#define DEFAULT_FADE_Y_MARGIN 2.0f

namespace godot {

namespace fade_geometry {

inline float get_xz_dist(Vector3 targetpos, Vector2 aabbmin, Vector2 aabbmax) {
    Vector2 t2(targetpos.x, targetpos.z);
    return t2.distance_to(
        Vector2(
            Math::clamp(t2.x, aabbmin.x, aabbmax.x),
            Math::clamp(t2.y, aabbmin.y, aabbmax.y)
        )
    );
}

inline void precompute_bounds(MeshInstance3D * mesh_inst, Vector2 & aabbmin, Vector2 & aabbmax, Vector2 & center2d, Vector2 & wallnormal) {
    Ref<Mesh> mesh = mesh_inst->get_mesh();
    if(!mesh.is_valid()) {
        UtilityFunctions::push_error("[FadeGeometry] ", mesh_inst->get_name(), " precompute_bounds : Mesh is invalid. abort");
        return;
    }
    
    float minx = INFINITY;
    float maxx = -INFINITY;

    float minz = INFINITY;
    float maxz = -INFINITY;

    float maxy = -INFINITY;

    Transform3D xf = mesh_inst->get_global_transform();
    Array arr = mesh->surface_get_arrays(0)[Mesh::ARRAY_VERTEX];

    for(int i=0; i<arr.size(); i++) {
        Vector3 v = arr[i];
        Vector3 w = xf.xform(v);
        minx = MIN(minx, w.x);
		maxx = MAX(maxx, w.x);

		minz = MIN(minz, w.z);
		maxz = MAX(maxz, w.z);

		maxy = MAX(maxy, w.y);
    }

    aabbmin = Vector2(minx, minz);
    aabbmax = Vector2(maxx, maxz);
    center2d = (aabbmin + aabbmax) * 0.5f;

    float dx = Math::abs(aabbmax.x - aabbmin.x);
    float dz = Math::abs(aabbmax.y - aabbmin.y);

    // pillar / square
    if(Math::abs(dx - dz) < 0.1f) {
        wallnormal = Vector2(0,0);
    // z-facing
    } else if(dx > dz) {
        wallnormal = Vector2(0,1);
    // x-facing
    } else {
        wallnormal = Vector2(1,0);
    }
}

} // namespace fade_geometry

} // namespace godot
