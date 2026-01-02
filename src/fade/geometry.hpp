#pragma once

#include "godot_cpp/core/math.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include <godot_cpp/classes/ref.hpp>

#define DEFAULT_FADE_Y_MARGIN 2.0f

namespace godot {

namespace fade_geometry {

inline bool point_in_aabb(Vector2 p, Vector2 aabbmin, Vector2 aabbmax) {
    return (
        p.x >= aabbmin.x && p.x <= aabbmax.x &&
        p.y >= aabbmin.y && p.y <= aabbmax.y
    );
}

inline bool seg_seg(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4) {
    Vector2 d1 = p2 - p1;
    Vector2 d2 = p4 - p3;

    float denom = d1.cross(d2);
    if(Math::abs<float>(denom) < 0.000001f) {
        return false;
    }

    float t = (p3 - p1).cross(d2) / denom;
    float u = (p3 - p1).cross(d1) / denom;

    return (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f);
}

inline bool segment_hits_aabb(Vector2 cam2, Vector2 ply2, Vector2 aabbmin, Vector2 aabbmax) {
    if(point_in_aabb(cam2, aabbmin, aabbmax) || point_in_aabb(ply2, aabbmin, aabbmax)) {
        return true;
    }
    if (seg_seg(cam2, ply2, Vector2(aabbmin.x, aabbmin.y), Vector2(aabbmin.x, aabbmax.y))) { return true; }
	if (seg_seg(cam2, ply2, Vector2(aabbmax.x, aabbmin.y), Vector2(aabbmax.x, aabbmax.y))) { return true; }
	if (seg_seg(cam2, ply2, Vector2(aabbmin.x, aabbmin.y), Vector2(aabbmax.x, aabbmin.y))) { return true; }
	if (seg_seg(cam2, ply2, Vector2(aabbmin.x, aabbmax.y), Vector2(aabbmax.x, aabbmax.y))) { return true; }
    return false;
}

inline bool projected_outside(Vector2 cam2, Vector2 ply2, Vector2 center2d) {
    Vector2 v = ply2 - cam2;
    Vector2 w = center2d - cam2;
	// if dot signs differ or wall lies further than target -> no occlusion
    return (v.dot(w) <= 0.0f || w.length() > v.length());
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

    float dx = Math::abs<float>(aabbmax.x - aabbmin.x);
    float dz = Math::abs<float>(aabbmax.y - aabbmin.y);

    // pillar / square
    if(Math::abs<float>(dx - dz) < 0.1f) {
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
