#include "destronoi.hpp"
#include "godot_cpp/classes/physics_server3d.hpp"
#include "godot_cpp/classes/primitive_mesh.hpp"
#include "godot_cpp/classes/rendering_server.hpp"
#include "godot_cpp/classes/world3d.hpp"
#include "godot_cpp/variant/basis.hpp"
#include "godot_cpp/variant/string_name.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/typed_array.hpp"
#include "vst_node.hpp"

#include <cstdlib>
#include <godot_cpp/classes/engine.hpp>
#include "godot_cpp/classes/array_mesh.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/core/defs.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/vector3.hpp"

#include <cmath>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/classes/mesh_data_tool.hpp>
#include <godot_cpp/classes/physics_body3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/scene_tree_timer.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/classes/random_number_generator.hpp>

using namespace godot;



// GODOT INTERFACE

void DestronoiNode::_bind_methods() {
	// ClassDB::bind_method(D_METHOD("_ready"), &DestronoiNode::_ready);
	ClassDB::bind_method(D_METHOD("destroy", "radial_velocity", "linear_velocity"), &DestronoiNode::destroy, DEFVAL(5.0), DEFVAL(Vector3()));
    ClassDB::bind_method(D_METHOD("_cleanup"), &DestronoiNode::_cleanup);

	ClassDB::bind_method(D_METHOD("set_granularity", "h"), &DestronoiNode::set_granularity);
	ClassDB::bind_method(D_METHOD("get_granularity"), &DestronoiNode::get_granularity);

    ClassDB::bind_method(D_METHOD("set_persistence", "s"), &DestronoiNode::set_persistence);
	ClassDB::bind_method(D_METHOD("get_persistence"), &DestronoiNode::get_persistence);

    ClassDB::bind_method(D_METHOD("set_tangible_shards", "s"), &DestronoiNode::set_tangible_shards);
	ClassDB::bind_method(D_METHOD("get_tangible_shards"), &DestronoiNode::get_tangible_shards);

    ClassDB::bind_method(D_METHOD("set_inner_material", "material"), &DestronoiNode::set_inner_material);
    ClassDB::bind_method(D_METHOD("get_inner_material"), &DestronoiNode::get_inner_material);
	ADD_PROPERTY(
        PropertyInfo(Variant::INT, "granularity", PROPERTY_HINT_RANGE, "1,9,1"),
        "set_granularity",
        "get_granularity");
    
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "persistence", PROPERTY_HINT_RANGE, "-1.0,20.0,0.1"),
        "set_persistence",
        "get_persistence");

    ADD_PROPERTY(
        PropertyInfo(Variant::BOOL, "tangible_shards"),
        "set_tangible_shards",
        "get_tangible_shards"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::OBJECT, "inner_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"),
        "set_inner_material",
        "get_inner_material"
    );
}

void DestronoiNode::set_granularity(int h) {
	granularity = h < 2 ? 2 : h;
}

int DestronoiNode::get_granularity() const {
	return granularity;
}

void DestronoiNode::set_persistence(float s) {
    persistence = s;
}
float DestronoiNode::get_persistence() const {
    return persistence;
}

void DestronoiNode::set_tangible_shards(bool s) {
    tangible_shards = s;
}
bool DestronoiNode::get_tangible_shards() const {
    return tangible_shards;
}

void DestronoiNode::set_inner_material(const Ref<Material> &m) {
    inner_material = m;
}

Ref<Material> DestronoiNode::get_inner_material() const {
    return inner_material;
}


// STATIC HELPERS

typedef struct {
    int a_id;
    int b_id;
    Vector3 p;
} intersection_t;

static void _emit_vertex(Ref<SurfaceTool> st, Ref<MeshDataTool> data, int vid) {
    Vector3 pos = data->get_vertex(vid);
    if(pos == Vector3(0,0,0)) {
        return;
    }

    st->set_uv(data->get_vertex_uv(vid));
    st->set_normal(data->get_vertex_normal(vid));
    st->set_tangent(data->get_vertex_tangent(vid));
    st->set_color(data->get_vertex_color(vid));
    st->add_vertex(pos);
}

static void _emit_intersection(Ref<SurfaceTool> st, Ref<MeshDataTool> data, const intersection_t * n) {
    Vector3 a = data->get_vertex(n->a_id);
    Vector3 b = data->get_vertex(n->b_id);

    float t = a.distance_to(n->p) / MAX(a.distance_to(b), 0.000001f);
    
    Vector2 uv = data->get_vertex_uv(n->a_id).lerp(data->get_vertex_uv(n->b_id), t);
    
    Vector3 nrm = data->get_vertex_normal(n->a_id).lerp(
		data->get_vertex_normal(n->b_id), t
	).normalized();

    Color col = data->get_vertex_color(n->a_id).lerp(
		data->get_vertex_color(n->b_id), t
	);

    Plane ta = data->get_vertex_tangent(n->a_id);
    Plane tb = data->get_vertex_tangent(n->b_id);
    Plane tan = Plane(
		ta.normal.x + (tb.normal.x - ta.normal.x) * t,
		ta.normal.y + (tb.normal.y - ta.normal.y) * t,
		ta.normal.z + (tb.normal.z - ta.normal.z) * t,
		ta.d + (tb.d - ta.d) * t
	);

    st->set_uv(uv);
	st->set_normal(nrm);
	st->set_tangent(tan);
	st->set_color(col);
	st->add_vertex(n->p);
}

// static inline bool _point_inside_convex(
//         const Vector3 &p,
//         const Vector<Plane> &planes,
//         float eps = 0.00001f) {

//     for (int i = 0; i < planes.size(); i++) {
//         if (planes[i].distance_to(p) > eps) {
//             return false;
//         }
//     }
//     return true;
// }


// static void _plot_sites_random(Ref<MeshDataTool> mdt, VSTNode * node, Ref<RandomNumberGenerator> rng) {
// 	if(node->_mesh_ref.is_null()) {
//         return;
//     }
    
//     node->_sites.clear();

//     mdt->clear();
// 	mdt->create_from_surface(node->_mesh_ref, 0);

// 	AABB aabb = node->_mesh_ref->get_aabb();
// 	Vector3 min_vec = aabb.position;
// 	Vector3 max_vec = aabb.get_end();

// 	float avg_x = 0.5f * (max_vec.x + min_vec.x);
// 	float avg_y = 0.5f * (max_vec.y + min_vec.y);
// 	float avg_z = 0.5f * (max_vec.z + min_vec.z);

// 	float dev = 0.1f; // put this on a noise texture?

// 	while (node->_sites.size() < 2) {
// 		Vector3 site(
// 				rng->randfn(avg_x, dev),
// 				rng->randfn(avg_y, dev),
// 				rng->randfn(avg_z, dev));

// 		int num_intersections = 0;

// 		for (int tri = 0; tri < mdt->get_face_count(); tri++) {
// 			int v0 = mdt->get_face_vertex(tri, 0);
// 			int v1 = mdt->get_face_vertex(tri, 1);
// 			int v2 = mdt->get_face_vertex(tri, 2);

// 			Vector3 a = mdt->get_vertex(v0);
// 			Vector3 b = mdt->get_vertex(v1);
// 			Vector3 c = mdt->get_vertex(v2);

// 			Variant hit = Geometry3D::get_singleton()->ray_intersects_triangle(site, Vector3(0, 1, 0), a, b, c);

// 			if (hit.get_type() != Variant::NIL) {
// 				num_intersections++;
// 			}
// 		}

// 		if (num_intersections == 1) {
// 			node->_sites.push_back(site);
//         }
// 	}
// }

static void _plot_sites_random(
        Ref<MeshDataTool> mdt,
        VSTNode *node,
        Ref<RandomNumberGenerator> rng) {

    if (node->_mesh_ref.is_null()) {
        return;
    }

    node->_sites.clear();

    mdt->clear();
    mdt->create_from_surface(node->_mesh_ref, 0);

    AABB aabb = node->_mesh_ref->get_aabb();
    Vector3 min_vec = aabb.position;
    Vector3 max_vec = aabb.get_end();

    float avg_x = 0.5f * (max_vec.x + min_vec.x);
    float avg_y = 0.5f * (max_vec.y + min_vec.y);
    float avg_z = 0.5f * (max_vec.z + min_vec.z);

    float dev = 0.1f;

    const int MAX_ATTEMPTS = 256;
    int attempts = 0;

    while (node->_sites.size() < 2 && attempts < MAX_ATTEMPTS) {
        attempts++;

        Vector3 site(
            rng->randfn(avg_x, dev),
            rng->randfn(avg_y, dev),
            rng->randfn(avg_z, dev)
        );

        int num_intersections = 0;

        for (int tri = 0; tri < mdt->get_face_count(); tri++) {
            int v0 = mdt->get_face_vertex(tri, 0);
            int v1 = mdt->get_face_vertex(tri, 1);
            int v2 = mdt->get_face_vertex(tri, 2);

            Vector3 a = mdt->get_vertex(v0);
            Vector3 b = mdt->get_vertex(v1);
            Vector3 c = mdt->get_vertex(v2);

            Variant hit = Geometry3D::get_singleton()->ray_intersects_triangle(
                site, Vector3(0, 1, 0), a, b, c
            );

            if (hit.get_type() != Variant::NIL) {
                num_intersections++;
            }
        }

        if (num_intersections == 1) {
            node->_sites.push_back(site);
        }
    }

    // hard fallback: guarantee progress
    if (node->_sites.size() < 2) {
        Vector3 c = aabb.get_center();
        node->_sites.clear();
        node->_sites.push_back(c + Vector3(0.01f, 0, 0));
        node->_sites.push_back(c - Vector3(0.01f, 0, 0));
    }
}


bool DestronoiNode::_bisect(Ref<SurfaceTool> sta, Ref<SurfaceTool> stb,
        Ref<MeshDataTool> data_tool, VSTNode * vst_node) {
    if(vst_node->_mesh_ref.is_null()) {
        return false;
    }
    // _bisection aborted! Must have exactly 2 sites
    if (vst_node->get_site_count() != 2) {
        return false;
    }

    // Create the plane
    // Equidistant from both sites; normal vector towards site B
    Vector3 site_a = vst_node->_sites[0];
    Vector3 site_b = vst_node->_sites[1];
    Vector3 plane_normal = (site_b - site_a).normalized();  // a → b
    Vector3 plane_position = site_a + 0.5f * (site_b - site_a); // midpoint
    Plane plane(plane_normal, plane_position);

    // Create MeshDataTool to parse mesh data of current VSTNode
    data_tool->clear();
    data_tool->create_from_surface(vst_node->_mesh_ref, 0);
    
    // Create SurfaceTool to construct the ABOVE mesh
    sta->clear();
    sta->begin(Mesh::PRIMITIVE_TRIANGLES);
    sta->set_material(vst_node->_mat_ref);
    sta->set_smooth_group(-1);

    // Create SurfaceTool to construct the BELOW mesh
    stb->clear();
    stb->begin(Mesh::PRIMITIVE_TRIANGLES);
    stb->set_material(vst_node->_mat_ref);
    stb->set_smooth_group(-1);

    Ref<SurfaceTool> tools[2] = {sta, stb};
    // UtilityFunctions::print("[Destronoi] _bisect : about to gen submeshes");

    // ---------------------------------------------------------
    // GENERATE SUB-MESHES: for each face, and for each side (above/below)
    // ---------------------------------------------------------
    for (int side = 0; side < 2; side++) {

        // Intermediate surface tool
        Ref<SurfaceTool> surface_tool = tools[side];

        // invert normal for other side (i.e. treat below as above)
        if (side == 1) {
            if(!inner_material.is_null()) {
                surface_tool->set_material(inner_material->duplicate());
            }
            plane.normal = -plane.normal;
            plane.d = -plane.d;
        } else {
            // surface_tool->set_material(outer);
        }

        Vector<Vector3> coplanar_vertices; // new vertices which intersect plane

        // ITERATE OVER EACH FACE OF BASE MESH
        // UtilityFunctions::print("[Destronoi] _bisect : about to iterate over faces");
        for (int face = 0; face < data_tool->get_face_count(); face++) {

            PackedInt32Array face_vertices;
            face_vertices.resize(3);

            PackedInt32Array vertices_above_plane;
            Vector<intersection_t> intersection_points;

            // ITERATE OVER VERTICES AND DETERMINE "ABOVENESS"
            for (int vi = 0; vi < 3; vi++) {
                int v_id = data_tool->get_face_vertex(face, vi);
                face_vertices[vi] = v_id;

                if (plane.is_point_over(data_tool->get_vertex(v_id))) {
                    vertices_above_plane.push_back(v_id);
                }
            }

            // INTERSECTION CASE 0/0.5: ALL or NOTHING above plane
            if (vertices_above_plane.size() == 0) {
                continue;
            }
            if (vertices_above_plane.size() == 3) {
                // pass through entire triangle
                for (int vi = 0; vi < 3; vi++) {
                    int id = face_vertices[vi];
                    _emit_vertex(surface_tool, data_tool, id);
                }
                continue;
            }

            // ---------------------------------------------------------
            // INTERSECTION CASE 1: ONE point above the plane
            // ---------------------------------------------------------
            if (vertices_above_plane.size() == 1) {
                int index_before = -1;
                int index_after  = -1;

                // find indices in winding order
                for (int i = 0; i < 3; i++) {
                    if (face_vertices[i] == vertices_above_plane[0]) {
                        index_after  = (i + 1) % 3;
                        index_before = (i + 2) % 3;
                        break;
                    }
                }

                Vector3 i_after;
                plane.intersects_segment(
                    data_tool->get_vertex(vertices_above_plane[0]),
                    data_tool->get_vertex(face_vertices[index_after]),
                    &i_after
                );
                intersection_points.push_back((intersection_t) {
                    .a_id = vertices_above_plane[0],
                    .b_id = face_vertices[index_after],
                    .p = i_after
                });
                coplanar_vertices.push_back(i_after);

                Vector3 i_before;
                plane.intersects_segment(
                    data_tool->get_vertex(vertices_above_plane[0]),
                    data_tool->get_vertex(face_vertices[index_before]),
                    &i_before
                );
                intersection_points.push_back((intersection_t) {
                    .a_id = vertices_above_plane[0],
                    .b_id = face_vertices[index_before],
                    .p = i_before
                });
                coplanar_vertices.push_back(i_before);

                // TRIANGLE CREATION
                _emit_vertex(surface_tool, data_tool, vertices_above_plane[0]);
                _emit_intersection(surface_tool, data_tool, &intersection_points[0]);
                _emit_intersection(surface_tool, data_tool, &intersection_points[1]);
                
                continue;
            }

            // ---------------------------------------------------------
            // INTERSECTION CASE 2: TWO points above the plane
            // ---------------------------------------------------------
            if (vertices_above_plane.size() == 2) {

                // index of point below plane
                int index_remaining = -1;

                // match GDScript exactly
                if (vertices_above_plane[0] != face_vertices[1] &&
                    vertices_above_plane[1] != face_vertices[1]) {

                    // reverse the actual array, not local vars
                    int tmp = vertices_above_plane[0];
                    vertices_above_plane[0] = vertices_above_plane[1];
                    vertices_above_plane[1] = tmp;

                    index_remaining = 1;
                }
                else if (vertices_above_plane[0] != face_vertices[0] &&
                        vertices_above_plane[1] != face_vertices[0]) {

                    index_remaining = 0;
                }
                else {
                    index_remaining = 2;
                }


                Vector3 i_after;
                plane.intersects_segment(
                    data_tool->get_vertex(vertices_above_plane[1]),
                    data_tool->get_vertex(face_vertices[index_remaining]),
                    &i_after
                );
                intersection_points.push_back((intersection_t) {
                    .a_id = vertices_above_plane[1],
                    .b_id = face_vertices[index_remaining],
                    .p = i_after
                });
                coplanar_vertices.push_back(i_after);

                Vector3 i_before;
                plane.intersects_segment(
                    data_tool->get_vertex(vertices_above_plane[0]),
                    data_tool->get_vertex(face_vertices[index_remaining]),
                    &i_before
                );
                intersection_points.push_back((intersection_t) {
                    .a_id = vertices_above_plane[0],
                    .b_id = face_vertices[index_remaining],
                    .p = i_before
                });
                coplanar_vertices.push_back(i_before);

                // find shortest 'cross-length' to make 2 triangles from 4 points
                int index_shortest = 0;
                float dist_0 = data_tool->get_vertex(vertices_above_plane[0]).distance_to(i_after);
                float dist_1 = data_tool->get_vertex(vertices_above_plane[1]).distance_to(i_before);
                if (dist_1 > dist_0) {
                    index_shortest = 1;
                }

                // TRIANGLE 1
                _emit_vertex(surface_tool, data_tool, vertices_above_plane[0]);
				_emit_vertex(surface_tool, data_tool, vertices_above_plane[1]);
                _emit_intersection(surface_tool, data_tool, &intersection_points[index_shortest]);

                // TRIANGLE 2
                _emit_intersection(surface_tool, data_tool, &intersection_points[0]);
                _emit_intersection(surface_tool, data_tool, &intersection_points[1]);
                _emit_vertex(surface_tool, data_tool, vertices_above_plane[index_shortest]);


                continue;
            }
        } // end face loop
        

        // ---------------------------------------------------------
        // DEFINE NEW FACE; FIND CENTROID; APPEND TRIANGLES
        // ---------------------------------------------------------
        // UtilityFunctions::print("[Destronoi] _bisect : about to find centroid");
        Vector3 centroid(0,0,0);
        for (int i = 0; i < coplanar_vertices.size(); i++) {
            centroid += coplanar_vertices[i];
        }
        centroid /= coplanar_vertices.size();
        // if (coplanar_vertices.is_empty()) {
        //     continue; // or skip cap generation entirely
        // }

        centroid /= (float)coplanar_vertices.size();


        for (int i = 0; i < coplanar_vertices.size() - 1; i++) {
            if (i % 2 != 0) continue;
            surface_tool->add_vertex(coplanar_vertices[i + 1]);
            surface_tool->add_vertex(coplanar_vertices[i]);
            surface_tool->add_vertex(centroid);
        }
    } // end side loop

    // finalize new meshes
    sta->index();
    sta->generate_normals();
    stb->index();
    stb->generate_normals();

    // UtilityFunctions::print("[Destronoi] _bisect : allocating _left and _right");
    Ref<Mesh> left_mesh = sta->commit();
    if (left_mesh.is_valid()) {
        vst_node->_left = memnew(VSTNode(
            left_mesh,
            _root->_mat_ref,
            vst_node->_level + 1,
            Laterality::LEFT
        ));
    }

    Ref<Mesh> right_mesh = stb->commit();
    if (right_mesh.is_valid()) {
        vst_node->_right = memnew(VSTNode(
            right_mesh,
            _root->_mat_ref,
            vst_node->_level + 1,
            Laterality::RIGHT
        ));
    }


    // UtilityFunctions::print("[Destronoi] _bisect : allocated ok");

    return true;
}

void DestronoiNode::_on_ready() {

    if(Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	Node *parent = get_parent();
	if (!parent) {
		return;
    }

    Ref<RandomNumberGenerator> rng;
    rng.instantiate();
    rng->randomize();

	MeshInstance3D *mesh_instance = nullptr;

	for (int i = 0; i < parent->get_child_count(); i++) {
		Object *child = parent->get_child(i);
		if (MeshInstance3D *mi = Object::cast_to<MeshInstance3D>(child)) {
			mesh_instance = mi;
			break;
		}
	}

	if (!mesh_instance) {
		UtilityFunctions::print("[Destronoi] No MeshInstance3D sibling found");
		return;
	}

    // Ref<Material> outer_material = mesh_instance->get_active_material(0);
    // if(inner_material.is_null()) {
    //     inner_material = outer_material;
    // }


    UtilityFunctions::print("[Destronoi] GOT HERE, EXPECTING THIS TO MAYBE HANG");
    Ref<Mesh> mm = mesh_instance->get_mesh();
    Ref<ArrayMesh> am = mm;
    if(am.is_null()) {
        Ref<PrimitiveMesh> pm = mm;
        if(pm.is_null()) {
    		UtilityFunctions::print("[Destronoi] UNSUPPORTED MESH TYPE. USE ARRAYMESH OR PRIMITIVE");
            return;
        }
        Array ar = pm->get_mesh_arrays();
        am.instantiate(); 
        am->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, ar);
    }
    // UtilityFunctions::print("[Destronoi] PAST HANGY PART");


    // UtilityFunctions::print("[Destronoi] allocating root");
    _root = memnew(VSTNode(am,
                        mesh_instance->get_active_material(0)));

    Ref<MeshDataTool> mdt;
    mdt.instantiate();
    Ref<SurfaceTool> sta;
    sta.instantiate();
    Ref<SurfaceTool> stb;
    stb.instantiate();

    // UtilityFunctions::print("[Destronoi] about to bisect");


	_plot_sites_random(mdt, _root, rng);
    
    // UtilityFunctions::print("[Destronoi] first plot ok");
	// _bisect(sta, stb, mdt, _root, outer_material, inner_material);
	_bisect(sta, stb, mdt, _root);
    // UtilityFunctions::print("[Destronoi] first bisect ok");

	for (int i = 0; i < granularity - 1; i++) {
		std::vector<VSTNode *> leaves;
		_root->get_leaf_nodes(leaves);

		for (VSTNode * leaf : leaves) {
			_plot_sites_random(mdt, leaf, rng);
			// _bisect(sta, stb, mdt, leaf, outer_material, inner_material);
			_bisect(sta, stb, mdt, leaf);
		}
	}
    // UtilityFunctions::print("[Destronoi] bisect done");
    UtilityFunctions::print("[Destronoi] _ON_READY OK");

}



void DestronoiNode::_cleanup() {


    // // /////////////  Rendering/Physics server direct attempt ///////////////////
    // PhysicsServer3D * ps = PhysicsServer3D::get_singleton();
    // RenderingServer * rs = RenderingServer::get_singleton();
    // for(size_t i=0; i<_body_rids.size(); i++) {
    //     ps->free_rid(_body_rids[i]);
    //     rs->free_rid(_mesh_rids[i]);
    // }
    // _body_rids.clear();
    // _mesh_rids.clear();
    // // /////////////


    queue_free();
}

// void DestronoiNode::_exit_tree() {
//     _cleanup();
// }

// DestronoiNode::~DestronoiNode() {
//     _cleanup();
// }


void DestronoiNode::destroy(float radial_velocity, Vector3 linear_velocity) {
    PhysicsBody3D *base_object = Object::cast_to<PhysicsBody3D>(get_parent());
    if (!base_object || !_root) {
        return;
    }

    int depth = std::pow(2, granularity-1);

    // collect leaves to use
    std::vector<VSTNode *> vst_leaves;
    _root->get_left_leaf_nodes(vst_leaves, depth);
    _root->get_right_leaf_nodes(vst_leaves, depth);

    float sum_mass = 0.0f;

    Transform3D base_xform = base_object->get_global_transform();

    // scale masses
    float base_mass = 1.0f;
    if (Object::cast_to<RigidBody3D>(base_object)) {
        base_mass = Object::cast_to<RigidBody3D>(base_object)->get_mass();
    }
    
    std::vector<RigidBody3D *> new_bodies;
    std::vector<MeshInstance3D *> new_meshes;
    std::vector<CollisionShape3D *> new_cols;

    new_bodies.reserve(vst_leaves.size());
    new_meshes.reserve(vst_leaves.size());
    new_cols.reserve(vst_leaves.size());

    TypedArray<StringName> pgroups = base_object->get_groups();

    for (size_t idx = 0; idx < vst_leaves.size(); idx++) {
        VSTNode * leaf = vst_leaves[idx];

        RigidBody3D *new_body = memnew(RigidBody3D);
        MeshInstance3D *new_mesh_instance = memnew(MeshInstance3D);
        CollisionShape3D *col = memnew(CollisionShape3D);

        new_bodies.push_back(new_body);
        new_meshes.push_back(new_mesh_instance);
        new_cols.push_back(col);

        // new rigidbody
        new_body->set_name("VFragment_" + String::num_int64(idx));

        // bring the mesh over
        new_mesh_instance->set_mesh(leaf->_mesh_ref);
        new_mesh_instance->set_name("MeshInstance3D");
        new_body->add_child(new_mesh_instance);

        // collision
        col->set_name("CollisionShape3D");
        col->set_shape(leaf->_mesh_ref->create_convex_shape(false, false));
        new_body->add_child(col);

        // // /////////////  Rendering/Physics server direct attempt ///////////////////
        // PhysicsServer3D * ps = PhysicsServer3D::get_singleton();
        // RID body_rid = ps->body_create();
        // Ref<ConvexPolygonShape3D> sh = leaf->_mesh_ref->create_convex_shape(true, true);
        // ps->body_set_space(body_rid, base_object->get_world_3d()->get_space());
        // RID sh_rid = sh->get_rid();
        // UtilityFunctions::prints("rid:",sh_rid);
        // ps->body_add_shape(body_rid, sh_rid);
        // ps->body_set_shape_transform(body_rid, 0, Transform3D());
        // ps->body_set_state(body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM, base_xform);
        // _body_rids.push_back(body_rid);

        // UtilityFunctions::prints("shape count:", ps->body_get_shape_count(body_rid));

        // RenderingServer * rs = RenderingServer::get_singleton();
        // RID mesh_rid = rs->instance_create2(leaf->_mesh_ref->get_rid(), base_object->get_world_3d()->get_scenario());
        // rs->instance_set_transform(mesh_rid, base_xform);
        // _mesh_rids.push_back(mesh_rid);
        // // /////////////


        // velocity dir = AABB center - base position   (NOTE: this is *local*, matches gdscript bug-for-bug)
        AABB aabb = leaf->_mesh_ref->get_aabb();
        Vector3 velocity_dir = aabb.get_center() - base_xform.origin;
        velocity_dir = velocity_dir.normalized();

        // mass = max(volume, 0.1)
        float vol = aabb.get_volume();
        float mass = vol < 0.1f ? 0.1f : vol;
        new_body->set_mass(mass);
        sum_mass += mass;

        // combustion outward velocity
        Vector<Vector3> endpoints;
        Vector3 estim_dir(0, 0, 0);

        for (int i = 0; i < 8; i++) {
            Vector3 ep = leaf->_mesh_ref->get_aabb().get_endpoint(i);
            ep = ep.normalized();

            float d = ep.dot(base_xform.origin.normalized());
            if (Math::abs(d) > 0.0f) {
                endpoints.push_back(ep);
            }
        }

        for (const Vector3 &ep : endpoints) {
            estim_dir += ep;
        }

        if (!endpoints.is_empty()) {
            estim_dir /= (float)endpoints.size();
        }

        estim_dir = estim_dir.normalized() * radial_velocity;
        // estim_dir = estim_dir.lerp(linear_velocity.normalized(), 0.5f);
        estim_dir += linear_velocity;
        new_body->set_axis_velocity(estim_dir);

        float scaled = new_body->get_mass() * (base_mass / sum_mass);
        new_body->set_mass(scaled);
        new_body->set_collision_mask(base_object->get_collision_mask());

        if(tangible_shards) {
            new_body->set_collision_layer(base_object->get_collision_layer());
        } else {
            new_body->set_collision_layer(0);
        }
        add_child(new_body);
        new_body->set_global_transform(base_xform);
        for(StringName g : pgroups) {
            new_body->add_to_group(g);
        }
        // UtilityFunctions::print("CREATE new_body ", new_body->get_name(), " in tree:", new_body->is_inside_tree());
    }


    reparent(base_object->get_parent());
    base_object->queue_free();

    // delete vst tree, original rigidbody
    // if(_root) {
    //     _root->free_tree();
    //     memdelete(_root);
    //     _root = nullptr;
    // }

    if(!(persistence > 0.0f)) {
        return;
    }
    Ref<SceneTreeTimer> tm = get_tree()->create_timer(persistence, true);
    tm->connect("timeout", Callable(this, "_cleanup"));
}

void DestronoiNode::_notification(int p_what) {
    switch (p_what) {
		case NOTIFICATION_READY: {
            _on_ready();
            break;
        }

        case NOTIFICATION_EXIT_TREE: {
            if(_root) {
                _root->free_tree();
                memdelete(_root);
                _root = nullptr;
            }
            break;
        }

        default:
            break;
    }
}
