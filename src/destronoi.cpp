#include "godot_cpp/variant/callable.hpp"
#include <random>

static std::mt19937 rng(std::random_device{}());

static float gaussian(float mean, float dev) {
	return std::normal_distribution<float>(mean, dev)(rng);
}

#include "destronoi.hpp"
#include "vst_node.hpp"

#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/classes/mesh_data_tool.hpp>
#include <godot_cpp/classes/physics_body3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/scene_tree_timer.hpp>
#include <godot_cpp/variant/callable.hpp>

using namespace godot;

void DestronoiNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("generate"), &DestronoiNode::generate);
	ClassDB::bind_method(D_METHOD("destroy", "left_val", "right_val", "combust_velocity"), &DestronoiNode::destroy);
    ClassDB::bind_method(D_METHOD("_cleanup"), &DestronoiNode::_cleanup);

	ClassDB::bind_method(D_METHOD("set_tree_height", "h"), &DestronoiNode::set_tree_height);
	ClassDB::bind_method(D_METHOD("get_tree_height"), &DestronoiNode::get_tree_height);

    ClassDB::bind_method(D_METHOD("set_visible_seconds", "s"), &DestronoiNode::set_visible_seconds);
	ClassDB::bind_method(D_METHOD("get_visible_seconds"), &DestronoiNode::get_visible_seconds);

	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "tree_height", PROPERTY_HINT_RANGE, "1,8,1"),
			"set_tree_height",
			"get_tree_height");
    
    ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "visible_seconds", PROPERTY_HINT_RANGE, "1.0,20.0,0.1"),
			"set_visible_seconds",
			"get_visible_seconds");
}

void DestronoiNode::set_tree_height(int h) {
	tree_height = h;
}

int DestronoiNode::get_tree_height() const {
	return tree_height;
}

void DestronoiNode::set_visible_seconds(float s) {
    visible_seconds = s;
}
float DestronoiNode::get_visible_seconds() const {
    return visible_seconds;
}

void DestronoiNode::generate() {
	Node *parent = get_parent();
	if (!parent) {
		return;
    }

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

	_root = memnew(VSTNode(mesh_instance));

	plot_sites_random(_root);
	bisect(_root);

	for (int i = 0; i < tree_height - 1; i++) {
		std::vector<VSTNode *> leaves;
		_root->get_leaf_nodes(leaves);

		for (VSTNode *leaf : leaves) {
			plot_sites_random(leaf);
			bisect(leaf);
		}
	}
}

void DestronoiNode::plot_sites(VSTNode *node, const Vector3 &s1, const Vector3 &s2) {
	node->_sites.clear();
	node->_sites.push_back(node->_mesh_instance->get_position() + s1);
	node->_sites.push_back(node->_mesh_instance->get_position() + s2);
}

void DestronoiNode::plot_sites_random(VSTNode *node) {
	node->_sites.clear();

	Ref<MeshDataTool> mdt;
    mdt.instantiate();
	mdt->create_from_surface(node->_mesh_instance->get_mesh(), 0);

	AABB aabb = node->_mesh_instance->get_aabb();
	Vector3 min_vec = aabb.position;
	Vector3 max_vec = aabb.get_end();

	float avg_x = 0.5f * (max_vec.x + min_vec.x);
	float avg_y = 0.5f * (max_vec.y + min_vec.y);
	float avg_z = 0.5f * (max_vec.z + min_vec.z);

	float dev = 0.1f;

	while (node->_sites.size() < 2) {
		Vector3 site(
				gaussian(avg_x, dev),
				gaussian(avg_y, dev),
				gaussian(avg_z, dev));

		int num_intersections = 0;

		for (int tri = 0; tri < mdt->get_face_count(); tri++) {
			int v0 = mdt->get_face_vertex(tri, 0);
			int v1 = mdt->get_face_vertex(tri, 1);
			int v2 = mdt->get_face_vertex(tri, 2);

			Vector3 a = mdt->get_vertex(v0);
			Vector3 b = mdt->get_vertex(v1);
			Vector3 c = mdt->get_vertex(v2);

			Variant hit = Geometry3D::get_singleton()->ray_intersects_triangle(site, Vector3(0, 1, 0), a, b, c);

			if (hit.get_type() != Variant::NIL) {
				num_intersections++;
			}
		}

		if (num_intersections == 1) {
			node->_sites.push_back(site);
        }
	}
}

bool DestronoiNode::bisect(VSTNode *vst_node) {
    // Bisection aborted! Must have exactly 2 sites
    if (vst_node->get_site_count() != 2) {
        return false;
    }

    // Colors for the new geometry
    // These colors are assigned to new vertices and may be used for a simple
    // distinction between interior and exterior faces. Example use in the debug.gdshader file.
    Vector<Color> outer_colors;
    outer_colors.push_back(Color(1, 1, 1)); // Color.WHITE
    Color inner_color = Color(1, 0, 0);     // Color.RED

    // Create the plane
    // Equidistant from both sites; normal vector towards site B
    Vector3 site_a = vst_node->_sites[0];
    Vector3 site_b = vst_node->_sites[1];
    Vector3 plane_normal = (site_b - site_a).normalized();  // a → b
    Vector3 plane_position = site_a + 0.5f * (site_b - site_a); // midpoint
    Plane plane(plane_normal, plane_position);

    // Create MeshDataTool to parse mesh data of current VSTNode
    Ref<MeshDataTool> data_tool;
    data_tool.instantiate();
    data_tool->create_from_surface(vst_node->_mesh_instance->get_mesh(), 0);

    // Create SurfaceTool to construct the ABOVE mesh
    Ref<SurfaceTool> surface_tool_a;
    surface_tool_a.instantiate();
    surface_tool_a->begin(Mesh::PRIMITIVE_TRIANGLES);
    surface_tool_a->set_material(vst_node->_mesh_instance->get_surface_override_material(0));
    surface_tool_a->set_smooth_group(-1);

    // Create SurfaceTool to construct the BELOW mesh
    Ref<SurfaceTool> surface_tool_b;
    surface_tool_b.instantiate();
    surface_tool_b->begin(Mesh::PRIMITIVE_TRIANGLES);
    surface_tool_b->set_material(vst_node->_mesh_instance->get_surface_override_material(0));
    surface_tool_b->set_smooth_group(-1);

    // ---------------------------------------------------------
    // GENERATE SUB-MESHES: for each face, and for each side (above/below)
    // ---------------------------------------------------------
    for (int side = 0; side < 2; side++) {

        // Intermediate surface tool
        Ref<SurfaceTool> surface_tool;
        surface_tool.instantiate();
        surface_tool->begin(Mesh::PRIMITIVE_TRIANGLES);
        surface_tool->set_material(vst_node->_mesh_instance->get_active_material(0));
        surface_tool->set_smooth_group(-1);

        // invert normal for other side (i.e. treat below as above)
        if (side == 1) {
            plane.normal = -plane.normal;
            plane.d = -plane.d;
            surface_tool->set_color(outer_colors[0]);
        } else {
            surface_tool->set_color(outer_colors[0]);
        }

        Vector<Vector3> coplanar_vertices; // new vertices which intersect plane

        // ITERATE OVER EACH FACE OF BASE MESH
        for (int face = 0; face < data_tool->get_face_count(); face++) {

            PackedInt32Array face_vertices;
            face_vertices.resize(3);

            PackedInt32Array vertices_above_plane;
            PackedVector3Array intersection_points;

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
                    surface_tool->add_vertex(data_tool->get_vertex(id));
                }
                continue;
            }

            // ---------------------------------------------------------
            // INTERSECTION CASE 1: ONE point above the plane
            // ---------------------------------------------------------
            if (vertices_above_plane.size() == 1) {
                int a = vertices_above_plane[0];

                int index_before = -1;
                int index_after  = -1;

                // find indices in winding order
                for (int i = 0; i < 3; i++) {
                    if (face_vertices[i] == a) {
                        index_after  = (i + 1) % 3;
                        index_before = (i + 2) % 3;
                        break;
                    }
                }

                Vector3 i_after;
                plane.intersects_segment(
                    data_tool->get_vertex(a),
                    data_tool->get_vertex(face_vertices[index_after]),
                    &i_after
                );
                intersection_points.push_back(i_after);
                coplanar_vertices.push_back(i_after);

                Vector3 i_before;
                plane.intersects_segment(
                    data_tool->get_vertex(a),
                    data_tool->get_vertex(face_vertices[index_before]),
                    &i_before
                );
                intersection_points.push_back(i_before);
                coplanar_vertices.push_back(i_before);

                // TRIANGLE CREATION
                surface_tool->add_vertex(data_tool->get_vertex(a));
                surface_tool->add_vertex(intersection_points[0]);
                surface_tool->add_vertex(intersection_points[1]);
                continue;
            }

            // ---------------------------------------------------------
            // INTERSECTION CASE 2: TWO points above the plane
            // ---------------------------------------------------------
            if (vertices_above_plane.size() == 2) {
                int va = vertices_above_plane[0];
                int vb = vertices_above_plane[1];

                // index of point below plane
                int index_remaining = -1;

                // match GDScript cyclic ordering logic exactly
                if (va != face_vertices[1] && vb != face_vertices[1]) {
                    // reverse order
                    int tmp = va;
                    va = vb;
                    vb = tmp;
                    index_remaining = 1;
                }
                else if (va != face_vertices[0] && vb != face_vertices[0]) {
                    index_remaining = 0;
                }
                else {
                    index_remaining = 2;
                }

                Vector3 i_after;
                plane.intersects_segment(
                    data_tool->get_vertex(vb),
                    data_tool->get_vertex(face_vertices[index_remaining]),
                    &i_after
                );
                intersection_points.push_back(i_after);
                coplanar_vertices.push_back(i_after);

                Vector3 i_before;
                plane.intersects_segment(
                    data_tool->get_vertex(va),
                    data_tool->get_vertex(face_vertices[index_remaining]),
                    &i_before
                );
                intersection_points.push_back(i_before);
                coplanar_vertices.push_back(i_before);

                // find shortest 'cross-length' to make 2 triangles from 4 points
                int index_shortest = 0;
                float dist_0 = data_tool->get_vertex(va).distance_to(intersection_points[0]);
                float dist_1 = data_tool->get_vertex(vb).distance_to(intersection_points[1]);
                if (dist_1 > dist_0) {
                    index_shortest = 1;
                }

                // TRIANGLE 1
                surface_tool->add_vertex(data_tool->get_vertex(va));
                surface_tool->add_vertex(data_tool->get_vertex(vb));
                surface_tool->add_vertex(intersection_points[index_shortest]);

                // TRIANGLE 2
                surface_tool->add_vertex(intersection_points[0]);
                surface_tool->add_vertex(intersection_points[1]);
                surface_tool->add_vertex(data_tool->get_vertex(
                    index_shortest == 0 ? va : vb
                ));

                continue;
            }
        } // end face loop

        // ---------------------------------------------------------
        // DEFINE NEW FACE; FIND CENTROID; APPEND TRIANGLES
        // ---------------------------------------------------------
        Vector3 centroid(0,0,0);
        for (int i = 0; i < coplanar_vertices.size(); i++) {
            centroid += coplanar_vertices[i];
        }
        centroid /= coplanar_vertices.size();

        surface_tool->set_color(inner_color);

        for (int i = 0; i < coplanar_vertices.size() - 1; i++) {
            if (i % 2 != 0) continue;
            surface_tool->add_vertex(coplanar_vertices[i + 1]);
            surface_tool->add_vertex(coplanar_vertices[i]);
            surface_tool->add_vertex(centroid);
        }

        if (side == 0)
            surface_tool_a = surface_tool;
        else
            surface_tool_b = surface_tool;
    } // end side loop

    // finalize new meshes
    surface_tool_a->index();
    surface_tool_a->generate_normals();
    surface_tool_b->index();
    surface_tool_b->generate_normals();

    // assign to child VSTNodes
    MeshInstance3D *mesh_instance_above = memnew(MeshInstance3D);
    mesh_instance_above->set_mesh(surface_tool_a->commit());
    vst_node->_left = memnew(VSTNode(mesh_instance_above, vst_node->_level + 1, Laterality::LEFT));

    MeshInstance3D *mesh_instance_below = memnew(MeshInstance3D);
    mesh_instance_below->set_mesh(surface_tool_b->commit());
    vst_node->_right = memnew(VSTNode(mesh_instance_below, vst_node->_level + 1, Laterality::RIGHT));

    return true;
}

void DestronoiNode::_cleanup() {

    for (RigidBody3D *body : new_bodies) {
        MeshInstance3D *mi = cast_to<MeshInstance3D>(body->get_child(0));
        if (mi && mi->get_mesh().is_valid()) {
            mi->set_mesh(Ref<Mesh>());   // releases RID
        } else {
    		UtilityFunctions::print("[Destronoi] no mesh found to free");
        }

        CollisionShape3D *col = cast_to<CollisionShape3D>(body->get_child(1));
        if (col && col->get_shape().is_valid()) {
            col->set_shape(Ref<Shape3D>());  // releases RID
        } else {
    		UtilityFunctions::print("[Destronoi] no collision shape found to free");
        }

        body->queue_free();
    }
    queue_free();
}


void DestronoiNode::destroy(int left_val, int right_val, float combust_velocity) {
    Node3D *base_object = Object::cast_to<Node3D>(get_parent());
    if (!base_object || !_root) {
        return;
    }

    // collect leaves to use
    std::vector<VSTNode *> vst_leaves;
    _root->get_left_leaf_nodes(vst_leaves, left_val);
    _root->get_right_leaf_nodes(vst_leaves, right_val);

    float sum_mass = 0.0f;

    Transform3D base_xform = base_object->get_global_transform();

    for (size_t idx = 0; idx < vst_leaves.size(); idx++) {
        VSTNode *leaf = vst_leaves[idx];

        // new rigidbody
        RigidBody3D *new_body = memnew(RigidBody3D);
        new_body->set_name("VFragment_" + String::num_int64(idx));

        // position = base_object.transform.origin
        // new_body->set_global_position(base_xform.origin);

        // bring the mesh instance over
        MeshInstance3D *new_mesh_instance = leaf->_mesh_instance;
        new_mesh_instance->set_name("MeshInstance3D");
        new_body->add_child(new_mesh_instance);

        // collision
        CollisionShape3D *col = memnew(CollisionShape3D);
        col->set_name("CollisionShape3D");
        col->set_shape(new_mesh_instance->get_mesh()->create_convex_shape(false, false));
        new_body->add_child(col);

        // velocity dir = AABB center - base position   (NOTE: this is *local*, matches gdscript bug-for-bug)
        AABB aabb = new_mesh_instance->get_mesh()->get_aabb();
        Vector3 velocity_dir = aabb.get_center() - base_xform.origin;
        velocity_dir = velocity_dir.normalized();

        // mass = max(volume, 0.1)
        float vol = aabb.get_volume();
        float mass = vol < 0.1f ? 0.1f : vol;
        new_body->set_mass(mass);
        sum_mass += mass;

        // combustion outward velocity
        if (combust_velocity != 0.0f) {
            std::vector<Vector3> endpoints;
            Vector3 estim_dir(0, 0, 0);

            for (int i = 0; i < 8; i++) {
                Vector3 ep = new_mesh_instance->get_mesh()->get_aabb().get_endpoint(i);
                ep = ep.normalized();

                float d = ep.dot(base_xform.origin.normalized());
                if (Math::abs(d) > 0.0f) {
                    endpoints.push_back(ep);
                }
            }

            for (const Vector3 &ep : endpoints) {
                estim_dir += ep;
            }

            if (!endpoints.empty()) {
                estim_dir /= (float)endpoints.size();
            }

            estim_dir = estim_dir.normalized();
            new_body->set_axis_velocity(estim_dir * combust_velocity);
        }

        new_bodies.push_back(new_body);
    }

    // scale masses
    float base_mass = 1.0f;
    if (Object::cast_to<RigidBody3D>(base_object)) {
        base_mass = Object::cast_to<RigidBody3D>(base_object)->get_mass();
    }

    for (RigidBody3D *body : new_bodies) {
        float scaled = body->get_mass() * (base_mass / sum_mass);
        body->set_mass(scaled);
        base_object->get_parent()->add_child(body);
        body->set_global_position(base_xform.origin);
    }

    // delete vst tree, original rigidbody
    _root->free_tree();
    memdelete(_root);
    _root = nullptr;

    reparent(base_object->get_parent());
    base_object->queue_free();

    Ref<SceneTreeTimer> tm = get_tree()->create_timer(visible_seconds, true);
    tm->connect("timeout", Callable(this, "_cleanup"));
}

