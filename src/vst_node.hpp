#pragma once

#include "destronoi.hpp"
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <vector>

namespace godot {

enum class Laterality {
    NONE = 0,
    LEFT,
    RIGHT
};

class DestronoiNode;

class VSTNode {
    friend class DestronoiNode;
public:
    VSTNode() = default;

    VSTNode(MeshInstance3D *mesh_instance,
            int level = 0,
            Laterality lat = Laterality::NONE)
        : _mesh_instance(mesh_instance),
          _level(level),
          _laterality(lat) {}

    MeshInstance3D *get_mesh() const { return _mesh_instance; }

    int get_site_count() const { return _sites.size(); }

    void get_leaf_nodes(std::vector<VSTNode *> &out) {
        if (!_left && !_right) {
            out.push_back(this);
            return;
        }
        if (_left)  _left->get_leaf_nodes(out);
        if (_right) _right->get_leaf_nodes(out);
    }

    void get_right_leaf_nodes(std::vector<VSTNode *> &out, int lim, int level = 0) {
        if ((!_left && !_right) || level == lim) {
            out.push_back(this);
            return;
        }
        if (_left && level > 0)
            _left->get_right_leaf_nodes(out, lim, level + 1);
        if (_right)
            _right->get_right_leaf_nodes(out, lim, level + 1);
    }

    void get_left_leaf_nodes(std::vector<VSTNode *> &out, int lim, int level = 0) {
        if ((!_left && !_right) || level == lim) {
            out.push_back(this);
            return;
        }
        if (_left)
            _left->get_left_leaf_nodes(out, lim, level + 1);
        if (_right && level > 0)
            _right->get_left_leaf_nodes(out, lim, level + 1);
    }

    // PackedVector3Array get_sites_gd() const {
    //     PackedVector3Array arr;
    //     arr.resize(_sites.size());
    //     for (size_t i = 0; i < _sites.size(); ++i)
    //         arr[i] = _sites[i];
    //     return arr;
    // }
    void free_tree() {
        // delete children first
        if (_left) {
            _left->free_tree();
            memdelete(_left);
            _left = nullptr;
        }

        if (_right) {
            _right->free_tree();
            memdelete(_right);
            _right = nullptr;
        }

        // DO NOT free _mesh_instance here.
        // Godot owns it through the scene tree.
        // When the Destronoi node is queue_free()'d, Godot frees them.
    }


private:
    MeshInstance3D *_mesh_instance = nullptr;
    std::vector<Vector3> _sites;

    VSTNode *_left  = nullptr;
    VSTNode *_right = nullptr;

    int _level = 0;
    Laterality _laterality = Laterality::NONE;
};

}
