#pragma once

#include "godot_cpp/classes/material.hpp"
#include "godot_cpp/core/memory.hpp"
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/mesh.hpp>
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

    VSTNode(Ref<Mesh> mesh_ref,
        Ref<Material> mat_ref,
        int level = 0,
        Laterality lat = Laterality::NONE) :
        _level(level),
        _laterality(lat)
    {
        if (mesh_ref.is_valid()) {
            _mesh_ref = mesh_ref->duplicate();
        }

        if (mat_ref.is_valid()) {
            _mat_ref = mat_ref->duplicate();
        }
    }


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

    void free_tree() {
        _mesh_ref.unref();
        _mat_ref.unref();
        
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
    }


    Ref<Mesh> _mesh_ref;
    Ref<Material> _mat_ref;
    std::vector<Vector3> _sites;

    VSTNode *_left  = nullptr;
    VSTNode *_right = nullptr;

    int _level = 0;
    Laterality _laterality = Laterality::NONE;
};

}
