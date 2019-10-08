/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/Auxi.hpp>
#include <gua/renderer/OctreeNode.hpp>

// guacamole headers
#include <gua/utils.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/node/PLodNode.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/renderer/LodResource.hpp>

// external headers
#include <lamure/ren/dataset.h>

#include <lamure/prov/auxi.h>
#include <lamure/prov/octree.h>

namespace gua
{
Auxi::Auxi() { _auxi = std::make_shared<lamure::prov::auxi>(); }

void Auxi::load_aux_file(std::string const& filename) { _auxi->load_aux_file(filename); }

const std::string Auxi::get_filename() const { return _auxi->get_filename(); }

const uint32_t Auxi::get_num_views() const { return _auxi->get_num_views(); }

const uint64_t Auxi::get_num_sparse_points() const { return _auxi->get_num_sparse_points(); }

const uint32_t Auxi::get_num_atlas_tiles() const { return _auxi->get_num_atlas_tiles(); }

const uint64_t Auxi::get_num_nodes() const { return _auxi->get_num_nodes(); }

const uint64_t Auxi::get_octree_query(const scm::math::vec3f& _pos) const { return _auxi->get_octree_query(_pos); }

std::shared_ptr<OctreeNode> Auxi::get_octree_node(uint64_t node_id) const
{
    const auto& on = _auxi->get_octree_node(node_id);

    gua::OctreeNode new_octree_node(on.get_idx(), on.get_child_mask(), on.get_child_idx(), on.get_min(), on.get_max(), on.get_fotos());

    return std::make_shared<OctreeNode>(new_octree_node);
}

std::shared_ptr<Auxi::view> Auxi::get_view(uint32_t id) const
{
    const auto& v = _auxi->get_view(id);

    Auxi::view new_view(v.camera_id_, v.position_, v.transform_, v.focal_value_x_, v.focal_value_y_, v.center_x_, v.center_y_, v.distortion_, v.image_width_, v.image_height_, v.atlas_tile_id_, v.image_file_);

    return std::make_shared<Auxi::view>(new_view);
}

std::shared_ptr<Auxi::atlas_tile> Auxi::get_atlas_tile(uint32_t id) const
{
    const auto& at = _auxi->get_atlas_tile(id);

    Auxi::atlas_tile new_atlas_tile(at.atlas_tile_id_, at.x_, at.y_, at.width_, at.height_);

    return std::make_shared<Auxi::atlas_tile>(new_atlas_tile);
}

std::shared_ptr<Auxi::sparse_point> Auxi::get_sparse_point(uint64_t id) const
{
    const auto& sp = _auxi->get_sparse_point(id);

    std::vector<Auxi::feature> new_features;
    for(auto const& f : sp.features_)
    {
        Auxi:feature new_feature(f.camera_id_, f.using_count_, f.coords_, f.error_);
        new_features.push_back(new_feature);
    }

    Auxi::sparse_point new_sparse_point(sp.pos_, sp.r_, sp.g_, sp.b_, sp.a_, new_features);

    return std::make_shared<Auxi::sparse_point>(new_sparse_point);
}

std::shared_ptr<Auxi::atlas> Auxi::get_atlas() const
{
    const auto& a = _auxi->get_atlas();

    Auxi::atlas new_atlas(a.num_atlas_tiles_, a.atlas_width_, a.atlas_height_, a.rotated_);

    return std::make_shared<Auxi::atlas>(new_atlas);
}

} // namespace gua