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

#include <lamure/prov/aux.h>

namespace gua {


    OctreeNode::OctreeNode()
        : idx_(0)
        , child_mask_(0)   //char r_, g_, b_, child_mask_
        , child_idx_(0)    //idx of first child
        , min_(std::numeric_limits<float>::max())
        , max_(std::numeric_limits<float>::lowest())
        , fotos_()
    { }

    OctreeNode::OctreeNode(uint64_t _idx, uint32_t _child_mask, uint32_t _child_idx,
                           const scm::math::vec3f& _min, const scm::math::vec3f& _max,
                           const std::set<uint32_t>& _fotos)
        : idx_(_idx)
        , child_mask_(_child_mask)
        , child_idx_(_child_idx)
        , min_(_min)
        , max_(_max)
        , fotos_(_fotos)
    { }


    void
    OctreeNode::test_wrapping() const {
        std::cout << "The wrapped function in gua has been called!" << std::endl;
    }

    uint64_t
    OctreeNode::get_idx() const {
        return idx_;
    }

    uint32_t
    OctreeNode::get_child_mask() const {
        return child_mask_;
    }

    uint32_t
    OctreeNode::get_child_idx() const {
        return child_idx_;
    }

    const scm::math::vec3f&
    OctreeNode::get_min() const {
        return min_;
    }

    const scm::math::vec3f&
    OctreeNode::get_max() const {
        return max_;
    }

    const std::set<uint32_t>&
    OctreeNode::get_fotos() const {
        return fotos_;
    }


    uint32_t
    OctreeNode::get_fotos_size() const {
        return fotos_.size();
    }

    uint32_t
    OctreeNode::get_foto_by_id(uint32_t id) const {
        if (id > 0 && id < fotos_.size()){
            return *std::next(fotos_.begin(), id);
        }
    }


} // namespace gua

