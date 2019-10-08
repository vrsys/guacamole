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
#include <gua/renderer/detail/NURBSData.hpp>

// guacamole headers
#include <gpucast/core/trimdomain_serializer_contour_map_kd.hpp>

// external headers
#include <algorithm>
#include <iterator>
#include <limits>

namespace gua
{
NURBSData::NURBSData(std::shared_ptr<gpucast::beziersurfaceobject> const& o, unsigned pre_subdivision_u, unsigned pre_subdivision_v, unsigned trim_texture)
    : object(o), tess_patch_data(), tess_index_data(), tess_parametric_data(), tess_attribute_data()
{
    if(!object->initialized())
    {
        object->init(pre_subdivision_u, pre_subdivision_v, trim_texture);
    }

    unsigned int patch_id(tess_attribute_data.size());

    // serialize trim domain
    typedef gpucast::math::domain::contour_map_kd<double> partition_type;
    std::unordered_map<gpucast::trimdomain_serializer_contour_map_kd::curve_ptr, unsigned> referenced_curves;
    std::unordered_map<gpucast::trimdomain_serializer_contour_map_kd::trimdomain_ptr, unsigned> referenced_domains;
    std::unordered_map<partition_type::contour_segment_ptr, unsigned> referenced_segments;

    gpucast::trimdomain_serializer_contour_map_kd serializer;
    gpucast::trim_kd_serialization serialization;

    auto uint_to_float = [](unsigned const& i) { return *((float*)(&i)); };

    //  serialize patch data
    for(auto it = object->begin(); it != object->end(); ++it, ++patch_id)
    {
        auto _p0 = (*it)->points().begin();
        tess_patch_data.push_back(scm::math::vec4f((*_p0)[0], (*_p0)[1], (*_p0)[2], uint_to_float(patch_id)));
        tess_patch_data.push_back(scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));

        auto _p1 = _p0 + (*it)->points().width() - 1;
        tess_patch_data.push_back(scm::math::vec4f((*_p1)[0], (*_p1)[1], (*_p1)[2], uint_to_float(patch_id)));
        tess_patch_data.push_back(scm::math::vec4f(1.0f, 0.0f, 0.0f, 0.0f));

        auto _p2 = (*it)->points().end() - (*it)->points().width();
        tess_patch_data.push_back(scm::math::vec4f((*_p2)[0], (*_p2)[1], (*_p2)[2], uint_to_float(patch_id)));
        tess_patch_data.push_back(scm::math::vec4f(0.0f, 1.0f, 0.0f, 0.0f));

        auto _p3 = (*it)->points().end() - 1;
        tess_patch_data.push_back(scm::math::vec4f((*_p3)[0], (*_p3)[1], (*_p3)[2], uint_to_float(patch_id)));
        tess_patch_data.push_back(scm::math::vec4f(1.0f, 1.0f, 0.0f, 0.0f));

        auto _v01 = (_p1->as_euclidian()) - (_p0->as_euclidian());
        auto _v13 = (_p3->as_euclidian()) - (_p1->as_euclidian());
        auto _v23 = (_p3->as_euclidian()) - (_p2->as_euclidian());
        auto _v02 = (_p2->as_euclidian()) - (_p0->as_euclidian());

        scm::math::vec4f edge_dist(0.0, 0.0, 0.0, 0.0);

        tess_index_data.push_back(patch_id * 4 + 0);
        tess_index_data.push_back(patch_id * 4 + 1);
        tess_index_data.push_back(patch_id * 4 + 3);
        tess_index_data.push_back(patch_id * 4 + 2);

        // serialize trim domain
        std::size_t trim_id = serializer.serialize((*it)->domain(), gpucast::kd_split_strategy::sah, serialization, 0, 0);

        // gather per patch data
        per_patch_data p;
        p.surface_offset = tess_parametric_data.size();
        p.order_u = (*it)->order_u();
        p.order_v = (*it)->order_v();

        p.trim_type = (*it)->trimtype();
        p.trim_id = trim_id;

        auto obb_id = object->serialized_obb_base_indices().find((*it));
        if(obb_id != object->serialized_obb_base_indices().end())
        {
            p.obb_id = obb_id->second;
        }

        p.nurbs_domain = scm::math::vec4f((*it)->bezierdomain().min[0], (*it)->bezierdomain().min[1], (*it)->bezierdomain().max[0], (*it)->bezierdomain().max[1]);
        p.bbox_min = scm::math::vec4f((*it)->bbox().min[0], (*it)->bbox().min[1], (*it)->bbox().min[2], (float)0);
        p.bbox_max = scm::math::vec4f((*it)->bbox().max[0], (*it)->bbox().max[1], (*it)->bbox().max[2], (float)0);
        // p.distance = math::vec4f(std::fabs(edge_dist[0]), std::fabs(edge_dist[1]), std::fabs(edge_dist[2]), std::fabs(edge_dist[3]));

        p.edge_length_u = (*it)->max_edge_length_u();
        p.edge_length_v = (*it)->max_edge_length_v();
        p.ratio_uv = p.edge_length_u / p.edge_length_v;
        p.curvature = (*it)->curvature();

        tess_attribute_data.push_back(p);

        // copy patch control points
        int current_size = tess_parametric_data.size();
        tess_parametric_data.resize(current_size + (*it)->points().size());

        auto serialize_homogenous_points = [](gpucast::math::point3d const& p) {
            auto ph = p.as_homogenous();
            return scm::math::vec4f(ph[0], ph[1], ph[2], ph[3]);
        };
        std::transform((*it)->points().begin(), (*it)->points().end(), tess_parametric_data.begin() + current_size, serialize_homogenous_points);
    }
}

} // namespace gua
