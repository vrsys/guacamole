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
#include <gua/physics/TriangleMeshShape.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/physics/PhysicsUtils.hpp>

// external headers
#include <stdexcept>

#include <HACD/hacdHACD.h>

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

TriangleMeshShape::~TriangleMeshShape()
{
    if(concave_shape_)
        delete concave_shape_;
    if(concave_tri_mesh_)
        delete concave_tri_mesh_;
    for(int i(0); i < convex_shapes_.size(); ++i)
    {
        delete convex_shapes_[i];
    }
}

////////////////////////////////////////////////////////////////////////////////

void TriangleMeshShape::build_from_geometry_static(const std::vector<std::string>& geometry_list)
{
    if(concave_shape_)
        delete concave_shape_;
    if(concave_tri_mesh_)
        delete concave_tri_mesh_;

    // TODO: Ideally, btBvhTriangleMeshShape construction should be done
    //      using btStridingMeshInterface. Example from Ogre3D:
    //      http://www.ogre3d.org/tikiwiki/BulletMeshStrider
    concave_tri_mesh_ = new btTriangleMesh();

    for(auto const& geom_name : geometry_list)
    {
        std::shared_ptr<TriMeshRessource> m = std::dynamic_pointer_cast<TriMeshRessource>(gua::GeometryDatabase::instance()->lookup(geom_name));
        if(m)
            for(unsigned i(0); i < m->num_faces(); ++i)
            {
                auto face = m->get_face(i);
                // no polygon triangulation
                assert(face.size() == 3);
                concave_tri_mesh_->addTriangle(math::vec3_to_btVector3(m->get_vertex(face[0])), math::vec3_to_btVector3(m->get_vertex(face[1])), math::vec3_to_btVector3(m->get_vertex(face[2])));
            }
    }
    concave_shape_ = new btBvhTriangleMeshShape(concave_tri_mesh_, true);
    concave_shape_->setLocalScaling(math::vec3_to_btVector3(scaling_));
    is_static_shape_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void TriangleMeshShape::build_from_geometry_dynamic(const std::vector<std::string>& geometry_list)
{
    for(auto const& geom_name : geometry_list)
    {
        std::shared_ptr<TriMeshRessource> m = std::dynamic_pointer_cast<TriMeshRessource>(gua::GeometryDatabase::instance()->lookup(geom_name));

        if(m)
            decompose_to_convex(m /*, "HACD_"+geom_name+".wrl"*/);
    }

    is_dynamic_shape_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void TriangleMeshShape::set_scaling(const math::vec3& scaling)
{
    scaling_ = scaling;
    if(is_static_shape_ && concave_shape_)
        concave_shape_->setLocalScaling(math::vec3_to_btVector3(scaling));
    if(is_dynamic_shape_)
        for(int i(0); i < convex_shapes_.size(); ++i)
            convex_shapes_[i]->setLocalScaling(math::vec3_to_btVector3(scaling));
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void TriangleMeshShape::construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform)
{
    if(is_dynamic_shape_)
    {
        for(int i(0); i < convex_shapes_.size(); ++i)
            bullet_shape->addChildShape(base_transform, convex_shapes_[i]);
    }
    else
        throw std::runtime_error("This shape hasn't been preconstructed to be "
                                 "used with dynamic rigid bodies. Please use "
                                 "build_from_geometry_dynamic() firstly.");
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ btCollisionShape* TriangleMeshShape::construct_static()
{
    if(is_static_shape_)
        return concave_shape_;
    else
        throw std::runtime_error("This shape hasn't been preconstructed to be "
                                 "used with static rigid bodies. Please use "
                                 "build_from_geometry_static() firstly.");
}

////////////////////////////////////////////////////////////////////////////////

/* static */ TriangleMeshShape* TriangleMeshShape::FromGeometry(const std::vector<std::string>& geometry_list, bool build_static, bool build_dynamic)
{
    TriangleMeshShape* shape = new TriangleMeshShape();
    if(build_static)
        shape->build_from_geometry_static(geometry_list);
    if(build_dynamic)
        shape->build_from_geometry_dynamic(geometry_list);
    return shape;
}

////////////////////////////////////////////////////////////////////////////////

/* static */ TriangleMeshShape* TriangleMeshShape::FromGeometryFile(const std::string& file_name, bool build_static, bool build_dynamic, unsigned flags)
{
    TriangleMeshShape* shape = new TriangleMeshShape();

    TriMeshLoader factory;
    auto node(factory.create_geometry_from_file("", file_name, std::make_shared<Material>(), flags));
    if(node)
    {
        std::vector<std::string> geom_list;

        std::function<void(std::shared_ptr<node::Node> const&)> add_all_geometries;

        add_all_geometries = [&](std::shared_ptr<node::Node> const& node) {
            auto gnode = std::dynamic_pointer_cast<node::TriMeshNode>(node);
            if(gnode)
            {
                geom_list.push_back(gnode->get_geometry_description());
            }

            for(auto const& n : node->get_children())
            {
                add_all_geometries(n);
            }
        };

        add_all_geometries(node);

        if(build_static)
            shape->build_from_geometry_static(geom_list);
        if(build_dynamic)
        {
            shape->build_from_geometry_dynamic(geom_list);
        }
    }

    return shape;
}

////////////////////////////////////////////////////////////////////////////////

/* static */ long TriangleMeshShape::get_point_or_add(std::vector<HACD::Vec3<double>>& points, const math::vec3& v)
{
    HACD::Vec3<HACD::Real> vertex(v.x, v.y, v.z);
    auto it = std::find_if(points.begin(), points.end(), [&vertex](const HACD::Vec3<HACD::Real>& n) -> bool { return n.X() == vertex.X() && n.Y() == vertex.Y() && n.Z() == vertex.Z(); });

    if(it != points.end())
    {
        return std::distance(points.begin(), it);
    }
    else
    {
        points.push_back(vertex);
        return points.size() - 1;
    }
}

////////////////////////////////////////////////////////////////////////////////

void TriangleMeshShape::decompose_to_convex(std::shared_ptr<TriMeshRessource> const& mesh, std::string const& file_name)
{
    std::vector<HACD::Vec3<HACD::Real>> points;
    std::vector<HACD::Vec3<long>> triangles;

    // Fill vertex and triangle arrays with merging duplicate vertices
    for(unsigned int i(0); i < mesh->num_faces(); ++i)
    {
        auto face = mesh->get_face(i);
        // No polygon triangulation
        assert(face.size() == 3);

        HACD::Vec3<long> triangle(get_point_or_add(points, mesh->get_vertex(face[0])), get_point_or_add(points, mesh->get_vertex(face[1])), get_point_or_add(points, mesh->get_vertex(face[2])));
        triangles.push_back(triangle);
    }

    HACD::HACD hacd;
    hacd.SetPoints(points.data());
    hacd.SetNPoints(points.size());
    hacd.SetTriangles(triangles.data());
    hacd.SetNTriangles(triangles.size());
    hacd.SetCompacityWeight(0.1);
    hacd.SetVolumeWeight(0.0);

    hacd.SetNClusters(hacd_min_clusters);
    hacd.SetNVerticesPerCH(hacd_max_ch_vertices);
    hacd.SetConcavity(hacd_concavity);
    hacd.SetAddExtraDistPoints(hacd_add_extra_dist_points);
    hacd.SetAddNeighboursDistPoints(hacd_add_neighbours_dist_points);
    hacd.SetAddFacesPoints(hacd_add_faces_points);

    hacd.Compute();
    size_t cluster_num = hacd.GetNClusters();

    if(file_name.size())
        hacd.Save(file_name.c_str(), false);
    std::cout << "HACD: Decomposed in " << cluster_num << " convex hulls." << std::endl;

    // create a convex hull
    for(size_t c(0); c < cluster_num; ++c)
    {
        size_t points_num = hacd.GetNPointsCH(c);

        HACD::Vec3<HACD::Real>* points_ch = new HACD::Vec3<HACD::Real>[points_num];
        HACD::Vec3<long>* triangles_ch = new HACD::Vec3<long>[hacd.GetNTrianglesCH(c)];
        hacd.GetCH(c, points_ch, triangles_ch);

        btConvexHullShape* shape = new btConvexHullShape();
        for(size_t v(0); v < points_num; ++v)
            shape->addPoint(btVector3(points_ch[v].X(), points_ch[v].Y(), points_ch[v].Z()));
        delete[] points_ch;
        delete[] triangles_ch;

        // reduce collision margin. It may slow down the simulation.
        // Normally, plane equation should be used instead this.
        shape->setMargin(0.012f);
        shape->setLocalScaling(math::vec3_to_btVector3(scaling_));
        convex_shapes_.push_back(shape);
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
