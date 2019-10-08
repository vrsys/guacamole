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
#include <gua/physics/ConvexHullShape.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>

#include <gua/node/TriMeshNode.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/TriMeshLoader.hpp>

// external headers
#include <btBulletDynamicsCommon.h>
#include <LinearMath/btConvexHull.h>
#include <LinearMath/btGeometryUtil.h>

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

ConvexHullShape::ConvexHullShape() : CollisionShape(true, true, true) { shape_ = new btConvexHullShape(); }

////////////////////////////////////////////////////////////////////////////////

ConvexHullShape::~ConvexHullShape() { delete shape_; }

////////////////////////////////////////////////////////////////////////////////

void ConvexHullShape::add_point(const math::vec3& point) { shape_->addPoint(math::vec3_to_btVector3(point)); }

////////////////////////////////////////////////////////////////////////////////

void ConvexHullShape::build_from_geometry(const std::vector<std::string>& geometry_list, bool compensate_collision_margin)
{
    btAlignedObjectArray<btVector3> vertices;

    for(auto const& geom_name : geometry_list)
    {
        std::shared_ptr<TriMeshRessource> mesh = std::dynamic_pointer_cast<TriMeshRessource>(gua::GeometryDatabase::instance()->lookup(geom_name));
        if(mesh)
            for(unsigned i(0); i < mesh->num_vertices(); ++i)
            {
                btVector3 v = math::vec3_to_btVector3(mesh->get_vertex(i));
                vertices.push_back(v);
            }
    }

    delete shape_;
    shape_ = new btConvexHullShape(&(vertices[0].getX()), vertices.size());

    if(compensate_collision_margin)
    {
        // workaround
        shape_->setMargin(0.012f);

        // this code works strange. Converting plain equations back to
        // vertices produces too many vertices and bullet becomes very slow.
        /*
            HullDesc hd;
            hd.mFlags = QF_TRIANGLES;
            hd.mVcount = static_cast<unsigned int>(vertices.size());
            hd.mVertices = &vertices[0];
            hd.mVertexStride = sizeof(btVector3);
            HullLibrary hl;
            HullResult hr;
            if (hl.CreateConvexHull (hd, hr) == QE_FAIL)
                return;

            vertices.clear();
            for (unsigned int i = 0; i < hr.mNumOutputVertices; i++)
                vertices.push_back(hr.m_OutputVertices[i]);

            hl.ReleaseResult (hr);

            btAlignedObjectArray<btVector3> planes, planes_s;
            btGeometryUtil::getPlaneEquationsFromVertices(vertices, planes);

            for (int i = 0 ; i < planes.size(); ++i) {
                btVector3 plane = planes[i];
                plane[3] += shape_->getMargin();
                planes_s.push_back(plane);
            }
            vertices.clear();
            btGeometryUtil::getVerticesFromPlaneEquations(planes_s, vertices);

            delete shape_;
            shape_ = new
        btConvexHullShape(&(vertices[0].getX()),vertices.size());
            */
    }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void ConvexHullShape::construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform) { bullet_shape->addChildShape(base_transform, shape_); }

////////////////////////////////////////////////////////////////////////////////

/* virtual */ btCollisionShape* ConvexHullShape::construct_static() { return shape_; }

////////////////////////////////////////////////////////////////////////////////

/* static */ ConvexHullShape* ConvexHullShape::FromGeometry(const std::vector<std::string>& geometry_list, bool compensate_collision_margin)
{
    ConvexHullShape* shape = new ConvexHullShape();
    shape->build_from_geometry(geometry_list, compensate_collision_margin);
    return shape;
}

////////////////////////////////////////////////////////////////////////////////

/* static */ ConvexHullShape* ConvexHullShape::FromGeometryFile(const std::string& file_name, bool compensate_collision_margin, unsigned flags)
{
    ConvexHullShape* shape = new ConvexHullShape();

    TriMeshLoader factory;
    auto node(factory.create_geometry_from_file("", file_name, std::make_shared<Material>(), flags));
    if(node)
    {
        std::vector<std::string> geom_list;

        if(node->has_children())
        {
            for(auto const& n : node->get_children())
            {
                auto gnode = std::dynamic_pointer_cast<node::TriMeshNode>(n);
                if(gnode)
                {
                    geom_list.push_back(gnode->get_geometry_description());
                }
            }
        }
        else
        {
            auto gnode = std::dynamic_pointer_cast<node::TriMeshNode>(node);
            if(gnode)
            {
                geom_list.push_back(gnode->get_geometry_description());
            }
        }

        shape->build_from_geometry(geom_list, compensate_collision_margin);
    }

    return shape;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
