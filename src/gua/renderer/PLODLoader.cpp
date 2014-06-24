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
#include <pbr/ren/lod_point_cloud.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/policy.h>

// guacamole headers
#include <gua/guacamole.hpp>
#include <gua/renderer/PLODLoader.hpp>
#include <gua/renderer/PLODRessource.hpp>
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/PLODNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/renderer/PLODRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>
#include <gua/generated/R.inl>
#include <iostream>

namespace gua {

unsigned PLODLoader::model_counter_ = 0;
  /////////////////////////////////////////////////////////////////////////////

  PLODLoader::PLODLoader() 
    : GeometryLoader(), 
      _supported_file_extensions(),
      node_counter_(0) 
  {
    _supported_file_extensions.insert("kdn");    

   if (!MaterialDatabase::instance()->is_supported("gua_pbr"))
   {
       create_resource_material("gua_pbr",
       Resources::materials_gua_pbr_gsd,
       Resources::materials_gua_pbr_gmd);
   }

  }


  /////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<Node> PLODLoader::create_geometry_from_file(std::string const& node_name,
                                                              std::string const& file_name) {
   
     std::string model_name("type=file&file=" + file_name);
   
     try{
       auto node(std::make_shared<PLODNode>(model_name));
       node->set_filename(model_name);
       node->set_material("gua_pbr");

       pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
  
       // load point cloud
       pbr::model_t model_id = database->AddModel(file_name, model_name);
       const pbr::ren::LodPointCloud* point_cloud = database->GetModel(model_id);

       GeometryDatabase::instance()->add(model_name, std::make_shared<PLODRessource>(point_cloud));
   
       ++model_counter_;
   
       return node;
     }
     catch (std::exception &e){
       Logger::LOG_WARNING << "Warning: " << e.what() << " : Failed to load LOD Pointcloud object " << file_name.c_str() << std::endl;
        return nullptr;
     }

  }


  ////////////////////////////////////////////////////////////////////////////////

  bool PLODLoader::is_supported(std::string const& file_name) const 
  {
    std::vector<std::string> filename_decomposition =
      gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty()
      ? false
      : _supported_file_extensions.count(filename_decomposition.back()) > 0;
  }

  void PLODLoader::set_upload_budget_in_mb(const size_t upload_budget)
  {
    pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
    policy->set_upload_budget_in_mb(upload_budget);
  }

  void PLODLoader::set_render_budget_in_mb(const size_t render_budget)
  {
    pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
    policy->set_render_budget_in_mb(render_budget);
  }

  void PLODLoader::set_out_of_core_budget_in_mb(const size_t out_of_core_budget)
  {
    pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
    policy->set_out_of_core_budget_in_mb(out_of_core_budget);
  }

const size_t PLODLoader::get_upload_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->upload_budget_in_mb();
}

const size_t PLODLoader::get_render_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->render_budget_in_mb();
}

const size_t PLODLoader::get_out_of_core_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->out_of_core_budget_in_mb();
}

}
