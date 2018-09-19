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
#include <gua/renderer/Aux.hpp>

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





Aux::Aux() {
  _aux = std::make_shared<lamure::prov::aux>();
}

void Aux::test_wrapping() const {
  std::cout << "The wrapped function in gua has been called!" << std::endl;
  _aux->test_wrapping();
  
}

void Aux::load_aux_file(std::string const& filename) {
  _aux->load_aux_file(filename);
}

const std::string Aux::get_filename() const {
	return _aux->get_filename(); 
}

const uint32_t Aux::get_num_views() const {
	return _aux->get_num_views(); 
}

const uint64_t Aux::get_num_sparse_points() const {
  return _aux->get_num_sparse_points(); 
}
const uint32_t Aux::get_num_atlas_tiles() const { 
	return _aux->get_num_atlas_tiles(); 
}



std::shared_ptr<Aux::view> Aux::get_view(uint32_t id) const {
	const auto& v = _aux->get_view(id);

	Aux::view new_view(
		v.camera_id_,
		v.position_,
		v.transform_, 
    v.focal_length_, 
    v.distortion_, 
    v.image_width_,
    v.image_height_,
    v.atlas_tile_id_,
    v.image_file_
  );

	return std::make_shared<Aux::view>(new_view);
}


std::shared_ptr<Aux::atlas_tile> Aux::get_atlas_tile(uint32_t id) const {
	const auto& at = _aux->get_atlas_tile(id);

	Aux::atlas_tile new_atlas_tile(
		at.atlas_tile_id_, 
		at.x_, 
		at.y_, 
		at.width_, 
		at.height_
	);

	return std::make_shared<Aux::atlas_tile>(new_atlas_tile);
}


std::shared_ptr<Aux::sparse_point> Aux::get_sparse_point(uint64_t id) const {


	const auto& sp = _aux->get_sparse_point(id);
	std::vector<Aux::feature> new_features;
	for(auto const& f: sp.features_){
    Aux::feature new_feature(
      f.camera_id_,
      f.using_count_,
      f.coords_,
      f.error_
    );
    new_features.push_back(new_feature);
	}
	
	Aux::sparse_point new_sparse_point(
		sp.pos_,
	  sp.r_,
		sp.g_,
		sp.b_, 
		sp.a_,
		new_features
	);

	return std::make_shared<Aux::sparse_point>(new_sparse_point);
}



std::shared_ptr<Aux::atlas> Aux::get_atlas() const {
	const auto& a = _aux->get_atlas();

	Aux::atlas new_atlas(
		a.num_atlas_tiles_,
		a.atlas_width_,
		a.atlas_height_,
		a.rotated_
	);

	return std::make_shared<Aux::atlas>(new_atlas);
}

}

//moeglichkeit 1 old code  for get_atlas ... not working because no shared_ptr is returned
// Aux::atlas Aux::get_atlas() const {
// const Aux::atlas Aux::get_atlas() const {
// const std::shared_ptr<Aux::atlas> Aux::get_atlas() const {
	// const auto& a = _aux->get_atlas();

	// Aux::atlas new_atlas(
	// 	a.num_atlas_tiles_,
	// 	a.atlas_width_,
	// 	a.atlas_height_,
	// 	a.rotated_
	// );

	// return std::shared_ptr<Aux::atlas>(&new_atlas);
	// return new_atlas;
// }