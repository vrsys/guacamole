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

}

