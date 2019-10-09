// class header
#include <gua/utils/DynamicGeometryImporter.hpp>
// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/ToGua.hpp>
// #include <gua/utils/Timer.hpp>

// external headers
#include <iostream>
#include <fstream>
#include <sstream>

namespace gua
{
void DynamicGeometryImporter::create_empty_dynamic_geometry(std::string const& empty_dynamic_geometry_name)
{
    parsing_successful_ = true;
    num_parsed_dynamic_geometries_ = 1;

    dynamic_geometry_object_ptr_ = std::make_shared<DynamicGeometryObject>(DynamicGeometryObject());
}

bool DynamicGeometryImporter::parsing_successful() const { return parsing_successful_; }

int DynamicGeometryImporter::num_parsed_dynamic_geometries() const { return num_parsed_dynamic_geometries_; }

std::shared_ptr<DynamicGeometryObject> DynamicGeometryImporter::get_dynamic_geometry_object_ptr() const { return dynamic_geometry_object_ptr_; }

} // namespace gua