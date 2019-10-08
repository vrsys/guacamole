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

#include <gua/renderer/ViewDependentUniform.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
ViewDependentUniform::ViewDependentUniform(UniformValue const& value) : default_(value) {}

////////////////////////////////////////////////////////////////////////////////
UniformValue const& ViewDependentUniform::get() const { return default_; }

////////////////////////////////////////////////////////////////////////////////
UniformValue const& ViewDependentUniform::get(int view) const
{
    auto overwrite(uniforms_.find(view));
    if(overwrite != uniforms_.end())
    {
        return overwrite->second;
    }
    return default_;
}

////////////////////////////////////////////////////////////////////////////////
void ViewDependentUniform::set(UniformValue const& value) { default_ = value; }

////////////////////////////////////////////////////////////////////////////////
void ViewDependentUniform::set(int view, UniformValue const& value) { uniforms_[view] = value; }

////////////////////////////////////////////////////////////////////////////////
void ViewDependentUniform::reset(int view) { uniforms_.erase(view); }

////////////////////////////////////////////////////////////////////////////////
void ViewDependentUniform::apply(RenderContext const& ctx, std::string const& name, int view, scm::gl::program_ptr const& prog, unsigned location) const
{
    try
    {
        auto overwrite(uniforms_.find(view));
        if(overwrite != uniforms_.end())
        {
            overwrite->second.apply(ctx, name, prog, location);
        }
        else
        {
            default_.apply(ctx, name, prog, location);
        }
    }
    catch(std::exception& e)
    {
        Logger::LOG_WARNING << "Error: ViewDependentUniform::apply(): Unable to apply ViewDependentUniform.\n";
    }
}

////////////////////////////////////////////////////////////////////////////////
std::ostream& ViewDependentUniform::serialize_to_stream(std::ostream& os) const
{
    // default value gets view id -1
    default_.serialize_to_stream(os);
    os << "|-1,";

    for(auto& uniform : uniforms_)
    {
        uniform.second.serialize_to_stream(os);
        os << "|" << uniform.first << ",";
    }

    return os;
}

////////////////////////////////////////////////////////////////////////////////
ViewDependentUniform ViewDependentUniform::create_from_serialized_string(std::string const& value)
{
    ViewDependentUniform new_uniform;

    auto tokens(string_utils::split(value, ','));
    for(auto& token : tokens)
    {
        auto parts(string_utils::split(token, '|'));
        std::stringstream id_string(parts[2]);
        int view;
        id_string >> view;

        if(view == -1)
        {
            new_uniform.set(UniformValue::create_from_strings(parts[1], parts[0]));
        }
        else
        {
            new_uniform.set(view, UniformValue::create_from_strings(parts[1], parts[0]));
        }
    }

    return new_uniform;
}

////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, ViewDependentUniform const& val) { return val.serialize_to_stream(os); }

} // namespace gua
