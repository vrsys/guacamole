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

// boost headers
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

// std headers
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <unordered_map>

// stores file data
std::unordered_map<std::string, std::vector<unsigned char>> data;

// forward declares
void read_directory(fs::path const& directory, std::string const& root);
void read_file(fs::path const& file, std::string const& root);
void write_output(fs::path const& hpp_file, fs::path const& cpp_file);
void make_variable_name(std::string& file);
bool is_temporary_file(std::string const& file);

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    fs::path input(fs::initial_path<fs::path>());
    fs::path cpp(fs::initial_path<fs::path>());
    fs::path hpp(fs::initial_path<fs::path>());

    // check for sane arguments
    if(argc > 3)
    {
        input = fs::system_complete(fs::path(argv[1]));
        hpp = fs::system_complete(fs::path(argv[2]));
        cpp = fs::system_complete(fs::path(argv[3]));
    }
    else
    {
        std::cout << "\nusage:   guarc [resource_path] [output_header_directory] [output_source_directory]";
        std::cout << std::endl;
    }

    if(!fs::exists(input))
    {
        std::cout << "\nDirectory not found: ";
        std::cout << input.string() << std::endl;
        return 1;
    }

    if(!fs::is_directory(input))
    {
        std::cout << "\nInput argument is not a directory: ";
        std::cout << input.string() << std::endl;
        return 1;
    }

    if(fs::is_directory(cpp))
    {
        std::cout << "\nOutput argument is a directory: ";
        std::cout << cpp.string() << std::endl;
        return 1;
    }

    if(fs::is_directory(hpp))
    {
        std::cout << "\nOutput argument is a directory: ";
        std::cout << hpp.string() << std::endl;
        return 1;
    }

    // remove trailing slash
    std::string root(input.string());

    if(root[root.length() - 1] == '/')
    {
        root = root.substr(0, root.length() - 1);
    }

    // read all input files recursively
    read_directory(input, root);

    // write R.hpp and R.cpp to output directory
    write_output(hpp, cpp);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

void read_directory(fs::path const& directory, std::string const& root)
{
    fs::directory_iterator end_iter;
    for(fs::directory_iterator dir_itr(directory); dir_itr != end_iter; ++dir_itr)
    {
        try
        {
            if(fs::is_directory(dir_itr->status()))
            {
                read_directory(dir_itr->path(), root);
            }
            else if(fs::is_regular_file(dir_itr->status()))
            {
                if(!is_temporary_file(dir_itr->path().filename().string()))
                {
                    read_file(dir_itr->path(), root);
                }
                else
                {
                    std::cout << "Skipping temporary file ";
                    std::cout << dir_itr->path().filename() << std::endl;
                }
            }
            else
            {
                std::cout << "Skipping non-regular file ";
                std::cout << dir_itr->path().filename() << std::endl;
            }
        }
        catch(const std::exception& ex)
        {
            std::cout << "Error:" << dir_itr->path().filename();
            std::cout << " " << ex.what() << std::endl;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void read_file(fs::path const& file, std::string const& root)
{
    std::string full_path(file.string());
    std::string rel_path(full_path.substr(root.length() + 1, std::string::npos));

    // replace windows '\' with '/' for map key
    std::replace(rel_path.begin(), rel_path.end(), '\\', '/');
    std::cout << "Embedding " << rel_path << " ..." << std::endl;

    // open the file:
    std::ifstream s(file.string(), std::ios::in | std::ios::binary);

    data[rel_path] = std::vector<unsigned char>(std::istreambuf_iterator<char>(s), std::istreambuf_iterator<char>());
    // std::cout << "Read " << data[rel_path].size() << " bytes from " << file.string() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void write_output(fs::path const& hpp_file, fs::path const& cpp_file)
{
    // open header file ----------------------------------------------------------
    std::ofstream hpp;
    hpp.open(hpp_file.string());

    if(!hpp)
    {
        std::cout << "Failed to open file for writing: " << hpp_file.string() << std::endl;
        return;
    }

    // open source file ----------------------------------------------------------
    std::ofstream cpp;
    cpp.open(cpp_file.string());

    if(!cpp)
    {
        std::cout << "Failed to open file for writing: " << cpp_file.string() << std::endl;
        return;
    }

    // write global variables ----------------------------------------------------
    for(auto const& entry : data)
    {
        const std::string type("std::vector<unsigned char> ");

        // write header declaration
        std::string variable_name(entry.first);
        make_variable_name(variable_name);

        hpp << "extern const " << type << variable_name << ";" << std::endl;

        // write source definition
        cpp << "const " << type << variable_name << " = {";

        for(int i(0); i < entry.second.size(); ++i)
        {
            if(i % 12 == 0)
                cpp << std::endl << "    ";

            std::ostringstream hex;
            hex << std::hex << std::setfill('0');
            hex << std::setw(2) << (int)entry.second[i];

            cpp << "0x" << hex.str();

            if(i < entry.second.size() - 1)
            {
                cpp << ", ";
            }
        }

        cpp << "};" << std::endl;
    }

    // write map fill method -----------------------------------------------------
    cpp << std::endl;
    cpp << "std::unordered_map<std::string, std::vector<unsigned char> const*> R_fill_map() {" << std::endl;
    cpp << "  std::unordered_map<std::string, std::vector<unsigned char> const*> data;" << std::endl;

    for(auto const& entry : data)
    {
        std::string variable_name(entry.first);
        make_variable_name(variable_name);

        cpp << "  data[\"" << entry.first << "\"] = &" << variable_name << ";" << std::endl;
    }

    cpp << "  return data;" << std::endl;
    cpp << "}" << std::endl;

    hpp.close();
    cpp.close();
}

////////////////////////////////////////////////////////////////////////////////

void make_variable_name(std::string& file)
{
    std::replace(file.begin(), file.end(), '/', '_');
    std::replace(file.begin(), file.end(), '\\', '_');
    std::replace(file.begin(), file.end(), '.', '_');
    std::replace(file.begin(), file.end(), '(', '_');
    std::replace(file.begin(), file.end(), ')', '_');
    std::replace(file.begin(), file.end(), ' ', '_');
}

////////////////////////////////////////////////////////////////////////////////

bool is_temporary_file(std::string const& file)
{
    if(*file.rbegin() == '~')
        return true;

    return false;
}
