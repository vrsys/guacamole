// class header
#include <gua/utils/LineStripImporter.hpp>
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
void LineStripImporter::create_empty_line(std::string const& empty_line_name)
{
    parsing_successful_ = true;
    num_parsed_line_strips_ = 1;

    parsed_line_objects_.push_back(std::make_pair(empty_line_name, LineObject()));
}

void LineStripImporter::read_file(std::string const& file_name)
{
    parsing_successful_ = false;
    std::ifstream in_lob_file(file_name, std::ios::in);

    if(!in_lob_file.is_open())
    {
        Logger::LOG_WARNING << "Could not open *.lob file for reading!" << std::endl;
    }

    std::string line_buffer("");

    auto ltrim = [](std::string& string_to_trim) {
        string_to_trim.erase(string_to_trim.begin(), std::find_if(string_to_trim.begin(), string_to_trim.end(), [](int ch) { return !std::isspace(ch); }));
    };

    NamedLineObject* current_line_object = nullptr;

    std::vector<scm::math::vec3f> line_strip_positions;

    while(std::getline(in_lob_file, line_buffer))
    {
        // trim whitespace to the left of the string
        ltrim(line_buffer);

        if(line_buffer.empty())
        {
            continue;
        }

        std::istringstream in_sstream(line_buffer);
        char line_prefix;

        in_sstream >> line_prefix;

        // create new object
        if('o' == line_prefix)
        {
            std::string new_object_name = "";

            in_sstream >> new_object_name;
            parsed_line_objects_.push_back(std::make_pair(new_object_name, LineObject()));

            current_line_object = &(parsed_line_objects_).back();

            ++num_parsed_line_strips_;
        }
        else
        {
            float* float_attributes_to_parse = nullptr;
            switch(line_prefix)
            {
            case 'v':
                float_attributes_to_parse = new float[3];
                for(int position_idx = 0; position_idx < 3; ++position_idx)
                {
                    in_sstream >> float_attributes_to_parse[position_idx];
                }
                current_line_object->second.vertex_position_database.emplace_back(float_attributes_to_parse[0], float_attributes_to_parse[1], float_attributes_to_parse[2]);
                delete[] float_attributes_to_parse;
                break;

            case 'c':
                float_attributes_to_parse = new float[4];
                for(int color_idx = 0; color_idx < 4; ++color_idx)
                {
                    in_sstream >> float_attributes_to_parse[color_idx];
                }
                current_line_object->second.vertex_color_database.emplace_back(float_attributes_to_parse[0], float_attributes_to_parse[1], float_attributes_to_parse[2], float_attributes_to_parse[3]);
                delete[] float_attributes_to_parse;
                break;

            case 't':
                float_attributes_to_parse = new float;
                in_sstream >> float_attributes_to_parse[0];

                current_line_object->second.vertex_thickness_database.emplace_back(float_attributes_to_parse[0]);
                delete float_attributes_to_parse;
                break;

            case 'n':
                float_attributes_to_parse = new float[3];
                for(int normal_idx = 0; normal_idx < 3; ++normal_idx)
                {
                    in_sstream >> float_attributes_to_parse[normal_idx];
                }
                current_line_object->second.vertex_normal_database.emplace_back(float_attributes_to_parse[0], float_attributes_to_parse[1], float_attributes_to_parse[2]);
                delete[] float_attributes_to_parse;
                break;

            case 's':
                Logger::LOG_WARNING << "*.lob-parser option 's' is not implemented yet" << std::endl;
                break;

            case 'g':
                // Ignore g-tag for now
                break;
            // lines to be interpreted as GL_LINES and need to duplicate_vertices
            case 'l':
            {
                int currently_read_idx = 0;

                std::vector<int> parsed_indices;
                while(!in_sstream.eof())
                {
                    in_sstream >> currently_read_idx;
                    parsed_indices.push_back(currently_read_idx - 1);
                }

                for(unsigned int vertex_idx = 0; vertex_idx < parsed_indices.size() - 2; ++vertex_idx)
                {
                    int read_index_start = parsed_indices[vertex_idx];
                    int read_index_end = parsed_indices[vertex_idx + 1];
                    auto read_vertex_pos_start = current_line_object->second.vertex_position_database[read_index_start];
                    line_strip_positions.push_back(read_vertex_pos_start);
                    auto read_vertex_pos_end = current_line_object->second.vertex_position_database[read_index_end];
                    line_strip_positions.push_back(read_vertex_pos_end);
                }
                break;

                break;
            }
            default:
                Logger::LOG_WARNING << "Unknown *.line-parser option " << line_prefix << std::endl;
            }
        }
    }

    if(!line_strip_positions.empty())
    {
        std::swap(line_strip_positions, current_line_object->second.vertex_position_database);
        line_strip_positions.clear();
    }

    for(auto& current_line_object : parsed_line_objects_)
    {
        if(current_line_object.second.vertex_position_database.size() > current_line_object.second.vertex_color_database.size())
        {
            current_line_object.second.vertex_color_database.resize(current_line_object.second.vertex_position_database.size(), scm::math::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        }

        if(current_line_object.second.vertex_position_database.size() > current_line_object.second.vertex_thickness_database.size())
        {
            current_line_object.second.vertex_thickness_database.resize(current_line_object.second.vertex_position_database.size(), 1.0f);
        }

        if(current_line_object.second.vertex_position_database.size() > current_line_object.second.vertex_normal_database.size())
        {
            current_line_object.second.vertex_normal_database.resize(current_line_object.second.vertex_position_database.size(), scm::math::vec3(0.0f, 1.0f, 0.0f));
        }
    }

    parsing_successful_ = true;
}

bool LineStripImporter::parsing_successful() const { return parsing_successful_; }

int LineStripImporter::num_parsed_line_strips() const { return num_parsed_line_strips_; }

NamedLineObject LineStripImporter::parsed_line_object_at(int line_object_index) const { return parsed_line_objects_[line_object_index]; }

} // namespace gua