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

#ifndef GUA_SPOINTS_FEEDBACK_COLLECTOR_HPP
#define GUA_SPOINTS_FEEDBACK_COLLECTOR_HPP

// guacamole headers
#include <gua/spoints/platform.hpp>
#include <gua/databases/Database.hpp>
#include <zmq.hpp>

#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
// external headers
#include <string>
#include <list>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <chrono>

namespace gua
{
using chrono_timestamp = std::chrono::time_point<std::chrono::system_clock>;

class GUA_SPOINTS_DLL SPointsFeedbackCollector : public Singleton<SPointsFeedbackCollector>
{
  public:
    SPointsFeedbackCollector()
    {
        feedback_zmq_context_ = std::make_shared<zmq::context_t>(1); // means single threaded
        m_reference_timestamp_ = std::chrono::system_clock::now();
    };
    ~SPointsFeedbackCollector() = default;

    chrono_timestamp get_reference_timestamp() { return m_reference_timestamp_; }

    void push_feedback_matrix(RenderContext const& ctx, std::string const& socket_string, spoints::matrix_package const& pushed_feedback_matrix)
    {
        std::lock_guard<std::mutex> lock(m_feedback_mutex_);

        auto const& collected_feedback_matrices_for_socket = queued_feedback_packages_per_context_per_socket_[ctx.id][socket_string];
        for(auto const& curr_matrix_package : collected_feedback_matrices_for_socket)
        {
            if(!memcmp(&curr_matrix_package, &pushed_feedback_matrix, sizeof(curr_matrix_package)))
            {
                return;
            }
        }
        queued_feedback_packages_per_context_per_socket_[ctx.id][socket_string].push_back(pushed_feedback_matrix);
        queued_new_feedback_for_context_[ctx.id] = true;
    }

    bool is_server() const { return m_is_server_; }

    void send_feedback_frame(RenderContext const& ctx)
    {
        std::lock_guard<std::mutex> lock(m_feedback_mutex_);

        if(queued_new_feedback_for_context_[ctx.id])
        {
            std::swap(queued_feedback_packages_per_context_per_socket_[ctx.id], finalized_feedback_packages_per_context_per_socket_[ctx.id]);
            queued_feedback_packages_per_context_per_socket_[ctx.id].clear();
            queued_new_feedback_for_context_[ctx.id] = false;
        }
        else
        {
            return;
        }
        std::map<std::string, std::vector<spoints::matrix_package>> serialized_matrices_per_socket;

        for(auto const& all_feedback_per_socket_per_context : finalized_feedback_packages_per_context_per_socket_)
        {
            std::vector<spoints::matrix_package> collected_finalized_matrices_for_socket;
            for(auto const& all_feedback_per_socket_for_current_context : all_feedback_per_socket_per_context.second)
            {
                std::string current_socket_string = all_feedback_per_socket_for_current_context.first;

                auto socket_iterator = socket_per_socket_string_.find(current_socket_string);
                if(socket_per_socket_string_.end() == socket_iterator)
                {
                    auto new_socket_ptr = std::make_shared<zmq::socket_t>(*feedback_zmq_context_, ZMQ_PUB);

                    int conflate_messages = 1;

                    new_socket_ptr->setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

                    std::string endpoint(std::string("tcp://") + current_socket_string.c_str());

                    try
                    {
                        new_socket_ptr->bind(endpoint.c_str());
                    }
                    catch(const std::exception& e)
                    {
                        std::cout << "Failed to bind feedback socket\n";
                        std::cout << "App is considered to be a server from now on\n";
                        m_is_server_ = true;
                        return;
                    }
                    socket_per_socket_string_.insert(std::make_pair(current_socket_string, new_socket_ptr));
                }

                auto const& matrix_packages_to_submit = all_feedback_per_socket_for_current_context.second;

                auto& matrix_collection_vector_for_socket_string = serialized_matrices_per_socket[current_socket_string];

                for(auto const& matrix_to_potentially_insert : matrix_packages_to_submit)
                {
                    bool already_inside = false;
                    // for (auto const& curr_matrix_package_to_compare_to : matrix_collection_vector_for_socket_string) {
                    //  if (!memcmp ( &matrix_to_potentially_insert, &curr_matrix_package_to_compare_to, sizeof(curr_matrix_package_to_compare_to) ) ) {
                    //    already_inside = true;
                    //    break;
                    //  }
                    // }

                    if(!already_inside)
                    {
                        matrix_collection_vector_for_socket_string.push_back(matrix_to_potentially_insert);
                    }
                }
            }

            for(auto const& collected_feedback_pair_per_socket : serialized_matrices_per_socket)
            {
                size_t byte_before_timestamp = 8;

                size_t feedback_header_byte = byte_before_timestamp + sizeof(int64_t);

                uint32_t num_recorded_matrix_packages = 0;

                auto& current_socket = socket_per_socket_string_[collected_feedback_pair_per_socket.first];

                // serialized_matrices_per_socket

                auto const& collected_matrices = collected_feedback_pair_per_socket.second;

                num_recorded_matrix_packages = collected_matrices.size();

                // HEADER DATA SO FAR:

                // 00000000 uint32_t num_matrices

                zmq::message_t zmqm(feedback_header_byte + num_recorded_matrix_packages * sizeof(spoints::matrix_package));

                int32_t request_package_id = ctx.framecount;

                memcpy((char*)zmqm.data(), (char*)&(num_recorded_matrix_packages), sizeof(uint32_t));
                memcpy((char*)zmqm.data() + sizeof(uint32_t), (char*)&(request_package_id), sizeof(int32_t));

                chrono_timestamp timestamp_during_request = std::chrono::system_clock::now();

                auto start_to_request_diff = timestamp_during_request - m_reference_timestamp_;

                int64_t request_time_stamp = std::chrono::duration<double>(start_to_request_diff).count() * 1000000;
                // std::cout << request_time_stamp << " microseconds\n";
                memcpy((char*)zmqm.data() + byte_before_timestamp, (char*)&(request_time_stamp), sizeof(int64_t));

                memcpy(((char*)zmqm.data()) + (feedback_header_byte), (char*)&(collected_matrices[0]), (num_recorded_matrix_packages) * sizeof(spoints::matrix_package));

                // std::cout << "actually recorded matrices: " << num_recorded_matrix_packages << "\n";
                // send feedback
                current_socket->send(zmqm); // blocking
            }
        }
    }

    chrono_timestamp m_reference_timestamp_;

    std::mutex m_feedback_mutex_;

    std::map<size_t, std::map<std::string, std::vector<spoints::matrix_package>>> finalized_feedback_packages_per_context_per_socket_;

    std::unordered_map<std::string, std::shared_ptr<zmq::socket_t>> socket_per_socket_string_;
    std::shared_ptr<zmq::context_t> feedback_zmq_context_;

    std::map<size_t, std::map<std::string, std::vector<spoints::matrix_package>>> queued_feedback_packages_per_context_per_socket_;

    std::map<size_t, bool> queued_new_feedback_for_context_;

    bool m_is_server_ = false;
};

} // namespace gua

#endif // GUA_SPOINTS_FEEDBACK_COLLECTOR_HPP
