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

namespace gua {


class GUA_SPOINTS_DLL SPointsFeedbackCollector : public Singleton<SPointsFeedbackCollector> {
 public:


  SPointsFeedbackCollector() {
    feedback_zmq_context_ = std::make_shared<zmq::context_t>(1); // means single threaded
  };
  ~SPointsFeedbackCollector() = default;


  void push_feedback_matrix(RenderContext const& ctx, std::string const& socket_string, spoints::matrix_package const& pushed_feedback_matrix) {

      std::unique_lock<std::mutex> lock(m_feedback_mutex_);

      std::cout << "Context " << ctx.id << " pushed feedback\n";

      //++num_called_push_feedbacks_per_context_[ctx.id];

      auto const& collected_feedback_matrices_for_socket = feedback_packages_per_socket_[socket_string];
      for (auto const& curr_matrix_package : collected_feedback_matrices_for_socket) {
        if (!memcmp ( &curr_matrix_package, &pushed_feedback_matrix, sizeof(curr_matrix_package) ) ) {
          //std::cout << "BUT WAS ALREADY REGISTERED\n";
          return;
        }
      }

      feedback_packages_per_socket_[socket_string].push_back(pushed_feedback_matrix);

  }


  void send_feedback_frame(RenderContext const& ctx, std::size_t application_frame_count) {



    std::unique_lock<std::mutex> lock(m_feedback_mutex_);

    auto feedback_id_map_iter = seen_feedback_ids_.find(ctx.id);

    std::cout << "Current CTX ID wants to push: " << ctx.id << "\n";
    

    if (application_frame_count == last_seen_application_frame_count_) {
      ++calls_in_this_frame_;
      return;
    } else {
      last_seen_application_frame_count_ = application_frame_count;
  
      std::cout << "Send feedback frame calls in last frame: " << calls_in_this_frame_ << "\n";
      calls_in_this_frame_ = 1;

      for (auto& entry : seen_feedback_ids_) {
        entry.second = false;
      }
      //std::cout << "Context " << ctx.id << " cleared the feedback\n";

      //for (auto& entry : seen_feedback_ids_) {
      //  entry.second = false;
      //}
      
    }

    std::cout << "After first check\n";

    for (auto entry : seen_feedback_ids_) {
      if (true == entry.second) {
        return;
      }
    }

    std::cout << "After second check\n";

    if (seen_feedback_ids_.end() == feedback_id_map_iter) {
      seen_feedback_ids_.insert(std::make_pair(ctx.id, true));
    } else {
      seen_feedback_ids_[ctx.id] = true;
    }

    std::cout << "Context actually pushes: " << ctx.id << "\n";



    //++num_attempted_send_feedback_calls_per_context_[ctx.id];


/*
    for (auto entry : num_attempted_send_feedback_calls_per_context_) {
      if (entry.second < num_called_push_feedbacks_per_context_[entry.first]){
        return;
      }
    }
*/
    for (auto const& feedback_vector_per_socket : feedback_packages_per_socket_) {
      std::string current_socket_string = feedback_vector_per_socket.first;

      auto socket_iterator              = socket_per_socket_string_.find(current_socket_string);
      if (socket_per_socket_string_.end() == socket_iterator) {

        auto new_socket_ptr = std::make_shared<zmq::socket_t>(*feedback_zmq_context_, ZMQ_PUB);

        int conflate_messages  = 1;



        new_socket_ptr->setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

        std::string endpoint(std::string("tcp://") + current_socket_string.c_str());

        try { 
          new_socket_ptr->bind(endpoint.c_str()); 
        } catch (const std::exception& e) {
          std::cout << "Failed to bind feedback socket\n";
          return;
        }

        socket_per_socket_string_.insert(std::make_pair(current_socket_string, new_socket_ptr) );
      }


      auto& current_socket = socket_per_socket_string_[current_socket_string];

      size_t feedback_header_byte = 100;



      uint32_t num_recorded_matrix_packages = 0;

      auto const& matrix_packages_to_submit = feedback_vector_per_socket.second;

      num_recorded_matrix_packages = matrix_packages_to_submit.size();


      //for (auto const& recorded_matrix : matrix_packages_to_submit) {
        //std::cout << "Mat " << mat_counter << ":\n";
        //std::cout << .mat_package << "\n";
      //}

      //HEADER DATA SO FAR:

      // 00000000 uint32_t num_matrices

      
      zmq::message_t zmqm(feedback_header_byte + num_recorded_matrix_packages * sizeof(spoints::matrix_package) );

      memcpy((char*)zmqm.data(), (char*)&(num_recorded_matrix_packages), sizeof(uint32_t));     
      memcpy( ((char*)zmqm.data()) + (feedback_header_byte), (char*)&(matrix_packages_to_submit[0]), (num_recorded_matrix_packages) *  sizeof(spoints::matrix_package) );

      std::cout << "actually recorded matrices: " << num_recorded_matrix_packages << "\n";
      // send feedback
      current_socket->send(zmqm); // blocking

    }



  }

  int32_t                                                                      main_context_id_ = -1;
  std::mutex                                                                   m_feedback_mutex_;
  std::unordered_map<std::string, std::vector<spoints::matrix_package>>        feedback_packages_per_socket_;

  std::unordered_map<std::string,std::shared_ptr<zmq::socket_t>>               socket_per_socket_string_;
  std::shared_ptr<zmq::context_t>                                              feedback_zmq_context_;

  std::unordered_map<unsigned, bool>                                           seen_feedback_ids_;
  std::unordered_map<unsigned, int>                                            num_called_push_feedbacks_per_context_;
  std::unordered_map<unsigned, int>                                            num_attempted_send_feedback_calls_per_context_;
  std::size_t                                                                  last_seen_application_frame_count_ = std::numeric_limits<std::size_t>::max();

  int64_t                                                                      calls_in_this_frame_ = 0;
};

}

#endif  // GUA_SPOINTS_FEEDBACK_COLLECTOR_HPP
