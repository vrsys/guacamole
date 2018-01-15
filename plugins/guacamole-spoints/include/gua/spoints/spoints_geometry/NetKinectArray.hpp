#ifndef SPOINTS_NETKINECTARRAY_HPP
#define SPOINTS_NETKINECTARRAY_HPP

#include <gua/renderer/RenderContext.hpp>

#include <atomic>
#include <mutex>
#include <thread>

namespace spoints{


struct key_package {
  bool is_camera;
  std::size_t view_uuid;
  bool stereo_mode;
  std::size_t framecount;
};

struct matrix_package {
  float modelview_matrix[16];
  float projection_matrix[16];
  uint32_t res_xy[2];

/*
  void swap(matrix_package& rhs) {
    modelview_matrix.swap(rhs.modelview_matrix);
    projection_matrix.swap(rhs.projection_matrix);
  }
*/
};


//is_camera, view_uuid, stereo_mode, current_package);

struct camera_matrix_package {
  key_package k_package;
  matrix_package mat_package;

  bool operator<(camera_matrix_package const& rhs) {
    if(k_package.is_camera < rhs.k_package.is_camera) {
      return true;
    } else if (rhs.k_package.is_camera < k_package.is_camera) {
      return false;
    } else {
      if(k_package.view_uuid < rhs.k_package.view_uuid) {
        return true;
      } else if(rhs.k_package.view_uuid < k_package.view_uuid) {
        return false;
      } else {
        if(k_package.stereo_mode < rhs.k_package.stereo_mode) {
          return true;
        } else if(rhs.k_package.stereo_mode < k_package.stereo_mode) {
          if(k_package.framecount < rhs.k_package.framecount) {
            return true;
          } else {
            return false;
          }
        }
      }
    }
  }
};



class NetKinectArray{

public:
  NetKinectArray(const std::string& server_endpoint,
                 const std::string& feedback_endpoint = "");
  ~NetKinectArray();

  void draw(gua::RenderContext const& ctx);
  bool update(gua::RenderContext const& ctx);
  void update_feedback(gua::RenderContext const& ctx);

  inline unsigned char* getBuffer() { return m_buffer.data(); }

  //void push_matrix_package(bool is_camera, std::size_t view_uuid, bool is_stereo_mode, matrix_package mp);
  void push_matrix_package(spoints::camera_matrix_package const& cam_mat_package);

private:
  void readloop();
  void sendfeedbackloop();

  //receiving geometry
  std::mutex m_mutex;
  bool           m_running;
  const std::string m_server_endpoint;
  const std::string m_feedback_endpoint;
  std::vector<uint8_t> m_buffer;
  std::vector<uint8_t> m_buffer_back;

  std::atomic<bool> m_need_swap;
  std::thread m_recv;

  //sending matrices
  std::mutex m_feedback_mutex;
  bool           m_feedback_running;
  const std::string m_server_feedback_endpoint;
  matrix_package m_matrix_package;
  matrix_package m_matrix_package_back;

  spoints::camera_matrix_package submitted_camera_matrix_package;
  spoints::camera_matrix_package submitted_camera_matrix_package_back;

  bool        current_feedback_is_camera_status = true;
  std::size_t current_feedback_view_uuid = {0};
  bool current_feedback_is_stereo_mode;


  //std::map<key_package, matrix_package> cam_matrix_packages;
  std::vector<matrix_package> matrix_packages_to_submit_;
  std::vector<matrix_package> matrix_packages_to_collect_;


  std::size_t last_frame_count_ = std::numeric_limits<std::size_t>::max();
  std::size_t last_omitted_frame_count_ = std::numeric_limits<std::size_t>::max();
/*
  std::map<bool, std::map<size_t, std::map<bool, std::vector<matrix_package>> > >
  camera_group_to_uuid_to_matrix_package_list;
  std::map<bool, std::map<size_t, std::map<bool, std::vector<matrix_package>> > > 
  camera_group_to_uuid_to_matrix_package_list_back;
*/

  std::atomic<bool> m_feedback_need_swap;
  std::thread m_send_feedback;



  scm::gl::vertex_array_ptr point_layout_{nullptr};
  scm::gl::buffer_ptr net_data_vbo_{nullptr};

  size_t num_points_to_draw;

  bool is_vbo_created = false;
};


}


#endif // #ifndef SPOINTS_NETKINECTARRAY_HPP
