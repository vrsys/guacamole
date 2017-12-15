#ifndef SPOINTS_NETKINECTARRAY_HPP
#define SPOINTS_NETKINECTARRAY_HPP

#include <gua/renderer/RenderContext.hpp>

#include <atomic>
#include <mutex>
#include <thread>

namespace spoints{


struct matrix_package{
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

class NetKinectArray{

public:
  NetKinectArray(const std::string& server_endpoint,
                 const std::string& feedback_endpoint = "");
  ~NetKinectArray();

  void draw(gua::RenderContext const& ctx);
  bool update(gua::RenderContext const& ctx);
  void update_feedback(gua::RenderContext const& ctx);

  inline unsigned char* getBuffer() { return m_buffer.data(); }

  void push_matrix_package(bool is_camera, std::size_t view_uuid, bool is_stereo_mode, matrix_package mp);

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

  bool        current_feedback_is_camera_status = true;
  std::size_t current_feedback_view_uuid = {0};
  bool current_feedback_is_stereo_mode;

  std::map<bool, std::map<size_t, std::map<bool, std::vector<matrix_package>> > > 
  camera_group_to_uuid_to_matrix_package_list;
  std::map<bool, std::map<size_t, std::map<bool, std::vector<matrix_package>> > > 
  camera_group_to_uuid_to_matrix_package_list_back;

  std::atomic<bool> m_feedback_need_swap;
  std::thread m_send_feedback;



  scm::gl::vertex_array_ptr point_layout_{nullptr};
  scm::gl::buffer_ptr net_data_vbo_{nullptr};

  size_t num_points_to_draw;

  bool is_vbo_created = false;
};


}


#endif // #ifndef SPOINTS_NETKINECTARRAY_HPP
