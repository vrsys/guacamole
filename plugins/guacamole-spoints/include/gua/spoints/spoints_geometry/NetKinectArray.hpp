#ifndef SPOINTS_NETKINECTARRAY_HPP
#define SPOINTS_NETKINECTARRAY_HPP

#include <gua/renderer/RenderContext.hpp>

#include <atomic>
#include <mutex>
#include <thread>

namespace spoints{


struct matrix_package{
  std::array<float,16> modelview_matrix;
  std::array<float,16> projection_matrix;

  void swap(matrix_package& rhs) {
    modelview_matrix.swap(rhs.modelview_matrix);
    projection_matrix.swap(rhs.projection_matrix);
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

  void push_matrix_package(matrix_package mp);

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

  std::atomic<bool> m_feedback_need_swap;
  std::thread m_send_feedback;



  scm::gl::vertex_array_ptr point_layout_{nullptr};
  scm::gl::buffer_ptr net_data_vbo_{nullptr};

  size_t num_points_to_draw;

  bool is_vbo_created = false;
};


}


#endif // #ifndef SPOINTS_NETKINECTARRAY_HPP
