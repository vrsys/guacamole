#ifndef GUA_VIRTUAL_TEXTURING_APP_GLFW_CALLBACKS_HPP
#define GUA_VIRTUAL_TEXTURING_APP_GLFW_CALLBACKS_HPP

#include <gua/guacamole.hpp>
#include <gua/utils/Trackball.hpp>

void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods);
void mouse_button(gua::utils::Trackball& trackball, int mousebutton, int action, int mods);

#endif //#ifndef GUA_VIRTUAL_TEXTURING_APP_GLFW_CALLBACKS_HPP
