#ifndef GUA_OCCLUSION_CULLING_AWARE_RENDERER_HPP
#define GUA_OCCLUSION_CULLING_AWARE_RENDERER_HPP


#include <gua/renderer/Pipeline.hpp>
#include <scm/gl_core/shader_objects.h>
#include <queue>
#include <gua/node/TriMeshNode.hpp>

namespace gua
{
class MaterialShader;
class Pipeline;
class PipelinePassDescription;
class RenderTarget;


//Structs
////////////////////////////////////////////////////////////////////////////////////////

struct NodeDistancePairComparator
{
    bool operator()(std::pair<gua::node::Node*, double> const& lhs, std::pair<gua::node::Node*, double> const& rhs)
    {
        return lhs.second > rhs.second;
    }
};

struct NodeVisibilityProbabilityPairComparator
{
    bool operator()(std::pair<gua::node::Node*, double> const& lhs, std::pair<gua::node::Node*, double> const& rhs)
    {
        return lhs.second < rhs.second;
    }
};

struct MultiQuery {
    scm::gl::occlusion_query_ptr occlusion_query_pointer;
    std::vector<gua::node::Node*> nodes_to_query;
    bool query_from_last_frame;
};

struct PreviousQueries {
    std::size_t node_uuid;
    scm::gl::occlusion_query_ptr occlusion_query_pointer;
};

struct LastVisibility {
    std::size_t camera_uuid; //setter and getter need to be updated for multiple cameras
    int32_t frame_id;
    bool result;
};


struct VisiblityPersistence {
    //for visibility persistence
    bool last_visibility;
    uint32_t persistence;

    //for pseudo-random queries of visible nodes
    uint32_t randomizer;
    bool query_reasonable;
};

struct RenderInfo {
    MaterialShader* material;
    std::shared_ptr<ShaderProgram> shader;
    scm::gl::rasterizer_state_ptr rasterizer_state;
};


struct BoundingBoxSide {
    BoundingBoxSide(gua::math::vec3f const& mi, gua::math::vec3f const& ma, int c_ax) : min(mi), max(ma), constant_axis(c_ax) {};

    gua::math::vec3f min = {gua::math::vec3f{std::numeric_limits<float>::max()}};
    gua::math::vec3f max = {gua::math::vec3f{std::numeric_limits<float>::min()}};
    int constant_axis = -1;
};


struct MinMax {
    gua::math::vec3f min;
    gua::math::vec3f max;
};


////////////////////////////////////////////////////////////////////////////////////////

class OcclusionCullingAwareRenderer {
public:
    OcclusionCullingAwareRenderer() {}

    ~OcclusionCullingAwareRenderer() {}

    OcclusionCullingAwareRenderer(RenderContext const& ctx, SubstitutionMap const& smap);

    void render_switch_occlusion_culling(Pipeline& pipe, PipelinePassDescription const& desc);
    virtual void render(Pipeline& pipe, PipelinePassDescription const& desc) = 0;
    void render_with_occlusion_culling(Pipeline& pipe, PipelinePassDescription const& desc);

protected:

    virtual void renderSingleNode(Pipeline& pipe, PipelinePassDescription const& desc, gua::node::Node* const current_node, RenderInfo& current_render_info) = 0;


    ////////////////////////////////////////////////////////////////////////////////////////
    scm::gl::rasterizer_state_ptr rs_cull_back_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_cull_none_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_back_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_none_ = nullptr;

    scm::gl::depth_stencil_state_ptr default_depth_test_ = nullptr;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_no_test_no_writing_state_ = nullptr;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_writing_without_test_state_ = nullptr;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_test_without_writing_state_ = nullptr;

    scm::gl::blend_state_ptr default_blend_state_ = nullptr;
    scm::gl::blend_state_ptr color_accumulation_state_ = nullptr;

    std::vector<ShaderProgramStage> default_rendering_program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> default_rendering_programs_;

    // these shaders are used only for visualizing the depth complexity in our system, together with disabled
    // depth tests and color accumulation blend states (expensive, but nevertheless only used for debug purposes)
    std::vector<ShaderProgramStage> depth_complexity_vis_program_stages_;
    std::shared_ptr<ShaderProgram> depth_complexity_vis_program_;

    // these shaders and the compilshaders are used in combination with hardware occlusion queries
    std::vector<ShaderProgramStage> occlusion_query_box_program_stages_;
    std::shared_ptr<ShaderProgram> occlusion_query_box_program_;

    // these shaders and the compilshaders are used in combination with CHC++ to enable DrawArrayInstanced***************not functioning?
    std::vector<ShaderProgramStage> occlusion_query_array_box_program_stages_;
    std::shared_ptr<ShaderProgram> occlusion_query_array_box_program_;


    bool depth_complexity_visualization_;
    bool occlusion_culling_geometry_visualization_;



private:

    //maps
    ////////////////////////////////////////////////////////////////////////////////////////
    SubstitutionMap global_substitution_map_;

    mutable std::unordered_map<std::size_t, std::unordered_map<std::size_t, bool> > was_not_frustum_culled_;
    mutable std::unordered_map<std::size_t, std::unordered_map<std::size_t, bool> > is_visible_for_camera_;
    mutable std::unordered_map<std::size_t, std::unordered_map<std::size_t, uint32_t> > last_visibility_check_frame_id_;
    mutable std::unordered_map<std::size_t, LastVisibility > last_visibility_checked_result_;
    mutable std::unordered_map<std::size_t, VisiblityPersistence > node_visibility_persistence;

    mutable scm::gl::buffer_ptr empty_vbo_ = nullptr;
    mutable scm::gl::vertex_array_ptr empty_vao_layout_ = nullptr;


    //getter and setter
    ////////////////////////////////////////////////////////////////////////////////////////
    int32_t get_last_visibility_check_frame_id(std::size_t in_unique_node_id, std::size_t in_camera_uuid) const;
    uint32_t get_visibility_persistence(std::size_t node_uuid);
    bool get_visibility(std::size_t node_path, std::size_t in_camera_uuid) const;
    LastVisibility get_last_visibility_checked_result(std::size_t in_unique_node_id) const;
    bool get_query_reasonable(std::size_t in_unique_node_id) const;

    void set_visibility_persistence(std::size_t node_uuid, bool visibility);
    void set_visibility(std::size_t in_unique_node_id, std::size_t in_camera_uuid, bool is_visible);
    void set_last_visibility_check_frame_id(std::size_t in_unique_node_id, std::size_t in_camera_uuid, int32_t current_frame_id);
    void set_occlusion_query_states(RenderContext const& ctx);
    void set_geometry_visualisation_states(RenderContext const& ctx);
    void set_last_visibility_checked_result(std::size_t node_path, std::size_t in_camera_uuid, int32_t current_frame_id, bool result);


    //CHC++ helper
    ////////////////////////////////////////////////////////////////////////////////////////
    void traverse_node(gua::node::Node* current_node,
                       RenderContext const& ctx,
                       Pipeline& pipe,
                       PipelinePassDescription const& desc,
                       RenderTarget& render_target,
                       MaterialShader* current_material,
                       std::shared_ptr<ShaderProgram> current_shader,
                       scm::gl::rasterizer_state_ptr current_rasterizer_state,
                       gua::math::vec3f const& world_space_cam_pos,
                       std::priority_queue<std::pair<gua::node::Node*, double>,
                       std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator >& traversal_priority_queue,
                       std::size_t in_camera_uuid, int64_t current_frame_id);

    void pull_up_visibility(gua::node::Node* current_node, int64_t current_frame_id, std::size_t in_camera_uuid);

    void issue_occlusion_query(RenderContext const& ctx, Pipeline& pipe,
                               PipelinePassDescription const& desc,
                               scm::math::mat4d const& view_projection_matrix,
                               gua::math::vec3f const& world_space_cam_pos,
                               int64_t current_frame_id, std::size_t in_camera_uuid,
                               std::vector<gua::node::Node*> const& current_nodes,
                               bool query_last_frame = false);


    void issue_multi_query(RenderContext const& ctx, Pipeline& pipe,
                           PipelinePassDescription const& desc,
                           scm::math::mat4d const& view_projection_matrix,
                           gua::math::vec3f const& world_space_cam_pos,
                           int64_t current_frame_id, std::size_t in_camera_uuid, std::queue<gua::node::Node*>& i_query_queue);

    void handle_returned_query(RenderContext const& ctx,
                               Pipeline& pipe,
                               PipelinePassDescription const& desc,
                               RenderTarget& render_target,
                               MaterialShader* current_material,
                               std::shared_ptr<ShaderProgram> current_shader,
                               scm::math::mat4d const& view_projection_matrix,
                               gua::math::vec3f const& world_space_cam_pos,
                               scm::gl::rasterizer_state_ptr current_rasterizer_state,
                               std::priority_queue<std::pair<gua::node::Node*, double>, std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator >& traversal_priority_queue,
                               std::size_t in_camera_uuid,
                               uint64_t query_result,
                               std::vector<gua::node::Node*> front_query_vector,
                               int64_t current_frame_id);



    void render_visible_leaf(gua::node::Node* current_query_node,
                             RenderContext const& ctx,
                             Pipeline& pipe,
                             RenderTarget& render_target,
                             MaterialShader* current_material,
                             std::shared_ptr<ShaderProgram> current_shader,
                             scm::gl::rasterizer_state_ptr current_rasterizer_state);

    gua::math::vec3f find_raycast_intersection(gua::node::Node* node, gua::math::vec3f const& world_space_cam_pos) const;
    bool is_inside(gua::math::vec3f const& intersection_pt,  BoundingBoxSide const& bounding_plane) const;


    std::vector<gua::node::Node*> find_tightest_bounding_volume(gua::node::Node* queried_node,
                                       RenderContext const& ctx,
                                       gua::math::vec3f const& world_space_cam_pos,
                                       std::shared_ptr<ShaderProgram>& current_shader,
                                       size_t in_camera_uuid,
                                       size_t current_frame_id,
                                       unsigned int const dmax,
                                       float const smax);

    bool check_children_surface_area(std::vector<gua::node::Node*> const& in_parent_nodes, float const smax) const;


    //Rendering related
    ////////////////////////////////////////////////////////////////////////////////////////

    void instanced_array_draw(std::vector<gua::node::Node*> const& leaf_node_vector,
                              RenderContext const& ctx,
                              std::shared_ptr<ShaderProgram>& current_shader,
                              size_t in_camera_uuid,
                              size_t current_frame_id);

    void unbind_and_reset(RenderContext const& ctx, RenderTarget& render_target);




    void upload_uniforms_for_node(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader,
                                  Pipeline& pipe, scm::gl::rasterizer_state_ptr& current_rasterizer_state);


    void switch_state_based_on_node_material(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader,
            MaterialShader* current_material, RenderTarget const& target, bool shadow_mode, std::size_t cam_uuid);


    void switch_state_for_depth_complexity_vis(RenderContext const& ctx, std::shared_ptr<ShaderProgram>& active_shader);
    //Member Variables
    ////////////////////////////////////////////////////////////////////////////////////////
    bool in_query_state_ = false;
    std::queue<MultiQuery> query_queue_;
    std::queue<PreviousQueries> previous_query_queue_;

};

}
#endif