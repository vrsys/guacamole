#include <gua/nrp/nrp_cam_node.hpp>
#include <gua/nrp/nrp_config.hpp>
#include <gua/nrp/nrp_interactive_node.hpp>
#include <gua/nrp/nrp_node.hpp>
#include <gua/nrp/nrp_scene.hpp>

#include "OgreGpuProgramManager.h"
#include "OgreHighLevelGpuProgramManager.h"
#include "OgreMaterialManager.h"
#include "OgreResourceGroupManager.h"
#include "OgreRoot.h"
#include "OgreTextureManager.h"

namespace gua
{
namespace nrp
{
class SimplifiedPassTranslator : public Ogre::PassTranslator
{
  public:
    void translate(Ogre::ScriptCompiler *compiler, const Ogre::AbstractNodePtr &node) override
    {
        using namespace Ogre;
        ObjectAbstractNode *obj = reinterpret_cast<ObjectAbstractNode *>(node.get());

        Technique *technique = any_cast<Technique *>(obj->parent->context);
        mPass = technique->createPass();
        obj->context = Any(mPass);

        // Get the name of the technique
        if(!obj->name.empty())
            mPass->setName(obj->name);

        for(AbstractNodeList::iterator i = obj->children.begin(); i != obj->children.end(); ++i)
        {
            if((*i)->type == ANT_PROPERTY)
            {
                PropertyAbstractNode *prop = reinterpret_cast<PropertyAbstractNode *>((*i).get());
                switch(prop->id)
                {
                case ID_AMBIENT:
                    if(prop->values.empty())
                    {
                        compiler->addError(ScriptCompiler::CE_NUMBEREXPECTED, prop->file, prop->line);
                    }
                    else if(prop->values.size() > 4)
                    {
                        compiler->addError(ScriptCompiler::CE_FEWERPARAMETERSEXPECTED, prop->file, prop->line, "ambient must have at most 4 parameters");
                    }
                    else
                    {
                        if(prop->values.front()->type == ANT_ATOM && ((AtomAbstractNode *)prop->values.front().get())->id == ID_VERTEXCOLOUR)
                        {
                            mPass->setVertexColourTracking(mPass->getVertexColourTracking() | TVC_AMBIENT);
                        }
                        else
                        {
                            ColourValue val = ColourValue::White;
                            if(getColour(prop->values.begin(), prop->values.end(), &val))
                                mPass->setAmbient(val);
                            else
                                compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line, "ambient requires 3 or 4 colour arguments, or a \"vertexcolour\" directive");
                        }
                    }
                    break;
                case ID_DIFFUSE:
                    if(prop->values.empty())
                    {
                        compiler->addError(ScriptCompiler::CE_NUMBEREXPECTED, prop->file, prop->line);
                    }
                    else if(prop->values.size() > 4)
                    {
                        compiler->addError(ScriptCompiler::CE_FEWERPARAMETERSEXPECTED, prop->file, prop->line, "diffuse must have at most 4 arguments");
                    }
                    else
                    {
                        if(prop->values.front()->type == ANT_ATOM && ((AtomAbstractNode *)prop->values.front().get())->id == ID_VERTEXCOLOUR)
                        {
                            mPass->setVertexColourTracking(mPass->getVertexColourTracking() | TVC_DIFFUSE);
                        }
                        else
                        {
                            ColourValue val = ColourValue::White;
                            if(getColour(prop->values.begin(), prop->values.end(), &val))
                                mPass->setDiffuse(val);
                            else
                                compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line, "diffuse requires 3 or 4 colour arguments, or a \"vertexcolour\" directive");
                        }
                    }
                    break;
                case ID_SPECULAR:
                    if(prop->values.empty())
                    {
                        compiler->addError(ScriptCompiler::CE_NUMBEREXPECTED, prop->file, prop->line);
                    }
                    else if(prop->values.size() > 5)
                    {
                        compiler->addError(ScriptCompiler::CE_FEWERPARAMETERSEXPECTED, prop->file, prop->line, "specular must have at most 5 arguments");
                    }
                    else
                    {
                        if(prop->values.front()->type == ANT_ATOM && ((AtomAbstractNode *)prop->values.front().get())->id == ID_VERTEXCOLOUR)
                        {
                            mPass->setVertexColourTracking(mPass->getVertexColourTracking() | TVC_SPECULAR);

                            if(prop->values.size() >= 2)
                            {
                                Real val = 0;
                                if(getReal(prop->values.back(), &val))
                                    mPass->setShininess(val);
                                else
                                    compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line,
                                                       "specular does not support \"" + prop->values.back()->getValue() + "\" as its second argument");
                            }
                        }
                        else
                        {
                            if(prop->values.size() < 4)
                            {
                                compiler->addError(ScriptCompiler::CE_NUMBEREXPECTED, prop->file, prop->line, "specular expects at least 4 arguments");
                            }
                            else
                            {
                                AbstractNodeList::const_iterator i0 = getNodeAt(prop->values, 0), i1 = getNodeAt(prop->values, 1), i2 = getNodeAt(prop->values, 2);
                                ColourValue val(0.0f, 0.0f, 0.0f, 1.0f);
                                if(getFloat(*i0, &val.r) && getFloat(*i1, &val.g) && getFloat(*i2, &val.b))
                                {
                                    if(prop->values.size() == 4)
                                    {
                                        mPass->setSpecular(val);

                                        AbstractNodeList::const_iterator i3 = getNodeAt(prop->values, 3);
                                        Real shininess = 0.0f;
                                        if(getReal(*i3, &shininess))
                                            mPass->setShininess(shininess);
                                        else
                                            compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line, "specular fourth argument must be a valid number for shininess attribute");
                                    }
                                    else
                                    {
                                        AbstractNodeList::const_iterator i3 = getNodeAt(prop->values, 3);
                                        if(!getFloat(*i3, &val.a))
                                            compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line, "specular fourth argument must be a valid color component value");
                                        else
                                            mPass->setSpecular(val);

                                        AbstractNodeList::const_iterator i4 = getNodeAt(prop->values, 4);
                                        Real shininess = 0.0f;
                                        if(getReal(*i4, &shininess))
                                            mPass->setShininess(shininess);
                                        else
                                            compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line, "specular fourth argument must be a valid number for shininess attribute");
                                    }
                                }
                                else
                                {
                                    compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line, "specular must have first 3 arguments be a valid colour");
                                }
                            }
                        }
                    }
                    break;
                case ID_EMISSIVE:
                    if(prop->values.empty())
                    {
                        compiler->addError(ScriptCompiler::CE_NUMBEREXPECTED, prop->file, prop->line);
                    }
                    else if(prop->values.size() > 4)
                    {
                        compiler->addError(ScriptCompiler::CE_FEWERPARAMETERSEXPECTED, prop->file, prop->line, "emissive must have at most 4 arguments");
                    }
                    else
                    {
                        if(prop->values.front()->type == ANT_ATOM && ((AtomAbstractNode *)prop->values.front().get())->id == ID_VERTEXCOLOUR)
                        {
                            mPass->setVertexColourTracking(mPass->getVertexColourTracking() | TVC_EMISSIVE);
                        }
                        else
                        {
                            ColourValue val(0.0f, 0.0f, 0.0f, 1.0f);
                            if(getColour(prop->values.begin(), prop->values.end(), &val))
                                mPass->setSelfIllumination(val);
                            else
                                compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line, "emissive requires 3 or 4 colour arguments, or a \"vertexcolour\" directive");
                        }
                    }
                    break;
                }
            }
            else if((*i)->type == ANT_OBJECT)
            {
                ObjectAbstractNode *child = reinterpret_cast<ObjectAbstractNode *>((*i).get());
                processNode(compiler, *i);
            }
        }
    }
};
class SimplifiedScriptTranslator : public Ogre::ScriptTranslatorManager
{
  public:
    SimplifiedScriptTranslator() {}
    size_t getNumTranslators() const { return 3; }
    Ogre::ScriptTranslator *getTranslator(const Ogre::AbstractNodePtr &node)
    {
        using namespace Ogre;

        Ogre::ScriptTranslator *translator = 0;

        if(node->type == ANT_OBJECT)
        {
            ObjectAbstractNode *obj = reinterpret_cast<ObjectAbstractNode *>(node.get());
            ObjectAbstractNode *parent = obj->parent ? reinterpret_cast<ObjectAbstractNode *>(obj->parent) : 0;
            if(obj->id == ID_MATERIAL)
                translator = &mMaterialTranslator;
            else if(obj->id == ID_TECHNIQUE && parent && parent->id == ID_MATERIAL)
                translator = &mTechniqueTranslator;
            else if(obj->id == ID_PASS && parent && parent->id == ID_TECHNIQUE)
                translator = &mPassTranslator;
            // TODO: investigate possibility of using textures, particles, etc.
            //          else if(obj->id == ID_TEXTURE_UNIT && parent && parent->id == ID_PASS)
            //              translator = &mTextureUnitTranslator;
            //          else if(obj->id == ID_TEXTURE_SOURCE && parent && parent->id == ID_TEXTURE_UNIT)
            //              translator = &mTextureSourceTranslator;
            //          else if(obj->id == ID_FRAGMENT_PROGRAM ||
            //              obj->id == ID_VERTEX_PROGRAM ||
            //              obj->id == ID_GEOMETRY_PROGRAM ||
            //              obj->id == ID_TESSELATION_HULL_PROGRAM ||
            //              obj->id == ID_TESSELATION_DOMAIN_PROGRAM ||
            //              obj->id == ID_COMPUTE_PROGRAM)
            //              translator = &mGpuProgramTranslator;
            //          else if(obj->id == ID_SHARED_PARAMS)
            //              translator = &mSharedParamsTranslator;
            //          else if(obj->id == ID_PARTICLE_SYSTEM)
            //              translator = &mParticleSystemTranslator;
            //          else if(obj->id == ID_EMITTER)
            //              translator = &mParticleEmitterTranslator;
            //          else if(obj->id == ID_AFFECTOR)
            //              translator = &mParticleAffectorTranslator;
            //          else if(obj->id == ID_COMPOSITOR)
            //              translator = &mCompositorTranslator;
            //          else if(obj->id == ID_TECHNIQUE && parent && parent->id == ID_COMPOSITOR)
            //              translator = &mCompositionTechniqueTranslator;
            //          else if((obj->id == ID_TARGET || obj->id == ID_TARGET_OUTPUT) && parent && parent->id == ID_TECHNIQUE)
            //              translator = &mCompositionTargetPassTranslator;
            //          else if(obj->id == ID_PASS && parent && (parent->id == ID_TARGET || parent->id == ID_TARGET_OUTPUT))
            //              translator = &mCompositionPassTranslator;
        }
        return translator;
    }

  private:
    Ogre::MaterialTranslator mMaterialTranslator;
    Ogre::TechniqueTranslator mTechniqueTranslator;
    SimplifiedPassTranslator mPassTranslator;
};

NRPScene::NRPScene() : _mutex_receive(), _mutex_scenegraph(), _mutex_pose_msgs()
{
    new Ogre::LodStrategyManager();
    new Ogre::LogManager();
    new Ogre::ResourceGroupManager();
    new Ogre::HighLevelGpuProgramManager();
    new Ogre::ScriptCompilerManager();
    new Ogre::MaterialManager();

    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    Ogre::MaterialManager::getSingleton().initialise();

    auto mock_translator = new SimplifiedScriptTranslator();

    Ogre::ScriptCompilerManager::getSingleton().clearTranslatorManagers();
    Ogre::ScriptCompilerManager::getSingleton().addTranslatorManager(mock_translator);

    Ogre::DataStreamPtr ptr_ds_grid(new Ogre::FileStreamDataStream(OGRE_NEW_T(std::fstream, Ogre::MEMCATEGORY_GENERAL)(NRPConfig::get_instance().get_nrp_grid_material(), std::fstream::in), false));
    Ogre::MaterialManager::getSingleton().parseScript(ptr_ds_grid, "General");
    ptr_ds_grid.setNull();

    Ogre::DataStreamPtr ptr_ds_gazebo(
        new Ogre::FileStreamDataStream(OGRE_NEW_T(std::fstream, Ogre::MEMCATEGORY_GENERAL)(NRPConfig::get_instance().get_nrp_gazebo_material(), std::fstream::in), false));
    Ogre::MaterialManager::getSingleton().parseScript(ptr_ds_gazebo, "General");
    ptr_ds_gazebo.setNull();

    auto uniform_color_desc = std::make_shared<gua::MaterialShaderDescription>(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/materials/uniform_color.gmd");
    auto uniform_color_shader(std::make_shared<gua::MaterialShader>("overwrite_color", uniform_color_desc));
    gua::MaterialShaderDatabase::instance()->add(uniform_color_shader);
}
NRPScene::~NRPScene()
{
    _msgs_model.clear();
    _msgs_visual.clear();
    _msgs_pose.clear();
    _msgs_scene.clear();
    _msgs_light_factory.clear();
    _msgs_light_modify.clear();
    _msgs_link.clear();

    _visuals.clear();
    _lights.clear();

    _world_visual.reset();
}
void NRPScene::set_root_node(gua::nrp::NRPNode *root_node)
{
    _mutex_scenegraph.lock();
    _root_node = root_node;
    _is_root_not_initialized = true;
    _world_visual.reset(new NRPVisual("world_visual", _root_node));
    _mutex_scenegraph.unlock();
}
void NRPScene::set_interactive_node(NRPInteractiveNode *interactive_node)
{
    _mutex_scenegraph.lock();
    _interactive_node = interactive_node;
    _mutex_scenegraph.unlock();
}
void NRPScene::set_cam_node(gua::nrp::NRPCameraNode *cam_node)
{
    _mutex_scenegraph.lock();
    _cam_node = cam_node;
    _mutex_scenegraph.unlock();
}
void NRPScene::on_skeleton_pose_msg(ConstPoseAnimationPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_receive);
    skeleton_msgs_list::iterator iter;

    // Find an old model message, and remove them
    for(iter = _msgs_skeleton_pose.begin(); iter != _msgs_skeleton_pose.end(); ++iter)
    {
        if((*iter)->model_name() == msg->model_name())
        {
            _msgs_skeleton_pose.erase(iter);
            break;
        }
    }

    _msgs_skeleton_pose.emplace_back(msg);
}
void NRPScene::on_model_msg(ConstModelPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_model.emplace_back(msg);
}
void NRPScene::on_scene_msg(ConstScenePtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_scene.emplace_back(msg);
}
void NRPScene::on_pose_msg(ConstPosesStampedPtr &msg)
{
    std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);

    for(int i = 0; i < msg->pose_size(); ++i)
    {
        auto iter = _msgs_pose.find(msg->pose(i).id());
        if(iter != _msgs_pose.end())
            iter->second.CopyFrom(msg->pose(i));
        else
            _msgs_pose.insert(std::make_pair(msg->pose(i).id(), msg->pose(i)));
    }
}
void NRPScene::on_light_factory_msg(ConstLightPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_light_factory.emplace_back(msg);
}
void NRPScene::on_light_modify_msg(ConstLightPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_light_modify.emplace_back(msg);
}
bool NRPScene::process_visual_msg(ConstVisualPtr &msg, NRPVisual::VisualType type)
{
    bool result = false;
    auto iter = _visuals.end();

    if(msg->has_id())
    {
        iter = _visuals.find(msg->id());
    }
    else
    {
        ptr_visual vis = this->get_visual(msg->name());
        iter = vis ? _visuals.find(vis->get_id()) : _visuals.end();
    }

    if(msg->has_delete_me() && msg->delete_me())
    {
        if(iter != _visuals.end())
        {
            _visuals.erase(iter);
            result = true;
        }
    }
    else if(iter != _visuals.end())
    {
        iter->second->update_from_msg(msg);
        result = true;
    }
    else
    {
        // std::cout << " a bit of a hack: Heightmap stuff" << std::endl;
        ptr_visual visual;

        // If the visual has a parent which is not the name of the scene...
        if(msg->has_parent_name() && msg->parent_name() != _name)
        {
            if(msg->has_id())
                iter = _visuals.find(msg->id());
            else
            {
                ptr_visual vis = this->get_visual(msg->name());
                iter = vis ? _visuals.find(vis->get_id()) : _visuals.end();
            }

            if(iter != _visuals.end())
                std::cerr << "Visual already exists. This shouldn't happen.\n";

            // Make sure the parent visual exists before trying to add a child
            // visual
            iter = _visuals.find(msg->parent_id());
            if(iter != _visuals.end())
            {
                visual.reset(new NRPVisual(msg->name(), iter->second));
                if(msg->has_id())
                    visual->set_id(msg->id());
            }
        }
        else
        {
            // Add a visual that is attached to the scene root
            visual.reset(new NRPVisual(msg->name(), _world_visual));
            if(msg->has_id())
                visual->set_id(msg->id());
        }

        if(visual)
        {
            result = true;
            visual->update_from_msg(msg);
            visual->set_type(type);

            _visuals[visual->get_id()] = visual;
        }
    }

    return result;
}

ptr_visual NRPScene::get_visual(const uint32_t id) const
{
    auto iter = _visuals.find(id);
    if(iter != _visuals.end())
        return iter->second;
    return ptr_visual();
}

ptr_visual NRPScene::get_visual(const std::string &name) const
{
    ptr_visual result;

    visuals_map::const_iterator iter;
    for(iter = _visuals.begin(); iter != _visuals.end(); ++iter)
    {
        if(iter->second == nullptr)
        {
            continue;
        }

        if(iter->second != nullptr && iter->second->get_name() == name)
        {
            break;
        }
    }

    if(iter != _visuals.end())
    {
        result = iter->second;
    }
    else
    {
        std::string otherName = _name + "::" + name;
        for(iter = _visuals.begin(); iter != _visuals.end(); ++iter)
        {
            if(iter->second != nullptr && iter->second->get_name() == otherName)
                break;
        }

        if(iter != _visuals.end())
            result = iter->second;
    }

    return result;
}

bool NRPScene::process_link_msg(ConstLinkPtr &msg)
{
    ptr_visual linkVis;

    if(msg->has_id())
        linkVis = this->get_visual(msg->id());
    else
        linkVis = this->get_visual(msg->name());

    if(!linkVis)
    {
        std::cerr << "No link visual with id[" << msg->id() << "] and name[" << msg->name() << "]\n";
        return false;
    }

    return true;
}
bool NRPScene::process_light_factory_msg(ConstLightPtr &msg)
{
    light_map::iterator iter;
    iter = _lights.find(msg->name());

    if(iter == _lights.end())
    {
        ptr_light light(new NRPLight(msg->name(), _root_node));
        light->load_from_msg(msg);
        _lights[msg->name()] = light;
    }
    else
    {
        //        std::cerr << "Light [" << msg->name() << "] already exists."
        //                  << " Use topic ~/light/modify to modify it." << std::endl;
        return process_light_modify_msg(msg);
    }

    return true;
}
bool NRPScene::process_light_modify_msg(ConstLightPtr &msg)
{
    light_map::iterator iter;
    iter = _lights.find(msg->name());

    if(iter == _lights.end())
    {
        //        std::cerr << "Light [" << msg->name() << "] not found."
        //                  << " Use topic ~/factory/light to spawn a new light." << std::endl;
        return process_light_factory_msg(msg);
    }
    else
    {
        iter->second->load_from_msg(msg);
    }

    return true;
}
bool NRPScene::process_model_msg(const gazebo::msgs::Model &msg)
{
    std::string modelName, linkName;

    modelName = msg.name() + "::";
    for(int j = 0; j < msg.visual_size(); ++j)
    {
        boost::shared_ptr<gazebo::msgs::Visual> vm(new gazebo::msgs::Visual(msg.visual(j)));
        _msgs_model_visual.emplace_back(vm);
    }

    // Set the scale of the model visual
    if(msg.has_scale())
    {
        // update scale using a visual msg
        boost::shared_ptr<gazebo::msgs::Visual> vm(new gazebo::msgs::Visual);
        if(msg.has_id())
            vm->set_id(msg.id());
        if(msg.has_name())
            vm->set_name(msg.name());
        vm->mutable_scale()->set_x(msg.scale().x());
        vm->mutable_scale()->set_y(msg.scale().y());
        vm->mutable_scale()->set_z(msg.scale().z());
        _msgs_model_visual.emplace_back(vm);
    }

    for(int j = 0; j < msg.link_size(); ++j)
    {
        linkName = modelName + msg.link(j).name();

        {
            std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);
            if(msg.link(j).has_pose())
            {
                auto iter = _msgs_pose.find(msg.link(j).id());
                if(iter != _msgs_pose.end())
                    iter->second.CopyFrom(msg.link(j).pose());
                else
                    _msgs_pose.insert(std::make_pair(msg.link(j).id(), msg.link(j).pose()));

                _msgs_pose[msg.link(j).id()].set_name(linkName);
                _msgs_pose[msg.link(j).id()].set_id(msg.link(j).id());
            }
        }

        if(msg.link(j).has_inertial())
        {
            boost::shared_ptr<gazebo::msgs::Link> lm(new gazebo::msgs::Link(msg.link(j)));
            _msgs_link.emplace_back(lm);
        }

        if(msg.link(j).visual_size() > 0)
        {
            // note: the first visual in the link is the link visual
            gazebo::msgs::VisualPtr vm(new gazebo::msgs::Visual(msg.link(j).visual(0)));
            _msgs_link_visual.emplace_back(vm);
        }

        for(int k = 1; k < msg.link(j).visual_size(); ++k)
        {
            boost::shared_ptr<gazebo::msgs::Visual> vm(new gazebo::msgs::Visual(msg.link(j).visual(k)));
            _msgs_visual.emplace_back(vm);
        }
    }

    for(int i = 0; i < msg.model_size(); ++i)
    {
        boost::shared_ptr<gazebo::msgs::Model> mm(new gazebo::msgs::Model(msg.model(i)));
        _msgs_model.emplace_back(mm);
    }

    return true;
}
bool NRPScene::process_scene_msg(ConstScenePtr &msg)
{
    {
        std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);
        for(int i = 0; i < msg->model_size(); ++i)
        {
            auto iter = _msgs_pose.find(msg->model(i).id());
            if(iter != _msgs_pose.end())
                iter->second.CopyFrom(msg->model(i).pose());
            else
                _msgs_pose.insert(std::make_pair(msg->model(i).id(), msg->model(i).pose()));

            _msgs_pose[msg->model(i).id()].set_name(msg->model(i).name());
            _msgs_pose[msg->model(i).id()].set_id(msg->model(i).id());

            this->process_model_msg(msg->model(i));
        }
    }

    for(int i = 0; i < msg->light_size(); ++i)
    {
        boost::shared_ptr<gazebo::msgs::Light> lm(new gazebo::msgs::Light(msg->light(i)));
        _msgs_light_factory.emplace_back(lm);
    }

    if(msg->has_ambient() && _cam_node != nullptr)
    {
        auto pipe = _cam_node->get_pipeline_description();

        pipe->get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP);
        pipe->get_resolve_pass()->environment_lighting(gua::utils::Color3f(msg->ambient().r(), msg->ambient().g(), msg->ambient().b()));
        pipe->get_resolve_pass()->environment_lighting_texture(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/textures/" + NRPConfig::get_instance().get_nrp_sky_map() + "-light.jpg");
        pipe->get_resolve_pass()->touch();
    }

    if(msg->has_background() && _cam_node != nullptr)
    {
        auto pipe = _cam_node->get_pipeline_description();

        pipe->get_resolve_pass()->background_color(gua::utils::Color3f(msg->background().r(), msg->background().g(), msg->background().b()));
        pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
        pipe->get_resolve_pass()->background_texture(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/textures/" + NRPConfig::get_instance().get_nrp_sky_map() + ".jpg");
        pipe->get_resolve_pass()->touch();
    }

    if(msg->has_shadows())
    {
        // ignore shadows
    }

    if(msg->has_grid())
    {
        // ignore grid
    }

    if(msg->has_origin_visual())
    {
        // ignore origin
    }

    if(msg->has_sky())
    {
        // ignore sky
    }

    if(msg->has_fog())
    {
        // ignore fog
    }

    return true;
}
void NRPScene::pre_render()
{
    _root_node->callback_pre_pass();

    bool contains_scene_message = !_msgs_scene.empty();

    {
        std::unique_lock<std::mutex> lock_scene(_mutex_scenegraph);

#if GUA_DEBUG == 1
        auto start = std::chrono::high_resolution_clock::now();
#endif

        scene_msgs_list scene_msgs_copy;
        model_msgs_list model_msgs_copy;
        light_msgs_list light_factory_msgs_copy;
        light_msgs_list light_modify_msgs_copy;
        visual_msgs_list model_visual_msgs_copy;
        visual_msgs_list link_visual_msgs_copy;
        visual_msgs_list visual_msgs_copy;
        link_msgs_list link_msgs_copy;

        {
            std::lock_guard<std::mutex> lock(_mutex_receive);

            std::copy(_msgs_scene.begin(), _msgs_scene.end(), std::back_inserter(scene_msgs_copy));
            _msgs_scene.clear();

            std::copy(_msgs_model.begin(), _msgs_model.end(), std::back_inserter(model_msgs_copy));
            _msgs_model.clear();

            std::copy(_msgs_light_factory.begin(), _msgs_light_factory.end(), std::back_inserter(light_factory_msgs_copy));
            _msgs_light_factory.clear();

            std::copy(_msgs_light_modify.begin(), _msgs_light_modify.end(), std::back_inserter(light_modify_msgs_copy));
            _msgs_light_modify.clear();

            std::copy(_msgs_model_visual.begin(), _msgs_model_visual.end(), std::back_inserter(model_visual_msgs_copy));
            _msgs_model_visual.clear();

            std::copy(_msgs_link_visual.begin(), _msgs_link_visual.end(), std::back_inserter(link_visual_msgs_copy));
            _msgs_link_visual.clear();

            _msgs_visual.sort(VisualMessageLessOp);
            std::copy(_msgs_visual.begin(), _msgs_visual.end(), std::back_inserter(visual_msgs_copy));
            _msgs_visual.clear();

            std::copy(_msgs_link.begin(), _msgs_link.end(), std::back_inserter(link_msgs_copy));
            _msgs_link.clear();
        }

        for(auto scene_msgs_iter = scene_msgs_copy.begin(); scene_msgs_iter != scene_msgs_copy.end();)
        {
            if(this->process_scene_msg(*scene_msgs_iter))
            {
                scene_msgs_copy.erase(scene_msgs_iter++);
            }
            else
                ++scene_msgs_iter;
        }

        if(!model_msgs_copy.empty())
        {
            for(auto model_msgs_iter = model_msgs_copy.begin(); model_msgs_iter != model_msgs_copy.end();)
            {
                if(this->process_model_msg(**model_msgs_iter))
                {
                    const gazebo::msgs::Model &modelMsg = **model_msgs_iter;
                    if(modelMsg.visual_size() > 0)
                    {
                        for(int k = 0; k < modelMsg.visual_size(); k++)
                        {
                            boost::shared_ptr<gazebo::msgs::Visual> msgCopy;
                            msgCopy.reset(new gazebo::msgs::Visual(modelMsg.visual(k)));
                            visual_msgs_copy.emplace_back(msgCopy);
                        }
                    }
                    model_msgs_copy.erase(model_msgs_iter++);
                }
                else
                {
                    ++model_msgs_iter;
                }
            }
        }

        // Process the light factory messages.
        for(auto light_factory_iter = light_factory_msgs_copy.begin(); light_factory_iter != light_factory_msgs_copy.end();)
        {
            if(this->process_light_factory_msg(*light_factory_iter))
            {
                light_factory_msgs_copy.erase(light_factory_iter++);
            }
            else
            {
                ++light_factory_iter;
            }
        }

        // Process the light modify messages.
        for(auto light_modify_iter = light_modify_msgs_copy.begin(); light_modify_iter != light_modify_msgs_copy.end();)
        {
            if(this->process_light_modify_msg(*light_modify_iter))
            {
                light_modify_msgs_copy.erase(light_modify_iter++);
            }
            else
            {
                ++light_modify_iter;
            }
        }

        for(auto model_visual_msgs_iter = model_visual_msgs_copy.begin(); model_visual_msgs_iter != model_visual_msgs_copy.end();)
        {
            if(this->process_visual_msg(*model_visual_msgs_iter, NRPVisual::VT_MODEL))
            {
                model_visual_msgs_copy.erase(model_visual_msgs_iter++);
            }
            else
            {
                ++model_visual_msgs_iter;
            }
        }

        for(auto link_visual_msgs_iter = link_visual_msgs_copy.begin(); link_visual_msgs_iter != link_visual_msgs_copy.end();)
        {
            if(this->process_visual_msg(*link_visual_msgs_iter, NRPVisual::VT_LINK))
            {
                link_visual_msgs_copy.erase(link_visual_msgs_iter++);
            }
            else
            {
                ++link_visual_msgs_iter;
            }
        }

        for(auto visual_msgs_iter = visual_msgs_copy.begin(); visual_msgs_iter != visual_msgs_copy.end();)
        {
            for(visual_msgs_iter = visual_msgs_copy.begin(); visual_msgs_iter != visual_msgs_copy.end();)
            {
                if(this->process_visual_msg(*visual_msgs_iter))
                {
                    visual_msgs_copy.erase(visual_msgs_iter++);
                }
                else
                {
                    ++visual_msgs_iter;
                }
            }
        }

        for(auto link_msgs_iter = link_msgs_copy.begin(); link_msgs_iter != link_msgs_copy.end();)
        {
            if(this->process_link_msg(*link_msgs_iter))
            {
                link_msgs_copy.erase(link_msgs_iter++);
            }
            else
            {
                ++link_msgs_iter;
            }
        }

        {
            std::lock_guard<std::mutex> lock(_mutex_receive);

            std::copy(scene_msgs_copy.begin(), scene_msgs_copy.end(), std::front_inserter(_msgs_scene));
            std::copy(model_msgs_copy.begin(), model_msgs_copy.end(), std::front_inserter(_msgs_model));
            std::copy(light_factory_msgs_copy.begin(), light_factory_msgs_copy.end(), std::front_inserter(_msgs_light_factory));
            std::copy(light_modify_msgs_copy.begin(), light_modify_msgs_copy.end(), std::front_inserter(_msgs_light_modify));
            std::copy(model_visual_msgs_copy.begin(), model_visual_msgs_copy.end(), std::front_inserter(_msgs_model_visual));
            std::copy(link_visual_msgs_copy.begin(), link_visual_msgs_copy.end(), std::front_inserter(_msgs_link_visual));
            std::copy(visual_msgs_copy.begin(), visual_msgs_copy.end(), std::front_inserter(_msgs_visual));
            std::copy(link_msgs_copy.begin(), link_msgs_copy.end(), std::front_inserter(_msgs_link));
        }

        {
            std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);

            // Process all the model messages last. Remove pose message from the list
            // only when a corresponding visual exits. We may receive pose updates
            // over the wire before  we recieve the visual
            auto pose_msgs_iter = _msgs_pose.begin();
            while(pose_msgs_iter != _msgs_pose.end())
            {
                auto iter = _visuals.find(pose_msgs_iter->first);
                if(iter != _visuals.end() && iter->second)
                {
                    ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(pose_msgs_iter->second);
                    iter->second->set_pose(pose);

                    auto prev = pose_msgs_iter++;
                    _msgs_pose.erase(prev);
                }
                else
                    ++pose_msgs_iter;
            }

            auto skeleton_pose_iter = _msgs_skeleton_pose.begin();
            while(skeleton_pose_iter != _msgs_skeleton_pose.end())
            {
                auto iter = _visuals.find((*skeleton_pose_iter)->model_id());
                for(int i = 0; i < (*skeleton_pose_iter)->pose_size(); ++i)
                {
                    const gazebo::msgs::Pose &pose_msg = (*skeleton_pose_iter)->pose(i);
                    if(pose_msg.has_id())
                    {
                        auto iter2 = _visuals.find(pose_msg.id());
                        if(iter2 != _visuals.end())
                        {
                            ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(pose_msg);
                            iter2->second->set_pose(pose);
                        }
                    }
                }

                if(iter != _visuals.end())
                {
                    // TODO: review the necessity of the method
                    // iter->second->set_skeleton_pose(**skeleton_pose_iter);
                    auto prev = skeleton_pose_iter++;
                    _msgs_skeleton_pose.erase(prev);
                }
                else
                {
                    ++skeleton_pose_iter;
                }
            }
        }
    }

    bool contains_hierarchy = !_world_visual->get_node()->get_children().empty();

    if(_is_root_not_initialized && contains_scene_message && contains_hierarchy)
    {
        _root_node->set_should_update_avango(true);

        _is_root_not_initialized = false;
    }

    _root_node->callback_post_pass();

#if GUA_DEBUG == 1
    auto end = std::chrono::high_resolution_clock::now();
    float pre_render_time = std::chrono::duration<float, std::milli>(end - start).count();

    std::cout << "pre_render_time: " << pre_render_time << std::endl;
#endif
}
std::mutex &NRPScene::get_mutex_scenegraph() { return _mutex_scenegraph; }
NRPInteractiveNode *NRPScene::get_interactive_node() const { return _interactive_node; }
NRPNode *NRPScene::get_root_node() const { return _root_node; }
} // namespace nrp
} // namespace gua