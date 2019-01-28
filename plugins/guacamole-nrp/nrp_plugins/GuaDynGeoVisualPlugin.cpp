#include "GuaDynGeoVisualPlugin.hpp"

namespace gazebo
{
GZ_REGISTER_VISUAL_PLUGIN(GuaDynGeoVisualPlugin)

GuaDynGeoVisualPlugin::GuaDynGeoVisualPlugin()
{
    gzerr << "DynGeo: constructor" << std::endl;
    std::cerr << "DynGeo: constructor" << std::endl;
}

GuaDynGeoVisualPlugin::~GuaDynGeoVisualPlugin() {}

void GuaDynGeoVisualPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
    gzerr << "DynGeo: load before" << std::endl;
    std::cerr << "DynGeo: load before" << std::endl;

    if(!visual || !sdf)
    {
        gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
        return;
    }

    _visual = visual;
    _update_connection = event::Events::ConnectPreRender(std::bind(&GuaDynGeoVisualPlugin::Update, this));

    gzerr << "DynGeo: load after" << std::endl;
    std::cerr << "DynGeo: load after" << std::endl;
}

void GuaDynGeoVisualPlugin::AddTriangle()
{
    _scene_node = _visual->GetSceneNode();
    _scene_manager = _scene_node->getCreator();

    if(!_scene_node || !_scene_manager)
    {
        return;
    }

    gzerr << "DynGeo: scene manager acquired" << std::endl;
    std::cerr << "DynGeo: scene manager acquired" << std::endl;

    Ogre::ManualObject *man = _scene_manager->createManualObject("test");

    man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    man->position(-2, 2, 2);
    man->normal(0, 0, 1);
    man->textureCoord(0, 0);

    man->position(-2, -2, 2);
    man->normal(0, 0, 1);
    man->textureCoord(0, 1);

    man->position(2, -2, -2);
    man->normal(0, 0, 1);
    man->textureCoord(1, 0);

    man->triangle(0, 1, 2);
    man->triangle(1, 0, 2);
    man->end();

    man->setVisible(true);

    gzerr << "DynGeo: triangle added" << std::endl;
    std::cerr << "DynGeo: triangle added" << std::endl;

    gzerr << "DynGeo: vertex count " << man->getCurrentVertexCount() << std::endl;
    std::cerr << "DynGeo: vertex count " << man->getCurrentVertexCount() << std::endl;

    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    // test access to scene
    const Ogre::ColourValue ambient(r, g, b, 1.f);
    _scene_manager->setAmbientLight(ambient);

    gzerr << "DynGeo: test values written" << std::endl;
    std::cerr << "DynGeo: test values written" << std::endl;

    _scene_node->createChildSceneNode("dyngeochild")->attachObject(man);
}
void GuaDynGeoVisualPlugin::Update()
{
    if(_callback_count % 1000 == 0)
    {
        AddTriangle();
    }

    _callback_count++;

    /*gzerr << "DynGeo: pre-render update before" << std::endl;
    std::cerr << "DynGeo: pre-render update before" << std::endl;

    gzerr << "DynGeo: pre-render update after" << std::endl;
    std::cerr << "DynGeo: pre-render update after" << std::endl;*/
}
}
