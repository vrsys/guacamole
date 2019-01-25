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

/////////////////////////////////////////////////
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

    gzerr << "DynGeo: update connection created" << std::endl;
    std::cerr << "DynGeo: update connection created" << std::endl;

    Ogre::SceneNode *scene_node = _visual->GetSceneNode();
    Ogre::SceneManager *scene_manager = scene_node->getCreator();

    gzerr << "DynGeo: scene manager acquired" << std::endl;
    std::cerr << "DynGeo: scene manager acquired" << std::endl;

    Ogre::ManualObject *man = scene_manager->createManualObject("test");
    man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    man->position(-2, 2, 2);
    man->normal(0, 0, 1);
    man->textureCoord(0, 0);
    man->position(-2, -2, 2);
    man->normal(0, 0, 1);
    man->textureCoord(0, 1);
    man->position(2, -2, 2);
    man->normal(0, 0, 1);
    man->textureCoord(1, 1);
    man->position(2, 2, 2);
    man->normal(0, 0, 1);
    man->textureCoord(1, 0);
    man->quad(0, 1, 2, 3);
    man->end();

    gzerr << "DynGeo: quad added" << std::endl;
    std::cerr << "DynGeo: quad added" << std::endl;

    // test access to scene
    const Ogre::ColourValue ambient(1.f,0.f,0.f,1.f);
    scene_manager->setAmbientLight(ambient);
    scene_manager->setDisplaySceneNodes(false);

    gzerr << "DynGeo: test values written" << std::endl;
    std::cerr << "DynGeo: test values written" << std::endl;

    scene_node->createChildSceneNode("dyngeochild")->attachObject(man);

    gzerr << "DynGeo: load after" << std::endl;
    std::cerr << "DynGeo: load after" << std::endl;
}

/////////////////////////////////////////////////
void GuaDynGeoVisualPlugin::Update()
{
    // gzerr << "DynGeo: pre-render update before" << std::endl;
    // std::cerr << "DynGeo: pre-render update before" << std::endl;

    if(!_visual)
    {
        gzerr << "The visual is null." << std::endl;
        return;
    }

    common::Color color(1., 0., 0., 1.);

    _visual->SetDiffuse(color);
    _visual->SetAmbient(color);

    // gzerr << "DynGeo: pre-render update before" << std::endl;
    // std::cerr << "DynGeo: pre-render update before" << std::endl;
}
}
