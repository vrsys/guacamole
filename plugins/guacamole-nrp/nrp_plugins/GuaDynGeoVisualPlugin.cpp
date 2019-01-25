#include "GuaDynGeoVisualPlugin.hpp"

namespace gazebo
{

GZ_REGISTER_VISUAL_PLUGIN(GuaDynGeoVisualPlugin)

GuaDynGeoVisualPlugin::GuaDynGeoVisualPlugin() {
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

    gzerr << "DynGeo: load after" << std::endl;
    std::cerr << "DynGeo: load after" << std::endl;
}

/////////////////////////////////////////////////
void GuaDynGeoVisualPlugin::Update()
{
    //gzerr << "DynGeo: pre-render update before" << std::endl;
    //std::cerr << "DynGeo: pre-render update before" << std::endl;

    if(!_visual)
    {
        gzerr << "The visual is null." << std::endl;
        return;
    }

    common::Color color(1., 0., 0., 1.);

    _visual->SetDiffuse(color);
    _visual->SetAmbient(color);

    //gzerr << "DynGeo: pre-render update before" << std::endl;
    //std::cerr << "DynGeo: pre-render update before" << std::endl;
}

}
