#include "GuaDynGeoVisualPlugin.hpp"

namespace gazebo
{
class GuaDynGeoVisualPluginPrivate
{
  public:
    rendering::VisualPtr visual;
    event::ConnectionPtr updateConnection;

    common::Color colorA;

    common::Color colorB;
    common::Time period;

    common::Time cycleStartTime;

    common::Time currentSimTime;

    transport::NodePtr node;

    std::mutex mutex;

    bool useWallTime;

    transport::SubscriberPtr infoSub;
};
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(GuaDynGeoVisualPlugin)

GuaDynGeoVisualPlugin::GuaDynGeoVisualPlugin() : dataPtr(new GuaDynGeoVisualPluginPrivate) {}

GuaDynGeoVisualPlugin::~GuaDynGeoVisualPlugin()
{
    this->dataPtr->infoSub.reset();
    if(this->dataPtr->node)
        this->dataPtr->node->Fini();
}

/////////////////////////////////////////////////
void GuaDynGeoVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
    if(!_visual || !_sdf)
    {
        gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
        return;
    }
    this->dataPtr->visual = _visual;

    // Get color A
    this->dataPtr->colorA.Set(1, 0, 0, 1);
    if(_sdf->HasElement("color_a"))
        this->dataPtr->colorA = _sdf->Get<common::Color>("color_a");

    // Get color B
    this->dataPtr->colorB.Set(0, 0, 0, 1);
    if(_sdf->HasElement("color_b"))
        this->dataPtr->colorB = _sdf->Get<common::Color>("color_b");

    // Get the period
    this->dataPtr->period.Set(1);
    if(_sdf->HasElement("period"))
        this->dataPtr->period = _sdf->Get<double>("period");

    if(this->dataPtr->period <= 0)
    {
        gzerr << "Period can't be lower than zero." << std::endl;
        return;
    }

    // Get whether to use wall time or sim time
    this->dataPtr->useWallTime = false;
    if(_sdf->HasElement("use_wall_time"))
        this->dataPtr->useWallTime = _sdf->Get<bool>("use_wall_time");

    // Connect to the world update signal
    this->dataPtr->updateConnection = event::Events::ConnectPreRender(std::bind(&GuaDynGeoVisualPlugin::Update, this));

    // Subscribe to world statistics to get sim time
    // Warning: topic ~/pose/local/info is meant for high-bandwidth local
    // network access. It will kill the system if a remote gzclient tries to
    // subscribe.
    if(!this->dataPtr->useWallTime)
    {
        this->dataPtr->node = transport::NodePtr(new transport::Node());
        this->dataPtr->node->Init();

        this->dataPtr->infoSub = this->dataPtr->node->Subscribe("~/pose/local/info", &GuaDynGeoVisualPlugin::OnInfo, this);
    }
}

/////////////////////////////////////////////////
void GuaDynGeoVisualPlugin::Update()
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    if(!this->dataPtr->visual)
    {
        gzerr << "The visual is null." << std::endl;
        return;
    }

    common::Time currentTime;
    if(this->dataPtr->useWallTime)
        currentTime = common::Time::GetWallTime();
    else
        currentTime = this->dataPtr->currentSimTime;

    if(this->dataPtr->cycleStartTime == common::Time::Zero || this->dataPtr->cycleStartTime > currentTime)
    {
        this->dataPtr->cycleStartTime = currentTime;
    }

    auto elapsed = currentTime - this->dataPtr->cycleStartTime;

    // Restart cycle
    if(elapsed >= this->dataPtr->period)
        this->dataPtr->cycleStartTime = currentTime;

    common::Color from;
    common::Color to;
    // Color A -> B
    if(elapsed < this->dataPtr->period * 0.5)
    {
        from = this->dataPtr->colorA;
        to = this->dataPtr->colorB;
    }
    // Color B -> A
    else if(elapsed >= this->dataPtr->period * 0.5)
    {
        from = this->dataPtr->colorB;
        to = this->dataPtr->colorA;
        elapsed -= this->dataPtr->period * 0.5;
    }

    // interpolate each color component
    double pos = (elapsed / (this->dataPtr->period * 0.5)).Double();

    double red = from.r + (to.r - from.r) * pos;
    double green = from.g + (to.g - from.g) * pos;
    double blue = from.b + (to.b - from.b) * pos;
    double alpha = from.a + (to.a - from.a) * pos;

    common::Color color(red, green, blue, alpha);

    this->dataPtr->visual->SetDiffuse(color);
    this->dataPtr->visual->SetAmbient(color);
    this->dataPtr->visual->SetTransparency(1 - color.a);
}

/////////////////////////////////////////////////
void GuaDynGeoVisualPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->currentSimTime = msgs::Convert(_msg->time());
}