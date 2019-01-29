#include "GuaDynGeoVisualPlugin.hpp"
#include "sgtp/SGTP.h"

namespace gazebo
{
GZ_REGISTER_VISUAL_PLUGIN(GuaDynGeoVisualPlugin)

GuaDynGeoVisualPlugin::GuaDynGeoVisualPlugin() : _buffer_rcv(SGTP::MAX_MESSAGE_SIZE), _faces(1000000), _is_need_swap(false), _is_recv_running(true), _mutex_swap()
{
    gzerr << "DynGeo: constructor" << std::endl;
    std::cerr << "DynGeo: constructor" << std::endl;
}

GuaDynGeoVisualPlugin::~GuaDynGeoVisualPlugin()
{
    _is_recv_running.store(false);
    _thread_recv.join();
}

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

    std::generate(_faces.begin(), _faces.end(), [n = 0]() mutable { return n++; });

    _thread_recv = std::thread([&]() { _ReadLoop(); });
}
void GuaDynGeoVisualPlugin::_ReadLoop()
{
    gzerr << "DynGeo: _ReadLoop" << std::endl;
    std::cerr << "DynGeo: _ReadLoop" << std::endl;

    zmq::context_t ctx(1);
    zmq::socket_t socket(ctx, ZMQ_SUB);

    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
#if ZMQ_VERSION_MAJOR < 3
    int64_t hwm = 1;
    socket.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
#else
    int hwm = 1;
    socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
#endif

    std::string endpoint("tcp://141.54.147.29:7050");
    socket.connect(endpoint.c_str());

    while(_is_recv_running.load())
    {
        zmq::message_t zmqm;
        socket.recv(&zmqm);

        gzerr << std::endl << "DynGeo: socket.recv" << std::endl;
        std::cerr << std::endl << "DynGeo: socket.recv" << std::endl;

        while(true)
        {
            std::lock_guard<std::mutex> lock(_mutex_swap);

            if(!_is_need_swap.load() || !_is_recv_running.load())
            {
                break;
            }
        }

        gzerr << std::endl << "DynGeo: memcpy" << std::endl;
        std::cerr << std::endl << "DynGeo: memcpy" << std::endl;

        SGTP::header_data_t header;
        memcpy(&header, (unsigned char *)zmqm.data(), SGTP::HEADER_BYTE_SIZE);

        // TODO: textures

        _num_geometry_bytes = SGTP::get_num_geometry_bytes(header);
        memcpy(&_bb_min, &header.global_bb_min, sizeof(float) * 3);
        memcpy(&_bb_max, &header.global_bb_max, sizeof(float) * 3);
        memcpy(&_buffer_rcv[0], (unsigned char *)zmqm.data() + SGTP::get_geometry_read_offset(header), SGTP::get_num_geometry_bytes(header));

        {
            std::lock_guard<std::mutex> lock(_mutex_swap);
            _is_need_swap.store(true);
        }
    }
}
void GuaDynGeoVisualPlugin::AddTriangleSoup()
{
    _scene_node = _visual->GetSceneNode();
    _scene_manager = _scene_node->getCreator();

    if(!_scene_node || !_scene_manager)
    {
        return;
    }

    gzerr << "DynGeo: scene manager acquired" << std::endl;
    std::cerr << "DynGeo: scene manager acquired" << std::endl;

    size_t num_vertices = _num_geometry_bytes / sizeof(float) / 5;
    size_t faces = num_vertices / 3;

    gzerr << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
    std::cerr << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;

    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(std::to_string(rand()), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    mesh->_setBounds(Ogre::AxisAlignedBox({_bb_min[0], _bb_min[1], _bb_min[2]}, {_bb_max[0], _bb_max[1], _bb_max[2]}));

    mesh->sharedVertexData = new Ogre::VertexData();
    mesh->sharedVertexData->vertexCount = num_vertices;
    Ogre::VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
    Ogre::VertexBufferBinding *bind = mesh->sharedVertexData->vertexBufferBinding;

    size_t offset = 0;
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

    Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(offset, num_vertices, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    vbuf->writeData(0, vbuf->getSizeInBytes(), &_buffer_rcv[0], true);
    bind->setBinding(0, vbuf);
    Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, faces, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    ibuf->writeData(0, ibuf->getSizeInBytes(), &_faces[0], true);

    Ogre::SubMesh *sub = mesh->createSubMesh();
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = faces;
    sub->indexData->indexStart = 0;

    // _scene_node->createChildSceneNode(std::to_string(rand()))->attachObject(mesh);

    /// Manual Object
    /*Ogre::ManualObject *man = _scene_manager->createManualObject(std::to_string(rand()));

    man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    man->setDynamic(true);

    man->position(-2.f, 2.f, 2.f);
    man->normal(0, 0, 1.f);
    man->textureCoord(0, 0);

    man->position(-2.f, -2.f, 2.f);
    man->normal(0, 0, 1.f);
    man->textureCoord(0, 1.f);

    man->position(2.f, -2.f, -2.f);
    man->normal(0, 0, 1.f);
    man->textureCoord(1.f, 0);

    gzerr << "DynGeo: vertex count " << man->getCurrentVertexCount() << std::endl;
    std::cerr << "DynGeo: vertex count " << man->getCurrentVertexCount() << std::endl;

    man->triangle(0, 1, 2);
    man->triangle(1, 0, 2);
    auto section_ptr = man->end();


    gzerr << "DynGeo: section ptr is null " << (section_ptr == nullptr) << std::endl;
    std::cerr << "DynGeo: section ptr is null " << (section_ptr == nullptr) << std::endl;

    man->setVisible(true);
     _scene_node->createChildSceneNode(std::to_string(rand()))->attachObject(man);
     */

    gzerr << "DynGeo: triangles added" << std::endl;
    std::cerr << "DynGeo: triangles added" << std::endl;

    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    // test access to scene
    const Ogre::ColourValue ambient(r, g, b, 1.f);
    _scene_manager->setAmbientLight(ambient);

    gzerr << "DynGeo: test values written" << std::endl;
    std::cerr << "DynGeo: test values written" << std::endl;
}
void GuaDynGeoVisualPlugin::RemoveTriangleSoup() {
    // TODO: remove meshes
}
void GuaDynGeoVisualPlugin::Update()
{
    /*gzerr << std::endl << "DynGeo: pre-render update before" << std::endl;
    std::cerr << std::endl << "DynGeo: pre-render update before" << std::endl;*/

    {
        std::lock_guard<std::mutex> lock(_mutex_swap);
        if(_is_need_swap.load())
        {
            RemoveTriangleSoup();
            AddTriangleSoup();
            _is_need_swap.store(false);
        }
    }

    /*gzerr << std::endl << "DynGeo: pre-render update after" << std::endl;
    std::cerr << std::endl << "DynGeo: pre-render update after" << std::endl;*/
}
}
