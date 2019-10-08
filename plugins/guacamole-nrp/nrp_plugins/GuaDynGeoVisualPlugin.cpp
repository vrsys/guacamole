#include "GuaDynGeoVisualPlugin.hpp"

namespace gazebo
{
GZ_REGISTER_VISUAL_PLUGIN(GuaDynGeoVisualPlugin)

#define GUA_DEBUG 0
#define TEX_DEBUG 0
#define MAX_VERTS 1000000

using namespace Ogre;

GuaDynGeoVisualPlugin::GuaDynGeoVisualPlugin() : _mutex_swap(), _mutex_recv(), _mutex_recv_swap(), _cv_recv(), _cv_recv_swap()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: constructor" << std::endl;
    std::cerr << std::endl << "DynGeo: constructor" << std::endl;
#endif

    _buffer_rcv = std::vector<unsigned char>(SGTP::MAX_MESSAGE_SIZE);
    _buffer_rcv_inflated = std::vector<unsigned char>(MAX_VERTS * sizeof(float) * 5);
    _buffer_rcv_texture = std::vector<unsigned char>(SGTP::MAX_MESSAGE_SIZE);
    _buffer_rcv_texture_decompressed = std::vector<unsigned char>(SGTP::MAX_MESSAGE_SIZE);
    _buffer_index = std::vector<int32_t>(MAX_VERTS);

    _jpeg_decompressor_per_layer = std::unordered_map<uint32_t, tjhandle>();

    _is_initialized.store(false);
    _is_need_swap.store(false);
    _is_recv_running.store(true);

    _texture_name = "";
    _material_name = "";
    _mesh_name = "";
    _submesh_name = "";
    _entity_name = "";
}

GuaDynGeoVisualPlugin::~GuaDynGeoVisualPlugin()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: destructor" << std::endl;
    std::cerr << std::endl << "DynGeo: destructor" << std::endl;
#endif

    _is_recv_running.store(false);
    _cv_recv.notify_one();
    _cv_recv_swap.notify_one();

    MaterialManager::getSingleton().remove(_material_name);
    TextureManager::getSingleton().remove(_texture_name);
    MeshManager::getSingleton().remove(_mesh_name);
}

void GuaDynGeoVisualPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: load before" << std::endl;
    std::cerr << std::endl << "DynGeo: load before" << std::endl;
#endif

    if(!visual)
    {
        gzerr << std::endl << "No visual element specified. Plugin won't load." << std::endl;
        return;
    }

    _visual = visual;

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: load after" << std::endl;
    std::cerr << std::endl << "DynGeo: load after" << std::endl;
#endif
}
void GuaDynGeoVisualPlugin::Init()
{
    if(_is_initialized.load())
    {
        return;
    }

    _scene_node = _visual->GetSceneNode();
    _scene_manager = _scene_node->getCreator();

    _update_connection = event::Events::ConnectPreRender(std::bind(&GuaDynGeoVisualPlugin::Update, this));

    std::iota(_buffer_index.begin(), _buffer_index.end(), 0);

    for(size_t i = 0; i < _buffer_index.size() - 2; i += 3)
    {
        int32_t swapSpace = _buffer_index[i + 2];
        _buffer_index[i + 2] = _buffer_index[i + 1];
        _buffer_index[i + 1] = swapSpace;
    }

    _texture_name = std::to_string(rand());

    _texture_width = 4096;

    TexturePtr texture = TextureManager::getSingleton().createManual(_texture_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, TEX_TYPE_2D, _texture_width, _texture_width, 0, PF_BYTE_BGR,
                                                                     TU_DYNAMIC_WRITE_ONLY);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: texture created" << std::endl;
    std::cerr << std::endl << "DynGeo: texture created" << std::endl;
#endif

#if TEX_DEBUG == 1

    HardwarePixelBufferSharedPtr pixel_buffer = texture->getBuffer();

    pixel_buffer->lock(Image::Box(0, 0, 1280, 720), HardwareBuffer::HBL_WRITE_ONLY);
    const PixelBox &pixel_box_1 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_1.getTopLeftFrontPixelPtr(), 0x20, PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    uint32_t *data = static_cast<uint32_t *>(pixel_box_1.data);
    for(int y = 0; y < pixel_box_1.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_1.getWidth(); ++x)
        {
            data[pixel_box_1.rowPitch * y + x] = 0x20202020;
        }
    }
    pixel_buffer->unlock();

    pixel_buffer->lock(Image::Box(0, 720, 1280, 1440), HardwareBuffer::HBL_WRITE_ONLY);
    const PixelBox &pixel_box_2 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_2.getTopLeftFrontPixelPtr(), 0x80, PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    data = static_cast<uint32_t *>(pixel_box_2.data);
    for(int y = 0; y < pixel_box_2.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_2.getWidth(); ++x)
        {
            data[pixel_box_2.rowPitch * y + x] = 0x80808080;
        }
    }
    pixel_buffer->unlock();

    pixel_buffer->lock(Image::Box(1280, 0, 2560, 720), HardwareBuffer::HBL_WRITE_ONLY);
    const PixelBox &pixel_box_3 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_3.getTopLeftFrontPixelPtr(), 0xC0, PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    data = static_cast<uint32_t *>(pixel_box_3.data);
    for(int y = 0; y < pixel_box_3.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_3.getWidth(); ++x)
        {
            data[pixel_box_3.rowPitch * y + x] = 0xC0C0C0C0;
        }
    }
    pixel_buffer->unlock();

    pixel_buffer->lock(Image::Box(1280, 720, 2560, 1440), HardwareBuffer::HBL_WRITE_ONLY);
    const PixelBox &pixel_box_4 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_4.getTopLeftFrontPixelPtr(), 0xFF, PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    data = static_cast<uint32_t *>(pixel_box_4.data);
    for(int y = 0; y < pixel_box_4.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_4.getWidth(); ++x)
        {
            data[pixel_box_4.rowPitch * y + x] = 0xFFFFFFFF;
        }
    }
    pixel_buffer->unlock();

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: PB updated" << std::endl;
    std::cerr << std::endl << "DynGeo: PB updated" << std::endl;
#endif

#endif

    _material_name = std::to_string(rand());

    MaterialPtr material = MaterialManager::getSingleton().create(_material_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

    material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material->getTechnique(0)->getPass(0)->createTextureUnitState(_texture_name);
    material->getTechnique(0)->getPass(0)->setSceneBlending(SBT_REPLACE);
    material->getTechnique(0)->getPass(0)->setCullingMode(CullingMode::CULL_NONE);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: material set" << std::endl;
    std::cerr << std::endl << "DynGeo: material set" << std::endl;
#endif

    {
        _scene_manager->sceneGraphMutex.lock();

        _avatar_node = _scene_node->createChildSceneNode(std::to_string(rand()));

        _scene_node->setVisible(false, false);
        _avatar_node->setVisible(true, true);
        _avatar_node->showBoundingBox(false);

        _scene_manager->sceneGraphMutex.unlock();
    }

    _is_recv_running.store(true);
    _thread_recv = std::thread([&]() { _ReadLoop(); });

    _is_initialized.store(true);
}
void GuaDynGeoVisualPlugin::_ReadLoop()
{
    _thread_recv.detach();

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: _ReadLoop" << std::endl;
    std::cerr << std::endl << "DynGeo: _ReadLoop" << std::endl;
#endif

    zmq::context_t ctx(1, 1);
    zmq::socket_t socket(ctx, ZMQ_SUB);

    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
#if ZMQ_VERSION_MAJOR < 3
    int64_t hwm = 1;
    socket.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
#else
    int hwm = 1;
    socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
#endif

    /// CONFLATE
    int conflate = 1;
    socket.setsockopt(ZMQ_CONFLATE, &conflate, sizeof(conflate));

    std::string endpoint("tcp://141.54.147.29:7050");
    socket.connect(endpoint.c_str());

    _tj_compressed_image_buffer = tjAlloc(_texture_width * _texture_width * 50);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: tjAlloc complete" << std::endl;
    std::cerr << std::endl << "DynGeo: tjAlloc complete" << std::endl;
#endif

    std::unique_lock<std::mutex> lk(_mutex_recv);
    while(_cv_recv.wait_for(lk, std::chrono::milliseconds(16), [&] { return _is_recv_running.load(); }))
    {
        zmq::message_t zmqm;
        socket.recv(&zmqm);

#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: socket.recv" << std::endl;
        std::cerr << std::endl << "DynGeo: socket.recv" << std::endl;
#endif

        {
            std::unique_lock<std::mutex> lk_recv_swap(_mutex_recv_swap);
            _cv_recv_swap.wait(lk_recv_swap, [&]() { return !_is_need_swap.load() || !_is_recv_running.load(); });
        }

#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: memcpy" << std::endl;
        std::cerr << std::endl << "DynGeo: memcpy" << std::endl;
#endif

        {
            std::unique_lock<std::mutex> lk_swap(_mutex_swap);

            SGTP::header_data_t header;
            memcpy(&header, (unsigned char *)zmqm.data(), SGTP::HEADER_BYTE_SIZE);
            memcpy(&_texture_bounding_boxes, (unsigned char *)header.tex_bounding_box, sizeof(header.tex_bounding_box));

            size_t geometry_payload = zmqm.size() - SGTP::HEADER_BYTE_SIZE - header.texture_payload_size;

            /*#if GUA_DEBUG == 1
                        gzerr << std::endl << "DynGeo: geometry_payload " << std::to_string(geometry_payload) << " header.geometry_payload_size " << header.geometry_payload_size << std::endl;
                        std::cerr << std::endl << "DynGeo: geometry_payload " << std::to_string(geometry_payload) << " header.geometry_payload_size " << header.geometry_payload_size << std::endl;
            #endif*/

            memcpy(&_bb_min, &header.global_bb_min, sizeof(float) * 3);
            memcpy(&_bb_max, &header.global_bb_max, sizeof(float) * 3);
            memcpy(&_buffer_rcv[0], (unsigned char *)zmqm.data() + SGTP::HEADER_BYTE_SIZE, geometry_payload);
            memcpy(&_buffer_rcv_texture[0], (unsigned char *)zmqm.data() + SGTP::HEADER_BYTE_SIZE + geometry_payload, header.texture_payload_size);

            _num_geometry_bytes = (size_t)LZ4_decompress_safe((const char *)_buffer_rcv.data(), (char *)&_buffer_rcv_inflated[0], (int)geometry_payload, MAX_VERTS * sizeof(float) * 5);

#if GUA_DEBUG == 1
            gzerr << std::endl << "DynGeo: decompressed LZ4" << std::endl;
            std::cerr << std::endl << "DynGeo: decompressed LZ4" << std::endl;
#endif

            std::size_t byte_offset_to_current_image = 0;
            std::size_t decompressed_image_offset = 0;

            for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < 4; sensor_layer_idx++)
            {
#if GUA_DEBUG == 1
                gzerr << std::endl << "DynGeo: sensor_layer_idx " << std::to_string(sensor_layer_idx) << std::endl;
                std::cerr << std::endl << "DynGeo: sensor_layer_idx " << std::to_string(sensor_layer_idx) << std::endl;
#endif

                if(_jpeg_decompressor_per_layer.find(sensor_layer_idx) == _jpeg_decompressor_per_layer.end())
                {
                    _jpeg_decompressor_per_layer[sensor_layer_idx] = tjInitDecompress();
                    if(_jpeg_decompressor_per_layer[sensor_layer_idx] == nullptr)
                    {
                        gzerr << std::endl << "DynGeo: ERROR INITIALIZING DECOMPRESSOR" << std::endl;
                        std::cerr << std::endl << "DynGeo: ERROR INITIALIZING DECOMPRESSOR" << std::endl;
                    }
                }

                uint32_t jpeg_size = header.jpeg_bytes_per_sensor[sensor_layer_idx];

                memcpy((char *)&_tj_compressed_image_buffer[byte_offset_to_current_image], (char *)&_buffer_rcv_texture[byte_offset_to_current_image], jpeg_size);

                int header_width, header_height, header_subsamp;

                auto &current_decompressor_handle = _jpeg_decompressor_per_layer[sensor_layer_idx];

                int error_handle =
                    tjDecompressHeader2(current_decompressor_handle, &_tj_compressed_image_buffer[byte_offset_to_current_image], jpeg_size, &header_width, &header_height, &header_subsamp);

                if(-1 == error_handle)
                {
#if GUA_DEBUG == 1
                    gzerr << std::endl << "DynGeo: ERROR DECOMPRESSING JPEG: " << tjGetErrorStr() << std::endl;
                    std::cerr << std::endl << "DynGeo: ERROR DECOMPRESSING JPEG: " << tjGetErrorStr() << std::endl;
#endif
                }

                tjDecompress2(current_decompressor_handle, &_tj_compressed_image_buffer[byte_offset_to_current_image], jpeg_size, &_buffer_rcv_texture_decompressed[decompressed_image_offset],
                              header_width, 0, header_height, TJPF_BGR, TJFLAG_FASTDCT);

                uint32_t copied_image_byte = (uint32_t)(header_height * header_width * 3);

                byte_offset_to_current_image += jpeg_size;
                decompressed_image_offset += copied_image_byte;
            }

#if GUA_DEBUG == 1
            gzerr << std::endl << "DynGeo: decompressed JPEG" << std::endl;
            std::cerr << std::endl << "DynGeo: decompressed JPEG" << std::endl;
#endif

            _is_need_swap.store(true);
        }

        /*#if GUA_DEBUG == 1
                gzerr << "DynGeo: geometry bytes " << header.geometry_payload_size << std::endl;
                std::cerr << "DynGeo: geometry bytes " << header.geometry_payload_size << std::endl;
        #endif

        #if GUA_DEBUG == 1
                gzerr << "DynGeo: texture payload " << header.texture_payload_size << std::endl;
                std::cerr << "DynGeo: texture payload " << header.texture_payload_size << std::endl;
        #endif*/
    }

    tjFree(_tj_compressed_image_buffer);
}
void GuaDynGeoVisualPlugin::UpdateTriangleSoup()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: UpdateTriangleSoup" << std::endl;
    std::cerr << std::endl << "DynGeo: UpdateTriangleSoup" << std::endl;
#endif

    if(!_visual || !_scene_node || !_scene_manager)
    {
        return;
    }

    _scene_manager->sceneGraphMutex.lock();

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: lock acquired" << std::endl;
    std::cerr << std::endl << "DynGeo: lock acquired" << std::endl;
#endif

    size_t texture_offset = 0;

    for(auto texture_bounding_box : _texture_bounding_boxes)
    {
        if(texture_bounding_box.min.u == texture_bounding_box.max.u)
        {
            continue;
        }

        /*#if GUA_DEBUG == 1
                gzerr << std::endl << "DynGeo: min x" << texture_bounding_box.min.u << " y " << texture_bounding_box.min.v << std::endl;
                std::cerr << std::endl << "DynGeo: min x" << texture_bounding_box.min.u << " y " << texture_bounding_box.min.v << std::endl;
        #endif

        #if GUA_DEBUG == 1
                gzerr << std::endl << "DynGeo: max x" << texture_bounding_box.max.u << " y " << texture_bounding_box.max.v << std::endl;
                std::cerr << std::endl << "DynGeo: max x" << texture_bounding_box.max.u << " y " << texture_bounding_box.max.v << std::endl;
        #endif*/

        HardwarePixelBufferSharedPtr pixel_buffer = TextureManager::getSingleton().getByName(_texture_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME)->getBuffer();

        pixel_buffer->lock(Image::Box(texture_bounding_box.min.u, texture_bounding_box.min.v, texture_bounding_box.max.u + 1, texture_bounding_box.max.v + 1), HardwareBuffer::HBL_WRITE_ONLY);
        const PixelBox &pixel_box = pixel_buffer->getCurrentLock();

        auto *data = static_cast<uint32_t *>(pixel_box.data);
        for(unsigned int y = 0; y < pixel_box.getHeight(); ++y)
        {
            for(unsigned int x = 0; x < pixel_box.getWidth(); ++x)
            {
                data[pixel_box.rowPitch * y + x] = (uint32_t)0xFF << 24 | (uint32_t)_buffer_rcv_texture_decompressed[texture_offset + (pixel_box.getWidth() * y + x) * 3 + 2] << 16 |
                                                   (uint32_t)_buffer_rcv_texture_decompressed[texture_offset + (pixel_box.getWidth() * y + x) * 3 + 1] << 8 |
                                                   (uint32_t)_buffer_rcv_texture_decompressed[texture_offset + (pixel_box.getWidth() * y + x) * 3 + 0];
            }
        }

        texture_offset += pixel_box.getWidth() * pixel_box.getHeight() * 3;

        pixel_buffer->unlock();
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: texture updated" << std::endl;
    std::cerr << std::endl << "DynGeo: texture updated" << std::endl;
#endif

    if(_avatar_node->numAttachedObjects() != 0)
    {
        _avatar_node->detachAllObjects();
        _scene_manager->destroyEntity(_entity);

        MeshManager::getSingleton().remove(_mesh_name);
    }

#if TEX_DEBUG == 1

    size_t num_vertices = 6;

    std::vector<unsigned char> buffer_quad(num_vertices * sizeof(float) * 5);

    for(unsigned int i = 0; i < num_vertices; i++)
    {
        float vx[5];
        vx[0] = 0.f;

        switch(i)
        {
        case 0:
            vx[1] = 0.f;
            vx[2] = 0.f;

            vx[3] = 0.f;
            vx[4] = 0.f;
            break;
        case 1:
            vx[1] = 0.f;
            vx[2] = 1.f;

            vx[3] = 0.f;
            vx[4] = 1.f;
            break;
        case 2:
            vx[1] = 1.f;
            vx[2] = 0.f;

            vx[3] = 1.f;
            vx[4] = 0.f;
            break;
        case 3:
            vx[1] = 1.f;
            vx[2] = 0.f;

            vx[3] = 1.f;
            vx[4] = 0.f;
            break;
        case 4:
            vx[1] = 1.f;
            vx[2] = 1.f;

            vx[3] = 1.f;
            vx[4] = 1.f;
            break;
        case 5:
            vx[1] = 0.f;
            vx[2] = 1.f;

            vx[3] = 0.f;
            vx[4] = 1.f;
            break;
        }

        memcpy(&buffer_quad[i * sizeof(float) * 5], &vx[0], sizeof(float) * 5);
    }

    std::vector<int32_t> buffer_quad_index(num_vertices);

    std::iota(buffer_quad_index.begin(), buffer_quad_index.end(), 0);

    for(size_t i = 0; i < buffer_quad_index.size() - 2; i += 3)
    {
        int32_t swapSpace = buffer_quad_index[i + 2];
        buffer_quad_index[i + 2] = buffer_quad_index[i + 1];
        buffer_quad_index[i + 1] = swapSpace;
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
    std::cerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
#endif

    _mesh_name = std::to_string(rand());

    MeshPtr mesh = MeshManager::getSingleton().createManual(_mesh_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    // "-" for Z axis and min <-> max flip is NOT A MISTAKE!
    mesh->_setBounds(AxisAlignedBox::BOX_INFINITE);
    mesh->_setBoundingSphereRadius(10.f);

    mesh->sharedVertexData = new VertexData();
    mesh->sharedVertexData->vertexCount = num_vertices;

    VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
    VertexBufferBinding *bind = mesh->sharedVertexData->vertexBufferBinding;

    size_t offset = 0;
    decl->addElement(0, offset, VET_FLOAT3, VES_POSITION);
    offset += VertexElement::getTypeSize(VET_FLOAT3);
    decl->addElement(0, offset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
    offset += VertexElement::getTypeSize(VET_FLOAT2);

    HardwareVertexBufferSharedPtr vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(offset, num_vertices, HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    vbuf->writeData(0, vbuf->getSizeInBytes(), &buffer_quad[0], true);
    bind->setBinding(0, vbuf);

    HardwareIndexBufferSharedPtr ibuf = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_32BIT, num_vertices, HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    ibuf->writeData(0, ibuf->getSizeInBytes(), &buffer_quad_index[0], true);

    SubMesh *sub = mesh->createSubMesh();
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = num_vertices;
    sub->indexData->indexStart = 0;

    mesh->load();

#else

    size_t num_vertices = _num_geometry_bytes / (sizeof(float) * 5);

    float z = 0;
    for(size_t i = 0; i < num_vertices; i++)
    {
        size_t z_offset = i * 5 * sizeof(float) + 2 * sizeof(float);

        memcpy(&z, &_buffer_rcv_inflated[z_offset], sizeof(float));

        z = -z;

        memcpy(&_buffer_rcv_inflated[z_offset], &z, sizeof(float));
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
    std::cerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
#endif

    _mesh_name = std::to_string(rand());

    MeshPtr mesh = MeshManager::getSingleton().createManual(_mesh_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    // "-" for Z axis and min <-> max flip is NOT A MISTAKE!
    mesh->_setBounds(AxisAlignedBox({_bb_min[0], _bb_min[1], -_bb_max[2]}, {_bb_max[0], _bb_max[1], -_bb_min[2]}));
    mesh->_setBoundingSphereRadius(1.73f);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: bounds set" << std::endl;
    std::cerr << std::endl << "DynGeo: bounds set" << std::endl;
#endif

#if GUA_DEBUG == 1
    float vx[3];
    float tx[2];

    memcpy(&vx[0], &_buffer_rcv_inflated[100 * 5 * sizeof(float)], 3 * sizeof(float));
    memcpy(&tx[0], &_buffer_rcv_inflated[100 * 5 * sizeof(float) + 3 * sizeof(float)], 2 * sizeof(float));

    gzerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;
    std::cerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;

    gzerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
    std::cerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
#endif

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: HW vertex buffer written" << std::endl;
    std::cerr << std::endl << "DynGeo: HW vertex buffer written" << std::endl;
#endif

    mesh->sharedVertexData = new VertexData();
    mesh->sharedVertexData->vertexCount = num_vertices;

    size_t offset = 0;

    VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
    decl->addElement(0, offset, VET_FLOAT3, VES_POSITION);
    offset += VertexElement::getTypeSize(VET_FLOAT3);
    decl->addElement(0, offset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
    offset += VertexElement::getTypeSize(VET_FLOAT2);

    HardwareIndexBufferSharedPtr ibuf = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_32BIT, num_vertices, HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    {
        HardwareIndexBufferLockGuard lockGuard(ibuf, 0, ibuf->getSizeInBytes(), HardwareBuffer::LockOptions::HBL_WRITE_ONLY);
        memcpy(lockGuard.pData, &_buffer_index[0], ibuf->getSizeInBytes());
    }

    HardwareVertexBufferSharedPtr vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(offset, num_vertices, HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    {
        HardwareVertexBufferLockGuard lockGuard(vbuf, 0, vbuf->getSizeInBytes(), HardwareBuffer::LockOptions::HBL_WRITE_ONLY);
        memcpy(lockGuard.pData, &_buffer_rcv_inflated[0], vbuf->getSizeInBytes());
    }

    mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vbuf);

    _submesh_name = std::to_string(rand());

    SubMesh *sub = mesh->createSubMesh(_submesh_name);
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = num_vertices;
    sub->indexData->indexStart = 0;

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: submesh created" << std::endl;
    std::cerr << std::endl << "DynGeo: submesh created" << std::endl;
#endif

    mesh->load();

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: mesh loaded" << std::endl;
    std::cerr << std::endl << "DynGeo: mesh loaded" << std::endl;
#endif

#endif

    _entity_name = std::to_string(rand());
    _entity = _scene_manager->createEntity(_entity_name, _mesh_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    _entity->setMaterialName(_material_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    _entity->setVisible(true);

    _avatar_node->attachObject(_entity);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: entity attached" << std::endl;
    std::cerr << std::endl << "DynGeo: entity attached" << std::endl;
#endif

    _scene_manager->sceneGraphMutex.unlock();

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: lock released" << std::endl;
    std::cerr << std::endl << "DynGeo: lock released" << std::endl;
#endif

    /*#if GUA_DEBUG == 1
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        // test access to scene
        const ColourValue ambient(r, g, b, 1.f);
        _scene_manager->setAmbientLight(ambient);
    #endif

    #if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: test colors written" << std::endl;
        std::cerr << std::endl << "DynGeo: test colors written" << std::endl;
    #endif*/
}
void GuaDynGeoVisualPlugin::Update()
{
    /*#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: pre-render update before" << std::endl;
        std::cerr << std::endl << "DynGeo: pre-render update before" << std::endl;
    #endif*/

    if(_is_initialized.load() && _is_need_swap.load())
    {
        std::unique_lock<std::mutex> lk_swap(_mutex_swap);

        /*#if GUA_DEBUG == 1
                std::thread::id this_id = std::this_thread::get_id();

                gzerr << std::endl << "DynGeo: Update thread " << this_id << std::endl;
                std::cerr << std::endl << "DynGeo: Update thread " << this_id << std::endl;
        #endif*/

#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: swap" << std::endl;
        std::cerr << std::endl << "DynGeo: swap" << std::endl;
#endif

        UpdateTriangleSoup();
        _is_need_swap.store(false);
        _cv_recv_swap.notify_one();
    }

    /*#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: pre-render update after" << std::endl;
        std::cerr << std::endl << "DynGeo: pre-render update after" << std::endl;
    #endif*/
}
} // namespace gazebo
