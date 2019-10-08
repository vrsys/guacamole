#ifndef PAGODA_LOG_H
#define PAGODA_LOG_H

#include <cassert>
#include <ctime>
#include <fstream>
#include <gua/node/Node.hpp>
#include <gua/nrp/platform.hpp>

namespace gua
{
namespace nrp
{
class GUA_NRP_DLL NRPLog
{
  public:
    enum LOG_LEVEL
    {
        INFO = 3,
        DEBUG = 2,
        WARNING = 1,
        ERROR = 0
    };

    NRPLog()
    {
        auto *rand_name = new char[8];

        gen_random(rand_name, 8);

        _ofstream = std::ofstream();
        _ofstream.open(std::string(rand_name) + ".txt", std::ios::out | std::ios::trunc);

        assert(_ofstream.is_open());
    }

    explicit NRPLog(const char *log_name)
    {
        _ofstream = std::ofstream();
        _ofstream.open(std::string(log_name) + ".txt", std::ios::out | std::ios::app);

        assert(_ofstream.is_open());
    }

    NRPLog(const char *log_name, LOG_LEVEL log_level)
    {
        _ofstream = std::ofstream();
        _ofstream.open(std::string(log_name) + ".txt", std::ios::out | std::ios::app);

        _log_level = log_level;

        assert(_ofstream.is_open());
    }

    ~NRPLog() { _ofstream.close(); }

    bool i(const char *msg) { return (_log_level >= LOG_LEVEL::INFO) && log(msg); }

    bool d(const char *msg) { return (_log_level >= LOG_LEVEL::DEBUG) && log(msg); }

    bool w(const char *msg) { return (_log_level >= LOG_LEVEL::WARNING) && log(msg); }

    bool e(const char *msg) { return (_log_level >= LOG_LEVEL::ERROR) && log(msg); }

    void print_scenegraph(std::shared_ptr<gua::node::Node> const &node, unsigned long tree_depth = 0)
    {
        std::stringstream out;

        out << "\n";

        auto position = gua::math::get_translation(node->get_world_transform());
        auto rotation = scm::math::quat<scm::math::mat4d::value_type>::from_matrix(node->get_world_transform());
        auto scale = gua::math::get_scale(node->get_world_transform());

        out << std::string(tree_depth, ' ') << "Name: " << node->get_name() << std::endl;
        out << std::string(tree_depth, ' ') << "Depth: " << node->get_depth() << std::endl;
        out << std::string(tree_depth, ' ') << "Position: " << position.x << ", " << position.y << ", " << position.z << std::endl;
        out << std::string(tree_depth, ' ') << "Rotation: " << rotation.w << ", " << rotation.x << ", " << rotation.y << ", " << rotation.z << std::endl;
        out << std::string(tree_depth, ' ') << "Scale: " << scale.x << ", " << scale.y << ", " << scale.z << std::endl;
        out << std::string(tree_depth, ' ') << "Children: " << node->get_children().size();

        for(auto const &subtree_child : node->get_children())
        {
            print_scenegraph(subtree_child, tree_depth + 1);
        }

        if(!tree_depth)
        {
            out << "\n\n";
        }

        std::cout << out.str();
    }

  private:
    std::ofstream _ofstream;
#if GUA_DEBUG == 1
    LOG_LEVEL _log_level = DEBUG;
#else
    LOG_LEVEL _log_level = ERROR;
#endif

    bool log(const char *msg)
    {
        if(!_ofstream.is_open())
            return false;

        _ofstream << std::endl;

        std::time_t timestamp = std::time(nullptr);
        _ofstream << std::asctime(std::localtime(&timestamp));
        _ofstream << std::endl;

        _ofstream << msg;

        _ofstream << std::endl;

        return !_ofstream.bad();
    }

    void gen_random(char *s, const int len)
    {
        static const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

        for(int i = 0; i < len; ++i)
        {
            s[i] = alphanum[std::rand() % (sizeof(alphanum) - 1)];
        }

        s[len] = 0;
    }
};
} // namespace nrp
} // namespace gua

#endif // PAGODA_LOG_H