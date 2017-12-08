#ifndef PAGODA_LOG_H
#define PAGODA_LOG_H

#include "common.h"
class Log
{
  public:
    enum LOG_LEVEL
    {
        INFO = 3,
        DEBUG = 2,
        WARNING = 1,
        ERROR = 0
    };

    Log()
    {
        auto *rand_name = new char[8];

        gen_random(rand_name, 8);

        _ofstream = std::ofstream();
        _ofstream.open(std::string(rand_name) + ".txt", std::ios::out | std::ios::app);

        assert(_ofstream.is_open());
    }

    explicit Log(const char *log_name)
    {
        _ofstream = std::ofstream();
        _ofstream.open(std::string(log_name) + ".txt", std::ios::out | std::ios::app);

        assert(_ofstream.is_open());
    }

    Log(const char *log_name, LOG_LEVEL log_level)
    {
        _ofstream = std::ofstream();
        _ofstream.open(std::string(log_name) + ".txt", std::ios::out | std::ios::app);

        _log_level = log_level;

        assert(_ofstream.is_open());
    }

    ~Log() { _ofstream.close(); }

    bool i(const char *msg) { return (_log_level >= LOG_LEVEL::INFO) && log(msg); }

    bool d(const char *msg) { return (_log_level >= LOG_LEVEL::DEBUG) && log(msg); }

    bool w(const char *msg) { return (_log_level >= LOG_LEVEL::WARNING) && log(msg); }

    bool e(const char *msg) { return (_log_level >= LOG_LEVEL::ERROR) && log(msg); }

  private:
    std::ofstream _ofstream;
    LOG_LEVEL _log_level = DEBUG;

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

    std::string print_scenegraph(std::shared_ptr<gua::node::Node> const &subtree_root, int tree_depth = 0)
    {
        std::stringstream out;

        out << "\n";
        for(int tab_print_index = 0; tab_print_index < tree_depth; ++tab_print_index)
        {
            out << "  ";
        }

        out << subtree_root->get_name();

        auto const &all_children_nodes = subtree_root->get_children();

        for(auto const &subtree_child : all_children_nodes)
        {
            print_scenegraph(subtree_child, tree_depth + 1);
        }

        if(!tree_depth)
        {
            out << "\n\n";
        }

        return out.str();
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

#endif // PAGODA_LOG_H