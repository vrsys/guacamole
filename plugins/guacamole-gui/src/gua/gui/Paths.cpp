////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of Swift2D.                                              //
//                                                                            //
// Copyright: (c) 2011-2014 Simon Schneegans & Felix Lauer                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// class header
#include <gua/gui/Paths.hpp>

#include <gua/utils/Logger.hpp>

#include <boost/filesystem.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

Paths::Paths() : executable_() {}

////////////////////////////////////////////////////////////////////////////////

void Paths::init(int argc, char** argv)
{
    executable_ = boost::filesystem::system_complete(argv[0]).normalize().remove_filename().string();

    if(executable_ == "")
    {
        Logger::LOG_ERROR << "Failed to get executable path!" << std::endl;
        return;
    }

    // create tmp directory ------------------------------------------------------
    boost::filesystem::create_directory(executable_ + "/tmp");
}

////////////////////////////////////////////////////////////////////////////////

void Paths::clean_up()
{
    // delete tmp directory ------------------------------------------------------
    if(!boost::filesystem::remove_all(executable_ + "/tmp"))
    {
        Logger::LOG_ERROR << "Failed to delete temporary directory!" << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////

std::string Paths::tmp_file(std::string const& suffix) const
{
    std::string file(boost::filesystem::unique_path().string());
    boost::filesystem::path p(executable_ + "/tmp/" + file + "." + suffix);
    return p.normalize().string();
}

////////////////////////////////////////////////////////////////////////////////

std::string Paths::resource(std::string const& type, std::string const& file) const
{
    boost::filesystem::path p(executable_ + "/resources/" + type + "/" + file);
    return p.normalize().string();
}

////////////////////////////////////////////////////////////////////////////////

std::string Paths::make_absolute(std::string const& file) const
{
    auto p(boost::filesystem::absolute(file, executable_));
    return p.normalize().string();
}

////////////////////////////////////////////////////////////////////////////////

std::string Paths::get_extension(std::string const& file) const
{
    boost::filesystem::path p(file);
    return p.extension().string();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
