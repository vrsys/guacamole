/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

class AweDataSource : public Awesomium::DataSource
{
    ///////////////////////////////////////////////////////////////////////////////
    // ----------------------------------------------------------- public interface
  public:
    // ------------------------------------------------------------ public methods
    void OnRequest(int request_id, Awesomium::ResourceRequest const& request, Awesomium::WebString const& path)
    {
        std::string html_str("<h1>Failed to load resource.</h1>");

        std::string absolute_path(Awesomium::ToString(path));

        // strip parameters
        int index = absolute_path.find("?");
        if(index != std::string::npos)
        {
            absolute_path = absolute_path.substr(0, index);
        }

        // make absolute
        absolute_path = Paths::instance()->make_absolute(absolute_path);

        std::string ext(Paths::instance()->get_extension(absolute_path));

        std::string mime("text/html");
        if(ext == ".png")
            mime = "image/png";
        else if(ext == ".jpg")
            mime = "image/jpg";
        else if(ext == ".jpeg")
            mime = "image/jpg";
        else if(ext == ".js")
            mime = "text/javascript";
        else if(ext == ".css")
            mime = "text/css";
        else if(ext == ".woff")
            mime = "application/x-font-woff";

        TextFile file(absolute_path);

        if(file.is_valid())
        {
            html_str = file.get_content();
        }
        else
        {
            Logger::LOG_WARNING << "Failed to load resource \"" << path << "\": File not found!" << std::endl;
        }

        SendResponse(request_id, html_str.size(), (unsigned char*)html_str.c_str(), Awesomium::WSLit(mime.c_str()));
    }
};
