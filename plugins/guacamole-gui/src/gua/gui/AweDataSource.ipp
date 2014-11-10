////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of Swift2D.                                              //
//                                                                            //
// Copyright: (c) 2011-2014 Simon Schneegans & Felix Lauer                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

class AweDataSource : public Awesomium::DataSource {

 ///////////////////////////////////////////////////////////////////////////////
 // ----------------------------------------------------------- public interface
 public:

  // ------------------------------------------------------------ public methods
  void OnRequest(int request_id, Awesomium::ResourceRequest const& request,
                 Awesomium::WebString const& path) {

    std::string html_str("<h1>Failed to load resource.</h1>");

	std::string absolute_path(Awesomium::ToString(path));

	// strip parameters
    int index = absolute_path.find("?");
    if (index != std::string::npos) {
      absolute_path = absolute_path.substr(0, index);
    }

    // make absolute
    absolute_path = Paths::get().make_absolute(absolute_path);

    std::string ext(Paths::get().get_extension(absolute_path));

	std::string mime("text/html");
	if      (ext == ".png")  mime = "image/png";
	else if (ext == ".jpg")  mime = "image/jpg";
	else if (ext == ".jpeg") mime = "image/jpg";
	else if (ext == ".js")   mime = "text/javascript";
	else if (ext == ".css")  mime = "text/css";
	else if (ext == ".woff") mime = "application/x-font-woff";

    TextFile file(absolute_path);

    if (file.is_valid()) {
      html_str = file.get_content();
    } else {
      LOG_WARNING << "Failed to load resource \"" << path << "\": File not found!" << std::endl;
    }

    SendResponse(request_id, html_str.size(), (unsigned char*)html_str.c_str(),
                 Awesomium::WSLit(mime.c_str()));
  }

};
