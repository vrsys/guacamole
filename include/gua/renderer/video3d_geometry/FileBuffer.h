#ifndef SYS_FILEBUFFER_H
#define SYS_FILEBUFFER_H

#include <string>
#include <fstream>

namespace sys{

  class FileBuffer{

  public:
    FileBuffer(std::string const& path);
    ~FileBuffer();

    bool is_open();
    bool open();

    void close();

    void setLooping(bool onoff);
    bool getLooping();

    unsigned read (void* buffer, unsigned numbytes);
    unsigned bytes_read() const;

  private:

    std::string m_path;
    std::ifstream m_file;

    unsigned  m_bytes_r;
    unsigned long long m_filesize;
    bool m_looping;
  };

}


#endif // #ifndef SYS_FILEBUFFER_H

