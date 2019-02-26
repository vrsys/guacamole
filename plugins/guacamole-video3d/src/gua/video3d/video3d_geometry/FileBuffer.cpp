#include <gua/video3d/video3d_geometry/FileBuffer.h>

#include <boost/filesystem.hpp>

#include <iostream>
/*

from: http://www.cplusplus.com/reference/clibrary/cstdio/setbuf/

void setbuf ( FILE * stream, char * buffer );

Set stream buffer
Specifies the buffer to be used by the stream for I/O operations, which becomes a fully buffered stream. Or, alternatively, if buffer is a null pointer, buffering is disabled for the stream, which
becomes an unbuffered stream.

This function should be called once the stream has been associated with an open file, but before any input or output operation is performed with it.

The buffer is assumed to be at least BUFSIZ bytes in size (see setvbuf to specify a size of the buffer).

A stream buffer is a block of data that acts as intermediary between the i/o operations and the physical file associated to the stream: For output buffers, data is output to the buffer until its
maximum capacity is reached, then it is flushed (i.e.: all data is sent to the physical file at once and the buffer cleared). Likewise, input buffers are filled from the physical file, from which data
is sent to the operations until exhausted, at which point new data is acquired from the file to fill the buffer again.

Stream buffers can be explicitly flushed by calling fflush. They are also automatically flushed by fclose and freopen, or when the program terminates normally.

A full buffered stream uses the entire size of the buffer as buffer whenever enough data is available (see setvbuf for other buffer modes).

All files are opened with a default allocated buffer (fully buffered) if they are known to not refer to an interactive device. This function can be used to either set a specific memory block to be
used as buffer or to disable buffering for the stream.

The default streams stdin and stdout are fully buffered by default if they are known to not refer to an interactive device. Otherwise, they may either be line buffered or unbuffered by default,
depending on the system and library implementation. The same is true for stderr, which is always either line buffered or unbuffered by default.

A call to this function is equivalent to calling setvbuf with _IOFBF as mode and BUFSIZ as size (when buffer is not a null pointer), or equivalent to calling it with _IONBF as mode (when it is a null
pointer).
*/
namespace sys
{
FileBuffer::FileBuffer(std::string const& path) : m_path(path), m_file(), m_bytes_r(0), m_filesize(0), m_looping(false) {}

FileBuffer::~FileBuffer() { close(); }

bool FileBuffer::is_open() { return m_file.is_open(); }

bool FileBuffer::open()
{
    close();

    m_filesize = boost::filesystem::file_size(m_path);
    m_file.open(m_path, std::ios::in | std::ios::binary);

    if(!m_file.good())
    {
        std::cerr << "Could not open " << m_path << std::endl;
        return false;
    }

    return true;
}

void FileBuffer::close()
{
    if(is_open())
    {
        m_file.close();
    }
}

void FileBuffer::setLooping(bool onoff) { m_looping = onoff; }

bool FileBuffer::getLooping() { return m_looping; }

unsigned FileBuffer::read(void* buffer, unsigned numbytes)
{
    if(!m_file.good())
    {
        std::cerr << "FileBuffer::read () : Could to read from " << m_path << std::endl;
        return 0;
    }

    if((m_bytes_r + numbytes) > m_filesize)
    {
        if(m_looping)
        {
            m_file.seekg(std::ios_base::beg);
            m_bytes_r = 0;
        }
        else
        {
            return 0;
        }
    }

    std::size_t strpos_before_read = m_file.tellg();
    m_file.read((char*)buffer, numbytes);
    std::size_t strpos_after_read = m_file.tellg();
    std::size_t bytes = strpos_after_read - strpos_before_read;

    m_bytes_r += bytes;

    return bytes;
}

unsigned FileBuffer::bytes_read() const { return m_bytes_r; }

} // namespace sys
