/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pangolin/utils/file_extension.h>

#include <algorithm>
#include <fstream>
#include <string.h>

namespace pangolin
{

std::string FileLowercaseExtention(const std::string& filename)
{
    size_t pos = filename.find_last_of('.');
    if(pos != std::string::npos) {
        std::string ext = filename.substr(pos);
        std::transform( ext.begin(), ext.end(), ext.begin(), tolower );
        return ext;
    }else{
        return "";
    }
}

ImageFileType FileTypeMagic(const unsigned char data[], size_t bytes)
{
    // Check we wont go over bounds when comparing.
    if(bytes >= 8) {
        const unsigned char magic_png[]  = "\211PNG\r\n\032\n";
        const unsigned char magic_jpg1[] = "\xFF\xD8";
        const unsigned char magic_jpg2[] = "\xFF\xD9";
        const unsigned char magic_gif1[] = "GIF87a";
        const unsigned char magic_gif2[] = "GIF89a";
        const unsigned char magic_tiff1[] = "\x49\x49\x2A\x00";
        const unsigned char magic_tiff2[] = "\x4D\x4D\x00\x2A";
        const unsigned char magic_exr[]   = "\x76\x2F\x31\x01";
        const unsigned char magic_pango[] = "PANGO";

        if( !strncmp((char*)data, (char*)magic_png, 8) ) {
            return ImageFileTypePng;
        }else if( !strncmp( (char*)data, (char*)magic_jpg1, 2)
                  || !strncmp( (char*)data, (char*)magic_jpg2, 2) ) {
            return ImageFileTypeJpg;
        }else if( !strncmp((char*)data, (char*)magic_gif1,6)
                  || !strncmp((char*)data, (char*)magic_gif2,6) ) {
            return ImageFileTypeGif;
        }else if( !strncmp((char*)data, (char*)magic_tiff1,4)
                  || !strncmp((char*)data, (char*)magic_tiff2,4) ) {
            return ImageFileTypeTiff;
        }else if( !strncmp((char*)data, (char*)magic_exr,4) ) {
            return ImageFileTypeExr;
        }else if( !strncmp((char*)data, (char*)magic_pango,5) ) {
            return ImageFileTypePango;
        }else if( data[0] == 'P' && '0' < data[1] && data[1] < '9') {
            return ImageFileTypePpm;
        }
    }
    return ImageFileTypeUnknown;
}

ImageFileType FileTypeExtension(const std::string& ext)
{
    if( ext == ".png" ) {
        return ImageFileTypePng;
    } else if( ext == ".tga" || ext == ".targa") {
        return ImageFileTypeTga;
    } else if( ext == ".jpg" || ext == ".jpeg" ) {
        return ImageFileTypeJpg;
    } else if( ext == ".gif" ) {
        return ImageFileTypeGif;
    } else if( ext == ".tif" || ext == ".tiff" ) {
        return ImageFileTypeTiff;
    } else if( ext == ".exr"  ) {
        return ImageFileTypeExr;
    } else if( ext == ".ppm" || ext == ".pgm" || ext == ".pbm" || ext == ".pxm" || ext == ".pdm" ) {
        return ImageFileTypePpm;
    } else if( ext == ".pvn"  ) {
        return ImageFileTypePvn;
    } else if( ext == ".pango"  ) {
        return ImageFileTypePango;
    } else {
        return ImageFileTypeUnknown;
    }
}

ImageFileType FileType(const std::string& filename)
{
    // Check magic number of file...
    FILE *fp = fopen(filename.c_str(), "rb");
    if(fp) {
        const size_t magic_bytes = 8;
        unsigned char magic[magic_bytes];
        size_t num_read = fread( (void *)magic, 1, magic_bytes, fp);
        ImageFileType magic_type = FileTypeMagic(magic, num_read);
        fclose(fp);
        if(magic_type != ImageFileTypeUnknown) {
            return magic_type;
        }
    }

    // Fallback on using extension...
    const std::string ext = FileLowercaseExtention(filename);
    return FileTypeExtension(ext);
}

}
