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

#include <pangolin/image/image_io.h>

#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <string.h>
#include <cstring>

#ifdef HAVE_PNG
#include <png.h>
#endif // HAVE_PNG

#ifdef HAVE_JPEG
#include <jpeglib.h>
#ifndef HAVE_PNG
// This should not be included when HAVE_PNG, as png.h includes its own.
#include <setjmp.h>
#endif // HAVE_PNG
#endif // HAVE_JPEG

#ifdef HAVE_OPENEXR
#error "XYZ"
#include <ImfOutputFile.h>
#include <ImfInputFile.h>
#include <ImfChannelList.h>
#endif // HAVE_OPENEXR

#ifdef _WIN_
// Undef windows Macro polution from jpeglib.h
#undef LoadImage
#endif

namespace pangolin {

VideoPixelFormat TgaFormat(int depth, int color_type, int color_map)
{
    if(color_map == 0) {
        if(color_type == 2) {
            // Colour
            if(depth == 24) {
                return VideoFormatFromString("RGB24");
            }else if(depth == 32) {
                return VideoFormatFromString("RGBA");
            }
        }else if(color_type == 3){
            // Greyscale
            if(depth == 8) {
                return VideoFormatFromString("GRAY8");
            }else if(depth == 16) {
                return VideoFormatFromString("Y400A");
            }
        }
    }
    throw std::runtime_error("Unsupported TGA format");    
}

TypedImage LoadTga(const std::string& filename)
{
    FILE *file;
    unsigned char type[4];
    unsigned char info[6];
    
    file = fopen(filename.c_str(), "rb");
    
    if(file) {
        bool success = true;
        success &= fread( &type, sizeof (char), 3, file ) == 3;
        fseek( file, 12, SEEK_SET );
        success &= fread( &info, sizeof (char), 6, file ) == 6;
        
        const int width  = info[0] + (info[1] * 256);
        const int height = info[2] + (info[3] * 256);
        
        TypedImage img;
        if(success) {
            img.Alloc(width, height, TgaFormat(info[4], type[2], type[1]) );
            
            //read in image data
            const size_t data_size = img.w * img.pitch;
            success &= fread(img.ptr, sizeof(unsigned char), data_size, file) == data_size;
        }
        
        fclose(file);
        
        if (success) {
            return img;
        }
    }
    
    throw std::runtime_error("Unable to load TGA file, '" + filename + "'");    
}

#ifdef HAVE_PNG
VideoPixelFormat PngFormat(png_structp png_ptr, png_infop info_ptr )
{
    const png_byte colour = png_get_color_type(png_ptr, info_ptr);
    const png_byte depth  = png_get_bit_depth(png_ptr, info_ptr);

    if( depth == 8 ) {
        if( colour == PNG_COLOR_MASK_COLOR ) {
            return VideoFormatFromString("RGB24");
        } else if( colour == (PNG_COLOR_MASK_COLOR | PNG_COLOR_MASK_ALPHA) ) {
            return VideoFormatFromString("RGBA");
        } else if( colour == PNG_COLOR_MASK_ALPHA ) {
            return VideoFormatFromString("Y400A");
        } else {
            return VideoFormatFromString("GRAY8");
        }
    }else if( depth == 16 ) {
        if( colour == 0 ) {
            return VideoFormatFromString("GRAY16LE");
        }
    }

    throw std::runtime_error("Unsupported PNG format");
}

void PNGAPI PngWarningsCallback(png_structp /*png_ptr*/, png_const_charp /*warning_message*/)
{
    // Override default behaviour - don't do anything.
}
#endif

TypedImage LoadPng(const std::string& filename)
{
#ifdef HAVE_PNG
    FILE *in = fopen(filename.c_str(), "rb");
    
    if( in )  {
        //check the header
        const size_t nBytes = 8;
        png_byte header[nBytes];
        size_t nread = fread(header, 1, nBytes, in);
        int nIsPNG = png_sig_cmp(header, 0, nread);
        
        if ( nIsPNG != 0 )  {
            throw std::runtime_error( filename + " is not a PNG file" );
        }
        
        //set up initial png structs
        png_structp png_ptr = png_create_read_struct( PNG_LIBPNG_VER_STRING, (png_voidp)NULL, NULL, &PngWarningsCallback);
        if (!png_ptr) {
            throw std::runtime_error( "PNG Init error 1" );
        }
        
        png_infop info_ptr = png_create_info_struct(png_ptr);
        if (!info_ptr)  {
            png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
            throw std::runtime_error( "PNG Init error 2" );
        }
        
        png_infop end_info = png_create_info_struct(png_ptr);
        if (!end_info) {
            png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
            throw std::runtime_error( "PNG Init error 3" );
        }
        
        png_init_io(png_ptr, in);
        png_set_sig_bytes(png_ptr, nBytes);
        
        //read the file
        png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);
        
        if( png_get_bit_depth(png_ptr, info_ptr) == 1)  {
            //Unpack bools to bytes to ease loading.
            png_set_packing(png_ptr);
        } else if( png_get_bit_depth(png_ptr, info_ptr) < 8) {
            //Expand nonbool colour depths up to 8bpp
            png_set_expand_gray_1_2_4_to_8(png_ptr);
        }
        
        //Get rid of palette, by transforming it to RGB
        if(png_get_color_type(png_ptr, info_ptr) == PNG_COLOR_TYPE_PALETTE) {
            png_set_palette_to_rgb(png_ptr);
        }
        
        if( png_get_interlace_type(png_ptr,info_ptr) != PNG_INTERLACE_NONE) {
            throw std::runtime_error( "Interlace not yet supported" );
        }
        
        const size_t w = png_get_image_width(png_ptr,info_ptr);
        const size_t h = png_get_image_height(png_ptr,info_ptr);
        const size_t pitch = png_get_rowbytes(png_ptr, info_ptr);
        
        TypedImage img;
        img.Alloc(w, h, PngFormat(png_ptr, info_ptr), pitch);
        
        png_bytepp rows = png_get_rows(png_ptr, info_ptr);
        for( unsigned int r = 0; r < h; r++) {
            memcpy( img.ptr + pitch*r, rows[r], pitch );
        }
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        
        fclose(in);
        return img;
    }
    
    throw std::runtime_error("Unable to load PNG file, '" + filename + "'");    
    
#else
    throw std::runtime_error("PNG Support not enabled. Please rebuild Pangolin.");
#endif
}

void SavePng(const Image<unsigned char>& image, const pangolin::VideoPixelFormat& fmt, const std::string& filename, bool top_line_first)
{
    // Check image has supported bit depth
    for(unsigned int i=1; i < fmt.channels; ++i) {
        if( fmt.channel_bits[i] != fmt.channel_bits[0] ) {
            throw std::runtime_error("PNG Saving only supported for images where each channel has the same bit depth.");
        }
    }

#ifdef HAVE_PNG
    FILE *fp;
    png_structp png_ptr;
    png_infop info_ptr;

    // Open file for writing (binary mode)
    fp = fopen(filename.c_str(), "wb");
    if (fp == NULL) {
        throw std::runtime_error( "PNG Error: Could not open file '" + filename + "' for writing" );
    }

    // Initialize write structure
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png_ptr == NULL) {
        fclose(fp);
        throw std::runtime_error( "PNG Error: Could not allocate write struct." );
    }

    // Initialize info structure
    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        fclose(fp);
        throw std::runtime_error( "PNG Error: Could not allocate info struct." );
    }

    // Setup Exception handling
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        fclose(fp);
        throw std::runtime_error( "PNG Error: Error during png creation." );
    }

    png_init_io(png_ptr, fp);

    const int bit_depth = fmt.channel_bits[0];

    int colour_type;
    switch (fmt.channels) {
    case 1: colour_type = PNG_COLOR_TYPE_GRAY; break;
    case 2: colour_type = PNG_COLOR_TYPE_GRAY_ALPHA; break;
    case 3: colour_type = PNG_COLOR_TYPE_RGB; break;
    case 4: colour_type = PNG_COLOR_TYPE_RGBA; break;
    default:
        throw std::runtime_error( "PNG Error: unexpected image channel number");
    }

    // Write header (8 bit colour depth)
    png_set_IHDR(
        png_ptr, info_ptr, image.w, image.h, bit_depth, colour_type,
        PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT
    );

    png_write_info(png_ptr, info_ptr);

    // Write image data

    if(top_line_first) {
        for (unsigned int y = 0; y< image.h; y++) {
            png_write_row(png_ptr, image.ptr + y*image.pitch);
        }
    }else{
        for (int y = (int)image.h-1; y >=0; y--) {
            png_write_row(png_ptr, image.ptr + y*image.pitch);
        }
    }

    // End write
    png_write_end(png_ptr, NULL);

    // Free resources
    fclose(fp);
    png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

#else
    throw std::runtime_error("PNG Support not enabled. Please rebuild Pangolin.");
#endif
}

#ifdef HAVE_JPEG
struct my_error_mgr
{
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

typedef struct my_error_mgr * my_error_ptr;

METHODDEF(void) my_error_exit(j_common_ptr cinfo)
{
  my_error_ptr myerr = (my_error_ptr) cinfo->err;
  (*cinfo->err->output_message) (cinfo);
  longjmp(myerr->setjmp_buffer, 1);
}

VideoPixelFormat JpgFormat(jpeg_decompress_struct& /*info*/ )
{
    // TODO: Actually work this out properly.
    return VideoFormatFromString("RGB24");
}
#endif

TypedImage LoadJpg(const std::string& filename)
{
#ifdef HAVE_JPEG
    FILE * infile = fopen(filename.c_str(), "rb");

    if(infile) {
        struct my_error_mgr jerr;
        jerr.pub.error_exit = my_error_exit;
    
        struct jpeg_decompress_struct cinfo;
        cinfo.err = jpeg_std_error(&jerr.pub);
    
        if (setjmp(jerr.setjmp_buffer)) {
            // If we get here, the JPEG code has signaled an error.
            jpeg_destroy_decompress(&cinfo);
            fclose(infile);
            throw std::runtime_error("Error whilst loading JPEG image, '" + filename + "'");
        }
    
        jpeg_create_decompress(&cinfo);
        jpeg_stdio_src(&cinfo, infile);
        jpeg_read_header(&cinfo, TRUE);
        jpeg_start_decompress(&cinfo);
    
        const int row_stride = cinfo.output_width * cinfo.output_components;
        
        TypedImage img;
        img.Alloc(cinfo.output_width, cinfo.output_height, JpgFormat(cinfo), row_stride);
        
        JSAMPARRAY row_buffer = (*cinfo.mem->alloc_sarray)
                ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
        
        while (cinfo.output_scanline < cinfo.output_height) {
            const int scanline = cinfo.output_scanline;
            jpeg_read_scanlines(&cinfo, row_buffer, 1);
            memcpy(img.ptr + scanline * img.pitch, row_buffer[0], img.pitch );
        }
        
        jpeg_finish_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        fclose(infile);  
        return img;
    }
    throw std::runtime_error("Unable to load JPEG file, '" + filename + "'");
#else   
    throw std::runtime_error("JPEG Support not enabled. Please rebuild Pangolin.");
#endif
}

VideoPixelFormat PpmFormat(const std::string& strType, int /*num_colours*/)
{
    if(strType == "P5") {
        return VideoFormatFromString("GRAY8"); 
    }else if(strType == "P6") {
        return VideoFormatFromString("RGB24"); 
    }else{
        throw std::runtime_error("Unsupported PPM/PGM format");        
    }
}

void PpmConsumeWhitespaceAndComments(std::ifstream& bFile)
{
    // TODO: Make a little more general / more efficient
    while( bFile.peek() == ' ' )  bFile.get();
    while( bFile.peek() == '\n' ) bFile.get();
    while( bFile.peek() == '#' )  bFile.ignore(4096, '\n');
}

TypedImage LoadPpm(std::ifstream& bFile)
{
    // Parse header
    std::string ppm_type = "";
    int num_colors = 0;
    int w = 0;
    int h = 0;

    bFile >> ppm_type;
    PpmConsumeWhitespaceAndComments(bFile);
    bFile >> w;
    PpmConsumeWhitespaceAndComments(bFile);
    bFile >> h;
    PpmConsumeWhitespaceAndComments(bFile);
    bFile >> num_colors;
    bFile.ignore(1,'\n');
    
    TypedImage img;
    bool success = !bFile.fail() && w > 0 && h > 0;

    if(success) {
        img.Alloc(w, h, PpmFormat(ppm_type, num_colors) );

        // Read in data
        for(size_t r=0; r<img.h; ++r) {
            bFile.read( (char*)img.ptr + r*img.pitch, img.pitch );
        }
        success = !bFile.fail();
    }

    if(!success) {
        img.Dealloc();
    }

    return img;
}

TypedImage LoadPpm(const std::string& filename)
{
    std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
    return LoadPpm(bFile);
}

#ifdef HAVE_OPENEXR
Imf::PixelType OpenEXRPixelType(int channel_bits)
{
    if( channel_bits == 16 ) {
        return Imf::PixelType::HALF;
    }else if( channel_bits == 32 ) {
        return Imf::PixelType::FLOAT;
    }else{
        throw std::runtime_error("Unsupported OpenEXR Pixel Type.");
    }
}

void SetOpenEXRChannels(Imf::ChannelList& ch, const pangolin::VideoPixelFormat& fmt)
{
    const char* CHANNEL_NAMES[] = {"R","G","B","A"};
    for(size_t c=0; c < fmt.channels; ++c) {
        ch.insert( CHANNEL_NAMES[c], Imf::Channel(OpenEXRPixelType(fmt.channel_bits[c])) );
    }
}
#endif //HAVE_OPENEXR

void SaveExr(const Image<unsigned char>& image_in, const pangolin::VideoPixelFormat& fmt, const std::string& filename, bool top_line_first)
{
#ifdef HAVE_OPENEXR
    Image<unsigned char> image;

    if(top_line_first) {
        image = image_in;
    }else{
        image.Alloc(image_in.w,image_in.h,image_in.pitch);
        for(size_t y=0; y<image_in.h; ++y) {
            std::memcpy(image.ptr + y*image.pitch, image_in.ptr + (image_in.h-y-1)*image_in.pitch, image.pitch);
        }
    }


    Imf::Header header (image.w, image.h);
    SetOpenEXRChannels(header.channels(), fmt);

    Imf::OutputFile file (filename.c_str(), header);
    Imf::FrameBuffer frameBuffer;

    int ch=0;
    size_t ch_bits = 0;
    for(Imf::ChannelList::Iterator it = header.channels().begin(); it != header.channels().end(); ++it)
    {
        frameBuffer.insert(
            it.name(),
            Imf::Slice(
                it.channel().type,
                (char*)image.ptr + ch_bits/8,
                fmt.channel_bits[ch]/8,
                image.pitch
            )
        );

        ch_bits += fmt.channel_bits[ch++];
    }

    file.setFrameBuffer(frameBuffer);
    file.writePixels(image.h);

    if(!top_line_first) {
        image.Dealloc();
    }

#else
    throw std::runtime_error("EXR Support not enabled. Please rebuild Pangolin.");
#endif // HAVE_OPENEXR
}


TypedImage LoadImage(const std::string& filename, ImageFileType file_type)
{
    switch (file_type) {
    case ImageFileTypeTga:
        return LoadTga(filename);
    case ImageFileTypePng:
        return LoadPng(filename);
    case ImageFileTypeJpg:
        return LoadJpg(filename);
    case ImageFileTypePpm:
        return LoadPpm(filename);
    default:
        throw std::runtime_error("Unsupported image file type, '" + filename + "'");
    }
}

TypedImage LoadImage(const std::string& filename)
{
    ImageFileType file_type = FileType(filename);
    return LoadImage( filename, file_type );
}

void SaveImage(const Image<unsigned char>& image, const pangolin::VideoPixelFormat& fmt, const std::string& filename, ImageFileType file_type, bool top_line_first)
{
    switch (file_type) {
    case ImageFileTypePng:
        return SavePng(image, fmt, filename, top_line_first);
    case ImageFileTypeExr:
        return SaveExr(image, fmt, filename, top_line_first);
    default:
        throw std::runtime_error("Unsupported image file type, '" + filename + "'");
    }
}

void SaveImage(const Image<unsigned char>& image, const pangolin::VideoPixelFormat& fmt, const std::string& filename, bool top_line_first)
{
    const std::string ext = FileLowercaseExtention(filename);
    const ImageFileType file_type = FileTypeExtension(ext);
    SaveImage(image, fmt, filename,file_type, top_line_first);
}

void SaveImage(const TypedImage& image, const std::string& filename, bool top_line_first)
{
    SaveImage(image, image.fmt, filename, top_line_first);
}

void FreeImage(TypedImage& img)
{
    img.Dealloc();
}

}
