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

#include <pangolin/platform.h>

#include <pangolin/utils/file_utils.h>

#ifdef _WIN_
#  define WIN32_LEAN_AND_MEAN
#  include <Windows.h>
#else
#  include <dirent.h>
#  include <sys/stat.h>
#endif // _WIN_

#include <algorithm>
#include <sstream>
#include <list>

namespace pangolin
{

std::vector<std::string>& Split(const std::string& s, char delim, std::vector<std::string>& elements) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elements.push_back(item);
    }
    return elements;
}

std::vector<std::string> Split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return Split(s, delim, elems);
}

std::vector<std::string> Expand(const std::string &s, char open, char close, char delim)
{
    const size_t no = s.find_first_of(open);
    if(no != std::string::npos) {
        const size_t nc = s.find_first_of(close, no);
        if(no != std::string::npos) {
            const std::string pre  = s.substr(0, no);
            const std::string mid  = s.substr(no+1, nc-no-1);
            const std::string post = s.substr(nc+1, std::string::npos);
            const std::vector<std::string> options = Split(mid, delim);
            std::vector<std::string> expansion;
            
            for(std::vector<std::string>::const_iterator iop = options.begin(); iop != options.end(); ++iop)
            {
                std::string full = pre + *iop + post;
                expansion.push_back(full);
            }
            return expansion;
        }
        // Open but no close is unusual. Leave it for caller to see if valid
    }
    
    std::vector<std::string> ret;
    ret.push_back(s);
    return ret;
}

// Make path seperator consistent for OS.
void PathOsNormaliseInplace(std::string& path)
{
#ifdef _WIN_
    std::replace(path.begin(), path.end(), '/', '\\');
#else
    std::replace(path.begin(), path.end(), '\\', '/');
#endif
}

std::string SanitizePath(const std::string& path)
{
    std::string path_copy(path.length(), '\0');

    int p_slash1 = -1;
    int p_slash0 = -1;
    int n_dots = 0;

    int dst = 0;
    for(int src=0; src < (int)path.length(); ++src) {
        if(path[src] == '/') {
            if(n_dots==1 && p_slash0 >=0) {
                dst = p_slash0;
                for(p_slash1=p_slash0-1; p_slash1>=0 && path_copy[p_slash1] != '/'; --p_slash1);
            }else if(n_dots==2) {
                if(p_slash1 >=0) {
                    dst = p_slash1;
                    p_slash0 = dst;
                    for(p_slash1=p_slash0-1; p_slash1>=0 && path_copy[p_slash1] != '/'; --p_slash1) {
                        if( path_copy[p_slash1] == '.' ) {
                            p_slash1=-1;
                            break;
                        }
                    }
                }else{
                    p_slash1 = -1;
                    p_slash0 = dst;
                }
            }else{
                p_slash1 = p_slash0;
                p_slash0 = dst;
            }
            n_dots = 0;
        }else if(path[src] == '.' ){
            if((dst-p_slash0) == n_dots+1) {
                ++n_dots;
            }
        }else{
            n_dots = 0;
        }
        path_copy[dst++] = path[src];
    }

    return path_copy.substr(0,dst);
}

// Return path 'levels' directories above 'path'
std::string PathParent(const std::string& path, int levels)
{
    std::string res = path;

    while (levels > 0) {
        if (res.length() == 0) {
            res = std::string();
            for (int l = 0; l < levels; ++l) {
#ifdef _WIN_
                res += std::string("..\\");
#else
                res += std::string("../");
#endif
            }
            return res;
        }else{
            const size_t nLastSlash = res.find_last_of("/\\");

            if (nLastSlash != std::string::npos) {
                res = path.substr(0, nLastSlash);
            } else{
                res = std::string();
            }

            --levels;
        }
    }

    return res;
}

std::string FindPath(const std::string& child_path, const std::string& signature_path)
{
    std::string path = child_path;
    std::string signature = signature_path;
    PathOsNormaliseInplace(path);
    PathOsNormaliseInplace(signature);

    while(!FileExists(path + signature)) {
        if (path.empty()) {
            return std::string();
        } else {
            path = PathParent(path);
        }
    }

    return path + signature;
}

std::string PathExpand(const std::string& sPath)
{
    if(sPath.length() >0 && sPath[0] == '~') {
#ifdef _WIN_
        const size_t buffer_size = 1024;
        char buffer[buffer_size];
        size_t size;
        std::string sHomeDir;
        if (!getenv_s(&size, buffer, buffer_size, "HOMEDRIVE")) {
            sHomeDir = std::string(buffer, buffer + size);
            if (!getenv_s(&size, buffer, buffer_size, "HOMEPATH")) {
                sHomeDir += std::string(buffer, buffer + size);
            }
        }
#else
        std::string sHomeDir = std::string(getenv("HOME"));
#endif
        return sHomeDir + sPath.substr(1,std::string::npos);
    }else{
        return sPath;
    }
}

// Based on http://www.codeproject.com/Articles/188256/A-Simple-Wildcard-Matching-Function
bool MatchesWildcard(const std::string& str, const std::string& wildcard)
{
    const char* psQuery = str.c_str();
    const char* psWildcard = wildcard.c_str();
    
    while(*psWildcard)
    {
        if(*psWildcard=='?')
        {
            if(!*psQuery)
                return false;

            ++psQuery;
            ++psWildcard;
        }
        else if(*psWildcard=='*')
        {
            if(MatchesWildcard(psQuery,psWildcard+1))
                return true;

            if(*psQuery && MatchesWildcard(psQuery+1,psWildcard))
                return true;

            return false;
        }
        else
        {
            if(*psQuery++ != *psWildcard++ )
                return false;
        }
    }

    return !*psQuery && !*psWildcard;
}

#ifdef _WIN_

#ifdef UNICODE
    typedef std::codecvt_utf8<wchar_t> WinStringConvert;
    typedef std::wstring WinString;

    WinString s2ws(const std::string& str) {
        std::wstring_convert<WinStringConvert, wchar_t> converter;
        return converter.from_bytes(str);
    }
    std::string ws2s(const WinString& wstr) {
        std::wstring_convert<WinStringConvert, wchar_t> converter;
        return converter.to_bytes(wstr);
    }
#else
    typedef std::string WinString;
    // No conversions necessary
#   define s2ws(X) (X)
#   define ws2s(X) (X)
#endif // UNICODE

bool FilesMatchingWildcard(const std::string& wildcard, std::vector<std::string>& file_vec)
{
	size_t nLastSlash = wildcard.find_last_of("/\\");
    
    std::string sPath;
    std::string sFileWc;
    
    if(nLastSlash != std::string::npos) {
        sPath =   wildcard.substr(0, nLastSlash);                  
        sFileWc = wildcard.substr(nLastSlash+1, std::string::npos);
    }else{
        sPath = ".";
        sFileWc = wildcard;
    }

    sPath = PathExpand(sPath);
    
    WIN32_FIND_DATA wfd;
    HANDLE fh = FindFirstFile( s2ws(sPath + "\\" + sFileWc).c_str(), &wfd);

    std::vector<std::string> files;

    if (fh != INVALID_HANDLE_VALUE) {
        do {
            if (!(wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))  {
                files.push_back( sPath + "\\" + ws2s(wfd.cFileName) );
            }
        } while (FindNextFile (fh, &wfd));
        FindClose(fh);
    }

    // TOOD: Use different comparison to achieve better ordering.
    std::sort(files.begin(), files.end() );

    // Put file list at end of file_vec
    file_vec.insert(file_vec.end(), files.begin(), files.end() );

    return files.size() > 0;
}

bool FileExists(const std::string& filename)
{
    std::string search_filename = filename;
    if(filename.length() > 0 && filename[filename.length()-1] == '\\') {
        search_filename.resize(filename.length()-1);
    }
    WIN32_FIND_DATA wfd;
    HANDLE fh = FindFirstFile( s2ws(search_filename).c_str(), &wfd);
    const bool exists = fh != INVALID_HANDLE_VALUE;
    FindClose(fh);
    return exists;
}

#else // _WIN_

bool FilesMatchingWildcard(const std::string& wildcard, std::vector<std::string>& file_vec)
{
	size_t nLastSlash = wildcard.find_last_of("/\\");
    
    std::string sPath;
    std::string sFileWc;
    
    if(nLastSlash != std::string::npos) {
        sPath =   wildcard.substr(0, nLastSlash);                  
        sFileWc = wildcard.substr(nLastSlash+1, std::string::npos);
    }else{
        sPath = ".";
        sFileWc = wildcard;
    }
    
    sPath = PathExpand(sPath);
        
    struct dirent **namelist;
    int n = scandir(sPath.c_str(), &namelist, 0, alphasort ); // sort alpha-numeric
//    int n = scandir(sPath.c_str(), &namelist, 0, versionsort ); // sort aa1 < aa10 < aa100 etc.
    if (n >= 0){
        std::list<std::string> file_list;
        while( n-- ){
            const std::string sName(namelist[n]->d_name);
            if( sName != "." && sName != ".." && MatchesWildcard(sName, sFileWc) ) {
                const std::string sFullName = sPath + "/" + sName;
                file_list.push_front( sFullName );
            }
            free(namelist[n]);
        }
        free(namelist);
        
        file_vec.reserve(file_list.size());
        file_vec.insert(file_vec.begin(), file_list.begin(), file_list.end());
        return file_vec.size() > 0;
    }
    return false;
}

bool FileExists(const std::string& filename)
{
    struct stat buf;
    return stat(filename.c_str(), &buf) != -1;
}

#endif //_WIN_

}
