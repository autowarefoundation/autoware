/* This is a (butchered) derivative of picojson, incorporated into the
 * Pangolin Project (http://github.com/stevenlovegrove/Pangolin)
 *
 * Modifications Copyright (c) 2014 Steven Lovegrove,
 * Original licence applies (below), https://github.com/kazuho/picojson
 */

/*
 * Copyright 2009-2010 Cybozu Labs, Inc.
 * Copyright 2011-2014 Kazuho Oku
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef picojson_h
#define picojson_h

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <pangolin/compat/type_traits.h>

// Enable INT64 support
#define PICOJSON_USE_INT64

// for isnan/isinf
#if __cplusplus>=201103L
# include <cmath>
#else
extern "C" {
# ifdef _MSC_VER
#  include <float.h>
# elif defined(__INTEL_COMPILER)
#  include <mathimf.h>
# else
#  include <math.h>
# endif
}
#endif

// experimental support for int64_t (see README.mkdn for detail)
#ifdef PICOJSON_USE_INT64
# define __STDC_FORMAT_MACROS
# include <errno.h>
# include <inttypes.h>
#endif // PICOJSON_USE_INT64

// to disable the use of localeconv(3), set PICOJSON_USE_LOCALE to 0
#ifndef PICOJSON_USE_LOCALE
# define PICOJSON_USE_LOCALE 1
#endif // PICOJSON_USE_LOCALE

#if PICOJSON_USE_LOCALE
extern "C" {
# include <locale.h>
}
#endif // PICOJSON_USE_LOCALE

#ifndef PICOJSON_ASSERT
# define PICOJSON_ASSERT(e) do { if (! (e)) throw std::runtime_error(#e); } while (0)
#endif

#ifdef _MSC_VER
#define SNPRINTF _snprintf_s
#pragma warning(push)
#pragma warning(disable : 4244) // conversion from int to char
#else
#define SNPRINTF snprintf
#endif

#ifdef _MSC_VER
#define PICOJSON_FALSE_TEMPLATE_TYPE nullptr
#else
#define PICOJSON_FALSE_TEMPLATE_TYPE ((void*)0)
#endif

namespace pangolin {
namespace json {

enum {
    null_type,
    boolean_type,
    number_type,
    string_type,
    array_type,
    object_type
#ifdef PICOJSON_USE_INT64
    , int64_type
#endif
};

enum {
    INDENT_WIDTH = 2
};

struct null {};

class value {
public:
    typedef std::vector<value> array;
    typedef std::map<std::string, value> object;

    union _storage {
        bool boolean_;
        double number_;
#ifdef PICOJSON_USE_INT64
        int64_t int64_;
#endif
        std::string* string_;
        array* array_;
        object* object_;
    };
protected:
    int type_;
    _storage u_;
public:


    /////////////////////////////
    // Constructors / Destructor
    /////////////////////////////

    ~value();
    value();
    value(int type, bool);

    /////////////////////////////
    // Implicit type constructors
    /////////////////////////////

    value(bool b) : type_(boolean_type) {
        u_.boolean_ = b;
    }

#ifdef PICOJSON_USE_INT64
    value(short v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
    value(unsigned short v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
    value(int v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
    value(unsigned int v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
    value(long v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
    value(unsigned long v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
    value(long long v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
    value(unsigned long long v) : type_(int64_type) {
        u_.int64_ = static_cast<int64_t>(v);
    }
#endif

    value(float n) : type_(number_type) {
        if (
            #ifdef _MSC_VER
                ! _finite(n)
            #elif __cplusplus>=201103L || !(defined(isnan) && defined(isinf))
                std::isnan(n) || std::isinf(n)
            #else
                isnan(n) || isinf(n)
            #endif
                ) {
            throw std::overflow_error("");
        }
        u_.number_ = static_cast<double>(n);
    }

    value(double n) : type_(number_type) {
        if (
            #ifdef _MSC_VER
                ! _finite(n)
            #elif __cplusplus>=201103L || !(defined(isnan) && defined(isinf))
                std::isnan(n) || std::isinf(n)
            #else
                isnan(n) || isinf(n)
            #endif
                ) {
            throw std::overflow_error("");
        }
        u_.number_ = n;
    }

    value(const array& a);
    value(const object& o);

    value(const std::string& s);
    value(const char* s);
    value(const char* s, size_t len);

    value(const value& x);
    value& operator=(const value& x);

    /////////////////////////////
    // Query / get type
    /////////////////////////////

    void swap(value& x);
    template <typename T> bool is() const;
    template <typename T> const T& get() const;
    template <typename T> T& get();
    bool evaluate_as_boolean() const;
    size_t size() const;

    /////////////////////////////
    // Usage as array
    /////////////////////////////

    value& operator[](size_t idx);
    const value& operator[](size_t idx) const;
    bool contains(size_t idx) const;
    value& push_back(const value& = value() );

    /////////////////////////////
    // Usage as object
    /////////////////////////////

    value& operator[](const std::string& key);
    const value& operator[](const std::string& key) const;
    bool contains(const std::string& key) const;

    /////////////////////////////
    // Serialization
    /////////////////////////////

    std::string to_str() const;
    template <typename Iter> void serialize(Iter os, bool prettify = false) const;
    std::string serialize(bool prettify = false) const;
private:
    template <typename T> value(const T*); // intentionally defined to block implicit conversion of pointer to bool
    template <typename Iter> static void _indent(Iter os, int indent);
    template <typename Iter> void _serialize(Iter os, int indent) const;
    std::string _serialize(int indent) const;
};

typedef value::array array;
typedef value::object object;

inline value::value() : type_(null_type) {}

inline value::value(int type, bool) : type_(type) {
    switch (type) {
#define INIT(p, v) case p##type: u_.p = v; break
    INIT(boolean_, false);
    INIT(number_, 0.0);
#ifdef PICOJSON_USE_INT64
    INIT(int64_, 0);
#endif
    INIT(string_, new std::string());
    INIT(array_, new array());
    INIT(object_, new object());
#undef INIT
    default: break;
    }
}

inline value::value(const std::string& s) : type_(string_type) {
    u_.string_ = new std::string(s);
}

inline value::value(const array& a) : type_(array_type) {
    u_.array_ = new array(a);
}

inline value::value(const object& o) : type_(object_type) {
    u_.object_ = new object(o);
}

inline value::value(const char* s) : type_(string_type) {
    u_.string_ = new std::string(s);
}

inline value::value(const char* s, size_t len) : type_(string_type) {
    u_.string_ = new std::string(s, len);
}

inline value::~value() {
    switch (type_) {
#define DEINIT(p) case p##type: delete u_.p; break
    DEINIT(string_);
    DEINIT(array_);
    DEINIT(object_);
#undef DEINIT
    default: break;
    }
}

inline value::value(const value& x) : type_(x.type_) {
    switch (type_) {
#define INIT(p, v) case p##type: u_.p = v; break
    INIT(string_, new std::string(*x.u_.string_));
    INIT(array_, new array(*x.u_.array_));
    INIT(object_, new object(*x.u_.object_));
#undef INIT
    default:
        u_ = x.u_;
        break;
    }
}

inline value& value::operator=(const value& x) {
    if (this != &x) {
        this->~value();
        new (this) value(x);
    }
    return *this;
}

inline void value::swap(value& x) {
    std::swap(type_, x.type_);
    std::swap(u_, x.u_);
}

#define IS(ctype, jtype)			     \
    template <> inline bool value::is<ctype>() const { \
    return type_ == jtype##_type;		     \
}
IS(null, null)
IS(bool, boolean)
#ifdef PICOJSON_USE_INT64
IS(int64_t, int64)
#endif
IS(std::string, string)
IS(array, array)
IS(object, object)
#undef IS
template <> inline bool value::is<double>() const {
    return type_ == number_type
        #ifdef PICOJSON_USE_INT64
            || type_ == int64_type
        #endif
            ;
}

#define GET(ctype, var)						\
    template <> inline const ctype& value::get<ctype>() const {	\
    PICOJSON_ASSERT("type mismatch! call is<type>() before get<type>()" \
    && is<ctype>());				        \
    return var;							\
}								\
    template <> inline ctype& value::get<ctype>() {		\
    PICOJSON_ASSERT("type mismatch! call is<type>() before get<type>()"	\
    && is<ctype>());					\
    return var;							\
}
GET(bool, u_.boolean_)
GET(std::string, *u_.string_)
GET(array, *u_.array_)
GET(object, *u_.object_)
#ifdef PICOJSON_USE_INT64
GET(double, (type_ == int64_type && (const_cast<value*>(this)->type_ = number_type, const_cast<value*>(this)->u_.number_ = u_.int64_), u_.number_))
GET(int64_t, u_.int64_)
#else
GET(double, u_.number_)
#endif
#undef GET

inline bool value::evaluate_as_boolean() const {
    switch (type_) {
    case null_type:
        return false;
    case boolean_type:
        return u_.boolean_;
    case number_type:
        return u_.number_ != 0;
    case string_type:
        return ! u_.string_->empty();
    default:
        return true;
    }
}

inline size_t value::size() const
{
    PICOJSON_ASSERT("Type mismatch! Not array." && is<array>());
    return u_.array_->size();
}

inline const value& value::operator[](size_t idx) const {
    PICOJSON_ASSERT("Type mismatch! Not array." && is<array>());
    PICOJSON_ASSERT("Out of bounds." && idx < u_.array_->size() );
    return (*u_.array_)[idx];
}

inline value& value::operator[](size_t idx) {
    if( type_ == null_type ) {
        // instantiate as array
        type_ = array_type;
        u_.array_ = new array();
    }

    PICOJSON_ASSERT("Type mismatch! Not array." && is<array>());
    PICOJSON_ASSERT("Out of bounds." && idx < u_.array_->size() );
    return (*u_.array_)[idx];
}

inline bool value::contains(size_t idx) const {
    if( type_ == array_type) {
        return idx < u_.array_->size();
    }else{
        return false;
    }
}

inline value& value::push_back(const value& val)
{
    if( type_ == null_type ) {
        // instantiate as array
        type_ = array_type;
        u_.array_ = new array();
    }
    PICOJSON_ASSERT("Type mismatch! Not array." && is<array>());
    u_.array_->push_back( val );
    return u_.array_->back();
}

inline const value& value::operator[](const std::string& key) const {
    PICOJSON_ASSERT("Type mismatch! Not object." && is<object>() );
    object::const_iterator i = u_.object_->find(key);
    PICOJSON_ASSERT("Key not found." && i != u_.object_->end() );
    return i->second;
}

inline value& value::operator[](const std::string& key) {
    if( type_ == null_type ) {
        // instantiate as object
        type_ = object_type;
        u_.object_ = new object();
    }
    PICOJSON_ASSERT("Type mismatch! Not object." && is<object>());
    return u_.object_->operator [](key);
}

inline bool value::contains(const std::string& key) const {
    if( type_ == object_type) {
        object::const_iterator i = u_.object_->find(key);
        return i != u_.object_->end();
    }else{
        return false;
    }
}

inline std::string value::to_str() const {
    switch (type_) {
    case null_type:      return "null";
    case boolean_type:   return u_.boolean_ ? "true" : "false";
#ifdef PICOJSON_USE_INT64
    case int64_type: {
        char buf[sizeof("-9223372036854775808")];
        SNPRINTF(buf, sizeof(buf), "%" PRId64, u_.int64_);
        return buf;
    }
#endif
    case number_type:    {
        char buf[256];
        double tmp;
        SNPRINTF(buf, sizeof(buf), fabs(u_.number_) < (1ULL << 53) && modf(u_.number_, &tmp) == 0 ? "%.f" : "%.17g", u_.number_);
#if PICOJSON_USE_LOCALE
        char *decimal_point = localeconv()->decimal_point;
        if (strcmp(decimal_point, ".") != 0) {
            size_t decimal_point_len = strlen(decimal_point);
            for (char *p = buf; *p != '\0'; ++p) {
                if (strncmp(p, decimal_point, decimal_point_len) == 0) {
                    return std::string(buf, p) + "." + (p + decimal_point_len);
                }
            }
        }
#endif
        return buf;
    }
    case string_type:    return *u_.string_;
    case array_type:     return "array";
    case object_type:    return "object";
    default:             PICOJSON_ASSERT("value::to_str() assert failed." && 0);
#ifdef _MSC_VER
        __assume(0);
#endif
    }
    return std::string();
}

template <typename Iter> void copy(const std::string& s, Iter oi) {
    std::copy(s.begin(), s.end(), oi);
}

template <typename Iter> void serialize_str(const std::string& s, Iter oi) {
    *oi++ = '"';
    for (std::string::const_iterator i = s.begin(); i != s.end(); ++i) {
        switch (*i) {
#define MAP(val, sym) case val: copy(sym, oi); break
        MAP('"', "\\\"");
        MAP('\\', "\\\\");
        MAP('/', "\\/");
        MAP('\b', "\\b");
        MAP('\f', "\\f");
        MAP('\n', "\\n");
        MAP('\r', "\\r");
        MAP('\t', "\\t");
#undef MAP
        default:
            if (static_cast<unsigned char>(*i) < 0x20 || *i == 0x7f) {
                char buf[7];
                SNPRINTF(buf, sizeof(buf), "\\u%04x", *i & 0xff);
                copy(buf, buf + 6, oi);
            } else {
                *oi++ = *i;
            }
            break;
        }
    }
    *oi++ = '"';
}

template <typename Iter> void value::serialize(Iter oi, bool prettify) const {
    return _serialize(oi, prettify ? 0 : -1);
}

inline std::string value::serialize(bool prettify) const {
    return _serialize(prettify ? 0 : -1);
}

template <typename Iter> void value::_indent(Iter oi, int indent) {
    *oi++ = '\n';
    for (int i = 0; i < indent * INDENT_WIDTH; ++i) {
        *oi++ = ' ';
    }
}

template <typename Iter> void value::_serialize(Iter oi, int indent) const {
    switch (type_) {
    case string_type:
        serialize_str(*u_.string_, oi);
        break;
    case array_type: {
        *oi++ = '[';
        if (indent != -1) {
            ++indent;
        }
        for (array::const_iterator i = u_.array_->begin();
             i != u_.array_->end();
             ++i) {
            if (i != u_.array_->begin()) {
                *oi++ = ',';
            }
            if (indent != -1) {
                _indent(oi, indent);
            }
            i->_serialize(oi, indent);
        }
        if (indent != -1) {
            --indent;
            if (! u_.array_->empty()) {
                _indent(oi, indent);
            }
        }
        *oi++ = ']';
        break;
    }
    case object_type: {
        *oi++ = '{';
        if (indent != -1) {
            ++indent;
        }
        for (object::const_iterator i = u_.object_->begin();
             i != u_.object_->end();
             ++i) {
            if (i != u_.object_->begin()) {
                *oi++ = ',';
            }
            if (indent != -1) {
                _indent(oi, indent);
            }
            serialize_str(i->first, oi);
            *oi++ = ':';
            if (indent != -1) {
                *oi++ = ' ';
            }
            i->second._serialize(oi, indent);
        }
        if (indent != -1) {
            --indent;
            if (! u_.object_->empty()) {
                _indent(oi, indent);
            }
        }
        *oi++ = '}';
        break;
    }
    default:
        copy(to_str(), oi);
        break;
    }
    if (indent == 0) {
        *oi++ = '\n';
    }
}

inline std::string value::_serialize(int indent) const {
    std::string s;
    _serialize(std::back_inserter(s), indent);
    return s;
}

template <typename Iter> class input {
protected:
    Iter cur_, end_;
    int last_ch_;
    bool ungot_;
    int line_;
public:
    input(const Iter& first, const Iter& last) : cur_(first), end_(last), last_ch_(-1), ungot_(false), line_(1) {}
    int getc() {
        if (ungot_) {
            ungot_ = false;
            return last_ch_;
        }
        if (cur_ == end_) {
            last_ch_ = -1;
            return -1;
        }
        if (last_ch_ == '\n') {
            line_++;
        }
        last_ch_ = *cur_ & 0xff;
        ++cur_;
        return last_ch_;
    }
    void ungetc() {
        if (last_ch_ != -1) {
            PICOJSON_ASSERT(! ungot_);
            ungot_ = true;
        }
    }
    Iter cur() const { return cur_; }
    int line() const { return line_; }
    void skip_ws() {
        while (1) {
            int ch = getc();
            if (! (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r')) {
                ungetc();
                break;
            }
        }
    }
    bool expect(int expect) {
        skip_ws();
        if (getc() != expect) {
            ungetc();
            return false;
        }
        return true;
    }
    bool match(const std::string& pattern) {
        for (std::string::const_iterator pi(pattern.begin());
             pi != pattern.end();
             ++pi) {
            if (getc() != *pi) {
                ungetc();
                return false;
            }
        }
        return true;
    }
};

template<typename Iter> inline int _parse_quadhex(input<Iter> &in) {
    int uni_ch = 0, hex;
    for (int i = 0; i < 4; i++) {
        if ((hex = in.getc()) == -1) {
            return -1;
        }
        if ('0' <= hex && hex <= '9') {
            hex -= '0';
        } else if ('A' <= hex && hex <= 'F') {
            hex -= 'A' - 0xa;
        } else if ('a' <= hex && hex <= 'f') {
            hex -= 'a' - 0xa;
        } else {
            in.ungetc();
            return -1;
        }
        uni_ch = uni_ch * 16 + hex;
    }
    return uni_ch;
}

template<typename String, typename Iter> inline bool _parse_codepoint(String& out, input<Iter>& in) {
    int uni_ch;
    if ((uni_ch = _parse_quadhex(in)) == -1) {
        return false;
    }
    if (0xd800 <= uni_ch && uni_ch <= 0xdfff) {
        if (0xdc00 <= uni_ch) {
            // a second 16-bit of a surrogate pair appeared
            return false;
        }
        // first 16-bit of surrogate pair, get the next one
        if (in.getc() != '\\' || in.getc() != 'u') {
            in.ungetc();
            return false;
        }
        int second = _parse_quadhex(in);
        if (! (0xdc00 <= second && second <= 0xdfff)) {
            return false;
        }
        uni_ch = ((uni_ch - 0xd800) << 10) | ((second - 0xdc00) & 0x3ff);
        uni_ch += 0x10000;
    }
    if (uni_ch < 0x80) {
        out.push_back(uni_ch);
    } else {
        if (uni_ch < 0x800) {
            out.push_back(0xc0 | (uni_ch >> 6));
        } else {
            if (uni_ch < 0x10000) {
                out.push_back(0xe0 | (uni_ch >> 12));
            } else {
                out.push_back(0xf0 | (uni_ch >> 18));
                out.push_back(0x80 | ((uni_ch >> 12) & 0x3f));
            }
            out.push_back(0x80 | ((uni_ch >> 6) & 0x3f));
        }
        out.push_back(0x80 | (uni_ch & 0x3f));
    }
    return true;
}

template<typename String, typename Iter> inline bool _parse_string(String& out, input<Iter>& in, int delim='"') {
    while (1) {
        int ch = in.getc();
        if (ch < ' ') {
            in.ungetc();
            return false;
        } else if (ch == delim) {
            return true;
        } else if (ch == '\\') {
            if ((ch = in.getc()) == -1) {
                return false;
            }
            switch (ch) {
#define MAP(sym, val) case sym: out.push_back(val); break
            MAP('"', '\"');
            MAP('\\', '\\');
            MAP('/', '/');
            MAP('b', '\b');
            MAP('f', '\f');
            MAP('n', '\n');
            MAP('r', '\r');
            MAP('t', '\t');
#undef MAP
            case 'u':
                if (! _parse_codepoint(out, in)) {
                    return false;
                }
                break;
            default:
                return false;
            }
        } else {
            out.push_back(ch);
        }
    }
    return false;
}

template <typename Context, typename Iter> inline bool _parse_array(Context& ctx, input<Iter>& in) {
    if (! ctx.parse_array_start()) {
        return false;
    }
    size_t idx = 0;
    if (in.expect(']')) {
        return ctx.parse_array_stop(idx);
    }
    do {
        if (! ctx.parse_array_item(in, idx)) {
            return false;
        }
        idx++;
    } while (in.expect(','));
    return in.expect(']') && ctx.parse_array_stop(idx);
}

template <typename Context, typename Iter> inline bool _parse_object(Context& ctx, input<Iter>& in) {
    if (! ctx.parse_object_start()) {
        return false;
    }
    if (in.expect('}')) {
        return true;
    }
    do {
        std::string key;
        // Support " and ' delimited strings
        if( in.expect('"') ) {
            if( !_parse_string(key, in, '"') )
                return false;
        }else if (!in.expect('\'') || !_parse_string(key, in,'\'')  ) {
            return false;
        }
        if (!in.expect(':') || !ctx.parse_object_item(in, key)) {
            return false;
        }
    } while (in.expect(','));
    return in.expect('}');
}

template <typename Iter> inline std::string _parse_number(input<Iter>& in) {
    std::string num_str;
    while (1) {
        int ch = in.getc();
        if (('0' <= ch && ch <= '9') || ch == '+' || ch == '-'
                || ch == 'e' || ch == 'E') {
            num_str.push_back(ch);
        } else if (ch == '.') {
#if PICOJSON_USE_LOCALE
            num_str += localeconv()->decimal_point;
#else
            num_str.push_back('.');
#endif
        } else {
            in.ungetc();
            break;
        }
    }
    return num_str;
}

template <typename Context, typename Iter> inline bool _parse(Context& ctx, input<Iter>& in) {
    in.skip_ws();
    int ch = in.getc();
    switch (ch) {
#define IS(ch, text, op) case ch: \
    if (in.match(text) && op) { \
    return true; \
    } else { \
    return false; \
    }
    IS('n', "ull", ctx.set_null());
    IS('f', "alse", ctx.set_bool(false));
    IS('t', "rue", ctx.set_bool(true));
#undef IS
    case '"':
        return ctx.parse_string(in);
    case '[':
        return _parse_array(ctx, in);
    case '{':
        return _parse_object(ctx, in);
    default:
        if (('0' <= ch && ch <= '9') || ch == '-') {
            double f;
            char *endp;
            in.ungetc();
            std::string num_str = _parse_number(in);
            if (num_str.empty()) {
                return false;
            }
#ifdef PICOJSON_USE_INT64
            {
                errno = 0;
                intmax_t ival = strtoimax(num_str.c_str(), &endp, 10);
                if (errno == 0
                        && std::numeric_limits<int64_t>::min() <= ival
                        && ival <= std::numeric_limits<int64_t>::max()
                        && endp == num_str.c_str() + num_str.size()) {
                    ctx.set_int64(ival);
                    return true;
                }
            }
#endif
            f = strtod(num_str.c_str(), &endp);
            if (endp == num_str.c_str() + num_str.size()) {
                ctx.set_number(f);
                return true;
            }
            return false;
        }
        break;
    }
    in.ungetc();
    return false;
}

class deny_parse_context {
public:
    bool set_null() { return false; }
    bool set_bool(bool) { return false; }
#ifdef PICOJSON_USE_INT64
    bool set_int64(int64_t) { return false; }
#endif
    bool set_number(double) { return false; }
    template <typename Iter> bool parse_string(input<Iter>&) { return false; }
    bool parse_array_start() { return false; }
    template <typename Iter> bool parse_array_item(input<Iter>&, size_t) {
        return false;
    }
    bool parse_array_stop(size_t) { return false; }
    bool parse_object_start() { return false; }
    template <typename Iter> bool parse_object_item(input<Iter>&, const std::string&) {
        return false;
    }
};

class default_parse_context {
protected:
    value* out_;
public:
    default_parse_context(value* out) : out_(out) {}
    bool set_null() {
        *out_ = value();
        return true;
    }
    bool set_bool(bool b) {
        *out_ = value(b);
        return true;
    }
#ifdef PICOJSON_USE_INT64
    bool set_int64(int64_t i) {
        *out_ = value(i);
        return true;
    }
#endif
    bool set_number(double f) {
        *out_ = value(f);
        return true;
    }
    template<typename Iter> bool parse_string(input<Iter>& in) {
        *out_ = value(string_type, false);
        return _parse_string(out_->get<std::string>(), in);
    }
    bool parse_array_start() {
        *out_ = value(array_type, false);
        return true;
    }
    template <typename Iter> bool parse_array_item(input<Iter>& in, size_t) {
        array& a = out_->get<array>();
        a.push_back(value());
        default_parse_context ctx(&a.back());
        return _parse(ctx, in);
    }
    bool parse_array_stop(size_t) { return true; }
    bool parse_object_start() {
        *out_ = value(object_type, false);
        return true;
    }
    template <typename Iter> bool parse_object_item(input<Iter>& in, const std::string& key) {
        object& o = out_->get<object>();
        default_parse_context ctx(&o[key]);
        return _parse(ctx, in);
    }
private:
    default_parse_context(const default_parse_context&);
    default_parse_context& operator=(const default_parse_context&);
};

class null_parse_context {
public:
    struct dummy_str {
        void push_back(int) {}
    };
public:
    null_parse_context() {}
    bool set_null() { return true; }
    bool set_bool(bool) { return true; }
#ifdef PICOJSON_USE_INT64
    bool set_int64(int64_t) { return true; }
#endif
    bool set_number(double) { return true; }
    template <typename Iter> bool parse_string(input<Iter>& in) {
        dummy_str s;
        return _parse_string(s, in);
    }
    bool parse_array_start() { return true; }
    template <typename Iter> bool parse_array_item(input<Iter>& in, size_t) {
        return _parse(*this, in);
    }
    bool parse_array_stop(size_t) { return true; }
    bool parse_object_start() { return true; }
    template <typename Iter> bool parse_object_item(input<Iter>& in, const std::string&) {
        return _parse(*this, in);
    }
private:
    null_parse_context(const null_parse_context&);
    null_parse_context& operator=(const null_parse_context&);
};

// obsolete, use the version below
template <typename Iter> inline std::string parse(value& out, Iter& pos, const Iter& last) {
    std::string err;
    pos = parse(out, pos, last, &err);
    return err;
}

template <typename Context, typename Iter> inline Iter _parse(Context& ctx, const Iter& first, const Iter& last, std::string* err) {
    input<Iter> in(first, last);
    if (! _parse(ctx, in) && err != NULL) {
        char buf[64];
        SNPRINTF(buf, sizeof(buf), "syntax error at line %d near: ", in.line());
        *err = buf;
        while (1) {
            int ch = in.getc();
            if (ch == -1 || ch == '\n') {
                break;
            } else if (ch >= ' ') {
                err->push_back(ch);
            }
        }
    }
    return in.cur();
}

template <typename Iter> inline Iter parse(value& out, const Iter& first, const Iter& last, std::string* err) {
    default_parse_context ctx(&out);
    return _parse(ctx, first, last, err);
}

inline std::string parse(value& out, std::istream& is) {
    std::string err;
    parse(out, std::istreambuf_iterator<char>(is.rdbuf()),
          std::istreambuf_iterator<char>(), &err);
    return err;
}

template <typename T> struct last_error_t {
    static std::string s;
};
template <typename T> std::string last_error_t<T>::s;

inline void set_last_error(const std::string& s) {
    last_error_t<bool>::s = s;
}

inline const std::string& get_last_error() {
    return last_error_t<bool>::s;
}

inline bool operator==(const value& x, const value& y) {
    if (x.is<null>())
        return y.is<null>();
#define PICOJSON_CMP(type)					\
    if (x.is<type>())						\
    return y.is<type>() && x.get<type>() == y.get<type>()
    PICOJSON_CMP(bool);
    PICOJSON_CMP(double);
    PICOJSON_CMP(std::string);
    PICOJSON_CMP(array);
    PICOJSON_CMP(object);
#undef PICOJSON_CMP
    PICOJSON_ASSERT(0);
#ifdef _MSC_VER
    __assume(0);
#endif
    return false;
}

inline bool operator!=(const value& x, const value& y) {
    return ! (x == y);
}

} // namespace json
} // namespace pangolin

namespace std {
template<> inline void swap(pangolin::json::value& x, pangolin::json::value& y)
{
    x.swap(y);
}
}

inline std::istream& operator>>(std::istream& is, pangolin::json::value& x)
{
    pangolin::json::set_last_error(std::string());
    std::string err = pangolin::json::parse(x, is);
    if (! err.empty()) {
        pangolin::json::set_last_error(err);
        is.setstate(std::ios::failbit);
    }
    return is;
}

inline std::ostream& operator<<(std::ostream& os, const pangolin::json::value& x)
{
    x.serialize(std::ostream_iterator<char>(os));
    return os;
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER

#endif // picojson_h




#ifdef TEST_PICOJSON

#ifdef _MSC_VER
#pragma warning(disable : 4127) // conditional expression is constant
#endif // _MSC_VER

using namespace std;

static bool success = true;
static int test_num = 0;

static void ok(bool b, const char* name = "")
{
    if (! b)
        success = false;
    printf("%s %d - %s\n", b ? "ok" : "ng", ++test_num, name);
}

static void done_testing()
{
    printf("1..%d\n", test_num);
}

template <typename T> void is(const T& x, const T& y, const char* name = "")
{
    if (x == y) {
        ok(true, name);
    } else {
        ok(false, name);
    }
}

#include <algorithm>
#include <sstream>
#include <float.h>
#include <limits.h>

int main(void)
{
#if PICOJSON_USE_LOCALE
    setlocale(LC_ALL, "");
#endif // PICOJSON_USE_LOCALE

    // constructors
#define TEST(expr, expected) \
    is(pangolin::json::value expr .serialize(), string(expected), "pangolin::json::value" #expr)

    TEST( (true),  "true");
    TEST( (false), "false");
    TEST( (42.0),   "42");
    TEST( (string("hello")), "\"hello\"");
    TEST( ("hello"), "\"hello\"");
    TEST( ("hello", 4), "\"hell\"");

    {
        double a = 1;
        for (int i = 0; i < 1024; i++) {
            pangolin::json::value vi(a);
            std::stringstream ss;
            ss << vi;
            pangolin::json::value vo;
            ss >> vo;
            double b = vo.get<double>();
            if ((i < 53 && a != b) || fabs(a - b) / b > 1e-8) {
                printf("ng i=%d a=%.18e b=%.18e\n", i, a, b);
            }
            a *= 2;
        }
    }

#undef TEST

#define TEST(in, type, cmp, serialize_test) {				\
    pangolin::json::value v;							\
    const char* s = in;							\
    string err = pangolin::json::parse(v, s, s + strlen(s));			\
    ok(err.empty(), in " no error");					\
    ok(v.is<type>(), in " check type");					\
    is<type>(v.get<type>(), cmp, in " correct output");			\
    is(*s, '\0', in " read to eof");					\
    if (serialize_test) {						\
    is(v.serialize(), string(in), in " serialize");			\
}									\
}
    TEST("false", bool, false, true);
    TEST("true", bool, true, true);
    TEST("90.5", double, 90.5, false);
    TEST("1.7976931348623157e+308", double, DBL_MAX, false);
    TEST("\"hello\"", string, string("hello"), true);
    TEST("\"\\\"\\\\\\/\\b\\f\\n\\r\\t\"", string, string("\"\\/\b\f\n\r\t"),
         true);
    TEST("\"\\u0061\\u30af\\u30ea\\u30b9\"", string,
         string("a\xe3\x82\xaf\xe3\x83\xaa\xe3\x82\xb9"), false);
    TEST("\"\\ud840\\udc0b\"", string, string("\xf0\xa0\x80\x8b"), false);
#ifdef PICOJSON_USE_INT64
    TEST("0", int64_t, 0, true);
    TEST("-9223372036854775808", int64_t, std::numeric_limits<int64_t>::min(), true);
    TEST("9223372036854775807", int64_t, std::numeric_limits<int64_t>::max(), true);
#endif // PICOJSON_USE_INT64

#undef TEST

#define TEST(type, expr) {					       \
    pangolin::json::value v;						       \
    const char *s = expr;					       \
    string err = pangolin::json::parse(v, s, s + strlen(s));		       \
    ok(err.empty(), "empty " #type " no error");		       \
    ok(v.is<pangolin::json::type>(), "empty " #type " check type");	       \
    ok(v.get<pangolin::json::type>().empty(), "check " #type " array size"); \
}
    TEST(array, "[]");
    TEST(object, "{}");
#undef TEST

    {
        pangolin::json::value v;
        const char *s = "[1,true,\"hello\"]";
        string err = pangolin::json::parse(v, s, s + strlen(s));
        ok(err.empty(), "array no error");
        ok(v.is<pangolin::json::array>(), "array check type");
        is(v.get<pangolin::json::array>().size(), size_t(3), "check array size");
        ok(v.contains(0), "check contains array[0]");
        ok(v.get(0).is<double>(), "check array[0] type");
        is(v.get(0).get<double>(), 1.0, "check array[0] value");
        ok(v.contains(1), "check contains array[1]");
        ok(v.get(1).is<bool>(), "check array[1] type");
        ok(v.get(1).get<bool>(), "check array[1] value");
        ok(v.contains(2), "check contains array[2]");
        ok(v.get(2).is<string>(), "check array[2] type");
        is(v.get(2).get<string>(), string("hello"), "check array[2] value");
        ok(!v.contains(3), "check not contains array[3]");
    }

    {
        pangolin::json::value v;
        const char *s = "{ \"a\": true }";
        string err = pangolin::json::parse(v, s, s + strlen(s));
        ok(err.empty(), "object no error");
        ok(v.is<pangolin::json::object>(), "object check type");
        is(v.get<pangolin::json::object>().size(), size_t(1), "check object size");
        ok(v.contains("a"), "check contains property");
        ok(v.get("a").is<bool>(), "check bool property exists");
        is(v.get("a").get<bool>(), true, "check bool property value");
        is(v.serialize(), string("{\"a\":true}"), "serialize object");
        ok(!v.contains("z"), "check not contains property");
    }

#define TEST(json, msg) do {				\
    pangolin::json::value v;					\
    const char *s = json;				\
    string err = pangolin::json::parse(v, s, s + strlen(s));	\
    is(err, string("syntax error at line " msg), msg);	\
} while (0)
    TEST("falsoa", "1 near: oa");
    TEST("{]", "1 near: ]");
    TEST("\n\bbell", "2 near: bell");
    TEST("\"abc\nd\"", "1 near: ");
#undef TEST

    {
        pangolin::json::value v1, v2;
        const char *s;
        string err;
        s = "{ \"b\": true, \"a\": [1,2,\"three\"], \"d\": 2 }";
        err = pangolin::json::parse(v1, s, s + strlen(s));
        s = "{ \"d\": 2.0, \"b\": true, \"a\": [1,2,\"three\"] }";
        err = pangolin::json::parse(v2, s, s + strlen(s));
        ok((v1 == v2), "check == operator in deep comparison");
    }

    {
        pangolin::json::value v1, v2;
        const char *s;
        string err;
        s = "{ \"b\": true, \"a\": [1,2,\"three\"], \"d\": 2 }";
        err = pangolin::json::parse(v1, s, s + strlen(s));
        s = "{ \"d\": 2.0, \"a\": [1,\"three\"], \"b\": true }";
        err = pangolin::json::parse(v2, s, s + strlen(s));
        ok((v1 != v2), "check != operator for array in deep comparison");
    }

    {
        pangolin::json::value v1, v2;
        const char *s;
        string err;
        s = "{ \"b\": true, \"a\": [1,2,\"three\"], \"d\": 2 }";
        err = pangolin::json::parse(v1, s, s + strlen(s));
        s = "{ \"d\": 2.0, \"a\": [1,2,\"three\"], \"b\": false }";
        err = pangolin::json::parse(v2, s, s + strlen(s));
        ok((v1 != v2), "check != operator for object in deep comparison");
    }

    {
        pangolin::json::value v1, v2;
        const char *s;
        string err;
        s = "{ \"b\": true, \"a\": [1,2,\"three\"], \"d\": 2 }";
        err = pangolin::json::parse(v1, s, s + strlen(s));
        pangolin::json::object& o = v1.get<pangolin::json::object>();
        o.erase("b");
        pangolin::json::array& a = o["a"].get<pangolin::json::array>();
        pangolin::json::array::iterator i;
        i = std::remove(a.begin(), a.end(), pangolin::json::value(std::string("three")));
        a.erase(i, a.end());
        s = "{ \"a\": [1,2], \"d\": 2 }";
        err = pangolin::json::parse(v2, s, s + strlen(s));
        ok((v1 == v2), "check erase()");
    }

    ok(pangolin::json::value(3.0).serialize() == "3",
       "integral number should be serialized as a integer");

    {
        const char* s = "{ \"a\": [1,2], \"d\": 2 }";
        pangolin::json::null_parse_context ctx;
        string err;
        pangolin::json::_parse(ctx, s, s + strlen(s), &err);
        ok(err.empty(), "null_parse_context");
    }

    {
        pangolin::json::value v1, v2;
        v1 = pangolin::json::value(true);
        swap(v1, v2);
        ok(v1.is<pangolin::json::null>(), "swap (null)");
        ok(v2.get<bool>() == true, "swap (bool)");

        v1 = pangolin::json::value("a");
        v2 = pangolin::json::value(1.0);
        swap(v1, v2);
        ok(v1.get<double>() == 1.0, "swap (dobule)");
        ok(v2.get<string>() == "a", "swap (string)");

        v1 = pangolin::json::value(pangolin::json::object());
        v2 = pangolin::json::value(pangolin::json::array());
        swap(v1, v2);
        ok(v1.is<pangolin::json::array>(), "swap (array)");
        ok(v2.is<pangolin::json::object>(), "swap (object)");
    }

    {
        pangolin::json::value v;
        const char *s = "{ \"a\": 1, \"b\": [ 2, { \"b1\": \"abc\" } ], \"c\": {}, \"d\": [] }";
        string err;
        err = pangolin::json::parse(v, s, s + strlen(s));
        ok(err.empty(), "parse test data for prettifying output");
        ok(v.serialize() == "{\"a\":1,\"b\":[2,{\"b1\":\"abc\"}],\"c\":{},\"d\":[]}", "non-prettifying output");
        ok(v.serialize(true) == "{\n  \"a\": 1,\n  \"b\": [\n    2,\n    {\n      \"b1\": \"abc\"\n    }\n  ],\n  \"c\": {},\n  \"d\": []\n}\n", "prettifying output");
    }

    try {
        pangolin::json::value v(std::numeric_limits<double>::quiet_NaN());
        ok(false, "should not accept NaN");
    } catch (std::overflow_error e) {
        ok(true, "should not accept NaN");
    }

    try {
        pangolin::json::value v(std::numeric_limits<double>::infinity());
        ok(false, "should not accept infinity");
    } catch (std::overflow_error e) {
        ok(true, "should not accept infinity");
    }

    try {
        pangolin::json::value v(123.);
        ok(! v.is<bool>(), "is<wrong_type>() should return false");
        v.get<bool>();
        ok(false, "get<wrong_type>() should raise an error");
    } catch (std::runtime_error e) {
        ok(true, "get<wrong_type>() should raise an error");
    }

#ifdef PICOJSON_USE_INT64
    {
        pangolin::json::value v1((int64_t)123);
        ok(v1.is<int64_t>(), "is int64_t");
        ok(v1.is<double>(), "is double as well");
        ok(v1.serialize() == "123", "serialize the value");
        ok(v1.get<int64_t>() == 123, "value is correct as int64_t");
        ok(v1.get<double>(), "value is correct as double");

        ok(! v1.is<int64_t>(), "is no more int64_type once get<double>() is called");
        ok(v1.is<double>(), "and is still a double");

        const char *s = "-9223372036854775809";
        ok(pangolin::json::parse(v1, s, s + strlen(s)).empty(), "parse underflowing int64_t");
        ok(! v1.is<int64_t>(), "underflowing int is not int64_t");
        ok(v1.is<double>(), "underflowing int is double");
        ok(v1.get<double>() + 9.22337203685478e+18 < 65536, "double value is somewhat correct");
    }
#endif // PICOJSON_USE_INT64

    done_testing();

    return success ? 0 : 1;
}

#endif // TEST_PICOJSON
