#ifndef PANGOLIN_CONFIG_H
#define PANGOLIN_CONFIG_H

/*
 * Configuration Header for Pangolin
 */

/// Version
#define PANGOLIN_VERSION_MAJOR 
#define PANGOLIN_VERSION_MINOR 
#define PANGOLIN_VERSION_STRING ""

/// Pangolin options
#define BUILD_PANGOLIN_GUI
#define BUILD_PANGOLIN_VARS
/* #undef BUILD_PANGOLIN_VIDEO */

/// Configured libraries
/* #undef HAVE_CUDA */
/* #undef HAVE_PYTHON */

/* #undef CPP11_NO_BOOST */
/* #undef HAVE_EIGEN */
/* #undef HAVE_TOON */

/* #undef HAVE_DC1394 */
/* #undef HAVE_V4L */
/* #undef HAVE_OPENNI */
/* #undef HAVE_OPENNI2 */
/* #undef HAVE_UVC */
/* #undef HAVE_DEPTHSENSE */
/* #undef HAVE_TELICAM */
/* #undef HAVE_PLEORA */

/* #undef HAVE_FFMPEG */
/* #undef HAVE_FFMPEG_MAX_ANALYZE_DURATION2 */
/* #undef HAVE_FFMPEG_AVFORMAT_ALLOC_OUTPUT_CONTEXT2 */

#define HAVE_GLEW
/* #undef GLEW_STATIC */

/* #undef HAVE_GLUT */
/* #undef HAVE_FREEGLUT */
/* #undef HAVE_APPLE_OPENGL_FRAMEWORK */
/* #undef HAVE_MODIFIED_OSXGLUT */
/* #undef HAVE_GLES */
/* #undef HAVE_GLES_2 */
/* #undef HAVE_OCULUS */

/* #undef HAVE_PNG */
/* #undef HAVE_JPEG */
/* #undef HAVE_TIFF */
/* #undef HAVE_OPENEXR */

/// Platform
#define _UNIX_
/* #undef _WIN_ */
/* #undef _OSX_ */
/* #undef _LINUX_ */
/* #undef _ANDROID_ */
/* #undef _IOS_ */

/// Compiler
#define _GCC_
/* #undef _CLANG_ */
/* #undef _MSVC_ */

/// Defines generated when calling into Pangolin API. Not to be
/// used in compiled library code, only inlined header code.
#if (__cplusplus > 199711L) || (_MSC_VER >= 1700)
#define CALLEE_HAS_CPP11
#define CALLEE_HAS_RVALREF
#endif

#if (__cplusplus > 199711L) || (_MSC_VER >= 1800)
#define CALLEE_HAS_VARIADIC_TEMPLATES
#endif

#endif //PANGOLIN_CONFIG_H
