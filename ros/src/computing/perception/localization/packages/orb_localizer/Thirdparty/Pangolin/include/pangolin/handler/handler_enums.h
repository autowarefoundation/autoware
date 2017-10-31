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

#ifndef PANGOLIN_HANDLER_ENUMS_H
#define PANGOLIN_HANDLER_ENUMS_H

namespace pangolin
{

// Supported Key modifiers for GlobalKeyPressCallback.
// e.g. PANGO_CTRL + 'r', PANGO_SPECIAL + GLUT_KEY_RIGHT, etc.
const int PANGO_SPECIAL = 128;
const int PANGO_CTRL = -96;
const int PANGO_OPTN = 132;

// Ordinary keys
const int PANGO_KEY_TAB       = 9;
const int PANGO_KEY_ESCAPE    = 27;

// Special Keys (same as GLUT_ defines)
const int PANGO_KEY_F1        = 1;
const int PANGO_KEY_F2        = 2;
const int PANGO_KEY_F3        = 3;
const int PANGO_KEY_F4        = 4;
const int PANGO_KEY_F5        = 5;
const int PANGO_KEY_F6        = 6;
const int PANGO_KEY_F7        = 7;
const int PANGO_KEY_F8        = 8;
const int PANGO_KEY_F9        = 9;
const int PANGO_KEY_F10       = 10;
const int PANGO_KEY_F11       = 11;
const int PANGO_KEY_F12       = 12;
const int PANGO_KEY_LEFT      = 100;
const int PANGO_KEY_UP        = 101;
const int PANGO_KEY_RIGHT     = 102;
const int PANGO_KEY_DOWN      = 103;
const int PANGO_KEY_PAGE_UP   = 104;
const int PANGO_KEY_PAGE_DOWN = 105;
const int PANGO_KEY_HOME      = 106;
const int PANGO_KEY_END	      = 107;
const int PANGO_KEY_INSERT	  = 108;

enum MouseButton
{
    MouseButtonLeft = 1,
    MouseButtonMiddle = 2,
    MouseButtonRight = 4,
    MouseWheelUp = 8,
    MouseWheelDown = 16,
    MouseWheelRight = 32,
    MouseWheelLeft = 64,
};

enum KeyModifier
{
    KeyModifierShift = 1<<16,
    KeyModifierCtrl  = 1<<17,
    KeyModifierAlt   = 1<<18,
    KeyModifierCmd   = 1<<19,
    KeyModifierFnc   = 1<<20
};

enum InputSpecial
{
    InputSpecialScroll,
    InputSpecialZoom,
    InputSpecialRotate,
    InputSpecialTablet
};

}

#endif // PANGOLIN_HANDLER_ENUMS_H
