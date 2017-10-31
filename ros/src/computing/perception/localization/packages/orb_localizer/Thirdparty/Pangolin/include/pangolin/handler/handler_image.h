#include <pangolin/display/viewport.h>
#include <pangolin/handler/handler.h>
#include <pangolin/plot/range.h>

namespace pangolin
{

class ImageViewHandler : public Handler
{
public:
    ImageViewHandler(size_t w, size_t h)
        : linked_view_handler(0),
          rview_default(-0.5,w-0.5,h-0.5,0-0.5),
          rview_max(-0.5,w-0.5,-0.5,h-0.5),
          rview(rview_default), target(rview),
          use_nn(false)
    {
    }

    void UpdateView()
    {
        // TODO: Base this on current framerate.
        const float animate_factor = 1.0f / 5.0f;

        if( linked_view_handler ) {
            // Synchronise rview and target with linked plotter
            rview = linked_view_handler->rview;
            target = linked_view_handler->target;
            selection = linked_view_handler->selection;
        }else{
            // Clamp target to image dimensions.
            AdjustScale();
            AdjustTranslation();

            // Animate view window toward target
            pangolin::XYRangef d = target - rview;
            rview += d * animate_factor;
        }
    }

    void ScreenToImage(Viewport& v, int xpix, int ypix, float& ximg, float& yimg)
    {
        ximg = rview.x.min + rview.x.Size() * (xpix - v.l) / (float)v.w;
        yimg = rview.y.min + rview.y.Size() * (ypix - v.b) / (float)v.h;
    }

    bool UseNN() const
    {
        return use_nn;
    }

    pangolin::XYRangef& GetViewToRender()
    {
        return rview;
    }

    pangolin::XYRangef& GetView()
    {
        return target;
    }

    pangolin::XYRangef& GetDefaultView()
    {
        return rview_default;
    }

    pangolin::XYRangef& GetSelection()
    {
        return selection;
    }

    void SetView(const pangolin::XYRangef& range)
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        tv.rview = range;
        tv.target = range;
    }

    void SetViewSmooth(const pangolin::XYRangef& range)
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        tv.target = range;
    }

    void ScrollView(float x, float y)
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        ScrollViewSmooth(x,y);
        tv.rview.x += x;
        tv.rview.y += y;
    }

    void ScrollViewSmooth(float x, float y)
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        tv.target.x += x;
        tv.target.y += y;
    }

    void ScaleView(float x, float y, float cx, float cy)
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        ScaleViewSmooth(x,y,cx,cy);
        tv.rview.x.Scale(x,cx);
        tv.rview.y.Scale(y,cy);
    }

    void ScaleViewSmooth(float x, float y, float cx, float cy)
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        tv.target.x.Scale(x,cx);
        tv.target.y.Scale(y,cy);
    }

    void ResetView()
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        tv.target = tv.rview_default;
    }

    ///////////////////////////////////////////////////////
    /// pangolin::Handler
    ///////////////////////////////////////////////////////

    void Keyboard(View&, unsigned char key, int x, int y, bool pressed) PANGOLIN_OVERRIDE
    {
        XYRangef& sel = linked_view_handler ? linked_view_handler->selection : selection;
        const float mvfactor = 1.0f / 10.0f;
        const float c[2] = { rview.x.Mid(), rview.y.Mid() };


        if(pressed) {
            if(key == '\r') {
                if( sel.Area() != 0.0f && std::isfinite(sel.Area()) ) {
                    // Set view to equal selection
                    SetViewSmooth(sel);

                    // Reset selection
                    sel.x.max = sel.x.min;
                    sel.y.max = sel.y.min;
                }
            }else if(key == 'n') {
                use_nn = !use_nn;
            }else if(key == 'l') {
                if(to_link) {
                    linked_view_handler = to_link;
                    to_link = 0;
                }else{
                    to_link = this;
                }
            }else if(key == PANGO_SPECIAL + PANGO_KEY_LEFT) {
                const float w = target.x.Size();
                const float dx = mvfactor*w;
                ScrollViewSmooth(-dx, 0);
            }else if(key == PANGO_SPECIAL + PANGO_KEY_RIGHT) {
                const float w = target.x.Size();
                const float dx = mvfactor*w;
                ScrollViewSmooth(+dx, 0);
            }else if(key == PANGO_SPECIAL + PANGO_KEY_DOWN) {
                const float h = target.y.Size();
                const float dy = mvfactor*h;
                ScrollViewSmooth(0, -dy);
            }else if(key == PANGO_SPECIAL + PANGO_KEY_UP) {
                const float h = target.y.Size();
                const float dy = mvfactor*h;
                ScrollViewSmooth(0, +dy);
            }else if(key == '=') {
                ScaleViewSmooth(0.5, 0.5, c[0], c[1]);
            }else if(key == '-') {
                ScaleViewSmooth(2.0, 2.0, c[0], c[1]);
            }else if(key == '#') {
                ResetView();
            }
        }
    }

    void Mouse(View& view, pangolin::MouseButton button, int x, int y, bool pressed, int button_state) PANGOLIN_OVERRIDE
    {
        XYRangef& sel = linked_view_handler ? linked_view_handler->selection : selection;
        ScreenToImage(view.v, x, y, hover[0], hover[1]);

        const float scinc = 1.05f;
        const float scdec = 1.0f/scinc;

        if(button_state & KeyModifierCtrl) {
            const float mvfactor = 1.0f/20.0f;

            if(button == MouseWheelUp) {
                ScrollViewSmooth(0.0f, +mvfactor*rview.y.Size() );
            }else if(button == MouseWheelDown) {
                ScrollViewSmooth(0.0f, -mvfactor*rview.y.Size() );
            }else if(button == MouseWheelLeft) {
                ScrollViewSmooth(+mvfactor*rview.x.Size(), 0.0f );
            }else if(button == MouseWheelRight) {
                ScrollViewSmooth(-mvfactor*rview.x.Size(), 0.0f );
            }
        }else{
            if(button == MouseButtonLeft) {
                // Update selected range
                if(pressed) {
                    sel.x.min = hover[0];
                    sel.y.min = hover[1];
                }
                sel.x.max = hover[0];
                sel.y.max = hover[1];
            }else if(button == MouseWheelUp) {
                ScaleViewSmooth(scdec, scdec, hover[0], hover[1]);
            }else if(button == MouseWheelDown) {
                ScaleViewSmooth(scinc, scinc, hover[0], hover[1]);
            }
        }

        FixSelection(sel);
        last_mouse_pos[0] = x;
        last_mouse_pos[1] = y;
    }

    void MouseMotion(View& view, int x, int y, int button_state) PANGOLIN_OVERRIDE
    {
        XYRangef& sel = linked_view_handler ? linked_view_handler->selection : selection;
        const int d[2] = {x-last_mouse_pos[0], y-last_mouse_pos[1]};

        // Update hover status (after potential resizing)
        ScreenToImage(view.v, x, y, hover[0], hover[1]);

        if( button_state == MouseButtonLeft )
        {
            // Update selected range
            sel.x.max = hover[0];
            sel.y.max = hover[1];
        }else if(button_state == MouseButtonRight )
        {
            Special(view, InputSpecialScroll, x, y, d[0], -d[1], 0.0f, 0.0f, button_state);
        }

        last_mouse_pos[0] = x;
        last_mouse_pos[1] = y;

    }

    void PassiveMouseMotion(View&, int x, int y, int button_state) PANGOLIN_OVERRIDE
    {
    }

    void Special(View& view, pangolin::InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state) PANGOLIN_OVERRIDE
    {
        ScreenToImage(view.v, x, y, hover[0], hover[1]);

        if(inType == InputSpecialScroll) {
            const float d[2] = {p1,p2};
            const float is[2] = {rview.x.Size(),rview.y.Size() };
            const float df[2] = {is[0]*d[0]/(float)view.v.w, is[1]*d[1]/(float)view.v.h};
            ScrollView(-df[0], df[1]);
        } else if(inType == InputSpecialZoom) {
            float scale = 1.0 - p1;
            ScaleView(scale, scale, hover[0], hover[1]);
        }

        // Update hover status (after potential resizing)
        ScreenToImage( view.v, (int)x, (int)y, hover[0], hover[1]);
    }

protected:
    void FixSelection(XYRangef& sel)
    {
        // Make sure selection matches sign of current viewport
        if( (sel.x.min<sel.x.max) != (rview.x.min<rview.x.max) ) {
            std::swap(sel.x.min, sel.x.max);
        }
        if( (sel.y.min<sel.y.max) != (rview.y.min<rview.y.max) ) {
            std::swap(sel.y.min, sel.y.max);
        }
    }

    void AdjustScale()
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        if(tv.target.x.AbsSize() > tv.rview_max.x.AbsSize())
            tv.target.x.Scale(tv.rview_max.x.AbsSize() / tv.target.x.AbsSize(), tv.target.x.Mid());
        if(tv.target.y.AbsSize() > tv.rview_max.y.AbsSize())
            tv.target.y.Scale(tv.rview_max.y.AbsSize() / tv.target.y.AbsSize(), tv.target.y.Mid());
    }

    void AdjustTranslation()
    {
        ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
        if( tv.target.x.max > tv.rview_max.x.max) tv.target.x -= tv.target.x.max - tv.rview_max.x.max;
        if( tv.target.x.min < tv.rview_max.x.min) tv.target.x -= tv.target.x.min - tv.rview_max.x.min;
        if( tv.target.y.min > tv.rview_max.y.max) tv.target.y -= tv.target.y.min - tv.rview_max.y.max;
        if( tv.target.y.max < tv.rview_max.y.min) tv.target.y -= tv.target.y.max - tv.rview_max.y.min;
    }

    static ImageViewHandler* to_link;
    static float animate_factor;

    ImageViewHandler* linked_view_handler;

    pangolin::XYRangef rview_default;
    pangolin::XYRangef rview_max;
    pangolin::XYRangef rview;
    pangolin::XYRangef target;
    pangolin::XYRangef selection;

    float hover[2];
    int last_mouse_pos[2];

    bool use_nn;
};

float ImageViewHandler::animate_factor = 1.0f / 2.0f;
ImageViewHandler* ImageViewHandler::to_link = 0;

}
