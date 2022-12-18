import cairo
import rsvg


img = cairo.ImageSurface(cairo.FORMAT_ARGB32, 640,480)

ctx = cairo.Context(img)

## handle= rsvg.Handle(None, str(<svg data>))
# or, for in memory SVG data:
#handle= rsvg.Handle(None, str(<svg data>))

handle = rsvg.Handle("qr4x4_1000-19.svg")

handle.render_cairo(ctx)

img.write_to_png("qr4x4_1000-19.png")
