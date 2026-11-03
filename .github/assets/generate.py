#!/usr/bin/env python3
"""Generate the SVG images that README.md uses.

The images follow the media kit of the Autoware documentation:
https://docs.autoware.org/main/home/media-kit/

Run it from the repository root. See .github/assets/README.md for the inputs.

    python3 .github/assets/generate.py --fontawesome FA_DIR --logos LOGOS [--demo-frame FRAME]

The script uses only the Python standard library.
"""

import argparse
import base64
from pathlib import Path
import re
import struct

ASSETS = Path(__file__).resolve().parent

# Media kit colors.
INDIGO = "#3f2ee3"
CYAN = "#00c8ff"
GREEN = "#00e678"
CRIMSON = "#dc143c"
NAVY = "#00002a"
BLUE = "#2a6df2"  # The end of the gradient of the primary button on the documentation site.
WHITE = "#ffffff"
GITHUB_TEXT = "#1f2328"  # The text color of GitHub in the light theme.

# cspell:ignore Segoe
FONT = "Montserrat, 'Segoe UI', Helvetica, Arial, sans-serif"

# Advance widths of the ASCII characters 32 to 126 in Montserrat, in units of 1/1000 of the font
# size. Measured in Chrome with canvas measureText on the Google Fonts Montserrat. The sums ignore
# kerning and are about 1% wider than the real text. The textLength attribute then makes the text
# fill the computed width in any fallback font.
ADVANCES = {
    600: "276,278,413,711,629,860,706,220,347,348,416,590,244,385,244,371,673,381,582,582,679,584,"
    "627,608,652,627,244,244,590,590,590,581,1034,749,761,718,826,670,637,772,810,319,527,729,"
    "599,955,810,842,726,842,731,629,602,789,729,1144,693,661,663,350,371,350,591,500,600,607,"
    "686,581,687,622,369,694,685,289,295,635,289,1053,685,645,686,686,420,515,424,681,578,919,"
    "573,578,532,370,303,370,590",
    700: "283,289,437,720,638,877,728,230,357,358,434,599,262,386,262,392,679,392,590,592,689,595,"
    "637,620,660,637,262,262,599,599,599,589,1035,766,765,724,826,671,639,771,808,328,541,740,"
    "604,955,808,844,732,844,735,638,618,788,746,1163,714,676,671,368,392,368,600,500,600,617,"
    "690,591,692,631,387,700,691,301,307,655,301,1049,691,655,690,690,431,531,435,687,598,937,"
    "595,598,543,391,309,391,599",
}

# The labels. To change one, edit it here and run the script.
PRIMARY_BUTTON = ("button-install-autoware", "Install Autoware", "solid/circle-down")
SECONDARY_BUTTON = ("button-run-in-simulation", "Run in simulation", "solid/gamepad")
NEWS_CHIPS = [
    ("chip-autoware-2-0-release-notes", "Autoware 2.0 release notes", "solid/rocket"),
    ("chip-autoware-index", "Autoware Index (community packages)", "solid/cubes"),
]
SOCIAL_CHIPS = [
    ("chip-license", "Apache-2.0 license", "solid/scale-balanced"),
    ("chip-discord", "Discord", "brands/discord"),
    ("chip-x", "X (Twitter)", "brands/x-twitter"),
    ("chip-linkedin", "LinkedIn", "brands/linkedin"),
]
USE_CASES = [
    ("use-case-robo-taxis", "Robo-taxis", "solid/taxi"),
    ("use-case-cargo-delivery", "Cargo delivery", "solid/truck-fast"),
    ("use-case-robo-bus-shuttles", "Robo-bus shuttles", "solid/bus"),
    ("use-case-privately-owned-vehicles", "Privately owned vehicles", "solid/car-side"),
]
CARD_ICONS = [
    "circle-down",
    "gamepad",
    "book",
    "compass-drafting",
    "circle-question",
    "circle-plus",
]
FEATURE_ICONS = ["robot", "eye", "location-dot", "route", "car", "gamepad"]


def gradient(name, stops, horizontal=True):
    x2, y2 = ("1", "0") if horizontal else ("1", "1")
    stop_tags = "".join(
        f'<stop offset="{offset}" stop-color="{color}" />' for offset, color in stops
    )
    return (
        f'<linearGradient id="{name}" x1="0" y1="0" x2="{x2}" y2="{y2}">'
        f"{stop_tags}</linearGradient>"
    )


BRAND_GRADIENT = gradient("brand", [(0, INDIGO), (0.55, CYAN), (1, GREEN)])
BUTTON_GRADIENT = gradient("button", [(0, INDIGO), (1, BLUE)], horizontal=False)
PILL_GRADIENT = gradient("pill", [(0, INDIGO), (1, BLUE)])
TILE_GRADIENT = gradient("tile", [(0, INDIGO), (1, CYAN)], horizontal=False)


class Generator:
    def __init__(self, fontawesome, logos):
        self.fontawesome = fontawesome
        self.logos = logos
        self.advances = {
            weight: [int(width) for width in table.split(",")] for weight, table in ADVANCES.items()
        }

    def text_width(self, label, size, weight):
        return sum(self.advances[weight][ord(char) - 32] for char in label) * size / 1000

    def text(self, x, cy, label, size, weight, color, anchor="start", adjust="spacingAndGlyphs"):
        width = self.text_width(label, size, weight)
        return (
            f'<text x="{x:g}" y="{cy:g}" fill="{color}" font-family="{FONT}" font-size="{size}" '
            f'font-weight="{weight}" dominant-baseline="central" text-anchor="{anchor}" '
            f'textLength="{width:.1f}" lengthAdjust="{adjust}">{label}</text>'
        )

    def icon(self, name):
        style, icon = name.split("/")
        source = (self.fontawesome / style / f"{icon}.svg").read_text()
        width, height = re.search(r'viewBox="0 0 (\d+) (\d+)"', source).groups()
        license_comment = re.search(r"<!--!.*?-->", source).group(0)
        path = re.search(r'\sd="([^"]+)"', source).group(1)
        return float(width), float(height), path, license_comment

    def glyph(self, name, cx, cy, size, color):
        width, height, path, _ = self.icon(name)
        scale = size / max(width, height)
        return (
            f'<svg x="{cx - width * scale / 2:g}" y="{cy - height * scale / 2:g}" '
            f'width="{width * scale:g}" height="{height * scale:g}" '
            f'viewBox="0 0 {width:g} {height:g}"><path fill="{color}" d="{path}" /></svg>'
        )

    def write(self, name, width, height, body, defs="", icon=None):
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:g}" height="{height:g}" '
            f'viewBox="0 0 {width:g} {height:g}">'
        ]
        if icon:
            parts.append(self.icon(icon)[3])
        if defs:
            parts.append(f"<defs>{defs}</defs>")
        parts += [body, "</svg>"]
        (ASSETS / f"{name}.svg").write_text("\n".join(parts) + "\n")

    # Buttons: 44 px pills on a canvas with room for the shadow of the primary button, so that
    # both buttons line up.
    def button(
        self, name, label, icon, fill, stroke, color, defs="", shadow=False, stroke_opacity=1
    ):
        height, margin, pad, icon_size, gap = 44, 8, 22, 17, 9
        width = pad + icon_size + gap + self.text_width(label, 15, 600) + pad
        cy = margin + height / 2
        rect = (
            f'<rect x="{margin + 0.5:g}" y="{margin + 0.5:g}" width="{width - 1:g}" '
            f'height="{height - 1}" rx="{(height - 1) / 2:g}" fill="{fill}" stroke="{stroke}"'
            f' stroke-opacity="{stroke_opacity:g}"'
        )
        if shadow:
            rect += ' filter="url(#shadow)"'
            defs += (
                '<filter id="shadow" x="-10%" y="-30%" width="120%" height="180%">'
                f'<feDropShadow dx="0" dy="4" stdDeviation="4" flood-color="{INDIGO}" '
                'flood-opacity="0.45" /></filter>'
            )
        body = "\n".join(
            [
                rect + " />",
                self.glyph(icon, margin + pad + icon_size / 2, cy, icon_size, color),
                self.text(margin + pad + icon_size + gap, cy, label, 15, 600, color),
            ]
        )
        self.write(name, width + 2 * margin, height + 2 * margin + 4, body, defs, icon)

    # News chips: a 1 px gradient border, an icon, a label and a crimson NEW tag.
    def news_chip(self, name, label, icon, fill, color):
        height, pad, icon_size, gap, tag_pad = 30, 13, 13, 7, 7
        tag_width = tag_pad + self.text_width("NEW", 10, 700) + 2 + tag_pad
        width = pad + icon_size + gap + self.text_width(label, 13, 600) + 8 + tag_width + 6
        cy = height / 2
        tag_x = width - 6 - tag_width
        new = self.text(tag_x + tag_width / 2, cy, "NEW", 10, 700, WHITE, "middle", "spacing")
        new = re.sub(r'textLength="([\d.]+)"', lambda m: f'textLength="{float(m[1]) + 2:.1f}"', new)
        body = "\n".join(
            [
                f'<rect x="0.5" y="0.5" width="{width - 1:g}" height="{height - 1:g}" '
                f'rx="{(height - 1) / 2:g}" fill="{fill}" stroke="url(#brand)" />',
                self.glyph(icon, pad + icon_size / 2, cy, icon_size, color),
                self.text(pad + icon_size + gap, cy, label, 13, 600, color),
                f'<rect x="{tag_x:g}" y="6" width="{tag_width:g}" height="{height - 12}" '
                f'rx="{(height - 12) / 2:g}" fill="{CRIMSON}" />',
                new,
            ]
        )
        self.write(name, width, height, body, BRAND_GRADIENT, icon)

    # Pills with a white icon and a white label: the social links and the use cases.
    def pill(self, name, label, icon, height, size, fill, defs=""):
        pad, icon_size, gap = height * 0.4, size + 1, size / 2
        width = pad + icon_size + gap + self.text_width(label, size, 600) + pad
        body = "\n".join(
            [
                f'<rect width="{width:g}" height="{height}" rx="{height / 2:g}" fill="{fill}" />',
                self.glyph(icon, pad + icon_size / 2, height / 2, icon_size, WHITE),
                self.text(pad + icon_size + gap, height / 2, label, size, 600, WHITE),
            ]
        )
        self.write(name, width, height, body, defs, icon)

    # Icon tiles: a 28 px tile and a transparent 8 px gap on its right, so that the title can
    # follow the image directly in the README.
    def tile(self, name, icon, defs, fill):
        size, gap = 28, 8
        body = "\n".join(
            [
                f'<rect width="{size}" height="{size}" rx="{size / 4:g}" fill="{fill}" />',
                self.glyph(icon, size / 2, size / 2, 14, WHITE),
            ]
        )
        self.write(f"icons/{name}", size + gap, size, body, defs, icon)

    def logo(self, file_name, x, y, width):
        source = (self.logos / file_name).read_text()
        view_width, view_height = map(
            float, re.search(r'viewBox="0 0 ([\d.]+) ([\d.]+)"', source).groups()
        )
        path = " ".join(re.search(r'\sd="([^"]+)"', source).group(1).split())
        height = width * view_height / view_width
        svg = (
            f'<svg x="{x:g}" y="{y:g}" width="{width:g}" height="{height:g}" '
            f'viewBox="0 0 {view_width:g} {view_height:g}"><path fill="{WHITE}" d="{path}" /></svg>'
        )
        return svg, height

    # Banners: a white logo on navy, as the media kit allows. The media kit does not allow a
    # recolored logo, so the banner gives the white logo its own dark background.
    def banner(self, name, width, height, body, defs=""):
        frame = (
            f'<clipPath id="round"><rect width="{width}" height="{height}" rx="16" /></clipPath>'
        )
        content = "\n".join(
            [
                '<g clip-path="url(#round)">',
                f'<rect width="{width}" height="{height}" fill="{NAVY}" />',
                body,
                "</g>",
                f'<rect x="0.5" y="0.5" width="{width - 1}" height="{height - 1}" rx="15.5" '
                f'fill="none" stroke="{WHITE}" stroke-opacity="0.12" />',
            ]
        )
        self.write(name, width, height, content, BRAND_GRADIENT + defs + frame)

    def banners(self):
        width, height = 880, 180
        logo, logo_height = self.logo("autoware-horizontal-white.svg", (width - 340) / 2, 46, 340)
        glows = (
            '<radialGradient id="glow1" cx="0.15" cy="0" r="0.6">'
            f'<stop offset="0" stop-color="{INDIGO}" stop-opacity="0.55" />'
            f'<stop offset="1" stop-color="{INDIGO}" stop-opacity="0" /></radialGradient>'
            '<radialGradient id="glow2" cx="0.9" cy="1" r="0.55">'
            f'<stop offset="0" stop-color="{CYAN}" stop-opacity="0.3" />'
            f'<stop offset="1" stop-color="{CYAN}" stop-opacity="0" /></radialGradient>'
        )
        body = "\n".join(
            [
                f'<rect width="{width}" height="{height}" fill="url(#glow1)" />',
                f'<rect width="{width}" height="{height}" fill="url(#glow2)" />',
                logo,
                f'<rect x="{(width - 120) / 2:g}" y="{46 + logo_height + 26:g}" width="120" '
                'height="4" rx="2" fill="url(#brand)" />',
            ]
        )
        self.banner("banner-autoware", width, height, body, glows)

        width, height, logo_width = 880, 110, 260
        logo_height = logo_width * 80.38 / 565.38
        logo, _ = self.logo(
            "autoware-foundation-one-line-white.svg",
            (width - logo_width) / 2,
            (height - logo_height) / 2 + 2,
            logo_width,
        )
        body = f'<rect width="{width}" height="4" fill="url(#brand)" />\n{logo}'
        self.banner("banner-autoware-foundation", width, height, body)

    # The demo image: a JPEG frame inside an SVG, which adds the rounded corners, the play button
    # and the credit label.
    def demo(self, frame):
        data = frame.read_bytes()
        width, height = jpeg_size(data)
        cx, cy = width / 2, height / 2
        image = base64.b64encode(data).decode()
        defs = (
            f'<clipPath id="round"><rect width="{width}" height="{height}" rx="18" /></clipPath>'
            '<filter id="shadow" x="-50%" y="-50%" width="200%" height="200%">'
            f'<feDropShadow dx="0" dy="6" stdDeviation="10" flood-color="{NAVY}" '
            'flood-opacity="0.5" /></filter>'
        )
        body = "\n".join(
            [
                '<g clip-path="url(#round)">',
                f'<image width="{width}" height="{height}" href="data:image/jpeg;base64,{image}" />',
                "</g>",
                f'<rect x="0.5" y="0.5" width="{width - 1}" height="{height - 1}" rx="18" '
                f'fill="none" stroke="{NAVY}" stroke-opacity="0.2" />',
                '<g filter="url(#shadow)">',
                f'<circle cx="{cx:g}" cy="{cy:g}" r="46" fill="{INDIGO}" />',
                f'<path d="M{cx - 13:g} {cy - 20:g}L{cx + 21:g} {cy:g}L{cx - 13:g} {cy + 20:g}Z" '
                f'fill="{WHITE}" />',
                "</g>",
                f'<rect x="{width - 150}" y="{height - 46}" width="132" height="30" rx="15" '
                f'fill="{NAVY}" fill-opacity="0.75" />',
                self.text(width - 84, height - 31, "Video: TIER IV", 14, 600, WHITE, "middle"),
            ]
        )
        self.write("obstacle-avoidance-demo", width, height, body, defs)

    def generate_all(self, demo_frame):
        name, label, icon = PRIMARY_BUTTON
        self.button(name, label, icon, "url(#button)", "none", WHITE, BUTTON_GRADIENT, shadow=True)

        # The secondary button and the news chips change with the theme, as on the documentation
        # site. GitHub picks a <picture> source by the system theme, not by the GitHub theme, so
        # each variant has an opaque fill and reads on light and dark pages alike.
        name, label, icon = SECONDARY_BUTTON
        self.button(f"{name}-light", label, icon, "#f2f0fd", "#b2abf4", INDIGO)
        self.button(f"{name}-dark", label, icon, NAVY, CYAN, CYAN, stroke_opacity=0.6)
        for name, label, icon in NEWS_CHIPS:
            self.news_chip(f"{name}-light", label, icon, WHITE, GITHUB_TEXT)
            self.news_chip(f"{name}-dark", label, icon, NAVY, WHITE)

        for name, label, icon in SOCIAL_CHIPS:
            self.pill(name, label, icon, 26, 12, INDIGO)
        for name, label, icon in USE_CASES:
            self.pill(name, label, icon, 36, 14, "url(#pill)", PILL_GRADIENT)

        (ASSETS / "icons").mkdir(exist_ok=True)
        for icon in CARD_ICONS:
            self.tile(f"card-{icon}", f"solid/{icon}", BUTTON_GRADIENT, "url(#button)")
        for icon in FEATURE_ICONS:
            self.tile(f"feature-{icon}", f"solid/{icon}", TILE_GRADIENT, "url(#tile)")

        # A transparent 1 px image. Each README card holds a 1000 px wide copy, floated left, so that
        # every card table fills the column and splits in the middle. GitHub sizes a table to its
        # content, and its max-width: 100% on images shrinks the spacer on a narrow screen.
        self.write("spacer", 1, 1, "")

        self.banners()
        if demo_frame:
            self.demo(demo_frame)


def jpeg_size(data):
    """Return the width and the height from the start-of-frame marker of a JPEG file."""
    index = 2
    while index < len(data):
        marker, length = data[index + 1], struct.unpack(">H", data[index + 2 : index + 4])[0]
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height, width = struct.unpack(">HH", data[index + 5 : index + 9])
            return width, height
        index += 2 + length
    raise ValueError("no start-of-frame marker in the JPEG file")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--fontawesome",
        type=Path,
        required=True,
        help="the folder of the Font Awesome Free 7 SVG icons",
    )
    parser.add_argument("--logos", type=Path, required=True, help="the media kit logos folder")
    parser.add_argument("--demo-frame", type=Path, help="the cropped JPEG frame of the demo video")
    args = parser.parse_args()
    Generator(args.fontawesome, args.logos).generate_all(args.demo_frame)


if __name__ == "__main__":
    main()
