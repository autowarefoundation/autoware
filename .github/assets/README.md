<!-- cspell:ignore svgs -->

# README images

The SVG files in this folder are the images of the repository `README.md`. They follow the [media kit](https://docs.autoware.org/main/home/media-kit/) of the Autoware documentation. `generate.py` makes all of them. Do not edit the SVG files by hand.

## Change a label or an image

1. Edit the label or the icon name at the top of `generate.py`.
2. Get the inputs (see below).
3. Run the script from the repository root:

   ```bash
   python3 .github/assets/generate.py \
     --fontawesome /path/to/fontawesome-free/svgs \
     --logos /path/to/autoware-documentation/docs/home/media-kit/logos
   ```

4. Run `pre-commit run --files .github/assets/*.svg .github/assets/icons/*.svg`. The `prettier-svg` hook formats the new files.
5. Look at the README on github.com in the light and the dark theme before you open the pull request.

The script computes the width of each label from the character widths of Montserrat. So a new label needs no manual measurement.

## Inputs

- Font Awesome Free 7.3.0: the `svgs` folder of the `@fortawesome/fontawesome-free` npm package. The documentation site uses the same icons. Use version 7.3.0, because another version can draw an icon differently. To get it, run `npm pack @fortawesome/fontawesome-free@7.3.0` and unpack the `.tgz` file. The folder is `package/svgs`.
- The media kit logos: the `docs/home/media-kit/logos` folder of [autowarefoundation/autoware-documentation](https://github.com/autowarefoundation/autoware-documentation).
- The demo frame (optional): only to change the demo image. Without `--demo-frame`, the script keeps `obstacle-avoidance-demo.svg` as it is.

To make the demo frame, crop the poster of the demo video in the documentation repository. The crop removes the RViz toolbar and the status bar:

```bash
ffmpeg -i docs/assets/videos/obstacle-avoidance-demo.jpg -vf "crop=1268:660:0:41" -q:v 3 frame.jpg
```

Then add `--demo-frame frame.jpg` to the command above.

## Design rules

- Every image has an opaque background. GitHub picks a `<picture>` source by the system theme, not by the GitHub theme. So an image must read on the light, dark and dark dimmed themes, whichever variant GitHub shows.
- The logos are white on navy banners. The media kit does not allow a recolored logo.
- White text has a contrast of 4.5:1 or more on each fill.

## License

The icons are from Font Awesome Free, under the CC BY 4.0 license. Each SVG file that holds an icon keeps the license comment of Font Awesome. Keep this comment when you change the files.
