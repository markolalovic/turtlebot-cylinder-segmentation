#!/bin/bash

ffmpeg -i demo.mp4 -vf "fps=10,scale=320:-1:flags=lanczos" -c:v pam -f image2pipe - | convert -delay 10 - -loop 0 -layers optimize demo.gif

gifsicle -O3 --lossy=30  demo.gif -o demo_compressed.gif

convert demo_compressed.gif -verbose -coalesce -layers OptimizeFrame demo_optframe.gif

gifsicle -O2 demo_optframe.gif -o demo_optimized.gif

