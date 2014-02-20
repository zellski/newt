# A script to help renders a .rib file and turns it into a movie.
# 

cd `dirname $0`/..

# This is the PIXIE renderer. Note that the 2.2.6 version has broken shaders.
# See here for fix: http://sourceforge.net/p/pixie/patches/15/
export PIXIEHOME=/usr/local/Cellar/pixie/2.2.6
RENDER="${PIXIEHOME}/bin/rndr"

# These can be substituted for absolute paths if you have ornery installations
FFMPEG="ffmpeg"
FFPLAY="ffplay"

DIR="render-`date +"%Y%m%d-%H%M%S"`"

mkdir $DIR

cp -p ./res/snapshot.dat $DIR/snapshot.rib

cd $DIR

${RENDER} snapshot.rib
${FFMPEG} -i %03d.tiff render.avi
${FFPLAY} render.avi

