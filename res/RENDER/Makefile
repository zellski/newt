# From newt/res, do make -f RENDER/Makefile.

rendrib=/home/zell/newt/BMRT2.6/bin/rendrib
transcode=/usr/bin/transcode

snapshot.mpg:	snapshot.dat
	rm -rf *.tiff
	$(rendrib) -res 512 512 snapshot.dat
	$(transcode) -z --export_fps 25 -yffmpeg -Fmpeg4 -i 000.tiff -o snapshot.avi
