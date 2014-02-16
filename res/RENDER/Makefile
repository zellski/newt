rendrib=/home/zell/projects/thesis/gfx/BMRT/bin/rendrib

snapshot.mpg:	snapshot.dat
	$(rendrib) -res 256 160 snapshot.dat
	mogrify -format rgb *.tiff
	transcode -g 256x160 -x rawlist=rgb,null -y ffmpeg -o test.foo -F mpeg4 -i filelist 
