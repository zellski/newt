snapshot.mpg:	snapshot.dat
	rendrib -res 256 160 snapshot.dat
	for i in *.tiff; do echo $$i; convert $$i `basename $$i .tiff`.yuv; done
	mpeg_encode mpeg.param 
