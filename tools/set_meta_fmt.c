// set_meta_fmt.c - set V4L2 META format on a CFE video node.
// Works around v4l2-ctl rejecting non-standard fourccs (e.g. 'RGGB').
//
// usage: ./set_meta_fmt /dev/videoN <width> <height> <fourcc(4 chars)>
//   e.g. ./set_meta_fmt /dev/video0 100 38 RGGB
//
// NOTE: like set_fmt, needs kernel >= 5.17 uapi headers (meta width/height);
// build it on the target Pi.
//
// build on the Pi: gcc -o set_meta_fmt set_meta_fmt.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

int main(int argc, char **argv)
{
	if (argc != 5 || strlen(argv[4]) != 4) {
		fprintf(stderr, "usage: %s /dev/videoN width height fourcc(4chars)\n",
			argv[0]);
		return 1;
	}

	int fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("open");
		return 1;
	}

	struct v4l2_format f;
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_META_CAPTURE;
	f.fmt.meta.width = atoi(argv[2]);
	f.fmt.meta.height = atoi(argv[3]);
	f.fmt.meta.dataformat = v4l2_fourcc(argv[4][0], argv[4][1],
			argv[4][2], argv[4][3]);

	if (ioctl(fd, VIDIOC_S_FMT, &f)) {
		perror("VIDIOC_S_FMT(meta)");
		close(fd);
		return 1;
	}

	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_META_CAPTURE;
	if (ioctl(fd, VIDIOC_G_FMT, &f)) {
		perror("VIDIOC_G_FMT(meta)");
		close(fd);
		return 1;
	}

	printf("meta fmt now: %u x %u fourcc=%c%c%c%c buffersize=%u\n",
	       f.fmt.meta.width, f.fmt.meta.height,
	       (char)(f.fmt.meta.dataformat & 0xff),
	       (char)((f.fmt.meta.dataformat >> 8) & 0xff),
        (char)((f.fmt.meta.dataformat >> 16) & 0xff),
	       (char)((f.fmt.meta.dataformat >> 24) & 0xff),
        f.fmt.meta.buffersize);

	close(fd);
	return 0;
}
