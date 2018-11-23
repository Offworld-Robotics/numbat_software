#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

void main(int argc, char **argv)
{
    const char *filename;
    int fd;

    if (argc < 2) {
        printf("Give USB device name as parameter\n");
        exit(1);
    }
    filename = argv[1];
    fd = open(filename, O_WRONLY);
    if (fd == -1) {
        perror("USB device open failed");
        exit(2);
    }
    if (ioctl(fd, USBDEVFS_RESET, 0) == -1) {
        perror("USBDEVFS_RESET device ioctl failed");
        exit(3);
    }
    close(fd);
}
