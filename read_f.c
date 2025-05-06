#include <fcntl.h>  // for open
#include <unistd.h> // for read, close
#include <stdio.h>  // for printf
#include <errno.h>  // for errno
#include <stdlib.h> // for exit

int main() {
    const char *filename = "inc/gnr.buf";
    int fd;
    ssize_t bytes_read;
    char buffer[1024];
    while(1){
    fd = open(filename, O_RDONLY);
    if (fd == -1) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }

    bytes_read = read(fd, buffer, sizeof(buffer));
    if (bytes_read == -1) {
        perror("Error reading file");
        close(fd);
        exit(EXIT_FAILURE);
    }

    printf("Bytes read: %zd\n", bytes_read);
    printf("Data read: %.*s\n", (int)bytes_read, buffer);

    if (close(fd) == -1) {
        perror("Error closing file");
        exit(EXIT_FAILURE);
    }
    }

    return 0;
}
