#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

int main() {
    const char *portname = "/dev/ttyUSB0";
    int fd;
    struct termios tty;
    char message[] = "Hello, serial port!\n";
    ssize_t bytes_written;

    // Open the serial port
    fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Error opening serial port");
        return 1;
    }

    // Get current serial port settings
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error getting termios settings");
        close(fd);
        return 1;
    }

    // Configure serial port settings (example: 9600 baud, 8N1)
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable read and ignore control lines
    tty.c_lflag &= ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of signals
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes
    tty.c_oflag &= ~OPOST; // Disable output processing

    // Set the modified settings
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error setting termios settings");
        close(fd);
        return 1;
    }
    
    // Write data to the serial port
    bytes_written = write(fd, message, sizeof(message) - 1);
     if (bytes_written == -1) {
        perror("Error writing to serial port");
        close(fd);
        return 1;
    } else {
        printf("Wrote %zd bytes to %s\n", bytes_written, portname);
    }
    
    // Close the serial port
    close(fd);
    return 0;
}
