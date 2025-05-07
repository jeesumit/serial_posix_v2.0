#include <fcntl.h>   // For file control options
#include <string.h>
#include <termios.h> // For terminal interface control
#include <unistd.h>  // For read(), close()
#include <errno.h>   // For error numbers
#include <string.h>  // For strerror()
#include <stdio.h>   // For standard input/output
#include <stdlib.h>
#include "gnss.h"
#include <math.h>
int serial_port,bytes_written;
extern double lat,lng,vel,head,hdop,hgt;
extern long int time ,date;
extern int nsv,gpsqa; 
void port_init(const char *usbport);  
struct termios tty; 
int main(){
    char data[255];
    char read_buf [256];
    port_init("/dev/ttyS0");
    char filename[]="inc/gnr.buf";
   /* 
    int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd == -1) {
        perror("open");
        // Handle error (e.g., exit, return)
    } To save file 
    */
    while(1){
     
     int num_bytes = read(serial_port, read_buf, sizeof(read_buf));
     if (num_bytes < 0) {
        fprintf(stderr, "Error reading: %s\n", strerror(errno));
     }
     
     
     int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd == -1) {
        perror("openERROR");
        // Handle error (e.g., exit, return)
    }
    
     id_filter(read_buf);
     if(lat != 0.0 && lng != 0.0 && hgt !=0.0){
     sprintf(data,"speed:%.2f Lat:%f Lng:%f head:%.2f hdop:%.2f height:%.2f\n",vel,lat,lng,head,hdop,hgt);
     printf("%s ",data);
    
    ssize_t bytes_written = write(fd, data, strlen(data));
    if (bytes_written == -1) {
        perror("writeERROR");
        close(fd);
        // Handle error
    }
    }
    
    
     if (close(fd) == -1) {
        perror("closeERROR");
        // Handle error
    }
     
     }
     
    memset(data,'\0',sizeof(data));
    close(serial_port);
    tcsetattr(serial_port,TCSANOW,&tty);
    return 0;
}

void port_init(const char *usbport){

///const char *portname = "/dev/ttyUSB0"; // Replace with your port
    serial_port = open(usbport, O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        fprintf(stderr, "Error opening serial port '%s': %s\n", usbport, strerror(errno));
        exit(1);
    }
    
        
    if (tcgetattr(serial_port, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        close(serial_port);
        exit(1);
    }

    // Set baud rate
     cfsetospeed(&tty, B9600); 
     cfsetispeed(&tty, B9600);

    // Configure other settings (example: 8N1 - 8 data bits, no parity, 1 stop bit) 
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading, ignore control lines

    tty.c_lflag |= ICANON; // Disable canonical mode (line-by-line input)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of signals

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST; // Raw output

    tty.c_cc[VMIN] = 1;  // Read at least 1 character
    tty.c_cc[VTIME] = 0; // No timeout
    tcflush(serial_port,TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
       fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        close(serial_port);
        exit(1);
    }
    
}






/*



serial_port = open(portname, O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        fprintf(stderr, "Error opening serial port '%s': %s\n", portname, strerror(errno));
        return -1; // Or handle the error as appropriate
    }
    
        
    if (tcgetattr(serial_port, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        close(serial_port);
        return -1;
    }

    // Set baud rate
     cfsetospeed(&tty, B9600); 
     cfsetispeed(&tty, B9600);

    // Configure other settings (example: 8N1 - 8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading, ignore control lines

    tty.c_lflag |= ICANON; // Disable canonical mode (line-by-line input)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of signals

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST; // Raw output

    tty.c_cc[VMIN] = 0;  // Read at least 1 character
    tty.c_cc[VTIME] = 0; // No timeout

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
       fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        close(serial_port);
        return -1;
    }
void id_filter(char buffer[]){
char *fil;
int i=0 ,j=0,d=0,k=0,l=0;

fil=strtok(buffer,",");
while(fil!=NULL){
if(strcmp("$GNRMC",fil)==0){
i=0;
j=1;
printf("%s --FOUND %d\n",fil,i);
}

if(strcmp("$GNGGA",fil)==0){
k=0;
l=1;
printf("%s --FOUND %d\n",fil,k);
}



if(i==1 && j==1){
time=atoi(fil);
printf("Time:%s %f\n",fil,atof(fil));
}
if(i==2 && j==1){
if(strcmp("A",fil)==0){
printf("%s \n",fil);
j=1;
}else if(strcmp("V",fil)==0){
j=0;
}
}

if(i==3 && j==1){
lat = lat_filt(atof(fil));
printf("Lat:%f\n",lat);
}
if(i==4 && j==1){
printf("%s \n",fil);
}

if(i==5 && j==1){
//lngk = fil;
lng = lng_filt(atof(fil));
printf("Lng:%f\n",lng);
}

if(i==6 && j==1){
printf("%s \n ",fil);
}

if(i==7 && j==1){
vel=atof(fil);
printf("spd:%s %f \n",fil,atof(fil));
}

if(i==8 && j==1){
if(atof(fil) >=0.0 && atof(fil) <=360.0){
printf("Head:%s %f -- \n",fil,atof(fil));
head=atof(fil);
d=1;
}
else{
d=0;
date = atoi(fil);
printf("Date:%s %d \n",fil,atoi(fil));
}
}
if(i==9 && j==1 && d==1){
date=atoi(fil);
printf("Date:%s %d \n",fil,atoi(fil));
}
i+=1;

if(k==1 && l==1){
time = atoi(fil);
printf("Time:%s %f\n",fil,atof(fil));
}
if(k==2 && l==1){
lat = lat_filt(atof(fil));
printf("Lat:%f \n",lat);
}
if(k==3 && l==1){
printf("%s \n",fil);
}

if(k==4 && l==1){
lng = lng_filt(atof(fil));
printf("Lng:%f\n",lng);
}

if(k==5 && l==1){
printf("%s \n ",fil);
 }

if(k==6 && l==1){
gpsqa = atoi(fil);
printf("GPS Quality indicator:%s %d \n",fil,atoi(fil));
}

if(k==7 && l==1){
nsv = atoi(fil);
printf("Number of SV:%s %d \n",fil,atoi(fil));
}

if(k==8 && l==1){
hdop = atof(fil);
printf("HDOP:%s %f \n",fil,atof(fil));
}


if(k==9 && l==1){
hgt = atof(fil);
printf("Orthometric height:%s %f \n",fil,atof(fil));
}


if(k==10 && l==1){
printf(":%s \n",fil);
}

if(k==11 && l==1){
printf("Geoid separation:%s %f \n",fil,atof(fil));
}

if(k==12 && l==1){
printf(":%s \n",fil);
}

if(k==13 && l==1){
printf(":%s %f \n",fil,atof(fil));
}

if(k==14 && l==1){
printf("GPS Age:%s %d \n",fil,atoi(fil));
}

if(k==15 && l==1){
printf("Reference station ID:%s %d \n",fil,atoi(fil));
}
k+=1;
fil=strtok(NULL,",");
}
}
    
float lng_filt(float kef){
    float longitude = 0.0;
    float k_lng_deg=(kef*0.01);
    unsigned int deglng = (int)k_lng_deg;
    if(deglng > 68 && 97 > deglng){
        float seclng = (kef- (float)deglng*100)/60;
        longitude = (float)deglng + seclng;

    }

    return longitude;
}

float lat_filt(float def) {
    float latitude = 0.0;
    float k_lat_deg=(def*0.01);
    unsigned int deg = (int)k_lat_deg;
    if(deg > 8 && 37 > deg){
        float sec = (def- (float)deg*100)/60;
        latitude = (float)deg + sec;
    }

    return latitude;
}
*/
