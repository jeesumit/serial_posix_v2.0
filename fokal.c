#include <fcntl.h>  // for open
#include <unistd.h> // for read, close
#include <stdio.h>  // for printf
#include <errno.h>  // for errno
#include <stdlib.h> // for exit
#include <math.h>

double update_filter(double measurement, double predict, double gain);

/*----------------------KALMAN GAIN PARAMETERS --------------------------*/
double cur_lat =0.0,cur_lng=0.0,prev_lat=0.0,prev_lng=0.0;
double pred_lat,pred_lng,sum_lat,sum_lng,out_lat=0.0,out_lng=0.0;
int n =0;
float gain=0.5;

/*-----------------------------------------------------------------------*/



int main() {
    const char *filename = "inc/gnr.buf";
    int fd;
    ssize_t bytes_read;
    float spd,lat,lng,vel,head,hdop,hgt;
    char buffer[1024];
    char data[255];
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

    
     //printf("Data read: %s\n", buffer);
     sscanf(buffer,"speed:%f Lat:%f Lng:%f head:%f hdop:%f height:%f", &spd, &lat,&lng,&head,&hdop,&hgt);

     if(lat != 0.0 && lng != 0.0){
     	n+=1;
     	
     	cur_lat = lat;cur_lng = lng;
   	if(prev_lat==0.0 && prev_lng==0.0){
   	prev_lat = cur_lat;
   	prev_lng = cur_lng; 
   }
   
   	out_lat= update_filter(cur_lat, prev_lat, gain);
   	out_lng= update_filter(cur_lng, prev_lng, gain);
   
   	sum_lat+=out_lat;
   	sum_lng+=out_lng;
   
   	pred_lat= (sum_lat/n);
   	pred_lng= (sum_lng/n);
   
   	sprintf(data,"speed:%.2f Lat:%f Lng:%f head:%.2f hdop:%.2f height:%.2f LatK:%f LngK:%f\n",spd,lat,lng,head,hdop,hgt,out_lat,out_lng);
   	printf("%s ",data);
   
  	prev_lat=pred_lat;
  	prev_lng=pred_lng;
  
  	if(n>=10){
  		n=0;
  		sum_lat=0;
  		sum_lng=0;
  	}
     }

    if (close(fd) == -1) {
        perror("Error closing file");
        exit(EXIT_FAILURE);
    }
    }

    return 0;
}


double update_filter(double measurement, double predict, double gain){
    double update = predict + gain*(measurement-predict);
    return update;
}
