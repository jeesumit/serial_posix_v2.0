#include <fcntl.h>  // for open
#include <unistd.h> // for read, close
#include <stdio.h>  // for printf
#include <errno.h>  // for errno
#include <stdlib.h> // for exit
#include <math.h>

double update_filter(double measurement, double predict, double gain);
double ConvertDegtoRad(double degree);
double ConvertRadtoDeg(double radians);
/*----------------------KALMAN GAIN PARAMETERS --------------------------*/
double cur_lat =0.0,cur_lng=0.0,prev_lat=0.0,prev_lng=0.0;
double pred_lat,pred_lng,sum_lat,sum_lng,out_lat=0.0,out_lng=0.0;
int n =0;
float gain=0.5;

/*-----------------------------------------------------------------------*/
/*-----------------------CONVERSION PARAMETER'S ------------------------*/
const double a = 6378137.0; // semi-major axis of the local geodetic datum ellipsoid.
double f, e = 0.0 , Rn=0.0; //eccentricity of the ellipsoid, flattening parameter
const double k=298.257223563; 
double x_vec,y_vec,z_vec,prevx_vec,prevy_vec,prevz_vec;
/*----------------------------------------------------------------------*/



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
     	
     	/*-------------WGS84 TO ECEF ----------------------------------------*/
   	
     f = 1 / (k);
     e = sqrt((2*f - (f*f)));
     Rn = a /(sqrt(1-(e*e)*(sin(ConvertDegtoRad(lat))*sin(ConvertDegtoRad(lat)))));    
     x_vec = ((Rn + hgt)*cos(ConvertDegtoRad(lat))*cos(ConvertDegtoRad(lng)));
     y_vec = ((Rn + hgt)*cos(ConvertDegtoRad(lat))*sin(ConvertDegtoRad(lng)));
     z_vec = ((Rn*(1-(e*e)) + hgt)*sin(ConvertDegtoRad(lat)));
     printf("x:%f y:%f z:%f \n",x_vec,y_vec,z_vec);
     

     
     	
     	
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

double ConvertDegtoRad(double degree) {
	double pi = 3.14159265359;
	return (degree * (pi /180));
}


double ConvertRadtoDeg(double radians) {
	double pi = 3.14159265359;
	return (radians * (180 /pi));
}


double update_filter(double measurement, double predict, double gain){
    double update = predict + gain*(measurement-predict);
    return update;
}


double distance(double lat1,double lat2,double lon1,double lon2){


	double  dlon = ConvertDegtoRad(lon2 - lon1) ;
	double dlat = ConvertDegtoRad(lat2 - lat1) ;
	double deg_lat1 = ConvertDegtoRad(lat1);
	double deg_lat2 = ConvertDegtoRad(lat2);
	double a= sin(dlat/2)*sin(dlat/2) + (cos(deg_lat1) * cos(deg_lat2))* (sin(dlon/2)*sin(dlon/2));
	double c=2 * atan2(sqrt(a),sqrt(1-a));
	double R= 6371000.0 ;
	double dist=c*R;
	return dist;
}






