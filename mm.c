#include "mm.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

double update_filter(double measurement, double predict, double gain){
	double update = predict + gain*(measurement-predict);
	return update;

}

float ratof(char *arr)
{
  float val = 0;
  int afterdot=0;
  float scale=1;
  int neg = 0;

  if (*arr == '-') {
    arr++;
    neg = 1;
  }
  while (*arr) {
    if (afterdot) {
      scale = scale/10;
      val = val + (*arr-'0')*scale;
    } else {
      if (*arr == '.')
    afterdot++;
      else
    val = val * 10.0 + (*arr - '0');
    }
    arr++;
  }
  if(neg) return -val;
  else    return  val;
}


double initial_bearing(double lat_a,double lat_b,double lon_a,double lon_b){
	double delta_lng;
	double lat1=ConvertDegtoRad(lat_a);
	double lat2=ConvertDegtoRad(lat_b);
	double lon1=ConvertDegtoRad(lon_a);
	double lon2=ConvertDegtoRad(lon_b);

	delta_lng = lon2-lon1;
	float y = sin(delta_lng) * cos(lat2);
	float x = cos(lat1)*sin(lat2) -sin(lat1)*cos(lat2)*cos(delta_lng);
	double bearing= ConvertRadtoDeg(atan2(y,x));

	double final;
	if (bearing>0){
		final=bearing;
		}else {
		final=360+bearing;
	}

	return final;
}

double conv_rhumb(double pts[],double pts_p[]){
	float delta_lng,mean_lat;
	delta_lng = pts[1]-pts_p[1];
	mean_lat = (pts[0] + pts_p[0])/2;
	double rad = ConvertDegtoRad((double)mean_lat);
	double con =(double)(delta_lng * sin(rad));
	return con;
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
/*
double cross_track_error(double cur_n,double bear_way, double curr_bear){
	double R= 6371000.0;
	double ang_disc = cur_n / R;
	double delta_bearing =ConvertDegtoRad( curr_bear - bear_way);
	double values= sin(ang_disc)*sin(delta_bearing);
	double cte= asin(values)*R;
	return cte;
}
*/
double cross_track_error(double cur_n,double bear_way, double curr_bear){
	double delta_bearing =ConvertDegtoRad( curr_bear - bear_way);
	double perd = cur_n*sin(delta_bearing);
	return perd;
}


float along_track_error(double curk,double cte_n){
	float alg = sqrt((curk*curk) - (cte_n*cte_n));
	return alg;
}

double ConvertDegtoRad(double degree) {
	double pi = 3.14159265359;
	return (degree * (pi /180));
}


double ConvertRadtoDeg(double radians) {
	double pi = 3.14159265359;
	return (radians * (180 /pi));
}


int pidHead(double val,double setp,double kp,double ki,double kd){

    int steer,start_ht ,final_ht=0;
    struct timeval tim; 
    gettimeofday(&tim, NULL);
    start_ht = tim.tv_usec;
    float preverrorh, error_h=0.0;
    double elapsed_time, head_h=0;
    long int PID_val=0;
    double setpoint_h, integral_h=0.0, derivative_h=0.0;
    setpoint_h=setp;
    
    error_h=(float)(setpoint_h-val);
    
    if (error_h<-180.0){
        error_h = error_h+360.0;
    }
    else if(error_h>180){
        error_h= error_h - 360.0;
    }
    
    elapsed_time=(double)(start_ht-final_ht);
    printf("%f \n",elapsed_time);
    integral_h+=((double)(error_h*elapsed_time));
    derivative_h = ((double)(error_h-preverrorh))/elapsed_time;
    PID_val=(int)(kp*error_h+ki*integral_h+kd*derivative_h);
    preverrorh=error_h;
    final_ht = tim.tv_usec;
    long int dutycycleh=Constrain(PID_val, -4096, 4096);
    int vah = MAP(dutycycleh, -4096,4096, 0,255);
    steer = (int)((1.76*0.0001*vah*vah)-(0.335*vah)+91);
    printf("steer:%d \n",steer);
    return steer;
}



int MAP(int au32_IN, long int au32_INmin, long int au32_INmax, int au32_OUTmin,int au32_OUTmax) {
	return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin))
	/ (au32_INmax - au32_INmin)) + au32_OUTmin);
}

int Constrain(int au32_IN, int au32_MIN, int au32_MAX) {
	if (au32_IN < au32_MIN) {
		return au32_MIN;
		} else if (au32_IN > au32_MAX) {
		return au32_MAX;
		} else {
		return au32_IN;
	}
}
