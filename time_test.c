
/*

#include <stdio.h>
#include <time.h>
int main() {
    // Get the current time as a time_t value (POSIX time)
    time_t current_time;
    current_time = time(NULL);

    if (current_time == ((time_t)-1)) {
        fprintf(stderr, "Error getting current time.\n");
        return 1;
    }
    
    printf("Current POSIX time: %ld seconds\n", (long)current_time);

    return 0;
}

uint64_t get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000 + (uint64_t)(tv.tv_usec) / 1000;
}



*/

#include <sys/time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
int pidHead(double val,double setp,double kp,double ki,double kd);
int MAP(int au32_IN, long int au32_INmin, long int au32_INmax, int au32_OUTmin,int au32_OUTmax);
int Constrain(int au32_IN, int au32_MIN, int au32_MAX);
int main() {
    while(1){
    pidHead(345.0,0.0,100.0,0.0,0.0);
    }
    return 0;
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





