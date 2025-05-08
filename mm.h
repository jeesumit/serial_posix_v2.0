#ifndef MM_H
#define MM_H
double update_filter(double measurement, double predict, double gain);
float ratof(char *arr);
double initial_bearing(double lat_a,double lat_b,double lon_a,double lon_b);
double conv_rhumb(double pts[],double pts_p[]);
double distance(double lat1,double lat2,double lon1,double lon2);
double cross_track_error(double cur_n,double bear_way, double curr_bear);
float along_track_error(double curk,double cte_n);
double ConvertDegtoRad(double degree);
double ConvertRadtoDeg(double radians);
int pidHead(double val,double spoint,double kp,double ki,double kd);
int MAP(int au32_IN, long int au32_INmin, long int au32_INmax, int au32_OUTmin,int au32_OUTmax);
int Constrain(int au32_IN, int au32_MIN, int au32_MAX);
#endif
