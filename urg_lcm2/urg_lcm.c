#include "urg_ctrl.h"
#include "delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <lcm/lcm.h>
#include "urg_range_t.h"

#define LinesMax  5

urg_t urg;
long * data;
lcm_t * lcm;
char * CHANNEL = "URG_RANGE";

static void urg_exit(urg_t *urg, const char *message)
{
  printf("%s: %s\n", message, urg_error(urg));
  urg_disconnect(urg);
	exit(1);
}

void cleanup(){
	urg_disconnect(&urg);
	free(data);
	lcm_destroy(lcm);
	printf("\n\nExited..\n");
	getchar();
}

void ex_program(int sig){
	(void) signal(SIGINT,SIG_DFL);
	exit(0);
}

int urg_connect_any(){
        int ret = urg_connect(&urg, "/dev/tty.usbmodemfa141", 115200);
		if (ret >= 0) return ret;
	return -1;

}

int main(int argc, char *argv[])
{
 // const char device[] = "/dev/ttyACM0"; /* For Linux */

  char buffer[LinesMax][UrgLineWidth];
  char *lines[LinesMax];
  int data_max;
  long* data;
  int scan_msec;
  urg_parameter_t parameter;
  int ret;
  int n;
  int i;

  if (argc == 2) {
        CHANNEL = argv[1];
  }

  /* Connection */  
  ret = urg_connect_any();
  if (ret < 0) {
    urg_exit(&urg, "urg_connect()");
	getchar();
    exit(1);
  }

  /* LCM Connection */
  
  lcm = lcm_create(NULL);
  if(!lcm){
     printf("LCM create error\n");
	getchar();
     exit(1);
  }

  /* Get version information */
  for (i = 0; i < LinesMax; ++i) {
    lines[i] = buffer[i];
  }
  ret = urg_versionLines(&urg, lines, LinesMax);
  if (ret < 0) {
    urg_disconnect(&urg);
    exit(1);
  }
  printf("URG Version:\n");
  for (i = 0; i < LinesMax; ++i) {
    printf("%s\n", lines[i]);
  }

  /* Reserve for receive buffer */
  data_max = urg_dataMax(&urg);
  data = (long*)malloc(sizeof(long) * data_max);

  if (data == NULL) {
    urg_exit(&urg, "data buffer allocation");
    exit(1);
  }
  urg_parameters(&urg, &parameter);
  scan_msec = urg_scanMsec(&urg);



  /* Request for MD data */
  urg_setCaptureTimes(&urg, UrgInfinityTimes);

  /* Request for data */
  ret = urg_requestData(&urg, URG_MD, URG_FIRST, URG_LAST);
//  ret = urg_requestData(&urg, URG_MD, parameter.area_min_, parameter.area_max_);
  if (ret < 0) {
    urg_exit(&urg, "urg_requestData()");
  }
  printf("\n");

	atexit(cleanup);
	(void) signal(SIGINT, ex_program);

    int failure_count = 0;
    int reconnect_thresh = 10;
    int epic_fail = 0;
    int max_reconn_attempts = 600;
	
  while(!epic_fail){
    /* Reception */
    n = urg_receiveData(&urg, data, data_max);
    //printf("n = %d\n", n);

    if (n < 0) {
      //urg_exit(&urg, "urg_receiveData()");
	  printf("urg_receiveData(): %s\n", urg_error(&urg));
            failure_count++;
            struct timespec ts = { 0, 300000000 };
            nanosleep(&ts, NULL);

            int reconn_failures = 0;
            while(failure_count > reconnect_thresh) {                
				urg_disconnect(&urg);                    
				printf("Comms failure.  Trying to reconnect...\n");                

                if(urg_connect_any()>=0) {
                    failure_count = 0; 
					ret = urg_requestData(&urg, URG_MD, URG_FIRST, URG_LAST);
                }else{
					printf("urg_Connect(): %s\n", urg_error(&urg));
				}

                // Throttle reconnect attempts
                struct timespec ts = { 0, 500000000 };
                nanosleep(&ts, NULL);

                reconn_failures++;
                if(reconn_failures > max_reconn_attempts) {
                    printf("Exceeded maximum reconnection attempts.\n");
                    exit(1);
                    epic_fail = 1;
                    break;
                }
            }
			
            continue;
    } else if (n > 0) {
		

		int i;
		float ranges[n];
		float range_tmp;
		float max_range	= ((float)parameter.distance_max_)/1000.0;
		float min_range	= ((float)parameter.distance_min_)/1000.0;

		for(i = 0; i < n; i++){
			range_tmp = ((float)data[i])/1000.0f;			   
			if ((range_tmp > min_range)&&(range_tmp < max_range)){
			    ranges[i] = range_tmp;
			}else{
				ranges[i] = -1.0;
			}
		}

        

	   /* Sending data out trought LCM */
		urg_range_t urg_data ;
		
        struct timeval tv;
        gettimeofday (&tv, NULL);
        urg_data.utime = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;

		urg_data.num_ranges   = n;
		urg_data.ranges       = ranges;
		urg_data.rad0		= (2.0 * M_PI) * (-(float)parameter.area_front_ / (float)parameter.area_total_);
		urg_data.radstep	= (2.0 * M_PI) / (float)parameter.area_total_;


		urg_range_t_publish(lcm, CHANNEL, &urg_data);	

		printf("\r t = %llu [usec]",urg_data.utime);
    }
  }

  urg_disconnect(&urg);
  free(data);
  lcm_destroy(lcm);
	getchar();
  exit(0);
}
