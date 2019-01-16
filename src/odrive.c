#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "odrive.h"

#include <time.h>

double odrive_time(){
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (double)t.tv_sec + (double)t.tv_nsec / 1000000000.0;
}

void odrive_print_time(double time){
    printf("ODrive took %.1f ms\n", time*1000);
}

void odrive_error(const char *error_message){
    fprintf(stderr, error_message);
    exit(1);
}
        
static int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        exit(1);
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
		exit(1);
	}
	return 0;
}

static void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "Error tcgetattr: %s\n", strerror(errno));
        exit(1);
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0){
		fprintf(stderr, "Error tcsetattr: %s\n", strerror(errno));
        exit(1);
    }
}

int odrive_open_port(char *portname, int baudrate){
	int fd;
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "Error opening %s: %s\n", portname, strerror(errno));
		exit(1);
	}

	int speed = 0;

	switch(baudrate){
		case 115200: speed = B115200; break;
		default:
			fprintf(stderr,"Unknown baudrate %d\n", baudrate);
			exit(1);
	}

	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, speed);
	//set_mincount(fd, 0);                /* set to pure timed read */3

    int f = (int) fdopen(fd, "r+");
#ifdef DEBUG_COMUNICATION
    printf("Port oppened with fp: %d\n", f);
#endif
	return f;
}

float odrive_read_float(int f, const char parameter[]){
	FILE *fp = (FILE *) f;
	float value=-1;
	char line[60];
	if(parameter != NULL){
#ifdef DEBUG_COMUNICATION
		printf("READ: r %s\n", parameter);
#endif
		fprintf(fp, "r %s\n", parameter);
	}
	if(fgets(line, 60, fp) == NULL){
		printf("Reading parameter \"%s\" error, no reply\n", parameter);
		return -1;
	}
	if(sscanf(line, "%f", &value) != 1){
		printf("Reading parameter \"%s\" error, reply: %s\n", parameter, line);
		return -1;
	}
#ifdef DEBUG_COMUNICATION
		printf("REPLY: %f\n", value);
#endif
	return value;
}

int odrive_read_int(int f, const char parameter[]){
	FILE *fp = (FILE *) f;
	int value=-1; 
	char line[60];
	if(parameter != NULL){
#ifdef DEBUG_COMUNICATION
		printf("READ: r %s\n", parameter);
#endif
		fprintf(fp, "r %s\n", parameter);
	}
	if(fgets(line, 60, fp) == NULL){
		printf("Reading parameter \"%s\" error, no reply", parameter);
		return -1;
	}
	if(sscanf(line, "%d", &value) != 1){
		printf("Reading parameter \"%s\" error, reply: %s", parameter, line);
		return -1;
	}
#ifdef DEBUG_COMUNICATION
		printf("REPLY: %d\n", value);
#endif
	return value;
}

void odrive_write_float(int f, const char parameter[], const float value){
	FILE *fp = (FILE *) f;
	fprintf(fp, "w %s %f\n", parameter, value);
#ifdef DEBUG_COMUNICATION
	printf("WRITE:w %s %f\n", parameter, value);
#endif
}

void odrive_write_int(int f, const char parameter[], const int value){
	FILE *fp = (FILE *) f;
	fprintf(fp, "w %s %d\n", parameter, value);
#ifdef DEBUG_COMUNICATION
	printf("WRITE:w %s %d\n", parameter, value);
#endif
}

void odrive_quick_write(int f, const char type, const int axis, const float value){
	FILE *fp = (FILE *) f;
	fprintf(fp, "%c %d %f\n", type, axis, value);
#ifdef DEBUG_COMUNICATION
	printf("WRITE:%c %d %f\n", type, axis, value);
#endif
}

void odrive_quick_write_int(int f, const char type, const int axis, const int value){
	FILE *fp = (FILE *) f;
	fprintf(fp, "%c %d %d\n", type, axis, value);
#ifdef DEBUG_COMUNICATION
	printf("WRITE:%c %d %d\n", type, axis, value);
#endif
}

float odrive_vbus_voltage(int f){
	return odrive_read_float(f, "vbus_voltage");
}

int odrive_wait_for_state(int fp, const int axis, const int requested_state, const int _usleep, int n_timeout){
	char parameter[100];
	sprintf(parameter, "axis%d.current_state", axis);
	int state=0;
	while(1){
		state = odrive_read_int(fp, parameter);
		if(state==requested_state) return 1;
		if(_usleep) usleep(_usleep);
		if(n_timeout == 0) continue;
		n_timeout--;
		if(n_timeout == 0) return 0;
	}

	return 0;
}
int odrive_request_state(int fp, const int axis, const int requested_state){
	char parameter[100];
	sprintf(parameter, "axis%d.requested_state", axis);
	odrive_write_int(fp, parameter, requested_state);
	return 0;
}

#ifdef DEBUG

void printHelp(int argc, char *argv[]){
	printf("Usage: %s port\n", argv[0]);
}

typedef void (*test_function)(int f, int argc, char *argv[]);

void test1(int f, int argc, char *argv[]){
	double start;
    double end;
    double elapsedSeconds;
    int iter = 10;
    char *parameter = "vbus_voltage";
    if(argc>1) iter = atoi(argv[1]);
    if(argc>2) parameter = argv[2];

    printf("Testing %d iterations of reading\n", iter);
    // Timed area, use only code which should be measured
	start = odrive_time();
	for(int i=0; i<iter; i++){
		odrive_read_float(f, parameter);
	}
	end = odrive_time();
	// End of timed area

	elapsedSeconds = end - start;
	printf("Time elapsed: %f ms (per read %f ms)\n",
		elapsedSeconds*1000,
		elapsedSeconds*1000/iter);
}

void test2(int f, int argc, char *argv[]){
	double start;
    double end;
    double elapsedSeconds;
    int iter = 10;
    char *parameter = "vbus_voltage";
    if(argc>1) iter = atoi(argv[1]);
    if(argc>2) parameter = argv[2];

    printf("Testing %d iterations of reading\n", iter);
    // Timed area, use only code which should be measured
	start = odrive_time();

	for(int i=0; i<iter; i++){
		fprintf((FILE *)f, "r %s\n", parameter);
	}

	for(int i=0; i<iter; i++){
		odrive_read_float(f, NULL);
	}
	end = odrive_time();
	// End of timed area

	elapsedSeconds = end - start;
	printf("Time elapsed: %f ms (per read %f ms)\n",
		elapsedSeconds*1000,
		elapsedSeconds*1000/iter);
}

void test3(int f, int argc, char *argv[]){
	double start;
    double end;
    double elapsedSeconds;

    printf("Testing 8 iterations of reading\n");
    // Timed area, use only code which should be measured
	start = odrive_time();

	fprintf((FILE *)f, "r vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\n");

	for(int i=0; i<8; i++){
		odrive_read_float(f, NULL);
	}
	end = odrive_time();
	// End of timed area

	elapsedSeconds = end - start;
	printf("Time elapsed: %f ms (per read %f ms)\n",
		elapsedSeconds*1000,
		elapsedSeconds*1000/8);
}

void test4(int f, int argc, char *argv[]){
	int use_index=1;
	odrive_write_int(f, "axis0.encoder.config.use_index", use_index);

	int motor_is_calibrated = odrive_read_int(f, "axis0.motor.is_calibrated");
	int encoder_is_ready = odrive_read_int(f, "axis0.encoder.is_ready");

	printf("motor_is_calibrated: %s\n", motor_is_calibrated?"True":"False");
	printf("encoder_is_ready: %s\n", encoder_is_ready?"True":"False");

	odrive_write_int(f, "axis0.config.startup_motor_calibration", !motor_is_calibrated);

	if(!encoder_is_ready){
		printf("Trying to encoder get ready\n");
		odrive_write_int(f, "axis0.config.startup_encoder_index_search", 1);
		if(use_index){
			printf("Using index\n");
			if(odrive_read_int(f, "axis0.encoder.config.pre_calibrated")){
				odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 0);
			}else{
				odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 1);
			}
		}else{
			odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 1);
		}
	}else{
		odrive_write_int(f, "axis0.config.startup_encoder_index_search", 0);
		odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 0);
	}

	odrive_request_state(f, /*axis=*/0, 2);
	//usleep(1000000);
	printf("Waiting for get ready\n");
	odrive_wait_for_state(f, 0, 1, 1000000, 0);
}

void ssleep(double t){
	usleep(t*1000000);
}

void test6(int f, int argc, char *argv[]);

void test5(int f, int argc, char *argv[]){
	printf("Reseting errors\n");
    odrive_write_int(f, "axis0.motor.error", 0);
    odrive_write_int(f, "axis0.encoder.error", 0);
    odrive_write_int(f, "axis0.controller.error", 0);
    odrive_write_int(f, "axis0.error", 0);

	test4(f, argc, argv);
	printf("Setting current limit to 100A\n");
	odrive_write_float(f, "axis0.motor.config.current_lim", 100);

	printf("Setting velocity limit\n");
	odrive_write_float(f, "axis0.controller.config.vel_limit", 100000);

	printf("Setting to position mode\n");
	odrive_write_int(f, "axis0.controller.config.control_mode", 3);

	printf("Activating regulator\n");
	odrive_request_state(f, /*axis=*/0, 8);
	test6(f, argc, argv);
	odrive_request_state(f, /*axis=*/0, 1);
}

void test6(int f, int argc, char *argv[]){
	printf("Controlling...\n");
	const double period = 1.0/50.0;
	const double sim_time = 5.0;

    const double start = odrive_time();
    int i = 0;
	while(1){
		double n = odrive_time();
		//if(n >= (start + sim_time)) break;

		//odrive_write_int(f, "axis0.controller.pos_setpoint", i*250);
		odrive_quick_write_int(f, 'p', 0, (((int)(n-start))%2)?0:5000);
		//printf("vbus_voltage: %f V\n", odrive_vbus_voltage(f));
		i++;
		const double t = period - (odrive_time()-n);
		if(t>0){
			printf("Sleeping for %fs\n", t);
			ssleep(t);	
		}
	}
	printf("Controll done\n");
}

void test7(int f, int argc, char *argv[]){
	odrive_write_float(f, argv[1], atof(argv[2]));
}

void test8(int f, int argc, char *argv[]){
	odrive_quick_write_int(f, 'p', 0, atoi(argv[1]));
}



int main(int argc, char *argv[])
{
	test_function tests[] = {test1, test1, test2, test3, test4, test5, test6, test7, test8};
	if(argc < 2){
		printf("Error: Not enough arguments\n");	
		printHelp(argc, argv);
		return -2;
	}
	int f = odrive_open_port(argv[1], 115200);

	if(argc < 3){
		printf("axis0.encoder.pos_estimate: %f\n", odrive_read_float(f, "axis0.encoder.pos_estimate"));
		printf("vbus_voltage: %f V\n", odrive_vbus_voltage(f));
		printf("axis0.current_state: %d\n", odrive_read_int(f, "axis0.current_state"));
		printf("axis0.encoder.is_ready: %s\n", odrive_read_int(f, "axis0.encoder.is_ready")?"True":"False");
	}else{
		int test_id = atoi(argv[2]);
		if(test_id >= (sizeof(tests)/sizeof(test_function))){
			printf("Test %d does not exists\n", test_id);
		}else{
			printf("Running test %d\n", test_id);
			tests[test_id](f, argc-2, argv+2);
		}		
	}
	printf("Waiting for fclose\n");
	fclose((FILE *) f);
	printf("Exiting\n");
}
#endif