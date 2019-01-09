#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "odrive.h"

//#define DEBUG_COMUNICATION

static int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
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
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

static void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}

int odrive_open_port(char *portname, int baudrate){
	int fd;
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return 0;
	}

	int speed = 0;

	switch(baudrate){
		case 115200: speed = B115200; break;
		default:
			printf("Unknown baudrate %d\n", baudrate);
			return 0;
	}

	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, speed);
	//set_mincount(fd, 0);                /* set to pure timed read */3

	return (int) fdopen(fd, "r+");
}

float odrive_read_float(int f, const char parameter[]){
	FILE *fp = (FILE *) f;
	float value=-1;
	char line[60];
	if(parameter != NULL){
		fprintf(fp, "r %s\n", parameter);
#ifdef DEBUG_COMUNICATION
		printf("WRITE:r %s\n", parameter);
#endif
	}
	if(fgets(line, 60, fp) == NULL){
		printf("Reading parameter \"%s\" error, no reply", parameter);
		return -1;
	}
#ifdef DEBUG_COMUNICATION
		printf("READ:%s ", line);
#endif
	if(sscanf(line, "%f", &value) != 1){
		printf("Reading parameter \"%s\" error, reply: %s", parameter, line);
		return -1;
	}
#ifdef DEBUG_COMUNICATION
		printf("VALUE: %f\n", value);
#endif
	return value;
}

int odrive_read_int(int f, const char parameter[]){
	FILE *fp = (FILE *) f;
	int value=-1; 
	char line[60];
	if(parameter != NULL){
		fprintf(fp, "r %s\n", parameter);
#ifdef DEBUG_COMUNICATION
		printf("WRITE:r %s\n", parameter);
#endif
	}
	if(fgets(line, 60, fp) == NULL){
		printf("Reading parameter \"%s\" error, no reply", parameter);
		return -1;
	}
#ifdef DEBUG_COMUNICATION
		printf("READ:%s", line);
#endif
	if(sscanf(line, "%d", &value) != 1){
		printf("Reading parameter \"%s\" error, reply: %s", parameter, line);
		return -1;
	}
#ifdef DEBUG_COMUNICATION
		printf("VALUE: %d\n", value);
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
	printf("WRITE:%c %d %f\n\n", type, axis, value);
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
#include <time.h>

void printHelp(int argc, char *argv[]){
	printf("Usage: %s port\n", argv[0]);
}

typedef void (*test_function)(int f, int argc, char *argv[]);

static double TimeSpecToSeconds(struct timespec* ts)
{
    return (double)ts->tv_sec + (double)ts->tv_nsec / 1000000000.0;
}

void test1(int f, int argc, char *argv[]){
	struct timespec start;
    struct timespec end;
    double elapsedSeconds;
    int iter = 10;
    char *parameter = "vbus_voltage";
    if(argc>1) iter = atoi(argv[1]);
    if(argc>2) parameter = argv[2];

    printf("Testing %d iterations of reading\n", iter);
    // Timed area, use only code which should be measured
	clock_gettime(CLOCK_MONOTONIC, &start);
	for(int i=0; i<iter; i++){
		odrive_read_float(f, parameter);
	}
	clock_gettime(CLOCK_MONOTONIC, &end);
	// End of timed area

	elapsedSeconds = TimeSpecToSeconds(&end) - TimeSpecToSeconds(&start);
	printf("Time elapsed: %f ms (per read %f ms)\n",
		elapsedSeconds*1000,
		elapsedSeconds*1000/iter);
}

void test2(int f, int argc, char *argv[]){
	struct timespec start;
    struct timespec end;
    double elapsedSeconds;
    int iter = 10;
    char *parameter = "vbus_voltage";
    if(argc>1) iter = atoi(argv[1]);
    if(argc>2) parameter = argv[2];

    printf("Testing %d iterations of reading\n", iter);
    // Timed area, use only code which should be measured
	clock_gettime(CLOCK_MONOTONIC, &start);

	for(int i=0; i<iter; i++){
		fprintf((FILE *)f, "r %s\n", parameter);
	}

	for(int i=0; i<iter; i++){
		odrive_read_float(f, NULL);
	}
	clock_gettime(CLOCK_MONOTONIC, &end);
	// End of timed area

	elapsedSeconds = TimeSpecToSeconds(&end) - TimeSpecToSeconds(&start);
	printf("Time elapsed: %f ms (per read %f ms)\n",
		elapsedSeconds*1000,
		elapsedSeconds*1000/iter);
}

void test3(int f, int argc, char *argv[]){
	struct timespec start;
    struct timespec end;
    double elapsedSeconds;

    printf("Testing 8 iterations of reading\n");
    // Timed area, use only code which should be measured
	clock_gettime(CLOCK_MONOTONIC, &start);

	fprintf((FILE *)f, "r vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\nr vbus_voltage\n");

	for(int i=0; i<8; i++){
		odrive_read_float(f, NULL);
	}
	clock_gettime(CLOCK_MONOTONIC, &end);
	// End of timed area

	elapsedSeconds = TimeSpecToSeconds(&end) - TimeSpecToSeconds(&start);
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

int main(int argc, char *argv[])
{
	test_function tests[] = {test1, test1, test2, test3, test4};
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
	fclose((FILE *) f);
}
#endif