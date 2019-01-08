#ifndef ODRIVE_H
#define ODRIVE_H

#include <stdio.h>

FILE* odrive_open_port(char *portname, int baudrate);
float odrive_read_float(FILE *fp, const char parameter[]);
int odrive_read_int(FILE *fp, const char parameter[]);
void odrive_write_float(FILE *fp, const char parameter[], const float value);
void odrive_write_int(FILE *fp, const char parameter[], const int value);
void odrive_quick_write(FILE *fp, const char type, const int axis, const float value);
float odrive_vbus_voltage(FILE *fp);
int odrive_check_calibration(FILE *fp, const int axis);

#endif