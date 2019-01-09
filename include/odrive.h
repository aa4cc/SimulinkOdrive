#ifndef ODRIVE_H
#define ODRIVE_H

#include <stdio.h>

int odrive_open_port(char *portname, int baudrate);
float odrive_read_float(int fp, const char parameter[]);
int odrive_read_int(int fp, const char parameter[]);
void odrive_write_float(int fp, const char parameter[], const float value);
void odrive_write_int(int fp, const char parameter[], const int value);
void odrive_quick_write(int fp, const char type, const int axis, const float value);
float odrive_vbus_voltage(int fp);
int odrive_check_calibration(int fp, const int axis);
int odrive_wait_for_state(int fp, const int axis, const int requested_state, const int _usleep, int n_timeout);
int odrive_request_state(int fp, const int axis, const int requested_state);

#endif