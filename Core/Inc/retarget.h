#ifndef RETARGET_H_
#define RETARGET_H_

#include <sys/stat.h>
#include <stdio.h>
#include "main.h"

int _write(int fd, char* ptr, int len);
int _read(int fd, char* ptr, int len);

#endif /* RETARGET_H_ */
