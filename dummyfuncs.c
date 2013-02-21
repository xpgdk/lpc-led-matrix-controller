#include <sys/types.h>
#include <unistd.h>
#include "uart.h"

void _exit(int status) {
	printf("_exit called\r\n");
	status = status;
}

void *_sbrk(int increment) {
	printf("_sbrk called\r\n");
	increment = increment;
	return 0x0;
}

int _kill(pid_t pid, int sig) {
	return 0;
}

pid_t _getpid(void) {
	return 0;
}

ssize_t _write(int fd, const void *buf, size_t count) {
	return 0;
}

int _close(int fd) {
	return 0;
}

int _fstat(int fd, struct stat *buf) {
	return 0;
}

int _isatty(int fd) {
	return 0;
}

off_t _lseek(int fd, off_t offset, int whence) {
	return 0;
}

ssize_t _read(int fd, void *buf, size_t count) {
	return 0;
}

void *malloc(size_t size) {
	return 0x0;
}

void __cxa_pure_virtual() { 
	printf("__cxa_pure_virtual called\r\n");
	while (1); 
}
