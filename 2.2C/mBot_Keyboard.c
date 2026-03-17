#include <stdio.h>
#include <ctype.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

int detect_keystroke(void);
void handle_sigint(int signal);
int open_serial(const char *device, int baud);
void restore_terminal(void);
void send_mBot_signal(char sig);
void set_terminal(void);

int serial_fd;
struct termios oldterm;

int main(){

	set_terminal();

	// open serial to mBot
	serial_fd = open_serial("/dev/ttyUSB0", B115200);

	char prev_signal = '0';
	char curr_signal = '0';

	while(1){
		if (detect_keystroke()){
			int ch = getchar();
			
			if (ch != EOF)
				ch = tolower(ch);
			
			switch(ch){
				case 'w': curr_signal = '1'; break;
				case 's': curr_signal = '2'; break;
				case 'd': curr_signal = '3'; break;
				case 'a': curr_signal = '4'; break;
			}
    	}
		else 
			curr_signal = '0'; 
		
		// only send a signal on the first detection of keystroke
		// saves sending the same signal over and over if the key is being held down
		if (prev_signal != curr_signal){
			send_mBot_signal(curr_signal);
			prev_signal = curr_signal;
		}
	}
	return 0;
}

// on each iteration checks whether a key is pressed, select() will be greater than 0 if true
int detect_keystroke(){
	struct timeval tv = { 0L, 50000 };
	fd_set fds;
	FD_ZERO(&fds);			// clears fds
	FD_SET(0, &fds);		// watches STDIN
	return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

// if ctrl + c is used to exits program (currently only option), this will handle the call
void handle_sigint(int sig){
	restore_terminal();
	exit(0);
}

// configuration for serial communication. Parameters are baud serial rate that mBot is listening to
// and USB port mBot is connected to
int open_serial(const char *device, int baud){
	int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        exit(1);
    }

    fcntl(fd, F_SETFL, 0);		// set file descriptor to blocking mode; read() will wait for data

    struct termios options;
    tcgetattr(fd, &options);
	// set I/O baud rates
	cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

	// Below overrides default terminal behaviour; configures local, control, input & output flags
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);		// apply configuration

    return fd;
}

// handles send signal data to serial port mBot is connected to
void send_mBot_signal(char sig){
    write(serial_fd, &sig, 1);
}

// overrides terminal to non-canonical mode; allows the program to respond
// immediately to keyboard presses
void set_terminal(){
	struct termios newterm;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldterm);		 		// save old terminal settings
	newterm = oldterm;
	newterm.c_lflag &= ~(ICANON | ECHO);			// disable keystroke blocking
	tcsetattr(STDIN_FILENO, TCSANOW, &newterm);
	
	// set non-blocking char to read held keystroke
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);	
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); 
	
	signal(SIGINT, handle_sigint); 					// handles ctrl + c exit
	atexit(restore_terminal);						// restores terminal on program exit
}

// restores terminal to reading from the buffer, also closes serial connection 
void restore_terminal(){
	close(serial_fd);
	tcsetattr(STDIN_FILENO, TCSANOW, &oldterm);
}
