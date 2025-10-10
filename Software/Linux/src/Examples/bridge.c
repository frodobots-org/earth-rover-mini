#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>

#define SERIAL_DEVICE "/dev/ttyS0"
#define TCP_PORT 8888
#define BUF_SIZE 1024

int uart_fd = -1;
int server_fd = -1;
int client_fd = -1;

void cleanup(int signo) {
    if (client_fd > 0) close(client_fd);
    if (server_fd > 0) close(server_fd);
    if (uart_fd > 0) close(uart_fd);
    printf("\n[Bridge] Cleaned up and exiting.\n");
    exit(0);
}

int setup_serial(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Open serial");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = 0;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    printf("[Bridge] Serial %s initialized.\n", device);
    return fd;
}

int setup_server(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket");
        return -1;
    }

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return -1;
    }

    if (listen(fd, 1) < 0) {
        perror("listen");
        close(fd);
        return -1;
    }

    printf("[Bridge] Listening on TCP port %d...\n", port);
    return fd;
}

int main() {
    signal(SIGINT, cleanup);
    signal(SIGTERM, cleanup);

    uart_fd = setup_serial(SERIAL_DEVICE);
    if (uart_fd < 0) return 1;

    server_fd = setup_server(TCP_PORT);
    if (server_fd < 0) return 1;

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    while (1) {
        printf("[Bridge] Waiting for connection...\n");
        client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            perror("accept");
            continue;
        }
        printf("[Bridge] Client connected from %s:%d\n",
               inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        uint8_t buf[BUF_SIZE];
        ssize_t n;
        while ((n = recv(client_fd, buf, sizeof(buf), 0)) > 0) {
            write(uart_fd, buf, n);
            printf("[Bridge] Forwarded %zd bytes to serial.\n", n);
        }

        printf("[Bridge] Client disconnected.\n");
        close(client_fd);
    }

    cleanup(0);
    return 0;
}
