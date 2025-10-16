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
#include <sys/epoll.h>
#include <signal.h>

#define SERIAL_DEVICE "/dev/ttyS0"
#define TCP_PORT 8888
#define BUF_SIZE 1024
#define MAX_EVENTS 2

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

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag &= ~CRTSCTS;


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
    fcntl(uart_fd, F_SETFL, fcntl(uart_fd, F_GETFL, 0) | O_NONBLOCK);

    server_fd = setup_server(TCP_PORT);
    if (server_fd < 0) return 1;

    printf("[Bridge] Waiting for TCP client...\n");

    int epfd = epoll_create1(0);
    if (epfd < 0) {
        perror("epoll_create1");
        cleanup(0);
    }

    struct epoll_event ev, events[MAX_EVENTS];
    uint8_t buf[BUF_SIZE];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    while (1) {
        // (Re)wait for a client connection if none is active
        if (client_fd < 0) {
            client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) {
                perror("accept");
                sleep(1);
                continue;
            }
            printf("[Bridge] Client connected from %s:%d\n",
                   inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

            // Add both FDs to epoll
            ev.events = EPOLLIN;
            ev.data.fd = client_fd;
            epoll_ctl(epfd, EPOLL_CTL_ADD, client_fd, &ev);

            ev.events = EPOLLIN;
            ev.data.fd = uart_fd;
            epoll_ctl(epfd, EPOLL_CTL_ADD, uart_fd, &ev);
        }

        int n_events = epoll_wait(epfd, events, MAX_EVENTS, -1);
        if (n_events < 0) {
            if (errno == EINTR) continue;
            perror("epoll_wait");
            break;
        }

        for (int i = 0; i < n_events; i++) {
            int fd = events[i].data.fd;

            // Handle disconnects or errors
            if (events[i].events & (EPOLLHUP | EPOLLERR)) {
                if (fd == client_fd) {
                    printf("[Bridge] Client disconnected (fd=%d).\n", fd);
                    epoll_ctl(epfd, EPOLL_CTL_DEL, client_fd, NULL);
                    close(client_fd);
                    client_fd = -1;
                    printf("[Bridge] Waiting for next client...\n");
                } else {
                    printf("[Bridge] EPOLL error on fd %d\n", fd);
                }
                continue;
            }

            // Client → UART
            if (fd == client_fd) {
                ssize_t n = recv(client_fd, buf, sizeof(buf), 0);
                if (n <= 0) {
                    printf("[Bridge] Client closed connection.\n");
                    epoll_ctl(epfd, EPOLL_CTL_DEL, client_fd, NULL);
                    close(client_fd);
                    client_fd = -1;
                    printf("[Bridge] Ready for next client.\n");
                    continue;
                }
                write(uart_fd, buf, n);
                // Optional debug line:
                // printf("[Bridge] TCP → UART %zd bytes\n", n);
            }

            // UART → Client
            else if (fd == uart_fd && client_fd > 0) {
                ssize_t m = read(uart_fd, buf, sizeof(buf));
                if (m > 0) {
                    send(client_fd, buf, m, 0);
                    // Optional debug line:
                    // printf("[Bridge] UART → TCP %zd bytes\n", m);
                }
            }
        }
    }

    close(epfd);
    cleanup(0);
    return 0;
}

// Previous version without persistent client handling
// int main() {
//     signal(SIGINT, cleanup);
//     signal(SIGTERM, cleanup);

//     uart_fd = setup_serial(SERIAL_DEVICE);
//     if (uart_fd < 0) return 1;

//     int uart_flags = fcntl(uart_fd, F_GETFL, 0);
//     fcntl(uart_fd, F_SETFL, uart_flags | O_NONBLOCK);

//     server_fd = setup_server(TCP_PORT);
//     if (server_fd < 0) return 1;

//     // int server_flags = fcntl(server_fd, F_GETFL, 0);
//     // fcntl(server_fd, F_SETFL, server_flags | O_NONBLOCK);


//     struct sockaddr_in client_addr;
//     socklen_t client_len = sizeof(client_addr);

//     printf("[Bridge] Waiting for TCP client...\n");
//     client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
//     if (client_fd < 0) {
//         perror("accept");
//         cleanup(0);
//     }

//     printf("[Bridge] Client connected from %s:%d\n",
//            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

//     // Create epoll instance
//     int epfd = epoll_create1(0);
//     if (epfd < 0) {
//         perror("epoll_create1");
//         cleanup(0);
//     }

//     struct epoll_event ev, events[MAX_EVENTS];
//     ev.events = EPOLLIN;
//     ev.data.fd = client_fd;
//     epoll_ctl(epfd, EPOLL_CTL_ADD, client_fd, &ev);

//     ev.events = EPOLLIN;
//     ev.data.fd = uart_fd;
//     epoll_ctl(epfd, EPOLL_CTL_ADD, uart_fd, &ev);

//     uint8_t buf[BUF_SIZE];
//     while (1) {
//         int n_events = epoll_wait(epfd, events, MAX_EVENTS, -1);
//         if (n_events < 0) {
//             if (errno == EINTR) continue;
//             perror("epoll_wait");
//             break;
//         }

//         for (int i = 0; i < n_events; i++) {
//             int fd = events[i].data.fd;

//             if (events[i].events & (EPOLLHUP | EPOLLERR)) {
//                 printf("[Bridge] EPOLL error or hangup on fd %d\n", fd);
//                 goto cleanup_exit;
//             }

//             if (fd == client_fd) {
//                 ssize_t n = recv(client_fd, buf, sizeof(buf), 0);
//                 if (n <= 0) {
//                     printf("[Bridge] Client disconnected.\n");
//                     goto cleanup_exit;
//                 }
//                 write(uart_fd, buf, n);
//                 printf("[Bridge] TCP → UART %zd bytes\n", n);
//             }

//             else if (fd == uart_fd) {
//                 ssize_t m = read(uart_fd, buf, sizeof(buf));
//                 if (m > 0) {
//                     send(client_fd, buf, m, 0);
//                     printf("[Bridge] UART → TCP %zd bytes\n", m);
//                 }
//             }
//         }
//     }

//     cleanup_exit:
//         close(epfd);
//         if (client_fd > 0) close(client_fd);
//         if (server_fd > 0) close(server_fd);
//         if (uart_fd > 0) close(uart_fd);
//         printf("[Bridge] Clean exit.\n");
//         return 0;

// }

