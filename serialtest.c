/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <asm/termbits.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

static unsigned long long num_read = 0;
static unsigned long long num_written = 0;

int run_test(int fd)
{
    nfds_t nfds = 2;
    struct pollfd *pfds;
    uint8_t counter = 0;
    int ret;
    uint8_t rd;
    int prev_rd = -1;
    uint8_t stdin_rd = 0;
    int rc = 0;

    pfds = calloc(nfds, sizeof(struct pollfd));
    if (pfds == NULL) {
        perror("malloc");
        return 1;
    }

    pfds[0].fd = STDIN_FILENO;
    pfds[0].events = POLLIN | POLLHUP;
    pfds[1].fd = fd;
    pfds[1].events = POLLIN;

    ret = poll(pfds, nfds, -1);
    if (ret < 0) {
        rc = 1;
        goto test_exit;
    }
    printf("Press q or ^D to exit\n");

    pfds[1].events |= POLLOUT;

    while (1) {
        ret = poll(pfds, nfds, -1);
        if (ret < 0) {
            rc = 1;
            goto test_exit;
        }
        if ((pfds[0].revents & POLLIN)) {
            //printf("reading stdin\n");
            ret = read(STDIN_FILENO, &stdin_rd, 1);
            //printf("read %x, ret = %d\n", stdin_rd, ret);
            if (stdin_rd == 'q' || ret == 0 || stdin_rd == 4) {
                printf("Quitting...\n");
                break;
            }
        }
        if (pfds[0].revents & POLLHUP) {
            break;
        }
        if (pfds[1].revents & POLLIN) {
            //printf("reading serial\n");
            if (read(fd, &rd, 1) > 0) {
                num_read++;
                //printf("<");
                if (prev_rd != -1) {
                    if ((prev_rd + 1) % 256 != rd) {
                        rc = 2;
                        goto test_exit;
                    }
                }
                prev_rd = rd;
            }
        }
        if (pfds[1].revents & POLLOUT) {
            //printf("writing serial\n");
            ret = write(fd, &counter, 1);
            if (ret == 1) {
                //printf(">");
                //fflush(stdout);
                counter++;
                num_written++;
            } else if (ret < 0 && errno != EAGAIN) {
                perror("write");
                rc = 3;
                goto test_exit;
            } else {
                //usleep(1);
            }
        }
    }

test_exit:
    free(pfds);
    return rc;
}

void reset_input_mode(int status, void * tio)
{
#if defined TCSETS2
    ioctl(STDIN_FILENO, TCSETS2, tio);
#else
    ioctl(STDIN_FILENO, TCSETS, tio);
#endif
}

int main(int argc, char *argv[])
{
#if !defined BOTHER
    fprintf(stderr, "BOTHER is unsupported\n");
    /* Program may fallback to TCGETS/TCSETS with Bnnn constants */
    exit(EXIT_FAILURE);
#else
    /* Declare tio structure, its type depends on supported ioctl */
# if defined TCGETS2
    struct termios2 tio;
    struct termios2 old_stdin_tio;
# else
    struct termios tio;
    struct termios old_stdin_tio;
# endif
    int fd, rc;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s DEVICE BAUDRATE\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    fd = open(argv[1], O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    /* Get the current serial port settings via supported ioctl */
# if defined TCGETS2
    rc = ioctl(fd, TCGETS2, &tio);
# else
    rc = ioctl(fd, TCGETS, &tio);
# endif
    if (rc) {
        perror("TCGETS");
        goto err;
    }

    tio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK |
                     ISTRIP | IXON);
    tio.c_oflag = 0;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    tio.c_cflag &= ~(CSIZE | PARENB);
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;

    /* Clear the current output baud rate and fill a new value */
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ospeed = atoi(argv[2]);

    /* Clear the current input baud rate and fill a new value */
    tio.c_cflag &= ~(CBAUD << IBSHIFT);
    tio.c_cflag |= BOTHER << IBSHIFT;
    tio.c_ispeed = atoi(argv[2]);

    /* Set new serial port settings via supported ioctl */
# if defined TCSETS2
    rc = ioctl(fd, TCSETS2, &tio);
# else
    rc = ioctl(fd, TCSETS, &tio);
# endif
    if (rc) {
        perror("TCSETS");
        goto err;
    }

    /* And get new values which were really configured */
# if defined TCGETS2
    rc = ioctl(fd, TCGETS2, &tio);
# else
    rc = ioctl(fd, TCGETS, &tio);
# endif
    if (rc) {
        perror("TCGETS");
        goto err;
    }

    printf("output baud rate: %u\n", tio.c_ospeed);
    printf("input baud rate: %u\n", tio.c_ispeed);
    printf("Press any key or receive data to start sending\n");

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
# if defined TCGETS2
    rc = ioctl(STDIN_FILENO, TCGETS2, &tio);
    rc = ioctl(STDIN_FILENO, TCGETS2, &old_stdin_tio);
# else
    rc = ioctl(STDIN_FILENO, TCGETS, &tio);
    rc = ioctl(STDIN_FILENO, TCGETS, &old_stdin_tio);
# endif
    if (rc) {
        perror("TCGETS");
        goto err;
    }

    tio.c_lflag &= ~(ICANON|ECHO); /* Clear ICANON and ECHO. */
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;

# if defined TCSETS2
    rc = ioctl(STDIN_FILENO, TCSETS2, &tio);
# else
    rc = ioctl(STDIN_FILENO, TCSETS, &tio);
# endif
    on_exit(reset_input_mode, &old_stdin_tio);
    if (rc) {
        perror("TCSETS");
        goto err;
    }

    rc = run_test(fd);
    if (rc != 0) {
        if (rc == 2)
            printf("Error found in data stream\n");
        goto err;
    }

    printf("\n");
    printf("No errors encountered, read %llu bytes, wrote %llu bytes\n",
           num_read, num_written);

    close(fd);
    exit(EXIT_SUCCESS);

err:
    close(fd);
    exit(EXIT_FAILURE);
#endif
}
