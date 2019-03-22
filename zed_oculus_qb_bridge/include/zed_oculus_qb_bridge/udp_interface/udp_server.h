#ifndef MY_UDP_RECEIVER_H
#define MY_UDP_RECEIVER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdexcept>
#include <string>

class udp_server
{
public:
    udp_server(const std::string& addr, int port);
    ~udp_server();

    int                 get_socket() const;
    int                 get_port() const;
    std::string         get_addr() const;

    int                 recv(char *msg, size_t max_size);
    int                 timed_recv(char *msg, size_t max_size, int max_wait_ms);
    int                 send(const char *msg, size_t max_size);

private:
    int                 f_socket;
    int                 f_port;
    std::string         f_addr;
    struct sockaddr_in si_other;
};

#endif