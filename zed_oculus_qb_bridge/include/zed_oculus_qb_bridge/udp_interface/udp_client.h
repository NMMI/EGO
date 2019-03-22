#ifndef MY_UDP_SENDER_H
#define MY_UDP_SENDER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdexcept>
#include <string>

class udp_client
{
public:
    udp_client(const std::string& address, int port);
    ~udp_client();

    int                 get_socket() const;
    int                 get_port() const;
    std::string         get_addr() const;

    int                 send(const char *msg, size_t size);
    int                 recv(char *msg, size_t size);
private:
    int                 f_socket;
    int                 f_port;
    std::string         f_address;
    struct sockaddr_in si_other;
};

#endif