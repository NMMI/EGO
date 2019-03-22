#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <udp_interface/udp_server.h>

udp_server::udp_server(const std::string& addr, int port): f_port(port), f_addr(addr)
{
    struct sockaddr_in si_me;

    //create a UDP socket
    if ((f_socket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        std::cout<<"ERROR creating socket"<<std::endl;
        exit(1);
    }
     
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
     
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    memset((char *) &si_other, 0, sizeof(si_other));

    //bind socket to port
    if( bind(f_socket , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        std::cout<<"ERROR binding"<<std::endl;
        exit(1);
    }

    std::cout<<"Bind to: "<<addr<<":"<<port<<std::endl;
}
udp_server::~udp_server()
{
    close(f_socket);
}

int udp_server::get_socket() const
{
    return f_socket;
}

int udp_server::get_port() const
{
    return f_port;
}

std::string udp_server::get_addr() const
{
    return f_addr;
}

int udp_server::recv(char *msg, size_t max_size)
{
    int s_len = sizeof(si_other);
    return recvfrom(f_socket, msg, max_size, 0,   (struct sockaddr *) &si_other, (socklen_t*)(&s_len));
    std::cout<<"Client is: "<<inet_ntoa(si_other.sin_addr)<<":"<<ntohs(si_other.sin_port)<<std::endl;
}

int udp_server::send(const char* msg, size_t max_size)
{
    return sendto(f_socket, msg, max_size, 0, (struct sockaddr *) &si_other, sizeof(si_other));
}