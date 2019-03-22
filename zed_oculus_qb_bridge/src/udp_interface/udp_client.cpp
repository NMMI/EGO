#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <udp_interface/udp_client.h>

udp_client::udp_client(const std::string& address, int port): f_port(port), f_address(address)
{ 
    if ( (f_socket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        std::cout<<"ERROR creating socket"<<std::endl;
        exit(1);
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(port);
     
    if (inet_aton(address.c_str() , &si_other.sin_addr) == 0) 
    {
        std::cout<<"ERROR inet_aton"<<std::endl;
        exit(1);
    }

    std::cout<<"Server is: "<<address<<":"<<port<<std::endl;
}

int udp_client::get_socket() const
{
    return f_socket;
}

int udp_client::get_port() const
{
    return f_port;
}

std::string udp_client::get_addr() const
{
    return f_address;
}

int udp_client::recv(char *msg, size_t size)
{
    int s_len = sizeof(si_other);
    return recvfrom(f_socket, msg, size, 0,  (struct sockaddr *) &si_other, (socklen_t*)(&s_len));
}

int udp_client::send(const char *msg, size_t size)
{
    return sendto(f_socket, msg, size, 0, (struct sockaddr *) &si_other, sizeof(si_other));
}

udp_client::~udp_client()
{
    close(f_socket);
}