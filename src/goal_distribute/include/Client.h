#ifndef __CLIENT_H__
#define __CLIENT_H__

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <unistd.h>
#include <stdexcept>

typedef struct 
{
    uint32_t goal;
    uint32_t config;
}
decision_pack;

class Client
{ 
    public:
    Client(int port = 8899);
    ~Client();
    int Read(char* buffer,int size);
    void Send(char* data,size_t size);
    void Close();
    private:
    int sock_fd;
    const int inline_port=8899;
    char *buffer;
    const int max_size = 1024;
    std::string current_recv;
};

#endif