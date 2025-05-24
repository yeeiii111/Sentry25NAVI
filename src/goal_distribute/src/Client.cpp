#include "Client.h"

Client::Client(int port):inline_port(port)
{
    this->sock_fd=socket(AF_INET,SOCK_STREAM,0);
    if(this->sock_fd<0)
    {
        throw std::runtime_error("socket error");
    }
    sockaddr_in server_addr;
    server_addr.sin_family=AF_INET;
    server_addr.sin_port=htons(this->inline_port);
    inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);
    if( connect(this->sock_fd,(struct sockaddr*)&server_addr,sizeof(server_addr))< 0 )
    {
        throw std::runtime_error("connect error");
    }

    this->buffer=new char[this->max_size];
}

Client::~Client()
{
    this->Close();
    delete this->buffer;
}

int  Client::Read(char* buffer,int size)
{
    return read(this->sock_fd,buffer,size);
}

void Client::Send(char* data,size_t size)
{
    send(this->sock_fd,data,size,0);
}

void Client::Close()
{
    close(this->sock_fd);
}