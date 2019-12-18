#include<stdio.h>
#include<sys/types.h>
#include<stdlib.h>
#include<string>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>

using namespace std;

int main()
{
    int socket_fd = socket(AF_INET, SOCK_STREAM,0);
    if(socket_fd == -1)
    {
        cout<<"socket 创建失败："<<endl;
        exit(-1);
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    addr.sin_addr.s_addr = inet_addr("192.168.103.84");

    int res = bind(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
    if(res == -1)
    {
        cout<<"bind 链接失败："<<endl;
        exit(-1);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(8888);
    addr.sin_addr.s_addr = inet_addr("192.168.103.83");

    res = connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
    if(res == -1)
    {
        cout<<"connect 链接失败："<<endl;
        exit(-1);
    }
    cout<<"connect 链接成功："<<endl;

    write(socket_fd,"hello hebinbing",15);

    close(socket_fd);

    return 0;
}
