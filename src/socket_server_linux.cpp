//Two-way Linux Server

#include <iostream>
#include <aris.hpp>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#define BUFSIZE 100


void *serverfunction(void* args)
{
    auto&cs = aris::server::ControlServer::instance();


    int serv_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in clnt_addr;
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("192.168.1.110");
    serv_addr.sin_port = htons(5010);
    bind(serv_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));


    listen(serv_sock, 20);



    printf("======waiting for client's request======\n");

     char strBuffer[BUFSIZE]="";
     int clnt_sock = -1;
     socklen_t clnt_addr_size = sizeof(clnt_addr);



         do
         {
              clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_addr, &clnt_addr_size);
              if (clnt_sock < 0)
              {
                  printf("Connected Error, re-try to get connected with client ... \n");
              }
         }
         while(clnt_sock < 0);
    while(1)
    {
    // int dataLength = read(clnt_sock, strBuffer, sizeof(strBuffer)-1);
    memset(strBuffer,0,BUFSIZE);
    int dataLength = read(clnt_sock, strBuffer, 100);
    if (dataLength < 0)
    {
        printf("Read Error ... \n");
        continue;
    }

    std::cout<<"The server has already got the data:"<<strBuffer << std::endl;


    cs.executeCmd(aris::core::Msg(strBuffer));

    //string cmd = strBuffer;
    //cs.executeCmd(aris::core::Msg(cmd.c_str()));

   }
     close(clnt_sock);
     close(serv_sock);
}



void sockerserver()
{

//线程
    pthread_t tids;
    int ret=pthread_create(&tids, NULL, serverfunction, NULL);
    if (ret != 0)
    {
       printf( "pthread_create error: error_code= %i\n", ret);
    }

}

