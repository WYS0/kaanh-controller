#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>



#ifdef WIN32
       #include<winsock2.h>
#endif
#ifdef UNIX
       #include"socket_server_linux.h"
#endif

#pragma comment(lib,"ws2_32.lib")
void initialization();
using namespace std;


auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";



int main(int argc, char *argv[])
{
	std::cout << "new" << std::endl;
	xmlpath = xmlpath / xmlfile;
	uixmlpath = uixmlpath / uixmlfile;
	std::cout << xmlpath << std::endl;
    auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);

	//生成kaanh.xml文档

	////-------for rokae robot begin//
	//cs.resetController(kaanh::createControllerRokaeXB4().release());
	//cs.resetModel(kaanh::createModelRokae().release());
	//cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	//cs.resetSensorRoot(new aris::sensor::SensorRoot);
 //   cs.saveXmlFile(xmlpath.string().c_str());
	////-------for rokae robot end//


    cs.loadXmlFile(xmlpath.string().c_str());
    cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
    cs.saveXmlFile(xmlpath.string().c_str());
    cs.start();

	//Start Web Socket//
    //cs.startWebSock("5866");
	//cs.startTcpSock("5867");



	//接收数据
	//测试开线程
    #ifdef WIN32
	thread socketwhile(thread001);
	socketwhile.detach();
	
    #endif

    #ifdef UNIX
    //sockerserver();
    #endif
	//system("en");


	//Receive Command//
	cs.runCmdLine();

	return 0;
}




#ifdef WIN32
       #include<winsock2.h>

void thread001()
{

        //定义长度变量
        int send_len = 0;
        int recv_len = 0;
        int len = 0;
        //定义发送缓冲区和接受缓冲区
        char send_buf[100];
        char recv_buf[100];
        //定义服务端套接字，接受请求套接字
        long s_server;
        long s_accept;
        //服务端地址客户端地址
        SOCKADDR_IN server_addr;
        SOCKADDR_IN accept_addr;
        initialization();
        //填充服务端信息
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
        //server_addr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(5010);
        //创建套接字
        s_server = socket(AF_INET, SOCK_STREAM, 0);
        _WINSOCK2API_::bind(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR));
        //设置套接字为监听状态
        if (listen(s_server, SOMAXCONN) < 0) {
            cout << "设置监听状态失败！" << endl;
            WSACleanup();
        }
        else {
            //cout << "设置监听状态成功！" << endl;
        }
        cout << "服务端正在监听连接，请稍候...." << endl;
        //接受连接请求
        len = sizeof(SOCKADDR);
        s_accept = accept(s_server, (SOCKADDR *)&accept_addr, &len);
        if (s_accept == SOCKET_ERROR) {
            cout << "连接失败！" << endl;
            WSACleanup();

        }
        cout << "连接建立，准备接受数据" << endl;

        while (1)
        {
        recv_len = recv(s_accept, recv_buf, 100, 0);
        if (recv_len < 0) {
            cout << "接受失败！" << endl;
            break;
        }
        else {
            cout << "客户端信息:" << recv_buf << endl;
            //int system(char *command)：将MSDOS命令command传递给DOS执行。 system(recv_buf);
            std::string cmd = recv_buf;

            cs.executeCmd(aris::core::Msg(cmd.c_str()));

            //break;

        }
        //cout << "请输入回复信息:";
        //cin >> send_buf;
        //send_len = send(s_accept, send_buf, 100, 0);
        //if (send_len < 0) {
        //	cout << "发送失败！" << endl;
        //	break;
        //}
    }
    //关闭套接字
    close(s_server);
    close(s_accept);
    //释放DLL资源
    WSACleanup();

    void initialization() {
        //初始化套接字库
        WORD w_req = MAKEWORD(2, 2);//版本号
        WSADATA wsadata;
        int err;
        err = WSAStartup(w_req, &wsadata);
        if (err != 0) {
            cout << "初始化套接字库失败！" << endl;
        }
        else {
            //cout << "初始化套接字库成功！" << endl;
        }
        //检测版本号
        if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
            cout << "套接字库版本号不符！" << endl;
            WSACleanup();
        }
        else {
            //cout << "套接字库版本正确！" << endl;
        }
        //填充服务端地址信息

    }


}


#endif
