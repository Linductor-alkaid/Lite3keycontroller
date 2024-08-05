/*
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#define SERVER_IP "192.168.1.120"
#define SERVER_PORT 43893

struct CommandHead {
    uint32_t code;
    uint32_t parameters_size;
    uint32_t type;
};

struct Command {
    CommandHead head;
    double data;
};

class SimpleCMD{
public:
  int32_t cmd_code;
  int32_t cmd_value;
  int32_t type;
};

class ComplexCMD : public SimpleCMD{
public:
  double data;
};

// 检查是否有键盘输入
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

// 发送简单指令的函数
void send_easy_command(int sockfd, struct sockaddr_in server_addr, uint32_t code) {
    CommandHead command;
    command.code = code;
    command.parameters_size = 0;
    command.type = 0;

    if (sendto(sockfd, &command, sizeof(command), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "发送指令失败!" << std::endl;
    } else {
        std::cout << "指令发送成功." << std::endl;
    }
}

// // 构造并发送复杂指令
void send_complex_command(int sockfd, struct sockaddr_in server_addr, uint32_t code, int32_t value){
    int nbytes;
    ComplexCMD complexcmd;
    complexcmd.cmd_code = code;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = value;                  ///< linear x velocity
    nbytes = sendto(sockfd, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr*)&server_addr, sizeof(server_addr));
}

int main() {
    int sockfd;
    struct sockaddr_in server_addr;

    // 创建UDP套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "套接字创建失败!" << std::endl;
        return -1;
    }

    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        std::cerr << "无效的地址或地址不受支持!" << std::endl;
        close(sockfd);
        return -1;
    }

    // 循环检测键盘输入
    while (true) {
        if (kbhit()) {
            char c = getchar();
            if (c == 'z') {
                send_easy_command(sockfd, server_addr, 0x21010202); // 站立模式
            } else if (c == 'y') {
                send_easy_command(sockfd, server_addr, 0x21010D06); // 移动模式
            } else if (c == 'u') {
                send_easy_command(sockfd, server_addr, 0x21010D05); // 原地模式
            } else if (c == 'x') {
                send_easy_command(sockfd, server_addr, 0x21010C03); // 自主模式
            } else if (c == 'c') {
                send_easy_command(sockfd, server_addr, 0x21010C02); // 手动模式
            } else if (c == 'h') {
                send_easy_command(sockfd, server_addr, 0x21010507); // 打招呼
            } else if (c == 'j') {
                send_easy_command(sockfd, server_addr, 0x2101050B); // 跳
            } else if (c == 'w') {
                send_complex_command(sockfd, server_addr, 0x21010130, 0.7); //x方向移动
            } else if (c == 's') {
                send_complex_command(sockfd, server_addr, 0x21010130, -0.7); //x方向移动
            } else if (c == 'a') {
                send_complex_command(sockfd, server_addr, 0x21010131, 25000); //y方向移动
            } else if (c == 'd') {
                send_complex_command(sockfd, server_addr, 0x21010131, -25000); //y方向移动
            } else if (c == 'q') {
                send_complex_command(sockfd, server_addr, 0x21010135, 25000); //w方向旋转
            } else if (c == 'e') {
                send_complex_command(sockfd, server_addr, 0x21010135, -25000); //w方向旋转
            }
        }

        // 可以添加更多逻辑来处理接收到的数据或其他操作
    }

    // 关闭套接字
    close(sockfd);
    return 0;
}
*/

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <chrono>
#include <thread>


#define SERVER_IP "192.168.1.120"
#define SERVER_PORT 43893

struct CommandHead {
    uint32_t code;
    uint32_t parameters_size;
    uint32_t type;
};

struct Command {
    CommandHead head;
    uint32_t data[256];
};

class SimpleCMD {
public:
    int32_t cmd_code;
    int32_t cmd_value;
    int32_t type;
};

class ComplexCMD : public SimpleCMD {
public:
    double data;
};

// 检查是否有键盘输入
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

// 发送简单指令的函数
void send_easy_command(int sockfd, struct sockaddr_in server_addr, uint32_t code) {
    CommandHead command;
    command.code = code;
    command.parameters_size = 0;
    command.type = 0;

    if (sendto(sockfd, &command, sizeof(command), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "发送指令失败!" << std::endl;
    } else {
        std::cout << "指令发送成功." << std::endl;
    }
}

void send_easy1_command(int sockfd, struct sockaddr_in server_addr, uint32_t code) {
    CommandHead command;
    command.code = 0x21010C0A;
    command.parameters_size = code;
    command.type = 0;

    if (sendto(sockfd, &command, sizeof(command), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "发送指令失败!" << std::endl;
    } else {
        std::cout << "指令发送成功." << std::endl;
    }
}

void send_easy2_command(int sockfd, struct sockaddr_in server_addr, uint32_t code) {
    CommandHead command;
    command.code = code;
    command.parameters_size = 0xC0;
    command.type = 0;

    if (sendto(sockfd, &command, sizeof(command), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "发送指令失败!" << std::endl;
    } else {
        std::cout << "指令发送成功." << std::endl;
    }
}

// 发送复杂指令的函数
void send_complex_command(int sockfd, struct sockaddr_in server_addr, uint32_t code, int value) {
    Command command;
    command.head.code = code;
    command.head.parameters_size = value;
    command.head.type = 0;

    // command.data[0] = 0; 
    // command.data[1] = 0;
    // command.data[2] = 0;
    // command.data[3] = 0;
    // command.data[4] = (value >> 24) & 0xff;
    // command.data[5] = (value >> 16) & 0xff;
    // command.data[6] = (value >> 8) & 0xff;
    // command.data[7] = value & 0xff;

    if (sendto(sockfd, &command, sizeof(command)+command.head.parameters_size, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "发送复杂指令失败!" << std::endl;
    } else {
        std::cout << "复杂指令发送成功." << std::endl;
    }
    int nbytes;
    ComplexCMD complexcmd;
    complexcmd.cmd_code = 320;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = 0.7;                  ///< linear x velocity
    nbytes = sendto(sockfd, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&server_addr, sizeof(server_addr));
}

// 定期执行任务的线程函数
void periodic_task(int sockfd, struct sockaddr_in server_addr) {
    while (true) {
        send_easy_command(sockfd, server_addr, 0x21040001);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

int main() {
    int sockfd;
    struct sockaddr_in server_addr;

    // 创建UDP套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "套接字创建失败!" << std::endl;
        return -1;
    }

    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        std::cerr << "无效的地址或地址不受支持!" << std::endl;
        close(sockfd);
        return -1;
    }

    send_easy_command(sockfd, server_addr, 0x2101030D); // 关闭扬声器

        // 启动定期任务线程
    std::thread periodic_thread(periodic_task, sockfd, server_addr);
    periodic_thread.detach(); // 使线程在后台运行

    // 循环检测键盘输入
    while (true) {
        if (kbhit()) {
            char c = getchar();
            if (c == 'z') {
                send_easy_command(sockfd, server_addr, 0x21010202); // 站立模式
            } else if (c == '2') {
                send_easy_command(sockfd, server_addr, 0x21020C0E); // 软急停
            } else if (c == 'y') {
                send_easy_command(sockfd, server_addr, 0x21010D06); // 移动模式
            } else if (c == 'u') {
                send_easy_command(sockfd, server_addr, 0x21010D05); // 原地模式
            } else if (c == 'x') {
                send_easy_command(sockfd, server_addr, 0x21010C03); // 自主模式
            } else if (c == 'c') {
                send_easy_command(sockfd, server_addr, 0x21010C02); // 手动模式
            } else if (c == 'v') {
                send_easy2_command(sockfd, server_addr, 0x21012109); // 跟随模式
            } else if (c == 'h') {
                send_easy_command(sockfd, server_addr, 0x21010507); // 打招呼
            } else if (c == 'j') {
                send_easy_command(sockfd, server_addr, 0x2101050B); // 跳
            } else if (c == '1') {
                send_easy_command(sockfd, server_addr, 0x21010204); // 扭身体
            } else if (c == 'w') {
                send_easy1_command(sockfd, server_addr, 3); //x方向移动
            } else if (c == 's') {
                send_easy1_command(sockfd, server_addr, 4); //x方向移动
            } else if (c == 'a') {
                send_easy1_command(sockfd, server_addr, 5); //y方向移动
            } else if (c == 'd') {
                send_easy1_command(sockfd, server_addr, 6); //y方向移动
            } else if (c == 'q') {
                send_easy1_command(sockfd, server_addr, 13); //w方向旋转
            } else if (c == 'e') {
                send_easy1_command(sockfd, server_addr, 14); //w方向旋转
            } else if (c == 'f') {
                send_easy1_command(sockfd, server_addr, 7); //停止
            }
        }

        // 可以添加更多逻辑来处理接收到的数据或其他操作
    }

    // 关闭套接字
    close(sockfd);
    return 0;
}

