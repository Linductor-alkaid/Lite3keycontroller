#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

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

int main() {
    int sockfd;
    struct sockaddr_in server_addr;

    // 创建UDP套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        return -1;
    }

    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported!" << std::endl;
        close(sockfd);
        return -1;
    }

        // 构造并发送简单指令
    CommandHead simple_command1;
    simple_command1.code = 0x21010202;  // 站立模式
    simple_command1.parameters_size = 0;
    simple_command1.type = 0;

    if (sendto(sockfd, &simple_command1, sizeof(simple_command1), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to send stand command!" << std::endl;
    } else {
        std::cout << "Stand command sent successfully." << std::endl;
    }

    // 构造并发送简单指令
    CommandHead simple_command2;
    simple_command2.code = 0x21010D06;  // 移动模式
    simple_command2.parameters_size = 0;
    simple_command2.type = 0;

    if (sendto(sockfd, &simple_command2, sizeof(simple_command2), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to send move command!" << std::endl;
    } else {
        std::cout << "Move command sent successfully." << std::endl;
    }

    // 构造并发送简单指令
    CommandHead simple_command3;
    simple_command3.code = 0x21010C03;  // 自主模式
    simple_command3.parameters_size = 0;
    simple_command3.type = 0;

    if (sendto(sockfd, &simple_command3, sizeof(simple_command3), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to send auto command!" << std::endl;
    } else {
        std::cout << "Auto command sent successfully." << std::endl;
    }

    // // 构造并发送复杂指令
    // Command complex_command;
    // complex_command.head.code = 52;
    // complex_command.head.parameters_size = sizeof(complex_command.data);
    // complex_command.head.type = 1;

    // for (int i = 0; i < 256; ++i) {
    //     complex_command.data[i] = i;  // 填充数据示例
    // }

    // if (sendto(sockfd, &complex_command, sizeof(complex_command.head) + complex_command.head.parameters_size, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    //     std::cerr << "Failed to send complex command!" << std::endl;
    // } else {
    //     std::cout << "Complex command sent successfully." << std::endl;
    // }

    // 接收响应
    char buffer[1024];
    while(true){
      socklen_t addr_len = sizeof(server_addr);
      int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&server_addr, &addr_len);
      if (n < 0) {
          std::cerr << "Failed to receive data!" << std::endl;
      } else {
          std::cout << "Received response: " << buffer << std::endl;
      }
    }
    // 关闭套接字
    close(sockfd);
    return 0;
}
