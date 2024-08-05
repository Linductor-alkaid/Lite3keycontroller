#include <iostream>
#include <cstring>      // for memset
#include <sys/socket.h> // for socket functions
#include <netinet/in.h> // for sockaddr_in
#include <unistd.h>     // for close
#include <cerrno>       // for errno
#include <cstdio>       // for perror

int main() {
    // 创建UDP套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Error: Could not create socket");
        return 1;
    }

    // 定义监听地址和端口
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(43893);
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有接口

    // 绑定地址和端口到套接字
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Error: Could not bind socket");
        close(sockfd);
        return 1;
    }

    std::cout << "Listening on all interfaces at port 43893\n";

    // 接收数据
    char buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    while (true) {
        ssize_t received = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&client_addr, &client_addr_len);
        if (received > 0) {
            buffer[received] = '\0'; // 确保字符串以空终止
            std::cout << "Received: " << buffer << std::endl;
        } else if (received < 0) {
            perror("Error: recvfrom failed");
        }
    }

    // 关闭套接字
    close(sockfd);
    return 0;
}

