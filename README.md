# Keyboard Control Program

该程序允许通过键盘输入控制远程服务器。它使用UDP协议向指定的服务器发送简单和复杂的命令。

## 功能

- 检测键盘输入
- 发送简单指令
- 构造并发送复杂指令

## 依赖

确保你的系统已安装以下库：

- `iostream`
- `cstring`
- `sys/socket.h`
- `arpa/inet.h`
- `unistd.h`
- `termios.h`
- `fcntl.h`

## 配置

在使用该程序之前，请确保已设置正确的服务器IP地址和端口号。默认情况下，这些值在代码中定义如下：

```cpp
#define SERVER_IP "192.168.1.120"
#define SERVER_PORT 43893
```

你可以根据实际情况修改这些值。

## 编译

使用以下命令编译程序：

```bash
g++ keyboardcontrol.cpp -o keyboardcontrol
```

## 运行

使用以下命令运行编译后的程序：

```bash
./keyboardcontrol
```

运行后，程序将等待键盘输入，并根据输入发送相应的命令到服务器。

## 文件说明

- `CommandHead` 结构体：定义了命令头部，包括命令代码、参数大小和类型。
- `Command` 结构体：包含命令头部和数据。
- `SimpleCMD` 类：包含简单命令的代码、值和类型。
- `ComplexCMD` 类：继承自 `SimpleCMD`，并添加了一个数据字段。
- `kbhit` 函数：检查是否有键盘输入。
- `send_easy_command` 函数：发送简单指令。
- `send_complex_command` 函数：构造并发送复杂指令。

## 注意事项

- 该程序默认使用UDP协议与服务器进行通信，请确保你的网络配置允许UDP通信。
- 确保服务器IP和端口号正确无误。

## 示例

以下是一个如何使用该程序的示例：

1. 修改代码中的服务器IP和端口号。
2. 编译程序：
    ```bash
    g++ keyboardcontrol.cpp -o keyboardcontrol
    ```
3. 运行程序：
    ```bash
    ./keyboardcontrol
    ```
4. 根据提示进行键盘输入，程序将发送相应的命令到服务器。

如果有任何问题或建议，请联系开发者。

