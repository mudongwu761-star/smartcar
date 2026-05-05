/*
 * @Author: Ilikara 3435193369@qq.com
 * @Date: 2025-02-11 15:23:11
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:17:38
 * @FilePath: /2k300_smartcar/wonderEcho_demo/wonderEcho_demo.cpp
 * @Description:
 *
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved.
 */
#include <atomic>
#include <cctype>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

constexpr const char* I2C_DEVICE = "/dev/i2c-2";
constexpr __u8 WONDER_ECHO_ADDR = 0x34;
constexpr __u8 WONDER_ECHO_REG_STATUS = 0x64;
constexpr __u8 WONDER_ECHO_REG_PLAY = 0x6e;

// 自定义固件播报语: "人行道前停车礼让行人"
// 协议表完整帧为 AA 55 FF 11 FB，I2C 播报寄存器只写中间两个字节 FF 11。
constexpr __u8 VOICE_TYPE_BROADCAST = 0xff;
constexpr __u8 VOICE_ID_CROSSWALK_YIELD = 0x11;

std::mutex i2c_mutex; // 互斥锁，用于保护I2C设备访问
std::atomic<bool> running { true }; // 控制线程运行状态

int i2c_read_byte_data(int file, __u8 reg)
{
    union i2c_smbus_data data;
    struct i2c_smbus_ioctl_data args;

    args.read_write = I2C_SMBUS_READ;
    args.command = reg;
    args.size = I2C_SMBUS_BYTE_DATA;
    args.data = &data;

    if (ioctl(file, I2C_SMBUS, &args) == -1) {
        return -1;
    }

    return data.byte & 0xFF;
}

int i2c_write_i2c_block_data(int file, __u8 reg, __u8 length, const __u8* values)
{
    union i2c_smbus_data data;
    struct i2c_smbus_ioctl_data args;

    if (length > I2C_SMBUS_BLOCK_MAX) {
        return -1;
    }

    for (int i = 0; i < length; i++) {
        data.block[i + 1] = values[i];
    }
    data.block[0] = length;

    args.read_write = I2C_SMBUS_WRITE;
    args.command = reg;
    args.size = I2C_SMBUS_I2C_BLOCK_DATA;
    args.data = &data;

    if (ioctl(file, I2C_SMBUS, &args) == -1) {
        return -1;
    }

    return 0;
}

bool play_voice(int file, __u8 type, __u8 id)
{
    if (type != 0x00 && type != 0xff) {
        std::cerr << "First byte must be 0x00 or 0xFF!" << std::endl;
        return false;
    }

    __u8 data[2] = { type, id };
    std::lock_guard<std::mutex> lock(i2c_mutex);
    if (i2c_write_i2c_block_data(file, WONDER_ECHO_REG_PLAY, sizeof(data), data) < 0) {
        std::cerr << "Failed to write voice command to the i2c device: type=0x"
                  << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)type
                  << " id=0x" << std::setw(2) << (int)id << std::dec << std::nouppercase
                  << std::setfill(' ') << std::endl;
        return false;
    }

    std::cout << "Voice command sent: 0x"
              << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)type
              << " 0x" << std::setw(2) << (int)id << std::dec << std::nouppercase
              << std::setfill(' ') << std::endl;
    return true;
}

bool parse_hex_byte(const std::string& text, __u8& value)
{
    if (text.empty()) {
        return false;
    }

    int parsed = 0;
    std::istringstream stream(text);
    stream >> std::hex >> parsed;
    if (stream.fail() || !stream.eof() || parsed < 0x00 || parsed > 0xff) {
        return false;
    }

    value = static_cast<__u8>(parsed);
    return true;
}

std::string to_lower(std::string text)
{
    for (char& ch : text) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

// 轮询线程函数
void poll_i2c_device(int file)
{
    while (running) {
        {
            std::lock_guard<std::mutex> lock(i2c_mutex); // 加锁
            __u8 reg = WONDER_ECHO_REG_STATUS;
            __u8 value = i2c_read_byte_data(file, reg);

            if (value != 0x00) {
                std::cout << "Value at 0x64 is not 0x00: " << std::hex << (int)value << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1Hz轮询
    }
}

// 用户输入线程函数
void handle_user_input(int file)
{
    while (running) {
        std::cout << "Press Enter/custom to play \"人行道前停车礼让行人\", or input two hex bytes (example: ff 11), q to quit: ";

        std::string line;
        if (!std::getline(std::cin, line)) {
            running = false;
            break;
        }

        std::istringstream stream(line);
        std::string first;
        std::string second;
        stream >> first >> second;
        std::string extra;
        stream >> extra;

        std::string command = to_lower(first);
        if (first.empty() || command == "custom" || command == "c") {
            play_voice(file, VOICE_TYPE_BROADCAST, VOICE_ID_CROSSWALK_YIELD);
            continue;
        }

        if (command == "q" || command == "quit" || command == "exit") {
            running = false;
            break;
        }

        __u8 type = 0x00;
        __u8 id = 0x00;
        if (!extra.empty() || second.empty() || !parse_hex_byte(first, type) || !parse_hex_byte(second, id)) {
            std::cerr << "Invalid input. Use Enter/custom, q, or two hex bytes such as: ff 11" << std::endl;
            continue;
        }

        play_voice(file, type, id);
    }
}

int main()
{
    int file;

    // 打开I2C设备
    if ((file = open(I2C_DEVICE, O_RDWR)) < 0) {
        std::cerr << "Failed to open the i2c bus" << std::endl;
        return 1;
    }

    // 设置I2C设备地址
    if (ioctl(file, I2C_SLAVE, WONDER_ECHO_ADDR) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave" << std::endl;
        close(file);
        return 1;
    }

    std::cout << "WonderEcho ready on " << I2C_DEVICE << ", slave address 0x"
              << std::hex << std::uppercase << (int)WONDER_ECHO_ADDR << std::dec
              << std::nouppercase << std::endl;
    std::cout << "Auto testing custom voice: \"人行道前停车礼让行人\" (send FF 11 to register 0x6E)" << std::endl;
    play_voice(file, VOICE_TYPE_BROADCAST, VOICE_ID_CROSSWALK_YIELD);

    // 创建轮询线程和用户输入线程
    std::thread poll_thread(poll_i2c_device, file);
    std::thread input_thread(handle_user_input, file);

    // 等待用户输入线程结束（按Ctrl+C退出）
    input_thread.join();

    // 停止轮询线程
    running = false;
    poll_thread.join();

    // 关闭I2C设备
    close(file);

    return 0;
}
