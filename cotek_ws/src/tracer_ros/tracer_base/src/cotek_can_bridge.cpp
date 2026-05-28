#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>

#include "ICANCmd.h"

int main() {
    // === 打开 SDK 设备 ===
    DWORD devHandle = CAN_DeviceOpen(USBCAN_E_1CH, 0, NULL);
    DWORD channel = 0;

    if (devHandle == 0) {
        std::cerr << "[Bridge] CAN_DeviceOpen failed! Check hardware connection." << std::endl;
        return -1;
    }

    CAN_InitConfig cfg{};
    cfg.bMode = 0;
    cfg.nBtrType = 1;
    cfg.dwBtr[0] = 0x00;
    cfg.dwBtr[1] = 0x1C;           // 500kbps
    cfg.dwAccCode = 0;
    cfg.dwAccMask = 0xFFFFFFFF;
    cfg.nFilter = 0;
    cfg.dwReserved = 0;

    if (CAN_ChannelStart(devHandle, channel, &cfg) != CAN_RESULT_OK) {
        std::cerr << "[Bridge] CAN_ChannelStart failed!" << std::endl;
        CAN_DeviceClose(devHandle);
        return -1;
    }

    std::cout << "[Bridge] SDK device opened successfully." << std::endl;

    // === 打开 vcan0 SocketCAN 接口 ===
    int vsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (vsock < 0) {
        perror("socket");
        return -1;
    }

    int loopback = 1;
    setsockopt(vsock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback));
    setsockopt(vsock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "vcan0");
    if (ioctl(vsock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(vsock);
        return -1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(vsock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(vsock);
        return -1;
    }

    std::cout << "[Bridge] Connected to vcan0 SocketCAN interface." << std::endl;

    // === 发送模式帧 0x231：进入命令模式 ===
    CAN_DataFrame mode_frame{};
    mode_frame.uID = 0x231;
    mode_frame.nDataLen = 8;
    memset(mode_frame.arryData, 0, 8);
    mode_frame.arryData[0] = 0x01;   // enable command mode
    ULONG ret = CAN_ChannelSend(devHandle, channel, &mode_frame, 1);
    if (ret == 0)
        std::cerr << "[Bridge] Failed to send command-mode enable frame (0x231)" << std::endl;
    else
        std::cout << "[Bridge] Sent enable-command-mode frame (0x231)" << std::endl;

    // === 主循环：SDK ↔ vcan0 双向转发 ===
    std::cout << "[Bridge] SDK <-> vcan0 双向桥接启动成功。" << std::endl;

    while (true) {
        // ① SDK → vcan0
        CAN_DataFrame rxFrames[50];
        struct can_frame canMsg{};
        ULONG n = CAN_ChannelReceive(devHandle, channel, rxFrames, 50, 1);
        for (ULONG i = 0; i < n; ++i) {
            memset(&canMsg, 0, sizeof(canMsg));
            canMsg.can_id  = rxFrames[i].uID;
            canMsg.can_dlc = rxFrames[i].nDataLen;
            memcpy(canMsg.data, rxFrames[i].arryData, rxFrames[i].nDataLen);
            ssize_t w = write(vsock, &canMsg, sizeof(canMsg));
            if (w != sizeof(canMsg)) perror("[Bridge] write(vcan0) failed");
        }

        // ② vcan0 → SDK
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(vsock, &readfds);
        struct timeval timeout = {0, 10000}; // 10ms
        int sel = select(vsock + 1, &readfds, NULL, NULL, &timeout);
        if (sel > 0 && FD_ISSET(vsock, &readfds)) {
            struct can_frame recvMsg{};
            int bytes = read(vsock, &recvMsg, sizeof(recvMsg));
            if (bytes > 0) {
                CAN_DataFrame tx{};
                memset(&tx, 0, sizeof(tx));
                tx.uID = recvMsg.can_id & 0x7FF;
                tx.nDataLen = recvMsg.can_dlc;
                memcpy(tx.arryData, recvMsg.data, recvMsg.can_dlc);
                int send_ret = CAN_ChannelSend(devHandle, channel, &tx, 1);
                
            }
        }

        usleep(1000);  // 防止CPU占用过高
    }

    // === 退出清理 ===
    CAN_ChannelStop(devHandle, channel);
    CAN_DeviceClose(devHandle);
    close(vsock);

    return 0;
}