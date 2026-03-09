#ifndef WA100_FAST_DRIVER_H
#define WA100_FAST_DRIVER_H

#include <windows.h>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional> // Added for callbacks
#include <cstring>    // for memset

class WA100FastDriver {
public:
    // Define callback types compatible with old SDK
    using ErrorCallback = std::function<void(uint8_t joint_id, uint8_t error_code)>;
    using LeakCallback  = std::function<void(uint8_t leak_status)>;

private:
    HANDLE hSerial;
    std::string port_name;
    uint32_t baud_rate;
    bool connected;

    // Callbacks
    ErrorCallback cb_stall;
    ErrorCallback cb_temp;
    ErrorCallback cb_current;
    ErrorCallback cb_motor;
    LeakCallback  cb_leak;

    // State tracking to prevent callback flood (Rising edge detection)
    uint8_t last_error_flags[7]; // Index 1-6 used
    uint8_t last_leak_status;

public:
    WA100FastDriver() : hSerial(INVALID_HANDLE_VALUE), connected(false), last_leak_status(0) {
        memset(last_error_flags, 0, sizeof(last_error_flags));
    }

    ~WA100FastDriver() {
        Close();
    }

    // Register Callbacks
    void RegisterLockedRotorCallback(ErrorCallback cb) { cb_stall = cb; }
    void RegisterTempCallback(ErrorCallback cb) { cb_temp = cb; }
    void RegisterCurrentCallback(ErrorCallback cb) { cb_current = cb; }
    void RegisterMotorErrorCallback(ErrorCallback cb) { cb_motor = cb; }
    void RegisterLeakCallback(LeakCallback cb) { cb_leak = cb; }

    bool Connect(const std::string& port, uint32_t baud) {
        port_name = port;
        baud_rate = baud;
        
        // Reset error state on new connection
        memset(last_error_flags, 0, sizeof(last_error_flags));
        last_leak_status = 0;
        
        // Windows COM port numbering workaround for COM > 9
        std::string full_port_name = "\\\\.\\" + port;

        hSerial = CreateFileA(full_port_name.c_str(),
                            GENERIC_READ | GENERIC_WRITE,
                            0,
                            0,
                            OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL,
                            0);

        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Win32 Error: Could not open port " << port << std::endl;
            return false;
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Win32 Error: Could not get comm state" << std::endl;
            Close();
            return false;
        }

        dcbSerialParams.BaudRate = baud;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Win32 Error: Could not set comm state" << std::endl;
            Close();
            return false;
        }

        // Set timeouts (aggressive for high freq)
        COMMTIMEOUTS timeouts = {0};
        // Blocking read until something arrives or timeout
        timeouts.ReadIntervalTimeout = 1;        // Max time between arriving chars
        
        // [Fix] Increase timeout to avoid failure on Config commands or system jitter
        // Previous 5ms was too strict for Windows non-RTOS environment
        timeouts.ReadTotalTimeoutConstant = 100;   // Increased to 100ms for safety
        timeouts.ReadTotalTimeoutMultiplier = 0;
        
        timeouts.WriteTotalTimeoutConstant = 100;  // Increased to 100ms
        timeouts.WriteTotalTimeoutMultiplier = 0;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
             std::cerr << "Win32 Error: Could not set timeouts" << std::endl;
             Close();
             return false;
        }
        
        // Clear buffers
        PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

        connected = true;
        return true;
    }

    void Close() {
        if (connected && hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
        connected = false;
    }

    // Enable all joints (Broadcast ID 0)
    bool EnableAll() {
        if (!connected) return false;

        // 1. Prepare Frame for CMD 0x04 (Enable)
        std::vector<uint8_t> tx_buf;
        // Header
        tx_buf.push_back(0x48);
        tx_buf.push_back(0x4D);
        // Length = 1 (Data is just ID)
        tx_buf.push_back(1);
        // Cmd = 0x04
        tx_buf.push_back(0x04);
        // Data = 0 (ID=0 for all)
        tx_buf.push_back(0x00);
        // Checksum = 1 + 4 + 0 = 5
        tx_buf.push_back(0x05);

        // 2. Clear buffers
        PurgeComm(hSerial, PURGE_RXCLEAR);

        // 3. Write
        if (!RawWrite(tx_buf)) return false;

        // 4. Read Response
        // Response is Status Packet (Length=37)
        // Total bytes = 37 + 5 = 42
        std::vector<uint8_t> rx_buf;
        if (!RawRead(42, rx_buf)) return false;

        // Basic validation
        if (rx_buf[0] != 0x48 || rx_buf[1] != 0x4D) return false;
        if (rx_buf[2] != 37) return false;
        // Cmd echo Check (Should be 0x04)
        if (rx_buf[3] != 0x04) return false;

        return true;
    }

    // Protection: Set Current Limit (Cmd 0x07)
    // val: mA (300-1500)
    bool SetCurrentProtection(uint8_t id, uint16_t val) {
        return SendConfigCmd(0x07, id, val);
    }

    // Protection: Set Over Temp Limit (Cmd 0x08)
    // val: 0.1 degree (e.g. 800 => 80.0 C)
    bool SetOverTemp(uint8_t id, uint16_t val) {
        return SendConfigCmd(0x08, id, val);
    }

    // Protection: Set Rise Temp Start (Cmd 0x09)
    // val: 0.1 degree
    bool SetRiseTemp(uint8_t id, uint16_t val) {
        return SendConfigCmd(0x09, id, val);
    }

    // Clear Error (Cmd 0x0A)
    bool ClearError(uint8_t id) {
        return SendStateCmd(0x0A, id);
    }
    
    // Emergency Stop (Cmd 0x03)
    bool EmergencyStop(uint8_t id) {
        return SendStateCmd(0x03, id);
    }

    // Sync write and read response
    // Returns true if success, populates positions (6 elements) and currents (6 elements)
    bool SyncWriteRead(const uint16_t* desired_pos, uint16_t* out_pos, uint16_t* out_Cur) {
        if (!connected) return false;

        // 1. Prepare Frame
        std::vector<uint8_t> tx_buf;
        PreparePacket(desired_pos, tx_buf);

        // 2. Clear Input Buffer (Fundamental for low latency!)
        // This drops any old data sitting in the OS buffer
        PurgeComm(hSerial, PURGE_RXCLEAR);

        // 3. Write
        if (!RawWrite(tx_buf)) return false;

        // 4. Read Response (Header Search + Fixed Length)
        // Response frame len for CMD 0x05 is header(4) + payload(data) + 1 checksum
        // Wait, protocol doc says:
        // Header(2) + Length(1) + Cmd(1) + Data(N) + Csum(1) => 5 + N
        // For 0x05 response: Length=37. Total = 37 + 5 = 42 bytes.
        
        std::vector<uint8_t> rx_buf;
        if (!RawRead(42, rx_buf)) {
            return false;
        }

        // 5. Parse
        return ParseResponse(rx_buf, out_pos, out_Cur);
    }

private:
    bool SendStateCmd(uint8_t cmd, uint8_t id) {
        if (!connected) return false;
        std::vector<uint8_t> tx_buf;
        tx_buf.push_back(0x48); tx_buf.push_back(0x4D);
        tx_buf.push_back(1); // Len
        tx_buf.push_back(cmd);
        tx_buf.push_back(id);
        uint8_t sum = 1 + cmd + id;
        tx_buf.push_back(sum);
        
        PurgeComm(hSerial, PURGE_RXCLEAR);
        if(!RawWrite(tx_buf)) return false;
        
        std::vector<uint8_t> rx_buf;
        // Status packet length 37 means 37+5=42 bytes
        if(!RawRead(42, rx_buf)) return false;
        
        if(rx_buf[0]!=0x48 || rx_buf[1]!=0x4D || rx_buf[3]!=cmd) return false;
        return true;
    }

    bool SendConfigCmd(uint8_t cmd, uint8_t id, uint16_t val) {
        if (!connected) return false;
        std::vector<uint8_t> tx_buf;
        tx_buf.push_back(0x48); tx_buf.push_back(0x4D);
        tx_buf.push_back(3); // Len
        tx_buf.push_back(cmd);
        tx_buf.push_back(id);
        tx_buf.push_back(val & 0xFF);
        tx_buf.push_back((val>>8) & 0xFF);
        
        uint8_t sum = 3 + cmd + id + (val & 0xFF) + ((val>>8) & 0xFF);
        tx_buf.push_back(sum);
        
        PurgeComm(hSerial, PURGE_RXCLEAR);
        if(!RawWrite(tx_buf)) return false;
        
        // Protocol 7.4/7.5: Response is ACK packet
        // Length=1, Cmd=Original Cmd, Data[0]=0x01(Success)/0x00(Fail)
        // Total bytes = 1(Len) + 1(Cmd) + 1(Data) + 1(Checksum) + 2(Header) = 6 bytes
        std::vector<uint8_t> rx_buf;
        if(!RawRead(6, rx_buf)) return false; 
        
        // Validation
        if(rx_buf[0]!=0x48 || rx_buf[1]!=0x4D) return false;
        if(rx_buf[3]!=cmd) return false; // Cmd echo
        if(rx_buf[4]!=0x01) return false; // 0x01 = Success
        
        return true;
    }

    void PreparePacket(const uint16_t* desired_pos, std::vector<uint8_t>& buf) {
        // Construct 0x05 frame
        // Header
        buf.push_back(0x48);
        buf.push_back(0x4D);
        
        // Data Length: 6 * 2 = 12 bytes
        uint8_t len = 12;
        buf.push_back(len);
        
        // Cmd
        uint8_t cmd = 0x05;
        buf.push_back(cmd);
        
        // Data
        uint8_t data_bytes[12];
        for (int i=0; i<6; i++) {
            uint16_t val = desired_pos[i];
            // Clamp and limit to 0-2000
            if(val > 2000) val = 2000; 
            
            data_bytes[i*2] = val & 0xFF;
            data_bytes[i*2+1] = (val >> 8) & 0xFF;
            
            buf.push_back(data_bytes[i*2]);
            buf.push_back(data_bytes[i*2+1]);
        }
        
        // Checksum
        // sum = Length + Cmd + Data...
        uint32_t sum = len + cmd;
        for(int i=0; i<12; i++) sum += data_bytes[i];
        
        buf.push_back(sum & 0xFF);
    }

    bool RawWrite(const std::vector<uint8_t>& data) {
        DWORD bytesWritten;
        if (!WriteFile(hSerial, data.data(), (DWORD)data.size(), &bytesWritten, NULL)) {
            return false;
        }
        return (bytesWritten == (DWORD)data.size());
    }

    bool RawRead(size_t expected_len, std::vector<uint8_t>& out_buf) {
        // Try to read exactly expected_len bytes
        // Since we cleared buffer and configured timeouts, this should be fast
        out_buf.resize(expected_len);
        DWORD bytesRead = 0;
        size_t totalRead = 0;
        uint8_t* ptr = out_buf.data();
        
        // Simple loop to ensure full frame
        while (totalRead < expected_len) {
            if (!ReadFile(hSerial, ptr + totalRead, (DWORD)(expected_len - totalRead), &bytesRead, NULL)) {
                return false;
            }
            if (bytesRead == 0) {
                // Timeout
                return false;
            }
            totalRead += bytesRead;
        }
        
        return true;
    }

    bool ParseResponse(const std::vector<uint8_t>& packet, uint16_t* out_pos, uint16_t* out_cur) {
        // Validate Header
        if (packet[0] != 0x48 || packet[1] != 0x4D) return false;
        
        // Length should be 37
        if (packet[2] != 37) return false;
        
        // Cmd should be 0x05 (or others returning status)
        // Strictly speaking SyncWriteRead sends 0x05, receives 0x05.
        if (packet[3] != 0x05) return false;
        
        // Data payload starts at index 4
        // Format: 6 * {Pos(2), Cur(2), Temp(1), Err(1)} + Leak(1) = 36 + 1 = 37 bytes
        const uint8_t* data = &packet[4];
        
        for (int i=0; i<6; i++) {
            int offset = i * 6;
            // Pos
            uint16_t p = data[offset] | (data[offset+1] << 8);
            if (out_pos) out_pos[i] = p;
            
            // Current
            uint16_t c = data[offset+2] | (data[offset+3] << 8);
            if (out_cur) out_cur[i] = c;

            // Error Checking & Exception Callbacks
            uint8_t err = data[offset+5];
            uint8_t id = i + 1;
            
            // Trigger only on rising edge (when error bit goes from 0 to 1)
            // or just present? Just present is too spammy. Using rising edge.
            // If user clears error, next read will have 0, so state resets.

            // [诊断] 如果状态发生变化（忽略 0x40 随动标志的影响），打印出来
            // 正常情况下 err 应该是 0x40 (随动模式) 或 0x00
            // 真实的错误位是 exclude 0x40 后的值
            uint8_t real_errors = err & (~0x40);
            uint8_t last_real = last_error_flags[id] & (~0x40);

            if (real_errors != last_real) {
                // 状态变了，打印日志
                std::cout << ">>> [Driver Debug] ID " << (int)id << " Status Changed: " 
                          << std::hex << (int)last_error_flags[id] << " -> " << (int)err << std::dec 
                          << " (Errors: " << (int)real_errors << ")" << std::endl;
            }
            
            // Stall (0x01)
            if ((err & 0x01) && !(last_error_flags[id] & 0x01)) {
                if(cb_stall) cb_stall(id, 0x01);
            }
            // OverTemp (0x02)
            if ((err & 0x02) && !(last_error_flags[id] & 0x02)) {
                if(cb_temp) cb_temp(id, 0x02);
            }
            // OverCurrent (0x04)
            if ((err & 0x04) && !(last_error_flags[id] & 0x04)) {
                if(cb_current) cb_current(id, 0x04);
            }
            // MotorFault (0x08)
            if ((err & 0x08) && !(last_error_flags[id] & 0x08)) {
                if(cb_motor) cb_motor(id, 0x08);
            }
            
            last_error_flags[id] = err;
        }

        // Leak Warning (Last Byte of Data payload)
        // Data starts at 4. Size 37. Last byte at 4+36 = 40.
        uint8_t leak = packet[40];
        if (leak != 0 && last_leak_status == 0) {
             // Rising edge of any leak warning
             if(cb_leak) cb_leak(leak);
        }
        last_leak_status = leak;
        
        return true;
    }
};

#endif
