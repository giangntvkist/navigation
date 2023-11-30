#include <ros/ros.h>
#include "CppLinuxSerial/SerialPort.hpp"
#include <vector>
#include <string>

using namespace mn::CppLinuxSerial;
using namespace std;

#define DEFAULT_BAURATE BaudRate::B_9600
#define DEFAULT_WAIT_MS 10 // with baudrate 115200
#define DEVICE_4 "/dev/RS485_04"

#define ID_START 0xDD
#define ID_STATE_READ 0xA5
#define ID_STATE_WRITE 0x5A
#define ID_STOP 0x77

#define PID_READ_INF_STT 0x03 /* read basic information and status */
#define PID_READ_BAT_CELL_VOL 0x04 /* read battery cell voltage */
#define PID_READ_HW_VER 0x05 /* read the hardware version number of the protection board */

uint16_t Byte2Int(uint8_t byLow, uint8_t byHigh) {
    return (byLow | (uint16_t)byHigh << 8);
}

/* Wait for n ms */
void Waitms(int n) {
    usleep(n*1000);
}

int CheckSum(string& data) {
    uint8_t csum = 0;
    for(uint16_t i = 0; i < data.size(); i++) {
        csum += data[i];
    }
    if(csum == 0) return 0;
    else return -1;
}

class BATTERY {
    private:
        SerialPort port;
        uint16_t voltage; /* unit 10mV */
        int16_t current; /* unit 10mA, if negative, It's discharge */
        uint16_t remain_capacity; /* unit is 10mAH */
        uint16_t nominal_capacity;
        vector<uint8_t> pkg;
        string rev_data;
    public:
        void ReadInfo_Battery(vector<uint8_t>& pkg) {
            pkg.push_back(ID_START);
            pkg.push_back(ID_STATE_READ);
            pkg.push_back(PID_READ_INF_STT);
            pkg.push_back(0x00);
            pkg.push_back(0xFF);
            pkg.push_back(0xFD);
            pkg.push_back(ID_STOP);
        }

        void SendPkg(vector<uint8_t>& pkg) {
            string str(pkg.begin(), pkg.end());
            port.Write(str);
            cout << "Sent package: ";
            for(int i = 0; i < pkg.size(); i++) {
                cout << hex << int(pkg[i]) << " ";
            }
            cout << endl;
        }

        void RevPkg(string& rev_data) {
            rev_data.erase();
            string buffer;
            do {
                Waitms(DEFAULT_WAIT_MS);
                port.Read(buffer);
                rev_data.append(buffer);
            }while(CheckSum(rev_data));
            cout << "BMS respond: ";
            for(int i = 0; i < rev_data.size(); i++) {
                cout << hex << int(rev_data[i]) << " ";
            }
            cout << endl;
        }

        void GetMainData(string& rev_data) {
            string data = rev_data.substr(4);
            voltage = Byte2Int(data[0], data[1]);
            current = Byte2Int(data[2], data[3]);
            remain_capacity = Byte2Int(data[4], data[5]);
            nominal_capacity = Byte2Int(data[6], data[7]);
            cout << dec << "Voltage: " << voltage/100.0 << "V" << "\n" \
                        << "Current: " << current/100.0 << "A" << "\n" \
                        << "Remain capacity: " << remain_capacity/100.0 << "AH" << "\n" \
                        << "Nominal capacity: " << nominal_capacity/100.0 << "AH" << endl;
        }

        BATTERY(ros::NodeHandle* nh) {
            voltage = 0;
            current = 0;
            remain_capacity = 0;
            nominal_capacity = 0;
            port.SetDevice(DEVICE_4);
            port.SetBaudRate(DEFAULT_BAURATE);
            port.SetTimeout(100);
            port.Open();

            ReadInfo_Battery(pkg);
            SendPkg(pkg);
            RevPkg(rev_data);
            GetMainData(rev_data);
        }
        
        ~BATTERY() {
            port.Close();
        }
};

