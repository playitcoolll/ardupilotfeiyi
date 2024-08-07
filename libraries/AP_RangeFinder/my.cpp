#include "my.h"


#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_InternalError/AP_InternalError.h>

#include <AP_HAL/AP_HAL.h>
#include <cctype>
#include <stdio.h>

uint8_t reset_flag(void);
float hexToFloat(uint32_t hexValue);

extern const AP_HAL::HAL &hal;

uint8_t  packstart1 = 0xEB;//包头
uint8_t  packstart2 = 0x90;
int  flag = 0;//接收完成标志
uint8_t  index_aoa = 0;//接收字节索引
uint8_t  index_ssa = 0;
uint8_t  index_buf1 = 0;
uint8_t aoa_buf[4];
uint8_t ssa_buf[4];
uint8_t buf1[24];
float floatValue_aoa = 0;
float floatValue_ssa = 0;
float last_aoa = 0;
float last_ssa = 0;
uint32_t _last_update_ms = 0;


aoassa::aoassa(): _aoa(),
                  _ssa(),
                  _last_update_ms(0)
{
    
    _singleton = this;
}


static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        return;
    }
    uart->begin(115200,148,148);
}

// static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
// {
//     if (uart == nullptr) {
//         return;
//     }

//     static uint8_t state = 0;
//     static uint8_t sum = 0;

    
//     int16_t nbytes = uart ->available();
//     hal.console->printf("Number of bytes available: %d\n",nbytes);
//     while (nbytes--) { //??????????????

       
//         uint8_t c;
//         ssize_t bytesRead = uart->read(c);
        
//         if(bytesRead == 1){
//             reset_flag();
//             switch(state){
//                 case 0:
//                     if(c == packstart1){
//                         sum += c;
//                         state = 1;
//                     }else{
//                         sum = 0;
//                         state = 0;
//                     }
//                     break;
//                 case 1:
//                     if(c == packstart2){
//                         sum += c;
//                         state = 2;
//                     }else{
//                         sum = 0;
//                         state = 0;
//                     }
//                     break;
//                 case 2:
//                     if(c == 0x25){
//                         sum += c;
//                         state = 3;
//                     }else{
//                         sum = 0;
//                         state = 0;
//                     }
//                     break;
//                 case 3:
//                     sum += c;
//                     state = 4;
//                     index_aoa = 0;
//                     index_ssa = 0;
//                     index_buf1 = 0;
//                     break;
//                 case 4:
//                     sum += c;
//                     aoa_buf[index_aoa] = c;
//                     index_aoa++;
//                     if(index_aoa >= 4){
//                         state = 5;
//                     }
//                     break;
//                 case 5:
//                     sum += c;
//                     ssa_buf[index_ssa] = c;
//                     index_ssa++;
//                     if(index_ssa >= 4){
//                         state = 6;
//                     }
//                     break;
//                 case 6:
//                     sum += c;
//                     buf1[index_buf1] = c;
//                     index_buf1++;
//                     if(index_buf1 >= 24){
//                         state = 7;
//                     }
//                     break;
//                 case 7:
//                     if(sum % 256 != c){
//                         state = 0;
//                         sum = 0;
//                     }else{
//                         flag = 1;
//                     }
//             }

          
//         }

//         if(flag == 1){
//             uint32_t combined_aoa = (aoa_buf[0] << 24) | (aoa_buf[1] << 16) | (aoa_buf[2] << 8) | aoa_buf[3];
//             uint32_t combined_ssa = (ssa_buf[0] << 24) | (ssa_buf[1] << 16) | (ssa_buf[2] << 8) | ssa_buf[3];
//             floatValue_aoa = hexToFloat(combined_aoa);
//             // hal.console->printf("Number of bytes available: %f\n", floatValue_aoa);//整数，没收到数据那就是0
//             floatValue_ssa = hexToFloat(combined_ssa);
//             break;  
//         }
//     }
//     //  uart->discard_input();


// }



//法二
static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        return;
    }

    static uint8_t state = 0;
    static uint8_t sum = 0;
    static const int bufferSize = 74;  // 预定义缓冲区大小
    uint8_t buffer[bufferSize];

    int16_t nbytes = uart->available();
    // hal.console->printf("Number of bytes available: %d\n", nbytes);
    
    // 确保一次读取74个字节
    if (nbytes >= bufferSize) {
        reset_flag();
        ssize_t bytesRead = uart->read(buffer, bufferSize);
        // hal.console->printf("Number of bytes available: %d\n", flag);
        if (bytesRead == bufferSize) {
            reset_flag();
            // hal.console->printf("Number of bytes available: %d\n", flag);
            for (int i = 0; i < bufferSize; ++i) {
                uint8_t c = buffer[i];
                
                switch (state) {
                    case 0:
                        if (c == packstart1) {
                            sum += c;
                            state = 1;
                        } else {
                            sum = 0;
                            state = 0;
                        }
                        break;
                    case 1:
                        if (c == packstart2) {
                            sum += c;
                            state = 2;
                        } else {
                            sum = 0;
                            state = 0;
                        }
                        break;
                    case 2:
                        if (c == 0x25) {
                            sum += c;
                            state = 3;
                        } else {
                            sum = 0;
                            state = 0;
                        }
                        break;
                    case 3:
                        sum += c;
                        state = 4;
                        index_aoa = 0;
                        index_ssa = 0;
                        index_buf1 = 0;
                        break;
                    case 4:
                        sum += c;
                        aoa_buf[index_aoa] = c;
                        index_aoa++;
                        if (index_aoa >= 4) {
                            state = 5;
                        }
                        break;
                    case 5:
                        sum += c;
                        ssa_buf[index_ssa] = c;
                        index_ssa++;
                        if (index_ssa >= 4) {
                            state = 6;
                        }
                        break;
                    case 6:
                        sum += c;
                        buf1[index_buf1] = c;
                        index_buf1++;
                        if (index_buf1 >= 24) {
                            state = 7;
                        }
                        break;
                    case 7:
                        if (sum % 256 == c) {
                            flag = 1;
                        }
                        state = 0;// 无论成功与否，都重置状态机
                        sum = 0;
                        break;
                }

                if (flag == 1) {
                    uint32_t combined_aoa = (aoa_buf[0] << 24) | (aoa_buf[1] << 16) | (aoa_buf[2] << 8) | aoa_buf[3];
                    uint32_t combined_ssa = (ssa_buf[0] << 24) | (ssa_buf[1] << 16) | (ssa_buf[2] << 8) | ssa_buf[3];
                    floatValue_aoa = hexToFloat(combined_aoa);
                    floatValue_ssa = hexToFloat(combined_ssa);
                    last_aoa = floatValue_aoa;
                    last_ssa = floatValue_ssa;
                    flag = 0;
                    break;
                }
            }
        }else{
        floatValue_aoa = last_aoa;
        floatValue_ssa = last_ssa;
        }
    }else{
        floatValue_aoa = last_aoa;
        floatValue_ssa = last_ssa;
    }
    // uart->discard_input();
}



void aoassa::init()
{
    hal.scheduler->delay(100);
    setup_uart(hal.serial(4), "SERIAL4");  
}



void aoassa::update(void)
{
    test_uart(hal.serial(4), "SERIAL4");
    _aoa = floatValue_aoa;
    _ssa = floatValue_ssa;

}



uint8_t reset_flag(void){
    if(flag == 1){
        flag = 0;
        
    }
    return flag;
}

float hexToFloat(uint32_t hexValue) {
    float floatValue;
    memcpy(&floatValue, &hexValue, sizeof(hexValue));
    return floatValue;
}

aoassa *aoassa::_singleton;

namespace AP {
    aoassa *Aoassa()//返回指针
    {
        return aoassa::get_singleton();
    }

}



