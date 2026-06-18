#include <main_core.h>

unsigned long last_1000hz_tick = 0;
unsigned long last_50hz_tick = 0;

//#define DEFAULT_ROLE 1 // 1: offense, 2: defense
#define DEFAULT_ROLE 2 // 1: offense, 2: defense

void setup() {
    main_core_init();
    while(1) {
        drawMessage("Waiting for SubCore...");
        if(DEFAULT_ROLE == 1) {// 1: offense, 2: defense
            Serial8.write(0x0A);
        } else if(DEFAULT_ROLE == 2) {// 1: offense, 2: defense
            Serial8.write(0x0D);
        }
        // Wait a short moment for the sub-core to respond 
        if(Serial8.available() > 0) {
            if(Serial8.read() == PROTOCAL_ACT) {
                break; // Connection confirmed
            }
        }
    }    
    while(UI_Interface()) {
        ;
    }
}


// 宣告微秒等級的計時器
unsigned long last_1hz_tick = 0;
unsigned long last_ball_request_tick = 0; // 新增：非阻塞球資訊請求計時器

void loop() {
    stopMotors();
    while(!digitalRead(COM_O1_PIN));
    while (1) {
        unsigned long current_micros = micros(); // 統一使用微秒，精準且減少暫存器切換

        // ==================================================================
        // 核心底層：全速非阻塞序列埠引擎 (盲抽緩衝區，0等待)
        // ==================================================================
        // 不管現在是幾 Hz，只要序列埠有不完整的字節進來，立刻抽空或解析，絕不卡死
        parseESP32(); 

        // ==================================================================
        // 1. 低頻排程 (50Hz = 20000 微秒)：主動發送請求（僅發送，不等待）
        // ==================================================================
        if (current_micros - last_50hz_tick >= 20000) {
            last_50hz_tick = current_micros;
            
            if (robotMonitor.role == RobotRole::DEFENSE) {
                Serial6.write(0xBD); // 只丟出請求權杖 (Request teammate)，丟完秒走人！
            }
        }

        // ==================================================================
        // 2. 高頻傳輸排程 (500Hz ~ 1000Hz)：主動發送球訊號請求
        // ==================================================================
        // 注意：不建議每 1ms 都向 ESP32 發 request，因為 ESP32 的中斷響應沒那麼快，會塞車。
        // 建議每 2ms (500Hz) 發一次請求，丟完立刻離開。
        if (current_micros - last_ball_request_tick >= 2000) {
            last_ball_request_tick = current_micros;
            Serial6.write(0xDD); // 只丟出請求權杖 (Request ball)，丟完秒走人！
        }

        // ==================================================================
        // 3. 高頻核心控制迴圈 (1000Hz = 1000 微秒)
        // ==================================================================
        if (current_micros - last_1000hz_tick >= 1000) {
            last_1000hz_tick = current_micros;
            //TODO; 
        }
        // ==================================================================
        // 3. 高頻核心控制迴圈 (1000Hz = 1000 微秒)
        // ==================================================================       
        if (current_micros - last_1hz_tick >= 1000000) { // 1Hz = 1,000,000 微秒
            last_1hz_tick = current_micros;
            if (digitalReadFast(COM_O1_PIN) == HIGH) {
                break;
            }
        }
    }
}
