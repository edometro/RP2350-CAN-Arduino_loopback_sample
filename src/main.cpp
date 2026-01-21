/*
 * RP2350 (Pico 2) + XL2515 CAN Bus Code
 * Reference: Provided working C SDK code & RP2350-CAN-Schematic
 *
 * Controller: XL2515 (MCP2515 Compatible)
 * Crystal: 16MHz 
 * Bitrate: 1Mbps
 */

#include <SPI.h>
#include <mcp_can.h>

// --- ピン定義 (回路図およびCコードに基づく) ---
// 注意: CコードではINTが8でしたが、回路図(GPIO21)を優先しています
const int PIN_CAN_INT  = 8;  // 
const int PIN_SPI_CS   = 9;   // [cite: 991]
const int PIN_SPI_SCK  = 10;  // [cite: 1019]
const int PIN_SPI_MOSI = 11;  // [cite: 999]
const int PIN_SPI_MISO = 12;  // [cite: 996]

// LEDピン (main.cと同様に動作確認用)
// const int PIN_LED      = 25;

// MCP_CANインスタンス
MCP_CAN CAN0(&SPI1, PIN_SPI_CS);

// 割り込みフラグ
volatile bool flagRecv = false;

// 割り込みサービスルーチン (Cコードの gpio_callback に相当)
void MCP2515_ISR() {
    flagRecv = true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);


    Serial.println("RP2350 CAN Bus Initializing based on SDK code...");

    pinMode(PIN_LED, OUTPUT);

    // // SPIピンの明示的設定 (Cコードの gpio_set_function に相当)
    SPI1.setSCK(PIN_SPI_SCK);
    SPI1.setTX(PIN_SPI_MOSI);
    SPI1.setRX(PIN_SPI_MISO);
    SPI1.begin();

    // CANコントローラー初期化
    // Cコードの xl2515_init に相当
    // 速度: 1Mbps, クロック: 16MHz
    // 注意: 初期化に失敗する場合は、MCP_ANYを外してフィルタ設定を見直す必要がありますが
    // まずは疎通確認のため全てのIDを受信する設定(MCP_ANY)にします。
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
        Serial.println("CAN Init OK!");
    } else {
        Serial.println("CAN Init Failed...");
        while (1);
    }

    // モード設定 (Cコードの REQOP_NORMAL に相当)
    // ループバックテストを行う場合はここを MCP_LOOPBACK に変更してください
    // CAN0.setMode(MCP_NORMAL);
    CAN0.setMode(MCP_LOOPBACK);

    // 割り込み設定 (Cコードの gpio_set_irq_enabled... FALLING に相当)
    pinMode(PIN_CAN_INT, INPUT_PULLDOWN); // 内部プルアップ有効化
    attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), MCP2515_ISR, FALLING);

    Serial.println("CAN Bus Ready. Sending pattern 0x00-0x77 to ID 0x123.");
}

void loop() {
    // --- 受信処理 (Cコードの xl2515_recv に相当) ---
    if (flagRecv) {
        flagRecv = false; // フラグクリア

        // 割り込み原因を確認してデータを読み出す
        while (CAN0.checkReceive() == CAN_MSGAVAIL) {
            long unsigned int rxId;
            unsigned char len = 0;
            unsigned char rxBuf[8];

            CAN0.readMsgBuf(&rxId, &len, rxBuf);

            // Cコードの printf フォーマットに合わせて表示
            Serial.print("recv 0x");
            Serial.print(rxId, HEX);
            Serial.print(": ");
            for (byte i = 0; i < len; i++) {
                if (rxBuf[i] < 0x10) Serial.print("0");
                Serial.print(rxBuf[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

    // --- 送信処理 (Cコードの main whileループ に相当) ---
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 1000) { // 1000ms間隔
        lastSendTime = millis();

        // Cコードと同じデータパターン
        // uint8_t data[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
        byte data[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
        
        // ID: 0x123 (Standard Frame)
        byte sndStat = CAN0.sendMsgBuf(0x123, 0, 8, data);

        if (sndStat == CAN_OK) {
            Serial.println("Message Sent: ID 0x123");
            // LEDトグル (Cコードの led_state = !led_state に相当)
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        } else {
            Serial.println("Error Sending Message...");
        }
    }
}

void setup1(){
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(100);
}

void loop1(){
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(100);
}