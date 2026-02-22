/*
 * RP2350 (Pico 2) + XL2515 slcan Firmware
 * 
 * slcan (Serial Line CAN) プロトコルを実装し、
 * Linux slcand / Windows CANツール (Savvycan, Cangaroo等) から
 * CANable互換デバイスとして利用可能にする。
 *
 * Controller: XL2515 (MCP2515 Compatible)
 * Crystal: 16MHz
 */

#include <SPI.h>
#include <mcp_can.h>

// --- ピン定義 (回路図に基づく) ---
const int PIN_CAN_INT  = 8;
const int PIN_SPI_CS   = 9;
const int PIN_SPI_SCK  = 10;
const int PIN_SPI_MOSI = 11;
const int PIN_SPI_MISO = 12;
// PIN_LED はフレームワークで定義済み (25)

// MCP_CANインスタンス
MCP_CAN CAN0(&SPI1, PIN_SPI_CS);

// --- slcan 状態管理 ---
bool     slcanOpened      = false;
bool     timestampEnabled = false;
uint8_t  canSpeedIdx      = 8;  // デフォルト: S8 = 1Mbps

// シリアルコマンドバッファ
#define SLCAN_CMD_BUF_SIZE 64
char     slcanBuf[SLCAN_CMD_BUF_SIZE];
uint8_t  slcanBufIdx = 0;

// 割り込みフラグ
volatile bool flagRecv = false;

// --- ビットレートマッピング ---
// S0=10k, S1=20k, S2=50k, S3=100k, S4=125k, S5=250k, S6=500k, S7=750k(未対応), S8=1M
uint8_t getMcpSpeed(uint8_t idx) {
    switch (idx) {
        case 0: return CAN_10KBPS;
        case 1: return CAN_20KBPS;
        case 2: return CAN_50KBPS;
        case 3: return CAN_100KBPS;
        case 4: return CAN_125KBPS;
        case 5: return CAN_250KBPS;
        case 6: return CAN_500KBPS;
        // case 7: 750kbps は MCP2515 に標準定義なし
        case 8: return CAN_1000KBPS;
        default: return CAN_500KBPS;
    }
}

// --- ユーティリティ関数 ---
// 16進1文字 → 4bitの値
int8_t hexCharToNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// 4bitの値 → 16進1文字（大文字）
char nibbleToHexChar(uint8_t n) {
    n &= 0x0F;
    return (n < 10) ? ('0' + n) : ('A' + n - 10);
}

// ACK: \r を送信
void slcanAck() {
    Serial.write('\r');
}

// NACK: BEL (0x07) を送信
void slcanNack() {
    Serial.write(0x07);
}

// --- MCP2515 割り込みハンドラ ---
void MCP2515_ISR() {
    flagRecv = true;
}

// --- CANチャンネルを開く ---
bool openCanChannel() {
    // SPIピン設定
    SPI1.setSCK(PIN_SPI_SCK);
    SPI1.setTX(PIN_SPI_MOSI);
    SPI1.setRX(PIN_SPI_MISO);
    SPI1.begin();

    uint8_t speed = getMcpSpeed(canSpeedIdx);
    if (CAN0.begin(MCP_ANY, speed, MCP_16MHZ) != CAN_OK) {
        return false;
    }
    CAN0.setMode(MCP_NORMAL);

    // 割り込み設定
    pinMode(PIN_CAN_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), MCP2515_ISR, FALLING);

    slcanOpened = true;
    return true;
}

// --- CANチャンネルを閉じる ---
void closeCanChannel() {
    detachInterrupt(digitalPinToInterrupt(PIN_CAN_INT));
    slcanOpened = false;
}

// --- 受信したCANフレームをslcan形式でシリアル出力 ---
void sendCanMsgToSerial() {
    while (CAN0.checkReceive() == CAN_MSGAVAIL) {
        long unsigned int rxId;
        unsigned char len = 0;
        unsigned char rxBuf[8];

        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        bool isExtended = (rxId & 0x80000000UL) != 0;
        rxId &= 0x1FFFFFFFUL; // 29bit以下に制限

        if (isExtended) {
            // 拡張フレーム: Tiiiiiiiildd...
            Serial.write('T');
            // 8桁の16進ID
            for (int i = 7; i >= 0; i--) {
                Serial.write(nibbleToHexChar((rxId >> (i * 4)) & 0x0F));
            }
        } else {
            // 標準フレーム: tiiildd...
            Serial.write('t');
            // 3桁の16進ID
            Serial.write(nibbleToHexChar((rxId >> 8) & 0x0F));
            Serial.write(nibbleToHexChar((rxId >> 4) & 0x0F));
            Serial.write(nibbleToHexChar(rxId & 0x0F));
        }

        // DLC
        Serial.write(nibbleToHexChar(len & 0x0F));

        // データバイト
        for (uint8_t i = 0; i < len && i < 8; i++) {
            Serial.write(nibbleToHexChar(rxBuf[i] >> 4));
            Serial.write(nibbleToHexChar(rxBuf[i] & 0x0F));
        }

        // タイムスタンプ (有効時: 16bit ミリ秒)
        if (timestampEnabled) {
            uint16_t ts = (uint16_t)(millis() & 0xFFFF);
            Serial.write(nibbleToHexChar((ts >> 12) & 0x0F));
            Serial.write(nibbleToHexChar((ts >> 8) & 0x0F));
            Serial.write(nibbleToHexChar((ts >> 4) & 0x0F));
            Serial.write(nibbleToHexChar(ts & 0x0F));
        }

        Serial.write('\r');

        // LED トグル
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
}

// --- slcan コマンドの解析・実行 ---
void slcanParseCmd(char *buf, uint8_t len) {
    if (len == 0) return;

    switch (buf[0]) {
        // --- チャンネル制御 ---
        case 'O': // CANチャンネルを開く
            if (openCanChannel()) {
                slcanAck();
            } else {
                slcanNack();
            }
            break;

        case 'C': // CANチャンネルを閉じる
            closeCanChannel();
            slcanAck();
            break;

        case 'L': // Listen-only mode
            if (!slcanOpened) {
                // SPIピン設定
                SPI1.setSCK(PIN_SPI_SCK);
                SPI1.setTX(PIN_SPI_MOSI);
                SPI1.setRX(PIN_SPI_MISO);
                SPI1.begin();
                uint8_t speed = getMcpSpeed(canSpeedIdx);
                if (CAN0.begin(MCP_ANY, speed, MCP_16MHZ) == CAN_OK) {
                    CAN0.setMode(MCP_LISTENONLY);
                    pinMode(PIN_CAN_INT, INPUT_PULLUP);
                    attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), MCP2515_ISR, FALLING);
                    slcanOpened = true;
                    slcanAck();
                } else {
                    slcanNack();
                }
            } else {
                slcanNack();
            }
            break;

        // --- ビットレート設定 ---
        case 'S': // S0〜S8
            if (len >= 2 && !slcanOpened) {
                uint8_t idx = buf[1] - '0';
                if (idx <= 8 && idx != 7) {
                    canSpeedIdx = idx;
                    slcanAck();
                } else {
                    slcanNack();
                }
            } else {
                slcanNack();
            }
            break;

        // --- 標準フレーム送信 ---
        case 't': { // tiiildd...\r  (11bit ID)
            if (!slcanOpened || len < 5) { slcanNack(); break; }

            uint16_t id = 0;
            for (int i = 1; i <= 3; i++) {
                int8_t n = hexCharToNibble(buf[i]);
                if (n < 0) { slcanNack(); return; }
                id = (id << 4) | n;
            }
            int8_t dlcN = hexCharToNibble(buf[4]);
            if (dlcN < 0 || dlcN > 8) { slcanNack(); break; }
            uint8_t dlc = (uint8_t)dlcN;

            if (len < (int)(5 + dlc * 2)) { slcanNack(); break; }

            uint8_t data[8] = {0};
            for (uint8_t i = 0; i < dlc; i++) {
                int8_t hi = hexCharToNibble(buf[5 + i * 2]);
                int8_t lo = hexCharToNibble(buf[6 + i * 2]);
                if (hi < 0 || lo < 0) { slcanNack(); return; }
                data[i] = (hi << 4) | lo;
            }

            if (CAN0.sendMsgBuf(id, 0, dlc, data) == CAN_OK) {
                // slcan仕様: 送信成功時 'z\r' (一部実装) または '\r'
                Serial.write('z');
                slcanAck();
                digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            } else {
                slcanNack();
            }
            break;
        }

        // --- 拡張フレーム送信 ---
        case 'T': { // Tiiiiiiiildd...\r  (29bit ID)
            if (!slcanOpened || len < 10) { slcanNack(); break; }

            uint32_t id = 0;
            for (int i = 1; i <= 8; i++) {
                int8_t n = hexCharToNibble(buf[i]);
                if (n < 0) { slcanNack(); return; }
                id = (id << 4) | n;
            }
            int8_t dlcN = hexCharToNibble(buf[9]);
            if (dlcN < 0 || dlcN > 8) { slcanNack(); break; }
            uint8_t dlc = (uint8_t)dlcN;

            if (len < (int)(10 + dlc * 2)) { slcanNack(); break; }

            uint8_t data[8] = {0};
            for (uint8_t i = 0; i < dlc; i++) {
                int8_t hi = hexCharToNibble(buf[10 + i * 2]);
                int8_t lo = hexCharToNibble(buf[11 + i * 2]);
                if (hi < 0 || lo < 0) { slcanNack(); return; }
                data[i] = (hi << 4) | lo;
            }

            if (CAN0.sendMsgBuf(id, 1, dlc, data) == CAN_OK) {
                Serial.write('Z');
                slcanAck();
                digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            } else {
                slcanNack();
            }
            break;
        }

        // --- 標準RTRフレーム送信 ---
        case 'r': { // riiil\r
            if (!slcanOpened || len < 5) { slcanNack(); break; }
            uint16_t id = 0;
            for (int i = 1; i <= 3; i++) {
                int8_t n = hexCharToNibble(buf[i]);
                if (n < 0) { slcanNack(); return; }
                id = (id << 4) | n;
            }
            int8_t dlcN = hexCharToNibble(buf[4]);
            if (dlcN < 0 || dlcN > 8) { slcanNack(); break; }
            uint8_t dlc = (uint8_t)dlcN;
            // RTRフレーム: sendMsgBuf の rtr パラメータを使う
            CAN0.sendMsgBuf(id, 0, dlc, NULL);
            slcanAck();
            break;
        }

        // --- 拡張RTRフレーム送信 ---
        case 'R': { // Riiiiiiiil\r
            if (!slcanOpened || len < 10) { slcanNack(); break; }
            uint32_t id = 0;
            for (int i = 1; i <= 8; i++) {
                int8_t n = hexCharToNibble(buf[i]);
                if (n < 0) { slcanNack(); return; }
                id = (id << 4) | n;
            }
            int8_t dlcN = hexCharToNibble(buf[9]);
            if (dlcN < 0 || dlcN > 8) { slcanNack(); break; }
            uint8_t dlc = (uint8_t)dlcN;
            CAN0.sendMsgBuf(id, 1, dlc, NULL);
            slcanAck();
            break;
        }

        // --- ステータス ---
        case 'F': // ステータスフラグ取得
            Serial.print("F00");
            slcanAck();
            break;

        // --- バージョン ---
        case 'V': // バージョン情報
            Serial.print("V0101");
            slcanAck();
            break;

        // --- シリアル番号 ---
        case 'N': // シリアル番号
            Serial.print("NRP25");
            slcanAck();
            break;

        // --- タイムスタンプ ---
        case 'Z': // Z0=無効, Z1=有効
            if (len >= 2) {
                timestampEnabled = (buf[1] == '1');
                slcanAck();
            } else {
                slcanNack();
            }
            break;

        default:
            slcanNack();
            break;
    }
}

// =============================================
void setup() {
    Serial.begin(115200);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // SPIピンの初期設定（openCanChannel時にも再設定される）
    SPI1.setSCK(PIN_SPI_SCK);
    SPI1.setTX(PIN_SPI_MOSI);
    SPI1.setRX(PIN_SPI_MISO);

    // 起動時LEDを3回点滅
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(100);
        digitalWrite(PIN_LED, LOW);
        delay(100);
    }

    slcanBufIdx = 0;
}

void loop() {
    // --- シリアルからコマンドを受信 ---
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\r') {
            // コマンド完了 → 解析
            slcanBuf[slcanBufIdx] = '\0';
            if (slcanBufIdx > 0) {
                slcanParseCmd(slcanBuf, slcanBufIdx);
            }
            slcanBufIdx = 0;
        } else if (c == '\n') {
            // LFは無視 (一部ツールは \r\n を送る)
        } else {
            if (slcanBufIdx < SLCAN_CMD_BUF_SIZE - 1) {
                slcanBuf[slcanBufIdx++] = c;
            } else {
                // バッファオーバーフロー → リセット
                slcanBufIdx = 0;
                slcanNack();
            }
        }
    }

    // --- CAN受信処理 ---
    if (slcanOpened && flagRecv) {
        flagRecv = false;
        sendCanMsgToSerial();
    }

    // 割り込みが来なくても定期的にポーリング（割り込み取りこぼし対策）
    if (slcanOpened) {
        static unsigned long lastPoll = 0;
        if (millis() - lastPoll > 10) {
            lastPoll = millis();
            if (CAN0.checkReceive() == CAN_MSGAVAIL) {
                sendCanMsgToSerial();
            }
        }
    }
}