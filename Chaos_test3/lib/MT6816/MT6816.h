/* MT6816.h */
#ifndef INC_MT6816_H_
#define INC_MT6816_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include "FOC_utils.h"
#include "bldc_quickLut.h"

#define MT6816_RESOLUTION       16384U
#define POLE_PAIRS              7
#define ENCODER_MASK            0x3FFFU
#define ANGLE_FILTER_ALPHA      0.71539f
#define MAX_ANGLE_JUMP_DEG      50.0f
#define SPIKE_REJECT_COUNT      10
#define MAX_RPM_JUMP            50.0f
#define ACTUAL_ANGLE_FILTER_ALPHA   0.71539f

#define USE_RPM_SPIKE_REJECTION
#define USE_RPM_FILTER

/* Lệnh đọc Burst: Đọc Reg 0x03, chip tự động trả về Reg 0x03 rồi Reg 0x04 */
#define MT6816_ANGLE_REG_ADDR   0x83U

typedef struct {
    uint16_t rawData;
    uint16_t rawAngle;
    bool     magCheck;
    bool     parityCheck;
    uint8_t  spikeCounter;
    float prevRawTheta;
    float prevMechTheta;
    float mechTheta;

    float multiturnAngleRound; // Nên đổi thành int32_t nếu chỉ dùng làm bộ đếm vòng
    float prevMultiturnAngle;
    float multiturnAngle;

    float dtheta;
    float prevRPM;
    float vel;

    uint8_t check;
    uint8_t initialized;
    volatile uint32_t validCount;
    volatile uint32_t parityErrorCount;
    volatile uint32_t spikeRejectCount;
    volatile uint32_t spiBusyCount;
    volatile uint32_t spiErrorCount;
    volatile uint32_t lastUpdateMs;

    /* Mảng 2 byte nhận về từ 1 lần đọc Burst */
    uint8_t rx_8Bit[2];
    uint8_t complete_16Bit[2]; // <-- Thêm: dùng để ghép 2 byte nhận được
    uint8_t read_flag;         // <-- Thêm: cờ trạng thái đọc (đã khởi tạo trong .c)

    GPIO_TypeDef       *CS_Port;
    uint16_t            CS_Pin;
    SPI_HandleTypeDef  *spi;
} Encoder_t;

extern Encoder_t encoder;
extern const uint8_t reg_addrs[2];

void MT6816_Config(GPIO_TypeDef *CS_Port, uint16_t CS_Pin, SPI_HandleTypeDef *spi);
void MT6816_Select(void);
void MT6816_DeSelect(void);
void MT6816_Start(void);
void MT6816_Update_Angle_8Bit(foc_t *foc);
void MT6816_Get_RPM(foc_t *foc, float dt_us);
void MT6816_get_multiturn_degree(foc_t *foc);
void Spi_Transmit_Receive_Data_8Bit(uint8_t reg); // <-- Thêm: khai báo hàm dùng trong .c

#endif /* INC_MT6816_H_ */
