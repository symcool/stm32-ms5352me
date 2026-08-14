#ifndef __MS5352ME_H
#define __MS5352ME_H

#include "i2c.h"
#include "sys.h"

/* ---------------- 公共类型 ---------------- */
typedef enum {
    ms5352me_PLL_A = 0,
    ms5352me_PLL_B,
} ms5352PLL_t;

typedef enum {
    ms5352me_R_DIV_1   = 0,
    ms5352me_R_DIV_2   = 1,
    ms5352me_R_DIV_4   = 2,
    ms5352me_R_DIV_8   = 3,
    ms5352me_R_DIV_16  = 4,
    ms5352me_R_DIV_32  = 5,
    ms5352me_R_DIV_64  = 6,
    ms5352me_R_DIV_128 = 7,
} ms5352RDiv_t;

typedef struct {
    int32_t mult;
    int32_t num;
    int32_t denom;
} ms5352PLLConfig_t;

/* ---------------- 寄存器定义 ---------------- */
enum {
    ms5352me_REGISTER_0_DEVICE_STATUS       = 0,
    ms5352me_REGISTER_3_OUTPUT_ENABLE       = 3,
    ms5352me_REGISTER_16_CLK0_CONTROL       = 16,
    ms5352me_REGISTER_17_CLK1_CONTROL       = 17,
    ms5352me_REGISTER_18_CLK2_CONTROL       = 18,
    ms5352me_REGISTER_24_DISABLE_STATE      = 24,
    ms5352me_REGISTER_26_PLLA_PARAMETERS    = 26,
    ms5352me_REGISTER_34_PLLB_PARAMETERS    = 34,
    ms5352me_REGISTER_42_DIV0_PARAMETERS    = 42,
    ms5352me_REGISTER_44_CLK0_RDIV          = 44,   /* CLK0 输出级分频 OUT0_DIV (R0_DIV) */
    ms5352me_REGISTER_50_DIV1_PARAMETERS    = 50,
    ms5352me_REGISTER_52_CLK1_RDIV          = 52,   /* CLK1 输出级分频 OUT1_DIV (R1_DIV) */
    ms5352me_REGISTER_58_DIV2_PARAMETERS    = 58,
    ms5352me_REGISTER_60_CLK2_RDIV          = 60,   /* CLK2 输出级分频 OUT2_DIV (R2_DIV) */
    ms5352me_REGISTER_165_CLK0_PHASE        = 165,
    ms5352me_REGISTER_166_CLK1_PHASE        = 166,
    ms5352me_REGISTER_167_CLK2_PHASE        = 167,
    ms5352me_REGISTER_177_PLL_RESET         = 177,
    ms5352me_REGISTER_183_XTAL_LOAD         = 183,
    ms5352me_REGISTER_187_FANOUT            = 187
};

#define MS5352ME_XTAL_FREQ       25000000    // 25MHz 有源晶振
#define MS5352ME_MIN_FREQ_CLK0   8000        // CLK0 算法可精确覆盖的下限 8kHz（低于则钳位）
#define MS5352ME_MAX_FREQ_CLK0   200000000   // CLK0 最大频率 200MHz
#define MS5352ME_MAX_FREQ_CLK12  500000000   // CLK1/CLK2 最大频率 500MHz

#define MS5352ME_DIV0 0   // 唯一的小数分频器（CLK0 专属），8kHz~200MHz
#define MS5352ME_DIV1 1   // 固定 /2 整数分频器（CLK1 专属），2MHz~500MHz
#define MS5352ME_DIV2 2   // 固定 /2 整数分频器（CLK2 专属），2MHz~500MHz

/* ============================================================
   MS5352ME 频率范围与组合限制（对照 appnote.txt L166/L199/L211/L245/L261）
   ------------------------------------------------------------
   【每路范围】(ms5352me_SetFrequencies 入参，越界自动静默钳位，不报错)
     CLK0 : 8kHz ~ 200MHz（DIV0 小数分频器）   -> <8k 钳 8k / >200M 钳 200M
     CLK1 : 2MHz ~ 500MHz（DIV1 固定 /2）       -> <2M 钳 2M / >500M 钳 500M
     CLK2 : 2MHz ~ 500MHz（DIV2 固定 /2）       -> <2M 钳 2M / >500M 钳 500M
     （注：芯片手册 CLK0 宣称 2.5kHz 起点，但驱动放大算法（2^n, n<=7）受
      DIV0 分频比 <=1800 约束，实际可精确覆盖下限 8kHz，与 MS5351M 一致）

   【内部硬约束】
     VCO         : 硬范围 500 ~ 1000MHz；推荐 600 ~ 900MHz（PLL_DIV 24~36 @25MHz）
     DIV0 分频比 : 仅 {4,6,8} 或 [8,1800]
     DIV1/DIV2   : 只能固定 2 分频（appnote L211，非小数分频器）
     >150MHz     : 强制 DIVBY4 + INT=1
     >120MHz     : 仅允许同时输出 2 路不同时钟

   【PLL 分配】
     CLK0 -> PLLA
     CLK1 -> CLK0 在场 ? PLLB : PLLA
     CLK2 -> CLK0 在场 ? PLLB : (CLK1 在场 ? PLLB : PLLA)

   【组合限制】
     Case B（CLK0 关闭）: CLK1/CLK2 各占独立 PLL，可任意异频（2M~500M）
     Case A（CLK0 在场）: CLK1/CLK2 共享 PLLB；因 DIV1/2 固定 /2 必同频，
       驱动强制 CLK1/CLK2 共用一个频率（取有效频率优先 CLK1；都无效取 CLK1 最近边界 2M/500M）
   ============================================================ */

/* ---------------- 接口 ---------------- */
void ms5352me_write(uint8_t reg, uint8_t value);
void ms5352me_Init(void);
void ms5352me_writePLL(uint8_t baseaddr, const ms5352PLLConfig_t* pll);
void ms5352me_WriteDivider(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3,
                         uint8_t divBy4, uint8_t divBy2, uint8_t rdiv);

/* 核心：按 drive/freq 自动分配 PLL 与 DIV0/1/2，越界静默钳位，void 接口（无返回值） */
void ms5352me_Set(int32_t freq0, uint8_t drive0,
                             int32_t freq1, uint8_t drive1,
                             int32_t freq2, uint8_t drive2);

#endif
