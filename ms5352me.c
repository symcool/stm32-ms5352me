#include "ms5352me.h"
#include "i2c.h"
#include "sys.h"
#include "rf.h"

/* ================= 底层 ================= */

void ms5352me_write(uint8_t reg, uint8_t value) {
    my_I2C_sendREG(reg, value);
}

/* MS5352ME 专用初始化 */
void ms5352me_Init(void) {
    ms5352me_write(ms5352me_REGISTER_3_OUTPUT_ENABLE, 0xFF);
    ms5352me_write(ms5352me_REGISTER_16_CLK0_CONTROL, 0x80);
    ms5352me_write(ms5352me_REGISTER_17_CLK1_CONTROL, 0x80);
    ms5352me_write(ms5352me_REGISTER_18_CLK2_CONTROL, 0x80);
    ms5352me_write(ms5352me_REGISTER_183_XTAL_LOAD, 0xC0 | 0x12);
    /* Reg187 恢复上电默认 0x00：CLK1/CLK2 已强制走各自专属的 DIV1/DIV2，
       绝不共享 DIV0，故不再需要 MS 扇出(MS_FANOUT_EN)与 XO 扇出。
       CLK0 直选 DIV0，扇出位对其无影响（实测 CLK0@1M 在原 0x00 状态下即正常）。 */
    ms5352me_write(ms5352me_REGISTER_187_FANOUT, 0x00);
}

/* ================= 寄存器写入（修改版实现） ================= */

/* 写 8 字节 PLL 参数（Reg26/34）
 * 由 pll 配置(mult,num,denom)按手册公式算出 P1/P2/P3 并突发写入。*/
void ms5352me_writePLL(uint8_t baseaddr, const ms5352PLLConfig_t* pll) {
    int64_t a = pll->mult;
    int64_t b = pll->num;
    int64_t c = pll->denom;
    int64_t P1 = 128 * a + (128 * b) / c - 512;
    int64_t P2 = 128 * b - c * ((128 * b) / c);
    int64_t P3 = c;
    ms5352me_write(baseaddr,   (P3 >> 8) & 0xFF);
    ms5352me_write(baseaddr+1, P3 & 0xFF);
    ms5352me_write(baseaddr+2, (P1 >> 16) & 0x03);
    ms5352me_write(baseaddr+3, (P1 >> 8) & 0xFF);
    ms5352me_write(baseaddr+4, P1 & 0xFF);
    ms5352me_write(baseaddr+5, ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0x0F));
    ms5352me_write(baseaddr+6, (P2 >> 8) & 0xFF);
    ms5352me_write(baseaddr+7, P2 & 0xFF);
}

/* 写分频器参数（DIV0->Reg42 / DIV1->Reg50 / DIV2->Reg58）
 * rdiv     : 输出级 OUTn_DIV 的 2^N 指数 N（D6:4）
 * divBy4   : DIV0 比值=4 时的 DIVBY4 标志（D3:2）
 * divBy2   : DIV1/DIV2 固定 /2 标志（D3:2）*/
void ms5352me_WriteDivider(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3,
                         uint8_t divBy4, uint8_t divBy2, uint8_t rdiv) {
    ms5352me_write(baseaddr,   (P3 >> 8) & 0xFF);
    ms5352me_write(baseaddr+1, P3 & 0xFF);
    ms5352me_write(baseaddr+2, ((P1 >> 16) & 0x03) |
                             ((divBy4 & 0x03) << 2) |
                             ((divBy2 & 0x03) << 2) |
                             ((rdiv & 0x07) << 4));
    ms5352me_write(baseaddr+3, (P1 >> 8) & 0xFF);
    ms5352me_write(baseaddr+4, P1 & 0xFF);
    ms5352me_write(baseaddr+5, ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0x0F));
    ms5352me_write(baseaddr+6, (P2 >> 8) & 0xFF);
    ms5352me_write(baseaddr+7, P2 & 0xFF);
}

/* DIV0（小数分频器）参数计算
 * 选出输出级分频 N，使 f_pre=freq*2^N 落在 [1MHz,200MHz]；
 * 再由 VCO(=Fxtal*mult, 500~1000MHz) 反算 DIV0 比值。*/
static void ms5352me_CalcDiv0(int32_t freq, ms5352PLLConfig_t* pll,
                            int32_t* P1, int32_t* P2, int32_t* P3,
                            uint8_t* divBy4, uint8_t* rdiv, uint8_t* isInt) {
    uint8_t n = 0;
    int32_t fpre = freq;
    while (fpre < 1000000 && n < 7) {
        n++;
        fpre = freq * (1 << n);
    }
    int32_t mult = 900000000 / MS5352ME_XTAL_FREQ;
    while ((int64_t)mult * MS5352ME_XTAL_FREQ > 1000000000LL) mult--;
    while ((int64_t)mult * MS5352ME_XTAL_FREQ <  500000000LL) mult++;
    int32_t Fpll = (int32_t)((int64_t)mult * MS5352ME_XTAL_FREQ);

    if (fpre > 81000000) {
        int32_t x;
        if      (fpre >= 150000000) x = 4;
        else if (fpre >= 100000000) x = 6;
        else                         x = 8;
        int64_t vco = (int64_t)x * fpre;
        pll->mult  = (int32_t)(vco / MS5352ME_XTAL_FREQ);
        pll->num   = (int32_t)((vco % MS5352ME_XTAL_FREQ) / 1000);
        pll->denom = MS5352ME_XTAL_FREQ / 1000;
        *P1 = 128 * x - 512;
        *P2 = 0;
        *P3 = 1;
        *divBy4 = (x == 4) ? 0x3 : 0;
        *isInt = 1;          // 分频比=x(整数) -> 可整数模式
    } else {
        int32_t x = Fpll / fpre;
        int32_t t = (fpre >> 20) + 1;
        int32_t y = (Fpll % fpre) / t;
        int32_t z = fpre / t;
        pll->mult  = Fpll / MS5352ME_XTAL_FREQ;
        pll->num   = (Fpll % MS5352ME_XTAL_FREQ) / 1000;
        pll->denom = MS5352ME_XTAL_FREQ / 1000;
        *P1 = 128 * x + (128 * y) / z - 512;
        *P2 = (128 * y) % z;
        *P3 = z;
        *divBy4 = 0;
        if (y == 0) {
            /* 分频比 x 恰好为整数：整数模式下 P3 必须=1（AN619 硬性要求）。
               否则部分芯片在 INT=1 时会误判分频比 -> 失锁/无输出。 */
            *P1 = 128 * x - 512;
            *P2 = 0;
            *P3 = 1;
            *isInt = 1;
        } else {
            *isInt = 0;
        }
    }
    *rdiv = n;
}

/* DIV1/DIV2（固定 /2 整数分频器）参数计算
 * VCO = 2*freq*2^N，选 N 使 VCO 落在 500~1000MHz。*/
static void ms5352me_CalcDiv2(int32_t freq, ms5352PLLConfig_t* pll, uint8_t* rdiv) {
    uint8_t n = 0;
    int64_t vco = (int64_t)2 * freq;
    while (vco < 500000000LL && n < 7) {
        n++;
        vco = (int64_t)2 * freq * (1 << n);
    }
    pll->mult  = (int32_t)(vco / MS5352ME_XTAL_FREQ);
    pll->num   = (int32_t)((vco % MS5352ME_XTAL_FREQ) / 1000);
    pll->denom = MS5352ME_XTAL_FREQ / 1000;
    *rdiv = n;
}

/* 写某路输出控制寄存器（Reg16/17/18）
 * divPdn : 该输出"专属分频器"(CLK0->DIV0, CLK1->DIV1, CLK2->DIV2)是否掉电。*/
static void ms5352me_WriteClkControl(uint8_t output, uint8_t div, uint8_t pll,
                                   uint8_t drive, uint8_t intMode, uint8_t divPdn) {
    uint8_t reg = 16 + output;
    uint8_t ctrl = 0;
    ctrl |= (divPdn & 0x01) << 7;          // D7 : 专属分频器掉电
    ctrl |= (pll    & 0x01) << 5;          // D5 : DIVn_SRC
    ctrl |= (intMode & 0x01) << 6;         // D6 : INT
    if (output == 0) ctrl |= 0x03 << 2;    // CLK0 只能选 DIV0
    else              ctrl |= (div == 0) ? (0x02 << 2) : (0x03 << 2);
    ctrl |= (drive & 0x03);                 // D1:0 : 驱动能力
    ms5352me_write(reg, ctrl);
}

/* 核心：频率设置与 PLL/DIV 分配（修改版）
 * 资源约束（数据手册 2.5/2.6）：
 *   - CLK0 只能 DIV0（小数，2.5kHz~200MHz），PLL 固定 A。
 *   - CLK1 只能 DIV1（固定/2，2MHz~500MHz），CLK2 只能 DIV2（固定/2，2MHz~500MHz）。
 *   - 已彻底弃用"CLK1/CLK2 共享 DIV0"的扇出路径：实测中该路径始终无法出波
 *     （扇出使能 / 整数模式 / 输出掉电位任一条件不满足即无输出），故强制 CLK1/CLK2
 *     走各自专属的 DIV1/DIV2，架构简单、一次写通即可稳定出波。
 *   - 因此 CLK1/CLK2 下限为 2MHz（DIV1/DIV2 硬件范围），低于 2MHz 由下方统一钳位到 2MHz。
 *   - 仅 2 个 PLL、3 路各自独立 MS：CLK0 用 PLLA；CLK1/CLK2 在 CLK0 在场时借 PLLB，
 *     三者同场且 CLK1/CLK2 频率不同时，因需共享同一 PLLB 的 VCO 而只能同频 ——
 *     **强制 CLK1/CLK2 共用一个频率**（见函数内强制同频段），保证一定出波。
 * 越界频率一律静默钳位到硬件边界并照常配置；本函数为 void 接口，不返回任何状态。*/
void ms5352me_Set(int32_t freq0, uint8_t drive0,
                            int32_t freq1, uint8_t drive1,
                            int32_t freq2, uint8_t drive2) {
    uint8_t en0 = (drive0 > 0) && (freq0 > 0);
    uint8_t en1 = (drive1 > 0) && (freq1 > 0);
    uint8_t en2 = (drive2 > 0) && (freq2 > 0);
    uint8_t sd0 = (drive0 > 0) ? (drive0 - 1) : 0;
    uint8_t sd1 = (drive1 > 0) ? (drive1 - 1) : 0;
    uint8_t sd2 = (drive2 > 0) ? (drive2 - 1) : 0;
    if (sd0 > 3) sd0 = 3;
    if (sd1 > 3) sd1 = 3;
    if (sd2 > 3) sd2 = 3;

    /* CLK0：DIV0，范围 8kHz~200MHz（算法实际可精确覆盖的下限 8kHz，
       与 MS5351M 一致；低于 8kHz 分频比会超 1800 非法），钳位处理 */
    int32_t f0 = freq0;
    if (en0) {
        if (f0 < MS5352ME_MIN_FREQ_CLK0)        f0 = MS5352ME_MIN_FREQ_CLK0;
        else if (f0 > MS5352ME_MAX_FREQ_CLK0)   f0 = MS5352ME_MAX_FREQ_CLK0;
    }

    /* CLK1/CLK2：只能用 DIV1/DIV2（2MHz~500MHz），绝不共享 DIV0。
       超出硬件边界的频率直接钳位到最近边界（下限 2MHz / 上限 500MHz），
       不报错、不当特殊情况，与 CLK0 超 200MHz 钳位到 200MHz 的处理完全一致。
       越界时直接钳位，照常完成配置，保证"一定出波"（本函数为 void 接口，无返回值）。 */
    int32_t f1 = freq1;
    int32_t f2 = freq2;
    if (en1) {
        if (f1 < 2000000)                       f1 = 2000000;
        else if (f1 > MS5352ME_MAX_FREQ_CLK12)  f1 = MS5352ME_MAX_FREQ_CLK12;
    }
    if (en2) {
        if (f2 < 2000000)                       f2 = 2000000;
        else if (f2 > MS5352ME_MAX_FREQ_CLK12)  f2 = MS5352ME_MAX_FREQ_CLK12;
    }

    /* PLL 分配（仅 2 个 PLL，3 路各自独立 MS）
       CLK0 -> PLLA；CLK1 -> 有 CLK0 在场借 PLLB 否则 PLLA；
       CLK2 -> 有 CLK0 在场借 PLLB，否则有 CLK1 借 PLLB，否则 PLLA。 */
    uint8_t pll1 = en0 ? (uint8_t)ms5352me_PLL_B : (uint8_t)ms5352me_PLL_A;
    uint8_t pll2 = en0 ? (uint8_t)ms5352me_PLL_B
                       : (en1 ? (uint8_t)ms5352me_PLL_B : (uint8_t)ms5352me_PLL_A);

    /* CLK0 在场且 CLK1/CLK2 均使能 -> 二者必须共享同一 PLLB 的 /2 分频器，只能同频。
       **强制 CLK1/CLK2 共用一个频率**：
         - 若二者中存在"有效频率"（使能且落在 [2M,500M]），取该频率（优先 CLK1）；
         - 若二者都"无效"（均未使能合法频率），取 CLK1 所接近的边界（<=2M -> 2M；>=500M -> 500M）。
       钳位后的 f1/f2 直接被覆写为 fcommon，照常完成配置，保证一定出波。 */
    if (en0 && en1 && en2) {
        int32_t fcommon;
        int c1_in = (freq1 >= 2000000) && (freq1 <= MS5352ME_MAX_FREQ_CLK12);
        int c2_in = (freq2 >= 2000000) && (freq2 <= MS5352ME_MAX_FREQ_CLK12);
        if (c1_in)                                fcommon = f1;   // 优先 CLK1 有效频率
        else if (c2_in)                           fcommon = f2;   // 否则 CLK2 有效频率
        else {                                                  // 都无效：取 CLK1 最近边界
            if (freq1 >= MS5352ME_MAX_FREQ_CLK12) fcommon = MS5352ME_MAX_FREQ_CLK12;
            else                                  fcommon = 2000000;
        }
        if (fcommon < 2000000)                       fcommon = 2000000;
        else if (fcommon > MS5352ME_MAX_FREQ_CLK12) fcommon = MS5352ME_MAX_FREQ_CLK12;
        f1 = fcommon;
        f2 = fcommon;
    }

    /* ================= 写寄存器 ================= */
    uint8_t usedPllA = 0, usedPllB = 0;

    /* CLK0 -> DIV0（PLLA） */
    if (en0) {
        ms5352PLLConfig_t pll0;
        int32_t P1=0,P2=0,P3=1; uint8_t divBy4=0, rdiv0=0, isInt=0;
        ms5352me_CalcDiv0(f0, &pll0, &P1, &P2, &P3, &divBy4, &rdiv0, &isInt);
        ms5352me_WriteDivider(ms5352me_REGISTER_42_DIV0_PARAMETERS, P1, P2, P3, divBy4, 0, rdiv0);
        ms5352me_writePLL(ms5352me_REGISTER_26_PLLA_PARAMETERS, &pll0);
        usedPllA = 1;
        ms5352me_WriteClkControl(0, MS5352ME_DIV0, ms5352me_PLL_A, sd0, isInt, 0);
    } else {
        ms5352me_write(ms5352me_REGISTER_16_CLK0_CONTROL, 0x80);
    }

    /* CLK1 -> DIV1（固定/2，PLL 见分配） */
    if (en1) {
        ms5352PLLConfig_t pll1cfg; uint8_t rdiv1 = 0;
        ms5352me_CalcDiv2(f1, &pll1cfg, &rdiv1);
				ms5352me_WriteDivider(ms5352me_REGISTER_50_DIV1_PARAMETERS, 0, 0, 1, 0, 0x3, rdiv1);
        ms5352me_writePLL((pll1 == ms5352me_PLL_B) ? ms5352me_REGISTER_34_PLLB_PARAMETERS
                                               : ms5352me_REGISTER_26_PLLA_PARAMETERS, &pll1cfg);
        if (pll1 == ms5352me_PLL_B) usedPllB = 1; else usedPllA = 1;
        ms5352me_WriteClkControl(1, MS5352ME_DIV1, pll1, sd1, 1, 0);
    } else {
        ms5352me_write(ms5352me_REGISTER_17_CLK1_CONTROL, 0x80);
    }

    /* CLK2 -> DIV2（固定/2，PLL 见分配） */
    if (en2) {
        ms5352PLLConfig_t pll2cfg; uint8_t rdiv2 = 0;
        ms5352me_CalcDiv2(f2, &pll2cfg, &rdiv2);
        ms5352me_WriteDivider(ms5352me_REGISTER_58_DIV2_PARAMETERS, 0, 0, 1, 0, 0x3, rdiv2);
        ms5352me_writePLL((pll2 == ms5352me_PLL_B) ? ms5352me_REGISTER_34_PLLB_PARAMETERS
                                               : ms5352me_REGISTER_26_PLLA_PARAMETERS, &pll2cfg);
        if (pll2 == ms5352me_PLL_B) usedPllB = 1; else usedPllA = 1;
        ms5352me_WriteClkControl(2, MS5352ME_DIV2, pll2, sd2, 1, 0);
    } else {
        ms5352me_write(ms5352me_REGISTER_18_CLK2_CONTROL, 0x80);
    }

    uint8_t reg3 = (en0 ? 0 : 1) | (en1 ? 0 : 2) | (en2 ? 0 : 4);
    ms5352me_write(ms5352me_REGISTER_3_OUTPUT_ENABLE, reg3);

    uint8_t rst = 0;
    if (usedPllA) rst |= 0x20;
    if (usedPllB) rst |= 0x80;
    ms5352me_write(ms5352me_REGISTER_177_PLL_RESET, rst);

    /* 越界静默钳位，照常配置，无返回值 */
}
