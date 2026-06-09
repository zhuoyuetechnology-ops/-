/*----------------------------------------------------------------------------------------
  版本: v2.40 (低静态功耗优化版) - Keil C51 兼容版
  基于: v2.39 (预焊声音提前到触发瞬间，预焊30ms，主焊100ms)
  芯片: STC8H1K08-36I-TSSOP-20 / 主频: 35MHz / 电源: 3.2V~5.1V
  
  修复: C51 编译器兼容性（变量声明必须在代码块开头）
----------------------------------------------------------------------------------------*/
#include "STC8H.h"
#include <intrins.h>

#define MAIN_Fosc 35000000UL

// ======================== 功耗模式定义 ========================
#define POWER_MODE_SLEEP       0   // 深度睡眠 (~1mA)
#define POWER_MODE_STANDBY     1   // 待机 (~5mA)
#define POWER_MODE_IDLE        2   // 空闲 (~15mA)
#define POWER_MODE_ACTIVE      3   // 工作 (~35mA)

#define FREQ_MODE_SLEEP        0   // 0.5MHz (PLL 关闭)
#define FREQ_MODE_LOW          1   // 1MHz
#define FREQ_MODE_MID          2   // 5MHz
#define FREQ_MODE_HIGH         3   // 35MHz

// ADC 分压系数
#define ADC_DIV_NUM   147
#define ADC_DIV_DEN   47

// 引脚定义
#define PIN_12  0x04
#define PIN_13  0x08
#define PIN_14  0x10
#define PIN_15  0x20
#define PIN_16  0x40
#define PIN_17  0x80
#define P1_LED_MASK  0xFC

sbit BEEP = P5^4;
sbit ENC_A = P1^2;
sbit ENC_B = P1^3;
sbit ENC_D = P1^4;
sbit TRIG_IN         = P3^4;
sbit DISCHARGE_CTRL  = P3^7;

#define ADC_CH_CAP    1

// EEPROM 地址
#define EE_SECTOR       0x0000
#define EE_ADDR_A       0x0000
#define EE_ADDR_B       0x0001
#define EE_ADDR_C       0x0002
#define EE_ADDR_D       0x0003
#define EE_ADDR_U       0x0004
#define EE_ADDR_CAL_H   0x0005
#define EE_ADDR_CAL_L   0x0006
#define EE_ADDR_CHK     0x0007

// 蜂鸣时长宏
#define BEEP_MS_INIT           100
#define BEEP_MS_NO_CAP         30
#define BEEP_MS_OVER_VOL       100
#define BEEP_MS_STATE_SWITCH   100
#define BEEP_MS_SHORT_PRESS    30
#define BEEP_MS_ENC_STEP       30
#define BEEP_MS_WELD_PRE       30
#define BEEP_MS_WELD_MAIN      100

// 参数阈值
#define NO_CAP_THRESHOLD_MV         100
#define POWER_ON_DELAY_MS           5000
#define ENC_DEBOUNCE_CNT_MAX        4
#define LONG_PRESS_MS               1500
#define SHORT_PRESS_MIN_MS          20
#define OVER_VOL_HYST_MV            40
#define OVER_VOL_RECOVER_MS         500
#define BEEP_LIMIT_MS               50
#define OVER_VOL_BEEP_INTERVAL      1000
#define IDLE_TIMEOUT_MS             2000
#define FAST_SHORT_GAP_MS           300
#define FAST_SHORT_COUNT            5

#define FILTER_WINDOW      16
#define LIMIT_STEP         100
#define OVER_VOL_CONFIRM_MS   1000
#define VCC_MEASURE_INTERVAL  5000
#define MAX_DISPLAY_VOLTAGE   6300
#define ADC_SAMPLE_TICKS      20
#define KEY_DEBOUNCE_CNT       5
#define KEY_SCAN_DIV          2

#define TRIG_HIGH_MS        300
#define TRIG_RELEASE_MS     300

// 动态滤波窗口
#define FAST_WINDOW       16
#define NORMAL_WINDOW     32
#define SLOW_WINDOW       64
#define DIFF_FAST         300
#define DIFF_MID          50

// 点焊阶段
#define WELD_IDLE           0
#define WELD_DELAY          1
#define WELD_PREWELD        2
#define WELD_GAP            3
#define WELD_MAIN           4
#define WELD_INTERVAL       5
#define WELD_DISPLAY_WAIT   6

// 结构体
typedef struct {
    unsigned char A;
    unsigned char B;
    unsigned char C;
    unsigned char D;
    unsigned char U;
} SysParams_t;

typedef struct {
    unsigned char no_cap;
    unsigned char over_vol;
    unsigned char fault_latch;
} FaultFlags_t;

typedef struct {
    unsigned char phase;
    unsigned char counter;
    unsigned long timer;
    unsigned char busy;
    unsigned long high_start;
    unsigned long low_start;
    unsigned char armed;
} WeldState_t;

typedef struct {
    unsigned char power_mode;
    unsigned char freq_mode;
    unsigned char disp_active;
    unsigned char adc_enabled;
    unsigned long sleep_until;
} PowerState_t;

// ========== 全局变量 ==========
xdata unsigned int filter_buf[64];

volatile unsigned int  idata beep_timer = 0;
volatile bit beepping = 0;

volatile unsigned int  idata vcc_mv = 5000;
volatile unsigned long idata key_block_until = 0;
volatile unsigned int  idata calib_factor = 1000;
volatile unsigned int  idata last_calib = 1000;

volatile SysParams_t idata params = {1, 1, 1, 1, 53};
volatile SysParams_t idata last_saved_params = {1, 1, 1, 1, 53};

volatile unsigned long idata g_millis = 0;
volatile unsigned char idata sys_state = 0;
volatile unsigned char idata set_item = 0;
volatile unsigned char idata cal_sub_state = 0;
volatile unsigned int  idata cal_target = 500;
volatile FaultFlags_t idata fault = {0, 0, 0};
volatile unsigned int  idata adc_voltage = 0;
volatile unsigned int  idata display_voltage = 0;
volatile unsigned char idata disp_buf[3] = {16,5,0};
volatile unsigned char idata disp_shadow[3] = {16,5,0};
volatile bit frame_ready = 0;
volatile unsigned char idata dp_flag[3] = {0, 1, 0};

volatile PowerState_t idata power_state = {POWER_MODE_STANDBY, FREQ_MODE_LOW, 0, 0, 0};
volatile unsigned char idata idle_counter = 0;
volatile unsigned char idata adc_skip_count = 0;
volatile bit wakeup_pending = 0;

code unsigned char seg7table[24] = {
    0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,
    0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
    0x3E, 0x1C, 0x79, 0x50,
    0x5E,0x76,0x38,0x40
};

code unsigned char seg_anode[24] = {
    PIN_17,PIN_16,PIN_16,PIN_17,PIN_17,PIN_17,PIN_17,PIN_16,
    PIN_15,PIN_15,PIN_15,PIN_15,PIN_16,PIN_16,PIN_15,PIN_14,
    PIN_14,PIN_13,PIN_13,PIN_14,PIN_14,PIN_14,PIN_13,PIN_13
};

code unsigned char seg_cathode[24] = {
    PIN_16,PIN_17,PIN_15,PIN_15,PIN_14,PIN_13,PIN_12,PIN_14,
    PIN_17,PIN_16,PIN_14,PIN_13,PIN_13,PIN_12,PIN_12,PIN_17,
    PIN_16,PIN_16,PIN_15,PIN_15,PIN_13,PIN_12,PIN_17,PIN_14
};

volatile unsigned char idata scan_index = 0;
volatile unsigned char idata millis_accum = 0;

volatile unsigned char idata key_short_flag = 0;
volatile unsigned char idata key_long_flag  = 0;
volatile signed   int  idata enc_accum = 0;

volatile unsigned char idata key_state = 0;
volatile unsigned long idata key_press_time = 0;
volatile unsigned char idata key_long_done = 0;
volatile unsigned char idata key_debounce_cnt = 0;
volatile unsigned char idata key_last_raw = 0;

volatile unsigned char idata enc_last_state = 0;
volatile unsigned char idata enc_current_state = 0;
volatile unsigned char idata enc_debounce_cnt = 0;
volatile signed char   idata enc_step_count = 0;

volatile unsigned long idata last_display_mv = 0;
volatile bit disp_first = 1;

volatile unsigned long idata no_cap_start = 0;
volatile unsigned long idata over_vol_start = 0;
volatile unsigned long idata over_vol_recover_start = 0;

volatile unsigned long idata power_on_time = 0;
volatile bit power_on_delay_done = 0;
volatile bit params_dirty = 0;
volatile bit save_pending = 0;

volatile unsigned char idata filter_head = 0;
volatile unsigned char idata filter_cnt = 0;
volatile unsigned long idata filter_sum = 0;
volatile unsigned int  idata last_valid = 0;
volatile unsigned char idata current_window = NORMAL_WINDOW;

volatile unsigned int  idata new_raw_mv = 0;
volatile bit adc_new_data = 0;
volatile bit adc_busy = 0;

volatile bit vcc_need_measure = 0;
volatile unsigned long idata last_vcc_time = 0;
volatile unsigned char idata last_disp_10x = 0;

volatile WeldState_t idata weld = {0, 0, 0, 0, 0, 0, 1};
volatile unsigned long idata last_over_vol_beep = 0;
volatile unsigned long idata last_action_time = 0;

// 函数声明
void IapIdle(void);
void IapInit(void);
unsigned char IapRead(unsigned int addr);
void IapEraseSector(unsigned int addr);
void IapWrite(unsigned int addr, unsigned char dat);
unsigned char calc_chk(unsigned char *vals);
void save_params(void);
void load_params(void);
void beep_ms(unsigned char ms);
void Timer0_Init(void);
void Timer1_Init(void);
void ADC_Init(void);
void ADC_DeInit(void);
unsigned int ADC_ReadFast(unsigned char ch);
unsigned int Get_VCC(void);
void ProcessADCFilter(void);
void update_display_shadow(void);
void update_display_set(void);
void update_display_u_set(void);
void adjust_param(signed char delta);
void sync_encoder_state(void);
void set_power_mode(unsigned char mode);
void set_cpu_freq(unsigned char freq_mode);
void update_power_state(void);

// ======================== 功率管理函数 ========================
void set_cpu_freq(unsigned char freq_mode) {
    unsigned char clkdiv;
    power_state.freq_mode = freq_mode;
    
    switch(freq_mode) {
        case FREQ_MODE_SLEEP:
            clkdiv = 0x70;
            break;
        case FREQ_MODE_LOW:
            clkdiv = 0x38;
            break;
        case FREQ_MODE_MID:
            clkdiv = 0x14;
            break;
        case FREQ_MODE_HIGH:
        default:
            clkdiv = 0x00;
            break;
    }
    P_SW2 |= 0x80;
    CLKDIV = clkdiv;
    P_SW2 &= ~0x80;
}

void set_power_mode(unsigned char mode) {
    power_state.power_mode = mode;
    
    switch(mode) {
        case POWER_MODE_SLEEP:
            power_state.adc_enabled = 0;
            power_state.disp_active = 0;
            set_cpu_freq(FREQ_MODE_SLEEP);
            break;
            
        case POWER_MODE_STANDBY:
            power_state.adc_enabled = 0;
            power_state.disp_active = 0;
            set_cpu_freq(FREQ_MODE_LOW);
            break;
            
        case POWER_MODE_IDLE:
            power_state.adc_enabled = 1;
            power_state.disp_active = 1;
            set_cpu_freq(FREQ_MODE_MID);
            break;
            
        case POWER_MODE_ACTIVE:
            power_state.adc_enabled = 1;
            power_state.disp_active = 1;
            set_cpu_freq(FREQ_MODE_HIGH);
            break;
    }
}

void update_power_state(void) {
    unsigned long now = g_millis;
    
    if(weld.busy) {
        if(power_state.power_mode != POWER_MODE_ACTIVE) {
            set_power_mode(POWER_MODE_ACTIVE);
        }
        return;
    }
    
    if(sys_state == 0) {
        if(power_state.power_mode != POWER_MODE_SLEEP) {
            set_power_mode(POWER_MODE_SLEEP);
        }
        return;
    }
    
    if(sys_state >= 1 && sys_state <= 3) {
        if(!power_on_delay_done) {
            if(power_state.power_mode != POWER_MODE_IDLE) {
                set_power_mode(POWER_MODE_IDLE);
            }
            return;
        }
        
        if(now - last_action_time >= IDLE_TIMEOUT_MS) {
            if(power_state.power_mode != POWER_MODE_STANDBY) {
                set_power_mode(POWER_MODE_STANDBY);
            }
        } else {
            if(power_state.power_mode != POWER_MODE_IDLE) {
                set_power_mode(POWER_MODE_IDLE);
            }
        }
    }
}

void ADC_DeInit(void) {
    P_SW2 |= 0x80;
    ADCCFG &= ~0x80;
    P_SW2 &= ~0x80;
}

// ======================== EEPROM 操作 ========================
void IapIdle() { IAP_CONTR=0; IAP_CMD=0; IAP_TRIG=0; IAP_ADDRH=0x80; IAP_ADDRL=0; }
void IapInit() { IAP_CONTR = 0; IAP_TPS = 35; _nop_(); }
unsigned char IapRead(unsigned int addr) {
    unsigned char dat;
    IAP_CONTR = 0xC7; IAP_CMD = 1; IAP_ADDRH = addr>>8; IAP_ADDRL = addr&0xFF;
    IAP_TRIG = 0x5A; IAP_TRIG = 0xA5; _nop_();
    dat = IAP_DATA; IapIdle(); return dat;
}
void IapEraseSector(unsigned int addr) {
    IAP_CONTR = 0xC7; IAP_CMD = 3; IAP_ADDRH = addr>>8; IAP_ADDRL = addr&0xFF;
    IAP_TRIG = 0x5A; IAP_TRIG = 0xA5; _nop_(); IapIdle();
}
void IapWrite(unsigned int addr, unsigned char dat) {
    IAP_CONTR = 0xC7; IAP_CMD = 2; IAP_ADDRH = addr>>8; IAP_ADDRL = addr&0xFF;
    IAP_DATA = dat; IAP_TRIG = 0x5A; IAP_TRIG = 0xA5; _nop_(); IapIdle();
}
unsigned char calc_chk(unsigned char *vals) {
    return vals[0]^vals[1]^vals[2]^vals[3]^vals[4]^vals[5]^vals[6]^0xA5;
}

void save_params(void) {
    unsigned char buf[7], chk;
    if (!params_dirty) return;

    if (params.A == last_saved_params.A &&
        params.B == last_saved_params.B &&
        params.C == last_saved_params.C &&
        params.D == last_saved_params.D &&
        params.U == last_saved_params.U &&
        calib_factor == last_calib)
    {
        params_dirty = 0;
        return;
    }

    buf[0]=params.A; buf[1]=params.B; buf[2]=params.C; buf[3]=params.D;
    buf[4]=params.U;
    buf[5]=calib_factor>>8; buf[6]=calib_factor&0xFF;
    chk=calc_chk(buf);
    EA=0;
    IapEraseSector(EE_SECTOR);
    IapWrite(EE_ADDR_A,buf[0]); IapWrite(EE_ADDR_B,buf[1]); IapWrite(EE_ADDR_C,buf[2]);
    IapWrite(EE_ADDR_D,buf[3]); IapWrite(EE_ADDR_U,buf[4]);
    IapWrite(EE_ADDR_CAL_H,buf[5]); IapWrite(EE_ADDR_CAL_L,buf[6]);
    IapWrite(EE_ADDR_CHK,chk);
    EA=1;
    last_saved_params = params;
    last_calib = calib_factor;
    params_dirty = 0;
}

void load_params(void) {
    unsigned char buf[7], chk, i;
    EA=0;
    for(i=0;i<7;i++) buf[i]=IapRead(EE_SECTOR+i);
    chk=IapRead(EE_ADDR_CHK);
    EA=1;
    if(chk==calc_chk(buf) && buf[0]>=1 && buf[0]<=10 && buf[1]>=1 && buf[1]<=99 &&
       buf[2]>=1 && buf[2]<=3 && buf[3]>=1 && buf[3]<=20 && buf[4]>=40 && buf[4]<=60) {
        params.A=buf[0]; params.B=buf[1]; params.C=buf[2]; params.D=buf[3]; params.U=buf[4];
        calib_factor = ((unsigned int)buf[5]<<8) | buf[6];
    } else {
        params.A=1; params.B=1; params.C=1; params.D=1; params.U=53;
        calib_factor = 1000;
    }
    last_saved_params = params;
    last_calib = calib_factor;
    params_dirty = 0;
}

// ======================== 蜂鸣器驱动 ========================
void beep_ms(unsigned char ms) {
    EA = 0;
    beep_timer = ms;
    beepping = 1;
    BEEP = 0;
    TR1 = 1;
    EA = 1;
    if(power_state.power_mode < POWER_MODE_IDLE) {
        set_power_mode(POWER_MODE_IDLE);
    }
}

void timer1_isr(void) interrupt 3 {
    BEEP = !BEEP;
}

// ======================== ADC 初始化 ========================
void ADC_Init(void) {
    unsigned char i;
    CMPCR1=0; CMPCR2=0;
    P1M1|=0x02; P1M0&=~0x02; P1IE&=~0x02;
    P_SW2|=0x80; ADCTIM=0x3F; ADCCFG=0x2F; P_SW2&=~0x80;
    ADC_CONTR=0x80;
    for(i=0;i<10;i++) {
        ADC_CONTR&=0xF0; ADC_CONTR|=ADC_CH_CAP|0x40;
        _nop_();_nop_();_nop_();_nop_();
        while(!(ADC_CONTR&0x20)); ADC_CONTR&=~0x20;
    }
}

unsigned int ADC_ReadFast(unsigned char ch) {
    unsigned int res;
    ADC_CONTR &= 0xF0;
    ADC_CONTR |= ch | 0x40;
    _nop_(); _nop_(); _nop_(); _nop_();
    while(!(ADC_CONTR & 0x20));
    res = (ADC_RES << 8) | ADC_RESL;
    ADC_CONTR &= ~0x20;
    return res;
}

unsigned int Get_VCC(void) {
    unsigned int adc_bg; unsigned long vcc; unsigned char old=ADC_CONTR&0x0F;
    ADC_CONTR&=0xF0; ADC_CONTR|=15|0x40;
    _nop_();_nop_();_nop_();_nop_();
    while(!(ADC_CONTR&0x20)); adc_bg=(ADC_RES<<8)|ADC_RESL; ADC_CONTR&=~0x20;
    ADC_CONTR=(ADC_CONTR&0xF0)|old;
    vcc=(unsigned long)1192*4096/adc_bg;
    return (unsigned int)vcc;
}

void ProcessADCFilter(void) {
    unsigned int raw, limited, diff, last_filtered;
    unsigned char i;
    
    if(!power_state.adc_enabled) return;
    
    EA=0; raw = new_raw_mv; EA=1;

    if(disp_first) {
        limited = raw;
        last_valid = raw;
        for(i = 0; i < current_window; i++) {
            filter_buf[i] = raw;
        }
        filter_head = 0;
        filter_cnt = current_window;
        filter_sum = (unsigned long)raw * current_window;
        disp_first = 0;
    } else {
        if(filter_cnt == 0) last_filtered = raw;
        else last_filtered = (unsigned int)(filter_sum / filter_cnt);
        diff = (raw > last_filtered) ? (raw - last_filtered) : (last_filtered - raw);
        if(diff > DIFF_FAST) current_window = FAST_WINDOW;
        else if(diff > DIFF_MID) current_window = NORMAL_WINDOW;
        else current_window = SLOW_WINDOW;

        if(filter_cnt > current_window) {
            filter_cnt = 0; filter_sum = 0; filter_head = 0;
        }
        if(diff > LIMIT_STEP) limited = raw;
        else limited = (last_valid + raw) >> 1;
        last_valid = limited;

        if(filter_cnt < current_window) {
            filter_buf[filter_cnt] = limited;
            filter_sum += limited;
            filter_cnt++;
        } else {
            filter_sum -= filter_buf[filter_head];
            filter_buf[filter_head] = limited;
            filter_sum += limited;
            filter_head = (filter_head + 1) % current_window;
        }
    }

    if(filter_cnt > 0)
        adc_voltage = (unsigned int)(filter_sum / filter_cnt);
    else
        adc_voltage = limited;

    if(calib_factor != 0)
        display_voltage = (unsigned int)(((unsigned long)adc_voltage * calib_factor + 500) / 1000);
    else
        display_voltage = adc_voltage;
}

void update_display_shadow(void) {
    unsigned int mv;
    unsigned char new_buf[3], raw_10x, next_disp;
    unsigned int threshold;
    static unsigned char trend_cnt = 0;
    static signed char   pending_dir = 0;

    if(weld.busy) {
        disp_shadow[0] = 23; disp_shadow[1] = 23; disp_shadow[2] = 23;
        dp_flag[0] = 0; dp_flag[1] = 0; dp_flag[2] = 0;
        trend_cnt = 0; pending_dir = 0;
        return;
    }

    if(sys_state == 1) {
        dp_flag[0] = 0; dp_flag[1] = 1; dp_flag[2] = 0;
    }

    mv = display_voltage;
    if(fault.no_cap) {
        new_buf[0] = 18; new_buf[1] = 0; new_buf[2] = 1;
    } else if(fault.over_vol) {
        new_buf[0] = 18; new_buf[1] = 0; new_buf[2] = 2;
    } else {
        if(mv > MAX_DISPLAY_VOLTAGE) mv = MAX_DISPLAY_VOLTAGE;
        raw_10x = mv / 100;
        if(raw_10x > 63) raw_10x = 63;

        if(raw_10x > last_disp_10x) {
            next_disp = last_disp_10x + 1;
            threshold = (unsigned int)next_disp * 100 - 10;
            if(mv >= threshold) {
                if(pending_dir == 1) {
                    trend_cnt++;
                    if(trend_cnt >= 4) {
                        last_disp_10x = next_disp;
                        trend_cnt = 0;
                        pending_dir = 0;
                    }
                } else {
                    pending_dir = 1;
                    trend_cnt = 1;
                }
            } else {
                pending_dir = 0;
                trend_cnt = 0;
            }
        } else if(raw_10x < last_disp_10x) {
            next_disp = last_disp_10x - 1;
            threshold = (unsigned int)last_disp_10x * 100 - 10;
            if(mv < threshold) {
                if(pending_dir == -1) {
                    trend_cnt++;
                    if(trend_cnt >= 4) {
                        last_disp_10x = next_disp;
                        trend_cnt = 0;
                        pending_dir = 0;
                    }
                } else {
                    pending_dir = -1;
                    trend_cnt = 1;
                }
            } else {
                pending_dir = 0;
                trend_cnt = 0;
            }
        } else {
            pending_dir = 0;
            trend_cnt = 0;
        }

        new_buf[0] = 16;
        new_buf[1] = last_disp_10x / 10;
        new_buf[2] = last_disp_10x % 10;
    }
    disp_shadow[0]=new_buf[0]; disp_shadow[1]=new_buf[1]; disp_shadow[2]=new_buf[2];
}

void update_display_set(void) {
    unsigned char val;
    switch(set_item){
        case 0:val=params.A;break; case 1:val=params.B;break;
        case 2:val=params.C;break; case 3:val=params.D;break;
        default:val=0;
    }
    if(set_item<3) disp_shadow[0]=10+set_item; else disp_shadow[0]=20;
    disp_shadow[1]=val/10; disp_shadow[2]=val%10;
}

void update_display_u_set(void) {
    if(cal_sub_state == 0) {
        disp_shadow[0]=17; disp_shadow[1]=params.U/10; disp_shadow[2]=params.U%10;
        dp_flag[0]=0; dp_flag[1]=1; dp_flag[2]=0;
    } else if(cal_sub_state == 1) {
        disp_shadow[0] = 12;
        disp_shadow[1] = 22;
        disp_shadow[2] = 16;
        dp_flag[0] = 0; dp_flag[1] = 0; dp_flag[2] = 0;
    } else if(cal_sub_state == 2) {
        disp_shadow[0] = cal_target/100;
        disp_shadow[1] = (cal_target/10)%10;
        disp_shadow[2] = cal_target%10;
        dp_flag[0]=1; dp_flag[1]=0; dp_flag[2]=0;
    }
}

void adjust_param(signed char delta) {
    if(delta==0) return;
    if(sys_state==3){
        if(cal_sub_state == 2) {
            int new_val = (int)cal_target + delta;
            if(new_val < 100) new_val = 100;
            if(new_val > 630) new_val = 630;
            cal_target = (unsigned int)new_val;
            return;
        } else if(cal_sub_state == 0) {
            unsigned char old = params.U;
            if(delta>0) { if(params.U<60) params.U++; } else { if(params.U>40) params.U--; }
            if(params.U!=old) params_dirty=1;
            return;
        }
    }
    if(sys_state==2){
        unsigned char *p, min, max, old;
        switch(set_item){
            case 0:p=&params.A;min=1;max=10;break;
            case 1:p=&params.B;min=1;max=99;break;
            case 2:p=&params.C;min=1;max=3;break;
            case 3:p=&params.D;min=1;max=20;break;
            default:return;
        }
        old=*p;
        if(delta>0){ if(*p<max) (*p)++; else *p=min; }
        else { if(*p>min) (*p)--; else *p=max; }
        if(*p!=old) params_dirty=1;
    }
}

void sync_encoder_state(void) {
    enc_last_state=((ENC_A?1:0)<<1)|(ENC_B?1:0);
    enc_current_state=enc_last_state; enc_debounce_cnt=ENC_DEBOUNCE_CNT_MAX;
    enc_accum=0; enc_step_count=0;
}

void Timer0_Init(void) {
    AUXR|=0x80; TMOD&=0xF0; TMOD|=0x00;
    TL0=0xED; TH0=0xFF;
    TR0=1; ET0=1;
}

void Timer1_Init(void) {
    AUXR |= 0x40;
    TMOD &= 0x0F;
    TMOD |= 0x00;
    TH1 = 0xEE;
    TL1 = 0xF1;
    ET1 = 1;
    PT1 = 1;
    TR1 = 0;
}

/* T0 中断 - 修复 C51 兼容性 */
void timer0_isr(void) interrupt 1 {
    unsigned char key_raw, a_now, b_now, state;
    unsigned char dig, seg, ch, segcode, light, anode, cathode;
    static unsigned int  adc_tick_local = 0;
    static unsigned char key_sample_cnt = 0;
    static unsigned char enc_sample_cnt = 0;
    static unsigned char vcc_tick = 0;
    static unsigned char scan_delay_cnt = 0;
    unsigned int raw_val;
    unsigned long raw_mv_calc;

    /* 关闭所有段 */
    P1M1|=P1_LED_MASK; P1M0&=~P1_LED_MASK; P1=0;

    /* 按键消抖 */
    if(++key_sample_cnt >= 2){
        key_sample_cnt = 0;
        key_raw = ENC_D ? 1 : 0;
        if(key_raw == key_last_raw){
            if(key_debounce_cnt < KEY_DEBOUNCE_CNT) key_debounce_cnt++;
            if(key_debounce_cnt == KEY_DEBOUNCE_CNT){
                if(key_raw == 1){
                    if(key_state == 0){
                        key_state = 1;
                        key_press_time = 0;
                        key_long_done = 0;
                        wakeup_pending = 1;
                    }
                } else {
                    if(key_state == 1) key_state = 0;
                }
            }
        } else {
            key_debounce_cnt = 0;
        }
        key_last_raw = key_raw;
    }

    /* 编码器解码 */
    if(++enc_sample_cnt >= 2){
        enc_sample_cnt = 0;
        if(power_state.power_mode >= POWER_MODE_IDLE && (sys_state == 2 || sys_state == 3)){
            a_now = ENC_A ? 1 : 0;
            b_now = ENC_B ? 1 : 0;
            state = (a_now << 1) | b_now;
            if(state != enc_current_state){
                enc_current_state = state;
                enc_debounce_cnt = 1;
            } else {
                if(enc_debounce_cnt < ENC_DEBOUNCE_CNT_MAX) enc_debounce_cnt++;
                if(enc_debounce_cnt == ENC_DEBOUNCE_CNT_MAX){
                    if(state != enc_last_state){
                        signed char dir = 0;
                        if((enc_last_state == 0x00 && state == 0x01) || (enc_last_state == 0x01 && state == 0x03) ||
                           (enc_last_state == 0x03 && state == 0x02) || (enc_last_state == 0x02 && state == 0x00)) dir = -1;
                        else if((enc_last_state == 0x00 && state == 0x02) || (enc_last_state == 0x02 && state == 0x03) ||
                                (enc_last_state == 0x03 && state == 0x01) || (enc_last_state == 0x01 && state == 0x00)) dir = 1;
                        if(dir != 0){
                            enc_accum += dir;
                            while(enc_accum >= 4){ enc_step_count++; enc_accum -= 4; }
                            while(enc_accum <= -4){ enc_step_count--; enc_accum += 4; }
                        }
                        enc_last_state = state;
                    }
                    enc_debounce_cnt = ENC_DEBOUNCE_CNT_MAX;
                }
            }
        }
    }

    /* ADC 采样 */
    if(power_state.adc_enabled) {
        unsigned char adc_interval = (power_state.power_mode == POWER_MODE_ACTIVE) ? ADC_SAMPLE_TICKS : (ADC_SAMPLE_TICKS * 2);
        if(++adc_tick_local >= adc_interval){
            adc_tick_local = 0;
            raw_val = ADC_ReadFast(ADC_CH_CAP);
            raw_mv_calc = (unsigned long)raw_val * vcc_mv * ADC_DIV_NUM / (4096UL * ADC_DIV_DEN);
            if(raw_mv_calc > MAX_DISPLAY_VOLTAGE) raw_mv_calc = MAX_DISPLAY_VOLTAGE;
            new_raw_mv = (unsigned int)raw_mv_calc;
            adc_new_data = 1;
        }
    }

    /* 毫秒计时 */
    if(++millis_accum >= 4){
        millis_accum = 0;
        g_millis++;
        
        if(key_state == 1){
            key_press_time++;
            if(key_press_time >= LONG_PRESS_MS && !key_long_done){
                key_long_done = 1;
                key_long_flag = 1;
            }
        } else {
            if(key_press_time >= SHORT_PRESS_MIN_MS && key_press_time < LONG_PRESS_MS)
                key_short_flag = 1;
            key_press_time = 0;
        }

        /* 蜂鸣器计时 */
        if(beepping && beep_timer > 0){
            beep_timer--;
            if(beep_timer == 0){
                beepping = 0;
                TR1 = 0;
                BEEP = 0;
            }
        }

        if(++vcc_tick >= 250){
            vcc_tick = 0;
            vcc_need_measure = 1;
        }
    }

    /* Charlieplexing 扫描 - 所有变量已在函数开头声明 */
    if(power_state.disp_active && ++scan_delay_cnt >= 1) {
        scan_delay_cnt = 0;
        dig = scan_index >> 3;
        seg = scan_index & 0x07;
        ch = disp_buf[dig];
        segcode = (ch <= 23) ? seg7table[ch] : 0;
        light = 0;
        if(seg == 7){
            if((fault.no_cap || fault.over_vol) && sys_state != 3) light = 0;
            else if(dig < 3 && dp_flag[dig]) light = 1;
        } else {
            light = (segcode >> seg) & 0x01;
        }
        if(light && sys_state != 0){
            anode = seg_anode[scan_index];
            cathode = seg_cathode[scan_index];
            P1M1 &= ~(anode | cathode);
            P1M0 |= (anode | cathode);
            P1 = anode;
        }
        if(++scan_index >= 24){
            scan_index = 0;
            frame_ready = 1;
        }
    }
}

void main(void) {
    unsigned char l_short, l_long, i;
    unsigned long now;
    static unsigned long last_short_time = 0;
    static unsigned char seq_count = 0;
    static unsigned long last_beep_ms = 0;
    signed char enc_steps;
    unsigned char beep_busy;

    P1M1 |= P1_LED_MASK; P1M0 &= ~P1_LED_MASK; P1 = 0;
    P5M1 &= ~0x10; P5M0 |= 0x10; BEEP = 0;
    P3M1 |= 0x10; P3M0 &= ~0x10; P3M1 &= ~0x80; P3M0 |= 0x80; DISCHARGE_CTRL = 0;

    IapInit(); IapIdle(); ADC_Init();
    Timer0_Init();
    Timer1_Init();
    load_params();

    filter_head = 0; filter_cnt = 0; filter_sum = 0;
    for(i=0;i<64;i++) filter_buf[i] = 0;
    adc_voltage = 0; display_voltage = 0; new_raw_mv = 0;
    fault.no_cap = 0; fault.over_vol = 0; fault.fault_latch = 0;
    no_cap_start = 0; over_vol_start = 0; over_vol_recover_start = 0;
    disp_first = 1; power_on_delay_done = 0;
    dp_flag[0] = 0; dp_flag[1] = 1; dp_flag[2] = 0;
    last_disp_10x = 0;

    update_display_shadow(); for(i=0;i<3;i++) disp_buf[i] = disp_shadow[i];

    set_power_mode(POWER_MODE_STANDBY);

    EA = 1;
    beep_ms(BEEP_MS_INIT);
    key_block_until = 0;
    save_pending = 0;
    weld.phase = WELD_IDLE; weld.busy = 0; weld.armed = 1;
    weld.high_start = 0; weld.low_start = 0;
    cal_sub_state = 0;
    last_action_time = 0;

    while(1) {
        l_short = key_short_flag; key_short_flag = 0;
        l_long  = key_long_flag;  key_long_flag  = 0;
        EA=0;
        now = g_millis;
        EA=1;

        if(now < key_block_until) { l_short = 0; l_long = 0; }

        beep_busy = beepping || (beep_timer != 0);
        if(save_pending && !beep_busy) { save_pending = 0; save_params(); }

        if(vcc_need_measure && (now - last_vcc_time >= VCC_MEASURE_INTERVAL)) {
            vcc_need_measure = 0; last_vcc_time = now;
            vcc_mv = Get_VCC();
        }

        if(adc_new_data) {
            adc_new_data = 0;
            ProcessADCFilter();
            if(sys_state == 1 || sys_state == 0) update_display_shadow();
            if(sys_state == 3 && cal_sub_state == 2) update_display_u_set();
        }

        if(frame_ready) {
            frame_ready = 0;
            for(i=0;i<3;i++) disp_buf[i] = disp_shadow[i];
        }

        if(sys_state == 2 || sys_state == 3) {
            DISCHARGE_CTRL = 0;
            weld.armed = 0;
            weld.high_start = 0; weld.low_start = 0;
        }

        // 触发与焊接
        if(!weld.busy && !fault.fault_latch) {
            unsigned char p34 = TRIG_IN ? 1 : 0;
            if(p34) {
                if(weld.high_start == 0) weld.high_start = now;
                else if((now - weld.high_start) >= TRIG_HIGH_MS) {
                    if(weld.armed) {
                        weld.armed = 0;
                        weld.high_start = 0;
                        weld.low_start = 0;
                        weld.phase = WELD_DELAY;
                        weld.counter = 0;
                        weld.timer = now + (unsigned long)params.D * 100;
                        weld.busy = 1;
                        DISCHARGE_CTRL = 0;
                        beep_ms(BEEP_MS_WELD_PRE);
                    }
                }
            } else {
                weld.high_start = 0;
                if(weld.low_start == 0) weld.low_start = now;
                else if((now - weld.low_start) >= TRIG_RELEASE_MS) {
                    weld.armed = 1;
                }
            }
        }
        else if(weld.busy) {
            unsigned long remain;
            remain = (weld.timer > now) ? (weld.timer - now) : 0;
            switch(weld.phase) {
                case WELD_DELAY:
                    if(remain == 0) {
                        weld.phase = WELD_PREWELD; weld.timer = now + params.A;
                        DISCHARGE_CTRL = 1;
                    } break;
                case WELD_PREWELD:
                    if(remain == 0) {
                        DISCHARGE_CTRL = 0; weld.phase = WELD_GAP; weld.timer = now + 2;
                    } break;
                case WELD_GAP:
                    if(remain == 0) {
                        weld.phase = WELD_MAIN; weld.timer = now + params.B;
                        DISCHARGE_CTRL = 1; weld.counter++;
                        beep_ms(BEEP_MS_WELD_MAIN);
                    } break;
                case WELD_MAIN:
                    if(remain == 0) {
                        DISCHARGE_CTRL = 0;
                        if(weld.counter >= params.C) {
                            weld.phase = WELD_DISPLAY_WAIT;
                            weld.timer = now + 300;
                        } else {
                            weld.phase = WELD_INTERVAL; weld.timer = now + 2;
                        }
                    } break;
                case WELD_INTERVAL:
                    if(remain == 0) {
                        weld.phase = WELD_MAIN; weld.timer = now + params.B;
                        DISCHARGE_CTRL = 1; weld.counter++;
                        beep_ms(BEEP_MS_WELD_MAIN);
                    } break;
                case WELD_DISPLAY_WAIT:
                    if(remain == 0) {
                        weld.busy = 0; weld.phase = WELD_IDLE;
                        weld.high_start = 0; weld.low_start = 0;
                        last_disp_10x = display_voltage / 100;
                        if(last_disp_10x > 63) last_disp_10x = 63;
                        update_display_shadow();
                    } break;
            }
        }

        // 编码器调整
        if(!weld.busy) {
            enc_steps = enc_step_count; enc_step_count = 0;
            if(enc_steps) {
                while(enc_steps > 0) { adjust_param(1); enc_steps--; }
                while(enc_steps < 0) { adjust_param(-1); enc_steps++; }
                if(now - last_beep_ms >= BEEP_LIMIT_MS) {
                    beep_ms(BEEP_MS_ENC_STEP);
                    last_beep_ms = now;
                }
                key_block_until = now + 300;
                last_action_time = now;
                if(sys_state == 2) update_display_set();
                else if(sys_state == 3) update_display_u_set();
            }
        } else {
            enc_step_count = 0; enc_accum = 0;
        }

        // 故障检测
        if(sys_state == 1 && !power_on_delay_done) {
            if(now - power_on_time >= POWER_ON_DELAY_MS) {
                power_on_delay_done = 1;
                if(adc_voltage < NO_CAP_THRESHOLD_MV) {
                    fault.no_cap = 1;
                    beep_ms(BEEP_MS_NO_CAP);
                }
            }
        }

        if(sys_state != 0 && power_on_delay_done && !weld.busy) {
            unsigned int limit_mv = (unsigned int)params.U * 100 + 100;
            if(adc_voltage >= limit_mv) {
                if(!fault.over_vol) {
                    if(over_vol_start == 0) over_vol_start = now;
                    else if(now - over_vol_start >= OVER_VOL_CONFIRM_MS) {
                        fault.over_vol = 1;
                        over_vol_recover_start = 0;
                    }
                }
            } else {
                over_vol_start = 0;
                if(fault.over_vol) {
                    if(adc_voltage <= limit_mv - OVER_VOL_HYST_MV) {
                        if(over_vol_recover_start == 0) over_vol_recover_start = now;
                        else if(now - over_vol_recover_start >= OVER_VOL_RECOVER_MS) {
                            fault.over_vol = 0; over_vol_recover_start = 0;
                            disp_first = 1; filter_cnt = 0; filter_sum = 0; filter_head = 0;
                            last_disp_10x = display_voltage / 100;
                            if(last_disp_10x > 63) last_disp_10x = 63;
                            vcc_mv = Get_VCC();
                        }
                    } else over_vol_recover_start = 0;
                }
            }

            if(adc_voltage < NO_CAP_THRESHOLD_MV) {
                if(!fault.no_cap) {
                    if(no_cap_start == 0) no_cap_start = now;
                    else if(now - no_cap_start >= 5000) {
                        fault.no_cap = 1;
                        beep_ms(BEEP_MS_NO_CAP);
                    }
                }
            } else {
                no_cap_start = 0;
                if(fault.no_cap) {
                    fault.no_cap = 0;
                    disp_first = 1; filter_cnt = 0; filter_sum = 0; filter_head = 0;
                    last_disp_10x = display_voltage / 100;
                    if(last_disp_10x > 63) last_disp_10x = 63;
                    vcc_mv = Get_VCC();
                }
            }
        }

        if(fault.over_vol && sys_state != 0) {
            if(now - last_over_vol_beep >= OVER_VOL_BEEP_INTERVAL) {
                beep_ms(BEEP_MS_OVER_VOL);
                last_over_vol_beep = now;
            }
        }

        // 状态机 + 功率管理
        if(!weld.busy) {
            switch(sys_state) {
                case 0:
                    if(l_long) {
                        sys_state = 1; fault.fault_latch = 0; fault.no_cap = 0; fault.over_vol = 0;
                        adc_voltage = 0; display_voltage = 0; no_cap_start = 0; over_vol_start = 0; over_vol_recover_start = 0;
                        disp_first = 1; filter_cnt = 0; filter_sum = 0; filter_head = 0;
                        vcc_mv = Get_VCC(); last_vcc_time = now;
                        power_on_time = now; power_on_delay_done = 0;
                        last_disp_10x = 0;
                        dp_flag[1] = 1; update_display_shadow(); beep_ms(BEEP_MS_STATE_SWITCH);
                        last_action_time = now; seq_count = 0; enc_step_count = 0;
                        set_power_mode(POWER_MODE_IDLE);
                    } break;
                case 1:
                    if(l_long) {
                        sys_state = 0; save_pending = 1; beep_ms(BEEP_MS_STATE_SWITCH); seq_count = 0; enc_step_count = 0;
                        set_power_mode(POWER_MODE_SLEEP);
                    }
                    if(l_short) {
                        sys_state = 2; set_item = 0; dp_flag[1] = 0; update_display_set(); beep_ms(BEEP_MS_SHORT_PRESS);
                        sync_encoder_state(); last_action_time = now; last_short_time = now; seq_count = 1;
                        set_power_mode(POWER_MODE_IDLE);
                    } break;
                case 2:
                    if(l_short) {
                        unsigned long dt = now - last_short_time;
                        if(dt < FAST_SHORT_GAP_MS) {
                            seq_count++;
                            if(seq_count >= FAST_SHORT_COUNT) {
                                sys_state = 3; cal_sub_state = 0; dp_flag[1] = 1; update_display_u_set();
                                beep_ms(BEEP_MS_SHORT_PRESS); sync_encoder_state(); seq_count = 0;
                            } else beep_ms(BEEP_MS_SHORT_PRESS);
                        } else {
                            seq_count = 1; set_item++; if(set_item > 3) set_item = 0;
                            update_display_set(); beep_ms(BEEP_MS_SHORT_PRESS);
                        }
                        last_short_time = now; last_action_time = now;
                    }
                    if(now - last_action_time >= IDLE_TIMEOUT_MS) {
                        sys_state = 1; dp_flag[1] = 1; update_display_shadow(); save_pending = 1; seq_count = 0; sync_encoder_state();
                        set_power_mode(POWER_MODE_STANDBY);
                    } break;
                case 3:
                    if(l_short) {
                        if(cal_sub_state == 0) { cal_sub_state = 1; update_display_u_set(); }
                        else if(cal_sub_state == 1) {
                            cal_sub_state = 2;
                            cal_target = (display_voltage / 10);
                            if(cal_target < 100) cal_target = 100;
                            if(cal_target > 630) cal_target = 630;
                            update_display_u_set();
                        } else if(cal_sub_state == 2) {
                            if(adc_voltage != 0) {
                                unsigned long new_factor = ((unsigned long)cal_target * 10 * 1000) / adc_voltage;
                                if(new_factor > 0 && new_factor < 5000) { calib_factor = (unsigned int)new_factor; params_dirty = 1; }
                            }
                            cal_sub_state = 0; dp_flag[0]=0; dp_flag[1]=0; dp_flag[2]=0;
                            update_display_u_set();
                        }
                        last_action_time = now; beep_ms(BEEP_MS_ENC_STEP);
                    }
                    if(cal_sub_state == 0 && (now - last_action_time) >= IDLE_TIMEOUT_MS) {
                        sys_state = 1; disp_first = 1; last_disp_10x = display_voltage / 100;
                        if(last_disp_10x > 63) last_disp_10x = 63;
                        filter_cnt = 0; filter_sum = 0; filter_head = 0;
                        dp_flag[0]=0; dp_flag[1]=1; dp_flag[2]=0;
                        update_display_shadow(); save_pending = 1; seq_count = 0; sync_encoder_state();
                        set_power_mode(POWER_MODE_STANDBY);
                    }
                    break;
            }
        } else {
            l_short = 0; l_long = 0;
        }

        // 更新功率状态
        update_power_state();
    }
}
