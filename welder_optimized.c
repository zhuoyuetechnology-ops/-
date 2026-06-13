#include <stc8h.h>
#include <intrins.h>
#include <math.h>
#include <string.h>

/************************ 硬件引脚定义 ************************/
sbit PWR      = P1^7;
sbit GATE     = P1^3;
sbit BEEP     = P5^4;
sbit SW       = P1^0;
sbit BL       = P1^1;
sbit TRIG     = P3^7;
sbit LCD_DC   = P3^6;
sbit LCD_RST  = P3^5;
sbit LCD_SDA  = P3^4;
sbit BOOST_EN = P3^3;
sbit LCD_SCL  = P3^2;
sbit EC11_A   = P3^0;
sbit EC11_B   = P3^1;

#define ADC_VOLT_CH    5
#define ADC_NTC_CH     4

/************************ 硬件&阈值参数 ************************/
#define MCU_VCC         5.0f
#define R4              10000.0f
#define R5              4700.0f
#define R3              10000.0f
#define NTC_R25         10000.0f
#define NTC_B           3950.0f

#define VOLT_LOW        10.0f
#define BOOST_VOLT_THRESHOLD 12.0f
#define TEMP_MAX        65.0f
#define PULSE_MIN       1
#define PULSE_MAX       30
#define VER_NUM         101

#define MAIN_LOOP_DELAY 50
#define KEY_DEBOUNCE_MS 20
#define LCD_REFRESH_MS  100

/************************ 功能宏定义 ************************/
#define BEEP_ON()       BEEP = 0
#define BEEP_OFF()      BEEP = 1
#define MOS_ON()        GATE = 1
#define MOS_OFF()       GATE = 0
#define BOOST_ON()      BOOST_EN = 1
#define BOOST_OFF()     BOOST_EN = 0
#define BL_ON()         BL = 1
#define BL_OFF()        BL = 0
#define SYS_POWER_OFF() PWR = 0

/************************ RGB565 颜色定义 ************************/
#define WHITE    0xFFFF
#define BLACK    0x0000
#define RED      0xF800
#define GREEN    0x07E0
#define BLUE     0x001F
#define YELLOW   0xFFE0
#define GRAY     0x8410
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define DARKBLUE 0x001A
#define DARKGREEN 0x03E0
#define DARKRED  0x7800
#define TEAL     0x07D8
#define PINK     0xF818
#define YELLOW_BG 0xFFC0
#define STATUS_BLUE 0x21F8

/************************ 全局变量 ************************/
unsigned int adc_raw = 0;
float bat_voltage = 0.0f;
float ntc_temp = 0.0f;
unsigned char pulse_pre  = 8;
unsigned char pulse_main = 12;
unsigned char pulse_gap  = 3;
unsigned char enc_data = 10;
bit work_flag = 0;
unsigned int weld_cnt = 0;
unsigned char weld_state = 0;
unsigned char sys_delay = 0;

// 菜单相关
bit menu_en = 0;
unsigned char menu_idx = 0;

// 用户设置项
bit beep_sw      = 1;
bit enc_rev      = 0;
bit temp_pro_sw  = 1;
bit lcd_bg_sw    = 0;
bit lcd_mirror   = 0;
bit auto_boost_sw = 1;
unsigned char bl_level = 3;

// 时间管理
unsigned int ms_counter = 0;
unsigned int last_lcd_refresh = 0;

// 焊接状态字符串
const unsigned char code weld_states[7][8] = {
    "空闲  ",
    "预热  ",
    "间隔  ",
    "焊接  ",
    "过热  ",
    "低压  ",
    "未知  "
};

/************************ 字模存入CODE区 ************************/
const unsigned char code F8X16[][16] = {
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x7E,0x81,0x81,0x81,0x7E,0x00,0x00,0x00,0x7E,0x81,0x81,0x81,0x7E,0x00},
	{0x00,0x00,0x82,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00},
	{0x00,0x00,0xF2,0x89,0x89,0x89,0x86,0x00,0x00,0x00,0x82,0x81,0x81,0x81,0x7E,0x00},
	{0x00,0x00,0x82,0x81,0x89,0x89,0x76,0x00,0x00,0x00,0x41,0x81,0x81,0x81,0x7E,0x00},
	{0x00,0x00,0x1F,0x10,0x10,0x10,0xFF,0x00,0x00,0x00,0x10,0x10,0x10,0x10,0xFF,0x00},
	{0x00,0x00,0x82,0x81,0x81,0x81,0x79,0x00,0x00,0x00,0x7F,0x80,0x80,0x80,0x7F,0x00},
	{0x00,0x00,0x7E,0x81,0x89,0x89,0x78,0x00,0x00,0x00,0x7F,0x80,0x80,0x80,0x7F,0x00},
	{0x00,0x00,0x81,0x81,0x81,0x81,0xFF,0x00,0x00,0x00,0x0E,0x01,0x01,0x01,0x01,0x00},
	{0x00,0x00,0x76,0x89,0x89,0x89,0x76,0x00,0x00,0x00,0x7E,0x81,0x81,0x81,0x7E,0x00},
	{0x00,0x00,0x86,0x89,0x89,0x89,0x7E,0x00,0x00,0x00,0x01,0x81,0x81,0x81,0x7E,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
};

// 10的幂次查表（优化 pow() 调用）
const unsigned int code pow10_table[5] = {10000, 1000, 100, 10, 1};

/************************ 基础延时 ************************/
void delay_ms(unsigned int ms)
{
    unsigned int i,j;
    for(i = ms; i > 0; i--)
        for(j = 110; j > 0; j--);
}

void delay_us(unsigned int us)
{
    unsigned int i;
    for(i = us; i > 0; i--)
        _nop_();
}

/************************ 系统初始化 ************************/
void System_Init(void)
{
    P_SW1 = 0x00;
    P_SW2 = 0x00;
    P1M0 = 0x00; P1M1 = 0x00;
    P3M0 = 0x00; P3M1 = 0x00;
    P5M0 = 0x00; P5M1 = 0x00;

    PWR      = 1;
    MOS_OFF();
    BEEP_OFF();
    BL_ON();
    BOOST_OFF();

    ADCCFG = 0x0F;
    ADC_CONTR = 0x80;
    delay_ms(1);

    TMOD |= 0x01;
    TH0 = 0; TL0 = 0;
    ET0 = 1;
    EA  = 1;
    
    ms_counter = 0;
    last_lcd_refresh = 0;
}

/************************ ADC读取与计算 ************************/
unsigned int Get_ADC(unsigned char ch)
{
    ADC_CONTR = 0x80;
    ADC_CONTR |= ch;
    ADC_CONTR |= 0x08;
    while(!(ADC_CONTR & 0x10));
    ADC_CONTR &= ~0x10;
    return (ADC_RES << 2) | ADC_RESL;
}

float Get_Battery_Volt(void)
{
    adc_raw = Get_ADC(ADC_VOLT_CH);
    bat_voltage = MCU_VCC * (R4 + R5) / R5 * adc_raw / 1024.0f;
    return bat_voltage;
}

float Get_NTC_Temperature(void)
{
    float adc_volt, ntc_res;
    adc_raw = Get_ADC(ADC_NTC_CH);
    adc_volt = MCU_VCC * adc_raw / 1024.0f;
    ntc_res = R3 * (MCU_VCC - adc_volt) / adc_volt;
    ntc_temp = 1.0f / (1.0f / 298.15f + log(ntc_res / NTC_R25) / NTC_B) - 273.15f;
    return ntc_temp;
}

/************************ ST7735S SPI驱动 ************************/
void SPI_Write_Byte(unsigned char dat)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        LCD_SCL = 0;
        if(dat & 0x80) LCD_SDA = 1;
        else           LCD_SDA = 0;
        dat <<= 1;
        LCD_SCL = 1;
    }
}

void LCD_Write_Cmd(unsigned char cmd)
{
    LCD_DC = 0;
    SPI_Write_Byte(cmd);
}

void LCD_Write_Data(unsigned char dat)
{
    LCD_DC = 1;
    SPI_Write_Byte(dat);
}

void LCD_Reset(void)
{
    LCD_RST = 0; delay_ms(50);
    LCD_RST = 1; delay_ms(50);
}

void LCD_Set_Window(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2)
{
    LCD_Write_Cmd(0x2A);
    LCD_Write_Data(0x00); LCD_Write_Data(x1);
    LCD_Write_Data(0x00); LCD_Write_Data(x2);
    LCD_Write_Cmd(0x2B);
    LCD_Write_Data(0x00); LCD_Write_Data(y1);
    LCD_Write_Data(0x00); LCD_Write_Data(y2);
    LCD_Write_Cmd(0x2C);
}

void LCD_Fill(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned int color)
{
    unsigned int i,j;
    LCD_Set_Window(x1,y1,x2,y2);
    for(i = y1; i <= y2; i++)
        for(j = x1; j <= x2; j++)
        {
            LCD_Write_Data(color >> 8);
            LCD_Write_Data(color & 0xFF);
        }
}

void LCD_Draw_Char(unsigned char x,unsigned char y,unsigned char chr,unsigned int color,unsigned int bg)
{
    unsigned char i,j;
    const unsigned char *p = F8X16[chr - 0x20];
    LCD_Set_Window(x,y,x+7,y+15);
    for(i = 0; i < 16; i++)
    {
        for(j = 0; j < 8; j++)
        {
            if(p[i] & (0x80 >> j))
            {
                LCD_Write_Data(color >> 8);
                LCD_Write_Data(color & 0xFF);
            }
            else
            {
                LCD_Write_Data(bg >> 8);
                LCD_Write_Data(bg & 0xFF);
            }
        }
    }
}

// ✅ 优化：使用查表替代 pow()
void LCD_Show_Num(unsigned char x,unsigned char y,unsigned int num,unsigned char len,unsigned int color,unsigned int bg)
{
    unsigned char t,i,enshow = 0;
    unsigned char divisor_idx = 4 - len;
    
    for(i = 0; i < len; i++)
    {
        t = (unsigned char)(num / pow10_table[divisor_idx + i]) % 10;
        if(enshow || (i == len-1) || (t != 0))
        {
            enshow = 1;
            LCD_Draw_Char(x + 8*i, y, t + 0x30, color, bg);
        }
        else
        {
            LCD_Draw_Char(x + 8*i, y, 0x20, color, bg);
        }
    }
}

void LCD_Show_Float(unsigned char x,unsigned char y,float dat,unsigned int color,unsigned int bg)
{
    unsigned int temp = (unsigned int)(dat * 10);
    LCD_Show_Num(x, y, temp / 10, 2, color, bg);
    LCD_Draw_Char(x+16, y, 0x2E, color, bg);
    LCD_Show_Num(x+24, y, temp % 10, 1, color, bg);
}

void LCD_Show_Str(unsigned char x,unsigned char y,unsigned char *str,unsigned int color,unsigned int bg)
{
    while(*str)
    {
        LCD_Draw_Char(x,y,*str,color,bg);
        x += 8;
        str++;
    }
}

void LCD_Init(void)
{
    unsigned char madctl = 0x60;
    LCD_Reset();
    LCD_Write_Cmd(0x11); delay_ms(120);
    LCD_Write_Cmd(0x21);
    LCD_Write_Cmd(0xB1); LCD_Write_Data(0x05); LCD_Write_Data(0x3A); LCD_Write_Data(0x3A);
    LCD_Write_Cmd(0xB2); LCD_Write_Data(0x05); LCD_Write_Data(0x3A); LCD_Write_Data(0x3A);
    LCD_Write_Cmd(0xB3); LCD_Write_Data(0x05); LCD_Write_Data(0x3A); LCD_Write_Data(0x3A);
    LCD_Write_Cmd(0xB4); LCD_Write_Data(0x03);
    LCD_Write_Cmd(0xC0); LCD_Write_Data(0x62); LCD_Write_Data(0x02); LCD_Write_Data(0x04);
    LCD_Write_Cmd(0xC1); LCD_Write_Data(0xC0);
    LCD_Write_Cmd(0xC2); LCD_Write_Data(0x0D); LCD_Write_Data(0x00);
    LCD_Write_Cmd(0xC3); LCD_Write_Data(0x8D); LCD_Write_Data(0x6A);
    LCD_Write_Cmd(0xC4); LCD_Write_Data(0x8D); LCD_Write_Data(0xEE);
    LCD_Write_Cmd(0xC5); LCD_Write_Data(0x0E);

    if(lcd_mirror) madctl = 0xA0;
    LCD_Write_Cmd(0x36); LCD_Write_Data(madctl);
    LCD_Write_Cmd(0x3A); LCD_Write_Data(0x05);
    LCD_Write_Cmd(0x29); delay_ms(100);
    LCD_Fill(0, 0, 127, 159, lcd_bg_sw ? WHITE : BLACK);
}

/************************ EC11编码器扫描 ************************/
void EC11_Scan(void)
{
    static unsigned char a_sta = 0, b_sta = 0;
    unsigned char a = EC11_A;
    unsigned char b = EC11_B;
    signed char dir = 1;

    if(enc_rev) dir = -1;
    if(a != a_sta || b != b_sta)
    {
        if(a_sta == 0 && a == 1)
        {
            if(b == 0) enc_data += dir;
            else       enc_data -= dir;
        }
        a_sta = a;
        b_sta = b;
    }
    if(enc_data > PULSE_MAX) enc_data = PULSE_MAX;
    if(enc_data < PULSE_MIN) enc_data = 1;

    pulse_pre  = enc_data - 2;
    pulse_main = enc_data;
    if(pulse_pre < 1) pulse_pre = 1;
}

/************************ 按键扫描（优化：加入超时机制） ************************/
void Key_Scan(void)
{
    unsigned int timeout = 0;
    
    if(SW == 0)
    {
        delay_ms(KEY_DEBOUNCE_MS);
        if(SW == 0)
        {
            if(beep_sw){BEEP_ON();delay_ms(40);BEEP_OFF();}
            if(!menu_en)
            {
                menu_en = 1;
                menu_idx = 0;
            }
            else
            {
                menu_idx++;
                if(menu_idx >= 10) menu_idx = 0;
            }
            
            // ✅ 优化：加入超时，防止卡死
            while(SW == 0 && timeout < 5000)
            {
                delay_ms(1);
                timeout++;
            }
        }
    }
}

/************************ 焊笔检测 ************************/
bit Welder_Pen_Detect(void)
{
    return (TRIG == 0) ? 1 : 0;
}

/************************ 双脉冲点焊 + 自动升压逻辑 ************************/
void Double_Pulse_Weld(void)
{
    if(bat_voltage < VOLT_LOW)
    {
        weld_state = 5;
        return;
    }
    if(temp_pro_sw && ntc_temp > TEMP_MAX)
    {
        weld_state = 4;
        return;
    }

    if(Welder_Pen_Detect() && !work_flag)
    {
        work_flag = 1;
        if(beep_sw){BEEP_ON();delay_ms(60);BEEP_OFF();}

        if(auto_boost_sw && bat_voltage < BOOST_VOLT_THRESHOLD)
        {
            BOOST_ON();
            delay_ms(10);
        }

        weld_state = 1;
        MOS_ON();
        delay_ms(pulse_pre);
        MOS_OFF();

        weld_state = 2;
        delay_ms(pulse_gap);

        weld_state = 3;
        MOS_ON();
        delay_ms(pulse_main);
        MOS_OFF();

        BOOST_OFF();
        work_flag = 0;
        weld_cnt++;
    }
}

// ✅ 优化：修复焊接状态字符串获取（原版指针赋值无效）
void Get_Weld_State_Str(unsigned char *buf, unsigned char state_val)
{
    unsigned char idx = state_val;
    if(idx >= 7) idx = 6;
    strcpy(buf, (unsigned char*)weld_states[idx]);
}

/************************ 主界面【优化版：代码清晰】 ************************/
void Main_Display(void)
{
    unsigned char state_buf[8];
    
    Get_Weld_State_Str(state_buf, weld_state);
    LCD_Fill(0,0,127,159, BLACK);

    // 第一行：电压 / 温度 / 计数
    LCD_Fill(2,2,40,38,DARKBLUE);
    LCD_Show_Str(6,14,(unsigned char*)"电压:",WHITE,DARKBLUE);
    LCD_Show_Float(42,14,bat_voltage,WHITE,DARKBLUE);

    LCD_Fill(44,2,82,38,DARKGREEN);
    LCD_Show_Str(48,14,(unsigned char*)"温度:",WHITE,DARKGREEN);
    LCD_Show_Float(84,14,ntc_temp,WHITE,DARKGREEN);

    LCD_Fill(86,2,125,38,DARKRED);
    LCD_Show_Str(90,14,(unsigned char*)"计数:",WHITE,DARKRED);
    LCD_Show_Num(118,14,weld_cnt,5,WHITE,DARKRED);

    // 第二行：脉冲1 / 间隔 / 脉冲2
    LCD_Fill(2,42,40,78,GRAY);
    LCD_Show_Str(6,54,(unsigned char*)"脉冲1:",WHITE,GRAY);
    LCD_Show_Num(42,54,pulse_pre,2,WHITE,GRAY);
    LCD_Show_Str(58,54,(unsigned char*)"ms",WHITE,GRAY);

    LCD_Fill(44,42,82,78,TEAL);
    LCD_Show_Str(48,54,(unsigned char*)"间隔:",WHITE,TEAL);
    LCD_Show_Num(80,54,pulse_gap,1,WHITE,TEAL);
    LCD_Show_Str(92,54,(unsigned char*)"ms",WHITE,TEAL);

    LCD_Fill(86,42,125,78,GRAY);
    LCD_Show_Str(90,54,(unsigned char*)"脉冲2:",WHITE,GRAY);
    LCD_Show_Num(124,54,pulse_main,2,WHITE,GRAY);
    LCD_Show_Str(140,54,(unsigned char*)"ms",WHITE,GRAY);

    // 第三行：延时 / 系统设置
    LCD_Fill(2,82,63,118,PINK);
    LCD_Show_Str(6,94,(unsigned char*)"延时:",WHITE,PINK);
    LCD_Show_Num(42,94,sys_delay,2,WHITE,PINK);
    LCD_Show_Str(58,94,(unsigned char*)"ms",WHITE,PINK);

    LCD_Fill(67,82,125,118,YELLOW_BG);
    LCD_Show_Str(80,94,(unsigned char*)"系统设置",BLACK,YELLOW_BG);

    // 第四行：状态
    LCD_Fill(2,122,125,158,STATUS_BLUE);
    LCD_Show_Str(6,134,(unsigned char*)"状态:",WHITE,STATUS_BLUE);
    LCD_Show_Str(38,134,state_buf,WHITE,STATUS_BLUE);
}

/************************ 系统菜单【优化版】 ************************/
void Menu_Display(void)
{
    unsigned int bg = lcd_bg_sw ? WHITE : BLACK;
    unsigned int fg = lcd_bg_sw ? BLACK : WHITE;
    unsigned int sel_color = RED;
    LCD_Fill(0,0,127,159,bg);

    LCD_Show_Str(20,0,(unsigned char*)"SYSTEM MENU",YELLOW,bg);

    // 菜单0：BEEP开关
    if(menu_idx == 0) LCD_Show_Str(0,16,(unsigned char*)">BEEP SW  :",sel_color,bg);
    else              LCD_Show_Str(0,16,(unsigned char*)" BEEP SW  :",fg,bg);
    LCD_Show_Str(80,16, beep_sw ? (unsigned char*)"ON" : (unsigned char*)"OFF",fg,bg);

    // 菜单1：亮度
    if(menu_idx == 1) LCD_Show_Str(0,32,(unsigned char*)">BRIGHT   :",sel_color,bg);
    else              LCD_Show_Str(0,32,(unsigned char*)" BRIGHT   :",fg,bg);
    LCD_Show_Num(80,32,bl_level,1,fg,bg);

    // 菜单2：编码器反向
    if(menu_idx == 2) LCD_Show_Str(0,48,(unsigned char*)">ENC REV  :",sel_color,bg);
    else              LCD_Show_Str(0,48,(unsigned char*)" ENC REV  :",fg,bg);
    LCD_Show_Str(80,48, enc_rev ? (unsigned char*)"ON" : (unsigned char*)"OFF",fg,bg);

    // 菜单3：温度保护
    if(menu_idx == 3) LCD_Show_Str(0,64,(unsigned char*)">TEMP PRO :",sel_color,bg);
    else              LCD_Show_Str(0,64,(unsigned char*)" TEMP PRO :",fg,bg);
    LCD_Show_Str(80,64, temp_pro_sw ? (unsigned char*)"ON" : (unsigned char*)"OFF",fg,bg);

    // 菜单4：自动升压
    if(menu_idx == 4) LCD_Show_Str(0,80,(unsigned char*)">AUTO BOOST:",sel_color,bg);
    else              LCD_Show_Str(0,80,(unsigned char*)" AUTO BOOST:",fg,bg);
    LCD_Show_Str(96,80, auto_boost_sw ? (unsigned char*)"ON" : (unsigned char*)"OFF",fg,bg);

    // 菜单5：LCD背景
    if(menu_idx == 5) LCD_Show_Str(0,96,(unsigned char*)">LCD BG   :",sel_color,bg);
    else              LCD_Show_Str(0,96,(unsigned char*)" LCD BG   :",fg,bg);
    LCD_Show_Str(80,96, lcd_bg_sw ? (unsigned char*)"WHT" : (unsigned char*)"BLK",fg,bg);

    // 菜单6：LCD镜像
    if(menu_idx == 6) LCD_Show_Str(0,112,(unsigned char*)">LCD MIR  :",sel_color,bg);
    else              LCD_Show_Str(0,112,(unsigned char*)" LCD MIR  :",fg,bg);
    LCD_Show_Str(80,112, lcd_mirror ? (unsigned char*)"ON" : (unsigned char*)"OFF",fg,bg);

    // 菜单7：版本号
    if(menu_idx == 7) LCD_Show_Str(0,128,(unsigned char*)">VERSION  :",sel_color,bg);
    else              LCD_Show_Str(0,128,(unsigned char*)" VERSION  :",fg,bg);
    LCD_Show_Num(80,128,VER_NUM,3,fg,bg);

    // 菜单8：关机
    if(menu_idx == 8) LCD_Show_Str(0,144,(unsigned char*)">POWER OFF:",sel_color,bg);
    else              LCD_Show_Str(0,144,(unsigned char*)" POWER OFF:",fg,bg);

    // 菜单9：退出
    if(menu_idx == 9) LCD_Show_Str(0,160,(unsigned char*)">EXIT     :",sel_color,bg);
    else              LCD_Show_Str(0,160,(unsigned char*)" EXIT     :",fg,bg);

    // 编码器修改参数逻辑
    if(menu_idx == 0) beep_sw = !beep_sw;
    if(menu_idx == 1)
    {
        bl_level++;
        if(bl_level > 5) bl_level = 1;
        if(bl_level >= 3) BL_ON(); else BL_OFF();
    }
    if(menu_idx == 2) enc_rev = !enc_rev;
    if(menu_idx == 3) temp_pro_sw = !temp_pro_sw;
    if(menu_idx == 4) auto_boost_sw = !auto_boost_sw;
    if(menu_idx == 5)
    {
        lcd_bg_sw = !lcd_bg_sw;
        LCD_Init();
    }
    if(menu_idx == 6)
    {
        lcd_mirror = !lcd_mirror;
        LCD_Init();
    }
    if(menu_idx == 8)
    {
        SYS_POWER_OFF();
        while(1);
    }
    if(menu_idx == 9) menu_en = 0;
}

/************************ 主函数 ************************/
void main(void)
{
    System_Init();
    LCD_Init();
    enc_data = pulse_main;
    weld_cnt = 0;
    weld_state = 0;
    sys_delay = 0;

    while(1)
    {
        Key_Scan();
        EC11_Scan();
        Get_Battery_Volt();
        Get_NTC_Temperature();

        // 温度保护
        if(temp_pro_sw && ntc_temp > TEMP_MAX)
        {
            MOS_OFF();
            BOOST_OFF();
            work_flag = 0;
            weld_state = 4;
            if(beep_sw){BEEP_ON();delay_ms(150);BEEP_OFF();}
        }

        Double_Pulse_Weld();

        if(menu_en) Menu_Display();
        else        Main_Display();

        delay_ms(MAIN_LOOP_DELAY);
    }
}
