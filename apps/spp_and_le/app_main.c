/*********************************************************************************************
    *   Filename        : app_main.c

    *   Description     :

    *   Copyright:(c)JIELI  2011-2019  @ , All Rights Reserved.
*********************************************************************************************/
#include "system/includes.h"
#include "app_config.h"
#include "app_action.h"
#include "app_main.h"
#include "update.h"
#include "update_loader_download.h"
#include "app_charge.h"
#include "app_power_manage.h"
#include "asm/charge.h"

#include "examples/trans_data/ble_trans.h"

#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
#include "jl_kws/jl_kws_api.h"
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */

#define LOG_TAG_CONST       APP
#define LOG_TAG             "[APP]"
//#define LOG_ERROR_ENABLE
//#define LOG_DEBUG_ENABLE
//#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
//#define LOG_CLI_ENABLE
#include "debug.h"

/*任务列表   */
const struct task_info task_info_table[] = {
#if CONFIG_APP_FINDMY
    {"app_core",            1,     0,   640 * 2,   128  },
#else
    {"app_core",            1,     0,   640,   128  },
#endif

    {"sys_event",           7,     0,   256,   0    },
    {"btctrler",            4,     0,   512,   256  },
    {"btencry",             1,     0,   512,   128  },
    {"btstack",             3,     0,   768,   256   },
    {"systimer",		    7,	   0,   128,   0	},
    {"update",				1,	   0,   512,   0    },
    {"dw_update",		 	2,	   0,   256,   128  },
#if (RCSP_BTMATE_EN)
    {"rcsp_task",		    2,	   0,   640,	0},
#endif
#if(USER_UART_UPDATE_ENABLE)
    {"uart_update",	        1,	   0,   256,   128	},
#endif
#if (XM_MMA_EN)
    {"xm_mma",   		    2,	   0,   640,   256	},
#endif
    {"usb_msd",           	1,     0,   512,   128  },
#if TCFG_AUDIO_ENABLE
    {"audio_dec",           3,     0,   768,   128  },
    {"audio_enc",           4,     0,   512,   128  },
#endif/*TCFG_AUDIO_ENABLE*/
#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
    {"kws",                 2,     0,   256,   64   },
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */
#if (TUYA_DEMO_EN)
    {"user_deal",           2,     0,   512,   512  },//定义线程 tuya任务调度
#endif
#if (CONFIG_APP_HILINK)
    {"hilink_task",         2,     0,   1024,   0},//定义线程 hilink任务调度
#endif
    {0, 0},
};

APP_VAR app_var;

void app_var_init(void)
{
    app_var.play_poweron_tone = 1;

    app_var.auto_off_time =  TCFG_AUTO_SHUT_DOWN_TIME;
    app_var.warning_tone_v = 340;
    app_var.poweroff_tone_v = 330;
}

__attribute__((weak))
u8 get_charge_online_flag(void)
{
    return 0;
}

void clr_wdt(void);
void check_power_on_key(void)
{
#if TCFG_POWER_ON_NEED_KEY

    u32 delay_10ms_cnt = 0;
    while (1) {
        clr_wdt();
        os_time_dly(1);

        extern u8 get_power_on_status(void);
        if (get_power_on_status()) {
            log_info("+");
            delay_10ms_cnt++;
            if (delay_10ms_cnt > 70) {
                /* extern void set_key_poweron_flag(u8 flag); */
                /* set_key_poweron_flag(1); */
                return;
            }
        } else {
            log_info("-");
            delay_10ms_cnt = 0;
            log_info("enter softpoweroff\n");
            power_set_soft_poweroff();
        }
    }
#endif
}

//extern u16 trans_con_handle;
#define ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE 0x000d
#define ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE 0x000c
static volatile u8 g_ble_notify_enabled = 0;
// Helper to update BLE notification state (Call this from your BLE event callback)
void app_set_ble_notify_status(u8 enable) {
    g_ble_notify_enabled = enable;
}

#define ADC 1
#define UART 0

#if UART
// Create a semaphore for RX notification
UT_Semaphore rx_semaphore;

static void my_uart_rx_callback(void *bus, u32 status)
{
    if (status == UT_RX) {
        // Notify a waiting task that data is ready
        UT_OSSemPost(&rx_semaphore);
    }
}

#define RX_BUFFER_SIZE   256
#define MSG_BUFFER_SIZE  20     //BLE Default MTU payload is 20 bytes.
#define TASK_STACK_SIZE  512
#define UART_TASK_PRIO   4

//static uint8_t rx_buffer[512];   // must be 2^n (e.g., 64, 128, 256...)
static uint8_t rx_buffer[RX_BUFFER_SIZE];

struct uart_platform_data_t uart_cfg = {
    .tx_pin       = IO_PORT_DP ,
    .rx_pin       = IO_PORT_DM ,
    .rx_cbuf      = rx_buffer,
    .rx_cbuf_size = sizeof(rx_buffer),
    //.frame_length = 1,          // interrupt after each byte (or desired threshold)
    .frame_length = 16,
    .rx_timeout   = 10,         // timeout in ms (for OT interrupt)
    .isr_cbfun    = my_uart_rx_callback,
    .argv         = NULL,       // optional user data passed to callback
    .is_9bit      = 0,
    .baud         = 115200,
    //.baud         = 19200,  // For Torqeedo
    // other fields (if any) should be zeroed
};

/*
static void uart_rx_task(void *p_arg)
{
    uart_bus_t *bus = (uart_bus_t *)p_arg;   // the UART bus pointer passed as argument
    uint8_t byte;

    while (1) {
        // Wait for the semaphore posted by the UART callback
        //UT_OSSemPend(&rx_semaphore, OS_WAIT_FOREVER);
        UT_OSSemPend(&rx_semaphore, 600000);

        // Read all available bytes (use timeout 0 for non blocking)
        while (bus->getbyte(&byte, 0)) {
            // Echo the byte back
            bus->putbyte(byte);
        }
    }
}
*/

static void uart_rx_task(void *p_arg)
{
    const uart_bus_t *uart_bus = (const uart_bus_t *)p_arg;
    uint8_t byte;
    // Buffer for accumulating a message (optional)
    //uint8_t msg_buffer[16];
    //uint8_t msg_buffer[16];
    uint8_t msg_buffer[MSG_BUFFER_SIZE];
    uint32_t msg_len = 0;

    while (1) {
        // Wait for the semaphore posted by the UART callback
        UT_OSSemPend(&rx_semaphore, 600000);

        // Read all available bytes (non‑blocking inside the task)
        while (uart_bus->getbyte(&byte, 0)) {
            // Echo the byte back (optional)
            //uart_bus->putbyte(byte);

            // Store byte in message buffer if you want to send larger chunks
            if (msg_len < sizeof(msg_buffer)) {
                msg_buffer[msg_len++] = byte;
            }

            // Optional: Break if buffer is full to process immediately
            // This breaks the inner while loop so the code moves to the SEND logic
            if (byte == '\r' || byte == '\n' || msg_len >= sizeof(msg_buffer)) {
                break;
            }
        }

        /*
        // If we have accumulated data and BLE is ready, send it
        if (msg_len > 0 && trans_con_handle) {
            //uart_bus->write("SEND\n", 5);
            // Check if notifications are enabled for the characteristic
            if (ble_gatt_server_characteristic_ccc_get(trans_con_handle,
                    ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE) == 1) {
                int ret = ble_comm_att_send_data(trans_con_handle,
                                                 ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE,
                                                 msg_buffer,
                                                 msg_len,
                                                 0);
            }
            msg_len = 0; // reset buffer for next message
        } */
        // Send via BLE if connected and notifications enabled
        if (msg_len > 0 && trans_con_handle && g_ble_notify_enabled) {
            // Fixed Macro Names (Removed spaces)
            int ret = ble_comm_att_send_data(trans_con_handle,
                                             ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE,
                                             msg_buffer,
                                             msg_len,
                                             0);
        }

        clr_wdt();
        // No os_time_dly needed – we block on semaphore
    }
}

void TestPrint(void *priv)
{
    const uart_bus_t *uart_bus = (const uart_bus_t *)priv;
    // Now you can use uart_bus, for example:
    uart_bus->write("Good\r\n", 8);
}
#endif // UART

#if ADC
#define ADC_MAX        1023.0f
#define ADC_REF        3.3f

#define VOLT_SCALE     (5.0f / 2.5f)   // divider compensation

#define SPEED_SCALE    12.0f           // 5V -> 60 m/s
#define ANGLE_SCALE    72.0f           // 5V -> 360 deg

#define LPF_ALPHA      0.1f            // ~0.5s smoothing @10Hz

float speed_offset = 0.0f;
float dir_offset   = 0.0f;

float speed_filt = 0.0f;

// circular filter
float dir_x = 1.0f;
float dir_y = 0.0f;

float adc_to_voltage(u16 adc)
{
    float v = (adc / ADC_MAX) * ADC_REF;
    return v * VOLT_SCALE;
}
void compute_wind(u16 adc_speed, u16 adc_dir, float *speed, float *angle)
{
    float v_speed = adc_to_voltage(adc_speed);
    float v_dir   = adc_to_voltage(adc_dir);

    // --- speed ---
    float s = v_speed * SPEED_SCALE;
    s -= speed_offset;
    if (s < 0) s = 0;

    // LPF
    speed_filt += LPF_ALPHA * (s - speed_filt);

    // --- angle ---
    float a = v_dir * ANGLE_SCALE;

    // apply offset
    a -= dir_offset;
    if (a < 0) a += 360;
    if (a >= 360) a -= 360;

    // circular filtering
    float rad = a * 3.1415926f / 180.0f;
    float x = cosf(rad);
    float y = sinf(rad);

    dir_x += LPF_ALPHA * (x - dir_x);
    dir_y += LPF_ALPHA * (y - dir_y);

    float a_filt = atan2f(dir_y, dir_x) * 180.0f / 3.1415926f;
    if (a_filt < 0) a_filt += 360;

    *speed = speed_filt;
    *angle = a_filt;
}



void initUSB_ADC(){
    usb_iomode(1);
    gpio_set_dieh(IO_PORT_DM, 0);gpio_set_die(IO_PORT_DM, 0);gpio_set_pull_down(IO_PORT_DM, 0);gpio_set_pull_up(IO_PORT_DM, 0);
    gpio_set_dieh(IO_PORT_DP, 0);gpio_set_die(IO_PORT_DP, 0);gpio_set_pull_down(IO_PORT_DP, 0);gpio_set_pull_up(IO_PORT_DP, 0);
    gpio_set_direction(IO_PORT_DM, 1); gpio_set_direction(IO_PORT_DP, 1);
}

void build_packet(uint8_t *buf, float speed, float angle)
{
    u16 s = (u16)(speed * 100.0f);
    u16 a = (u16)(angle * 100.0f);

    buf[0] = 'W';       // 0x57
    buf[1] = 0x01;      // flags

    buf[2] = s >> 8;
    buf[3] = s & 0xFF;

    buf[4] = a >> 8;
    buf[5] = a & 0xFF;

    u8 cs = 0;
    for (int i = 0; i < 6; i++) cs ^= buf[i];
    buf[6] = cs;
}

void readWindADC(){
    u8 packet[7];
    u16 adc_dir    = adc_get_voltage(AD_CH_DM0);
    u16 adc_speed  = adc_get_voltage(AD_CH_DP0);

    float speed, angle;
    compute_wind(adc_speed, adc_dir, &speed, &angle);
    build_packet(packet, speed, angle);

    //ble_notify(packet, 7);
    if (trans_con_handle && g_ble_notify_enabled) {
        // Fixed Macro Names (Removed spaces)
        int ret = ble_comm_att_send_data(trans_con_handle, ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE, packet, 7, 0);
    }
}
#endif // ADC

//static u16 timerID;
void app_main()
{
#if UART
    UT_OSSemCreate(&rx_semaphore, 0);
    //const uart_bus_t *uart_bus = uart_dev_open(&uart_cfg);
    const uart_bus_t *uart_bus = uart_dev_open(&uart_cfg);
    if (uart_bus) {
        ble_trans_init(uart_bus);
    }

    //int ret = os_task_create(uart_rx_task,(void *)uart_bus,15,128,0,"uart_rx");
    int ret = os_task_create(uart_rx_task, (void *)uart_bus,
                             UART_TASK_PRIO, TASK_STACK_SIZE,
                             0, "uart_rx");
    //timerID = sys_timer_add((void *)uart_bus, TestPrint , 1000);
#endif // UART

#if ADC
    initUSB_ADC();
    adc_add_sample_ch(AD_CH_DM0);adc_set_sample_freq(AD_CH_DM0, 100);
    adc_add_sample_ch(AD_CH_DP0);adc_set_sample_freq(AD_CH_DP0, 100);
    u16 timerID = sys_timer_add(NULL, readWindADC, 100); // 10Hz

#endif // ADC

    struct intent it;

    if (!UPDATE_SUPPORT_DEV_IS_NULL()) {
        int update = 0;
        update = update_result_deal();
    }

    //printf(">>>>>>>>>>>>>>>>>app_main...\n");
    //printf(">>> v220,2022-11-23 >>>\n");

    if (get_charge_online_flag()) {
#if(TCFG_SYS_LVD_EN == 1)
        vbat_check_init();
#endif
    } else {
        check_power_on_voltage();
    }

#if TCFG_POWER_ON_NEED_KEY
    check_power_on_key();
#endif

#if TCFG_AUDIO_ENABLE
    extern int audio_dec_init();
    extern int audio_enc_init();
    audio_dec_init();
    audio_enc_init();
#endif/*TCFG_AUDIO_ENABLE*/

#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
    jl_kws_main_user_demo();
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */

    init_intent(&it);

#if CONFIG_APP_SPP_LE
    it.name = "spp_le";
    it.action = ACTION_SPPLE_MAIN;

#elif CONFIG_APP_AT_COM || CONFIG_APP_AT_CHAR_COM
    it.name = "at_com";
    it.action = ACTION_AT_COM;

#elif CONFIG_APP_DONGLE
    it.name = "dongle";
    it.action = ACTION_DONGLE_MAIN;

#elif CONFIG_APP_MULTI
    it.name = "multi_conn";
    it.action = ACTION_MULTI_MAIN;

#elif CONFIG_APP_NONCONN_24G
    it.name = "nonconn_24g";
    it.action = ACTION_NOCONN_24G_MAIN;

#elif CONFIG_APP_HILINK
    it.name = "hilink";
    it.action = ACTION_HILINK_MAIN;

#elif CONFIG_APP_LL_SYNC
    it.name = "ll_sync";
    it.action = ACTION_LL_SYNC;

#elif CONFIG_APP_TUYA
    it.name = "tuya";
    it.action = ACTION_TUYA;

#elif CONFIG_APP_CENTRAL
    it.name = "central";
    it.action = ACTION_CENTRAL_MAIN;

#elif CONFIG_APP_DONGLE
    it.name = "dongle";
    it.action = ACTION_DONGLE_MAIN;

#elif CONFIG_APP_BEACON
    it.name = "beacon";
    it.action = ACTION_BEACON_MAIN;

#elif CONFIG_APP_IDLE
    it.name = "idle";
    it.action = ACTION_IDLE_MAIN;

#elif CONFIG_APP_CONN_24G
    it.name = "conn_24g";
    it.action = ACTION_CONN_24G_MAIN;

#elif CONFIG_APP_FINDMY
    it.name = "findmy";
    it.action = ACTION_FINDMY;

#elif CONFIG_APP_FTMS
    it.name = "ftms";
    it.action = ACTION_FTMS;

#else
    while (1) {
        printf("no app!!!");
    }
#endif


    log_info("run app>>> %s", it.name);
    log_info("%s,%s", __DATE__, __TIME__);

    start_app(&it);

#if TCFG_CHARGE_ENABLE
    set_charge_event_flag(1);
#endif
}

/*
 * app模式切换
 */
void app_switch(const char *name, int action)
{
    struct intent it;
    struct application *app;

    log_info("app_exit\n");

    init_intent(&it);
    app = get_current_app();
    if (app) {
        /*
         * 退出当前app, 会执行state_machine()函数中APP_STA_STOP 和 APP_STA_DESTORY
         */
        it.name = app->name;
        it.action = ACTION_BACK;
        start_app(&it);
    }

    /*
     * 切换到app (name)并执行action分支
     */
    it.name = name;
    it.action = action;
    start_app(&it);
}

int eSystemConfirmStopStatus(void)
{
    /* 系统进入在未来时间里，无任务超时唤醒，可根据用户选择系统停止，或者系统定时唤醒(100ms) */
    //1:Endless Sleep
    //0:100 ms wakeup
    /* log_info("100ms wakeup"); */
    return 1;
}

__attribute__((used)) int *__errno()
{
    static int err;
    return &err;
}

