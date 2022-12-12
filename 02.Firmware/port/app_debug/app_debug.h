#ifndef APP_DBG_H
#define APP_DBG_H

#include <stdint.h>
#include <stdbool.h>

#define APP_DBG_LV_ALL                  0
#define APP_DBG_LV_VERBOSE              1
#define APP_DBG_LV_INF                  2
#define APP_DBG_LV_WRN                  3
#define APP_DBG_LV_ERR                  4
#define APP_DBG_LV_NOTHING              5
#define APP_DBG_LV                      APP_DBG_LV_INF  

#ifndef APP_DBG_ISR_ENABLE
#define APP_DBG_ISR_ENABLE            0       // allow debug inside interrupt service
#define APP_DBG_ISR_RINGBUFFER_SIZE   256
#endif

#ifndef APP_DBG_NUMBER_OF_DBG_PORT
#define APP_DBG_NUMBER_OF_DBG_PORT 2
#endif

// Uncomment to disable float print
#ifndef APP_APP_DBG_HAS_FLOAT
//#define APP_APP_DBG_HAS_FLOAT
#endif

#if 0
    #define KNRM  "\x1B[0m"
    #define KRED  RTT_CTRL_TEXT_RED
    #define KGRN  RTT_CTRL_TEXT_GREEN
    #define KYEL  RTT_CTRL_TEXT_YELLOW
    #define KBLU  RTT_CTRL_TEXT_BLUE
    #define KMAG  RTT_CTRL_TEXT_MAGENTA
    #define KCYN  RTT_CTRL_TEXT_CYAN
    #define KWHT  RTT_CTRL_TEXT_WHITE
#else
    #define KNRM  "\x1B[0m"
    #define KRED  "\x1B[31m"
    #define KGRN  "\x1B[32m"
    #define KYEL  "\x1B[33m"
    #define KBLU  "\x1B[34m"
    #define KMAG  "\x1B[35m"
    #define KCYN  "\x1B[36m"
    #define KWHT  "\x1B[37m"
#endif

#define APP_DBG_RAW                               app_dbg_print_raw
#define APP_DBG_DUMP                              app_dbg_dump

#if APP_DBG_ISR_ENABLE
    #define APP_DBG_ISR(s, args...)               app_dbg_print_isr(KBLU "[ISR] %s : " s KNRM, "", ##args)
#else
    #define APP_DBG_ISR(s, args...)                            
#endif

#if (APP_DBG_LV_VERBOSE >= APP_DBG_LV)
#define APP_DBG_VERBOSE(s, args...)               app_dbg_print_raw(KMAG "<%u> [I] %s : " s KNRM,  app_dbg_get_ms(), "", ##args)
#else
#define APP_DBG_VERBOSE(s, args...)               // app_dbg_print_nothing(s, ##args)
#endif

#if (APP_DBG_LV_INF >= APP_DBG_LV)
#define APP_DBG_INF(s, args...)                  app_dbg_print_raw(KGRN "<%u> [I] %s : " s KNRM,  app_dbg_get_ms(), "", ##args)
#else
#define APP_DBG_INF(s, args...)                  // app_dbg_print_nothing(s, ##args)
#endif

#if (APP_DBG_LV_ERR >= APP_DBG_LV)
#define APP_DBG_ERR(s, args...)                 app_dbg_print_raw(KRED "<%u> [E] %s : " s KNRM,  app_dbg_get_ms(), "", ##args)
#else
#define APP_DBG_ERR(s, args...)                 // app_dbg_print_nothing(s, ##args)
#endif

#if (APP_DBG_LV_WRN >= APP_DBG_LV)
#define APP_DBG_WRN(s, args...)                  app_dbg_print_raw(KYEL "<%u> [W] %s : " s KNRM,  app_dbg_get_ms(), "", ##args)
#else
#define APP_DBG_WRN(s, args...)                  // app_dbg_print_nothing(s, ##args)
#endif

#define APP_DBG_COLOR(color, s, args...)          app_dbg_print_raw(color s KNRM, ##args)


#ifndef APP_DBG_FLUSH
#define APP_DBG_FLUSH()                           while(0)
#endif

typedef uint32_t (*app_dbg_get_timestamp_ms_cb_t)(void);      // Get timestamp data 
typedef uint32_t (*app_dbg_output_cb_t)(const void *buffer, uint32_t len);
typedef bool (*app_dbg_lock_cb_t)(bool lock, uint32_t timeout_ms);

/**
 * @brief           Initialize debug module
 * @param[in]       get_ms System get tick in ms
 */
void app_dbg_init(app_dbg_get_timestamp_ms_cb_t get_ms, app_dbg_lock_cb_t lock_cb);

/**
 * @brief           Add call back function to debug array
 *                  Add your function to print data on screen, such as UartTransmit, CDCTransmit,...
 * @param[in]       output_cb Output data callback
 */
void app_dbg_register_callback_print(app_dbg_output_cb_t output_cb);

/**
 * @brief           Remove callback function from debug callback array
 * @param[in]       output_cb  Function which do you want to remove
 */
void app_dbg_unregister_callback_print(app_dbg_output_cb_t output_cb);

/**
 * @brief           Get timebase value in milliseconds
 * @retval          Current counter value of the timebase 
 */
uint32_t app_dbg_get_ms(void);

/**
 * @brief           Print nothing on screen
 *                  Called when the DEBUG_FUNCTION level is less than the APP_DBG_LV value 
 */
void app_dbg_print_nothing(const char *fmt, ...);

/**
 * @brief           Print data on screen, link standard C printf
 * @param[in]       fmt Pointer to the format want to print on the screen
 * @retval          Number of bytes was printed, -1 on error
 */
int32_t app_dbg_print(const char *fmt, ...);

#if APP_DBG_ISR_ENABLE
/**
 * @brief           Print data on screen, link standard C printf inside isr service
 * @param[in]       fmt Pointer to the format want to print on the screen
 * @retval          Number of bytes was printed, -1 on error
 */
void app_dbg_print_isr(const char *fmt, ...);

/**
 * @brief           Flush all data in ringbuffer of ISR
 */
void app_dbg_isr_ringbuffer_flush(void);

#endif /* APP_DBG_ISR_ENABLE */


/**
 * @brief           Print raw data
 * @param[in]       fmt Pointer to the data want to print on the screen      
 */
void app_dbg_print_raw(const char *fmt, ...);

/**
 * @brief           Dump hexa data to screen
 * @param[in]       data Pointer to the data want to print on the screen
 * @param[in]       len Number of bytes want to print
 * @param[in]       message Debug message
 * @retval          Number of bytes was printed, -1 on error    
 */
void app_dbg_dump(const void* data, int32_t len, const char* message);

/**
 * @brief           Put byte to debug port
 * @param[in]       c Data to send
 */
void app_dbg_putc(uint8_t c);

/**
 * @brief           Put byte to debug port in ISR context
 * @param[in]       c Data to send
 */
void app_dbg_raw_isr(uint8_t c);

/**
 * @brief           Disable debug log output
 */
void app_dbg_disable(void);

#endif // !APP_DBG_H
