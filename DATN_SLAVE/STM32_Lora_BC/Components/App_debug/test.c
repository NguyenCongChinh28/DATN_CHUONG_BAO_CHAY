#include "app_debug.h"
#include "stdio.h"

#include "stdint.h"

static uint32_t tick = 0;
static char dump_buffer[32];
uint32_t sys_get_ms(void)
{
    return tick++;
}

uint32_t app_debug_output_cb(const void *buffer, uint32_t len)
{
    const uint8_t *ptr = buffer;
    for (uint32_t i = 0; i < len; i++)
    {
        putchar(ptr[i]);
    }
    fflush(stdout);
    return len;
}


int main(void)
{
    app_debug_init(sys_get_ms, NULL);
    app_debug_register_callback_print(app_debug_output_cb);

    DEBUG_VERBOSE("Test debug module, level Verbose\r\n");
    DEBUG_INFO("Function %s, line %u\r\n", __FUNCTION__, __LINE__);
    DEBUG_INFO("Print string format %%%u\r\n", 9);
    DEBUG_INFO("%s\r\n", 0);
    DEBUG_INFO("Print string with unknown format\r\n", "123");

    DEBUG_INFO("Print valid unsigned : %u\r\n", 1);
    DEBUG_INFO("Print invalid unsigned : %u\r\n", -1);
    DEBUG_INFO("Print invalid unsigned format : %u\r\n");
    
    DEBUG_WARN("Print valid 1 byte hexa  0x%02X\r\n", 1);
    DEBUG_WARN("Print valid 2 bytes hexa 0x%04X\r\n", 2);
    DEBUG_WARN("Print valid 6 bytes hexa 0x%06X\r\n", 3);
    DEBUG_WARN("Print valid 8 bytes hexa 0x%08X\r\n", 4);
    
    DEBUG_WARN("Print invalid hexa 0x%02X\r\n", -1);
    DEBUG_WARN("Print invalid hexa 0x%04X\r\n", -1);
    DEBUG_WARN("Print invalid hexa 0x%06X\r\n", -1);
    DEBUG_WARN("Print invalid hexa 0x%08X\r\n", -1);

    DEBUG_ERROR("Print ascii of character 1 : %c\r\n", '1');
    DEBUG_ERROR("Print ascii of character NULL : %c\r\n");
    DEBUG_ERROR("Print signed int : %d\r\n", -1234567890);
    DEBUG_ERROR("Print long signed int : %ld\r\n", 1234567890);

    DEBUG_ERROR("Tick with : %d\r\n");
    DEBUG_ERROR("Tick1 : %x\r\n", tick);
    DEBUG_ERROR("Tick2 : %X\r\n");
    DEBUG_ERROR("Tick3 : %d\r\n", tick);

    DEBUG_ERROR("Dump buffer with limited len : %.*s\r\n", 4, "Xin chao cac");

    DEBUG_INFO("Dump buffer\r\n");
    for (uint32_t i = 0; i < sizeof(dump_buffer); i++)
    {
        dump_buffer[i] = i;
    }
    app_debug_dump(dump_buffer, sizeof(dump_buffer), "Dump buffer");

    DEBUG_INFO("[0]Print float : %f\r\n", 21.23456);
    DEBUG_INFO("[1]Print float with limited len : %.*f\r\n", 3, 21.23456);

    // printf("Done\r\n");
    return 0;
}

/* va_end example */
// #include <stdio.h>      /* puts */
// #include <stdarg.h>     /* va_list, va_start, va_arg, va_end */

// void PrintLines (char* first, ...)
// {
//   char* str;
//   va_list vl;

//   str=first;

//   va_start(vl,first);

//   do {
//     puts(str);
//     str=va_arg(vl,char*);
//   } while (str!=NULL);

//   va_end(vl);
// }

// int main ()
// {
//   PrintLines ("First","Second","Third","Fourth",NULL);
//   return 0;
// }