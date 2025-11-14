//
// Created by 14483 on 2025/11/14.
//
#include "tx_api.h"
#include "main.h"

TX_THREAD my_thread;
uint8_t my_thread_stack[1024];


[[noreturn]] void my_thread_entry(ULONG initial_input)
{
    UNUSED(initial_input);//传入的这个没用

    //在这里你可以做一些复杂任务的初始化，比如说电机的init，etc.

    /* Enter into a forever loop. */
    while(1)
    {
        //LED_blink();//just an example, implement it yourself
        /* Sleep for 1 tick. */
        tx_thread_sleep(1000);

    }
}