//
// Created by 14483 on 2025/11/14.
//
#include "main.h"
#include "tx_api.h"
#include "appstart.hpp"

extern TX_THREAD my_thread;
extern uint8_t my_thread_stack[1024];
extern void my_thread_entry(ULONG initial_input);


#define TX_NAME(s) const_cast<CHAR*>(s)

void app_task_start(void){
    tx_thread_create(&my_thread, TX_NAME("my_thread1"),
    my_thread_entry, 0x1234, my_thread_stack, sizeof(my_thread_stack),
    10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

}

