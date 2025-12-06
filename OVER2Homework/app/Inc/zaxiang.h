#ifndef __ZAXIANG_
#define __ZAXIANG_


typedef struct {
    uint32_t system_tick;
    uint32_t can_tx_count;
    uint32_t can_rx_count;
    uint32_t last_error;
    uint8_t  test_phase;
    uint8_t  any_motor_online;
    uint32_t phase_start_time;
} Debug_Info_t;

#endif
