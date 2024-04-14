#include <string.h>

#include "mppt.h"
#include "MPPTCANStruct.h"

#ifndef EINVAL
#    define EINVAL 22
#endif


int rivanna2_mppt_commands_180_unpack(
    const struct rivanna2_mppt_commands_180_t *dst_p,
    uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(dst_p, 0, sizeof(struct rivanna2_mppt_commands_180_t));

    memcpy(&dst_p->current_in, src_p, 4);
    memcpy(&dst_p->voltage_in, src_p + 4, 4);

    return (0);
}

int rivanna2_mppt_commands_280_unpack(
    const struct rivanna2_mppt_commands_280_t *dst_p,
    uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(dst_p, 0, sizeof(struct rivanna2_mppt_commands_280_t));

    memcpy(&dst_p->voltage_out, src_p, 4);
    memcpy(&dst_p->power_in, src_p + 4, 4);

    return (0);
}

int rivanna2_mppt_commands_480_unpack(
    const struct rivanna2_mppt_commands_480_t *dst_p,
    uint8_t *src_p,
    size_t size)
{
    memset(dst_p, 0, sizeof(struct rivanna2_mppt_commands_480_t));

    memcpy(&dst_p->temperature_pcb, src_p, 2);
    memcpy(&dst_p->temperature_mosfet, src_p + 2, 2);

    return (0);
}


       /*
    Identifier: 0x180
    bytes 0-3 current in
    bytes 4-7 voltage in 
    
    Identifier: 0x280
    bytes 0-3 voltage out
    bytes 4-7 power in 

    Identifier: 0x480
    bytes 0-1 temp pcb
    bytes 2-3 temp temp mosfet
    */
