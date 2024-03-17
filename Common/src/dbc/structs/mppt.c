#include <string.h>

#include "mppt.h"


int rivanna2_mppt_commands_pack(
    uint8_t *dst_p,
    const struct rivanna2_mppt_commands_t *src_p,
    size_t size)
{

    memset(&dst_p[0], 0, 6); //6 for 6 values? idk man

    //something like this I assume? that will shift and get the values from the CAN bytes msg

 

    // dst_p[0] |= pack_left_shift_u16(src_p->panel1_photo, 0u, 0xffu);
    // dst_p[1] |= pack_right_shift_u16(src_p->panel1_photo, 8u, 0xffu);
    // dst_p[2] |= pack_left_shift_u16(src_p->panel2_photo, 0u, 0xffu);
    // dst_p[3] |= pack_right_shift_u16(src_p->panel2_photo, 8u, 0xffu);
    // dst_p[4] |= pack_left_shift_u16(src_p->panel3_photo, 0u, 0xffu);
    // dst_p[5] |= pack_left_shift_u16(src_p->panel3_photo, 0u, 0xffu);


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
}
