#define RIVANNA2_MPPT_COMMANDS_180_LENGTH (8u)
#define RIVANNA2_MPPT_COMMANDS_280_LENGTH (8u)
#define RIVANNA2_MPPT_COMMANDS_480_LENGTH (4u)

#define RIVANNA2_MPPT_COMMANDS_180_ID (0x180u)
#define RIVANNA2_MPPT_COMMANDS_280_ID (0x280u)
#define RIVANNA2_MPPT_COMMANDS_480_ID (0x480u)

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>



/*
 Data from CAN message in MPPTCANStruct
*/
struct rivanna2_mppt_commands_180_t {

    //float, in mA
    float current_in;

    //float in V
    float voltage_in;

};

struct rivanna2_mppt_commands_280_t {

    //float in V
    float voltage_out; 

    //float in mW
    float power_in;

};

struct rivanna2_mppt_commands_480_t {

    //SN16 in 0.01 C
    uint16_t temperature_pcb;

    //SN16 in 0.01 C
    uint16_t temperature_mosfet;
};

int rivanna2_mppt_commands_180_unpack(
    const struct rivanna2_mppt_commands_180_t *dst_p,
    uint8_t *src_p,
    size_t size);

int rivanna2_mppt_commands_280_unpack(
    const struct rivanna2_mppt_commands_280_t *dst_p,
    uint8_t *src_p,
    size_t size);
    
int rivanna2_mppt_commands_480_unpack(
    const struct rivanna2_mppt_commands_480_t *dst_p,
    uint8_t *src_p,
    size_t size);

    
