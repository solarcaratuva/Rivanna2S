#define RIVANNA2_MPPT_COMMANDS_180_LENGTH (8u)
#define RIVANNA2_MPPT_COMMANDS_280_LENGTH (8u)
#define RIVANNA2_MPPT_COMMANDS_480_LENGTH (4u)

#define RIVANNA2_MPPT_COMMANDS_180_ID (0x180u)
#define RIVANNA2_MPPT_COMMANDS_280_ID (0x280u)
#define RIVANNA2_MPPT_COMMANDS_480_ID (0x480u)


/*
 Data from CAN message in MPPTCANStruct
*/
struct rivanna2_mppt_commands_180 {

    //float, in mA
    float current_in;

    //float in V
    float voltage_in;

}

struct rivanna2_mppt_commands_280 {

    //float in V
    float voltage_out; 

    //float in mW
    float power_in;

}

struct rivanna2_mppt_commands_480 {

    //SN16 in 0.01 C
    uint16_t temperature_pcb;

    //SN16 in 0.01 C
    uint16_t temperature_mosfet;
}

int rivanna2_mppt_commands_180_pack(
    uint8_t *dst_p,
    const struct rivanna2_mppt_commands_180_t *src_p,
    size_t size);

int rivanna2_mppt_commands_280_pack(
    uint8_t *dst_p,
    const struct rivanna2_mppt_commands_280_t *src_p,
    size_t size);
    
int rivanna2_mppt_commands_480_pack(
    uint8_t *dst_p,
    const struct rivanna2_mppt_commands_480_t *src_p,
    size_t size);

    
