/*
 Data from CAN message in MPPTCANStruct
*/
struct rivanna2_mppt_commands_180 {

    //float, in mA
    float current_in;

    //float in V
    uint32_t voltage_in;

}

struct rivanna2_mppt_commands_280 {

    //float in V
    uint32_t voltage_out; 

    //float in mW
    uint32_t power_in;

}

struct rivanna2_mppt_commands_480 {

    //SN16 in 0.01 C
    uint16_t temperature_pcb;

    //SN16 in 0.01 C
    uint16_t temperature_mosfet;

}
    

    
