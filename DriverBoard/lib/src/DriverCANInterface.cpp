#include "DriverCANInterface.h"
#include "MotorControllerCANStructs.h"
#include "log.h"

DriverCANInterface::DriverCANInterface(PinName rd, PinName td, PinName standby_pin)
    : CANInterface(rd, td, standby_pin) {
    can.frequency(250000);
}


int DriverCANInterface::send(CANStruct *can_struct) {
    CANMessage message;
    can_struct->serialize(&message);
    message.id = can_struct->get_message_ID();
    int result = can.write(message);

    char message_data[17];

    CANInterface::write_CAN_message_data_to_buffer(message_data, &message);

    //send_to_pi(message.id, message_data);

    if (result == 1) {  
        log_debug("Sent CAN message with ID 0x%03X Length %d Data 0x%s",
                  message.id, message.len, message_data);
    } else {
        // this error logging requires changes to mbed-os. make the _can field in CAN.h public instead of private
        // log_error("%d", HAL_FDCAN_GetError(&can._can.CanHandle));
        log_error(
            "Failed to send CAN message with ID 0x%03X Length %d Data 0x%s",
            message.id, message.len, message_data);
        
    }

    return result;
}

void DriverCANInterface::message_handler() {
    while (true) {
        ThisThread::flags_wait_all(0x1);
        CANMessage message;
        while (can.read(message)) {
            char message_data[17];

            //TODO: Write to serial message_id, message_data

            CANInterface::write_CAN_message_data_to_buffer(message_data,
                                                           &message);
            // send_to_pi(message.id, message_data);

            log_debug("Received CAN message with ID 0x%03X Length %d Data 0x%s ", message.id, message.len, message_data);
            if (message.id == BPSError_MESSAGE_ID) {
                BPSError can_struct;
                can_struct.deserialize(&message);
                handle(&can_struct);
                // uint32_t err_to_pi = message_data[0];
                // err_to_pi <<= 1;
                // err_to_pi |= message_data[1];
                // err_to_pi <<= 1;
                // err_to_pi |= message_data[2];
                // fprintf(stderr, "bms_fault %d\n", err_to_pi);
                // fflush(stderr);
                lock.lock();
                fprintf(stderr, "internal_communications_fault %d\n", can_struct.internal_communications_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "internal_conversion_fault %d\n", can_struct.internal_conversion_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "weak_cell_fault %d\n", can_struct.weak_cell_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "low_cell_voltage_fault %d\n", can_struct.low_cell_voltage_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "open_wiring_fault %d\n", can_struct.open_wiring_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "current_sensor_fault %d\n", can_struct.current_sensor_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "pack_voltage_sensor_fault %d\n", can_struct.pack_voltage_sensor_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "weak_pack_fault %d\n", can_struct.weak_pack_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "voltage_redundancy_fault %d\n", can_struct.voltage_redundancy_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "fan_monitor_fault %d\n", can_struct.fan_monitor_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "thermistor_fault %d\n", can_struct.thermistor_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "canbus_communications_fault %d\n", can_struct.canbus_communications_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "always_on_supply_fault %d\n", can_struct.always_on_supply_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "high_voltage_isolation_fault %d\n", can_struct.high_voltage_isolation_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "power_supply_12v_fault %d\n", can_struct.power_supply_12v_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "charge_limit_enforcement_fault %d\n", can_struct.charge_limit_enforcement_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "discharge_limit_enforcement_fault %d\n", can_struct.discharge_limit_enforcement_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "charger_safety_relay_fault %d\n", can_struct.charger_safety_relay_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "internal_memory_fault %d\n", can_struct.internal_memory_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "internal_thermistor_fault %d\n", can_struct.internal_thermistor_fault ? 1 : 0);
                fflush(stderr);
                fprintf(stderr, "internal_logic_fault %d\n", can_struct.internal_logic_fault ? 1 : 0);
                fflush(stderr);
                lock.unlock();
            } else if (message.id == MotorControllerPowerStatus_MESSAGE_ID || message.id == MotorControllerPowerStatus_AUX_BUS_MESSAGE_ID) {
                MotorControllerPowerStatus can_struct;
                can_struct.deserialize(&message);
                lock.lock();
                fprintf(stderr, "motor_rpm %d\n", can_struct.motor_rpm);
                fflush(stderr);
                lock.unlock();
                handle(&can_struct);
            } else if(message.id == ECUPowerAuxCommands_MESSAGE_ID) {
                ECUPowerAuxCommands can_struct;
                can_struct.deserialize(&message);
                if(can_struct.headlights) {
                  lock.lock();
                  fprintf(stderr, "other_error %d\n", can_struct.hazards ? 1 : 0);
                  fflush(stderr);
                  lock.unlock();
                }
                handle(&can_struct);
            } else if(message.id == BPSPackInformation_MESSAGE_ID) {
                BPSPackInformation can_struct;
                can_struct.deserialize(&message);
                lock.lock();
                fprintf(stderr, "pack_voltage %d\n", can_struct.pack_voltage);
                fflush(stderr);
                fprintf(stderr, "pack_current %d\n", can_struct.pack_current);
                fflush(stderr);
                fprintf(stderr, "pack_soc %d\n", can_struct.pack_soc);
                fflush(stderr);
                lock.unlock();
            } else if(message.id == BPSCellTemperature_MESSAGE_ID) {
                BPSCellTemperature can_struct;
                can_struct.deserialize(&message);
                lock.lock();
                fprintf(stderr, "tmp %d\n", can_struct.high_temperature);
                fflush(stderr);
                lock.unlock();
            }
        }
    }
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint8_t unpack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) << shift);
}

int motor_commands_rate_divisor = 30;
int curr_motor_commands_ct = 0;

void DriverCANInterface::send_to_pi(int id, char* data) {
  switch(id) {
    case (1030): {
      int pack_voltage = ((data[1]<<8) | data[0]);
      fprintf(stderr, "pack_voltage %d\n", pack_voltage);
      int16_t pack_current = unpack_right_shift_u16(data[2], 0u, 0xffu);
      pack_current |= unpack_left_shift_u16(data[3], 8u, 0xffu);
      fprintf(stderr, "pack_current %d\n", pack_current);
      // uint8_t byte2 = data[2];
      // uint8_t byte3 = data[3];
      // fprintf(stderr, "byte 2: %d, byte 3: %d\n", byte2, byte3);
      break;
    }
    case (805): {
      int motor_rpm = unpack_right_shift_u16(data[4], 3u, 0xf8u);
      motor_rpm |= unpack_left_shift_u16(data[5], 5u, 0x7fu);
      fprintf(stderr, "motor_rpm %d\n", motor_rpm);
      break;
    }
    case(0x08850225): {
      int motor_rpm = unpack_right_shift_u16(data[4], 3u, 0xf8u);
      motor_rpm |= unpack_left_shift_u16(data[5], 5u, 0x7fu);
      fprintf(stderr, "motor_rpm %d\n", motor_rpm);
      break;
    }
    case (1062): {
      int high_cell_tmp = data[2];
      fprintf(stderr, "tmp %d\n", high_cell_tmp);
      break;
    }
    case(513): {
      ++curr_motor_commands_ct;
      if(curr_motor_commands_ct < motor_commands_rate_divisor) {
        break;
      }
      curr_motor_commands_ct = 0;
      // int throttle = unpack_right_shift_u16(data[0], 0u, 0xffu);
      // throttle |= unpack_left_shift_u16(data[1], 8u, 0x01u);
      // fprintf(stderr, "throttle %d\n", throttle);
      int regen = unpack_right_shift_u16(data[1], 1u, 0xfeu);
      regen |= unpack_left_shift_u16(data[2], 7u, 0x03u);
      fprintf(stderr, "regen %d\n", regen);
      int cruise_control_speed = unpack_right_shift_u8(data[2], 2u, 0xfcu);
      cruise_control_speed |= unpack_left_shift_u8(data[3], 6u, 0x03u);
      fprintf(stderr, "cc_speed %d\n", cruise_control_speed);
      int cruise_control_en = (data[3]>>2)&1;
      fprintf(stderr, "cc_en %d\n", cruise_control_en);
      break;
    }
    case(262): {
      uint32_t bms_out = data[2];
      bms_out = (bms_out<<8) | data[1];
      bms_out = (bms_out<<8) | data[0];
      fprintf(stderr, "bms_fault %d\n", bms_out);

      break;
    }
    case(769): {
      uint8_t headlights = (data[0]>>2)&1;
      uint8_t hazards = data[0]&1;
      if(headlights) {
        // Sent from battery board
        // Hazards represent error value
        fprintf(stderr, "other_error %d\n", hazards);
        break;
      }
      fprintf(stderr, "hazards %d\n", hazards);
      uint8_t left_turn = (data[0]>>3)&1;
      fprintf(stderr, "left_turn %d\n", left_turn);
      uint8_t right_turn = (data[0]>>4)&1;
      fprintf(stderr, "right_turn %d\n", right_turn);
      break;
    }
    default: {
      fprintf(stderr, "unknown can msg id: %d\n", id);
      break;
    }
    }
}