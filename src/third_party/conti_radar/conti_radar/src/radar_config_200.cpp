#include "radar_config_200.h"
#include "byte.h"


const uint32_t RadarConfig200::ID = 0x200;

RadarConfig200::RadarConfig200()
{

  radar_conf_.max_distance_valid=true;
  radar_conf_.sensor_id_valid=false;
  radar_conf_.radar_power_valid=false;
  radar_conf_.output_type_valid=true;
  radar_conf_.send_quality_valid=true;
  radar_conf_.send_ext_info_valid=true;
  radar_conf_.sort_index_valid=false;
  radar_conf_.store_in_nvm_valid=true;
  radar_conf_.ctrl_relay_valid=false;
  radar_conf_.rcs_threshold_valid=true;

  radar_conf_.max_distance=250;
  radar_conf_.sensor_id=0;
  radar_conf_.output_type=OUTPUT_TYPE_OBJECTS;
  radar_conf_.radar_power=0;
  radar_conf_.ctrl_relay=0;
  radar_conf_.send_ext_info=1;
  radar_conf_.send_quality=1;
  radar_conf_.sort_index==0;
  radar_conf_.store_in_nvm=1;
  radar_conf_.rcs_threshold=RCS_THRESHOLD_STANDARD;

}

RadarConfig200::~RadarConfig200()
 {

 }

uint32_t RadarConfig200::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void RadarConfig200::UpdateData(uint8_t* data) {
  set_max_distance_valid_p(data, radar_conf_.max_distance_valid);
  set_sensor_id_valid_p(data, radar_conf_.sensor_id_valid);
  set_radar_power_valid_p(data, radar_conf_.radar_power_valid);
  set_output_type_valid_p(data, radar_conf_.output_type_valid);
  set_send_quality_valid_p(data, radar_conf_.send_quality_valid);
  set_send_ext_info_valid_p(data, radar_conf_.send_ext_info_valid);
  set_sort_index_valid_p(data, radar_conf_.sort_index_valid);
  set_store_in_nvm_valid_p(data, radar_conf_.store_in_nvm_valid);
  set_ctrl_relay_valid_p(data, radar_conf_.ctrl_relay_valid);
  set_rcs_threshold_valid_p(data, radar_conf_.rcs_threshold_valid);

  set_max_distance_p(data, radar_conf_.max_distance);
  set_sensor_id_p(data, radar_conf_.sensor_id);
  set_output_type_p(data, radar_conf_.output_type);
  set_radar_power_p(data, radar_conf_.radar_power);
  set_ctrl_relay_p(data, radar_conf_.ctrl_relay);
  set_send_ext_info_p(data, radar_conf_.send_ext_info);
  set_send_quality_p(data, radar_conf_.send_quality);
  set_sort_index_p(data, radar_conf_.sort_index);
  set_store_in_nvm_p(data, radar_conf_.store_in_nvm);
  set_rcs_threshold_p(data, radar_conf_.rcs_threshold);
}

/**
 * @brief reset the private variables
 */
void RadarConfig200::Reset() {
  radar_conf_.max_distance_valid=false;
  radar_conf_.sensor_id_valid=false;
  radar_conf_.radar_power_valid=false;
  radar_conf_.output_type_valid=true;
  radar_conf_.send_quality_valid=true;
  radar_conf_.send_ext_info_valid=true;
  radar_conf_.sort_index_valid=false;
  radar_conf_.store_in_nvm_valid=true;
  radar_conf_.ctrl_relay_valid=false;
  radar_conf_.rcs_threshold_valid=true;

  radar_conf_.max_distance=125;
  radar_conf_.sensor_id=0;
  radar_conf_.output_type=OUTPUT_TYPE_NONE;
  radar_conf_.radar_power=0;
  radar_conf_.ctrl_relay=0;
  radar_conf_.send_ext_info=1;
  radar_conf_.send_quality=1;
  radar_conf_.sort_index==0;
  radar_conf_.store_in_nvm=1;
  radar_conf_.rcs_threshold=RCS_THRESHOLD_STANDARD;
}


void RadarConfig200::set_max_distance_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void RadarConfig200::set_sensor_id_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_bit_1(1);
  } else {
    frame.set_bit_0(1);
  }
}

void RadarConfig200::set_radar_power_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 2, 1);
  } else {
    frame.set_value(0, 2, 1);
  }
}

void RadarConfig200::set_output_type_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 3, 1);
  } else {
    frame.set_value(0, 3, 1);
  }
}

void RadarConfig200::set_send_quality_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 4, 1);
  } else {
    frame.set_value(0, 4, 1);
  }
}

void RadarConfig200::set_send_ext_info_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 5, 1);
  } else {
    frame.set_value(0, 5, 1);
  }
}

void RadarConfig200::set_sort_index_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 6, 1);
  } else {
    frame.set_value(0, 6, 1);
  }
}

void RadarConfig200::set_store_in_nvm_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 7, 1);
  } else {
    frame.set_value(0, 7, 1);
  }
}

void RadarConfig200::set_ctrl_relay_valid_p(uint8_t* data, bool valid) {
  Byte frame(data + 5);
  if (valid) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void RadarConfig200::set_rcs_threshold_valid_p(uint8_t* data, bool valid) {
  Byte frame(data + 6);
  if (valid) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void RadarConfig200::set_max_distance_p(uint8_t* data, uint16_t value) {
  value /= 2;
  uint8_t low = value >> 2;
  Byte frame_low(data + 1);
  frame_low.set_value(low, 0, 8);

  uint8_t high = value << 6;
  high &= 0xc0;
  Byte frame_high(data + 2);
  frame_high.set_value(high, 0, 8);
}

void RadarConfig200::set_sensor_id_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 4);
  frame.set_value(value, 0, 3);
}

void RadarConfig200::set_output_type_p(uint8_t* data, OutputType type) {
  Byte frame(data + 4);
  uint8_t value = static_cast<uint8_t>(type);
  frame.set_value(value, 3, 2);
}

void RadarConfig200::set_radar_power_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 4);
  frame.set_value(value, 5, 3);
}

void RadarConfig200::set_ctrl_relay_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 1, 1);
}

void RadarConfig200::set_send_ext_info_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 3, 1);
}

void RadarConfig200::set_send_quality_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 2, 1);
}

void RadarConfig200::set_sort_index_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 4, 3);
}

void RadarConfig200::set_store_in_nvm_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 7, 1);
}

void RadarConfig200::set_rcs_threshold_p(uint8_t* data,
                                         RcsThreshold rcs_threshold) {
  Byte frame(data + 6);
  uint8_t value = static_cast<uint8_t>(rcs_threshold);
  frame.set_value(value, 1, 3);
}
