#ifndef RADAR_CONFIG_200_H_
#define RADAR_CONFIG_200_H_
#include "protocol_data.h"
#include "conti_radar.hpp"
class RadarConfig200
    : public ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  RadarConfig200();
  ~RadarConfig200();
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  void UpdateData(uint8_t* data) override;

  /**
   * @brief reset the private variables
   */
  void Reset() override;

  void set_max_distance_valid_p(uint8_t* data, bool valid);
  void set_sensor_id_valid_p(uint8_t* data, bool valid);
  void set_radar_power_valid_p(uint8_t* data, bool valid);
  void set_output_type_valid_p(uint8_t* data, bool valid);
  void set_send_quality_valid_p(uint8_t* data, bool valid);
  void set_send_ext_info_valid_p(uint8_t* data, bool valid);
  void set_sort_index_valid_p(uint8_t* data, bool valid);
  void set_store_in_nvm_valid_p(uint8_t* data, bool valid);
  void set_ctrl_relay_valid_p(uint8_t* data, bool valid);
  void set_rcs_threshold_valid_p(uint8_t* data, bool valid);

  void set_max_distance_p(uint8_t* data, uint16_t value);
  void set_sensor_id_p(uint8_t* data, uint8_t value);
  void set_output_type_p(uint8_t* data, OutputType type);
  void set_radar_power_p(uint8_t* data, uint8_t value);
  void set_ctrl_relay_p(uint8_t* data, uint8_t value);
  void set_send_ext_info_p(uint8_t* data, uint8_t value);
  void set_send_quality_p(uint8_t* data, uint8_t value);
  void set_sort_index_p(uint8_t* data, uint8_t value);
  void set_store_in_nvm_p(uint8_t* data, uint8_t value);
  void set_rcs_threshold_p(uint8_t* data, RcsThreshold rcs_theshold);

 private:
  RadarConf radar_conf_;
};
#endif  // RADAR_CONFIG_200_H_
