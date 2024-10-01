const SteeringLimits RIVIAN_STEERING_LIMITS = {
  .angle_deg_to_can = 10,
  .max_angle_error = 2,
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {5., .8, .15}
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {5., 3.5, .4}
  },
};

const LongitudinalLimits RIVIAN_LONG_LIMITS = {
  .max_accel = 200,       // 2. m/s^2
  .min_accel = -350,       // -3.50 m/s^2
  .inactive_accel = 0,
};

const int FLAG_RIVIAN_LONG_CONTROL = 1;

const CanMsg RIVIAN_TX_MSGS[] = {
  {0x110, 0, 8},  // ACM_SteeringControl
  {0x160, 0, 5},  // ACM_longitudinalRequest
  // {0x162, 2, 8},  // ACM_AdasSts
};

RxCheck rivian_rx_checks[] = {
  {.msg = {{0x390, 0, 7, .frequency = 100U}, { 0 }, { 0 }}},  // EPAS_AdasStatus (steering angle)
  {.msg = {{0x38b, 0, 6, .frequency = 50U}, { 0 }, { 0 }}},   // ESPiB1 (speed)
  {.msg = {{0x150, 0, 7, .frequency = 50U}, { 0 }, { 0 }}},   // VDM_PropStatus (gas pedal)
  {.msg = {{0x38f, 0, 6, .frequency = 50U}, { 0 }, { 0 }}},   // iBESP2 (brakes)
  {.msg = {{0x162, 0, 8, .frequency = 100U}, { 0 }, { 0 }}},  // VDM_AdasSts
  {.msg = {{0x100, 2, 8, .frequency = 100U}, { 0 }, { 0 }}},  // ACM_Status (cruise state)
  {.msg = {{0x101, 2, 8, .frequency = 100U}, { 0 }, { 0 }}},  // ACM_AebRequest (aeb)
  {.msg = {{0x160, 2, 5, .frequency = 100U}, { 0 }, { 0 }}},  // ACM_longitudinalRequest (cruise control)
};

bool rivian_longitudinal = false;
bool rivian_stock_aeb = false;

static void rivian_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if(bus == 0) {

    // Steering angle
    if(addr == 0x390) {
      // Angle: (0.1 * val) - 819.2 in deg.
      // Store it 1/10 deg to match steering request
      int angle_meas_new = ((GET_BYTE(to_push, 5) << 6) | (GET_BYTE(to_push, 6) >> 2)) - 8192U;
      update_sample(&angle_meas, angle_meas_new);
    }

    // Vehicle speed
    if(addr == 0x38b){
      float speed = (GET_BYTE(to_push, 2)  * 0.4);
      vehicle_moving = ABS(speed) > 0.1;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // Gas pressed
    if(addr == 0x150){
      gas_pressed = (((GET_BYTE(to_push, 3) << 2) | (GET_BYTE(to_push, 4) >> 6)) != 0U);
    }

    // Brake pressed
    if(addr == 0x38f){
      brake_pressed = GET_BIT(to_push, 23U);
    }
  }

  if (bus == 2) {

    // Cruise state
    if(addr == 0x100) {
      bool cruise_engaged = (((GET_BYTE(to_push, 2)) >> 5) == 2U);
      pcm_cruise_check(cruise_engaged);
    }

    if (rivian_longitudinal && (addr == 0x101)) {
      // "AEB_ACTIVE"
      rivian_stock_aeb = GET_BIT(to_push, 47U);
    }
  }

  if (rivian_longitudinal) {
      generic_rx_checks((addr == 0x110) && (bus == 0));
  }

  generic_rx_checks((addr == 0x160) && (bus == 0));
}


static bool rivian_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  // Steering control
  if(addr == 0x110) {
    // angle: (0.1 * val) - 1638.35 in deg.
    // We use 1/10 deg as a unit here
    int raw_angle_can = (((GET_BYTE(to_send, 2)) << 7) | ((GET_BYTE(to_send, 3) & 0xfeU) >> 1));
    int desired_angle = raw_angle_can - 16384;
    bool steer_control_enabled = ((GET_BYTE(to_send, 1) & 0x30U) >> 4);
    if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, RIVIAN_STEERING_LIMITS)) {
      violation = true;
    }
  }

  // Longitudinal control
  if(addr == 0x160) {
    if (rivian_longitudinal) {

      // Don't send messages when the stock AEB system is active
      if (rivian_stock_aeb) {
        violation = true;
      }

      // Don't allow any acceleration limits above the safety limits
      int raw_accel = ((GET_BYTE(to_send, 2) << 3) | (GET_BYTE(to_send, 3) >> 5)) - 1024U;
      violation |= longitudinal_accel_checks(raw_accel, RIVIAN_LONG_LIMITS);
    } else {
      violation = true;
    }
  }

  return tx;
}

static int rivian_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  bool block_msg = false;

  if(bus_num == 0) {
    // ACM_AdasSts
    // if (addr == 0x162) {
    //   block_msg = true;
    // }

    if(!block_msg) {
      bus_fwd = 2;
    }
  }

  if(bus_num == 2) {
    // ACM_SteeringControl
    if (addr == 0x110) {
      block_msg = true;
    }

    // ACM_longitudinalRequest
    if (rivian_longitudinal && (addr == 0x160) && !rivian_stock_aeb) {
      block_msg = true;
    }

    if(!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config rivian_init(uint16_t param) {
  rivian_longitudinal = GET_FLAG(param, FLAG_RIVIAN_LONG_CONTROL);
  rivian_stock_aeb = false;

  safety_config ret;
  ret = BUILD_SAFETY_CFG(rivian_rx_checks, RIVIAN_TX_MSGS);
  return ret;
}

const safety_hooks rivian_hooks = {
  .init = rivian_init,
  .rx = rivian_rx_hook,
  .tx = rivian_tx_hook,
  .fwd = rivian_fwd_hook,
};
