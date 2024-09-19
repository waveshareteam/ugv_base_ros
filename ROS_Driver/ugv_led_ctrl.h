void led_pin_init(){
  ledcAttach(IO4_PIN, FREQ, ANALOG_WRITE_BITS);
  ledcAttach(IO5_PIN, FREQ, ANALOG_WRITE_BITS);
}

void led_pwm_ctrl(int io4Input, int io5Input) {
  ledcWrite(IO4_PIN, constrain(io4Input, 0, 255));
  ledcWrite(IO5_PIN, constrain(io5Input, 0, 255));
}