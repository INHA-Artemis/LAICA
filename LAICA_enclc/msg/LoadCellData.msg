std_msgs/Header header
int32   raw_count    # 24-bit signed ADC count (AIN0 - AIN1 differential)
float64 voltage_mv   # Differential voltage [mV]
float64 force_n      # Calibrated force [N]  (= force_gradient_n_per_mv * voltage_mv + force_bias_n)
