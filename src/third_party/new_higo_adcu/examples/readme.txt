1 can_test<channelNumber><BaudRate><IDE>
./can_test 1 1 1

2 rs232_test
./rs232_test

3 adc_test<channelNum>
./adc_test 1

4 dac_test<channelNum,voltage_mv>
./dac_test 1 3500

5 pwm_in_test<channelNum>
./pwm_in_test 1

6 pwm_out_test<channelNum,frequency,dutyCycle>
./pwm_out_test 1 1000 30

7 highside_test<channelNum,state>
./highside_test 1 1

8 lin_test<channelNum,date_dir>
./lin_test 1 0

9 power_test
./power_test
