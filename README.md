# Project Name
nimmsta_task

# Description
This project is concerned with initiating a Piezo buzzer at startup with a single frequency, 880 Hz, tone for 1 sec. A push button is implemented as an external interrupt that will play the buzzer 
three consecutive tones of different frequencies relative to tones C6, D6, and E6 with 500 ms play time for each tone. The piezo is attached to the H-bridge driver and is driven with
complementary PWM signals. The H-bridge function and PWM buzzer logic functions are enabled with microcontroller GPIO pins. 
