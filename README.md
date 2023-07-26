# Description
This project is concerned with initiating a Piezo buzzer at startup with a single frequency, 880 Hz, tone for 1 sec. A push button is implemented as an external interrupt that will play the buzzer 
three consecutive tones of different frequencies relative to tones C6, D6, and E6 with 500 ms play time for each tone. The piezo is attached to the H-bridge driver and is driven with
complementary PWM signals. The H-bridge function and PWM buzzer logic functions are enabled with microcontroller GPIO pins. 

# Changes made in task requirements
Changed  **CPU_BUZZER_OUT2**   from **PA3** to **PG9** to get complementary PWM output form the same channel


# Implementation
The main routine generates a test tone at the startup. **GPIO PC5** is implemented as an Interrupt pin for the **PUSH BUTTON** interrupt. 
After the test tone is generated, the microcontroller is sent to **LOW POWER MODE**. The **LOW POWER MODE** is exited when the PUSH BUTTON interrupt is obtained. 
Getting the **PUSH BUTTON** interrupt, system peripherals are reinitialized, and one user-defined interrupt flag is set, which is later checked in the main routine 
to generate 3 tones with a 500ms delay. The flag is reset after the tones are played and the controller goes back to **LOW POWER MODE**


# PWM generation
PWM is geenratied using **TIMER15** channel. Complementary PWM is generated to drive both the membrane of the piezo for louder sound. 
**DEAD_TIME_INSERTION** is used to obtain a small time delay(~10 us) between the two PWM signals to avoid overshoot in the H-bridge.

