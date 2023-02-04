#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "encoders.pio.h"
#include "hardware/adc.h"
#include "lib/I2C_comm.h"
#include "lib/vcnl4040.h"
#include "lib/servo.h"
#include "lib/dynamixel/XL330.h"
#include "lib/ads1115.h"

#define SmartMode 1
#define RCMode 0
#define ADS1115_I2C_ADDR 0x48
int directions[num_of_axis] = {z_dir};

uint encoder_pin[num_of_axis] = {2};
uint number_of_encoders = num_of_axis;
uint encoder_resoultion[num_of_axis] = {encoder_PPR * 4};

float volts0, volts1, volts2, volts3;
uint16_t adc_value0, adc_value1, adc_value2, adc_value3;

float encoder_mm_per_revolution[num_of_axis] = {Z_LEADSCREW_PITCH};
volatile float encoder_mm_per_click[num_of_axis];
volatile float axis_offset[num_of_axis] = {0}, added_offset[num_of_axis] = {0};
float homing_position[num_of_axis] = {2}; // the absolute position of the loader when it is homed
volatile int32_t capture_buf[num_of_axis] = {0};

// Set up the state machine.
PIO pio = pio0;
PIO pio2 = pio1;
struct ads1115_adc adc;

// vars
int continuousModeOn = false;
volatile int continuousModeOn_marlinsig = false;
int imageCounter = 0;
int LED_intensity, servo_angle;
uint32_t servo_current_consumption = 0, temp_current_consumption = 0, servo_current_integrate = 1, servo_counter;
bool servo_flag = true;

void encoders_init()
{
    uint offset = pio_add_program(pio, &encoders_program);
    uint offset2 = pio_add_program(pio2, &encoders_program);
    char c;

    for (int i = 0; i < number_of_encoders; i++)
    {
        if (i < 4)
        {
            encoders_program_init(pio, i, offset, encoder_pin[i], false);
        }
        else
        {
            encoders_program_init(pio2, i % 4, offset2, encoder_pin[i], false);
        }
        dma_channel_config c = dma_channel_get_default_config(i);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);

        if (i < 4)
        {
            channel_config_set_dreq(&c, pio_get_dreq(pio, i, false));
            dma_channel_configure(i, &c,
                                  &capture_buf[i], // Destinatinon pointer
                                  &pio->rxf[i],    // Source pointer
                                  0xffff,          // Number of transfers
                                  true             // Start immediately
            );
        }
        else
        {
            channel_config_set_dreq(&c, pio_get_dreq(pio2, i % 4, false));
            dma_channel_configure(i, &c,
                                  &capture_buf[i],   // Destinatinon pointer
                                  &pio2->rxf[i % 4], // Source pointer
                                  0xffff,            // Number of transfers
                                  true               // Start immediately
            );
        }
        irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        dma_channel_set_irq0_enabled(i, true);
    }
}

void current_check()
{
    if (servo_current_integrate < 100)
    {
        temp_current_consumption += adc_read();
        servo_current_integrate++;
        // printf_debug("consumption: %d current_counter: %d\n", servo_current_consumption, servo_current_integrate);
    }
    else
    {
        servo_current_consumption = temp_current_consumption / servo_current_integrate;
        temp_current_consumption = 0;
        servo_current_integrate = 1;
    }

    if (servo_counter > 50)
        servo_flag = true;
    else
        servo_counter++;

    if (servo_flag)
    {
        if (servo_current_consumption > 1700)
        {
            gpio_put(boost_enable, false);
            // if (servo_angle >2) servo_angle--;
            // else servo_angle+=2;
            // servo_set_position(servo_pin, servo_angle);
            // printf("%d\n", servo_angle);
            // servo_counter = 0;
            servo_flag = false;
        }
    }
}

void GPIO_init(void)
{
    // Strobing Pins initialisations
    gpio_init(boost_enable);
    gpio_init(status_led_pin);
    gpio_init(led_sig);
    gpio_init(led_pin);
    gpio_set_dir(boost_enable, GPIO_OUT);
    gpio_set_dir(status_led_pin, GPIO_OUT);
    gpio_set_dir(led_sig, GPIO_OUT);

    // Interrupt pin declarations
    gpio_pull_up(boost_enable);

    // Strobing Pins initialisation
    gpio_set_function(led_pin, GPIO_FUNC_PWM);
    pwm_set_enabled(pwm_gpio_to_slice_num(led_pin), true);
    pwm_set_freq_duty(led_pin, 2000, 0);

    // Initially led_pin is in off state
    gpio_put(led_pin, LOW);
    gpio_put(status_led_pin, LOW);
    gpio_put(led_sig, LOW);
    gpio_put(boost_enable, LOW);
}

int main()
{
    stdio_init_all();

    for (int i = 0; i < num_of_axis; i++)
    {
        encoder_mm_per_click[i] = encoder_mm_per_revolution[i] / encoder_resoultion[i];
    }

    encoders_init();

    int16_t proxValue, ambientValue, whiteValue;
    axis_offset[0] = 0;
    axis_offset[1] = 0;
    axis_offset[2] = 0;

    i2c_begin(i2c0, I2C_Baudrate, i2c0_scl_pin, i2c0_sda_pin);
    i2c_begin(i2c1, I2C_Baudrate, i2c1_scl_pin, i2c1_sda_pin);

    ads1115_init(i2c0, ADS1115_I2C_ADDR, &adc);
    ads1115_set_pga(ADS1115_PGA_4_096, &adc);
    ads1115_set_operating_mode(ADS1115_MODE_SINGLE_SHOT, &adc);
    ads1115_set_data_rate(ADS1115_RATE_475_SPS, &adc);

    ads1115_write_config(&adc);
    GPIO_init();

    // servo_enable(servo_pin);
    while (1)
    {
        ads1115_set_input_mux(ADS1115_MUX_SINGLE_0 , &adc);
        ads1115_write_config(&adc);
        sleep_ms(25);
        ads1115_read_adc(&adc_value0, &adc);

        ads1115_set_input_mux(ADS1115_MUX_SINGLE_1, &adc);
        ads1115_write_config(&adc);
        sleep_ms(25);
        ads1115_read_adc(&adc_value1, &adc);

        ads1115_set_input_mux(ADS1115_MUX_SINGLE_2, &adc);
        ads1115_write_config(&adc);
        sleep_ms(25);
        ads1115_read_adc(&adc_value2, &adc);

        // ads1115_set_input_mux(ADS1115_MUX_SINGLE_3, &adc);
        // ads1115_write_config(&adc);
        // sleep_ms(25);
        // ads1115_read_adc(&adc_value3, &adc);

        volts0 = ads1115_raw_to_volts(adc_value0, &adc);
        volts1 = ads1115_raw_to_volts(adc_value1, &adc);
        volts2 = ads1115_raw_to_volts(adc_value2, &adc);
        volts3 = ads1115_raw_to_volts(adc_value3, &adc);

        printf("V0: %f\t", volts0);
        printf("V1: %f\t", volts1);
        printf("V2: %f\t", volts2);
        printf("V3: %f\n", volts3);

        sleep_ms(600);
    } 

    for (int i = 0; i < 4; i++)
    {
        axis_Union.axis_val[i] = flash_target_contents[0 + i];
    }
    if (axis_Union.axis_fval > 0 && axis_Union.axis_fval < 2000)
    {
        LED_intensity = axis_Union.axis_fval;
    }
    for (int i = 0; i < 4; i++)
    {
        axis_Union.axis_val[i] = flash_target_contents[4 + i];
    }
    if (axis_Union.axis_fval > 0 && axis_Union.axis_fval < 200)
    {
        servo_angle = axis_Union.axis_fval;
    }

    int angle = 0;
    // servo_set_position(servo_pin, angle);

    int servoID = 1;
    XL330 testservo(servoID, UART_RX, UART_TX, UART_DIR);
    sleep_ms(250);
    testservo.begin();
    while (!stdio_usb_connected())
    {
        gpio_put(status_led_pin, false); // For indication error that USB comm isnt happening
        sleep_ms(250);
        gpio_put(status_led_pin, true); // For indication error that USB comm isnt happening
        sleep_ms(250);
    }
    sleep_ms(250);
    testservo.configure();
    sleep_ms(250);
    if (testservo.getHardwareStatus() == 1)
    {
        printf("servo init done\n");
    }
    else
    {
        printf("servo init error\n");
    }
    watchdog_enable(5000, 1);
    while (stdio_usb_connected() == 0)
    {
        tight_loop_contents();
    }

    if (watchdog_caused_reboot())
    {
        printf("Last reboot by Watchdog!\n");
    }

    uint32_t timeout_power;
    while (true)
    {
        current_check();

        watchdog_update();
        if (stdio_usb_connected())
        {
            gpio_put(status_led_pin, true); // For indication error that USB comm isnt happening
            if (servo_flag)
                gpio_put(boost_enable, true); // Disable Boost circuit if USB is disconnected from PC
            timeout_power = 0;
        }
        else if (timeout_power > 50000)
        {
            gpio_put(boost_enable, false); // Disable Boost circuit if USB is disconnected from PC

            gpio_put(status_led_pin, false); // For indication error that USB comm isnt happening
            sleep_ms(250);
            gpio_put(status_led_pin, true); // For indication error that USB comm isnt happening
            sleep_ms(250);
        }
        else
        {
            timeout_power++;
        }

        // to get the input
        int command = getchar_timeout_us(0);
        if (command >= ' ')
        {
            gpio_put(led_sig, true); // For indication error that USB comm isnt happening
            sleep_ms(1);
            gpio_put(led_sig, false); // For indication error that USB comm isnt happening
            sleep_ms(1);
        }
        if (command == 'b')
        {
            int intensity = parse_numeric_value();
            if (intensity >= 0)
            {
                pwm_set_freq_duty(led_pin, 2000, intensity);
                printf("%c %d\n", command, intensity);
            }
        }
        else if (command == 's')
        {
            angle = parse_numeric_value();
            angle = round((4095 / 360) * angle);

            if (angle > 0)
            {
                // servo_set_position(servo_pin, angle);
                // testservo.setLED(HIGH);
                // testservo.setLED(LOW);
                // int status = testservo.getHardwareStatus();
                // printf("%d\n", status);
                // if (status == 1)
                // testservo.setPosition(angle);

                uint8_t counter = 0;
                while (testservo.isServoMoving() && counter < 100)
                {
                    counter++;
                    int position = testservo.getPosition();
                    printf("position: %d \n", position);
                    if (testservo.getHardwareStatus() == 5)
                    {
                        printf_debug("overload error\n");
                        testservo.reboot();
                        break;
                    }
                    if (abs(position - angle) <= 10)
                    {
                        break;
                    }
                    watchdog_update();
                };

                printf("%c %d\n",command, servo_angle);
            }
            else
            {
                printf("#\n");
            }
        }
        else if (command == 'l')
        {
            angle = parse_numeric_value();
            if (angle >= 0)
            {
                for (int count = 0; count < angle; count++)
                {
                    // servo_set_position(servo_pin, servo_angle);
                    testservo.setLED(HIGH);
                    testservo.setLED(LOW);
                }
            }
        }
        else if (command == 'c')
        {
            testservo.configure();
        }
        else if (command == 'n')
        {
            bool mode = parse_numeric_value();
            testservo.newsetup(mode);
        }
        else if (command == 'w')
        {
            int offset_val = parse_numeric_value();
            testservo.setHomingOffset(offset_val);
        }
        // to print the name of the device
        else if (command == 'u' || command == 'U')
        {
            int sub_command = getchar_timeout_us(0);
            if (sub_command == '2')
            {
                printf(BOARD_TYPE "\n");
                printf("ok\n");
            }
        }
        // to set the home to the current encoder position
        else if (command == 'h')
        {
            int sub_command = getchar_timeout_us(0);
            switch (sub_command)
            {
            case 'z':
                axis_offset[0] = capture_buf[0];
                added_offset[0] = 0;
                break;
            case 'b':
                axis_offset[1] = capture_buf[1];
                added_offset[1] = 0;
                break;
            case 'a':
                for (int j = 0; j < num_of_axis; j++)
                {
                    axis_offset[j] = capture_buf[j];
                    added_offset[j] = 0;
                }
                break;
            }

            printf("%c\n", command);
        }
        else if (command == 'd')
        {
            int sub_command = getchar_timeout_us(0);
            float value = parse_numeric_value();
            printf("%f \n", value);
            switch (sub_command)
            {
            case 'z':
                added_offset[0] = (value - homing_position[0]);
                axis_offset[0] = capture_buf[0];
                break;
            case 'b':
                added_offset[1] = (value - homing_position[1]);
                axis_offset[1] = capture_buf[1];
                break;
            }
        }
        else if (command == '9')
        {
            if (isConfigured(i2c0) == false)
            {
                sensorConfiguration(i2c0);
            }
            proxValue = getProximity(i2c0);
            if (proxValue == -1)
            {
                i2c_bus_hang_resolve(i2c0, i2c0_scl_pin, i2c0_sda_pin);
                proxValue = getProximity(i2c0);
            }
            if (proxValue == -1)
            {
                printf("# ");
                i2c_bus_hang_resolve(i2c0, i2c0_scl_pin, i2c0_sda_pin);
                i2c_begin(i2c0, I2C_Baudrate, i2c0_scl_pin, i2c0_sda_pin);
            }
            else
            {
                printf("%d ", proxValue);
            }
            if (isConfigured(i2c1) == false)
            {
                sensorConfiguration(i2c1);
            }
            proxValue = getProximity(i2c1);
            if (proxValue == -1)
            {
                i2c_bus_hang_resolve(i2c1, i2c1_scl_pin, i2c1_sda_pin);
                proxValue = getProximity(i2c1);
            }
            if (proxValue == -1)
            {
                printf("#\n");
                i2c_bus_hang_resolve(i2c1, i2c1_scl_pin, i2c1_sda_pin);
                i2c_begin(i2c1, I2C_Baudrate, i2c1_scl_pin, i2c1_sda_pin);
            }
            else
            {
                printf("%d\n", proxValue);
            }
            // ambientValue = getAmbient();
            // whiteValue = getWhite();
            sleep_ms(1);
        }
        else if (command == 'x' || command == 'X')
        {
            i2c_begin(i2c0, I2C_Baudrate, i2c0_scl_pin, i2c0_sda_pin);
            i2c_begin(i2c1, I2C_Baudrate, i2c1_scl_pin, i2c1_sda_pin);
            getProximity(i2c0);
            getProximity(i2c1);
            printf("%c\n", command);
        }
        // to print the current encoder position minus the home position
        else if (command == 'p')
        {
            for (int i = 0; i < number_of_encoders; i++)
            {
                printf("%f ", homing_position[i] + added_offset[i] + directions[i] * (capture_buf[i] - axis_offset[i]) * encoder_mm_per_click[i]);
                // printf("%d ", capture_buf[i]-axis_offset[i]);
            }
            printf("\n");
            // printf("%f, %f, %f, %f, %f, %f \n", homing_position[1], added_offset[1], directions[1], capture_buf[1], axis_offset[1], encoder_mm_per_click[1]);
            sleep_ms(1);
        }
        else if (command == '?')
        {
            int inchar = getchar_timeout_us(0);
            if (inchar == 'I')
            {
                inchar = getchar_timeout_us(0);
                if (inchar == 'D')
                {
                    printf(BOARD_TYPE " " HW_VERSION " " FW_VERSION "\n");
                }
            }
            else if (inchar == 's')
            {
                int angle = testservo.getPosition();
                if (angle > 0)
                {
                    float converted_angle = ((angle / 4095) * 360);
                    printf("%f %d\n", converted_angle, angle);
                }
            }
            else if (inchar == 'z')
            {
                printf("%d \n", testservo.getPresetCurrent());
            }

            else if (inchar == 't')
            {
                testservo.setCurrentlimit(1700);
                printf("%d \n", testservo.getCurrentlimit());
            }

            else if (command == 'D' || command == 'd')
            {
                // ads();
                printf("ADS Data");
            }
            else if (inchar == 'v')
            {

                printf("min voltage: %d \n", testservo.getVoltageMin());
                printf("max voltage: %d \n", testservo.getVoltageMax());
                printf("current voltage: %d \n", testservo.getVoltageCurr());
                testservo.setVoltageMax(62);
                testservo.setVoltageMin(32);
            }
            else if (inchar == 'r')
            {
                servo_counter = 0;
                servo_flag = false;
                gpio_put(boost_enable, true);
                testservo.reboot();
            }
            else if (inchar == 'f')
            {
                testservo.factoryreset();
            }
            else if (inchar == 'n')
            {
                printf("Protocol type: %d\t", testservo.getProtocolType());
                printf("Drive mode: %d\t", testservo.getDriveMode());
                printf("Operating mode: %d\t", testservo.getOperatingMode());
                printf("Profile Acceleration: %d\t", testservo.getProfileAcceleration());
                printf("Profile Velocity: %d\t", testservo.getProfileVelocity());
                printf("Min pos: %d\t", testservo.getMinPositionLimit());
                printf("Max pos: %d\t", testservo.getMaxPositionLimit());
            }
            else if (inchar == 'w')
            {
                printf("homing offset: %d \n", testservo.getHomingOffset());
            }
            else if (inchar == 'e')
            {
                printf("hardware_status: %d\n", testservo.getHardwareStatus());
            }
            else if (inchar == 'q')
            {
                servo_angle = testservo.getPosition();
                servo_current_consumption = testservo.getPresetCurrent();
                printf("%d %d\n", servo_angle, servo_current_consumption);
            }
            else if (inchar == '\n')
            {
                printf("hardware_status: %d\n", testservo.getHardwareStatus());

                print_commandlist();
            }
        }
        else if (command == '#')
        {
            int inchar = getchar_timeout_us(0);
            if (inchar == 'F')
            {
                if (getchar_timeout_us(0) == 'P')
                {
                    command = getchar_timeout_us(0);
                    if (command == 'I')
                    {
                        int dummy_data = parse_numeric_value();
                        if (dummy_data > 0)
                        {
                            LED_intensity = dummy_data;
                            update_flash_data();
                            printf("LED_intensity updated to %d \n", LED_intensity);
                        }
                    }
                    else if (command == 'A')
                    {
                        int dummy_data = parse_numeric_value();
                        if (dummy_data > 0)
                        {
                            servo_angle = dummy_data;
                            update_flash_data();
                            printf("servo_angle updated to %d \n", servo_angle);
                        }
                    }
                }
            }
            else if (inchar == 'R')
            {
                if (getchar_timeout_us(0) == 'P')
                {
                    int dummy_data = parse_numeric_value();
                    if (dummy_data == SECRET_CODE)
                    {
                        reset_usb_boot(0, 0);
                    }
                }
            }
        }
    }
}
