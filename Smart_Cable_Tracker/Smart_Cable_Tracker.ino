#include <Wire.h>   //includes the wire library for i2c communication (mpu6050)
#include <avr/io.h> //for low level access
#include <util/delay.h> //for delay functions
#include <SoftwareSerial.h> //for gps communication

//mpu6050 i2c address
#define mpu_address 0x68

//the constants
#define sample_count 100 //the number of samples for calibration
#define filter_alpha 0.98 //complementary filter coefficient
#define pi_value 3.14159265358979323846
#define drum_radius 7.0 //the radius of the drum (cm)

//gps pins
#define gps_rx_pin A0 //gps tx pin connected to arduino rx 'A0'
#define gps_tx_pin A1 //gps rx pin connected to arduino tx 'A1'

//global variables
float angle_gyro = 0.0; //angle from gyroscope
float angle_accel = 0.0; //angle from accelerometer
float fused_angle = 0.0; //fused angle
unsigned long previous_time = 0;  //previous time for delta time calculation
float gyro_calibration_offset = 0.0; //gyroscope offset for calibration

//gps variables
float gps_latitude = 0.0; //latitude from gps
float gps_longitude = 0.0; //longitude from gps
bool gps_data_valid = false; //flag for valid gps data

//gps object
SoftwareSerial gps_serial(gps_rx_pin, gps_tx_pin);

//low level lcd functions
void pulse_lcd_enable();
void send_to_lcd(unsigned char data, int8_t mode);
void initialize_lcd();
void display_lcd_string(char* str);
void display_lcd_string_position(char row, char pos, char* str);
void clear_lcd();

//lcd D4:7 to PD4:7
#define lcd_direction DDRD //lcd direction register 
#define lcd_port PORTD //lcd write register

#define rs_en_direction DDRB //rs,en direction register
#define rs_en_port PORTB //rs,en write register

#define rs_pin PB0 //rs pin 8
#define en_pin PB1 //en pin 9

//temperature sensor 'lm35'
#define lm35_sensor_pin PC2 //A2
void initialize_adc();
uint16_t read_adc(uint8_t pin);

//pulse the enable pin to send data to the lcd
void pulse_lcd_enable() {
    rs_en_port &= ~(1 << en_pin); //set en low
    _delay_us(1);

    rs_en_port |= (1 << en_pin); //set en high
    _delay_us(1);

    rs_en_port &= ~(1 << en_pin); //set en low
    _delay_us(1);
}

//send data or command to lcd
void send_to_lcd(unsigned char data, int8_t mode) {
    switch (mode) {
        case 0: //rs = 0 command mode
            rs_en_port &= ~(1 << rs_pin);
            break;
        case 1: //rs= 1 data mode
            rs_en_port |= (1 << rs_pin);
            break;
        default:
            break;
    }

    //send upper nibble
    lcd_port = (lcd_port & 0x0F) | (data & 0xF0);
    pulse_lcd_enable();
    _delay_us(200);

    //send lower nibble
    lcd_port = (lcd_port & 0x0F) | (data << 4);
    pulse_lcd_enable();
    _delay_ms(2);
}

//initialize the lcd
void initialize_lcd() {
    lcd_direction = 0xFF; //set lcd data pins to output
    rs_en_direction |= (1 << en_pin) | (1 << rs_pin); //set en and rs to output

    _delay_ms(20); //lcd power on delay 15ms

    send_to_lcd(0x02, 0); //initialization of lcd send for 4-bit 
    send_to_lcd(0x28, 0); //2 lines, 5x7 matrix in 4-bit mode
    send_to_lcd(0x0C, 0); //display on, cursor off
    send_to_lcd(0x06, 0); //increment cursor (shift cursor to right)
    send_to_lcd(0x01, 0); //clear display screen

    _delay_ms(2);
}

//send a string to the lcd
void display_lcd_string(char* str) {
    for (int i = 0; str[i] != 0; i++) {
        send_to_lcd(str[i], 1); //send each character in data mode
    }
}

//send a string to the wanted position in lcd
void display_lcd_string_position(char row, char pos, char* str) {
    if (pos < 16) {
        if (row == 0) {
            send_to_lcd(0x80 | (pos & 0x0F), 0); //position in row 1
        } else if (row == 1) {
            send_to_lcd(0xC0 | (pos & 0x0F), 0); //position in row 2
        }
    }
    display_lcd_string(str); //send the string
}

//clear lcd screen
void clear_lcd() {
    send_to_lcd(0x01, 0); //clear display
    _delay_ms(2);
    send_to_lcd(0x80, 0); //cursor at 0, 0
}

//initialize adc for lm35
void initialize_adc() {
    //set adc vcc to internal 5v and left adjust the result
    ADMUX = (1 << REFS0) | (1 << ADLAR);
    //enable adc and set prescaler: adc clock = mcu clock/prescaler
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

//read adc value from a specific pin
uint16_t read_adc(uint8_t pin) {
    //select the channel and start conversion
    ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);
    ADCSRA |= (1 << ADSC); //start conversion
    while (ADCSRA & (1 << ADSC)); //wait conversion to complete
    return ADC;
}

//calibration function
float calibrate_gyro(int num_samples) {
    float sum = 0;
    float sensor_data[3];
    for (int i = 0; i < num_samples; i++) {
        read_mpu(sensor_data); //read sensor data
        sum += sensor_data[0]; //sum gyroscope 'z' axis data
        delay(10); //small delay between readings
    }
    return sum / num_samples; //return the average
}

void setup() {
    //initialize serial communication
    Serial.begin(9600);

    //initialize i2c for mpu6050
    initialize_i2c();

    //wake up mpu6050
    start_i2c();
    write_i2c(mpu_address << 1);
    write_i2c(0x6B);
    write_i2c(0x00);
    stop_i2c();

    //initialize lcd
    initialize_lcd();
    display_lcd_string_position(0, 0, "Initializing...");
    _delay_ms(1000);
    clear_lcd();
    display_lcd_string_position(0, 0, "System Ready");
    _delay_ms(1000);

    //calibrate gyroscope
    gyro_calibration_offset = calibrate_gyro(sample_count);

    //initialize adc for lm35
    initialize_adc();

    //initialize gps
    gps_serial.begin(9600);

    //initialize previous time
    previous_time = micros();
}

void loop() {
    //array to store sensor data: [gyro(z), acc(x), acc(y)]
    float sensor_data[3];

    //read sensor data
    read_mpu(sensor_data);

    //calculate delta time
    unsigned long current_time = micros();
    float delta_time = (current_time - previous_time) / 1000000.0; //convert to seconds
    previous_time = current_time;

    //calibrate gyroscope data
    float gyro_z_calibrated = sensor_data[0] - gyro_calibration_offset;

    //integrate gyroscope data to get angle
    angle_gyro += gyro_z_calibrated * delta_time;

    //calculate angle from accelerometer
    angle_accel = -atan2(sensor_data[2], sensor_data[1]) * 180 / pi_value;

    //fuse angles using complementary filter
    fused_angle = complementary_filter(angle_gyro, angle_accel);

    //use kalman filter for better accuracy
    static float error_covariance = 1.0; //initial error covariance
    float process_noise = 0.01; //process noise covariance
    float measurement_noise = 0.1; //measurement noise covariance
    fused_angle = kalman_filter(error_covariance, process_noise, measurement_noise, angle_gyro, angle_accel, delta_time);

    //calculate distance decreased
    float distance_decreased = calculate_distance(fused_angle, drum_radius);

    //read temperature from lm35
    uint16_t temp_adc = read_adc(lm35_sensor_pin);
    float temperature = (float)temp_adc * 1.1 * 100.0 / 1024.0;

    //read gps data
    while (gps_serial.available() > 0) {
        String nmea_sentence = gps_serial.readStringUntil('\n');
        if (nmea_sentence.startsWith("$GPRMC")) {
            parse_gprmc(nmea_sentence);
        }
    }

    //display the data on lcd
    char buffer[16];
    clear_lcd();

    dtostrf(distance_decreased, 4, 2, buffer);
    display_lcd_string_position(0, 0, "Dist: ");
    display_lcd_string(buffer);
    display_lcd_string(" cm");

    //temperature display
    dtostrf(temperature, 4, 2, buffer);
    display_lcd_string_position(1, 0, "Temp: ");
    display_lcd_string(buffer);
    display_lcd_string(" C");

    //print data to serial monitor
    Serial.print("Distance Decreased: ");
    Serial.print(distance_decreased);
    Serial.print(" cm, Temp: ");
    Serial.print(temperature);
    Serial.println(" C");

    if (gps_data_valid) {
        Serial.print("Latitude: ");
        Serial.print(gps_latitude, 6);
        Serial.print(", Longitude: ");
        Serial.println(gps_longitude, 6);
    } else {
        Serial.println("Waiting for GPS fix...");
    }

    //delay before the next reading
    delay(100);
}

//i2c initialization
void initialize_i2c() {
    TWSR = 0x00; //no prescaler
    TWBR = ((F_CPU / 100000UL) - 16) / 2; //set bit rate for 100khz i2c
    TWCR = (1 << TWEN); //enable TWI
}

//i2c start condition
void start_i2c() {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));  //wait for start condition
}

//i2c stop condition
void stop_i2c() {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    while (TWCR & (1 << TWSTO)); //make sure stop condition is sent
}

//i2c write
void write_i2c(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT))); //wait for transmission
}

//i2c read
uint8_t read_i2c(bool ack) {
    TWCR = (1 << TWEN) | (1 << TWINT) | (ack ? (1 << TWEA) : 0);
    while (!(TWCR & (1 << TWINT))); //wait for completion
    return TWDR;
}

//read sensor data from mpu6050
void read_mpu(float *data) {
    start_i2c();
    write_i2c(mpu_address << 1);
    write_i2c(0x3B); //start reading from accelerometer x register
    start_i2c();
    write_i2c((mpu_address << 1) | 1);

    //read accelerometer data (x, y)
    int16_t accel_x = (read_i2c(1) << 8) | read_i2c(1);
    int16_t accel_y = (read_i2c(1) << 8) | read_i2c(1);

    //empty i2c reads to skip unwanted readings (acc (z), temp, gyro (x), gyro (y))
    for (int i = 0; i < 8; i++) {
        read_i2c(1);
    }

    //read gyroscope data (z)
    int16_t gyro_z = (read_i2c(1) << 8) | read_i2c(0);

    stop_i2c();

    //convert raw data to meaningful values
    data[0] = gyro_z / 16384.0; //gyroscope z axis (degrees per second)
    data[1] = accel_x / 131.0; //accelerometer x axis (g)
    data[2] = accel_y / 131.0; //accelerometer y axis (g)
}

//complementary filter function
float complementary_filter(float gyro_angle, float accel_angle) {
    return filter_alpha * gyro_angle + (1 - filter_alpha) * accel_angle;
}

//kalman filter function
float kalman_filter(float &error_covariance, float process_noise, float measurement_noise, float gyro_angle, float accel_angle, float delta_time) {
    static float state = 0.0;
    static float kalman_gain = 0.0;

    state += gyro_angle * delta_time; //predict state
    error_covariance += process_noise * delta_time; //predict error covariance

    kalman_gain = error_covariance / (error_covariance + measurement_noise); //kalman gain
    state += kalman_gain * (accel_angle - state); //update state
    error_covariance = (1 - kalman_gain) * error_covariance; //update error covariance

    return state;
}

//calculate distance that decreased
float calculate_distance(float angle, float radius) {
    return (angle * 2 * pi_value * radius) / 360.0;
}

//function to parse the GPRMC NMEA sentence
void parse_gprmc(String sentence) {
    //split the sentence to parts
    String parts[13];
    int index = 0;
    int start = 0;
    int end = sentence.indexOf(',');

    while (end != -1 && index < 13) {
        parts[index++] = sentence.substring(start, end);
        start = end + 1;
        end = sentence.indexOf(',', start);
    }

    //check if the data is valid
    if (parts[2] == "A") { //if A means valid data
        //extract latitude
        String lat_str = parts[3];
        float lat_deg = lat_str.substring(0, 2).toFloat();
        float lat_min = lat_str.substring(2).toFloat();
        gps_latitude = lat_deg + (lat_min / 60.0);
        if (parts[4] == "S") {
            gps_latitude = -gps_latitude; //south is negative
        }

        //extract longitude
        String lon_str = parts[5];
        float lon_deg = lon_str.substring(0, 3).toFloat();
        float lon_min = lon_str.substring(3).toFloat();
        gps_longitude = lon_deg + (lon_min / 60.0);
        if (parts[6] == "W") {
            gps_longitude = -gps_longitude; //west is negative
        }

        gps_data_valid = true; //set flag to indicate valid data
    }
}