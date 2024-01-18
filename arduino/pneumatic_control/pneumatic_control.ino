#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h> 
#include <time.h>
// for valves
#define pin_spi_cs1  10 // 1_19=51
#define pin_spi_other 0 // 0_2=2
#define pin_spi_mosi 11 // 3_15=112
#define pin_spi_sclk 13 // 0_3=3
#define pin_spi_cs2 9 // 0_7 =7
#define NUM_OF_CHANNELS 16
bool clock_edge = false;
unsigned short resolution = 0x0FFF;
void set_SCLK(bool value) { digitalWrite(pin_spi_sclk, value); }
void set_OTHER(bool value) { digitalWrite(pin_spi_other, value); }
void set_MOSI(bool value) { digitalWrite(pin_spi_mosi, value); }
void setCS1(bool value){ digitalWrite(pin_spi_cs1, value); }
void setCS2(bool value){ digitalWrite(pin_spi_cs2, value); }
void set_clock_edge(bool value){ clock_edge = value; }
bool get_MISO(void) { return false; } // dummy    
void wait_SPI(void){}
void chipSelect1(bool value){ setCS1(!value); wait_SPI(); wait_SPI(); }
void chipSelect2(bool value){ setCS2(!value); wait_SPI(); wait_SPI(); }
unsigned char transmit8bit(unsigned char output_data){
	unsigned char input_data = 0;
	int i;
	for(i = 7; i >= 0; i--){
		// MOSI - Master : write with down trigger
		//        Slave  : read with up trigger
		// MISO - Master : read before down trigger
		//        Slave  : write after down trigger        
		set_SCLK(!clock_edge);
		set_MOSI( (bool)((output_data>>i)&0x01) );
		input_data <<= 1;
		wait_SPI();
		set_SCLK(clock_edge);
		input_data |= get_MISO() & 0x01;
		wait_SPI();
	} 
	return input_data;
}

unsigned short transmit16bit(unsigned short output_data){
	unsigned char input_data_H, input_data_L;      
	unsigned short input_data;
	input_data_H = transmit8bit( (unsigned char)(output_data>>8) );
	input_data_L = transmit8bit( (unsigned char)(output_data) );
	input_data = (((unsigned short)input_data_H << 8)&0xff00) | (unsigned short)input_data_L;
	return input_data;
}

void setDARegister(unsigned char ch, unsigned short dac_data){
	unsigned short register_data;

	if (ch < 8) {
		register_data = (((unsigned short)ch << 12) & 0x7000) | (dac_data & 0x0fff);
		chipSelect1(true);
		transmit16bit(register_data);
		chipSelect1(false);
	}
	else if (ch >= 8) {
		register_data = (((unsigned short)(ch & 0x0007) << 12) & 0x7000) | (dac_data & 0x0fff);
		chipSelect2(true);
		transmit16bit(register_data);   
		chipSelect2(false);
	}
}
// pressure coeff: [0.0, 1.0]
void setState(unsigned int ch, double pressure_coeff)
{
	setDARegister(ch, (unsigned short)(pressure_coeff * resolution));
}
void init_pin_valves(){
  pinMode(pin_spi_cs1,OUTPUT);
  pinMode(pin_spi_other,OUTPUT);
  pinMode(pin_spi_mosi,OUTPUT);
  pinMode(pin_spi_sclk,OUTPUT);
  pinMode(pin_spi_cs2,OUTPUT);

}
void init_pins()
{
	set_SCLK(LOW);
	set_MOSI(LOW);
	set_OTHER(LOW);
	setCS1(HIGH);
	setCS2(HIGH);
}                 
void init_DAConvAD5328(void) {                     
	set_clock_edge(false);// negative clock (use                                  falling-edge)

	// initialize chip 1
  
  chipSelect1(true); 
  transmit16bit(0xa000);
  chipSelect1(false);
  chipSelect1(true); 
  transmit16bit(0x8003);
  chipSelect1(false);
  chipSelect2(true);   
  transmit16bit(0xa000);     
  chipSelect2(false);
  chipSelect2(true);
  transmit16bit(0x8003);                                        
  chipSelect2(false);
}                                   
class SerialReader 
{
  public:
    explicit constexpr SerialReader() {}
    int readInt() const {
        char buf[40] = {0};
        wait_();
        int ret = 0;
        int size = min(40, Serial.available());
        for (int i = 0; i < size; ++i) {
            buf[i] = (char)Serial.read();
        }
        int now = 0;
        char c = buf[now];
        while ((c < '0' || '9' < c) && c != '-') {
            c = buf[++now];
        }
        const bool f = (c == '-') && (c = buf[++now]);
        while (now + 1 < size) {
            ret = 10 * ret + c - '0';
            c = buf[++now];
        }
        return f ? -ret : ret;
    }

    float readFloat() const {
        char buf[40] = {0};
        wait_();
        int size = min(40, Serial.available());
        for (int i = 0; i < size; ++i) {
            buf[i] = (char)Serial.read();
        }
        int dot_idx = size - 1;
        for (int i = 0; i < size; ++i) {
            if (buf[i] == '.') {
                dot_idx = i;
                break;
            }
        }
        float ret = 0;
        float e = pow(10, dot_idx - 1);
        int now = 0;
        char c = buf[now];
        const bool f = (buf[now] == '-') && (c = buf[++now], e /= 10);
        while (now + 1 < size) {
            if (c != '.') {
                ret += (c - '0') * e;
                e /= 10;
            }
            c = buf[++now];
        }
        return f ? -ret : ret;
    }

  private:
    void wait_() const {
        while (Serial.available() == 0) {
            ;
        }
        delay(50);
    }
};                                 
constexpr SerialReader Reader;

void setup() {
 Serial.begin(115200);
 init_pin_valves();
 init_pins();   
 //init()
 init_DAConvAD5328();                                     
 int i;              
 for (i = 0; i < NUM_OF_CHANNELS; i++) {    
		setState(i, 0.0); 
    delay(100);
 }
}

void loop() {
  float p1=0,p2=0,p3=0,p4=0,p5=0,p6=0,p7=0,p8=0;
  while (Serial.available() >= 8) {
    int i;
    float pp[8];
    float p[8];
    for(i=0;i<8;i++){
      pp[i]=Serial.read();
      p[i]=pp[i]/255;
      setState(i,p[i]);

    }
    delay(50);
  }
}

// void loop()
// {
//   float p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0, p7 = 0, p8 = 0;
//   while (Serial.available() >= 1)
//   {
//     int i;
//     char inkey=Serial.read();
//     // Serial.println("inkey");
//     // Serial.println(inkey);
//     if(inkey='0'){
//       setState(0,1);
//       setState(1,0);
//       Serial.println(inkey);
//     }
//     else if(inkey='1'){
//       setState(1,1);
//       setState(0,0);
//       Serial.println(inkey);
//     }

//     else
//     {
//       for(i=0;i<NUM_OF_CHANNELS; i++){
//         setState(i,0.0);
//       }
//     }
//     delay(50);
//   }
// }
