/*****************************************************************//**
 * @file main_sampler_test.cpp
 *
 * @brief Basic test of nexys4 ddr mmio cores
 *
 * @author p chu
 * @version v1.0: initial release
 *********************************************************************/

// #define _DEBUG
#include "chu_init.h"
#include "gpio_cores.h"
#include "xadc_core.h"
#include "sseg_core.h"
#include "spi_core.h"
#include "i2c_core.h"
#include "ps2_core.h"
#include "ddfs_core.h"
#include "adsr_core.h"
#include <math.h>
#define DECLINATION 6.11

//float cmps2_getHeading();
//float measured_angle = cmps2_getHeading();
void cmps2_set(bool reset, I2cCore *C){
	uint8_t wbytes[2];
	uint8_t dev_addr;
	dev_addr = 0x30;
	wbytes[0] = 0x07;
	wbytes[1] = 0x80;
	C->write_transaction(dev_addr, wbytes, 2, 1);
}
void CMPS2_read_XYZ(I2cCore *C) {
    float Max[2], Mid[2], Min[2], X, Y;
  //command internal control register 0 bit 0 (measure)
    uart.disp("hello");
    const uint8_t dev_addr = 0x30;
    uint8_t wbytes[2];
    wbytes[0] = 0x07;
    wbytes[1] = 0x01;
    uint8_t bytes[4];
    C->write_transaction(dev_addr, wbytes, 2,1);
    sleep_ms(8);
    do{
    	wbytes[0] = 0x03;
    	C->write_transaction(dev_addr, wbytes, 1, 1);
    	C->read_transaction(dev_addr, bytes, 1,0);
        uart.disp("bytes: ");
        uart.disp(bytes[0]);
        uart.disp("\n\r");
    }while((bytes[0] & 0x01) ==0);
    ///////////////////
    do{
    wbytes[0] = 0x00;
    C->write_transaction(dev_addr, wbytes, 1,1);
    C->read_transaction(dev_addr, bytes,6,0);
    }while((bytes[0] & 0x01) ==0);
//while(1){
//    write_transaction(dev_addr, wbytes, 2,0);
//    sleep_ms(10);
//    wbytes[0] = 0x03;
//    do{
//        write_transaction(dev_addr, wbytes, 1,1);
//        uint8_t tmp[6];
//        read_transaction(dev_addr, tmp, 2, 0);
//        uart.disp("status reg: ");
//        uart.disp(bytes[0]);
//        uart.disp("\n\r");
//    }while((bytes[0] & 0x01) ==0);
//    uart.disp("ready to read: ");
//    uart.disp(bytes[0]);
//    uart.disp("\n\r");
//}

//
  float measured_data[2];
//
//  //reconstruct raw data
  measured_data[0] = 1.0 * (int)(bytes[1] << 8 | bytes[0]); //x
  measured_data[1] = 1.0 * (int)(bytes[3] << 8 | bytes[2]); //y

  //convert raw data to mG
  for (int i = 0; i < 2; i++) {
    measured_data[i] = 0.48828125 * (float)measured_data[i];
  }

  X = measured_data[0];
  Y = measured_data[1];

  //correct minimum, mid and maximum values
  if (measured_data[0] > Max[0]) { //x max
    Max[0] = measured_data[0];
    uart.disp("x max: ");
    uart.disp(Max[0]);
    uart.disp("\n\r");
  }
  if (measured_data[0] < Min[0]) { //x min
    Min[0] = measured_data[0];
    uart.disp("x min: ");
    uart.disp(Min[0]);
    uart.disp("\n\r");
  }
  if (measured_data[1] > Max[1]) { //y max
    Max[1] = measured_data[1];
    uart.disp("y min: ");
    uart.disp(Max[1]);
    uart.disp("\n\r");
  }
  if (measured_data[1] < Min[1]) { //y min
    Min[1] = measured_data[1];
    uart.disp("y min: ");
    uart.disp(Min[1]);
    uart.disp("\n\r");
  }
  for (int i = 0; i < 2; i++) { //mid
    Mid[i] = (Max[i] + Min[i]) / 2;
    uart.disp("mid: ");
    uart.disp(Mid[i]);
    uart.disp("\n\r");
  }
return;
}
void cmps_uartdisp(float measured_angle, PwmCore *pwm){
//    float measured_angle = cmps2_getHeading();
    uart.disp("measured angle: ");
    uart.disp(measured_angle);
    uart.disp("\n\r");

    if ((measured_angle > 337.25) | (measured_angle < 22.5)){
        uart.disp("North");
        pwm->set_duty(0.12589, 0);

    }
    else if(measured_angle > 292.5){
        uart.disp("North-west");
        pwm->set_duty(0.12589, 0);
    }
    else if(measured_angle > 247.5){
        uart.disp("west");
        pwm->set_duty(0.12589, 1);

    }
    else if(measured_angle > 202.5){
        uart.disp("south-west");
        pwm->set_duty(0.12589, 1);
    }
    else if(measured_angle > 157.5){
        uart.disp("south");
        pwm->set_duty(0.12589, 1);
    }
    ////
    else if(measured_angle > 112.5){
        uart.disp("south-east");
        pwm->set_duty(0.12589, 1);

    }
    else if(measured_angle > 67.5){
        uart.disp("east");
        pwm->set_duty(0.12589, 1);
    }
    else {
        uart.disp("north-east");
        pwm->set_duty(0.12589, 0);

    }
}
    float cmps2_getHeading(I2cCore *C){
        float components[2];
        float X,Y;
        float Mid[2];

        //cmps2_set(false, &C); //set polarity to normal
        CMPS2_read_XYZ(C);
        components[0] = X;
        components[1] = Y;
        CMPS2_read_XYZ(C);
        components[0] = (components[0] - X) /2.0;
        components[1] = (components[1] - Y) /2.0;

        float temp0 = 0;
        float temp1= 0;     //for storing partial results

        float deg = 0; // final result

        if(components[0] < Mid[0] ){
            if(components[1] > Mid[1]){
                //quad1
                temp0 = components[1] -Mid[1];
                temp1 = Mid[0] - components[0];
                deg = 90 - atan(temp0 / temp1) * (180/3.14159);
            }
            else
            {
                //quad2
                temp0 = Mid[1] - components[1];
                temp1 = Mid[0] - components[0];
                deg = 90 + atan(temp0 / temp1) *(180/3.14159);
            }
        }
            else{
                //quad3
                if (components[1] < Mid[1]){
                    temp0 = Mid[1] - components[1];
                    temp1 = components[0] - Mid[0];
                    deg = 270 - atan(temp0 / temp1)*(180/3.14159);
                }

                else{
                //quadrant 4
                temp0 = components[1] - Mid[1];
                temp1 = components[0] - Mid[0];
                deg = 270 + atan(temp0/temp1)*(180/3.14259);
            }

            deg += DECLINATION;
            if(DECLINATION > 0){
                if(deg>360){
                    deg -= 360;
                }
            }
            else{
                if(deg<0){
                    deg += 360;
                }
            }
            return deg;
        }
}





GpoCore led(get_slot_addr(BRIDGE_BASE, S2_LED));
GpiCore sw(get_slot_addr(BRIDGE_BASE, S3_SW));
I2cCore C(get_slot_addr(BRIDGE_BASE, S4_I2C));
XadcCore adc(get_slot_addr(BRIDGE_BASE, S5_XDAC));
PwmCore pwm(get_slot_addr(BRIDGE_BASE, S6_PWM));
DebounceCore btn(get_slot_addr(BRIDGE_BASE, S7_BTN));
SsegCore sseg(get_slot_addr(BRIDGE_BASE, S8_SSEG));
SpiCore spi(get_slot_addr(BRIDGE_BASE, S9_SPI));
//I2cCore C(get_slot_addr(BRIDGE_BASE, S10_I2C));

Ps2Core ps2(get_slot_addr(BRIDGE_BASE, S11_PS2));
DdfsCore ddfs(get_slot_addr(BRIDGE_BASE, S12_DDFS));
AdsrCore adsr(get_slot_addr(BRIDGE_BASE, S13_ADSR), &ddfs);


int main() {
	float measured_angle;
   while (1) {
	   CMPS2_read_XYZ(&C);
	   cmps_uartdisp(measured_angle, &pwm);
   } //while
} //main

