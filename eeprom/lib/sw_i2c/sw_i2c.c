/**
 * @file sw_i2c.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "sw_i2c.h"
#include "sw_i2c_port.h"
#include "sw_i2c_private.h"

void sw_i2c_init()
{
    sw_i2c_port_init();
}

/* Function to setup and execute I2C transfer request */
bool sw_i2c_transfer(uint8_t dev_addr, uint8_t *tx_buffer,uint16_t tx_size,uint8_t *rx_buffer,uint16_t rx_size)
{
    uint16_t i;

    if (tx_size > 0) {
        /* start */
        i2c_start();
        /* address + write */
        i2c_send_byte(dev_addr<<1);
        if (i2c_wait_ack() == false) {
            goto error_device_nack;
        }
        /* write data */
        for (i=0; i<tx_size; i++) {
            i2c_send_byte(tx_buffer[i]);
            if (i2c_wait_ack() == false) {
                goto error_device_nack;
            }
        }
    }
    if (rx_size > 0) {
        /* start */
        i2c_start();
        /* address + read */
        i2c_send_byte(dev_addr<<1 | 1);
        if (i2c_wait_ack() == false) {
            goto error_device_nack;
        }
        /* read data */        
        for (i=0; i<rx_size; i++) {
            rx_buffer[i] = i2c_recv_byte(i+1<rx_size);
        }
    }
    i2c_stop();
    return true;
    
error_device_nack:
    i2c_stop();
    return false;
}

// Scan the I2C bus between addresses from_addr and to_addr.
// On each address, call the callback function with the address and result.
// If result==0, address was found, otherwise, address wasn't found
// (can use result to potentially get other status on the I2C bus, see twi.c)
// Assumes Wire.begin() has already been called
void scan_i2c_bus(uint8_t from_addr, uint8_t to_addr, void(*callback)(uint8_t address, uint8_t result))
{
    bool rc;
    uint8_t dev_addr_7bit = 0;

    for( uint8_t addr = from_addr; addr <= to_addr; addr++) {

        /* start */
        i2c_start();

        /* address + write */
        i2c_send_byte(addr);

        if (i2c_wait_ack() == false) {
            rc = false;
        }else{
            rc = true;
        }

        i2c_stop();

        dev_addr_7bit = addr>>1;

        callback(dev_addr_7bit, rc);

        //dealy for sometime, 5 clk
        for(char i=0; i<5; i++){
            sw_i2c_delay();
        }

        //ignore add+1, read

        addr++;
        if(addr > to_addr){
            break;
        }
        
    }
}
