#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "keycode.h"
#include "serial.h"
#include "host.h"
#include "action.h"
#include "action_util.h"
#include "lufa.h"
#include "ble51_task.h"
#include "ble51.h"
#include "debug.h"
#include "timer.h"
#include "wait.h"
#include "command.h"
#define TIMEOUT 100

bool force_usb = false;
uint32_t kb_idle_timer = 0; 
static bool usb_nkro_status = false;
static bool usb_connected = false;


uint8_t BLE51_PowerState = 1;  

void ble51_task_init(void)
{
    kb_idle_timer = timer_read32();
}

void ble51_task(void)
{
    if (BT_POWERED) {
        if (USB_DeviceState != DEVICE_STATE_Configured) {
            if (BLE51_PowerState == 1 && (timer_elapsed32(kb_idle_timer) > 15000)) {
                print("dozing\n");
                BLE51_PowerState = 3;
            }

            if (BLE51_PowerState == 3 && (timer_elapsed32(kb_idle_timer) > 9000000)) {
                print("BT is idle for a long time. Turn off. \n");
                BLE51_PowerState = 4;
                turn_off_bt();
            }
        }
    }

    /* Bluetooth mode | USB mode */
    if (!force_usb) {
        if (host_get_driver() != &ble51_driver) {
            clear_keyboard(); 
            print("Bluetooth\n");
            usb_nkro_status = keyboard_nkro;
            keyboard_nkro = 0;
            host_set_driver(&ble51_driver);
        } else if (!usb_connected && (USB_DeviceState == DEVICE_STATE_Configured)) {
            force_usb = 1;
            usb_connected = 1;
        }
    } else {
        if (host_get_driver() != &lufa_driver) {
            usb_connected = 1;
            clear_keyboard();
            print("USB\n");
            keyboard_nkro = usb_nkro_status;
            host_set_driver(&lufa_driver);
        } else if (USB_DeviceState != DEVICE_STATE_Configured) {
            force_usb = 0;
            usb_connected = 0;
        }           
    }
}    
  
bool command_extra(uint8_t code)
{
    switch (code) {
        case KC_B:
            clear_keyboard();
            print("\n\nbootloader... ");
            wait_ms(1000);
            bootloader_jump(); 
            break;
        case KC_U:
            force_usb ^= 1;
            return true;
        default:
            return false;   
    }
    return true;
}