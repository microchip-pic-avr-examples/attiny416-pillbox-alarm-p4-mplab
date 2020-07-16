<a href="https://www.microchip.com" rel="nofollow"><img src="images/microchip.png" alt="MCHP" width="300"/></a>

# ATtiny416 Pillbox Alarm Application

Theapplication allows the user to set up to four different daily alarm times, such as reminders to take pills atrequired times. When an alarm occurs, a piezoelectric buzzer generates a pulsating sound and the LEDassociated with that alarm flashes.

The example is explained in more details in the application note [AN2387](#Related-Documentation)

## Related Documentation

- [AN2607 - Pillbox Alarm Application for the AVR® P4 FEB](https://www.microchip.com//wwwAppNotes/AppNotes.aspx?appnote=en603897)
- [ATtiny416 Product Page](https://www.microchip.com/wwwproducts/en/ATtiny416)

## Software Used

- [MPLAB X IDE v5.40 or later](https://www.microchip.com/mplab/mplab-x-ide)
- [XC8 (v2.20)](https://www.microchip.com/mplab/compilers) alternativly [AVR/GNU C Compiler 5.4.0](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers) can be used
- ATtiny_DFP 2.2.89 or later

## Hardware Used

-  [AVR P4](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/ATAVRFEB-P4)

## Operation

1. Open `PillboxAlarmP4.X` in MPLAB.
2. Connect the AVR P4 with your computer.
3. Make sure the kit is selected as the tool to be programmed under project settings.
4. Press the make and program button to program the device.
4. Interact with the alarm according to the application note.

## Conclusion

We have here shown the possibility to use the ATtiny416 as a Pillbox Alarm Application.