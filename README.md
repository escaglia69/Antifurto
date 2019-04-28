# Antifurto
Anti intrusion system made with Arduino compatible boards.

See more at the [Wiki page](https://github.com/escaglia69/Antifurto/wiki)

It is made of one or more sensors (sensore) and one base.

Base can be of different kind, depending on the hardware you use.

A standard base can be made with Arduino Mega or compatible boards (basemega), with memory big enough to have full features.

![Schema](https://raw.githubusercontent.com/escaglia69/Antifurto/master/images/Antifurto.jpg)

Prior to building base  you need to modify a file of the RF24 library to reflect the pinout of the board.
The modification is as follows:

open the file RF24_Config.h

uncomment the line starting with //#define SOFTSPI (line 22)

change the lines:
```
      const uint8_t SOFT_SPI_MISO_PIN = 16; 
      const uint8_t SOFT_SPI_MOSI_PIN = 15; 
      const uint8_t SOFT_SPI_SCK_PIN = 14;  
```
in:
```
      const uint8_t SOFT_SPI_MISO_PIN = 7; 
      const uint8_t SOFT_SPI_MOSI_PIN = 8; 
      const uint8_t SOFT_SPI_SCK_PIN = 6;  
```

