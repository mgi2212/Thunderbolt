Thunderbolt
==========
Thunderbolt is an [Arduino](http://arduino.cc) library for interacting with a Trimble Thunderbolt or other TSIP-based GPS and timing units. The libary can configure and read asynchronous TSIP primary and secondary timing packets. The library is structured and has primatives to support sending and parsing TSIP in general.  

The library code is in-part derived from the great Lady Heather work by Mark Sims, John Miles and Tom Van Baak.  Please note this is not a "complete" Lady Heather-like implementation, but the structure and primative functionality is there to advance it to that point.

Install
-------
The library can be installed using the [standard Arduino library install procedure](http://arduino.cc/en/Guide/Libraries#.UwxndHX5PtY)  

Examples
--------
basic - Illustrates how to use hardware serial ports (like those on Due and Mega boards) with the Thunderbolt library. Displays UTC time 1pps using serial print. 

led_clock - Demonstrats how to use a software serial port with the Thunderbolt library. Displays UTC time on a 8-digit 7-segment SPI display. Requires LedControl library if you wish to retain the display feature.
