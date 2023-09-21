# LIS2DG12TR_for_PIC
Adaptation LIS2DG12TR ST driver for PIC MCC
Based on ST lis2dh12 driver. 
Modified and optimized for PIC (and other 8-bit MCUs). 
Use functions with the "_o" suffix, the original ones are left for compatibility.
Disable (disconnect) the built-in SA0 pull-up if this pin is connected 
to ground to reduce consumption by approximately 120 µA.
Checking the temperature sensor showed its practical uselessness. 
The results are extremely unstable, their conversion to degrees 
is not described, the code for converting them to degrees in the ST source 
code looks strange, the result of its work strongly does not correspond 
to the real temperature. Turning on the temperature sensor adds 15..20 µA 
to the consumption.
