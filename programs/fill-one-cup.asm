# ASM example: fill one cup, then return the X axis to its start coordinate.
# ASSERT A.X == 0
A X0 F5000
A X220 F5000
A X130 Z150 F5000
A Z110 F5000
DISPENSE pump_driver 10000
A Z150 F5000
WAIT 1500

A X0 F10000
