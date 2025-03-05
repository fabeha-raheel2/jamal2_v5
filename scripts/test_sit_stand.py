#!/usr/bin/env python

from Quadruped import Quadruped

# INPUTS
L1 = 0.105  # Length of Link 1 (Hip) m
L2 = 0.225  # Length of Link 2 (Thigh) m
L3 = 0.250  # Length of Link 3 (Lower Leg / Shin) m

robotNameSpace = "jamal2_v5"

jamal2 = Quadruped(ns=robotNameSpace, debug_mode=True)
jamal2.set_link_lengths([L1, L2, L3])

 ### TEST QUADRUPED SIT .......................................................
jamal2.sit()
jamal2.stand()