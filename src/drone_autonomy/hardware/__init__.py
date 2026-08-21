"""Raspberry Pi hardware adapters: Pixhawk, HC-SR04 sensors, servo, and CV link.

Importing a module from this package pulls in a hardware library, so nothing
here is imported by simulation mode. Each adapter keeps its decoding, filtering,
and encoding rules as plain functions or small classes that can be tested on a
laptop, with the input and output work kept in a thin shell around them.
"""
