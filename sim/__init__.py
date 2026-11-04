"""TELLTALE simulator and evaluation.

A batched physics model of an SG90-class servo pulling a spring-loaded latch
through a compliant linkage, an INA219 current-sense model, fault and attack
scenarios, and the evaluation that runs the firmware's own C detector on the
simulated current traces (through ctypes).
"""
