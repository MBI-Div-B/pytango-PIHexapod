#!/usr/bin/env python3

"""
PI Hexapod tango device server

@author: M. Schneider, MBI Berlin, 2021
"""

from pipython import GCS2Device
import tango
from tango import AttrWriteType, DevState, DevDouble, DevBoolean, DeviceProxy
from tango.server import Device, attribute, command, device_property
import sys
import time


class PIGCSAxis(Device):
    """"""

    controller = device_property(
        dtype=str,
        default_value="",
        doc="tango fqdn of controller device server",
    )

    axis = device_property(
        dtype=str,
        default_value="",
        doc="name of axis on controller (e.g., X, Y, Z, U, V, W)",
    )

    position = attribute(
        dtype=float,
        access=AttrWriteType.READ,
    )

    limit_switch = attribute(
        dtype=int,
        access=AttrWriteType.READ,
    )

    def init_device(self):
        super(PIGCSController, self).init_device()
        self.ctrl = DeviceProxy(self.controller)
        ctrl_axes = self.ctrl.get_axis_names()
        if self.axis in ctrl_axes:
            self.set_state(DevState.ON)
            self._position = 0
            self._limit = 0
        else:
            print(f"Axis {self.axis} not in {ctrl_axes}", file=self.log_error)
            self.set_state(DevState.FAULT)

    def always_executed_hook(self):
        state = self.ctrl.query_axis_state(self.axis)
        print(f"READ STATE: {self.axis} {state}")
        self._position = state[0]
        self._limit = bool(state[1])
        if state[2]:
            self.set_state(DevState.MOVING)
        else:
            self.set_state(DevState.ON)

    def read_position(self):
        return self._position

    def write_position(self, position):
        ans = self.ctrl.set_position(f"{self.axis}={position}")
        print(f"SET POS: {self.axis} -> {position} (ans={ans})")
        if ans == 0:
            self.set_state(DevState.MOVING)

    def read_limit_switch(self):
        return self._limit

    @command
    def stop_axis(self):
        """Smoothly stop motion"""
        self.ctrl.halt()


if __name__ == "__main__":
    PIGCSAxis.run_server()
