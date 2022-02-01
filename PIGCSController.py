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


class PIGCSController(Device):
    """ """

    host = device_property(
        dtype=str,
        default_value="192.168.0.100",
        doc="controller IP address",
    )

    port = device_property(
        dtype=int,
        default_value=50000,
        doc="controller port",
    )

    def init_device(self):
        """Establish connection to controller."""
        super(PIGCSController, self).init_device()
        try:
            print(
                f"Connecting to GCS controller on {self.host}:{self.port}",
                file=self.log_info,
            )
            self.ctrl = GCS2Device()
            self.ctrl.ConnectTCPIP(self.host, self.port)
            idn = self.ctrl.qIDN()
            print(f"Connection established on {self.host}:\n{idn}", file=self.log_info)
            self._positions = None
            self._limits = None
            self._moving = None
            self._last_query = 0.0
            self._query_timeout = 0.1
            self._axis_names = self.ctrl.allaxes
            self.set_state(DevState.ON)
        except Exception as ex:
            print(f"Error on initialization: {ex}", file=self.log_error)
            self.set_state(DevState.OFF)
            sys.exit(255)

    def always_executed_hook(self):
        """Query all axes positions, limits and move states."""
        if (time.time() - self._last_query) > self._query_timeout:
            self._positions = self.ctrl.qPOS()
            self._limits = self.ctrl.qLIM()
            self._moving = self.ctrl.IsMoving(self._axis_names)
            if any(self._moving.values()):
                self.set_state(DevState.MOVING)
            else:
                self.set_state(DevState.ON)
            self._last_query = time.time()

    def query_position(self, axis):
        return self._positions[axis]

    @command(
        dtype_in=str,
        doc_in="formatted string '<axis_name>=<position>'",
        dtype_out=int,
        doc_out="Result code (0: moving)",
    )
    def set_position(self, position):
        axis = position[0]
        target = float(position[1])
        self.ctrl.MOV(axis, target)
        error = self.ctrl.qERR()
        return error

    def query_limit(self, axis):
        return self._limits[axis]

    @command(
        dtype_in=str,
        doc_in="axis to query",
        dtype_out=(float,),
        doc_out="[pos, limit, move]",
    )
    def query_axis_state(self, axis):
        """Return position, limit and moving state for axis."""
        return self._positions[axis], self._limits[axis], self._moving[axis]

    @command(
        dtype_out=(str,),
        doc_out="available axis names on controller",
    )
    def get_axis_names(self):
        """Get list of axis names"""
        return self._axis_names

    @command(
        dtype_in=str,
        dtype_out=(float,),
    )
    def query_axis_limits(self, axis):
        """Return limit positions for axis."""
        lower = self.ctrl.qTMN()[axis]
        upper = self.ctrl.qTMX()[axis]
        return [lower, upper]

    @command
    def find_references(self):
        """Find reference marks for all axes."""
        self.ctrl.FRF()
        result = self.ctrl.qFRF()
        if all(result.values()):
            self.set_state(DevState.ON)
        else:
            self.set_state(DevState.FAULT)

    @command
    def halt(self):
        """Halt motion smoothly."""
        self.ctrl.HLT(noraise=True)


if __name__ == "__main__":
    PIGCSController.run_server()
