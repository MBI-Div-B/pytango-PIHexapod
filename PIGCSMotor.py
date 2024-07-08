#!/usr/bin/env python3

"""
PI Hexapod tango device server

@author: M. Schneider, MBI Berlin, 2021
"""

from pipython import GCS2Device
from tango import AttrWriteType, DevState, DevDouble, DevBoolean, DeviceProxy, DevVarDoubleArray
from tango.server import Device, attribute, command, device_property, run
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

    pivot_X = attribute(
        label='pivot X',       
        dtype=float,
        unit='mm',
        access=AttrWriteType.READ,
    )

    pivot_Y = attribute( 
        label='pivot Y',       
        dtype=float,
        unit='mm',
        access=AttrWriteType.READ,
    )

    pivot_Z = attribute( 
        label='pivot Z',
        dtype=float,
        unit='mm',
        access=AttrWriteType.READ,
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
            self._referenced = None
            self._moving = None
            self._pivot_point = None
            self._last_query = 0.0
            self._query_timeout = 0.1
            self._axis_names = self.ctrl.allaxes
            self.set_state(DevState.ON)
        except Exception as ex:
            print(f"Error on initialization: {ex}", file=self.log_error)
            self.set_state(DevState.FAULT)

    def always_executed_hook(self):
        """Query all axes positions, limits and move states."""
        if self.dev_state() == DevState.ON:
            if (time.time() - self._last_query) > self._query_timeout:
                self._positions = self.ctrl.qPOS()
                self._limits = self.ctrl.qLIM()
                self._moving = self.ctrl.IsMoving(self._axis_names)
                self._referenced = self.ctrl.qFRF()
                self._last_query = time.time()
                self._pivot_point = self.ctrl.qSPI()

    def query_position(self, axis):
        return self._positions[axis]

    def read_pivot_X(self):
        return self._pivot_point['R']

    def read_pivot_Y(self):
        return self._pivot_point['S']

    def read_pivot_Z(self):
        return self._pivot_point['T']

    @command(
        dtype_in=str,
        doc_in="formatted string '<axis_name>=<position>'",
        dtype_out=int,
        doc_out="Result code (0: moving)",
    )
    def set_position(self, position):
        axis, target = position.split("=")
        target = float(target)
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
        """Return state for axis.

        Values:
            position (float)
            limit state (bool)
            movement state (bool)
            reference state (bool)
        """
        state = (
            self._positions[axis],
            self._limits[axis],
            self._moving[axis],
            self._referenced[axis],
        )
        return state

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

    @command(
        dtype_in=str,
        dtype_out=str,
    )
    def query_axis_unit(self, axis):
        """Return unit string for axis."""
        return self.ctrl.qPUN()[axis]

    @command(
        dtype_in=(float,),
        doc_in="X,Y,Z pivot point positions.",
    )
    def set_pivot_point(self, values):
        """Sets the pivot point of the hexapod."""
        self.ctrl.SPI(axes=['X', 'Y', 'Z'], values=list(values[0:3]))

    @command
    def find_references(self):
        """Find reference marks for all axes."""
        self.ctrl.FRF()

    @command
    def halt(self):
        """Halt motion smoothly. Doesn't stop reference moves."""
        self.ctrl.HLT(noraise=True)

    @command
    def stop(self):
        """Abruptly stop all motions, including reference moves."""
        self.ctrl.STP(noraise=True)


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
        access=AttrWriteType.READ_WRITE,
    )

    limit_switch = attribute(
        dtype=int,
        access=AttrWriteType.READ,
    )

    referenced = attribute(
        dtype=bool,
        access=AttrWriteType.READ,
    )

    def init_device(self):
        super(PIGCSAxis, self).init_device()
        self.ctrl = DeviceProxy(self.controller)
        ctrl_axes = self.ctrl.get_axis_names()
        if self.axis in ctrl_axes:
            self.set_state(DevState.ON)
            self._position = 0
            self._limit = 0
            self._referenced = False
            self.update_attribute_config()
        else:
            print(f"Axis {self.axis} not in {ctrl_axes}", file=self.log_error)
            self.set_state(DevState.FAULT)

    def update_attribute_config(self):
        vmin, vmax = self.ctrl.query_axis_limits(self.axis)
        self.position.set_min_value(vmin)
        self.position.set_max_value(vmax)

    def always_executed_hook(self):
        state = self.ctrl.query_axis_state(self.axis)
        print(f"READ STATE: {self.axis} {state}", file=self.log_debug)
        self._position = state[0]
        self._limit = bool(state[1])
        self._referenced = bool(state[3])
        if state[2]:
            self.set_state(DevState.MOVING)
        else:
            self.set_state(DevState.ON)
        if not self._referenced:
            self.set_state(DevState.WARN)

    def read_position(self):
        return self._position

    def write_position(self, position):
        ans = self.ctrl.set_position(f"{self.axis}={position}")
        # print(f"SET POS: {self.axis} -> {position} (ans={ans})")
        if ans == 0:
            self.set_state(DevState.MOVING)

    def read_limit_switch(self):
        return self._limit

    def read_referenced(self):
        return self._referenced

    @command
    def halt_axis(self):
        """Smoothly stop motion"""
        self.ctrl.halt()

    @command
    def stop_axis(self):
        """Abruptly stop all motion."""
        self.ctrl.stop()


if __name__ == "__main__":
    # PIGCSController.run_server()
    run((PIGCSController, PIGCSAxis))
