#!/usr/bin/env python3

"""
PI Hexapod tango device server

@author: M. Schneider, MBI Berlin, 2021
"""

from pipython import GCS2Device
import tango
from tango import AttrWriteType, DevState, DevDouble, DevBoolean, DeviceProxy
from tango.server import Device, attribute, command, device_property, run
import sys


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
            print(f"Connection established:\n{idn}",
                  file=self.log_info)
            self._axis_names = self.ctrl.allaxes
            self.set_state(DevState.ON)
        except Exception as ex:
            print(f"Error on initialization: {ex}", file=self.log_error)
            self.set_state(DevState.FAULT)

    @command(dtype_in=str, dtype_out=float)
    def query_axis_position(self, axis):
        return float(self.ctrl.qPOS(axis)[axis])

    @command(
        dtype_in=(str,),
        doc_in="list of str [<axis_name>, <position>]",
        dtype_out=int,
        doc_out="Result code (0: moving)",
    )
    def set_axis_position(self, position):
        axis, target = position
        target = float(target)
        self.ctrl.MOV(axis, target)
        error = self.ctrl.qERR()
        return error

    @command(dtype_in=str, dtype_out=bool)
    def query_axis_referenced(self, axis):
        return bool(self.ctrl.qFRF(axis)[axis])

    @command(dtype_in=str, dtype_out=bool)
    def query_axis_moving(self, axis):
        return self.ctrl.IsMoving()[axis]

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
        lower = self.ctrl.qTMN(axis)[axis]
        upper = self.ctrl.qTMX(axis)[axis]
        return [lower, upper]

    @command(
        dtype_in=str,
        dtype_out=str,
    )
    def query_axis_unit(self, axis):
        """Return unit string for axis."""
        return self.ctrl.qPUN()[axis]

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

    referenced = attribute(
        dtype=bool,
        access=AttrWriteType.READ,
    )

    def init_device(self):
        super(PIGCSAxis, self).init_device()
        self.ctrl = DeviceProxy(self.controller)
        print(f"Create attribute poxy: {self.controller}", file=self.log_debug)
        ctrl_axes = self.ctrl.get_axis_names()
        if self.axis in ctrl_axes:
            self.set_state(DevState.ON)
            self.update_attribute_config()
        else:
            print(f"Axis {self.axis} not in {ctrl_axes}", file=self.log_error)
            self.set_state(DevState.FAULT)

    def update_attribute_config(self):
        vmin, vmax = self.ctrl.query_axis_limits(self.axis)
        self.position.set_min_value(vmin)
        self.position.set_max_value(vmax)

    def read_position(self):
        pos = self.ctrl.query_axis_position(self.axis)
        moving = self.ctrl.query_axis_moving(self.axis)
        if moving:
            self.set_state(DevState.MOVING)
        else:
            self.set_state(DevState.ON)
        return pos

    def write_position(self, position):
        ans = self.ctrl.set_axis_position([self.axis, str(position)])
        print(f"SET POS: {self.axis} -> {position} (ans={ans})",
              file=self.log_debug)
        if ans == 0:
            self.set_state(DevState.MOVING)

    def read_referenced(self):
        referenced = self.ctrl.query_axis_referenced(self.axis)
        return referenced

    @command
    def stop(self):
        """Smoothly stop motion"""
        self.ctrl.halt()

    @command
    def abort(self):
        """Abruptly stop all motion."""
        self.ctrl.stop()


if __name__ == "__main__":
    # PIGCSController.run_server()
    run((PIGCSController, PIGCSAxis))
