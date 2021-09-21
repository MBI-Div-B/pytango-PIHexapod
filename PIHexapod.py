#!/usr/bin/env python3

'''
PI Hexapod tango device server

@author: M. Schneider, MBI Berlin, 2021
'''

from pipython import GCS2Device
import tango
from tango import AttrWriteType, DevState, DevDouble, DevBoolean
from tango.server import Device, attribute, command, device_property
import sys
import time


class PIHexapod(Device):
    '''
    '''

    host = device_property(
        dtype=str,
        default_value='192.168.0.100',
        doc='controller IP address',
    )

    port = device_property(
        dtype=int,
        default_value=50000,
        doc='controller port',
    )

    def init_device(self):
        '''Establish connection to controller.'''
        super(PIHexapod, self).init_device()
        try:
            print(f'Connecting to Hexpod controller on {self.host}:{self.port}',
                  file=self.log_info)
            self.ctrl = GCS2Device()
            self.ctrl.ConnectTCPIP(self.host, self.port)
            idn = self.ctrl.qIDN()
            print(f'Connection established on {self.host}:\n{idn}',
                  file=self.log_info)
            self._positions = None
            self._limits = None
            self._moving = None
            self._last_query = 0.
            self._query_timeout = .1
            self.set_state(DevState.ON)
        except Exception as ex:
            print(f'Error on initialization: {ex}', file=self.log_error)
            self.set_state(DevState.OFF)
            sys.exit(255)
        
    def initialize_dynamic_attributes(self):
        '''Create axis position and limit switch attributes.

        This gets called automatically on instantiation.
        '''
        for ax in 'xyz':
            self.create_axis_attributes(ax, unit='mm')
        for ax in 'uvw':
            self.create_axis_attributes(ax, unit='deg')
    
    def create_axis_attributes(self, ax:str, **kwargs):
        '''Dynamically create position, limit switch and moving attributes.'''
        attr_pos = tango.Attr(ax, tango.DevDouble, AttrWriteType.READ_WRITE)
        prop = tango.UserDefaultAttrProp()
        for k, v in kwargs.items():
            try:
                property_setter = getattr(prop, 'set_' + k)
                property_setter(v)
            except AttributeError:
                print('error setting attribute property:', ax, k, v,
                      file=self.log_error)
        attr_pos.set_default_properties(prop)
        self.add_attribute(attr_pos, self.read_pos, self.set_pos)

        attr_lim = tango.Attr(f'lim_{ax}', tango.DevBoolean, AttrWriteType.READ)
        self.add_attribute(attr_lim, self.read_limit)

        attr_mov = tango.Attr(f'mov_{ax}', tango.DevBoolean, AttrWriteType.READ)
        self.add_attribute(attr_mov, self.read_is_moving)

    def read_pos(self, attr):
        '''Query all positions and return requested.'''
        name = attr.get_name()
        value = self._positions[name.upper()]
        print(f'READ: {name} = {value}', file=self.log_debug)
        attr.set_value(value)
        return value
    
    def set_pos(self, attr):
        '''Move axis to position.'''
        axis = attr.get_name().upper()
        target = attr.get_write_value()
        print(f'MOVE: {axis} -> {target}', file=self.log_debug)
        self.ctrl.MOV(axis, target)
        error = self.ctrl.qERR()
        if error == 0:
            self.set_state(DevState.MOVING)
    
    def read_limit(self, attr):
        '''Check limit switch state.'''
        name = attr.get_name().upper()[-1]
        value = self._limits[name.upper()]
        print(f'READ: limit switch {name}: {value}', file=self.log_debug)
        attr.set_value(value)
        return value
    
    def read_is_moving(self, attr):
        '''Check whether axis is moving.'''
        name = attr.get_name().upper()[-1]
        value = self._moving[name.upper()]
        print(f'READ: {name} moving: {value}', file=self.log_debug)
        attr.set_value(value)
        return value

    def always_executed_hook(self):
        '''Query all axes positions, limits and move states.'''
        if (time.time() - self._last_query) > self._query_timeout:
            self._positions = self.ctrl.qPOS()
            self._limits = self.ctrl.qLIM()
            self._moving = self.ctrl.IsMoving(list('XYZUVW'))
            if any(self._moving.values()):
                self.set_state(DevState.MOVING)
            else:
                self.set_state(DevState.ON)
            self._last_query = time.time()
            
    @command
    def find_references(self):
        '''Find reference marks for all axes.'''
        self.ctrl.FRF()
        result = self.ctrl.qFRF()
        if all(result.values()):
            self.set_state(DevState.ON)
        else:
            self.set_state(DevState.FAULT)

    @command
    def halt(self):
        '''Halt motion smoothly.'''
        self.ctrl.HLT(noraise=True)
    


if __name__ == '__main__':
    PIHexapod.run_server()