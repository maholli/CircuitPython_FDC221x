from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from math import pi

FDC2212_I2C_ADDR_0   =const(0x2A)
FDC2212_I2C_ADDR_1   =const(0x2B)
# Address is 0x2A (default) or 0x2B (if ADDR is high)

#bitmasks
FDC2212_CH0_UNREADCONV =const(0x0008)         #denotes unread CH0 reading in STATUS register
FDC2212_CH1_UNREADCONV =const(0x0004)         #denotes unread CH1 reading in STATUS register
FDC2212_CH2_UNREADCONV =const(0x0002)         #denotes unread CH2 reading in STATUS register
FDC2212_CH3_UNREADCONV =const(0x0001)         #denotes unread CH3 reading in STATUS register
DEGLITCH_MASK          =const(0xFFF8)

#registers
FDC2212_DEVICE_ID            =const(0x7F)
FDC2212_MUX_CONFIG           =const(0x1B)
FDC2212_CONFIG               =const(0x1A)
FDC2212_RCOUNT_CH0           =const(0x08)
FDC2212_RCOUNT_CH1           =const(0x09)
FDC2212_OFFSET_CH0           =const(0x0C)
FDC2212_OFFSET_CH1           =const(0x0D)
FDC2212_SETTLECOUNT_CH0      =const(0x10)
FDC2212_SETTLECOUNT_CH1      =const(0x11)
FDC2212_CLOCK_DIVIDERS_CH0   =const(0x14)
FDC2212_CLOCK_DIVIDERS_CH1   =const(0x15)
FDC2212_STATUS               =const(0x18)
FDC2212_DATA_CH0_MSB         =const(0x00)
FDC2212_DATA_CH0_LSB         =const(0x01)
FDC2212_DATA_CH1_MSB         =const(0x02)
FDC2212_DATA_CH1_LSB         =const(0x03)
FDC2212_DRIVE_CH0            =const(0x1E)
FDC2212_DRIVE_CH1            =const(0x1F)

# mask for 28bit data to filter out flag bits
FDC2212_DATA_CHx_MASK_DATA          =const(0x0FFF) 
FDC2212_DATA_CHx_MASK_ERRAW         =const(0x1000) 
FDC2212_DATA_CHx_MASK_ERRWD         =const(0x2000) 

class FDC2212(object):
    """
    :param i2c: The `busio.I2C` object to use. This is the only required parameter.
    :param 
    # Class-level buffer to reduce allocations and fragmentation.
    # Note this is NOT thread-safe or re-entrant by design!   
    """
    _BUFFER = bytearray(8)
    def __init__(self, i2c, *, address=FDC2212_I2C_ADDR_0,debug=False):
        self._device = I2CDevice(i2c, address)
        # Check for valid chip ID
        if self._read16(FDC2212_DEVICE_ID) not in (0x3055,0x3054):
            raise RuntimeError('Failed to find FD2212, check wiring!')
        self.debug=debug
        self._mux = 0x020D
        self._config = 0x1C01
        self._fclk=43.3e6 # 43.3 MHz (internal)
        self._L=18e-6 # 18uH
        self._cap=33e-12 # 33pf
        self._diff=False # differential
        self._div=1 # 
        self._Fsense = self._Csense = 0
        self._channel = 0
        self._MSB = FDC2212_DATA_CH0_MSB
        self._LSB = FDC2212_DATA_CH0_LSB

        self._write16(FDC2212_MUX_CONFIG,         self._mux)
        self._write16(FDC2212_CONFIG,             self._config)
        self._write16(FDC2212_RCOUNT_CH0,         0xFFFF)
        self._write16(FDC2212_RCOUNT_CH1,         0xFFFF)
        self._write16(FDC2212_OFFSET_CH0,         0x0000)
        self._write16(FDC2212_OFFSET_CH1,         0x0000)
        self._write16(FDC2212_SETTLECOUNT_CH0,    0x0400)
        self._write16(FDC2212_SETTLECOUNT_CH1,    0x0400)
        self._write16(FDC2212_CLOCK_DIVIDERS_CH0, 0x1001)
        self._write16(FDC2212_CLOCK_DIVIDERS_CH1, 0x1001)
        self._write16(FDC2212_DRIVE_CH0,          0x8C40)
        self._write16(FDC2212_DRIVE_CH1,          0x8C40)
    
    @property
    def clock(self):
        return self._fclk
    @clock.setter
    def clock(self, freq):
        self._fclk = freq 
    
    @property
    def inductance(self):
        return self._L
    @inductance.setter
    def inductance(self, induc):
        self._L = induc 

    @property
    def capacitance(self):
        return self._cap
    @capacitance.setter
    def clock(self, cap):
        self._cap = cap 

    @property
    def divider(self):
        # Sets differential/single-ended for BOTH channels
        return self._divider
    @divider.setter
    def divider(self, div):
        self._divider = div
        if self._divider:
            # differential
            self._write16(FDC2212_CLOCK_DIVIDERS_CH0, 0x2001)
            self._write16(FDC2212_CLOCK_DIVIDERS_CH1, 0x2001)
            self._div = 2
        else:
            # single-ended
            self._write16(FDC2212_CLOCK_DIVIDERS_CH0, 0x1001)
            self._write16(FDC2212_CLOCK_DIVIDERS_CH1, 0x1001)
            self._div = 1
    @property
    def sleep(self):
        return self._sleep
    @sleep.setter
    def sleep(self,value):
        if value:
            self._config |= (1<<13)
        else:
            self._config &= ~(1<<13)            
        self._write16(FDC2212_CONFIG, self._config)
    
    @property
    def scan(self):
        return self._scan
    @scan.setter
    def scan(self, value):
        if value:
            self._mux |= (1<<15)
        else:
            self._mux &= ~(1<<15)            
        self._write16(FDC2212_MUX_CONFIG, self._mux)

    @property
    def deglitch(self):
        return self._deglitch
    @deglitch.setter
    def deglitch(self, value):
        '''
        Input deglitch filter bandwidth.
        Select the lowest setting that exceeds the oscillation tank
        oscillation frequency.
        1MHz,3.3MHz,10MHz,33MHz
        '''
        if value not in (1,4,5,7):
            raise ValueError("Unsupported deglitch setting.")
        self._mux = (self._mux & DEGLITCH_MASK) | value
        self._write16(FDC2212_MUX_CONFIG, self._mux)
        if self.debug: print(hex(self._mux))

    @property
    def channel(self):
        return self._channel
    @channel.setter
    def channel(self, channel=0):
        if channel not in (0,1):
            raise ValueError("Unsupported channel.")
        if channel == 0:
            self._config &= ~(1<<14)
            self._write16(FDC2212_CONFIG,self._config)
            self._MSB = FDC2212_DATA_CH0_MSB
            self._LSB = FDC2212_DATA_CH0_LSB
        if channel == 1:
            self._config |= (1<<14)
            self._write16(FDC2212_CONFIG,self._config)
            self._MSB = FDC2212_DATA_CH1_MSB
            self._LSB = FDC2212_DATA_CH1_LSB
        self._channel = channel

    def _read_into(self, address, buf, count=None):
        # Read bytes from the specified address into the provided buffer.
        # If count is not specified (the default) the entire buffer is filled,
        # otherwise only count bytes are copied in.
        assert len(buf) > 0
        if count is None:
            count = len(buf)
        with self._device as i2c:
            i2c.write_then_readinto(bytes([address & 0xFF]), buf,
                                    in_end=count, stop=False)    
    def _read16(self, address):
        # Read a 16-bit unsigned value for from the specified address.
        self._read_into(address, self._BUFFER, count=2)
        return self._BUFFER[0] << 8 | self._BUFFER[1]

    def _write16(self, address, value):
        # Write a 16-bit unsigned value to the specified address.
        with self._device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = (value >> 8) & 0xFF
            i2c.write(self._BUFFER, end=2)

    def _read_raw(self):
        _reading = (self._read16(self._MSB) & FDC2212_DATA_CHx_MASK_DATA) << 16
        _reading |= self._read16(self._LSB)
        return _reading

    def read(self):
        _reading = self._read_raw()
        try:
            # calculate fsensor (40MHz external ref)
            self._Fsense=(self._div*_reading*self._fclk/(2**28))
            # calculate Csensor (18uF and 33pF LC tank)
            self._Csense = (1e12)*((1/(self._L*(2*pi*self._Fsense)**2))-self._cap)
        except Exception as e:
            if self.debug: print('Error on read:',e)
            pass
        return self._Csense