import fdc2212,board,busio
import time

i2c = busio.I2C(board.SCL,board.SDA)
cap = fdc2212.FDC2212(i2c, debug=False)

while True:
    print(time.monotonic(),'{:.9E}'.format(cap.read()))
    time.sleep(1)

