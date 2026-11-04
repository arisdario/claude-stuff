#!/usr/bin/env python3
"""Is this USB device capable of HIGH speed (480 Mb/s), whatever speed it is running at now?  Read-only: one standard GET_DESCRIPTOR request.

  python3 usb_hs_capable.py 0a05:7211        (vendor:product, as in lsusb / /sys/bus/usb/devices/*/idVendor)

A high-speed-capable device answers a DEVICE_QUALIFIER request (descriptor type 6) in any mode; a full-speed-only device refuses it (the
request stalls). So an answer means "capable of 480 Mb/s: if it runs at 12 the link, cable or adapter is holding it back", and a stall means
"full-speed only". Needs Linux usbfs (/dev/bus/usb) and root; changes nothing.
"""
import ctypes
import errno
import fcntl
import os
import struct
import sys

USBDEVFS_CONTROL = 0xC0185500          # _IOWR('U', 0, struct usbdevfs_ctrltransfer) on a 64-bit kernel (24-byte struct)


def find(vid, pid):
  base = '/sys/bus/usb/devices'
  for d in sorted(os.listdir(base)):
    p = os.path.join(base, d)
    try:
      if open(os.path.join(p, 'idVendor')).read().strip() == vid and open(os.path.join(p, 'idProduct')).read().strip() == pid:
        return d, int(open(os.path.join(p, 'busnum')).read()), int(open(os.path.join(p, 'devnum')).read()), open(os.path.join(p, 'speed')).read().strip()
    except (OSError, ValueError):
      continue
  return None


def main():
  if len(sys.argv) != 2 or ':' not in sys.argv[1]:
    sys.exit(__doc__)
  vid, pid = sys.argv[1].lower().split(':')
  hit = find(vid, pid)
  if not hit:
    sys.exit('no device %s:%s found' % (vid, pid))
  name, bus, dev, speed = hit
  print('device %s (bus %d, device %d) is running at %s Mb/s' % (name, bus, dev, speed))
  path = '/dev/bus/usb/%03d/%03d' % (bus, dev)
  buf = ctypes.create_string_buffer(10)
  # bmRequestType 0x80 (device to host, standard, device), bRequest 6 (GET_DESCRIPTOR), wValue 0x0600 (DEVICE_QUALIFIER), wIndex 0, wLength 10
  req = struct.pack('<BBHHHI4xQ', 0x80, 6, 0x0600, 0, 10, 2000, ctypes.addressof(buf))
  assert len(req) == 24
  fd = os.open(path, os.O_RDWR)
  try:
    n = fcntl.ioctl(fd, USBDEVFS_CONTROL, req)
    data = bytes(buf.raw[:10])
    print('answered the device-qualifier request: %s' % data.hex())
    print('=> CAPABLE of high speed (480 Mb/s). It runs at %s Mb/s, so the cable, adapter or connection is holding it back.' % speed)
  except OSError as e:
    if e.errno in (errno.EPIPE, errno.EPROTO, errno.EOVERFLOW):
      print('the device refused the device-qualifier request (%s)' % os.strerror(e.errno))
      print('=> FULL-SPEED ONLY (12 Mb/s): it cannot carry the camera, whatever its label says.')
    else:
      print('could not tell (%s)' % e)
  finally:
    os.close(fd)


if __name__ == '__main__':
  main()
