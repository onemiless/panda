#!/usr/bin/env python3
import os
import sys
import struct
import hashlib
from Crypto.PublicKey import RSA
import binascii

# increment this to make new hardware not run old versions
VERSION = 2

if __name__ == "__main__":
  with open(sys.argv[3]) as k:
    rsa = RSA.importKey(k.read())

  with open(sys.argv[1], "rb") as f:
    dat = f.read()

  print("signing", len(dat), "bytes")

  if os.getenv("SETLEN") is not None:
    # add the version at the end
    dat += b"VERS" + struct.pack("I", VERSION)
    # add the length at the beginning
    x = struct.pack("I", len(dat)) + dat[4:]
    # mock signature of dat[4:]
    dd = hashlib.sha1(dat[4:]).digest()
  else:
    x = dat
    dd = hashlib.sha1(dat).digest()

  print("hash:", str(binascii.hexlify(dd), "utf-8"))
  dd = b"\x00\x01" + b"\xff" * 0x69 + b"\x00" + dd
  rsa_out = pow(int.from_bytes(dd, byteorder='big', signed=False), rsa.d, rsa.n)
  sig = (hex(rsa_out)[2:].rjust(0x100, '0'))
  x += binascii.unhexlify(sig)

  max_size = os.getenv("MAX_SIZE")
  if max_size is not None and len(x) > int(max_size, 0):
    raise ValueError(f"signed image is {len(x)} bytes, exceeds reserved application space {int(max_size, 0)}")

  with open(sys.argv[2], "wb") as f:
    f.write(x)
