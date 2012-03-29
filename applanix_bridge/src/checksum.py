
import struct


checksum_struct = struct.Struct("<hh")

def checksum(buff):
  """ Compute Applanix checksum. Expects a StringIO with a 
      size that is a multiple of four bytes. """
  checksum = 0
  while True:
    data = buff.read(checksum_struct.size)
    if len(data) == 0:
      break
    if len(data) < 4:
      raise ValueError("Checksum data length is not a multiple of 4.")
    c1, c2 = checksum_struct.unpack(data)
    checksum += c1 + c2
  return checksum % 65536

