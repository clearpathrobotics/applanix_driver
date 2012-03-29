
import applanix_bridge.msg as common

# Node source 
from translator import Translator

# Python
import threading
import socket
import struct
from cStringIO import StringIO


class Port(threading.Thread):
  """ Common base class for DataPort and ControlPort. Provides functionality to
      recv/send Applanix-formatted packets from the socket. Could in future 
      support LoggingPort and DisplayPort."""
  checksum_struct = struct.Struct("<hh")

  def __init__(self, sock, **opts):
    super(Port, self).__init__()
    self.sock = sock
    self.opts = opts
    self.daemon = False
    self.finish = threading.Event()

    # These are only for receiving. 
    self.header = common.CommonHeader()
    self.footer = common.CommonFooter()

  def recv(self):
    """ Receive a packet from the port's socket.
        Returns (pkt_id, pkt_str), where pkt_id is ("$GRP"|"$MSG", num)
        Returns None, None when no data. """
    try:
      header_str = self.sock.recv(Translator.for_msg(self.header).size)
    except socket.timeout:
      return None, None

    header_data = StringIO(header_str)
    Translator.for_msg(self.header).deserialize(header_data)
    pkt_id = (str(self.header.start).encode('string_escape'), self.header.id)

    # Initial sanity check.
    if pkt_id[0] not in (common.CommonHeader.START_GROUP, common.CommonHeader.START_MESSAGE):
      raise ValueError("Bad header %s.%d" % pkt_id)

    # Special case for a troublesome undocumented packet.
    if pkt_id == ("$GRP", 20015):
      self.sock.recv(135)
      return None, None

    # Receive remainder of packet from data socket. 
    pkt_str = self.sock.recv(self.header.length)

    # Check package footer.
    footer_data = StringIO(pkt_str[-Translator.for_msg(self.footer).size:])
    Translator.for_msg(self.footer).deserialize(footer_data)
    if str(self.footer.end) != common.CommonFooter.END:
      raise("Bad footer from pkt %s.%d" % pkt_id)

    # Check package checksum.
    if self._checksum(StringIO(header_str + pkt_str)) != 0:
      raise("Bad checksum from pkt %s.%d: %%d" % pkt_id % checksum)

    return pkt_id, pkt_str 

  def send(self, header, msg):
    """ Sends a header/msg/footer out the socket. Takes care of computing
        length field for header and checksum field for footer. """
    msg_buff = StringIO()
    Translator.for_msg(msg).preserialize() 
    Translator.for_msg(msg).serialize(msg_buff)

    header.length = msg_buff.tell() + Translator.for_msg(self.footer).size
    footer = common.CommonFooter(end=common.CommonFooter.END)

    # Write header and message to main buffer.
    buff = StringIO()
    Translator.for_msg(header).serialize(buff)
    buff.write(msg_buff.getvalue())
    
    # Write pad bytes.
    pad_count = -buff.tell() % 4
    buff.write("\x00" * pad_count)

    # Write footer.
    footer_start = buff.tell()
    Translator.for_msg(footer).serialize(buff) 

    # Compute checksum.
    buff.seek(0)
    footer.checksum = 65536 - self._checksum(buff)

    # Rewrite footer with correct checksum.
    buff.seek(footer_start)
    Translator.for_msg(footer).serialize(buff) 

    #print buff.getvalue().encode("string_escape")
    self.sock.send(buff.getvalue())

  @classmethod
  def _checksum(cls, buff):
    """ Compute Applanix checksum. Expects a StringIO with a 
      size that is a multiple of four bytes. """
    checksum = 0
    while True:
      data = buff.read(cls.checksum_struct.size)
      if len(data) == 0:
        break
      if len(data) < 4:
        raise ValueError("Checksum data length is not a multiple of 4.")
      c1, c2 = cls.checksum_struct.unpack(data)
      checksum += c1 + c2
    return checksum % 65536

