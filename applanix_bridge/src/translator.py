
import roslib, roslib.genpy
import struct
from itertools import izip
from cStringIO import StringIO


class EndOfBuffer(BaseException):
  pass


class TranslatorError(ValueError):
  pass


class SubMessageHandler:
  def __init__(self, field):
    self.name = field.name
    self.msg_cls = roslib.message.get_message_class(field.type)
    self.translator = get(self.msg_cls)

  def deserialize(self, buff, msg):
    submessage = self.msg_cls()
    self.translator.deserialize(buff, submessage)
    setattr(msg, self.name, submessage)


class FixedFieldsHandler:
  def __init__(self, fields):
    struct_strs = ['<']
    def pattern(field):
      try:
        return roslib.genpy.SIMPLE_TYPES_DICT[field.type]
      except KeyError:
        if field.base_type in ['uint8', 'char'] and field.array_len is not None:
          return "%is" % field.array_len
        else:
          raise
          
    struct_strs.extend([pattern(f) for f in fields])
    self.struct = struct.Struct(''.join(struct_strs))
    self.names = [f.name for f in fields]
    self.size = self.struct.size

  def serialize(self, buff, msg):
    pass

  def deserialize(self, buff, msg):
    st = buff.read(self.struct.size)
    if st == '': raise EndOfBuffer()
    values = self.struct.unpack(st) 
    for name, value in izip(self.names, values):
      setattr(msg, name, value)


class SubMessageArrayHandler:
  struct_uint16 = struct.Struct('<H')
  struct_uint8 = struct.Struct('<B')

  def __init__(self, field):
    self.name = field.name
    self.msg_cls = roslib.message.get_message_class(field.base_type)
    self.translator = get(self.msg_cls)

  def deserialize(self, buff, msg):
    if msg.array_mode == "uint16_bytes":
      length = self.struct_uint16.unpack(buff.read(self.struct_uint16.size))[0]
      data = StringIO(buff.read(length))
    elif msg.array_mode == "uint8_items":
      length = self.struct_uint8.unpack(buff.read(self.struct_uint8.size))[0] * self.translator.size
      data = StringIO(buff.read(length))
    elif msg.array_mode == "uint16_items":
      length = self.struct_uint16.unpack(buff.read(self.struct_uint16.size))[0] * self.translator.size
      data = StringIO(buff.read(length))
    elif msg.array_mode == "infer":
      data = buff
    else:
      raise ValueError("Unrecognized array_mode.")

    # Find and empty the array to be populated.
    array = getattr(msg, self.name)
    array[:] = []

    try:
      while True:
        submessage = self.msg_cls()
        self.translator.deserialize(data, submessage)
        array.append(submessage)
    except EndOfBuffer:
      pass


class VariableStringHandler:
  struct_bytes = struct.Struct('<H')

  def __init__(self, field):
    self.name = field.name

  def deserialize(self, buff, msg):
    # TODO: mechanism to skip length check for Group 26. (grrr)
    length = self.struct_bytes.unpack(buff.read(self.struct_bytes.size))[0]
    setattr(msg, self.name, str(buff.read(length)))


class Translator:
  def __init__(self, msg_cls):
    self.handlers = []

    cls_name, spec = roslib.msgs.load_by_type(msg_cls._type)

    fixed_fields = []
    for field in spec.parsed_fields():
      if roslib.genpy.is_simple(field.base_type) and (field.array_len != None or not field.is_array):
        # Simple types and fixed-length character arrays.
        fixed_fields.append(field)
      else:
        # Before dealing with this non-simple field, add a handler for the fixed fields
        # encountered so far.
        if len(fixed_fields) > 0:
          self.handlers.append(FixedFieldsHandler(fixed_fields))
          fixed_fields = []

        # Handle this other type.
        if field.type == 'string' or (field.base_type == 'uint8' and field.is_array):
          self.handlers.append(VariableStringHandler(field))
        elif field.is_array:
          self.handlers.append(SubMessageArrayHandler(field))
        else:
          self.handlers.append(SubMessageHandler(field))

    if len(fixed_fields) > 0:
      self.handlers.append(FixedFieldsHandler(fixed_fields))

    if len(self.handlers) == 1 and hasattr(self.handlers[0], 'size'):
      self.size = self.handlers[0].size

  def deserialize(self, buff, msg):
    try:
      for handler in self.handlers:
        handler.deserialize(buff, msg)
    except struct.error as e:
      raise TranslatorError(e)

  def serialize(self, buff, msg):
    for handler in self.handlers:
      handler.serialize(buff, msg)


def get(msg_cls):
  # Lazy initialize these, so we don't create them all upfront
  # when we're only using a few.
  if not hasattr(msg_cls, "translator"):
    msg_cls.translator = Translator(msg_cls)
  return msg_cls.translator
