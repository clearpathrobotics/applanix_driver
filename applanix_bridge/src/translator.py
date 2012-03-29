
import roslib, roslib.genpy
import struct
from itertools import izip
from cStringIO import StringIO
from functools import partial


class EndOfBuffer(BaseException):
  pass


class TranslatorError(ValueError):
  pass


class Handler(object):
  def field(self, msg):
    return getattr(msg, self.name)

  def deserialize(self, buff, msg):
    """ Read bytes from buff to populate msg. """
    raise NotImplementedError()

  def serialize(self, buff, msg):
    """ Write bytes to buff in accordance with contents of msg. """
    raise NotImplementedError()

  def preserialize(self, msg):
    """ If necessary, populate any length fields or other metadata
        in preparation for serializing. """
    pass


class SubMessageHandler(Handler):
  def __init__(self, field):
    self.name = field.name

  def deserialize(self, buff, msg):
    Translator.for_msg(self.field(msg)).deserialize(buff)

  def serialize(self, buff, msg):
    Translator.for_msg(self.field(msg)).serialize(buff)

  def preserialize(self, msg):
    Translator.for_msg(self.field(msg)).preserialize()


class FixedFieldsHandler(Handler):
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
    buff.write(self.struct.pack(*[getattr(msg, name) for name in self.names]))

  def deserialize(self, buff, msg):
    st = buff.read(self.struct.size)
    if st == '': raise EndOfBuffer()
    values = self.struct.unpack(st) 
    for name, value in izip(self.names, values):
      setattr(msg, name, value)


class SubMessageArrayHandler(Handler):
  struct_uint16 = struct.Struct('<H')
  struct_uint8 = struct.Struct('<B')

  def __init__(self, field):
    self.name = field.name
    self.name_count = "%s_count" % self.name
    self.msg_cls = roslib.message.get_message_class(field.base_type)
    self.item_size = Translator.for_msg_cls(self.msg_cls).size

  def deserialize(self, buff, msg):
    if hasattr(msg, self.name_count):
      # Another field specifies number of array items to deserialize.
      length = getattr(msg, self.name_count) * self.item_size
      data = StringIO(buff.read(length))
    else:
      # Consume as much as we can straight from the buffer.
      data = buff

    # Find and empty the array to be populated.
    array = self.field(msg)
    array[:] = []

    try:
      while True:
        submessage = self.msg_cls()
        Translator.for_msg(submessage).deserialize(data)
        array.append(submessage)
    except EndOfBuffer:
      pass

  def serialize(self, buff, msg):
    pass

  def preserialize(self, buff, msg):
    if hasattr(msg, self.name_count):
      setattr(msg, self.name_count, len(field(msg)))


class VariableStringHandler(Handler):
  struct_bytes = struct.Struct('<H')

  def __init__(self, field):
    self.name = field.name

  def deserialize(self, buff, msg):
    length = self.struct_bytes.unpack(buff.read(self.struct_bytes.size))[0]
    setattr(msg, self.name, str(buff.read(length)))


class Translator(object):
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
    try:
      for handler in self.handlers:
        handler.serialize(buff, msg)
    except struct.error as e:
      raise TranslatorError(e)

  def preserialize(self, msg):
    for handler in self.handlers:
      handler.preserialize(msg)

  @classmethod
  def for_msg_cls(cls, msg_cls):
    """ Create the translator for a message class. """
    if not hasattr(msg_cls, "translator"):
      msg_cls.translator = cls(msg_cls)
    return msg_cls.translator

  @classmethod
  def for_msg(cls, msg):
    """ Return a translator bound to a specific message instance. """
    return TranslatorProxy(cls.for_msg_cls(msg.__class__), msg)


class TranslatorProxy(object):
  """ Convenience class which binds a translator to a specific msg. """
  def __init__(self, translator, msg):
    self.translator = translator
    self.msg = msg

  def __getattr__(self, name):
    attr = getattr(self.translator, name)
    if callable(attr): 
      attr = partial(attr, msg=self.msg)
    setattr(self, name, attr)
    return attr
