#!/usr/bin/python

import roslib; roslib.load_manifest('applanix_bridge')
import rospy
import applanix_msgs.msg
from functools import partial

a = applanix_msgs.msg.IMUData()
b = applanix_msgs.msg.IMUData()
c = applanix_msgs.msg.DMIData()


class Translator(object):
  def __init__(self, blah):
    pass
  def foo(self, msg):
    return "foo", msg.__class__
  def bar(self, a, msg):
    return "bar", msg.__class__, a
  def baz(self, b, msg):
    return "baz", msg.__class__, b


class TranslatorProxy(object):
  """ Convenience class which binds a translator to a specific msg. """
  def __init__(self, translator, msg):
    self.translator = translator
    self.msg = msg


def mk_wrapper(fn):
  def wrapper(self, *args):
    return fn(self, *args, msg=self.msg)
  return wrapper


for name, fn in Translator.__dict__.items():
  if callable(fn) and not hasattr(TranslatorProxy, name):
    setattr(TranslatorProxy, name, mk_wrapper(fn))


'''class Translateable(object):
  @classmethod
  def get_cls_translator(cls):
    if not hasattr(cls, "_translator"):
      cls._translator = Translator("abc")
    return cls._translator

  @property
  def translator(self):
    """ Can't cache this as Message instances have fixed attributes. """
    return TranslatorProxy(self.get_cls_translator(), self)


def mixin(base, addition):
  """ Mixes in place, i.e. the base class is modified. """
  for item, val in addition.__dict__.items():
    if not hasattr(base, item):
      setattr(base, item, val)
'''
def get_cls_translator(cls):
  if not hasattr(cls, "_translator"):
    cls._translator = Translator("abc")   return cls._translator

def translator(self):
  """ Can't cache this as Message instances have fixed attributes. """
  return TranslatorProxy(self.get_cls_translator(), self)

roslib.message.Message.get_cls_translator = classmethod(get_cls_translator)
roslib.message.Message.translator = property(translator)

#mixin(roslib.message.Message, Translateable)

print a
print a.translator.foo()
print a.translator.bar("barr")
print a.translator.baz("bbaz")
print b.translator.bar("barr")
print c.translator.baz("barr")
