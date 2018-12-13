# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from usv_sim/actuator.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class actuator(genpy.Message):
  _md5sum = "058476a8fb2069bcb447843b2fbad502"
  _type = "usv_sim/actuator"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 T_p
float64 Alpha_p
float64 T_s
float64 Alpha_s
"""
  __slots__ = ['T_p','Alpha_p','T_s','Alpha_s']
  _slot_types = ['float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       T_p,Alpha_p,T_s,Alpha_s

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(actuator, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.T_p is None:
        self.T_p = 0.
      if self.Alpha_p is None:
        self.Alpha_p = 0.
      if self.T_s is None:
        self.T_s = 0.
      if self.Alpha_s is None:
        self.Alpha_s = 0.
    else:
      self.T_p = 0.
      self.Alpha_p = 0.
      self.T_s = 0.
      self.Alpha_s = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_4d().pack(_x.T_p, _x.Alpha_p, _x.T_s, _x.Alpha_s))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.T_p, _x.Alpha_p, _x.T_s, _x.Alpha_s,) = _get_struct_4d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_4d().pack(_x.T_p, _x.Alpha_p, _x.T_s, _x.Alpha_s))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.T_p, _x.Alpha_p, _x.T_s, _x.Alpha_s,) = _get_struct_4d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
