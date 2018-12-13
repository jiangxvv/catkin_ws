# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from usv_sim/WaypointTrackingGoal.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class WaypointTrackingGoal(genpy.Message):
  _md5sum = "8ec2419d2a0aec459c39bee2c36ebd66"
  _type = "usv_sim/WaypointTrackingGoal"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# goal definition
float64[] pos_x
float64[] pos_y
float64[] heading
bool mission_type
float64 mission_duration
"""
  __slots__ = ['pos_x','pos_y','heading','mission_type','mission_duration']
  _slot_types = ['float64[]','float64[]','float64[]','bool','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pos_x,pos_y,heading,mission_type,mission_duration

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(WaypointTrackingGoal, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pos_x is None:
        self.pos_x = []
      if self.pos_y is None:
        self.pos_y = []
      if self.heading is None:
        self.heading = []
      if self.mission_type is None:
        self.mission_type = False
      if self.mission_duration is None:
        self.mission_duration = 0.
    else:
      self.pos_x = []
      self.pos_y = []
      self.heading = []
      self.mission_type = False
      self.mission_duration = 0.

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
      length = len(self.pos_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.pos_x))
      length = len(self.pos_y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.pos_y))
      length = len(self.heading)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.heading))
      _x = self
      buff.write(_get_struct_Bd().pack(_x.mission_type, _x.mission_duration))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.pos_x = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.pos_y = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.heading = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 9
      (_x.mission_type, _x.mission_duration,) = _get_struct_Bd().unpack(str[start:end])
      self.mission_type = bool(self.mission_type)
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
      length = len(self.pos_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.pos_x.tostring())
      length = len(self.pos_y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.pos_y.tostring())
      length = len(self.heading)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.heading.tostring())
      _x = self
      buff.write(_get_struct_Bd().pack(_x.mission_type, _x.mission_duration))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.pos_x = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.pos_y = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.heading = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 9
      (_x.mission_type, _x.mission_duration,) = _get_struct_Bd().unpack(str[start:end])
      self.mission_type = bool(self.mission_type)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_Bd = None
def _get_struct_Bd():
    global _struct_Bd
    if _struct_Bd is None:
        _struct_Bd = struct.Struct("<Bd")
    return _struct_Bd