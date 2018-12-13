# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from usv_towing/VehicleState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class VehicleState(genpy.Message):
  _md5sum = "267dc2162d9371f64d61a71605f7ca88"
  _type = "usv_towing/VehicleState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 t
geometry_msgs/Pose2D pose
geometry_msgs/Twist vel
geometry_msgs/Twist acc
================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['t','pose','vel','acc']
  _slot_types = ['float64','geometry_msgs/Pose2D','geometry_msgs/Twist','geometry_msgs/Twist']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       t,pose,vel,acc

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(VehicleState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.t is None:
        self.t = 0.
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      if self.vel is None:
        self.vel = geometry_msgs.msg.Twist()
      if self.acc is None:
        self.acc = geometry_msgs.msg.Twist()
    else:
      self.t = 0.
      self.pose = geometry_msgs.msg.Pose2D()
      self.vel = geometry_msgs.msg.Twist()
      self.acc = geometry_msgs.msg.Twist()

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
      buff.write(_get_struct_16d().pack(_x.t, _x.pose.x, _x.pose.y, _x.pose.theta, _x.vel.linear.x, _x.vel.linear.y, _x.vel.linear.z, _x.vel.angular.x, _x.vel.angular.y, _x.vel.angular.z, _x.acc.linear.x, _x.acc.linear.y, _x.acc.linear.z, _x.acc.angular.x, _x.acc.angular.y, _x.acc.angular.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      if self.vel is None:
        self.vel = geometry_msgs.msg.Twist()
      if self.acc is None:
        self.acc = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 128
      (_x.t, _x.pose.x, _x.pose.y, _x.pose.theta, _x.vel.linear.x, _x.vel.linear.y, _x.vel.linear.z, _x.vel.angular.x, _x.vel.angular.y, _x.vel.angular.z, _x.acc.linear.x, _x.acc.linear.y, _x.acc.linear.z, _x.acc.angular.x, _x.acc.angular.y, _x.acc.angular.z,) = _get_struct_16d().unpack(str[start:end])
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
      buff.write(_get_struct_16d().pack(_x.t, _x.pose.x, _x.pose.y, _x.pose.theta, _x.vel.linear.x, _x.vel.linear.y, _x.vel.linear.z, _x.vel.angular.x, _x.vel.angular.y, _x.vel.angular.z, _x.acc.linear.x, _x.acc.linear.y, _x.acc.linear.z, _x.acc.angular.x, _x.acc.angular.y, _x.acc.angular.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      if self.vel is None:
        self.vel = geometry_msgs.msg.Twist()
      if self.acc is None:
        self.acc = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 128
      (_x.t, _x.pose.x, _x.pose.y, _x.pose.theta, _x.vel.linear.x, _x.vel.linear.y, _x.vel.linear.z, _x.vel.angular.x, _x.vel.angular.y, _x.vel.angular.z, _x.acc.linear.x, _x.acc.linear.y, _x.acc.linear.z, _x.acc.angular.x, _x.acc.angular.y, _x.acc.angular.z,) = _get_struct_16d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_16d = None
def _get_struct_16d():
    global _struct_16d
    if _struct_16d is None:
        _struct_16d = struct.Struct("<16d")
    return _struct_16d