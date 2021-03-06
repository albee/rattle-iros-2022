# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from reswarm_msgs_matlab/ReswarmMsgMRPI.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class ReswarmMsgMRPI(genpy.Message):
  _md5sum = "58a198290d3ee8c3c6acb609374259e3"
  _type = "reswarm_msgs_matlab/ReswarmMsgMRPI"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """std_msgs/Float64MultiArray K
std_msgs/Float64MultiArray Au
std_msgs/Float64MultiArray bu
std_msgs/Float64MultiArray AZ
std_msgs/Float64MultiArray bZ
std_msgs/Bool using_fallback_mrpi

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/Bool
bool data"""
  __slots__ = ['K','Au','bu','AZ','bZ','using_fallback_mrpi']
  _slot_types = ['std_msgs/Float64MultiArray','std_msgs/Float64MultiArray','std_msgs/Float64MultiArray','std_msgs/Float64MultiArray','std_msgs/Float64MultiArray','std_msgs/Bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       K,Au,bu,AZ,bZ,using_fallback_mrpi

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ReswarmMsgMRPI, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.K is None:
        self.K = std_msgs.msg.Float64MultiArray()
      if self.Au is None:
        self.Au = std_msgs.msg.Float64MultiArray()
      if self.bu is None:
        self.bu = std_msgs.msg.Float64MultiArray()
      if self.AZ is None:
        self.AZ = std_msgs.msg.Float64MultiArray()
      if self.bZ is None:
        self.bZ = std_msgs.msg.Float64MultiArray()
      if self.using_fallback_mrpi is None:
        self.using_fallback_mrpi = std_msgs.msg.Bool()
    else:
      self.K = std_msgs.msg.Float64MultiArray()
      self.Au = std_msgs.msg.Float64MultiArray()
      self.bu = std_msgs.msg.Float64MultiArray()
      self.AZ = std_msgs.msg.Float64MultiArray()
      self.bZ = std_msgs.msg.Float64MultiArray()
      self.using_fallback_mrpi = std_msgs.msg.Bool()

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
      length = len(self.K.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.K.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.K.layout.data_offset))
      length = len(self.K.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.K.data))
      length = len(self.Au.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.Au.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.Au.layout.data_offset))
      length = len(self.Au.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.Au.data))
      length = len(self.bu.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.bu.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.bu.layout.data_offset))
      length = len(self.bu.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.bu.data))
      length = len(self.AZ.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.AZ.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.AZ.layout.data_offset))
      length = len(self.AZ.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.AZ.data))
      length = len(self.bZ.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.bZ.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.bZ.layout.data_offset))
      length = len(self.bZ.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.bZ.data))
      buff.write(_get_struct_B().pack(self.using_fallback_mrpi.data))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.K is None:
        self.K = std_msgs.msg.Float64MultiArray()
      if self.Au is None:
        self.Au = std_msgs.msg.Float64MultiArray()
      if self.bu is None:
        self.bu = std_msgs.msg.Float64MultiArray()
      if self.AZ is None:
        self.AZ = std_msgs.msg.Float64MultiArray()
      if self.bZ is None:
        self.bZ = std_msgs.msg.Float64MultiArray()
      if self.using_fallback_mrpi is None:
        self.using_fallback_mrpi = std_msgs.msg.Bool()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.K.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.K.layout.dim.append(val1)
      start = end
      end += 4
      (self.K.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.K.data = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.Au.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.Au.layout.dim.append(val1)
      start = end
      end += 4
      (self.Au.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.Au.data = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.bu.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.bu.layout.dim.append(val1)
      start = end
      end += 4
      (self.bu.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.bu.data = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.AZ.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.AZ.layout.dim.append(val1)
      start = end
      end += 4
      (self.AZ.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AZ.data = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.bZ.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.bZ.layout.dim.append(val1)
      start = end
      end += 4
      (self.bZ.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.bZ.data = struct.unpack(pattern, str[start:end])
      start = end
      end += 1
      (self.using_fallback_mrpi.data,) = _get_struct_B().unpack(str[start:end])
      self.using_fallback_mrpi.data = bool(self.using_fallback_mrpi.data)
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
      length = len(self.K.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.K.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.K.layout.data_offset))
      length = len(self.K.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.K.data.tostring())
      length = len(self.Au.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.Au.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.Au.layout.data_offset))
      length = len(self.Au.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.Au.data.tostring())
      length = len(self.bu.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.bu.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.bu.layout.data_offset))
      length = len(self.bu.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.bu.data.tostring())
      length = len(self.AZ.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.AZ.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.AZ.layout.data_offset))
      length = len(self.AZ.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.AZ.data.tostring())
      length = len(self.bZ.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.bZ.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.size, _x.stride))
      buff.write(_get_struct_I().pack(self.bZ.layout.data_offset))
      length = len(self.bZ.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.bZ.data.tostring())
      buff.write(_get_struct_B().pack(self.using_fallback_mrpi.data))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.K is None:
        self.K = std_msgs.msg.Float64MultiArray()
      if self.Au is None:
        self.Au = std_msgs.msg.Float64MultiArray()
      if self.bu is None:
        self.bu = std_msgs.msg.Float64MultiArray()
      if self.AZ is None:
        self.AZ = std_msgs.msg.Float64MultiArray()
      if self.bZ is None:
        self.bZ = std_msgs.msg.Float64MultiArray()
      if self.using_fallback_mrpi is None:
        self.using_fallback_mrpi = std_msgs.msg.Bool()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.K.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.K.layout.dim.append(val1)
      start = end
      end += 4
      (self.K.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.K.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.Au.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.Au.layout.dim.append(val1)
      start = end
      end += 4
      (self.Au.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.Au.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.bu.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.bu.layout.dim.append(val1)
      start = end
      end += 4
      (self.bu.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.bu.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.AZ.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.AZ.layout.dim.append(val1)
      start = end
      end += 4
      (self.AZ.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.AZ.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.bZ.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _get_struct_2I().unpack(str[start:end])
        self.bZ.layout.dim.append(val1)
      start = end
      end += 4
      (self.bZ.layout.data_offset,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.bZ.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 1
      (self.using_fallback_mrpi.data,) = _get_struct_B().unpack(str[start:end])
      self.using_fallback_mrpi.data = bool(self.using_fallback_mrpi.data)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
