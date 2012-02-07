"""autogenerated by genmsg_py from LaserBlobs.msg. Do not edit."""
import roslib.message
import struct

import this_robot.msg
import std_msgs.msg

class LaserBlobs(roslib.message.Message):
  _md5sum = "7a472916c2c09f109a03f0585dacd394"
  _type = "this_robot/LaserBlobs"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

int16 blob_count
this_robot/LaserBlob[] blobs

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: this_robot/LaserBlob
int32 start
int32 end
float32 distance
float32 width
float32 angle

"""
  __slots__ = ['header','angle_min','angle_max','angle_increment','time_increment','scan_time','range_min','range_max','blob_count','blobs']
  _slot_types = ['Header','float32','float32','float32','float32','float32','float32','float32','int16','this_robot/LaserBlob[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max,blob_count,blobs
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(LaserBlobs, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.angle_min is None:
        self.angle_min = 0.
      if self.angle_max is None:
        self.angle_max = 0.
      if self.angle_increment is None:
        self.angle_increment = 0.
      if self.time_increment is None:
        self.time_increment = 0.
      if self.scan_time is None:
        self.scan_time = 0.
      if self.range_min is None:
        self.range_min = 0.
      if self.range_max is None:
        self.range_max = 0.
      if self.blob_count is None:
        self.blob_count = 0
      if self.blobs is None:
        self.blobs = []
    else:
      self.header = std_msgs.msg._Header.Header()
      self.angle_min = 0.
      self.angle_max = 0.
      self.angle_increment = 0.
      self.time_increment = 0.
      self.scan_time = 0.
      self.range_min = 0.
      self.range_max = 0.
      self.blob_count = 0
      self.blobs = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7fh.pack(_x.angle_min, _x.angle_max, _x.angle_increment, _x.time_increment, _x.scan_time, _x.range_min, _x.range_max, _x.blob_count))
      length = len(self.blobs)
      buff.write(_struct_I.pack(length))
      for val1 in self.blobs:
        _x = val1
        buff.write(_struct_2i3f.pack(_x.start, _x.end, _x.distance, _x.width, _x.angle))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 30
      (_x.angle_min, _x.angle_max, _x.angle_increment, _x.time_increment, _x.scan_time, _x.range_min, _x.range_max, _x.blob_count,) = _struct_7fh.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.blobs = []
      for i in range(0, length):
        val1 = this_robot.msg.LaserBlob()
        _x = val1
        start = end
        end += 20
        (_x.start, _x.end, _x.distance, _x.width, _x.angle,) = _struct_2i3f.unpack(str[start:end])
        self.blobs.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7fh.pack(_x.angle_min, _x.angle_max, _x.angle_increment, _x.time_increment, _x.scan_time, _x.range_min, _x.range_max, _x.blob_count))
      length = len(self.blobs)
      buff.write(_struct_I.pack(length))
      for val1 in self.blobs:
        _x = val1
        buff.write(_struct_2i3f.pack(_x.start, _x.end, _x.distance, _x.width, _x.angle))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 30
      (_x.angle_min, _x.angle_max, _x.angle_increment, _x.time_increment, _x.scan_time, _x.range_min, _x.range_max, _x.blob_count,) = _struct_7fh.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.blobs = []
      for i in range(0, length):
        val1 = this_robot.msg.LaserBlob()
        _x = val1
        start = end
        end += 20
        (_x.start, _x.end, _x.distance, _x.width, _x.angle,) = _struct_2i3f.unpack(str[start:end])
        self.blobs.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2i3f = struct.Struct("<2i3f")
_struct_3I = struct.Struct("<3I")
_struct_7fh = struct.Struct("<7fh")
