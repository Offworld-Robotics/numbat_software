; Auto-generated. Do not edit!


(cl:in-package agvc_gps-srv)


;//! \htmlinclude ConvertGPSOrigin-request.msg.html

(cl:defclass <ConvertGPSOrigin-request> (roslisp-msg-protocol:ros-message)
  ((gps_global
    :reader gps_global
    :initarg :gps_global
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry)))
)

(cl:defclass ConvertGPSOrigin-request (<ConvertGPSOrigin-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertGPSOrigin-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertGPSOrigin-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvc_gps-srv:<ConvertGPSOrigin-request> is deprecated: use agvc_gps-srv:ConvertGPSOrigin-request instead.")))

(cl:ensure-generic-function 'gps_global-val :lambda-list '(m))
(cl:defmethod gps_global-val ((m <ConvertGPSOrigin-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvc_gps-srv:gps_global-val is deprecated.  Use agvc_gps-srv:gps_global instead.")
  (gps_global m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertGPSOrigin-request>) ostream)
  "Serializes a message object of type '<ConvertGPSOrigin-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gps_global) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertGPSOrigin-request>) istream)
  "Deserializes a message object of type '<ConvertGPSOrigin-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gps_global) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertGPSOrigin-request>)))
  "Returns string type for a service object of type '<ConvertGPSOrigin-request>"
  "agvc_gps/ConvertGPSOriginRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertGPSOrigin-request)))
  "Returns string type for a service object of type 'ConvertGPSOrigin-request"
  "agvc_gps/ConvertGPSOriginRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertGPSOrigin-request>)))
  "Returns md5sum for a message object of type '<ConvertGPSOrigin-request>"
  "ecd21f5fc40d2acf04caba644024bc82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertGPSOrigin-request)))
  "Returns md5sum for a message object of type 'ConvertGPSOrigin-request"
  "ecd21f5fc40d2acf04caba644024bc82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertGPSOrigin-request>)))
  "Returns full string definition for message of type '<ConvertGPSOrigin-request>"
  (cl:format cl:nil "~%nav_msgs/Odometry gps_global~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertGPSOrigin-request)))
  "Returns full string definition for message of type 'ConvertGPSOrigin-request"
  (cl:format cl:nil "~%nav_msgs/Odometry gps_global~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertGPSOrigin-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gps_global))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertGPSOrigin-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertGPSOrigin-request
    (cl:cons ':gps_global (gps_global msg))
))
;//! \htmlinclude ConvertGPSOrigin-response.msg.html

(cl:defclass <ConvertGPSOrigin-response> (roslisp-msg-protocol:ros-message)
  ((gps_local
    :reader gps_local
    :initarg :gps_local
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry)))
)

(cl:defclass ConvertGPSOrigin-response (<ConvertGPSOrigin-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertGPSOrigin-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertGPSOrigin-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvc_gps-srv:<ConvertGPSOrigin-response> is deprecated: use agvc_gps-srv:ConvertGPSOrigin-response instead.")))

(cl:ensure-generic-function 'gps_local-val :lambda-list '(m))
(cl:defmethod gps_local-val ((m <ConvertGPSOrigin-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvc_gps-srv:gps_local-val is deprecated.  Use agvc_gps-srv:gps_local instead.")
  (gps_local m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertGPSOrigin-response>) ostream)
  "Serializes a message object of type '<ConvertGPSOrigin-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gps_local) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertGPSOrigin-response>) istream)
  "Deserializes a message object of type '<ConvertGPSOrigin-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gps_local) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertGPSOrigin-response>)))
  "Returns string type for a service object of type '<ConvertGPSOrigin-response>"
  "agvc_gps/ConvertGPSOriginResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertGPSOrigin-response)))
  "Returns string type for a service object of type 'ConvertGPSOrigin-response"
  "agvc_gps/ConvertGPSOriginResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertGPSOrigin-response>)))
  "Returns md5sum for a message object of type '<ConvertGPSOrigin-response>"
  "ecd21f5fc40d2acf04caba644024bc82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertGPSOrigin-response)))
  "Returns md5sum for a message object of type 'ConvertGPSOrigin-response"
  "ecd21f5fc40d2acf04caba644024bc82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertGPSOrigin-response>)))
  "Returns full string definition for message of type '<ConvertGPSOrigin-response>"
  (cl:format cl:nil "nav_msgs/Odometry gps_local~%~%~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertGPSOrigin-response)))
  "Returns full string definition for message of type 'ConvertGPSOrigin-response"
  (cl:format cl:nil "nav_msgs/Odometry gps_local~%~%~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertGPSOrigin-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gps_local))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertGPSOrigin-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertGPSOrigin-response
    (cl:cons ':gps_local (gps_local msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConvertGPSOrigin)))
  'ConvertGPSOrigin-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConvertGPSOrigin)))
  'ConvertGPSOrigin-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertGPSOrigin)))
  "Returns string type for a service object of type '<ConvertGPSOrigin>"
  "agvc_gps/ConvertGPSOrigin")