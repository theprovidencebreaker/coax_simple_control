; Auto-generated. Do not edit!


(cl:in-package coax_simple_control-srv)


;//! \htmlinclude SetWaypoint-request.msg.html

(cl:defclass <SetWaypoint-request> (roslisp-msg-protocol:ros-message)
  ((way_x
    :reader way_x
    :initarg :way_x
    :type cl:float
    :initform 0.0)
   (way_y
    :reader way_y
    :initarg :way_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetWaypoint-request (<SetWaypoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWaypoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWaypoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coax_simple_control-srv:<SetWaypoint-request> is deprecated: use coax_simple_control-srv:SetWaypoint-request instead.")))

(cl:ensure-generic-function 'way_x-val :lambda-list '(m))
(cl:defmethod way_x-val ((m <SetWaypoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coax_simple_control-srv:way_x-val is deprecated.  Use coax_simple_control-srv:way_x instead.")
  (way_x m))

(cl:ensure-generic-function 'way_y-val :lambda-list '(m))
(cl:defmethod way_y-val ((m <SetWaypoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coax_simple_control-srv:way_y-val is deprecated.  Use coax_simple_control-srv:way_y instead.")
  (way_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWaypoint-request>) ostream)
  "Serializes a message object of type '<SetWaypoint-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'way_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'way_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWaypoint-request>) istream)
  "Deserializes a message object of type '<SetWaypoint-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'way_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'way_y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWaypoint-request>)))
  "Returns string type for a service object of type '<SetWaypoint-request>"
  "coax_simple_control/SetWaypointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWaypoint-request)))
  "Returns string type for a service object of type 'SetWaypoint-request"
  "coax_simple_control/SetWaypointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWaypoint-request>)))
  "Returns md5sum for a message object of type '<SetWaypoint-request>"
  "fd3ab5378b9774d100960ee0ab169b03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWaypoint-request)))
  "Returns md5sum for a message object of type 'SetWaypoint-request"
  "fd3ab5378b9774d100960ee0ab169b03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWaypoint-request>)))
  "Returns full string definition for message of type '<SetWaypoint-request>"
  (cl:format cl:nil "float64 way_x~%float64 way_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWaypoint-request)))
  "Returns full string definition for message of type 'SetWaypoint-request"
  (cl:format cl:nil "float64 way_x~%float64 way_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWaypoint-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWaypoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWaypoint-request
    (cl:cons ':way_x (way_x msg))
    (cl:cons ':way_y (way_y msg))
))
;//! \htmlinclude SetWaypoint-response.msg.html

(cl:defclass <SetWaypoint-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetWaypoint-response (<SetWaypoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWaypoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWaypoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coax_simple_control-srv:<SetWaypoint-response> is deprecated: use coax_simple_control-srv:SetWaypoint-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetWaypoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coax_simple_control-srv:result-val is deprecated.  Use coax_simple_control-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWaypoint-response>) ostream)
  "Serializes a message object of type '<SetWaypoint-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWaypoint-response>) istream)
  "Deserializes a message object of type '<SetWaypoint-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWaypoint-response>)))
  "Returns string type for a service object of type '<SetWaypoint-response>"
  "coax_simple_control/SetWaypointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWaypoint-response)))
  "Returns string type for a service object of type 'SetWaypoint-response"
  "coax_simple_control/SetWaypointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWaypoint-response>)))
  "Returns md5sum for a message object of type '<SetWaypoint-response>"
  "fd3ab5378b9774d100960ee0ab169b03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWaypoint-response)))
  "Returns md5sum for a message object of type 'SetWaypoint-response"
  "fd3ab5378b9774d100960ee0ab169b03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWaypoint-response>)))
  "Returns full string definition for message of type '<SetWaypoint-response>"
  (cl:format cl:nil "int8 result~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWaypoint-response)))
  "Returns full string definition for message of type 'SetWaypoint-response"
  (cl:format cl:nil "int8 result~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWaypoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWaypoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWaypoint-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetWaypoint)))
  'SetWaypoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetWaypoint)))
  'SetWaypoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWaypoint)))
  "Returns string type for a service object of type '<SetWaypoint>"
  "coax_simple_control/SetWaypoint")