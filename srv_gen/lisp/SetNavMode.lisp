; Auto-generated. Do not edit!


(cl:in-package coax_simple_control-srv)


;//! \htmlinclude SetNavMode-request.msg.html

(cl:defclass <SetNavMode-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetNavMode-request (<SetNavMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetNavMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetNavMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coax_simple_control-srv:<SetNavMode-request> is deprecated: use coax_simple_control-srv:SetNavMode-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetNavMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coax_simple_control-srv:mode-val is deprecated.  Use coax_simple_control-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetNavMode-request>) ostream)
  "Serializes a message object of type '<SetNavMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetNavMode-request>) istream)
  "Deserializes a message object of type '<SetNavMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetNavMode-request>)))
  "Returns string type for a service object of type '<SetNavMode-request>"
  "coax_simple_control/SetNavModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetNavMode-request)))
  "Returns string type for a service object of type 'SetNavMode-request"
  "coax_simple_control/SetNavModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetNavMode-request>)))
  "Returns md5sum for a message object of type '<SetNavMode-request>"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetNavMode-request)))
  "Returns md5sum for a message object of type 'SetNavMode-request"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetNavMode-request>)))
  "Returns full string definition for message of type '<SetNavMode-request>"
  (cl:format cl:nil "int8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetNavMode-request)))
  "Returns full string definition for message of type 'SetNavMode-request"
  (cl:format cl:nil "int8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetNavMode-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetNavMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetNavMode-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetNavMode-response.msg.html

(cl:defclass <SetNavMode-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetNavMode-response (<SetNavMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetNavMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetNavMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coax_simple_control-srv:<SetNavMode-response> is deprecated: use coax_simple_control-srv:SetNavMode-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetNavMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coax_simple_control-srv:result-val is deprecated.  Use coax_simple_control-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetNavMode-response>) ostream)
  "Serializes a message object of type '<SetNavMode-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetNavMode-response>) istream)
  "Deserializes a message object of type '<SetNavMode-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetNavMode-response>)))
  "Returns string type for a service object of type '<SetNavMode-response>"
  "coax_simple_control/SetNavModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetNavMode-response)))
  "Returns string type for a service object of type 'SetNavMode-response"
  "coax_simple_control/SetNavModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetNavMode-response>)))
  "Returns md5sum for a message object of type '<SetNavMode-response>"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetNavMode-response)))
  "Returns md5sum for a message object of type 'SetNavMode-response"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetNavMode-response>)))
  "Returns full string definition for message of type '<SetNavMode-response>"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetNavMode-response)))
  "Returns full string definition for message of type 'SetNavMode-response"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetNavMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetNavMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetNavMode-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetNavMode)))
  'SetNavMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetNavMode)))
  'SetNavMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetNavMode)))
  "Returns string type for a service object of type '<SetNavMode>"
  "coax_simple_control/SetNavMode")