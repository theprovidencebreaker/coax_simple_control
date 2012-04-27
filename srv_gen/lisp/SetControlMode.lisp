; Auto-generated. Do not edit!


(cl:in-package coax_simple_control-srv)


;//! \htmlinclude SetControlMode-request.msg.html

(cl:defclass <SetControlMode-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetControlMode-request (<SetControlMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coax_simple_control-srv:<SetControlMode-request> is deprecated: use coax_simple_control-srv:SetControlMode-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coax_simple_control-srv:mode-val is deprecated.  Use coax_simple_control-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-request>) ostream)
  "Serializes a message object of type '<SetControlMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-request>) istream)
  "Deserializes a message object of type '<SetControlMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-request>)))
  "Returns string type for a service object of type '<SetControlMode-request>"
  "coax_simple_control/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-request)))
  "Returns string type for a service object of type 'SetControlMode-request"
  "coax_simple_control/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-request>)))
  "Returns md5sum for a message object of type '<SetControlMode-request>"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-request)))
  "Returns md5sum for a message object of type 'SetControlMode-request"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-request>)))
  "Returns full string definition for message of type '<SetControlMode-request>"
  (cl:format cl:nil "int8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-request)))
  "Returns full string definition for message of type 'SetControlMode-request"
  (cl:format cl:nil "int8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetControlMode-response.msg.html

(cl:defclass <SetControlMode-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetControlMode-response (<SetControlMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coax_simple_control-srv:<SetControlMode-response> is deprecated: use coax_simple_control-srv:SetControlMode-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coax_simple_control-srv:result-val is deprecated.  Use coax_simple_control-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-response>) ostream)
  "Serializes a message object of type '<SetControlMode-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-response>) istream)
  "Deserializes a message object of type '<SetControlMode-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-response>)))
  "Returns string type for a service object of type '<SetControlMode-response>"
  "coax_simple_control/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-response)))
  "Returns string type for a service object of type 'SetControlMode-response"
  "coax_simple_control/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-response>)))
  "Returns md5sum for a message object of type '<SetControlMode-response>"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-response)))
  "Returns md5sum for a message object of type 'SetControlMode-response"
  "b887f88391f1a13413ffd9a38fd72e8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-response>)))
  "Returns full string definition for message of type '<SetControlMode-response>"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-response)))
  "Returns full string definition for message of type 'SetControlMode-response"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode)))
  "Returns string type for a service object of type '<SetControlMode>"
  "coax_simple_control/SetControlMode")