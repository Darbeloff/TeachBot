; Auto-generated. Do not edit!


(cl:in-package button_box-srv)


;//! \htmlinclude ButtonInfo-request.msg.html

(cl:defclass <ButtonInfo-request> (roslisp-msg-protocol:ros-message)
  ((button
    :reader button
    :initarg :button
    :type cl:string
    :initform ""))
)

(cl:defclass ButtonInfo-request (<ButtonInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ButtonInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ButtonInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name button_box-srv:<ButtonInfo-request> is deprecated: use button_box-srv:ButtonInfo-request instead.")))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <ButtonInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader button_box-srv:button-val is deprecated.  Use button_box-srv:button instead.")
  (button m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ButtonInfo-request>) ostream)
  "Serializes a message object of type '<ButtonInfo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'button))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'button))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ButtonInfo-request>) istream)
  "Deserializes a message object of type '<ButtonInfo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'button) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'button) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ButtonInfo-request>)))
  "Returns string type for a service object of type '<ButtonInfo-request>"
  "button_box/ButtonInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ButtonInfo-request)))
  "Returns string type for a service object of type 'ButtonInfo-request"
  "button_box/ButtonInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ButtonInfo-request>)))
  "Returns md5sum for a message object of type '<ButtonInfo-request>"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ButtonInfo-request)))
  "Returns md5sum for a message object of type 'ButtonInfo-request"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ButtonInfo-request>)))
  "Returns full string definition for message of type '<ButtonInfo-request>"
  (cl:format cl:nil "~%~%string button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ButtonInfo-request)))
  "Returns full string definition for message of type 'ButtonInfo-request"
  (cl:format cl:nil "~%~%string button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ButtonInfo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'button))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ButtonInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ButtonInfo-request
    (cl:cons ':button (button msg))
))
;//! \htmlinclude ButtonInfo-response.msg.html

(cl:defclass <ButtonInfo-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass ButtonInfo-response (<ButtonInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ButtonInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ButtonInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name button_box-srv:<ButtonInfo-response> is deprecated: use button_box-srv:ButtonInfo-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <ButtonInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader button_box-srv:response-val is deprecated.  Use button_box-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ButtonInfo-response>) ostream)
  "Serializes a message object of type '<ButtonInfo-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ButtonInfo-response>) istream)
  "Deserializes a message object of type '<ButtonInfo-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ButtonInfo-response>)))
  "Returns string type for a service object of type '<ButtonInfo-response>"
  "button_box/ButtonInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ButtonInfo-response)))
  "Returns string type for a service object of type 'ButtonInfo-response"
  "button_box/ButtonInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ButtonInfo-response>)))
  "Returns md5sum for a message object of type '<ButtonInfo-response>"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ButtonInfo-response)))
  "Returns md5sum for a message object of type 'ButtonInfo-response"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ButtonInfo-response>)))
  "Returns full string definition for message of type '<ButtonInfo-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ButtonInfo-response)))
  "Returns full string definition for message of type 'ButtonInfo-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ButtonInfo-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ButtonInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ButtonInfo-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ButtonInfo)))
  'ButtonInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ButtonInfo)))
  'ButtonInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ButtonInfo)))
  "Returns string type for a service object of type '<ButtonInfo>"
  "button_box/ButtonInfo")