;; Auto-generated. Do not edit!


(when (boundp 'button_box::ButtonInfo)
  (if (not (find-package "BUTTON_BOX"))
    (make-package "BUTTON_BOX"))
  (shadow 'ButtonInfo (find-package "BUTTON_BOX")))
(unless (find-package "BUTTON_BOX::BUTTONINFO")
  (make-package "BUTTON_BOX::BUTTONINFO"))
(unless (find-package "BUTTON_BOX::BUTTONINFOREQUEST")
  (make-package "BUTTON_BOX::BUTTONINFOREQUEST"))
(unless (find-package "BUTTON_BOX::BUTTONINFORESPONSE")
  (make-package "BUTTON_BOX::BUTTONINFORESPONSE"))

(in-package "ROS")





(defclass button_box::ButtonInfoRequest
  :super ros::object
  :slots (_button ))

(defmethod button_box::ButtonInfoRequest
  (:init
   (&key
    ((:button __button) "")
    )
   (send-super :init)
   (setq _button (string __button))
   self)
  (:button
   (&optional __button)
   (if __button (setq _button __button)) _button)
  (:serialization-length
   ()
   (+
    ;; string _button
    4 (length _button)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _button
       (write-long (length _button) s) (princ _button s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _button
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _button (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass button_box::ButtonInfoResponse
  :super ros::object
  :slots (_response ))

(defmethod button_box::ButtonInfoResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass button_box::ButtonInfo
  :super ros::object
  :slots ())

(setf (get button_box::ButtonInfo :md5sum-) "fce55d8d6e1bc1923599409813fddc68")
(setf (get button_box::ButtonInfo :datatype-) "button_box/ButtonInfo")
(setf (get button_box::ButtonInfo :request) button_box::ButtonInfoRequest)
(setf (get button_box::ButtonInfo :response) button_box::ButtonInfoResponse)

(defmethod button_box::ButtonInfoRequest
  (:response () (instance button_box::ButtonInfoResponse :init)))

(setf (get button_box::ButtonInfoRequest :md5sum-) "fce55d8d6e1bc1923599409813fddc68")
(setf (get button_box::ButtonInfoRequest :datatype-) "button_box/ButtonInfoRequest")
(setf (get button_box::ButtonInfoRequest :definition-)
      "

string button
---
string response

")

(setf (get button_box::ButtonInfoResponse :md5sum-) "fce55d8d6e1bc1923599409813fddc68")
(setf (get button_box::ButtonInfoResponse :datatype-) "button_box/ButtonInfoResponse")
(setf (get button_box::ButtonInfoResponse :definition-)
      "

string button
---
string response

")



(provide :button_box/ButtonInfo "fce55d8d6e1bc1923599409813fddc68")


