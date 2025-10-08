(require "package://kxreus/euslisp/kxr-interface.l")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "hand_control")
(kxr-init :create-viewer nil)
(send *ri* :servo-on)

(defun cb (msg)
  (let ((av (send *ri* :state :potentio-vector))
	(msg-joint-list (mapcar #'(lambda (name position) (cons name position))
				(send msg :name) (coerce (send msg :position) cons)))
	(robot-joint-list (mapcar #'(lambda (jnt) (send jnt :name)) (send *robot* :joint-list))))
    (dotimes (i (length robot-joint-list))
      (let ((jnt (assoc (elt robot-joint-list i) msg-joint-list :test #'string=)))
	(when jnt
	  (setf (elt av i) (rad2deg (cdr jnt)))
	  )))
    (print av)
    (send *ri* :angle-vector av 5000)
    ;; (send *ri* :wait-interpolation)
    )
  )

(ros::subscribe "/hand_joint_states" sensor_msgs::JointState #'cb)

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep)
 )
