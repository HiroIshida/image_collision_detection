(load "package://pr2eus/pr2-interface.l")
(load "client.lsp")
(ros::roseus "test-image-collision-detection" :anonymous t)
(defun stop nil (send *ri* :stop-motion)) 

;; warning cancel
(defun warning-message (hoge hogen &rest hoenn) nil) ;; overriden
(ros::set-logger-level "ros.roseus" ros::*rosfatal*)

(setq init-angle-vector #f(179.821 51.0643 25.1347 83.4318 -82.9128 52.5284 -59.2714 -20.2535 -74.8208 -19.4982 -38.4855 -110.41 19.994 -10.9107 -9.78539 5.2327 74.2702))

(ros::roseus "col_detect_tester" :anonymous t)
(pr2-init)
(setq *robot* *pr2*)
(send *robot* :angle-vector init-angle-vector)
(send *ri* :angle-vector (send *robot* :angle-vector))
(send *ri* :wait-interpolation)



(setq *cost-absolute* 0)
(ros::subscribe "cost_absolute" std_msgs::Int64
                #'(lambda (msg) 
                    (setq *cost-absolute* (send msg :data))))



(speak-jp "はじめます")
(send *robot* :larm :move-end-pos #f(0 -300 0) :world)
(send *ri* :angle-vector (send *robot* :angle-vector) 100000)

(image-collision-detection-init)
(loop 
  (ros::spin-once)
  (print *cost-absolute*)
  (when (> *cost-absolute* 800)
    (speak-jp "あ")
    (send *ri* :stop-motion)
    (send *ri* :wait-interpolation)
    (return)))
