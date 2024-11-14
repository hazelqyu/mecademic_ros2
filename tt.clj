(require '[babashka.process :refer [shell]])
;(shell "ros2" "launch" "launch/camera_launch.py")
(def robot "192.168.0.100")

(println "Hi" robot)
(prn (slurp "camera_feed.py"))




