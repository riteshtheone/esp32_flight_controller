192.168.43.1/        -> pid panel
192.168.43.1/pid     -> get pid value
192.168.43.1/debug   -> change debug type
192.168.43.1/battery -> get battery Voltage
192.168.43.1/error   -> get error code
192.168.43.1/calibration_data -> get calibration data


192.168.43.1/takeoff        -> get manual_takeoff_throttle
192.168.43.1/takeoff?set=?  -> set manual_takeoff_throttle
(0, 1400 < ? < 1600) (now -> 0)

192.168.43.1/press          -> get (press) pressure for height
192.168.43.1/press?set=?    -> set (press) pressure for height
(22 -> 2.2 meter, 1.5 -> 15cm) (now -> 1.5)

192.168.43.1/throttle_diff          -> get throttle_diff
192.168.43.1/throttle_diff?set=?    -> set throttle_diff
default -> 1530

