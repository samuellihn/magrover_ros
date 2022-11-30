****```mermaid
flowchart TD
	A>MagRover-PC]-."WebSocket (10Hz, protobuf)".-B[ws_receiver]
	B--"/motor_control (10Hz, geometry_msgs/Twist)"-->C[motor_control]
	D[battery_imu] --"/battery (0.1Hz, sensor_msgs/BatteryState)"-->B
	D--"/imu (10 Hz, sensor_msgs/Imu)"-->B
	
	
	
	
	
	
	
```