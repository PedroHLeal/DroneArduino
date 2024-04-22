## ARDUINO FLIGHT CONTROLLER

![dronatellogif](https://github.com/PedroHLeal/DroneArduino/assets/5844621/92295875-5e58-4f1e-9142-fa274aa36034)

This is a POC (as if this wasn't proved a thousand times already; let's call it proof of myself then) to make a usable FC on arduino.

This project only uses a MPU6050 gyro to stabilize itself, which makes the thing very hard to control, specially it's height (which is throttle based only).
I've also attached a bluetooth module to control it from cellphone.

Next steps: 
- I'm getting rid of the Arduino as a FC and replace it for a proper BETAFLIGHT one, this should improve the awful wiring a lot and i'll be attaching some proximity and other sensors to the arduino, so i can make some cool stabilization features to the drone (like height and position)
- Also, in a further future, i'll maybe replace the arduino for a PI and a camera, and experiment with some CV to try to create some neat navigational feats to the drone.
