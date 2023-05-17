import rclpy

from . import run_lid
import time
#from . import functions

BLUE_CAR_NAME = "vehicle_blue"

def main():
    rclpy.init()

    #blue_car = functions.MinimalPublisher(BLUE_CAR_NAME)
    blue_car = run_lid.MinimalPublisher(BLUE_CAR_NAME)

    rclpy.spin(blue_car)

    blue_car.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
