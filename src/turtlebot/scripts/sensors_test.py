from turtlebot import Turtlebot

turtle = Turtlebot()

#turtle.wait_for_rgb_image()
#rgb = turtle.get_rgb_image()
#print str(rgb.shape)

turtle.wait_for_odometry()
print "Odometry: ", turtle.get_odometry()
