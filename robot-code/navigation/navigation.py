import math

class Navigate:

    def __init__(self, hardware, sensors):
        self.hardware = hardware
        self.sensors = sensors

        self.velocity = 5
        self.turn_velocity = 3

        self.exploring = False
        self.action_list = []

        #Left wheel
        self.wheel_left = self.hardware.getDevice("wheel1 motor")
        self.wheel_left.setPosition(float("inf"))

        #Right wheel
        self.wheel_right = self.hardware.getDevice("wheel2 motor")
        self.wheel_right.setPosition(float("inf"))


    def speed(self, left_speed, right_speed):
        self.wheel_left.setVelocity(left_speed)
        self.wheel_right.setVelocity(right_speed)
        return
    

    def walk_to(self, point):
        ang = math.atan2(point[1]-self.sensors.last_gps[1], point[0]-self.sensors.last_gps[0])
        delta_angle = ang-self.sensors.last_gyro

        if abs(delta_angle) >= 0.05:
            print("rotating", delta_angle)
            print(ang, self.sensors.last_gyro)

            while delta_angle < -math.pi:
                delta_angle = delta_angle + 2*math.pi
            while delta_angle > math.pi:
                delta_angle = delta_angle - 2*math.pi
            if delta_angle >= 0:
                self.speed(self.turn_velocity, -self.turn_velocity)
            else:
                self.speed(-self.turn_velocity, self.turn_velocity)
        
        else:
            print("walking", point)
            print(self.sensors.last_gps)
            print(self.dist_coords(self.sensors.last_gps, point))

            if self.dist_coords(self.sensors.last_gps, point) > 0.005:
                self.speed(self.velocity, self.velocity)
            else:
                self.action_list.pop(0)

        return


    def navigate(self):
        print("navigating")

        if len(self.action_list) == 0:
            self.exploring = False

        else:
            name = self.action_list[0][0]
            action = self.action_list[0][1]

            if name == "Walk To":
                self.walk_to(action)

        return


    def make_list(self, point):
        print("append walk_to", point)
        self.action_list.append(["Walk To", point])
        return

    
    def solve(self, point, graph):
        print("solve para", point)

        self.exploring = True

        walk_list = []
        while(self.dist_coords(self.sensors.last_gps, point[0]) > 0.05):
            walk_list.append(graph[point[1]][0])
            point = [graph[graph[point[1]][1]][0], graph[point[1]][1]] # [coords do pai, pos do pai]

        for i in range(len(walk_list)-1, -1, -1):
            self.make_list(walk_list[i])

        return
    

    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist