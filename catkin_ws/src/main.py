#!/usr/bin/env python
import cv2
import torch
from ultralytics import YOLO
import rospy
import time
from geometry_msgs.msg import Twist

model_path = '/your/weights.pt'
model = YOLO(model_path)
fps = 0  
tolerance = 0.1
fps_counter = 0 
x_deviation = 0
y_deviation = 0
center_box_x= 0
center_box_y= 0
frame_center_y= 0
frame_center_x= 0
static_acc = 0.05
static_velocity = 1
start_time = time.time() 


class MinimalPublisher:
    def __init__(self):
        rospy.init_node('minimal_publisher', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  
        self.twist_msg = Twist()
        self.cap = cv2.VideoCapture(0)
        self.acceleration = static_acc
        self.max_velocity = 20
        self.xl_acc = static_acc
        self.xr_acc = static_acc
        self.zf_acc = static_acc
        self.zb_acc = static_acc

    def move(self, linear, angular):
        self.twist_msg.linear.x = linear
        self.twist_msg.angular.z = angular
        self.publisher.publish(self.twist_msg)
        pass

    def forward(self, acceleration):
        self.move(0.0, acceleration)


    def back(self, acceleration):
        self.move(0.0, -acceleration)


    def right(self, acceleration):
        self.move(acceleration, 0.0)


    def left(self, acceleration):
        self.move(-acceleration, 0.0)


    def stop(self):
        self.move(0.0, 0.0)



    def move_robot(self, x_deviation, y_deviation,check_tolerance):

        if check_tolerance == True:
            self.stop()

        if abs(x_deviation) < tolerance and abs(y_deviation) < tolerance: 
            self.stop()
            print("stop1")
        else:
            if abs(x_deviation) > abs(y_deviation):
                if x_deviation >= tolerance:
                    self.xr_acc = static_acc
                    self.zf_acc = static_acc
                    self.zb_acc = static_acc
                    xl_velocity = static_velocity + self.xl_acc
                    if xl_velocity < self.max_velocity:
                        self.xl_acc += self.acceleration
                    self.left(xl_velocity)

                elif x_deviation <= -tolerance:
                    self.xl_acc = static_acc
                    self.zf_acc = static_acc
                    self.zb_acc = static_acc
                    xr_velocity = static_velocity + self.xr_acc
                    if xr_velocity < self.max_velocity:
                        self.xr_acc += self.acceleration
                    self.right(xr_velocity)

            else:
                if y_deviation >= tolerance:
                    self.xl_acc = static_acc
                    self.xr_acc = static_acc
                    self.zb_acc = static_acc
                    zf_velocity = static_velocity + self.zf_acc
                    if zf_velocity < self.max_velocity:
                        self.zf_acc += self.acceleration
                    self.forward(zf_velocity)

                elif y_deviation <= -tolerance:
                    self.xl_acc = static_acc
                    self.xr_acc = static_acc
                    self.zf_acc = static_acc
                    zb_velocity = static_velocity + self.zb_acc
                    if zb_velocity < self.max_velocity:
                        self.zb_acc += self.acceleration
                    self.back(zb_velocity)



    def timer_callback(self, event):
        global fps_counter, fps, start_time # Bu satır Kaldırılacak

        _, frame = self.cap.read()
        if frame is None:
            raise Exception("Failed to capture frame")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        detections = model(frame)
        for detection in detections:
            boxes = detection.boxes
            b = torch.Tensor([0, 0, 0, 0])
            for box in boxes:
                b = box.xyxy[0]
            frame, x_deviation, y_deviation, check_tolerance = self.draw_overlays(detection.plot(), b) 
            frame = cv2.resize(frame, (1280, 720))
            self.move_robot(x_deviation, y_deviation, check_tolerance)
            cv2.imshow('Frame', frame) 
            fps_counter += 1 
        
        if time.time() - start_time >= 1.0:
            fps = round(fps_counter / (time.time() - start_time), 1)
            fps_counter = 0
            start_time = time.time()
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            cv2.destroyAllWindows()

    def draw_overlays(self, cv2_im, b):


        height, width, _ = cv2_im.shape
        font=cv2.FONT_HERSHEY_SIMPLEX

        tensor_degerleri = torch.tensor(b)
        x0, y0, x1, y1 = tensor_degerleri.tolist()

        center_box_x= abs(int ((x0+x1)/2))
        center_box_y=abs(int ((y0+y1)/2))
        frame_center_y=int(height/2)
        frame_center_x=int(width/2)

        #Frame merkezindeki tolerans box kordinat hesaplaması
        tol_x_min = frame_center_x - (width * tolerance) / 2
        tol_x_max = frame_center_x + (width * tolerance) / 2
        tol_y_min = frame_center_y - (height * tolerance) / 2
        tol_y_max = frame_center_y + (height * tolerance) / 2

        x_deviation = 0
        y_deviation = 0

        check_tolerance = self.check_tolerance(x0,y0,x1,y1,tol_x_min,tol_y_min,tol_x_max,tol_y_max)

        if not center_box_x == 0 or not center_box_y == 0 :
            x_deviation=frame_center_x - center_box_x
            y_deviation=frame_center_y - center_box_y
 
        #draw black rectangle on top
        cv2_im = cv2.rectangle(cv2_im, (0,0), (width, 24), (0,0,0), -1)
    
        #draw black rectangle at bottom
        cv2_im = cv2.rectangle(cv2_im, (0,height-24), (width, height), (0,0,0), -1)

        #write deviations and tolerance 
        str_tol='Tol : {}'.format(tolerance)
        cv2_im = cv2.putText(cv2_im, str_tol, (10, height-8),font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        x_tol='x_dev : {}'.format(x_deviation)
        cv2_im = cv2.putText(cv2_im, x_tol, (130, height-8),font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y_tol='y_dev : {}'.format(y_deviation)
        cv2_im = cv2.putText(cv2_im, y_tol, (280, height-8),font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    
        #draw center cross lines
        cv2_im = cv2.rectangle(cv2_im, (0,int(height/2)-1), (width, int(height/2)+1), (255,0,0), -1)
        cv2_im = cv2.rectangle(cv2_im, (int(width/2)-1,0), (int(width/2)+1,height), (255,0,0), -1)
    
        #draw the center red dot on the object and arrowdline
        cv2_im = cv2.circle(cv2_im, (center_box_x,center_box_y), 7, (0,0,255), -1)
        if x0>0 and x1>0 and y0>0 and y1>0:
            cv2_im=cv2.arrowedLine(cv2_im, (frame_center_x,frame_center_y), (center_box_x,center_box_y), (39,237,250), 3)

        #draw the tolerance box
        cv2_im = cv2.rectangle(cv2_im, (int(tol_x_min),int(tol_y_min)), (int(tol_x_max),int(tol_y_max)), (0,255,0), 2)
        cv2_im = cv2.circle(cv2_im, (frame_center_x, frame_center_y), 7, (0,0,255), -1)
    
        #Write FPS
        cv2_im = cv2.putText(cv2_im, f"FPS: {fps}", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        return cv2_im, x_deviation, y_deviation, check_tolerance # Bu satırda sadece cv2_im değişkeni kaldırılacak
    
    # Check if the detected box is within the tolerance box
    def check_tolerance(self,x_min,y_min,x_max,y_max,tol_x_min,tol_y_min,tol_x_max,tol_y_max):
        if (tol_x_min >= x_min and tol_x_max <= x_max and 
            tol_y_min >= y_min  and tol_y_max <= y_max):
            return True
        else:
            return False

def main():
    minimal_publisher = MinimalPublisher()
    rospy.Timer(rospy.Duration(0.05), minimal_publisher.timer_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
