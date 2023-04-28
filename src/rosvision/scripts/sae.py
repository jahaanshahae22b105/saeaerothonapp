import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

 

lower = {'red':([166, 84, 141]), 'green':([50, 50, 120]), 'blue':([97, 100, 117]),'yellow':([23, 59, 119]), 'orange':([0, 50, 80]), 'purple':([130, 80, 80])} 
upper = {'red':([186,255,255]), 'green':([70, 255, 255]), 'blue':([117,255,255]), 'yellow':([54,255,255]), 'orange':([20,255,255]), 'purple':([150, 255, 255])}


colors = {'red':(0,0,255), 'green':(0,255,0), 'blue':(255,0,0), 'yellow':(0, 255, 217), 'orange':(0,140,255), 'purple':(211,0,148)}

vid = cv2.VideoCapture(2)
globalobjectlist = []
cvbridge = CvBridge()


def publisher():

    pub = rospy.Publisher("/livedetect", Image, queue_size=1)
    rospy.init_node("livepublishernode")
    rospy.loginfo("Publisher Started")



    while not rospy.is_shutdown():
        ret, frame = vid.read()
        if not ret:
            break
        
        blurred = cv2.GaussianBlur(frame, (19, 19), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        masklist=[]
        contourlist=[]
        liveclrlist=[]

        for (key, value) in upper.items():
            kernel = np.ones((2,2),np.uint8)
            mask = cv2.inRange(hsv, np.array(lower[key]), np.array(upper[key]))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.dilate(mask, kernel, iterations=1)
            masklist.append(mask)
            cnts,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(cnts)>=1:
                contourlist.append(cnts[-1])
                liveclrlist.append(key)
            
        
        for i,cnt in enumerate(contourlist):
            approx = cv2.approxPolyDP(cnt, 0.014*cv2.arcLength(cnt, True), True)
            cv2.drawContours(frame, [approx], 0, (0), 2)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
        
            if len(approx) == 3:
                text = liveclrlist[i] + " triangle"
                cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                if text in globalobjectlist:
                    pass
                else:
                    globalobjectlist.append(text)

                
            elif len(approx) == 4:
                x2 = approx.ravel()[2]
                y2 = approx.ravel()[3]
                x4 = approx.ravel()[6]
                y4 = approx.ravel()[7]
                side1 = abs(x2-x)
                side2 = abs(y4-y)
                
                if abs(side1-side2) <= 2:
                    text = liveclrlist[i] + " square"
                    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                    if text in globalobjectlist:
                        pass
                    else:
                        globalobjectlist.append(text)
                else:
                    text = liveclrlist[i] + " quadrilateral"
                    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                    if text in globalobjectlist:
                        pass
                    else:
                        globalobjectlist.append(text)
                    
            elif len(approx) == 5:
                text = liveclrlist[i] + " pentagon"
                cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                if text in globalobjectlist:
                    pass
                else:
                    globalobjectlist.append(text)

            elif len(approx) == 6:
                text = liveclrlist[i] + " hexagon"
                cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                if text in globalobjectlist:
                    pass
                else:
                    globalobjectlist.append(text)
            
            elif len(approx) == 7:
                text = liveclrlist[i] + " heptagon"
                cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                if text in globalobjectlist:
                    pass
                else:
                    globalobjectlist.append(text)

            elif len(approx) == 8:
                text = liveclrlist[i] + " octagon"
                cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                if text in globalobjectlist:
                    pass
                else:
                    globalobjectlist.append(text)

            
            else:
                text = liveclrlist[i] + " polygon/circle"
                cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[liveclrlist[i]],2)
                if text in globalobjectlist:
                    pass
                else:
                    globalobjectlist.append(text)
        

        # Roouting OpenCV imageframe to ROS using cv bridge
        msg = cvbridge.cv2_to_imgmsg(frame, 'bgr8')

        #publishing the image on the topic '/livedetect'
        pub.publish(msg)
        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if rospy.is_shutdown():
            vid.release()

    rospy.loginfo(globalobjectlist)
    






if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
