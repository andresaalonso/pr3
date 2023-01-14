#!/usr/bin/python3
import rospy
import actionlib
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback, RecoveryStatus
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import atan2, sqrt, cos, sin
import random
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge


class ClienteMoveBase:
    def __init__(self, rate, puntos):
        # Conexion al servidor move_base
        self.escapando = False
        self.puntos = puntos
        self.rate = rate
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.position = None
        self.orientation = None
        self.status = None
        self.goal = None
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_image, queue_size=1)
        cv.namedWindow("Image Window", 1)
        self.img = None
        self.anterior = False

    def callback_feedback(self, msg):
        self.position = msg.feedback.base_position.pose.position
        self.orientation = msg.feedback.base_position.pose.orientation
        self.status = msg.status.status
        dist = sqrt((self.goal.target_pose.pose.position.x-self.position.x)**2+(self.goal.target_pose.pose.position.y-self.position.y)**2)
        if dist<0.6:
            self.client.cancel_all_goals()
    
    def callback_recovery(self, msg):
        # Si hay un recovery, cambiamos el goal (cancelando el actual)
        print("Cambio por recovery")
        self.client.cancel_all_goals()

    def show_image(self, img):
        cv.imshow("Image Window", img)
        cv.waitKey(3)


    def callback_image(self, msg):
        ### IMPORTANTE COMPROBAR QUE LA CAMARA VEA LOS COLORES CORRECTOS
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print("Llamada")
        except e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
        # resize
        width = int(self.img.shape[1] * 0.5)
        height = int(self.img.shape[0] * 0.5)
        dim = (width, height)
        self.img = cv.resize(self.img, dim, interpolation = cv.INTER_AREA)

        hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)

        # rojo
        minRedRange = np.array([0, 100, 20], np.uint8)
        maxRedRange = np.array([5, 255, 255], np.uint8)

        # Creamos las mascaras para los dos jugadores
        self.img = cv.inRange(hsv, minRedRange, maxRedRange)

        kernel = np.ones((3, 3), np.uint8)
        # Eliminamos el ruido de las mascaras
        self.img = cv.morphologyEx(self.img, cv.MORPH_OPEN, kernel)
        self.img = cv.morphologyEx(self.img, cv.MORPH_CLOSE, kernel)



        # Obtenemos los contornos del jugador 2 (rojo)
        contoursP2, _ = cv.findContours(self.img, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

        if len(contoursP2)==0 and self.anterior:
            self.anterior = False


        if len(contoursP2) > 0 and not self.escapando and self.position!=None and not self.anterior:
            self.escapando = True
            self.anterior = True
            self.img = np.zeros(self.img.shape)
            print ("Hay rojo")
            euler = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
            vector = (100*cos(euler[2]+3.14)+self.position.x, 100*sin(euler[2]+3.14)+self.position.y)
            print("Posicion:",self.position.x, self.position.y)
            print("Vector:",vector)
            minimo = 5000
            punto_de_escape = (0, 0)
            punto_mas_cercano_robot = (0, 0)
            # Punto mas cercano al robot
            for punto in self.puntos:
                temp = sqrt((punto[0]-self.position.x)**2+(punto[1]-self.position.y)**2)
                if temp<minimo:
                    minimo = temp
                    punto_mas_cercano_robot = punto
            minimo=5000
            # Punto mas cercano al vector
            for punto in self.puntos:
                temp = sqrt((punto[0]-vector[0])**2+(punto[1]-vector[1])**2)
                # temp = sqrt((punto[0]-self.position.x)**2+(punto[1]-self.position.y)**2)
                if temp<minimo and punto!=punto_mas_cercano_robot:
                    minimo = temp
                    punto_de_escape = punto
            self.client.cancel_all_goals()
            print("Punto de escape:", punto_de_escape)
            resultado = self.moveTo(punto_de_escape[0], punto_de_escape[1], 1.57)
            print("Escape finalizado")
            self.escapando = False
            
        self.show_image(self.img)





    def moveTo(self, x_, y_, z_):
        self.goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = x_
        self.goal.target_pose.pose.position.y = y_
        temp = quaternion_from_euler(0, 0, z_)
        self.goal.target_pose.pose.orientation.z = temp[2]
        self.goal.target_pose.pose.orientation.w = temp[3]

        #enviamos el goal 
        self.client.send_goal(self.goal)


        feedback_subscriber = rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, self.callback_feedback)
        recovery_subscriber = rospy.Subscriber("move_base/recovery_status", RecoveryStatus, self.callback_recovery)



        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while (state==GoalStatus.ACTIVE or state==GoalStatus.PENDING):
            # sqrt((position.x - goal.target_pose.pose.position.x)**2+(position.y - goal.target_pose.pose.position.y)**2)>0.5:
            state = self.client.get_state()
            # print(state)
            self.rate.sleep()

        return self.client.get_result()









if __name__ == "__main__":

    #### TODA ESTA PRIMERA PARTE COMENTADA
    #### AUTOMATIZA EL PROCESO DE CREAR LOS PUNTOS
    #### EN EL MAPA, ESTA COMENTADA PORQUE ES
    #### MAS EFECTIVO ANHADIRLOS MANUALMENTE

    # img = cv.imread("/home/andres/catkin_ws/src/kkk/src/map.pgm", cv.IMREAD_GRAYSCALE)

    # img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
    # img = cv.resize(img, (200, 200), interpolation = cv.INTER_AREA)
    # img = cv.blur(img,(6,6))

    # src = np.full(img.shape, 254.0)

    # for i in range(len(img)):
    #     for j in range(len(img[i])):
    #         if img[i][j]<230:
    #             src[i][j]=0.0

    # cv.imwrite("salida.jpg", src)

    # puntos = []

    # for i in range(len(src)):
    #     for j in range(len(src[i])):
    #         if src[i][j]>150.0:
    #             puntos.append((j/10-10, i/10-10))


    puntos = [
        (0.56, 0.48),
        (0.47, -0.65),
        (-0.53, 0.54),
        (-0.53, -0.56),
        (0.47, 1.83),
        (0.47, -1.94),
        (1.9, 0.56),
        (1.80, -0.64),
        (-0.56, 1.94),
        (-0.56, -1.87),
        (-1.8, 0.5),
        (-1.8, -0.63),
    ]

    #### YA TENEMOS TODOS LOS PUNTOS A LOS QUE SE PUEDE
    #### MOVER EL TURTLEBOT EN EL MAPA

    rospy.init_node('prueba_clientemovebase')
    rate = rospy.Rate(10)
    cliente = ClienteMoveBase(rate, puntos)

    
    # while position==None and status==None:
    #     rate.sleep()
    #     print("Waiting for feedback")


    result = True
    while True:
        pos = puntos[random.randint(0, len(puntos)-1)]
        print(pos)
        z = 1.57
        result = cliente.moveTo(pos[0], pos[1], z)
        print("Result main:",result)
        # if result:
        #     rospy.loginfo("Goal conseguido!")
        rospy.loginfo("Goal conseguido!")