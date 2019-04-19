def laser_scan(distancia):
    if distancia == None:
        return

    for idx, val in distancia:
        if 0 < val < 0.3:
            if idx <= 90:
                return 1 #direita
            elif 270 <= idx <= 360:
                return -1 #esquerda
    return


sensor_laser_scan = laser_scan(dist2)
if sensor_proximidade == None:
    # Bump/Tracking/Cor
    pass
elif sensor_proximidade == -1:
    print("Girando Esquerda")
    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0.6))
    velocidade_saida.publish(vel)
    rospy.sleep(1)
elif sensor_proximidade == 1:
    print("Girando Direita")
    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,-0.6))
    velocidade_saida.publish(vel)
    rospy.sleep(1)
