
#Camview
global media_cor, centro_cor, area_cor
media_cor, centro_cor, area_cor = cormodule.identifica_cor(cv_image)


if type(media_cor) != tuple and len(centro_cor) != 0:
    if media_cor[0] > centro_cor[0]:
        vel = Twist(Vector3(0.2, 0, 0), Vector3(0,0,-0.3))
    elif media_cor[0] < centro_cor[0]:
        vel = Twist(Vector3(0.2, 0, 0), Vector3(0,0,0.3))
    print("Girando em direção a cor")
    velocidade_saida.publish(vel)
    rospy.sleep(0.2)
