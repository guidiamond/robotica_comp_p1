# Bumper
def bumpeiras(dado):
    global bump
    bump = dado.data
    print('Bump:',bump)
# Loop de execucao, apos verificar o scan
if bump:
    if bump == 1:
        vel = Twist(Vector3(-1.5, 0, 0), Vector3(0, 0, -1))
    elif bump == 2:
        vel = Twist(Vector3(-1.5, 0, 0), Vector3(0, 0, 1))
    elif bump == 3:
        vel = Twist(Vector3(0.4, 0, 0), Vector3(0, 0, pi/4))
    else:
        vel = Twist(Vector3(0.4, 0, 0), Vector3(0, 0, -pi/4))
    bump = 0
    velocidade_saida.publish(vel)
    rospy.sleep(3)
    continue