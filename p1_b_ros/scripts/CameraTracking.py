def CamView(imagem):
    global cv_image
    global media
    global centro
    
    global media_cor
    global centro_cor
    global area_cor

    global viu_obj
    global is_centered
    global coef
    global objeto
    global x_obj
    global dist_obj_centro_imagem_x
    global contador_objeto
    global detect_mode
    global initBB
    global box

    # Lag detection
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.nsecs
    if print_delay:
        print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay:
        print("Descartando por causa do delay do frame:", delay)
        return None

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        media_cor, centro_cor, area_cor = cormodule.identifica_cor(cv_image)

        if detect_mode:
            centro, imagem, resultados = visao_module.processa(cv_image)

            viu_obj = False
            resultados = [i for i in resultados if i[0]==objeto]
            # Procura pelo objeto
            for r in resultados:
                # if r[0] == objeto:
                viu_obj = True
                # Dimensoes do objeto
                x0_obj, y0_obj = r[2]
                x1_obj, y1_obj = r[3]
                posicao_obj = (int((x0_obj+x1_obj)/2.0), int((y0_obj+y1_obj)/2.0))
                # Dimensoes da imagem
                x_obj, y_obj = posicao_obj
                centro_imagem_x = res_x / 2.0
                dist_obj_centro_imagem_x = x_obj - centro_imagem_x

            if viu_obj:
                contador_objeto += 1
                if contador_objeto >= 5:
                    detect_mode = False
                    contador_objeto = 0
                    (x1, y1), (x2, y2) = resultados[0][2:]
                    initBB = (x1, y1, x2, y2)
                    tracker.init(cv_image, initBB)
                    print("\n\nTracking\n\n")
            else:
                contador_objeto = 0

        if not detect_mode:
            # TRACKER AQUI
            if initBB is not None: # TODO
                # grab the new bounding box coordinates of the object
                (success, box_) = tracker.update(cv_image)

                if success: # check to see if the tracking was a success
                    viu_obj = True
                    # print("B", box_)
                    box = [int(v) for v in box_]
                    (x, y, w, h) = box
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 255, 0), 2)

                else:
                    detect_mode = True
            else:
                detect_mode = True

        cv2.imshow("Camera", cv_image)

    except CvBridgeError as e:
        print('ex', e)



# Tracking
if viu_obj and not detect_mode and box:
    print("Tracking objeto")
    x_obj = box[0]
    x_obj_width = box[2]
    media_x = x_obj + x_obj_width / 2
    centrox_x = centro[0]
    if media_x - centro_x > 20:
        vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,-0.6))
    elif media_x - centro_x < 20:
        vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0.6))
    velocidade_saida.publish(vel)
    rospy.sleep(0.1)
    continue

if not viu_obj: # and not detect_mode:
    # Resetar
    detect_mode = False
    contador_objeto = 0

