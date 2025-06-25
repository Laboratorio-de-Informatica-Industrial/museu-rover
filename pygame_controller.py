import pygame
import rospy


from geometry_msgs.msg import Twist

def leo_rover_controller():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Nenhum joystick detectado. Conecte um joystick.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick inicializado: {joystick.get_name()}")

    # Inicializa o nó ROS
    rospy.init_node('pygame_leo_rover_controller', anonymous=True)
    pub = rospy.Publisher('leo_main_duque/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    print("Controlando o Leo Rover. Pressione Ctrl+C ou feche a janela do pygame para sair.")

    try:
        while not rospy.is_shutdown():
            # Processa eventos para poder fechar o pygame corretamente
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            linear_joy = joystick.get_axis(1)  # Normalmente o eixo Y precisa ser invertido
            angular_joy = joystick.get_axis(0)

            # Debug
            print(f"Linear: {linear_joy:.2f}, Angular: {angular_joy:.2f}")


            twist_msg = Twist()


            twist_msg.linear.x = -linear_joy * 1  # ajuste o valor para o ganho desejado
            twist_msg.angular.z = -angular_joy * 1

            # Botão para parar 
            if joystick.get_button(0):
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                print("Rover Parado.")

            # Publica comando
            pub.publish(twist_msg)

            # Espera até o próximo ciclo
            rate.sleep()

    except KeyboardInterrupt:
        print("Encerrando controle...")

    # Para o robô ao sair
    stop_msg = Twist()
    pub.publish(stop_msg)
    pygame.quit()
    print("Controle finalizado.")


if __name__ == '__main__':
    try:
        leo_rover_controller()
    except rospy.ROSInterruptException:
        pass

