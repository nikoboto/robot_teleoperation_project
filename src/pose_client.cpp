// Bibliotecas necesarias
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath> // para M_PI
#include <std_msgs/Float64MultiArray.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/PoseVelocity.h>   // NUEVO: para comandos de velocidad
#include <kinova_msgs/HomeArm.h>


geometry_msgs::PoseStamped current_pose;
bool pose_received = false;
ros::Publisher vel_pub;

// Cliente de acción global
actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction>* poseClientPtr;

// Al inicio, se envía al robot a Home
// Nombre del servicio: /j2n6s300_driver/in/home_arm
// Clase kinova_msgs::HomeArm : Generada por ROS a partir de la definición de la interfaz del servicio (HomeArm.srv)
bool ServicioHome(ros::NodeHandle& nh){ 
    ros::ServiceClient homeClient = nh.serviceClient<kinova_msgs::HomeArm>("/j2n6s300_driver/in/home_arm");
    kinova_msgs::HomeArm srv; 

    if (homeClient.call(srv)) {
        // Llamada al servicio exitosa, a nivel de comunicación
        ROS_INFO("Brazo enviado a posicion home.");
        return true;
    } else {
        ROS_ERROR("No se pudo llamar al servicio home_arm.");
        return false;
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;       // guarda la pose recibida
    pose_received = true;      // marca que ya tienes datos
}

// Variables globales para la lógica de prioridad
int current_priority_id = -1; // -1: nadie tiene el control, 0: default, 1: Luis (prioridad), 2: Joystick
ros::Time last_command_time;
const double COMMAND_TIMEOUT = 2.0; // Segundos antes de liberar el control si no llegan comandos

// Se reciben comandos del nodo secundario 'comandos_node.cpp'
void comandosCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Verificar tamaño del mensaje
    int msg_size = msg->data.size();
    int source_id = 0; // Default ID
    if (msg_size >= 6) {
        source_id = (int)msg->data[5];
    }

    // Lógica de Prioridad
    // Si ha pasado mucho tiempo desde el último comando, reseteamos la prioridad
    if ((ros::Time::now() - last_command_time).toSec() > COMMAND_TIMEOUT) {
        current_priority_id = -1;
    }

    // Definir ID de Luis (Prioridad Alta)
    const int LUIS_ID = 1;

    // Reglas:
    // 1. Si el comando viene de Luis, siempre se acepta y toma el control.
    // 2. Si el control lo tiene Luis, ignorar a otros.
    // 3. Si nadie tiene el control, cualquiera puede tomarlo.
    // 4. Si el control lo tiene otro (ej. Joystick), Luis puede interrumpir (Regla 1), pero otros no.
    
    if (source_id == LUIS_ID) {
        current_priority_id = source_id;
        ROS_INFO_THROTTLE(1, "Control tomado por LUIS (ID: %d)", source_id);
    } else if (current_priority_id == LUIS_ID) {
        ROS_WARN_THROTTLE(1, "Comando de ID %d ignorado. LUIS tiene prioridad.", source_id);
        return; // Ignorar comando
    } else {
        // Si no es Luis, y nadie tiene prioridad o es el mismo que ya la tiene
        current_priority_id = source_id;
    }

    last_command_time = ros::Time::now();

    int opcion = (int)msg->data[0];
    double var1 = msg->data[1];
    double var2 = msg->data[2];
    double var3 = msg->data[3];
    double tiempo = msg->data[4];

    kinova_msgs::ArmPoseGoal goal;

    if (pose_received)
    {
        if (opcion == 1 || opcion == 2)
        {
            if (opcion == 1)
            {
                goal.pose.header.stamp = ros::Time::now();
                goal.pose.header.frame_id = "j2n6s300_link_base";
                goal.pose.pose.position.x = current_pose.pose.position.x + var1; //var1 = xVar
                goal.pose.pose.position.y = current_pose.pose.position.y + var2; //var2 = yVar
                goal.pose.pose.position.z = current_pose.pose.position.z + var3; //var3 = zVar
                goal.pose.pose.orientation = current_pose.pose.orientation;
            }
            else
            {
                // Convertir a tf::Quaternion
                tf::Quaternion q(current_pose.pose.orientation.x, 
                                 current_pose.pose.orientation.y,
                                 current_pose.pose.orientation.z,
                                 current_pose.pose.orientation.w);

                // Extraer RPY
                double roll, pitch, yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                // Nuevo RPY en rad, te mueves respecto a lo actual
                roll  += (var1)*(M_PI/180);
                pitch += (var2)*(M_PI/180);
                yaw   += (var3)*(M_PI/180);

                // Crear cuaternión, contiene los 4 campos
                geometry_msgs::Quaternion qu = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

                goal.pose.header.stamp = ros::Time::now();
                goal.pose.header.frame_id = "j2n6s300_link_base";
                goal.pose.pose.position = current_pose.pose.position;
                goal.pose.pose.orientation = qu;
            }

            // Envías lo deseado (goal)
            poseClientPtr->sendGoal(goal);
            poseClientPtr->waitForResult();

            if (poseClientPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("Pose alcanzada!");}
            else{
                ROS_WARN("Movimiento fallido");}
        }
        else
        {
            if (opcion == 3)
            {
                // Control por velocidad lineal
                // vel.lin.[m/s]+tiempo[s]
                
                // Pequeña saturacion de seguridad (pueden ajustar estos limites)
				const double MAX_VEL_LIN = 0.5;  // 0.2 m/s
				if (std::abs(var1) > MAX_VEL_LIN) var1 = (var1 > 0 ? MAX_VEL_LIN : -MAX_VEL_LIN);
				if (std::abs(var2) > MAX_VEL_LIN) var2 = (var2 > 0 ? MAX_VEL_LIN : -MAX_VEL_LIN);
				if (std::abs(var3) > MAX_VEL_LIN) var3 = (var3 > 0 ? MAX_VEL_LIN : -MAX_VEL_LIN);

				kinova_msgs::PoseVelocity vel_msg;
				vel_msg.twist_linear_x  = var1;
				vel_msg.twist_linear_y  = var2;
				vel_msg.twist_linear_z  = var3;
				vel_msg.twist_angular_x = 0.0;
				vel_msg.twist_angular_y = 0.0;
				vel_msg.twist_angular_z = 0.0;

                // Publicar a una frecuencia fija mientras dure "tiempo"
				// Ejemplo: 100 Hz = 0.01 s entre mensajes
				double freq = 100.0;
				ros::Rate r(freq);
				int steps = static_cast<int>(tiempo * freq);

				ROS_INFO("Aplicando velocidad lineal (%.3f, %.3f, %.3f) m/s durante %.2f s",
						 var1, var2, var3, tiempo);

				for (int i = 0; i < steps && ros::ok(); ++i)
				{
					vel_pub.publish(vel_msg);
					//ros::spinOnce();    // por si llega nueva pose, etc.
					r.sleep();
				}

				// Al final, mandar velocidad cero por seguridad
				vel_msg.twist_linear_x = 0.0;
				vel_msg.twist_linear_y = 0.0;
				vel_msg.twist_linear_z = 0.0;
				vel_pub.publish(vel_msg);

				ROS_INFO("Movimiento por velocidad lineal terminado.");
            }
            else
            {
                if (opcion == 4)
                {
                    // Control por velocidad angular
                    // vel.ang.[°/s] + tiempo[s]
                    const double MAX_VEL_ANG = 90;  // °/s (ajustar segun pruebas)
                    if (std::abs(var1) > MAX_VEL_ANG) var1 = (var1 > 0 ? MAX_VEL_ANG : -MAX_VEL_ANG);
                    if (std::abs(var2) > MAX_VEL_ANG) var2 = (var2 > 0 ? MAX_VEL_ANG : -MAX_VEL_ANG);
                    if (std::abs(var3) > MAX_VEL_ANG) var3 = (var3 > 0 ? MAX_VEL_ANG : -MAX_VEL_ANG);

                    // Conversión a radianes
                    var1 = (var1)*(M_PI/180);
                    var2 = (var2)*(M_PI/180);
                    var3 = (var3)*(M_PI/180);

                    kinova_msgs::PoseVelocity vel_msg;
                    vel_msg.twist_linear_x  = 0.0;
                    vel_msg.twist_linear_y  = 0.0;
                    vel_msg.twist_linear_z  = 0.0;
                    vel_msg.twist_angular_x = var2;
                    vel_msg.twist_angular_y = var3;
                    vel_msg.twist_angular_z = var1; // roll

                    double freq = 100.0;
                    ros::Rate r(freq);
                    int steps = static_cast<int>(tiempo * freq);

                    ROS_INFO("Aplicando velocidad angular (%.3f, %.3f, %.3f) rad/s durante %.2f s",
                            var1, var2, var3, tiempo);

                    for (int i = 0; i < steps && ros::ok(); ++i)
                    {
                        vel_pub.publish(vel_msg);
                        //ros::spinOnce();
                        r.sleep();
                    }

                    // Detener rotacion
                    vel_msg.twist_angular_x = 0.0;
                    vel_msg.twist_angular_y = 0.0;
                    vel_msg.twist_angular_z = 0.0;
                    vel_pub.publish(vel_msg);

                    ROS_INFO("Movimiento por velocidad angular terminado.");
                }
                else
                {
                    if (opcion == 0)
                    {
                        // Se desea salir del programa
                        std::cout << "Saliendo del programa ...\n";
                        ros::shutdown();
                        return;
                    }
                    else
                    {
                        // Opción no válida
                        std::cout << "Opcion no valida.\n";
                    }
                }
            }
        }
        
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_client");
    ros::NodeHandle nh;

    // Cliente de acción inicializado una sola vez
    actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> poseClient("/j2n6s300_driver/pose_action/tool_pose", true);
    poseClient.waitForServer();
    poseClientPtr = &poseClient;
    
    // Evaluar si la solicitud del servicio 'home' fue exitosa
    if (!ServicioHome(nh)) {
        ROS_WARN("Continuando sin haber alcanzado home...");
    }

    // Suscripcion a ese tópico para recibir la pose del efector final desde el Kinova
    ros::Subscriber pose_sub = nh.subscribe("/j2n6s300_driver/out/tool_pose", 1, poseCallback);
    // Publicador de velocidad cartesiana hacia el driver Kinova
    vel_pub = nh.advertise<kinova_msgs::PoseVelocity>("/j2n6s300_driver/in/cartesian_velocity", 1);
    // Recibir comandos del nodo secundario
    ros::Subscriber comandos_sub = nh.subscribe("/comandos", 10, comandosCallback);

    ros::spin();
    return 0;
}