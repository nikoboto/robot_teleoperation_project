#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "comandos_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/comandos", 10);

    // Default source_id is 0
    int source_id = 0;
    if (argc > 1) {
        source_id = atoi(argv[1]);
    }
    std::cout << "Node started with Source ID: " << source_id << std::endl;

    int opcion;
    double var1, var2, var3, tiempo;

    while(ros::ok())
    {
        std::cout << "Escriba el número de opción según el modo de control:" << std::endl;
        std::cout << "Opcion 1: Control de posición x y z" << std::endl;
        std::cout << "Opcion 2: Control de orientaciones roll pitch yaw" << std::endl;
        std::cout << "Opcion 3: Control de velocidades lineales x y z, & tiempo" << std::endl;
        std::cout << "Opcion 4: Control de velocidades angulares roll pitch yaw, & tiempo" << std::endl;
        std::cin >> opcion;

        if (opcion == 1)
        {
            std::cout << "Ingrese los incrementos de posición en x y z:" << std::endl;
            std::cout << "Ingrese como cuarto parámetro 0" << std::endl;
        }
        else
        {
            if (opcion == 2)
            {
                std::cout << "Ingrese los incrementos de orientacion en roll pitch yaw." << std::endl;
                std::cout << "Ingrese como cuarto parámetro 0" << std::endl;
            }
            else
            {
                if (opcion == 3)
                {
                    std::cout << "Ingrese las velocidades lineales vx vy vz & el tiempo" << std::endl;
                }
                else
                {
                    std::cout << "Ingrese las velocidades angulares roll pitch yaw & el tiempo" << std::endl;
                }
            }
        }

        std::cin >> var1 >> var2 >> var3 >> tiempo;

        //Creando mensaje
        std_msgs::Float64MultiArray msg;
        msg.data.resize(6); // Resized to 6 to include source_id
        msg.data[0] = opcion;  // opción como número entero
        msg.data[1] = var1;
        msg.data[2] = var2;
        msg.data[3] = var3;
        msg.data[4] = tiempo;
        msg.data[5] = (double)source_id; // Added source_id

        //Publicando
        pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}