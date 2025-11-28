#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

class JoyCommander
{
public:
    JoyCommander()
    {
        // Initialize publisher and subscriber
        pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/comandos", 10);
        sub_ = nh_.subscribe("/joy", 10, &JoyCommander::joyCallback, this);

        // Parameters
        nh_.param("source_id", source_id_, 2); // Default ID for Joystick is 2
        nh_.param("linear_scale", linear_scale_, 0.2);
        nh_.param("angular_scale", angular_scale_, 30.0); // degrees/s

        ROS_INFO("JoyCommander started with Source ID: %d", source_id_);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        // Logitech F310 Mapping (XInput Mode)
        // Axes: 0=L_Left/Right, 1=L_Up/Down, 2=LT, 3=R_Left/Right, 4=R_Up/Down, 5=RT
        // Buttons: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=Back, 7=Start, 8=Logitech, 9=LStick, 10=RStick

        // Deadman switch: LB (Button 4) must be pressed
        if (!joy->buttons[4]) {
            return; // Do nothing if deadman switch is not held
        }

        std_msgs::Float64MultiArray msg;
        msg.data.resize(6);

        // Check for Linear vs Angular mode or mixed
        // We will use Option 3 (Linear Vel) primarily, but we can mix? 
        // The current protocol separates Linear (Opt 3) and Angular (Opt 4).
        // We need to decide based on inputs.
        
        // Simple logic: If Right Stick moves, send Angular. Else if Left Stick moves, send Linear.
        // Priority to Linear if both move? Or alternate?
        // Let's check magnitude.
        
        double lin_x = joy->axes[1]; // Up/Down
        double lin_y = joy->axes[0]; // Left/Right
        double lin_z = (joy->axes[2] < 0 ? 1.0 : 0.0) - (joy->axes[5] < 0 ? 1.0 : 0.0); // LT/RT triggers (often -1 to 1, or 0 to 1 depending on driver)
        // Note: Triggers on F310 often are 1.0 released, -1.0 pressed. Or 0 to 1. 
        // Let's assume standard: 1.0 released, -1.0 fully pressed.
        // Actually, let's map Buttons for Z for simplicity if axes are tricky.
        // Buttons Y(3) = Up, A(0) = Down
        if (joy->buttons[3]) lin_z = 1.0;
        else if (joy->buttons[0]) lin_z = -1.0;
        else lin_z = 0.0;

        double ang_yaw = joy->axes[3]; // Left/Right
        double ang_pitch = joy->axes[4]; // Up/Down
        double ang_roll = (joy->buttons[1] ? 1.0 : 0.0) - (joy->buttons[2] ? 1.0 : 0.0); // B - X

        bool is_linear = (std::abs(lin_x) > 0.1 || std::abs(lin_y) > 0.1 || std::abs(lin_z) > 0.1);
        bool is_angular = (std::abs(ang_yaw) > 0.1 || std::abs(ang_pitch) > 0.1 || std::abs(ang_roll) > 0.1);

        if (is_linear)
        {
            msg.data[0] = 3.0; // Linear Velocity
            msg.data[1] = lin_x * linear_scale_;
            msg.data[2] = lin_y * linear_scale_;
            msg.data[3] = lin_z * linear_scale_;
            msg.data[4] = 0.1; // Small time step
            msg.data[5] = (double)source_id_;
            pub_.publish(msg);
        }
        else if (is_angular)
        {
            msg.data[0] = 4.0; // Angular Velocity
            msg.data[1] = ang_roll * angular_scale_;
            msg.data[2] = ang_pitch * angular_scale_;
            msg.data[3] = ang_yaw * angular_scale_;
            msg.data[4] = 0.1; // Small time step
            msg.data[5] = (double)source_id_;
            pub_.publish(msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    int source_id_;
    double linear_scale_;
    double angular_scale_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_commander");
    JoyCommander joy_commander;
    ros::spin();
    return 0;
}
