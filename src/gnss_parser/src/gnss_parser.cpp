#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>

serial::Serial gnss_serial;
ros::Publisher gnss_pub;

template <class Type>
Type stringToNum(const std::string &str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

std::vector<std::string> stringSplit(const std::string &s, const char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

void gnssMessageParser(const std::string &msg)
{
    //$GNGGA,090031.00,2057.59811809,N,10546.17292292,E,1,18,2.2,16.4378,M,-28.2478,M,,*64
    //ROS_INFO_STREAM(msg);
    std::vector<std::string> message = stringSplit(msg, ',');
    if (message[0] == "$GNGGA")
    {
        sensor_msgs::NavSatFix navfix_msg;
        //if (gnss_pub.getNumSubscribers() > 0)
        //{
        navfix_msg.latitude = stringToNum<double>(message[2]);
        navfix_msg.longitude = stringToNum<double>(message[4]);
        navfix_msg.altitude = stringToNum<double>(message[9]);
        ROS_INFO_STREAM(std::setprecision(20) << navfix_msg.latitude << ", " << navfix_msg.longitude << ", " << navfix_msg.altitude);
        gnss_pub.publish(navfix_msg);
        //}
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_parser");
    ros::NodeHandle node;
    gnss_pub = node.advertise<sensor_msgs::NavSatFix>("gnss_position", 100);

    std::string port;
    int32_t baudrate;

    ros::param::get("~port", port);
    ros::param::get("~baudrate", baudrate);

    ROS_INFO_STREAM("Opening " << port << " with baudrate " << baudrate);
    gnss_serial.setPort(port);
    gnss_serial.setBaudrate(baudrate);

    while (ros::ok())
    {
        try
        {
            gnss_serial.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open the serial port!");
        }

        if (gnss_serial.isOpen())
        {
            ROS_INFO_STREAM("Successfully connected to serial port.");
            gnss_serial.flushInput();
            try
            {
                while (ros::ok())
                {
                    ros::spinOnce();
                    std::string message;
                    gnss_serial.waitReadable();
                    if (gnss_serial.available())
                    {
                        message = gnss_serial.readline();
                        gnssMessageParser(message);
                    }
                }
            }
            catch (const std::exception &e)
            {
                if (gnss_serial.isOpen())
                {
                    gnss_serial.close();
                }
                ROS_ERROR_STREAM(e.what());
                ROS_INFO_STREAM("Re-connect in 1 second.");
                ros::Duration(1.0).sleep();
            }
        }
        else
        {
            ROS_ERROR_STREAM("The serial port is not connected correctly!");
            ROS_INFO_STREAM("Re-connect in 1 second.");
            ros::Duration(1.0).sleep();
        }
    }
}
