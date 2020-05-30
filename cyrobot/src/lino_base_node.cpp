#include "lino_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "cyrobot_base_node");
    LinoBase lino;
    ros::spin();
    return 0;
}