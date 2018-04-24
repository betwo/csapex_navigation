/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <path_msgs/PathSequence.h>
#include <path_msgs/DirectionalPath.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN RegisterNavigation : public CorePlugin
{
public:
    RegisterNavigation()
    {
    }

    void prepare(Settings&) override
    {
        connection_types::MessageConversionHook<connection_types::GenericPointerMessage, nav_msgs::OccupancyGrid>::registerConversion();
        connection_types::MessageConversionHook<connection_types::GenericPointerMessage, path_msgs::DirectionalPath>::registerConversion();
        connection_types::MessageConversionHook<connection_types::GenericPointerMessage, path_msgs::PathSequence>::registerConversion();
    }

    void shutdown()
    {
    }

};

}

CSAPEX_REGISTER_CLASS(csapex::RegisterNavigation, csapex::CorePlugin)
