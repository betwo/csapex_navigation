
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class StopMovement : public Node
{
public:
    StopMovement()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        slot_ = modifier.addSlot("stop", [this]() {
        });
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
    }

private:
    Slot* slot_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::StopMovement, csapex::Node)

