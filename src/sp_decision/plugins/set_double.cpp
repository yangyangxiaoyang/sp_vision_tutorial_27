#include "set_double.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
    factory.registerNodeType<sp_decision::SetDouble>("SetDouble");
}
