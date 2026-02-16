#include "behaviortree_cpp/bt_factory.h"
#include "nodes/flexiv/flexiv_nodes.hpp"
#include "nodes/gripper/gripper_nodes.hpp"
#include "nodes/camera/camera_nodes.hpp"
#include "nodes/motor/motor_nodes.hpp"
#include "nodes/amr/amr_nodes_new.hpp"
#include "nodes/amr/amr_nodes_async.hpp"
#include "nodes/vision/vision_nodes.hpp"
#include "nodes/monitoring/monitoring_nodes.hpp"
#include "core/emergency_stop.hpp"

#include <thread>
#include <chrono>

using namespace BT;
using namespace std;

int main() {
    try {
        BehaviorTreeFactory factory;

        // Register all node families (sync + async)
        flexiv_bt::registerFlexivNodes(factory);
        gripper_bt::registerGripperNodes(factory);
        camera_bt::registerCameraNodes(factory);
        motor_bt::registerMotorNodes(factory);
        amr_bt::registerAMRNodes(factory);
        amr_bt::registerAMRNodesAsync(factory);
        vision_bt::registerVisionNodes(factory);
        monitoring_bt::registerMonitoringNodes(factory);

        // Load the behavior tree
        auto tree = factory.createTreeFromFile("C:/Users/SatwikAgarwal/Documents/cynlr_software/behaviour_trees_exploration/trees/test_convergence.xml");

        // Start emergency stop monitoring (ESC key)
        core::EmergencyStopManager::instance().start();

        // Custom tick loop with emergency stop check
        NodeStatus status = NodeStatus::RUNNING;
        while (status == NodeStatus::RUNNING) {
            if (core::EmergencyStopManager::instance().isHaltRequested()) {
                cerr << "[main] Emergency stop - halting tree" << endl;
                tree.haltTree();
                break;
            }
            status = tree.tickOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        core::EmergencyStopManager::instance().stop();

        if (status == NodeStatus::SUCCESS) {
            cout << "[main] Tree completed successfully" << endl;
        } else if (status == NodeStatus::FAILURE) {
            cerr << "[main] Tree failed" << endl;
        } else {
            cerr << "[main] Tree halted by emergency stop" << endl;
        }

        return 0;

    }
    catch (const exception& e) {
        cerr << "[main] Exception: " << e.what() << endl;
    }
    catch (...) {
        cerr << "[main] Unknown exception caught" << endl;
    }
    return 0;
}
