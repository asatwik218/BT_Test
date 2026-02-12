#include "behaviortree_cpp/bt_factory.h"
#include "nodes/flexiv/flexiv_nodes.hpp"
#include "nodes/gripper/gripper_nodes.hpp"
#include "nodes/camera/camera_nodes.hpp"
#include "nodes/motor/motor_nodes.hpp"
#include "nodes/amr/amr_nodes_new.hpp"
#include "nodes/vision/vision_nodes.hpp"

using namespace BT;
using namespace std;

int main() {
    try {   
        // ORIGINAL CODE - commented out for debugging
        BehaviorTreeFactory factory;

        // Register all node families
        flexiv_bt::registerFlexivNodes(factory);
        gripper_bt::registerGripperNodes(factory);
        camera_bt::registerCameraNodes(factory);
        motor_bt::registerMotorNodes(factory);
        amr_bt::registerAMRNodes(factory);
        vision_bt::registerVisionNodes(factory);

        // Load and run the behavior tree
        auto tree = factory.createTreeFromFile("C:/Users/SatwikAgarwal/Documents/cynlr_software/behaviour_trees_exploration/trees/test_convergence.xml");
        tree.tickWhileRunning();
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

//how does it execute 
//what is a tick
//how to make a tree dynamically
//