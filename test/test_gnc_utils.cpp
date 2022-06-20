#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <std_msgs/Int32.h>

using namespace BT;

//-------------------------------------------------------------
// Simple Action to print a number
//-------------------------------------------------------------

class PrintValue : public SyncActionNode {
public:
    PrintValue(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

    NodeStatus tick() override {
        int value = 0;
        if (getInput("message", value)) {
            std::cout << "[ INFO] [" << ros::Time::now() << "]: PrintValue: " << value << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << "PrintValue FAILED " << std::endl;
            return NodeStatus::FAILURE;
        }
    }

    static PortsList providedPorts() { return {InputPort<int>("message")}; }
};

//-------------------------------------------------------------
// Simple Action to print a bool
//-------------------------------------------------------------

class PrintBool : public SyncActionNode {
public:
    PrintBool(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

    NodeStatus tick() override {
        bool value;
        if (getInput("message", value)) {
            std::cout << "[ INFO] [" << ros::Time::now() << "]: PrintBool: " << value << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << "PrintBool FAILED " << std::endl;
            return NodeStatus::FAILURE;
        }
    }

    static PortsList providedPorts() { return {InputPort<bool>("message")}; }
};

NodeStatus isWsStateReached(TreeNode &self) {
    auto wsState = self.getInput<int>("ws_state");
    auto targetWSState = self.getInput<int>("target_ws_state");
    if (wsState == targetWSState) {
        std::cout << "[ INFO] [" << ros::Time::now() << "]: WS State " << wsState.value() << " reached!" << std::endl;
        self.setOutput<bool>("ws_state_visited", true);
        return NodeStatus::SUCCESS;
    } else {
        return NodeStatus::RUNNING;
    }
}

NodeStatus hasWsStateBeenVisited(TreeNode &self) {
    auto visited = self.getInput<bool>("ws_state_visited");
    auto wsState = self.getInput<int>("ws_state");
    if (visited.has_value() && visited.value()) {
        std::cout << "[ INFO] [" << ros::Time::now() << "]: WS State " << wsState.value() << " already visited!"
                  << std::endl;
        return NodeStatus::SUCCESS;
    } else {
        return NodeStatus::FAILURE;
    }
}


int main(int argc, char **argv) {
    BehaviorTreeFactory factory;

    factory.registerNodeType<PrintValue>("PrintValue");
    factory.registerNodeType<PrintBool>("PrintBool");

    PortsList isWsStateReachedPorts = {InputPort<int>("ws_state"), InputPort<int>("target_ws_state"),
                                       OutputPort<bool>("ws_state_visited")};
    factory.registerSimpleCondition("isWsStateReached", isWsStateReached, isWsStateReachedPorts);

    PortsList hasWsStateBeenVisitedPorts = {InputPort<bool>("ws_state_visited"), InputPort<int>("ws_state")};
    factory.registerSimpleCondition("hasWsStateBeenVisited", hasWsStateBeenVisited, hasWsStateBeenVisitedPorts);

    std::string bt_pkg_path = ros::package::getPath("behaviortree_ros");
    std::string xml_file_path = bt_pkg_path + "/data/gnc.xml";
    auto tree = factory.createTreeFromFile(xml_file_path);
    PublisherZMQ publisher_zmq(tree);
    printTreeRecursively(tree.rootNode());

    NodeStatus status = NodeStatus::IDLE;

    while (status == NodeStatus::IDLE || status == NodeStatus::RUNNING) {
        status = tree.tickRoot();
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Behavior Tree -> [" << status << "]" << std::endl;
        sleep(1);
    }

    return 0;
}
