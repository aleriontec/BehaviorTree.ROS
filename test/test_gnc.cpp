#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <std_msgs/Int32.h>

using namespace BT;

class WaitReachWsState : public AsyncActionNode {
public:
    WaitReachWsState(const std::string &name, const NodeConfiguration &config, ros::NodeHandle &handle)
        : AsyncActionNode(name, config), nh(handle) {}

    virtual ~WaitReachWsState() { halt(); }

    virtual void halt() override{};

    static PortsList providedPorts() { return {InputPort<int>("target_ws_state")}; }

    NodeStatus tick() override {
        setStatus(NodeStatus::RUNNING);
        wsStateSub = nh.subscribe(topicName, 1, &WaitReachWsState::wsStateCb, this);

        if (!getInput("target_ws_state", targetWSState))
            return NodeStatus::FAILURE;

        ros::Rate rate(10);
        while (ros::ok() && this->status() == NodeStatus::RUNNING) {
            ros::spinOnce();
            rate.sleep();
        }

        wsStateSub.shutdown();
        return this->status();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber wsStateSub;
    std::string topicName = "/windsurveyor_application/ws_state_topic";

    int targetWSState, currentWSState = 0;
    void wsStateCb(const std_msgs::Int32::ConstPtr &msg) {
        currentWSState = msg->data;
        if (currentWSState == targetWSState) {
            setStatus(NodeStatus::SUCCESS);
        } else if (currentWSState > targetWSState) {
            setStatus(NodeStatus::FAILURE);
        }
        std::cout << "[ INFO] [" << ros::Time::now() << "] Node name: " << this->name()
                  << ", status: " << this->status() << " (" << currentWSState << " | " << targetWSState << ")"
                  << std::endl;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_behavior_tree");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    BehaviorTreeFactory factory;

    NodeBuilder builder_WaitReachWsState = [&nh](const std::string &name, const NodeConfiguration &config) {
        return std::make_unique<WaitReachWsState>(name, config, nh);
    };
    factory.registerBuilder<WaitReachWsState>("WaitReachWsState", builder_WaitReachWsState);

    std::string xml_file_path = "/home/alerion/Documents/behavior_trees/gnc-test.xml";
    auto tree = factory.createTreeFromFile(xml_file_path);
    PublisherZMQ publisher_zmq(tree);
    printTreeRecursively(tree.rootNode());

    NodeStatus status, pre_status = NodeStatus::IDLE;
    while (ros::ok() && status != NodeStatus::FAILURE && status != NodeStatus::SUCCESS) {
        status = tree.tickRoot();
        if (status != pre_status) {
            std::cout << "[ INFO] [" << ros::Time::now() << "] Root name: " << tree.rootNode()->name()
                      << ", status: " << status << std::endl;
            pre_status = status;
        }
        rate.sleep();
    }

    return 0;
}
