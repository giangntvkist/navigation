#include "human_tracking/defs.hpp"
#include "human_tracking/imm_filter.hpp"
#include "human_tracking/detection.hpp"
#include "human_tracking/visualization.hpp"
#include "human_tracking/observation.hpp"


using namespace vk;
ObsVector human_pose_;
bool data_;

void humanPoseCallback(const std_msgs::Float64MultiArray& msg) {
    human_pose_ = ObsVector::Zero();
    human_pose_(0) = msg.data[0];
    human_pose_(1) = msg.data[1];
    std::cout << human_pose_(0) << human_pose_(1) << std::endl;
    data_ = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "human_tracking");
    ROS_INFO("Running human_tracking node!");
    ros::NodeHandle nh;
    ros::Publisher observation_pub = nh.advertise<visualization_msgs::MarkerArray>("observation_states", 5);
    ros::Publisher imm_pub = nh.advertise<visualization_msgs::MarkerArray>("imm_states", 5);
    ros::Publisher predict_pub = nh.advertise<visualization_msgs::MarkerArray>("predict_states", 5);
    ros::Subscriber human_pose_sub = nh.subscribe("human_pose", 1,humanPoseCallback);

    double publish_frequency;
    visualization_msgs::Marker node_red, node_blue, node_gre, edge_red, edge_blue, edge_gre;
    visualization_msgs::MarkerArray SetOfMarker, SetOfMarker_est, SetOfMarker_predict;

    if(!ros::param::get("~publish_frequency", publish_frequency)) publish_frequency = 10;

    visualizeMaker(node_red, edge_red, 0, "Observation", "Observation");
    visualizeMaker(node_gre, edge_gre, 1, "IMM", "IMM");
    visualizeMaker(node_blue, edge_blue, 2, "Predict", "Predict");
    
    ObsMatrix m_R = ObsMatrix::Zero();
    m_R(STATE_X_IDX, STATE_X_IDX) = 0.5;
    m_R(STATE_Y_IDX, STATE_Y_IDX) = 0.5;

    vk::IMMFilter pedestrian(&nh);
    Observation::Ptr observation (new Observation);
    FilterState::Ptr imm_state;
    IMMState::Ptr imm;
    StateVector human_state_observation = StateVector::Zero();

    bool first_time = true;
    double T;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate rate(publish_frequency);
    while(nh.ok()) {
        ros::spinOnce();
        if(data_) {
            data_ = false;
            current_time = ros::Time::now();
            T = (current_time - last_time).toSec();
            observation->z = human_pose_;
            observation->R = m_R;
            if(first_time) {
                imm_state = pedestrian.initializeTrackState(observation);
                first_time = false;
            }
            pedestrian.predictTrackState(imm_state, T);
            pedestrian.updateMatchedTrack(imm_state, observation);
            std::cout<< "Velocity: vx = " << imm_state->x()(2) << "  vy = " << imm_state->x()(3) << "  w = " << imm_state->x()(4) << std::endl;

            human_state_observation.head(2) = human_pose_;
            trajectory_visualization(human_state_observation, node_red, edge_red, SetOfMarker);
            trajectory_visualization(imm_state->x(), node_gre, edge_gre, SetOfMarker_est);

            pedestrian.statePredictionForHorizonal(imm_state, 0.1);
            imm = boost::dynamic_pointer_cast<IMMState>(imm_state);
            prediction_visualization(imm->getStatePrediction(), node_blue, edge_blue, SetOfMarker_predict);

            observation_pub.publish(SetOfMarker);
            imm_pub.publish(SetOfMarker_est);
            predict_pub.publish(SetOfMarker_predict);

            last_time = current_time;
        }
        rate.sleep();
    }
}