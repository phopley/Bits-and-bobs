#ifndef RODNEY_RECOGNITION_TEST_NODE_H_
#define RODNEY_RECOGNITION_TEST_NODE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition_msgs/scan_for_facesAction.h>
#include <keyboard/Key.h>

class RodneyRecognitionTestNode
{
public:
    RodneyRecognitionTestNode(ros::NodeHandle n);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber key_sub_;
    actionlib::SimpleActionClient<face_recognition_msgs::scan_for_facesAction> ac_;        
    bool scanning_;
    
    void keyboardCallBack(const keyboard::Key::ConstPtr& msg);
    void doneCB(const actionlib::SimpleClientGoalState& state,
                const face_recognition_msgs::scan_for_facesResultConstPtr& result);
    void activeCB();
    void feedbackCB(const face_recognition_msgs::scan_for_facesFeedbackConstPtr& feedback);
};

#endif // RODNEY_RECOGNITION_TEST_NODE_H_

