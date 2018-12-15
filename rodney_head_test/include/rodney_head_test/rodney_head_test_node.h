#ifndef RODNEY_HEAD_TEST_NODE_H_
#define RODNEY_HEAD_TEST_NODE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <head_control/point_headAction.h>
#include <keyboard/Key.h>

class RodneyHeadTestNode
{
public:
    RodneyHeadTestNode(ros::NodeHandle n);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber key_sub_;
    actionlib::SimpleActionClient<head_control::point_headAction> ac_;        
    bool moving_;

    const uint16_t SHIFT_CAPS_NUM_LOCK_ = (keyboard::Key::MODIFIER_NUM | keyboard::Key::MODIFIER_CAPS | 
                                           keyboard::Key::MODIFIER_LSHIFT | keyboard::Key::MODIFIER_RSHIFT);
    
    void keyboardCallBack(const keyboard::Key::ConstPtr& msg);
    void doneCB(const actionlib::SimpleClientGoalState& state,
                const head_control::point_headResultConstPtr& result);
    void activeCB();
    void feedbackCB(const head_control::point_headFeedbackConstPtr& feedback);
};

#endif // RODNEY_HEAD_TEST_NODE_H_


