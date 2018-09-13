#ifndef RODNEY_VOICE_TEST_NODE_H_
#define RODNEY_VOICE_TEST_NODE_H_

#include <ros/ros.h>
#include <keyboard/Key.h>

class RodneyVoiceTestNode
{
public:
    RodneyVoiceTestNode(ros::NodeHandle n);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber key_sub_;
    ros::Publisher speech_pub_;
    ros::Publisher face_status_pub_;
    ros::Publisher text_out_pub_;
    
    void keyboardCallBack(const keyboard::Key::ConstPtr& msg);

};

#endif // RODNEY_VOICE_TEST_NODE_H_
