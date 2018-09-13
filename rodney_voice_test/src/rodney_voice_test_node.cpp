#include <rodney_voice_test/rodney_voice_test_node.h>
#include <std_msgs/String.h>
#include <speech/voice.h>
#include <ros/package.h>

// Constructor 
RodneyVoiceTestNode::RodneyVoiceTestNode(ros::NodeHandle n)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyVoiceTestNode::keyboardCallBack, this);

    // Advertise the topics we publish
    speech_pub_ = nh_.advertise<speech::voice>("/speech/to_speak", 5);
    face_status_pub_ = nh_.advertise<std_msgs::String>("/robot_face/expected_input", 5);
    text_out_pub_ = nh_.advertise<std_msgs::String>("/robot_face/text_out", 5);

}
//---------------------------------------------------------------------------

void RodneyVoiceTestNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{  
    // Check no modifiers apart from num lock is excepted
    if((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0)
    {
        // Lower case
        if(msg->code == keyboard::Key::KEY_s)
        {            
            // Test status display
            std_msgs::String status_msg;            
            status_msg.data = "Rodney on line";
            face_status_pub_.publish(status_msg);                                       
        }
        else if(msg->code == keyboard::Key::KEY_t)
        {
            // Test speech and animation
                      
            // String to send to robot face
            std_msgs::String greeting;
            greeting.data = "Hello my name is Rodney";
            
            // Voice message
            speech:: voice voice_msg;
            voice_msg.text = greeting.data;
            voice_msg.wav = "";
            
            // Add the smiley
            greeting.data += ":)";
            
            // Publish topics for speech and robot face animation
            text_out_pub_.publish(greeting);
            speech_pub_.publish(voice_msg);
        }
        else if(msg->code == keyboard::Key::KEY_w)
        {
            // Test wav playback and animation
            // String to send to robot face
            std_msgs::String greeting;
            greeting.data = "Danger Will Robinson danger:&";
            
            speech:: voice voice_msg;            
            std::string path = ros::package::getPath("rodney_voice_test");
            voice_msg.text = "";
            voice_msg.wav = path + "/sounds/lost_in_space_danger.wav";            
        
            // Publish topics for sound and robot face animation
            text_out_pub_.publish(greeting);
            speech_pub_.publish(voice_msg);        
        }
        else
        {
            ;
        }
    }
}
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rodney_voice_test");
    ros::NodeHandle n;    
    RodneyVoiceTestNode rodney_test_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
    ros::spin();
    return 0;
}
