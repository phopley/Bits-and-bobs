#include <rodney_recognition_test/rodney_recognition_test_node.h>
#include <algorithm>
#include <actionlib/client/terminal_state.h>

// Constructor 
RodneyRecognitionTestNode::RodneyRecognitionTestNode(ros::NodeHandle n) : ac_("head_control_node", true)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyRecognitionTestNode::keyboardCallBack, this);

    ROS_INFO("RodneyRecognitionTestNode: Waiting for action server to start");

    // wait for the action server to start
    ac_.waitForServer(); //will wait for infinite time
    
    scanning_ = false;

    ROS_INFO("RodneyRecognitionTestNode: Action server started"); 
}
//---------------------------------------------------------------------------

void RodneyRecognitionTestNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{        
    // Check no modifiers apart from num lock is excepted
    if((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0)
    {
        // Lower case
        if(msg->code == keyboard::Key::KEY_s)
        {            
            // Lower case 's', start a complete scan looking for faces
            // Send a goal to the action
            face_recognition_msgs::scan_for_facesGoal goal;
        
            // Need boost::bind to pass in the 'this' pointer
            ac_.sendGoal(goal,
                boost::bind(&RodneyRecognitionTestNode::doneCB, this, _1, _2),
                boost::bind(&RodneyRecognitionTestNode::activeCB, this),                
                boost::bind(&RodneyRecognitionTestNode::feedbackCB, this, _1));                                        
        }
        else if(msg->code == keyboard::Key::KEY_c)
        {          
            // Lower case 'c', cancel scan if one is running
            if(scanning_ == true)
            {
                ac_.cancelGoal();
            }        
        }
        else
        {
            ;
        }
    }
}
//---------------------------------------------------------------------------

// Called once when the goal completes
void RodneyRecognitionTestNode::doneCB(const actionlib::SimpleClientGoalState& state,
                        const face_recognition_msgs::scan_for_facesResultConstPtr& result)                 
{
    ROS_DEBUG("RodneyRecognitionTestNode: Finished in state [%s]", state.toString().c_str());
    scanning_ = false;    

    if(result->detected.ids_detected.size() > 0)
    {  
        for(unsigned long x = 0; x < result->detected.ids_detected.size(); x++)
        {
            // Log we have seen you now!
            ROS_INFO("RodneyRecognitionTestNode: Hello %s, how are you?", result->detected.names_detected[x].c_str());            
        }            
    }
}
//---------------------------------------------------------------------------

// Called once when the goal becomes active
void RodneyRecognitionTestNode::activeCB()
{
    ROS_DEBUG("RodneyRecognitionTestNode: Goal just went active");
    
    scanning_ = true;
}
//---------------------------------------------------------------------------

// Called every time feedback is received for the goal
void RodneyRecognitionTestNode::feedbackCB(const face_recognition_msgs::scan_for_facesFeedbackConstPtr& feedback)
{
    ROS_DEBUG("Got Feedback percentage complete %f", feedback->progress);    
    
    if(feedback->detected.ids_detected.size() > 0)
    {  
        for(unsigned long x = 0; x < feedback->detected.ids_detected.size(); x++)
        {
            // Log just seen
            ROS_INFO("RodneyRecognitionTestNode: Just seen %s", feedback->detected.names_detected[x].c_str());          
        }          
    }
}
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rodney_test");
    ros::NodeHandle n;    
    RodneyRecognitionTestNode rodney_test_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
    ros::spin();
    return 0;
}

