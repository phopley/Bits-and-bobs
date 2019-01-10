#include <rodney_head_test/rodney_head_test_node.h>
#include <actionlib/client/terminal_state.h>

// Constructor 
RodneyHeadTestNode::RodneyHeadTestNode(ros::NodeHandle n) : ac_("head_control_node", true)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyHeadTestNode::keyboardCallBack, this);

	nh_.param("/servo/index0/pan/max", max_pan_radians_, M_PI/2.0);
	nh_.param("/servo/index0/pan/min", min_pan_radians_, -(M_PI/2.0));
   	nh_.param("/servo/index0/tilt/max", max_tilt_radians_, M_PI/2.0);
    nh_.param("/servo/index0/tilt/min", min_tilt_radians_, -(M_PI/2.0));


    ROS_INFO("RodneyHeadTestNode: Waiting for action server to start");

    // wait for the action server to start
    ac_.waitForServer(); //will wait for infinite time
    
    moving_ = false;

    ROS_INFO("RodneyHeadTestNode: Action server started"); 
}
//---------------------------------------------------------------------------

void RodneyHeadTestNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{
    head_control::point_headGoal goal;

    // Check for key 1 with no modifiers apart from num lock is allowed
    if((msg->code == keyboard::Key::KEY_1) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 1, Test 1 move to max pan and tilt
        goal.absolute = true;
        goal.pan      = max_pan_radians_;
        goal.tilt     = max_tilt_radians_;
        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&RodneyHeadTestNode::doneCB, this, _1, _2),
            boost::bind(&RodneyHeadTestNode::activeCB, this),                
            boost::bind(&RodneyHeadTestNode::feedbackCB, this, _1));
    }
    if((msg->code == keyboard::Key::KEY_2) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 2, test 2 move to min pan and tilt
        goal.absolute = true;
        goal.pan      = min_pan_radians_;
        goal.tilt     = min_tilt_radians_;
        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&RodneyHeadTestNode::doneCB, this, _1, _2),
            boost::bind(&RodneyHeadTestNode::activeCB, this),                
            boost::bind(&RodneyHeadTestNode::feedbackCB, this, _1));
    }
    if((msg->code == keyboard::Key::KEY_3) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 3, test 3 move to pan 0, tilt 0
        goal.absolute = true;
        goal.pan      = 0.0;
        goal.tilt     = 0.0;
        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&RodneyHeadTestNode::doneCB, this, _1, _2),
            boost::bind(&RodneyHeadTestNode::activeCB, this),                
            boost::bind(&RodneyHeadTestNode::feedbackCB, this, _1));
    }
    if((msg->code == keyboard::Key::KEY_4) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 4, test 4 move to pan 0, tilt -45 degress
        goal.absolute = true;
        goal.pan      = 0.0;
        goal.tilt     = -0.785398;
        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&RodneyHeadTestNode::doneCB, this, _1, _2),
            boost::bind(&RodneyHeadTestNode::activeCB, this),                
            boost::bind(&RodneyHeadTestNode::feedbackCB, this, _1));
    }
    if((msg->code == keyboard::Key::KEY_5) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 5, test 5 move tilt up by 10 degrees
        goal.absolute = false;
        goal.pan      = 0;
        goal.tilt     = -0.174533;
        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&RodneyHeadTestNode::doneCB, this, _1, _2),
            boost::bind(&RodneyHeadTestNode::activeCB, this),                
            boost::bind(&RodneyHeadTestNode::feedbackCB, this, _1));
    }
    if((msg->code == keyboard::Key::KEY_6) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 6, test 6 move pan by 20 anti-clockwise
        goal.absolute = false;
        goal.pan      = 0.349066;
        goal.tilt     = 0;
        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&RodneyHeadTestNode::doneCB, this, _1, _2),
            boost::bind(&RodneyHeadTestNode::activeCB, this),                
            boost::bind(&RodneyHeadTestNode::feedbackCB, this, _1));
    }
    if((msg->code == keyboard::Key::KEY_7) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 7, test 7 move pan by 20 clockwise and tilt by 10 down
        goal.absolute = false;
        goal.pan      = -0.349066;
        goal.tilt     = 0.174533;
        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&RodneyHeadTestNode::doneCB, this, _1, _2),
            boost::bind(&RodneyHeadTestNode::activeCB, this),                
            boost::bind(&RodneyHeadTestNode::feedbackCB, this, _1));
    }
    else if((msg->code == keyboard::Key::KEY_c) && ((msg->modifiers & ~RodneyHeadTestNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {          
        // Key 'c' or 'C', cancel action
        if(moving_ == true)
        {
            ac_.cancelGoal();
        }  
    }
    else
    {
        ;
    }
}
//---------------------------------------------------------------------------

// Called once when the goal completes
void RodneyHeadTestNode::doneCB(const actionlib::SimpleClientGoalState& state,
                        const head_control::point_headResultConstPtr& result)                 
{
    ROS_INFO("RodneyHeadTestNode: Finished in state [%s]", state.toString().c_str());
    moving_ = false;    
}
//---------------------------------------------------------------------------

// Called once when the goal becomes active
void RodneyHeadTestNode::activeCB()
{
    ROS_INFO("RodneyHeadTestNode: Goal just went active");
    
    moving_ = true;
}
//---------------------------------------------------------------------------

// Called every time feedback is received for the goal
void RodneyHeadTestNode::feedbackCB(const head_control::point_headFeedbackConstPtr& feedback)
{
    ROS_INFO("Feedback pan=%f, tilt=%f", feedback->current_pan, feedback->current_tilt);    
}
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rodney_head_test");
    ros::NodeHandle n;    
    RodneyHeadTestNode rodney_head_test_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
    ros::spin();
    return 0;
}


