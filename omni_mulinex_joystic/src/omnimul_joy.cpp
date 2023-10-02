#include "omni_mulinex_joystic/omnimul_joy.hpp"
#include "rclcpp/qos.hpp"
#include <cassert>

#define BASE_WS "/home/jacopo/Documents/"
#define BAG_NAME "/Experiment_OM"

namespace omni_mulinex_joy
{
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    void OmniMulinex_Joystic::get_param()
    {
        // get the parameter and saturate their value with the define values
        sup_vx_ =  this->get_parameter("sup_vel_x").as_double();
        sup_vx_ = sup_vx_>MAX_LIN_VEL?MAX_LIN_VEL:sup_vx_;
        sup_vy_ = this->get_parameter("sup_vel_y").as_double();
        sup_vy_ = sup_vy_>MAX_LIN_VEL?MAX_LIN_VEL:sup_vy_;
        sup_omega_ = this->get_parameter("sup_omega").as_double();
        sup_omega_ = sup_omega_>MAX_ROT_VEL?MAX_ROT_VEL:sup_omega_;
        sup_height_rate_ = this->get_parameter("sup_height_rate").as_double();
        sup_height_rate_ = sup_height_rate_>MAX_HEIGHT_RATE?MAX_HEIGHT_RATE:sup_height_rate_;
        register_state_ = this->get_parameter("save_state").as_bool();
        timer_dur_ = this->get_parameter("timer_duration").as_int();
        bag_folder_ = this->get_parameter("bag_folder").as_string();
        if(register_state_)
            stt_period_ = this->get_parameter("state_duration").as_int();

    };

    void OmniMulinex_Joystic::set_tools()
    {
        rclcpp::QoS cmd_qos(10),stt_qos(10);
        std::string bag_exp_name;
        // std::chrono::duration dur = std::chrono::milliseconds timer_dur_;//, stt_dur(stt_period_);
        std::chrono::duration dur = std::chrono::milliseconds(timer_dur_), stt_dur = std::chrono::milliseconds(stt_period_);
        std::shared_ptr<rclcpp::CallbackGroup> node_cb_grp = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions opt_stt, input_opt;
        rclcpp::PublisherOptions opt_cmd;

        // get local time
        time_t tm_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        auto lt_now = std::localtime(&tm_now);
        // make writer and save in bag called as defined 
        
        bag_exp_name = BASE_WS + bag_folder_ + BAG_NAME + std::string("_") + std::to_string(lt_now->tm_year+1900) + "_" + std::to_string(lt_now->tm_mon+1) + "_" + std::to_string(lt_now->tm_mday) + "_" +
            std::to_string(lt_now->tm_hour) + ":" + std::to_string(lt_now->tm_min) + ":" +std::to_string(lt_now->tm_sec);

        // create the writer 
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        try
        {
            writer_->open(bag_exp_name);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            assert(true);
        }

        
        // set the QOS for the Wireless communication for both the command data, containing the joystic input, and the 
        // state data, provided by the interface 
        cmd_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        cmd_qos.deadline(dur);
        stt_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        stt_qos.deadline(stt_dur);

        
        // associate to the publisher a event callback to detect the missed deadline
        opt_cmd.event_callbacks.deadline_callback = [logger = this -> get_logger()](rclcpp::QOSDeadlineOfferedInfo& event)->void
        {
            RCLCPP_INFO(logger,"The command deadline has been missed, the counter is %d and its variation is %d"
            ,event.total_count,event.total_count_change);
        };

        // associate to the subscriber both the callback group to access to the data variable with Mutex Exclusion and the event
        // callback tp detect the missed deadline if the parameter is set
        if(register_state_)
        {
            opt_stt.callback_group = node_cb_grp;
            opt_stt.event_callbacks.deadline_callback = [logger = this -> get_logger()](rclcpp::QOSDeadlineRequestedInfo& event)->void
            {
                RCLCPP_INFO(logger,"The state subscriber deadline has been missed, the counter is %d and its variation is %d"
                ,event.total_count,event.total_count_change);
            };
            stt_sub_ = this->create_subscription<OM_State>("/joint_state",stt_qos,std::bind(&OmniMulinex_Joystic::stt_callback,this,_1),opt_stt);  
        }
        // associate the subscription to joy topic to the CallbackGroup
        input_opt.callback_group = node_cb_grp;

        cmd_sub_ = this->create_subscription<JoyCommand>("joystic_command",10,std::bind(&OmniMulinex_Joystic::joy_command,this,_1),input_opt);
        

        cmd_pub_ = this->create_publisher<OM_JoyCmd>("OM_command",cmd_qos,opt_cmd);

        timer_ = this->create_wall_timer(dur,std::bind(&OmniMulinex_Joystic::main_callback,this),node_cb_grp);


    }
    void OmniMulinex_Joystic::joy_command(const std::shared_ptr<JoyCommand> msg)
    {
        // axis 1 is vx, axis 0 is vy, axis 3 is omega and axis 4 is height rate
        v_x_ = - sup_vx_*msg->axes[1];
        v_y_ = sup_vy_*msg->axes[0];
        omega_ = sup_omega_*msg->axes[3];
        h_rate_ = sup_height_rate_*msg->axes[4];

        // add writer to save the command data
        
    }

    void OmniMulinex_Joystic::stt_callback(const std::shared_ptr<OM_State> msg)
    {
        // it directly save all the data 
    }

    void OmniMulinex_Joystic::main_callback()
    {
        // set the message 
        cmd_msg_.set__v_x(v_x_);
        cmd_msg_.set__v_y(v_y_);
        cmd_msg_.set__omega(omega_);
        cmd_msg_.set__height_rate(h_rate_);
        cmd_msg_.header.set__stamp(this->now());
        // publish the message
        cmd_pub_->publish(cmd_msg_);
    }
}