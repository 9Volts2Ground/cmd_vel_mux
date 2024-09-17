// Copyright (c) 2012 Yujin Robot, Daniel Stonier, Jorge Santos
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Yujin Robot nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef CMD_VEL_MUX__CMD_VEL_MUX_HPP_
#define CMD_VEL_MUX__CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//=========================================================
// Namespaces
//=========================================================
namespace cmd_vel_mux
{

//=========================================================
// CmdVelMux
//=========================================================
struct ParameterValues
{
    std::string topic;      /**< The name of the topic */
    double timeout{-1.0};   /**< Timer's timeout, in seconds  */
    int64_t priority{-1};   /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string short_desc; /**< Short description */
    bool stamped;           /**< TODO: make this node flexible for twist messages with or w/out stamps */
};

//-------------------------------------
struct CmdVelProperties final
    {
        /// Descriptive name; must be unique to this subscriber
        std::string name_;
        ParameterValues values_;
        /// The subscriber itself
        /// No incoming messages timeout
        rclcpp::TimerBase::SharedPtr timer_;
    };

//=========================================================
// Check that the parameter values are identical
bool operator==(
    const ParameterValues & parameters1,
    const ParameterValues & parameters2
)
{
    if (parameters1.topic != parameters2.topic) {
        return false;
    } else if (parameters1.timeout != parameters2.timeout) {
        return false;
    } else if (parameters1.priority != parameters2.priority) {
        return false;
    } else if (parameters1.short_desc != parameters2.short_desc) {
        return false;
    } else if (parameters1.stamped != parameters2.stamped) {
        return false;
    }
    return true;
}

//=========================================================
class CmdVelMux final : public rclcpp::Node
{
public:
    explicit CmdVelMux(rclcpp::NodeOptions options);
    ~CmdVelMux() override = default;
    CmdVelMux(CmdVelMux && c) = delete;
    CmdVelMux & operator=(CmdVelMux && c) = delete;
    CmdVelMux(const CmdVelMux & c) = delete;
    CmdVelMux & operator=(const CmdVelMux & c) = delete;

private:
    /// ID for "nobody" active input
    static const char * const VACANT;

    /// Multiplexed command velocity topic
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_topic_pub_;

    /// Currently allowed cmd_vel subscriber
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_subscriber_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::string allowed_;
    std::string frame_id_default_;

    void timerCallback(const std::string & key);
    void cmdVelCallback(
        const std::shared_ptr<geometry_msgs::msg::Twist> msg,
        const std::string & key);
    void cmdVelStampedCallback(
        const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg,
        const std::string & key);
    bool checkToPublish(const std::string & key);

    std::map<std::string, ParameterValues> parseFromParametersMap(
        const std::map<std::string,
        rclcpp::Parameter> & parameters);
    bool parametersAreValid(
        const std::map<std::string, ParameterValues> & parameters) const;
    void configureFromParameters(
        const std::map<std::string, ParameterValues> & parameters);
    bool addInputToParameterMap(
        std::map<std::string, ParameterValues> & parsed_parameters,
        const std::string & input_name, const std::string & parameter_name,
        const rclcpp::Parameter & parameter_value);
    rcl_interfaces::msg::SetParametersResult parameterUpdate(
        const std::vector<rclcpp::Parameter> & update_parameters);

    std::map<std::string, std::shared_ptr<CmdVelProperties>> map_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> map_subs_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> map_stamped_subs_;
};

}  // namespace cmd_vel_mux

#endif  // CMD_VEL_MUX__CMD_VEL_MUX_HPP_
