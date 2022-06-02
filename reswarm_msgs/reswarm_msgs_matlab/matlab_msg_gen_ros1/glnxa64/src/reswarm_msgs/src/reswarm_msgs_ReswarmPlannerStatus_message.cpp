// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for reswarm_msgs/ReswarmPlannerStatus
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "reswarm_msgs/ReswarmPlannerStatus.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RESWARM_MSGS_EXPORT reswarm_msgs_msg_ReswarmPlannerStatus_common : public MATLABROSMsgInterface<reswarm_msgs::ReswarmPlannerStatus> {
  public:
    virtual ~reswarm_msgs_msg_ReswarmPlannerStatus_common(){}
    virtual void copy_from_struct(reswarm_msgs::ReswarmPlannerStatus* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmPlannerStatus* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_msg_ReswarmPlannerStatus_common::copy_from_struct(reswarm_msgs::ReswarmPlannerStatus* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //stamp
        const matlab::data::StructArray stamp_arr = arr["Stamp"];
        static auto msgClassPtr_stamp = loader->createInstance<MATLABROSMsgInterface<ros::Time>>("ros_msg_Time_common");
        msgClassPtr_stamp->copy_from_struct(&msg->stamp,stamp_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Stamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Stamp' is wrong type; expected a struct.");
    }
    try {
        //planner_finished
        const matlab::data::TypedArray<bool> planner_finished_arr = arr["PlannerFinished"];
        msg->planner_finished = planner_finished_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PlannerFinished' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PlannerFinished' is wrong type; expected a logical.");
    }
    try {
        //sent_robustMPC
        const matlab::data::TypedArray<bool> sent_robustMPC_arr = arr["SentRobustMPC"];
        msg->sent_robustMPC = sent_robustMPC_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SentRobustMPC' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SentRobustMPC' is wrong type; expected a logical.");
    }
    try {
        //sent_PID
        const matlab::data::TypedArray<bool> sent_PID_arr = arr["SentPID"];
        msg->sent_PID = sent_PID_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SentPID' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SentPID' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_msg_ReswarmPlannerStatus_common::get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmPlannerStatus* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Stamp","PlannerFinished","SentRobustMPC","SentPID"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs/ReswarmPlannerStatus");
    // stamp
    auto currentElement_stamp = (msg + ctr)->stamp;
    static auto msgClassPtr_stamp = loader->createInstance<MATLABROSMsgInterface<ros::Time>>("ros_msg_Time_common");
    outArray[ctr]["Stamp"] = msgClassPtr_stamp->get_arr(factory, &currentElement_stamp, loader);
    // planner_finished
    auto currentElement_planner_finished = (msg + ctr)->planner_finished;
    outArray[ctr]["PlannerFinished"] = factory.createScalar(static_cast<bool>(currentElement_planner_finished));
    // sent_robustMPC
    auto currentElement_sent_robustMPC = (msg + ctr)->sent_robustMPC;
    outArray[ctr]["SentRobustMPC"] = factory.createScalar(static_cast<bool>(currentElement_sent_robustMPC));
    // sent_PID
    auto currentElement_sent_PID = (msg + ctr)->sent_PID;
    outArray[ctr]["SentPID"] = factory.createScalar(static_cast<bool>(currentElement_sent_PID));
    }
    return std::move(outArray);
  } 
class RESWARM_MSGS_EXPORT reswarm_msgs_ReswarmPlannerStatus_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~reswarm_msgs_ReswarmPlannerStatus_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          reswarm_msgs_ReswarmPlannerStatus_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<reswarm_msgs::ReswarmPlannerStatus,reswarm_msgs_msg_ReswarmPlannerStatus_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         reswarm_msgs_ReswarmPlannerStatus_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<reswarm_msgs::ReswarmPlannerStatus,reswarm_msgs::ReswarmPlannerStatus::ConstPtr,reswarm_msgs_msg_ReswarmPlannerStatus_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_msg_ReswarmPlannerStatus_common, MATLABROSMsgInterface<reswarm_msgs::ReswarmPlannerStatus>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_ReswarmPlannerStatus_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1