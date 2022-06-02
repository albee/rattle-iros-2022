// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for reswarm_msgs/ReswarmInfoStatus
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
#include "reswarm_msgs/ReswarmInfoStatus.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RESWARM_MSGS_EXPORT reswarm_msgs_msg_ReswarmInfoStatus_common : public MATLABROSMsgInterface<reswarm_msgs::ReswarmInfoStatus> {
  public:
    virtual ~reswarm_msgs_msg_ReswarmInfoStatus_common(){}
    virtual void copy_from_struct(reswarm_msgs::ReswarmInfoStatus* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmInfoStatus* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_msg_ReswarmInfoStatus_common::copy_from_struct(reswarm_msgs::ReswarmInfoStatus* msg, const matlab::data::Struct& arr,
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
        //info_traj_sent
        const matlab::data::TypedArray<bool> info_traj_sent_arr = arr["InfoTrajSent"];
        msg->info_traj_sent = info_traj_sent_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'InfoTrajSent' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'InfoTrajSent' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_msg_ReswarmInfoStatus_common::get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmInfoStatus* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Stamp","InfoTrajSent"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs/ReswarmInfoStatus");
    // stamp
    auto currentElement_stamp = (msg + ctr)->stamp;
    static auto msgClassPtr_stamp = loader->createInstance<MATLABROSMsgInterface<ros::Time>>("ros_msg_Time_common");
    outArray[ctr]["Stamp"] = msgClassPtr_stamp->get_arr(factory, &currentElement_stamp, loader);
    // info_traj_sent
    auto currentElement_info_traj_sent = (msg + ctr)->info_traj_sent;
    outArray[ctr]["InfoTrajSent"] = factory.createScalar(static_cast<bool>(currentElement_info_traj_sent));
    }
    return std::move(outArray);
  } 
class RESWARM_MSGS_EXPORT reswarm_msgs_ReswarmInfoStatus_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~reswarm_msgs_ReswarmInfoStatus_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          reswarm_msgs_ReswarmInfoStatus_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<reswarm_msgs::ReswarmInfoStatus,reswarm_msgs_msg_ReswarmInfoStatus_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         reswarm_msgs_ReswarmInfoStatus_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<reswarm_msgs::ReswarmInfoStatus,reswarm_msgs::ReswarmInfoStatus::ConstPtr,reswarm_msgs_msg_ReswarmInfoStatus_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_msg_ReswarmInfoStatus_common, MATLABROSMsgInterface<reswarm_msgs::ReswarmInfoStatus>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_ReswarmInfoStatus_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1