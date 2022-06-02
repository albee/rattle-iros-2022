// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for reswarm_msgs_matlab/ReswarmCasadiStatus
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
#include "reswarm_msgs_matlab/ReswarmCasadiStatus.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RESWARM_MSGS_MATLAB_EXPORT reswarm_msgs_matlab_msg_ReswarmCasadiStatus_common : public MATLABROSMsgInterface<reswarm_msgs_matlab::ReswarmCasadiStatus> {
  public:
    virtual ~reswarm_msgs_matlab_msg_ReswarmCasadiStatus_common(){}
    virtual void copy_from_struct(reswarm_msgs_matlab::ReswarmCasadiStatus* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs_matlab::ReswarmCasadiStatus* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_matlab_msg_ReswarmCasadiStatus_common::copy_from_struct(reswarm_msgs_matlab::ReswarmCasadiStatus* msg, const matlab::data::Struct& arr,
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
        //coord_ok
        const matlab::data::TypedArray<bool> coord_ok_arr = arr["CoordOk"];
        msg->coord_ok = coord_ok_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'CoordOk' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'CoordOk' is wrong type; expected a logical.");
    }
    try {
        //mrpi_finished
        const matlab::data::TypedArray<bool> mrpi_finished_arr = arr["MrpiFinished"];
        msg->mrpi_finished = mrpi_finished_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'MrpiFinished' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'MrpiFinished' is wrong type; expected a logical.");
    }
    try {
        //traj_finished
        const matlab::data::TypedArray<bool> traj_finished_arr = arr["TrajFinished"];
        msg->traj_finished = traj_finished_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TrajFinished' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TrajFinished' is wrong type; expected a logical.");
    }
    try {
        //control_mode
        const matlab::data::CharArray control_mode_arr = arr["ControlMode"];
        msg->control_mode = control_mode_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ControlMode' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ControlMode' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_matlab_msg_ReswarmCasadiStatus_common::get_arr(MDFactory_T& factory, const reswarm_msgs_matlab::ReswarmCasadiStatus* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Stamp","CoordOk","MrpiFinished","TrajFinished","ControlMode"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs_matlab/ReswarmCasadiStatus");
    // stamp
    auto currentElement_stamp = (msg + ctr)->stamp;
    static auto msgClassPtr_stamp = loader->createInstance<MATLABROSMsgInterface<ros::Time>>("ros_msg_Time_common");
    outArray[ctr]["Stamp"] = msgClassPtr_stamp->get_arr(factory, &currentElement_stamp, loader);
    // coord_ok
    auto currentElement_coord_ok = (msg + ctr)->coord_ok;
    outArray[ctr]["CoordOk"] = factory.createScalar(static_cast<bool>(currentElement_coord_ok));
    // mrpi_finished
    auto currentElement_mrpi_finished = (msg + ctr)->mrpi_finished;
    outArray[ctr]["MrpiFinished"] = factory.createScalar(static_cast<bool>(currentElement_mrpi_finished));
    // traj_finished
    auto currentElement_traj_finished = (msg + ctr)->traj_finished;
    outArray[ctr]["TrajFinished"] = factory.createScalar(static_cast<bool>(currentElement_traj_finished));
    // control_mode
    auto currentElement_control_mode = (msg + ctr)->control_mode;
    outArray[ctr]["ControlMode"] = factory.createCharArray(currentElement_control_mode);
    }
    return std::move(outArray);
  } 
class RESWARM_MSGS_MATLAB_EXPORT reswarm_msgs_matlab_ReswarmCasadiStatus_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~reswarm_msgs_matlab_ReswarmCasadiStatus_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          reswarm_msgs_matlab_ReswarmCasadiStatus_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<reswarm_msgs_matlab::ReswarmCasadiStatus,reswarm_msgs_matlab_msg_ReswarmCasadiStatus_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         reswarm_msgs_matlab_ReswarmCasadiStatus_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<reswarm_msgs_matlab::ReswarmCasadiStatus,reswarm_msgs_matlab::ReswarmCasadiStatus::ConstPtr,reswarm_msgs_matlab_msg_ReswarmCasadiStatus_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_matlab_msg_ReswarmCasadiStatus_common, MATLABROSMsgInterface<reswarm_msgs_matlab::ReswarmCasadiStatus>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_matlab_ReswarmCasadiStatus_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1