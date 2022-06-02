// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for reswarm_msgs_matlab/ReswarmStatusSecondary
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
#include "reswarm_msgs_matlab/ReswarmStatusSecondary.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RESWARM_MSGS_MATLAB_EXPORT reswarm_msgs_matlab_msg_ReswarmStatusSecondary_common : public MATLABROSMsgInterface<reswarm_msgs_matlab::ReswarmStatusSecondary> {
  public:
    virtual ~reswarm_msgs_matlab_msg_ReswarmStatusSecondary_common(){}
    virtual void copy_from_struct(reswarm_msgs_matlab::ReswarmStatusSecondary* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs_matlab::ReswarmStatusSecondary* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_matlab_msg_ReswarmStatusSecondary_common::copy_from_struct(reswarm_msgs_matlab::ReswarmStatusSecondary* msg, const matlab::data::Struct& arr,
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
        //test_number
        const matlab::data::TypedArray<int32_t> test_number_arr = arr["TestNumber"];
        msg->test_number = test_number_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TestNumber' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TestNumber' is wrong type; expected a int32.");
    }
    try {
        //default_control
        const matlab::data::TypedArray<bool> default_control_arr = arr["DefaultControl"];
        msg->default_control = default_control_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'DefaultControl' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'DefaultControl' is wrong type; expected a logical.");
    }
    try {
        //flight_mode
        const matlab::data::CharArray flight_mode_arr = arr["FlightMode"];
        msg->flight_mode = flight_mode_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FlightMode' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FlightMode' is wrong type; expected a string.");
    }
    try {
        //test_finished
        const matlab::data::TypedArray<bool> test_finished_arr = arr["TestFinished"];
        msg->test_finished = test_finished_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TestFinished' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TestFinished' is wrong type; expected a logical.");
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
        //solver_status
        const matlab::data::TypedArray<int32_t> solver_status_arr = arr["SolverStatus"];
        msg->solver_status = solver_status_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SolverStatus' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SolverStatus' is wrong type; expected a int32.");
    }
    try {
        //cost_value
        const matlab::data::TypedArray<float> cost_value_arr = arr["CostValue"];
        msg->cost_value = cost_value_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'CostValue' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'CostValue' is wrong type; expected a single.");
    }
    try {
        //kkt_value
        const matlab::data::TypedArray<float> kkt_value_arr = arr["KktValue"];
        msg->kkt_value = kkt_value_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'KktValue' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'KktValue' is wrong type; expected a single.");
    }
    try {
        //sol_time
        const matlab::data::TypedArray<float> sol_time_arr = arr["SolTime"];
        msg->sol_time = sol_time_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'SolTime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'SolTime' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_matlab_msg_ReswarmStatusSecondary_common::get_arr(MDFactory_T& factory, const reswarm_msgs_matlab::ReswarmStatusSecondary* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Stamp","TestNumber","DefaultControl","FlightMode","TestFinished","CoordOk","SolverStatus","CostValue","KktValue","SolTime"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs_matlab/ReswarmStatusSecondary");
    // stamp
    auto currentElement_stamp = (msg + ctr)->stamp;
    static auto msgClassPtr_stamp = loader->createInstance<MATLABROSMsgInterface<ros::Time>>("ros_msg_Time_common");
    outArray[ctr]["Stamp"] = msgClassPtr_stamp->get_arr(factory, &currentElement_stamp, loader);
    // test_number
    auto currentElement_test_number = (msg + ctr)->test_number;
    outArray[ctr]["TestNumber"] = factory.createScalar(currentElement_test_number);
    // default_control
    auto currentElement_default_control = (msg + ctr)->default_control;
    outArray[ctr]["DefaultControl"] = factory.createScalar(static_cast<bool>(currentElement_default_control));
    // flight_mode
    auto currentElement_flight_mode = (msg + ctr)->flight_mode;
    outArray[ctr]["FlightMode"] = factory.createCharArray(currentElement_flight_mode);
    // test_finished
    auto currentElement_test_finished = (msg + ctr)->test_finished;
    outArray[ctr]["TestFinished"] = factory.createScalar(static_cast<bool>(currentElement_test_finished));
    // coord_ok
    auto currentElement_coord_ok = (msg + ctr)->coord_ok;
    outArray[ctr]["CoordOk"] = factory.createScalar(static_cast<bool>(currentElement_coord_ok));
    // solver_status
    auto currentElement_solver_status = (msg + ctr)->solver_status;
    outArray[ctr]["SolverStatus"] = factory.createScalar(currentElement_solver_status);
    // cost_value
    auto currentElement_cost_value = (msg + ctr)->cost_value;
    outArray[ctr]["CostValue"] = factory.createScalar(currentElement_cost_value);
    // kkt_value
    auto currentElement_kkt_value = (msg + ctr)->kkt_value;
    outArray[ctr]["KktValue"] = factory.createScalar(currentElement_kkt_value);
    // sol_time
    auto currentElement_sol_time = (msg + ctr)->sol_time;
    outArray[ctr]["SolTime"] = factory.createScalar(currentElement_sol_time);
    }
    return std::move(outArray);
  } 
class RESWARM_MSGS_MATLAB_EXPORT reswarm_msgs_matlab_ReswarmStatusSecondary_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~reswarm_msgs_matlab_ReswarmStatusSecondary_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          reswarm_msgs_matlab_ReswarmStatusSecondary_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<reswarm_msgs_matlab::ReswarmStatusSecondary,reswarm_msgs_matlab_msg_ReswarmStatusSecondary_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         reswarm_msgs_matlab_ReswarmStatusSecondary_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<reswarm_msgs_matlab::ReswarmStatusSecondary,reswarm_msgs_matlab::ReswarmStatusSecondary::ConstPtr,reswarm_msgs_matlab_msg_ReswarmStatusSecondary_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_matlab_msg_ReswarmStatusSecondary_common, MATLABROSMsgInterface<reswarm_msgs_matlab::ReswarmStatusSecondary>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_matlab_ReswarmStatusSecondary_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1