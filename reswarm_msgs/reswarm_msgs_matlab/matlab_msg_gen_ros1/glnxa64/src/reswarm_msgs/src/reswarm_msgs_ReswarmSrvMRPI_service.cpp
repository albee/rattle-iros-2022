// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for reswarm_msgs/ReswarmSrvMRPIRequest
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
#include "reswarm_msgs/ReswarmSrvMRPI.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class RESWARM_MSGS_EXPORT reswarm_msgs_msg_ReswarmSrvMRPIRequest_common : public MATLABROSMsgInterface<reswarm_msgs::ReswarmSrvMRPI::Request> {
  public:
    virtual ~reswarm_msgs_msg_ReswarmSrvMRPIRequest_common(){}
    virtual void copy_from_struct(reswarm_msgs::ReswarmSrvMRPI::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmSrvMRPI::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_msg_ReswarmSrvMRPIRequest_common::copy_from_struct(reswarm_msgs::ReswarmSrvMRPI::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //w
        const matlab::data::StructArray w_arr = arr["W"];
        static auto msgClassPtr_w = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_w->copy_from_struct(&msg->w,w_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'W' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'W' is wrong type; expected a struct.");
    }
    try {
        //u_max
        const matlab::data::StructArray u_max_arr = arr["UMax"];
        static auto msgClassPtr_u_max = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_u_max->copy_from_struct(&msg->u_max,u_max_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'UMax' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'UMax' is wrong type; expected a struct.");
    }
    try {
        //dt
        const matlab::data::TypedArray<double> dt_arr = arr["Dt"];
        msg->dt = dt_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Dt' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Dt' is wrong type; expected a double.");
    }
    try {
        //mass
        const matlab::data::TypedArray<double> mass_arr = arr["Mass"];
        msg->mass = mass_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Mass' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Mass' is wrong type; expected a double.");
    }
    try {
        //Q_pos_anc
        const matlab::data::TypedArray<double> Q_pos_anc_arr = arr["QPosAnc"];
        msg->Q_pos_anc = Q_pos_anc_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QPosAnc' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QPosAnc' is wrong type; expected a double.");
    }
    try {
        //Q_vel_anc
        const matlab::data::TypedArray<double> Q_vel_anc_arr = arr["QVelAnc"];
        msg->Q_vel_anc = Q_vel_anc_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QVelAnc' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QVelAnc' is wrong type; expected a double.");
    }
    try {
        //R_anc
        const matlab::data::TypedArray<double> R_anc_arr = arr["RAnc"];
        msg->R_anc = R_anc_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RAnc' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RAnc' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_msg_ReswarmSrvMRPIRequest_common::get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmSrvMRPI::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","W","UMax","Dt","Mass","QPosAnc","QVelAnc","RAnc"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs/ReswarmSrvMRPIRequest");
    // w
    auto currentElement_w = (msg + ctr)->w;
    static auto msgClassPtr_w = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["W"] = msgClassPtr_w->get_arr(factory, &currentElement_w, loader);
    // u_max
    auto currentElement_u_max = (msg + ctr)->u_max;
    static auto msgClassPtr_u_max = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["UMax"] = msgClassPtr_u_max->get_arr(factory, &currentElement_u_max, loader);
    // dt
    auto currentElement_dt = (msg + ctr)->dt;
    outArray[ctr]["Dt"] = factory.createScalar(currentElement_dt);
    // mass
    auto currentElement_mass = (msg + ctr)->mass;
    outArray[ctr]["Mass"] = factory.createScalar(currentElement_mass);
    // Q_pos_anc
    auto currentElement_Q_pos_anc = (msg + ctr)->Q_pos_anc;
    outArray[ctr]["QPosAnc"] = factory.createScalar(currentElement_Q_pos_anc);
    // Q_vel_anc
    auto currentElement_Q_vel_anc = (msg + ctr)->Q_vel_anc;
    outArray[ctr]["QVelAnc"] = factory.createScalar(currentElement_Q_vel_anc);
    // R_anc
    auto currentElement_R_anc = (msg + ctr)->R_anc;
    outArray[ctr]["RAnc"] = factory.createScalar(currentElement_R_anc);
    }
    return std::move(outArray);
  }
class RESWARM_MSGS_EXPORT reswarm_msgs_msg_ReswarmSrvMRPIResponse_common : public MATLABROSMsgInterface<reswarm_msgs::ReswarmSrvMRPI::Response> {
  public:
    virtual ~reswarm_msgs_msg_ReswarmSrvMRPIResponse_common(){}
    virtual void copy_from_struct(reswarm_msgs::ReswarmSrvMRPI::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmSrvMRPI::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_msg_ReswarmSrvMRPIResponse_common::copy_from_struct(reswarm_msgs::ReswarmSrvMRPI::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //K
        const matlab::data::StructArray K_arr = arr["K"];
        static auto msgClassPtr_K = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_K->copy_from_struct(&msg->K,K_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'K' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'K' is wrong type; expected a struct.");
    }
    try {
        //Au
        const matlab::data::StructArray Au_arr = arr["Au"];
        static auto msgClassPtr_Au = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_Au->copy_from_struct(&msg->Au,Au_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Au' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Au' is wrong type; expected a struct.");
    }
    try {
        //bu
        const matlab::data::StructArray bu_arr = arr["Bu"];
        static auto msgClassPtr_bu = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_bu->copy_from_struct(&msg->bu,bu_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Bu' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Bu' is wrong type; expected a struct.");
    }
    try {
        //AZ
        const matlab::data::StructArray AZ_arr = arr["AZ"];
        static auto msgClassPtr_AZ = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_AZ->copy_from_struct(&msg->AZ,AZ_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'AZ' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'AZ' is wrong type; expected a struct.");
    }
    try {
        //bZ
        const matlab::data::StructArray bZ_arr = arr["BZ"];
        static auto msgClassPtr_bZ = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_bZ->copy_from_struct(&msg->bZ,bZ_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BZ' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BZ' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_msg_ReswarmSrvMRPIResponse_common::get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmSrvMRPI::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","K","Au","Bu","AZ","BZ"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs/ReswarmSrvMRPIResponse");
    // K
    auto currentElement_K = (msg + ctr)->K;
    static auto msgClassPtr_K = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["K"] = msgClassPtr_K->get_arr(factory, &currentElement_K, loader);
    // Au
    auto currentElement_Au = (msg + ctr)->Au;
    static auto msgClassPtr_Au = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["Au"] = msgClassPtr_Au->get_arr(factory, &currentElement_Au, loader);
    // bu
    auto currentElement_bu = (msg + ctr)->bu;
    static auto msgClassPtr_bu = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["Bu"] = msgClassPtr_bu->get_arr(factory, &currentElement_bu, loader);
    // AZ
    auto currentElement_AZ = (msg + ctr)->AZ;
    static auto msgClassPtr_AZ = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["AZ"] = msgClassPtr_AZ->get_arr(factory, &currentElement_AZ, loader);
    // bZ
    auto currentElement_bZ = (msg + ctr)->bZ;
    static auto msgClassPtr_bZ = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["BZ"] = msgClassPtr_bZ->get_arr(factory, &currentElement_bZ, loader);
    }
    return std::move(outArray);
  } 
class RESWARM_MSGS_EXPORT reswarm_msgs_ReswarmSrvMRPI_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~reswarm_msgs_ReswarmSrvMRPI_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          reswarm_msgs_ReswarmSrvMRPI_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<reswarm_msgs::ReswarmSrvMRPI::Request,reswarm_msgs_msg_ReswarmSrvMRPIRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<reswarm_msgs::ReswarmSrvMRPI::Response,reswarm_msgs_msg_ReswarmSrvMRPIResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          reswarm_msgs_ReswarmSrvMRPI_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<reswarm_msgs::ReswarmSrvMRPI::Request,reswarm_msgs::ReswarmSrvMRPI::Request::ConstPtr,reswarm_msgs_msg_ReswarmSrvMRPIRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<reswarm_msgs::ReswarmSrvMRPI::Response,reswarm_msgs::ReswarmSrvMRPI::Response::ConstPtr,reswarm_msgs_msg_ReswarmSrvMRPIResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          reswarm_msgs_ReswarmSrvMRPI_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<reswarm_msgs::ReswarmSrvMRPI::Request,reswarm_msgs::ReswarmSrvMRPI::Response,reswarm_msgs_msg_ReswarmSrvMRPIRequest_common,reswarm_msgs_msg_ReswarmSrvMRPIResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          reswarm_msgs_ReswarmSrvMRPI_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<reswarm_msgs::ReswarmSrvMRPI,reswarm_msgs::ReswarmSrvMRPI::Request,reswarm_msgs::ReswarmSrvMRPI::Response,reswarm_msgs_msg_ReswarmSrvMRPIRequest_common,reswarm_msgs_msg_ReswarmSrvMRPIResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_msg_ReswarmSrvMRPIRequest_common, MATLABROSMsgInterface<reswarm_msgs::ReswarmSrvMRPI::Request>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_msg_ReswarmSrvMRPIResponse_common, MATLABROSMsgInterface<reswarm_msgs::ReswarmSrvMRPI::Response>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_ReswarmSrvMRPI_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
