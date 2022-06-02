// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for reswarm_msgs/ReswarmMsgMRPI
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
#include "reswarm_msgs/ReswarmMsgMRPI.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RESWARM_MSGS_EXPORT reswarm_msgs_msg_ReswarmMsgMRPI_common : public MATLABROSMsgInterface<reswarm_msgs::ReswarmMsgMRPI> {
  public:
    virtual ~reswarm_msgs_msg_ReswarmMsgMRPI_common(){}
    virtual void copy_from_struct(reswarm_msgs::ReswarmMsgMRPI* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmMsgMRPI* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_msg_ReswarmMsgMRPI_common::copy_from_struct(reswarm_msgs::ReswarmMsgMRPI* msg, const matlab::data::Struct& arr,
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
    try {
        //using_fallback_mrpi
        const matlab::data::StructArray using_fallback_mrpi_arr = arr["UsingFallbackMrpi"];
        static auto msgClassPtr_using_fallback_mrpi = loader->createInstance<MATLABROSMsgInterface<std_msgs::Bool>>("std_msgs_msg_Bool_common");
        msgClassPtr_using_fallback_mrpi->copy_from_struct(&msg->using_fallback_mrpi,using_fallback_mrpi_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'UsingFallbackMrpi' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'UsingFallbackMrpi' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_msg_ReswarmMsgMRPI_common::get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmMsgMRPI* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","K","Au","Bu","AZ","BZ","UsingFallbackMrpi"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs/ReswarmMsgMRPI");
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
    // using_fallback_mrpi
    auto currentElement_using_fallback_mrpi = (msg + ctr)->using_fallback_mrpi;
    static auto msgClassPtr_using_fallback_mrpi = loader->createInstance<MATLABROSMsgInterface<std_msgs::Bool>>("std_msgs_msg_Bool_common");
    outArray[ctr]["UsingFallbackMrpi"] = msgClassPtr_using_fallback_mrpi->get_arr(factory, &currentElement_using_fallback_mrpi, loader);
    }
    return std::move(outArray);
  } 
class RESWARM_MSGS_EXPORT reswarm_msgs_ReswarmMsgMRPI_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~reswarm_msgs_ReswarmMsgMRPI_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          reswarm_msgs_ReswarmMsgMRPI_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<reswarm_msgs::ReswarmMsgMRPI,reswarm_msgs_msg_ReswarmMsgMRPI_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         reswarm_msgs_ReswarmMsgMRPI_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<reswarm_msgs::ReswarmMsgMRPI,reswarm_msgs::ReswarmMsgMRPI::ConstPtr,reswarm_msgs_msg_ReswarmMsgMRPI_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_msg_ReswarmMsgMRPI_common, MATLABROSMsgInterface<reswarm_msgs::ReswarmMsgMRPI>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_ReswarmMsgMRPI_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1