// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for reswarm_msgs/ReswarmCasadiDebug
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
#include "reswarm_msgs/ReswarmCasadiDebug.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RESWARM_MSGS_EXPORT reswarm_msgs_msg_ReswarmCasadiDebug_common : public MATLABROSMsgInterface<reswarm_msgs::ReswarmCasadiDebug> {
  public:
    virtual ~reswarm_msgs_msg_ReswarmCasadiDebug_common(){}
    virtual void copy_from_struct(reswarm_msgs::ReswarmCasadiDebug* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmCasadiDebug* msg, MultiLibLoader loader, size_t size = 1);
};
  void reswarm_msgs_msg_ReswarmCasadiDebug_common::copy_from_struct(reswarm_msgs::ReswarmCasadiDebug* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["Header"];
        static auto msgClassPtr_header = loader->createInstance<MATLABROSMsgInterface<std_msgs::Header>>("std_msgs_msg_Header_common");
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Header' is wrong type; expected a struct.");
    }
    try {
        //wrench
        const matlab::data::StructArray wrench_arr = arr["Wrench"];
        static auto msgClassPtr_wrench = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Wrench>>("geometry_msgs_msg_Wrench_common");
        msgClassPtr_wrench->copy_from_struct(&msg->wrench,wrench_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Wrench' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Wrench' is wrong type; expected a struct.");
    }
    try {
        //u0_mpc
        const matlab::data::StructArray u0_mpc_arr = arr["U0Mpc"];
        static auto msgClassPtr_u0_mpc = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
        msgClassPtr_u0_mpc->copy_from_struct(&msg->u0_mpc,u0_mpc_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'U0Mpc' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'U0Mpc' is wrong type; expected a struct.");
    }
    try {
        //u0_dr
        const matlab::data::StructArray u0_dr_arr = arr["U0Dr"];
        static auto msgClassPtr_u0_dr = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
        msgClassPtr_u0_dr->copy_from_struct(&msg->u0_dr,u0_dr_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'U0Dr' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'U0Dr' is wrong type; expected a struct.");
    }
    try {
        //x_nom
        const matlab::data::StructArray x_nom_arr = arr["XNom"];
        static auto msgClassPtr_x_nom = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_x_nom->copy_from_struct(&msg->x_nom,x_nom_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'XNom' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'XNom' is wrong type; expected a struct.");
    }
    try {
        //accel
        const matlab::data::StructArray accel_arr = arr["Accel"];
        static auto msgClassPtr_accel = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Vector3>>("geometry_msgs_msg_Vector3_common");
        msgClassPtr_accel->copy_from_struct(&msg->accel,accel_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Accel' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Accel' is wrong type; expected a struct.");
    }
    try {
        //casadi_comp_time
        const matlab::data::StructArray casadi_comp_time_arr = arr["CasadiCompTime"];
        static auto msgClassPtr_casadi_comp_time = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64>>("std_msgs_msg_Float64_common");
        msgClassPtr_casadi_comp_time->copy_from_struct(&msg->casadi_comp_time,casadi_comp_time_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'CasadiCompTime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'CasadiCompTime' is wrong type; expected a struct.");
    }
    try {
        //total_comp_time
        const matlab::data::StructArray total_comp_time_arr = arr["TotalCompTime"];
        static auto msgClassPtr_total_comp_time = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64>>("std_msgs_msg_Float64_common");
        msgClassPtr_total_comp_time->copy_from_struct(&msg->total_comp_time,total_comp_time_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TotalCompTime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TotalCompTime' is wrong type; expected a struct.");
    }
    try {
        //control_mode
        const matlab::data::StructArray control_mode_arr = arr["ControlMode"];
        static auto msgClassPtr_control_mode = loader->createInstance<MATLABROSMsgInterface<std_msgs::String>>("std_msgs_msg_String_common");
        msgClassPtr_control_mode->copy_from_struct(&msg->control_mode,control_mode_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ControlMode' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ControlMode' is wrong type; expected a struct.");
    }
    try {
        //state_mode
        const matlab::data::StructArray state_mode_arr = arr["StateMode"];
        static auto msgClassPtr_state_mode = loader->createInstance<MATLABROSMsgInterface<std_msgs::String>>("std_msgs_msg_String_common");
        msgClassPtr_state_mode->copy_from_struct(&msg->state_mode,state_mode_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'StateMode' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'StateMode' is wrong type; expected a struct.");
    }
    try {
        //Q_pos_factor
        const matlab::data::TypedArray<double> Q_pos_factor_arr = arr["QPosFactor"];
        msg->Q_pos_factor = Q_pos_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QPosFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QPosFactor' is wrong type; expected a double.");
    }
    try {
        //Q_vel_factor
        const matlab::data::TypedArray<double> Q_vel_factor_arr = arr["QVelFactor"];
        msg->Q_vel_factor = Q_vel_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QVelFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QVelFactor' is wrong type; expected a double.");
    }
    try {
        //R_factor
        const matlab::data::TypedArray<double> R_factor_arr = arr["RFactor"];
        msg->R_factor = R_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RFactor' is wrong type; expected a double.");
    }
    try {
        //QN_pos_factor
        const matlab::data::TypedArray<double> QN_pos_factor_arr = arr["QNPosFactor"];
        msg->QN_pos_factor = QN_pos_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QNPosFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QNPosFactor' is wrong type; expected a double.");
    }
    try {
        //QN_vel_factor
        const matlab::data::TypedArray<double> QN_vel_factor_arr = arr["QNVelFactor"];
        msg->QN_vel_factor = QN_vel_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QNVelFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QNVelFactor' is wrong type; expected a double.");
    }
    try {
        //Q_pos_tube_factor
        const matlab::data::TypedArray<double> Q_pos_tube_factor_arr = arr["QPosTubeFactor"];
        msg->Q_pos_tube_factor = Q_pos_tube_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QPosTubeFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QPosTubeFactor' is wrong type; expected a double.");
    }
    try {
        //Q_vel_tube_factor
        const matlab::data::TypedArray<double> Q_vel_tube_factor_arr = arr["QVelTubeFactor"];
        msg->Q_vel_tube_factor = Q_vel_tube_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QVelTubeFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QVelTubeFactor' is wrong type; expected a double.");
    }
    try {
        //R_tube_factor
        const matlab::data::TypedArray<double> R_tube_factor_arr = arr["RTubeFactor"];
        msg->R_tube_factor = R_tube_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RTubeFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RTubeFactor' is wrong type; expected a double.");
    }
    try {
        //QN_pos_tube_factor
        const matlab::data::TypedArray<double> QN_pos_tube_factor_arr = arr["QNPosTubeFactor"];
        msg->QN_pos_tube_factor = QN_pos_tube_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QNPosTubeFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QNPosTubeFactor' is wrong type; expected a double.");
    }
    try {
        //QN_vel_tube_factor
        const matlab::data::TypedArray<double> QN_vel_tube_factor_arr = arr["QNVelTubeFactor"];
        msg->QN_vel_tube_factor = QN_vel_tube_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QNVelTubeFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QNVelTubeFactor' is wrong type; expected a double.");
    }
    try {
        //Q_pos_anc_factor
        const matlab::data::TypedArray<double> Q_pos_anc_factor_arr = arr["QPosAncFactor"];
        msg->Q_pos_anc_factor = Q_pos_anc_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QPosAncFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QPosAncFactor' is wrong type; expected a double.");
    }
    try {
        //Q_vel_anc_factor
        const matlab::data::TypedArray<double> Q_vel_anc_factor_arr = arr["QVelAncFactor"];
        msg->Q_vel_anc_factor = Q_vel_anc_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QVelAncFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QVelAncFactor' is wrong type; expected a double.");
    }
    try {
        //R_anc_factor
        const matlab::data::TypedArray<double> R_anc_factor_arr = arr["RAncFactor"];
        msg->R_anc_factor = R_anc_factor_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RAncFactor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RAncFactor' is wrong type; expected a double.");
    }
    try {
        //T
        const matlab::data::TypedArray<double> T_arr = arr["T"];
        msg->T = T_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'T' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'T' is wrong type; expected a double.");
    }
    try {
        //N
        const matlab::data::TypedArray<int32_t> N_arr = arr["N"];
        msg->N = N_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'N' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'N' is wrong type; expected a int32.");
    }
    try {
        //control_dt
        const matlab::data::TypedArray<double> control_dt_arr = arr["ControlDt"];
        msg->control_dt = control_dt_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ControlDt' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ControlDt' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T reswarm_msgs_msg_ReswarmCasadiDebug_common::get_arr(MDFactory_T& factory, const reswarm_msgs::ReswarmCasadiDebug* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Wrench","U0Mpc","U0Dr","XNom","Accel","CasadiCompTime","TotalCompTime","ControlMode","StateMode","QPosFactor","QVelFactor","RFactor","QNPosFactor","QNVelFactor","QPosTubeFactor","QVelTubeFactor","RTubeFactor","QNPosTubeFactor","QNVelTubeFactor","QPosAncFactor","QVelAncFactor","RAncFactor","T","N","ControlDt"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("reswarm_msgs/ReswarmCasadiDebug");
    // header
    auto currentElement_header = (msg + ctr)->header;
    static auto msgClassPtr_header = loader->createInstance<MATLABROSMsgInterface<std_msgs::Header>>("std_msgs_msg_Header_common");
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // wrench
    auto currentElement_wrench = (msg + ctr)->wrench;
    static auto msgClassPtr_wrench = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Wrench>>("geometry_msgs_msg_Wrench_common");
    outArray[ctr]["Wrench"] = msgClassPtr_wrench->get_arr(factory, &currentElement_wrench, loader);
    // u0_mpc
    auto currentElement_u0_mpc = (msg + ctr)->u0_mpc;
    static auto msgClassPtr_u0_mpc = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
    outArray[ctr]["U0Mpc"] = msgClassPtr_u0_mpc->get_arr(factory, &currentElement_u0_mpc, loader);
    // u0_dr
    auto currentElement_u0_dr = (msg + ctr)->u0_dr;
    static auto msgClassPtr_u0_dr = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
    outArray[ctr]["U0Dr"] = msgClassPtr_u0_dr->get_arr(factory, &currentElement_u0_dr, loader);
    // x_nom
    auto currentElement_x_nom = (msg + ctr)->x_nom;
    static auto msgClassPtr_x_nom = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["XNom"] = msgClassPtr_x_nom->get_arr(factory, &currentElement_x_nom, loader);
    // accel
    auto currentElement_accel = (msg + ctr)->accel;
    static auto msgClassPtr_accel = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Vector3>>("geometry_msgs_msg_Vector3_common");
    outArray[ctr]["Accel"] = msgClassPtr_accel->get_arr(factory, &currentElement_accel, loader);
    // casadi_comp_time
    auto currentElement_casadi_comp_time = (msg + ctr)->casadi_comp_time;
    static auto msgClassPtr_casadi_comp_time = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64>>("std_msgs_msg_Float64_common");
    outArray[ctr]["CasadiCompTime"] = msgClassPtr_casadi_comp_time->get_arr(factory, &currentElement_casadi_comp_time, loader);
    // total_comp_time
    auto currentElement_total_comp_time = (msg + ctr)->total_comp_time;
    static auto msgClassPtr_total_comp_time = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64>>("std_msgs_msg_Float64_common");
    outArray[ctr]["TotalCompTime"] = msgClassPtr_total_comp_time->get_arr(factory, &currentElement_total_comp_time, loader);
    // control_mode
    auto currentElement_control_mode = (msg + ctr)->control_mode;
    static auto msgClassPtr_control_mode = loader->createInstance<MATLABROSMsgInterface<std_msgs::String>>("std_msgs_msg_String_common");
    outArray[ctr]["ControlMode"] = msgClassPtr_control_mode->get_arr(factory, &currentElement_control_mode, loader);
    // state_mode
    auto currentElement_state_mode = (msg + ctr)->state_mode;
    static auto msgClassPtr_state_mode = loader->createInstance<MATLABROSMsgInterface<std_msgs::String>>("std_msgs_msg_String_common");
    outArray[ctr]["StateMode"] = msgClassPtr_state_mode->get_arr(factory, &currentElement_state_mode, loader);
    // Q_pos_factor
    auto currentElement_Q_pos_factor = (msg + ctr)->Q_pos_factor;
    outArray[ctr]["QPosFactor"] = factory.createScalar(currentElement_Q_pos_factor);
    // Q_vel_factor
    auto currentElement_Q_vel_factor = (msg + ctr)->Q_vel_factor;
    outArray[ctr]["QVelFactor"] = factory.createScalar(currentElement_Q_vel_factor);
    // R_factor
    auto currentElement_R_factor = (msg + ctr)->R_factor;
    outArray[ctr]["RFactor"] = factory.createScalar(currentElement_R_factor);
    // QN_pos_factor
    auto currentElement_QN_pos_factor = (msg + ctr)->QN_pos_factor;
    outArray[ctr]["QNPosFactor"] = factory.createScalar(currentElement_QN_pos_factor);
    // QN_vel_factor
    auto currentElement_QN_vel_factor = (msg + ctr)->QN_vel_factor;
    outArray[ctr]["QNVelFactor"] = factory.createScalar(currentElement_QN_vel_factor);
    // Q_pos_tube_factor
    auto currentElement_Q_pos_tube_factor = (msg + ctr)->Q_pos_tube_factor;
    outArray[ctr]["QPosTubeFactor"] = factory.createScalar(currentElement_Q_pos_tube_factor);
    // Q_vel_tube_factor
    auto currentElement_Q_vel_tube_factor = (msg + ctr)->Q_vel_tube_factor;
    outArray[ctr]["QVelTubeFactor"] = factory.createScalar(currentElement_Q_vel_tube_factor);
    // R_tube_factor
    auto currentElement_R_tube_factor = (msg + ctr)->R_tube_factor;
    outArray[ctr]["RTubeFactor"] = factory.createScalar(currentElement_R_tube_factor);
    // QN_pos_tube_factor
    auto currentElement_QN_pos_tube_factor = (msg + ctr)->QN_pos_tube_factor;
    outArray[ctr]["QNPosTubeFactor"] = factory.createScalar(currentElement_QN_pos_tube_factor);
    // QN_vel_tube_factor
    auto currentElement_QN_vel_tube_factor = (msg + ctr)->QN_vel_tube_factor;
    outArray[ctr]["QNVelTubeFactor"] = factory.createScalar(currentElement_QN_vel_tube_factor);
    // Q_pos_anc_factor
    auto currentElement_Q_pos_anc_factor = (msg + ctr)->Q_pos_anc_factor;
    outArray[ctr]["QPosAncFactor"] = factory.createScalar(currentElement_Q_pos_anc_factor);
    // Q_vel_anc_factor
    auto currentElement_Q_vel_anc_factor = (msg + ctr)->Q_vel_anc_factor;
    outArray[ctr]["QVelAncFactor"] = factory.createScalar(currentElement_Q_vel_anc_factor);
    // R_anc_factor
    auto currentElement_R_anc_factor = (msg + ctr)->R_anc_factor;
    outArray[ctr]["RAncFactor"] = factory.createScalar(currentElement_R_anc_factor);
    // T
    auto currentElement_T = (msg + ctr)->T;
    outArray[ctr]["T"] = factory.createScalar(currentElement_T);
    // N
    auto currentElement_N = (msg + ctr)->N;
    outArray[ctr]["N"] = factory.createScalar(currentElement_N);
    // control_dt
    auto currentElement_control_dt = (msg + ctr)->control_dt;
    outArray[ctr]["ControlDt"] = factory.createScalar(currentElement_control_dt);
    }
    return std::move(outArray);
  } 
class RESWARM_MSGS_EXPORT reswarm_msgs_ReswarmCasadiDebug_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~reswarm_msgs_ReswarmCasadiDebug_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          reswarm_msgs_ReswarmCasadiDebug_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<reswarm_msgs::ReswarmCasadiDebug,reswarm_msgs_msg_ReswarmCasadiDebug_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         reswarm_msgs_ReswarmCasadiDebug_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<reswarm_msgs::ReswarmCasadiDebug,reswarm_msgs::ReswarmCasadiDebug::ConstPtr,reswarm_msgs_msg_ReswarmCasadiDebug_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_msg_ReswarmCasadiDebug_common, MATLABROSMsgInterface<reswarm_msgs::ReswarmCasadiDebug>)
CLASS_LOADER_REGISTER_CLASS(reswarm_msgs_ReswarmCasadiDebug_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1