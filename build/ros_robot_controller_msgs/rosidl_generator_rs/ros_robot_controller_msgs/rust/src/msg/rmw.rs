#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ros_robot_controller_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ros_robot_controller_msgs__msg__MotorState() -> *const std::ffi::c_void;
}

#[link(name = "ros_robot_controller_msgs__rosidl_generator_c")]
extern "C" {
    fn ros_robot_controller_msgs__msg__MotorState__init(msg: *mut MotorState) -> bool;
    fn ros_robot_controller_msgs__msg__MotorState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorState>, size: usize) -> bool;
    fn ros_robot_controller_msgs__msg__MotorState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorState>);
    fn ros_robot_controller_msgs__msg__MotorState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorState>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorState>) -> bool;
}

// Corresponds to ros_robot_controller_msgs__msg__MotorState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rps: f64,

}



impl Default for MotorState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ros_robot_controller_msgs__msg__MotorState__init(&mut msg as *mut _) {
        panic!("Call to ros_robot_controller_msgs__msg__MotorState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ros_robot_controller_msgs__msg__MotorState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ros_robot_controller_msgs__msg__MotorState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ros_robot_controller_msgs__msg__MotorState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorState where Self: Sized {
  const TYPE_NAME: &'static str = "ros_robot_controller_msgs/msg/MotorState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ros_robot_controller_msgs__msg__MotorState() }
  }
}


#[link(name = "ros_robot_controller_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ros_robot_controller_msgs__msg__MotorsState() -> *const std::ffi::c_void;
}

#[link(name = "ros_robot_controller_msgs__rosidl_generator_c")]
extern "C" {
    fn ros_robot_controller_msgs__msg__MotorsState__init(msg: *mut MotorsState) -> bool;
    fn ros_robot_controller_msgs__msg__MotorsState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorsState>, size: usize) -> bool;
    fn ros_robot_controller_msgs__msg__MotorsState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorsState>);
    fn ros_robot_controller_msgs__msg__MotorsState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorsState>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorsState>) -> bool;
}

// Corresponds to ros_robot_controller_msgs__msg__MotorsState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorsState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub data: rosidl_runtime_rs::Sequence<super::super::msg::rmw::MotorState>,

}



impl Default for MotorsState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ros_robot_controller_msgs__msg__MotorsState__init(&mut msg as *mut _) {
        panic!("Call to ros_robot_controller_msgs__msg__MotorsState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorsState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ros_robot_controller_msgs__msg__MotorsState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ros_robot_controller_msgs__msg__MotorsState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ros_robot_controller_msgs__msg__MotorsState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorsState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorsState where Self: Sized {
  const TYPE_NAME: &'static str = "ros_robot_controller_msgs/msg/MotorsState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ros_robot_controller_msgs__msg__MotorsState() }
  }
}


