#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ros_robot_controller_msgs__msg__MotorState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorState::default())
  }
}

impl rosidl_runtime_rs::Message for MotorState {
  type RmwMsg = super::msg::rmw::MotorState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        rps: msg.rps,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      rps: msg.rps,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      rps: msg.rps,
    }
  }
}


// Corresponds to ros_robot_controller_msgs__msg__MotorsState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorsState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub data: Vec<super::msg::MotorState>,

}



impl Default for MotorsState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorsState::default())
  }
}

impl rosidl_runtime_rs::Message for MotorsState {
  type RmwMsg = super::msg::rmw::MotorsState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data
          .into_iter()
          .map(|elem| super::msg::MotorState::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data
          .iter()
          .map(|elem| super::msg::MotorState::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data: msg.data
          .into_iter()
          .map(super::msg::MotorState::from_rmw_message)
          .collect(),
    }
  }
}


