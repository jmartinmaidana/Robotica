#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "robmovil_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__EncoderTicks() -> *const std::ffi::c_void;
}

#[link(name = "robmovil_msgs__rosidl_generator_c")]
extern "C" {
    fn robmovil_msgs__msg__EncoderTicks__init(msg: *mut EncoderTicks) -> bool;
    fn robmovil_msgs__msg__EncoderTicks__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EncoderTicks>, size: usize) -> bool;
    fn robmovil_msgs__msg__EncoderTicks__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EncoderTicks>);
    fn robmovil_msgs__msg__EncoderTicks__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EncoderTicks>, out_seq: *mut rosidl_runtime_rs::Sequence<EncoderTicks>) -> bool;
}

// Corresponds to robmovil_msgs__msg__EncoderTicks
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EncoderTicks {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ticks_left: std_msgs::msg::rmw::Int32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ticks_right: std_msgs::msg::rmw::Int32,

}



impl Default for EncoderTicks {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robmovil_msgs__msg__EncoderTicks__init(&mut msg as *mut _) {
        panic!("Call to robmovil_msgs__msg__EncoderTicks__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EncoderTicks {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__EncoderTicks__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__EncoderTicks__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__EncoderTicks__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EncoderTicks {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EncoderTicks where Self: Sized {
  const TYPE_NAME: &'static str = "robmovil_msgs/msg/EncoderTicks";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__EncoderTicks() }
  }
}


#[link(name = "robmovil_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__MultiEncoderTicks() -> *const std::ffi::c_void;
}

#[link(name = "robmovil_msgs__rosidl_generator_c")]
extern "C" {
    fn robmovil_msgs__msg__MultiEncoderTicks__init(msg: *mut MultiEncoderTicks) -> bool;
    fn robmovil_msgs__msg__MultiEncoderTicks__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MultiEncoderTicks>, size: usize) -> bool;
    fn robmovil_msgs__msg__MultiEncoderTicks__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MultiEncoderTicks>);
    fn robmovil_msgs__msg__MultiEncoderTicks__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MultiEncoderTicks>, out_seq: *mut rosidl_runtime_rs::Sequence<MultiEncoderTicks>) -> bool;
}

// Corresponds to robmovil_msgs__msg__MultiEncoderTicks
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiEncoderTicks {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Each array element correspond to one joint encoder information.
    pub ticks: rosidl_runtime_rs::Sequence<i32>,

}



impl Default for MultiEncoderTicks {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robmovil_msgs__msg__MultiEncoderTicks__init(&mut msg as *mut _) {
        panic!("Call to robmovil_msgs__msg__MultiEncoderTicks__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MultiEncoderTicks {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__MultiEncoderTicks__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__MultiEncoderTicks__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__MultiEncoderTicks__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MultiEncoderTicks {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MultiEncoderTicks where Self: Sized {
  const TYPE_NAME: &'static str = "robmovil_msgs/msg/MultiEncoderTicks";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__MultiEncoderTicks() }
  }
}


#[link(name = "robmovil_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__Trajectory() -> *const std::ffi::c_void;
}

#[link(name = "robmovil_msgs__rosidl_generator_c")]
extern "C" {
    fn robmovil_msgs__msg__Trajectory__init(msg: *mut Trajectory) -> bool;
    fn robmovil_msgs__msg__Trajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Trajectory>, size: usize) -> bool;
    fn robmovil_msgs__msg__Trajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Trajectory>);
    fn robmovil_msgs__msg__Trajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Trajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<Trajectory>) -> bool;
}

// Corresponds to robmovil_msgs__msg__Trajectory
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// The header is used to specify the coordinate frame
/// and the reference time for the trajectory durations.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Trajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// List of Trajectory waypoints.
    pub points: rosidl_runtime_rs::Sequence<super::super::msg::rmw::TrajectoryPoint>,

}



impl Default for Trajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robmovil_msgs__msg__Trajectory__init(&mut msg as *mut _) {
        panic!("Call to robmovil_msgs__msg__Trajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Trajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__Trajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__Trajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__Trajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Trajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Trajectory where Self: Sized {
  const TYPE_NAME: &'static str = "robmovil_msgs/msg/Trajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__Trajectory() }
  }
}


#[link(name = "robmovil_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__TrajectoryPoint() -> *const std::ffi::c_void;
}

#[link(name = "robmovil_msgs__rosidl_generator_c")]
extern "C" {
    fn robmovil_msgs__msg__TrajectoryPoint__init(msg: *mut TrajectoryPoint) -> bool;
    fn robmovil_msgs__msg__TrajectoryPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrajectoryPoint>, size: usize) -> bool;
    fn robmovil_msgs__msg__TrajectoryPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrajectoryPoint>);
    fn robmovil_msgs__msg__TrajectoryPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrajectoryPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<TrajectoryPoint>) -> bool;
}

// Corresponds to robmovil_msgs__msg__TrajectoryPoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Each multi-dof joint can specify a transform (up to 6 DOF)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrajectoryPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub transform: geometry_msgs::msg::rmw::Transform,

    /// There can be a velocity specified for the origin of the joint
    pub velocity: geometry_msgs::msg::rmw::Twist,

    /// There can be an acceleration specified for the origin of the joint
    pub acceleration: geometry_msgs::msg::rmw::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_from_start: builtin_interfaces::msg::rmw::Duration,

}



impl Default for TrajectoryPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robmovil_msgs__msg__TrajectoryPoint__init(&mut msg as *mut _) {
        panic!("Call to robmovil_msgs__msg__TrajectoryPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrajectoryPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__TrajectoryPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__TrajectoryPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__TrajectoryPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrajectoryPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrajectoryPoint where Self: Sized {
  const TYPE_NAME: &'static str = "robmovil_msgs/msg/TrajectoryPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__TrajectoryPoint() }
  }
}


#[link(name = "robmovil_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__Landmark() -> *const std::ffi::c_void;
}

#[link(name = "robmovil_msgs__rosidl_generator_c")]
extern "C" {
    fn robmovil_msgs__msg__Landmark__init(msg: *mut Landmark) -> bool;
    fn robmovil_msgs__msg__Landmark__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Landmark>, size: usize) -> bool;
    fn robmovil_msgs__msg__Landmark__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Landmark>);
    fn robmovil_msgs__msg__Landmark__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Landmark>, out_seq: *mut rosidl_runtime_rs::Sequence<Landmark>) -> bool;
}

// Corresponds to robmovil_msgs__msg__Landmark
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Landmark {

    // This member is not documented.
    #[allow(missing_docs)]
    pub range: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bearing: f32,

}



impl Default for Landmark {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robmovil_msgs__msg__Landmark__init(&mut msg as *mut _) {
        panic!("Call to robmovil_msgs__msg__Landmark__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Landmark {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__Landmark__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__Landmark__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__Landmark__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Landmark {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Landmark where Self: Sized {
  const TYPE_NAME: &'static str = "robmovil_msgs/msg/Landmark";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__Landmark() }
  }
}


#[link(name = "robmovil_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__LandmarkArray() -> *const std::ffi::c_void;
}

#[link(name = "robmovil_msgs__rosidl_generator_c")]
extern "C" {
    fn robmovil_msgs__msg__LandmarkArray__init(msg: *mut LandmarkArray) -> bool;
    fn robmovil_msgs__msg__LandmarkArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LandmarkArray>, size: usize) -> bool;
    fn robmovil_msgs__msg__LandmarkArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LandmarkArray>);
    fn robmovil_msgs__msg__LandmarkArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LandmarkArray>, out_seq: *mut rosidl_runtime_rs::Sequence<LandmarkArray>) -> bool;
}

// Corresponds to robmovil_msgs__msg__LandmarkArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LandmarkArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub landmarks: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Landmark>,

}



impl Default for LandmarkArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robmovil_msgs__msg__LandmarkArray__init(&mut msg as *mut _) {
        panic!("Call to robmovil_msgs__msg__LandmarkArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LandmarkArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__LandmarkArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__LandmarkArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robmovil_msgs__msg__LandmarkArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LandmarkArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LandmarkArray where Self: Sized {
  const TYPE_NAME: &'static str = "robmovil_msgs/msg/LandmarkArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robmovil_msgs__msg__LandmarkArray() }
  }
}


