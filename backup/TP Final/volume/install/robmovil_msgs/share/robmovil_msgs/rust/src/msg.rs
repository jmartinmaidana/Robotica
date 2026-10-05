#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to robmovil_msgs__msg__EncoderTicks

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EncoderTicks {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ticks_left: std_msgs::msg::Int32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ticks_right: std_msgs::msg::Int32,

}



impl Default for EncoderTicks {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EncoderTicks::default())
  }
}

impl rosidl_runtime_rs::Message for EncoderTicks {
  type RmwMsg = super::msg::rmw::EncoderTicks;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        ticks_left: std_msgs::msg::Int32::into_rmw_message(std::borrow::Cow::Owned(msg.ticks_left)).into_owned(),
        ticks_right: std_msgs::msg::Int32::into_rmw_message(std::borrow::Cow::Owned(msg.ticks_right)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        ticks_left: std_msgs::msg::Int32::into_rmw_message(std::borrow::Cow::Borrowed(&msg.ticks_left)).into_owned(),
        ticks_right: std_msgs::msg::Int32::into_rmw_message(std::borrow::Cow::Borrowed(&msg.ticks_right)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      ticks_left: std_msgs::msg::Int32::from_rmw_message(msg.ticks_left),
      ticks_right: std_msgs::msg::Int32::from_rmw_message(msg.ticks_right),
    }
  }
}


// Corresponds to robmovil_msgs__msg__MultiEncoderTicks

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiEncoderTicks {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Each array element correspond to one joint encoder information.
    pub ticks: Vec<i32>,

}



impl Default for MultiEncoderTicks {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MultiEncoderTicks::default())
  }
}

impl rosidl_runtime_rs::Message for MultiEncoderTicks {
  type RmwMsg = super::msg::rmw::MultiEncoderTicks;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        ticks: msg.ticks.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        ticks: msg.ticks.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      ticks: msg.ticks
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to robmovil_msgs__msg__Trajectory
/// The header is used to specify the coordinate frame
/// and the reference time for the trajectory durations.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Trajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// List of Trajectory waypoints.
    pub points: Vec<super::msg::TrajectoryPoint>,

}



impl Default for Trajectory {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Trajectory::default())
  }
}

impl rosidl_runtime_rs::Message for Trajectory {
  type RmwMsg = super::msg::rmw::Trajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| super::msg::TrajectoryPoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| super::msg::TrajectoryPoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      points: msg.points
          .into_iter()
          .map(super::msg::TrajectoryPoint::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to robmovil_msgs__msg__TrajectoryPoint
/// Each multi-dof joint can specify a transform (up to 6 DOF)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrajectoryPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub transform: geometry_msgs::msg::Transform,

    /// There can be a velocity specified for the origin of the joint
    pub velocity: geometry_msgs::msg::Twist,

    /// There can be an acceleration specified for the origin of the joint
    pub acceleration: geometry_msgs::msg::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_from_start: builtin_interfaces::msg::Duration,

}



impl Default for TrajectoryPoint {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrajectoryPoint::default())
  }
}

impl rosidl_runtime_rs::Message for TrajectoryPoint {
  type RmwMsg = super::msg::rmw::TrajectoryPoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        transform: geometry_msgs::msg::Transform::into_rmw_message(std::borrow::Cow::Owned(msg.transform)).into_owned(),
        velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.acceleration)).into_owned(),
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_from_start)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        transform: geometry_msgs::msg::Transform::into_rmw_message(std::borrow::Cow::Borrowed(&msg.transform)).into_owned(),
        velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.acceleration)).into_owned(),
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_from_start)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      transform: geometry_msgs::msg::Transform::from_rmw_message(msg.transform),
      velocity: geometry_msgs::msg::Twist::from_rmw_message(msg.velocity),
      acceleration: geometry_msgs::msg::Twist::from_rmw_message(msg.acceleration),
      time_from_start: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_from_start),
    }
  }
}


// Corresponds to robmovil_msgs__msg__Landmark

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Landmark::default())
  }
}

impl rosidl_runtime_rs::Message for Landmark {
  type RmwMsg = super::msg::rmw::Landmark;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        range: msg.range,
        bearing: msg.bearing,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      range: msg.range,
      bearing: msg.bearing,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      range: msg.range,
      bearing: msg.bearing,
    }
  }
}


// Corresponds to robmovil_msgs__msg__LandmarkArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LandmarkArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub landmarks: Vec<super::msg::Landmark>,

}



impl Default for LandmarkArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LandmarkArray::default())
  }
}

impl rosidl_runtime_rs::Message for LandmarkArray {
  type RmwMsg = super::msg::rmw::LandmarkArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        landmarks: msg.landmarks
          .into_iter()
          .map(|elem| super::msg::Landmark::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        landmarks: msg.landmarks
          .iter()
          .map(|elem| super::msg::Landmark::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      landmarks: msg.landmarks
          .into_iter()
          .map(super::msg::Landmark::from_rmw_message)
          .collect(),
    }
  }
}


