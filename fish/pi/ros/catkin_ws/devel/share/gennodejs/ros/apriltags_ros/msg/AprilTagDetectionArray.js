// Auto-generated. Do not edit!

// (in-package apriltags_ros.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let AprilTagDetection = require('./AprilTagDetection.js');

//-----------------------------------------------------------

class AprilTagDetectionArray {
  constructor() {
    this.detections = [];
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type AprilTagDetectionArray
    // Serialize the length for message field [detections]
    bufferInfo = _serializer.uint32(obj.detections.length, bufferInfo);
    // Serialize message field [detections]
    obj.detections.forEach((val) => {
      bufferInfo = AprilTagDetection.serialize(val, bufferInfo);
    });
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type AprilTagDetectionArray
    let tmp;
    let len;
    let data = new AprilTagDetectionArray();
    // Deserialize array length for message field [detections]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [detections]
    data.detections = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = AprilTagDetection.deserialize(buffer);
      data.detections[i] = tmp.data;
      buffer = tmp.buffer;
    }
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'apriltags_ros/AprilTagDetectionArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93c0a301ed9e6633dc34b8117d49ebd4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    AprilTagDetection[] detections
    ================================================================================
    MSG: apriltags_ros/AprilTagDetection
    int32 id
    float64 size
    geometry_msgs/PoseStamped pose
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

};

module.exports = AprilTagDetectionArray;
