// Auto-generated. Do not edit!

// (in-package yolov5_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class M {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.arr1 = null;
      this.arr2 = null;
      this.arr3 = null;
      this.arr4 = null;
      this.arr5 = null;
      this.num = null;
    }
    else {
      if (initObj.hasOwnProperty('arr1')) {
        this.arr1 = initObj.arr1
      }
      else {
        this.arr1 = [];
      }
      if (initObj.hasOwnProperty('arr2')) {
        this.arr2 = initObj.arr2
      }
      else {
        this.arr2 = [];
      }
      if (initObj.hasOwnProperty('arr3')) {
        this.arr3 = initObj.arr3
      }
      else {
        this.arr3 = [];
      }
      if (initObj.hasOwnProperty('arr4')) {
        this.arr4 = initObj.arr4
      }
      else {
        this.arr4 = [];
      }
      if (initObj.hasOwnProperty('arr5')) {
        this.arr5 = initObj.arr5
      }
      else {
        this.arr5 = [];
      }
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num
      }
      else {
        this.num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type M
    // Serialize message field [arr1]
    bufferOffset = _arraySerializer.int32(obj.arr1, buffer, bufferOffset, null);
    // Serialize message field [arr2]
    bufferOffset = _arraySerializer.int32(obj.arr2, buffer, bufferOffset, null);
    // Serialize message field [arr3]
    bufferOffset = _arraySerializer.int32(obj.arr3, buffer, bufferOffset, null);
    // Serialize message field [arr4]
    bufferOffset = _arraySerializer.int32(obj.arr4, buffer, bufferOffset, null);
    // Serialize message field [arr5]
    bufferOffset = _arraySerializer.int32(obj.arr5, buffer, bufferOffset, null);
    // Serialize message field [num]
    bufferOffset = _serializer.int32(obj.num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type M
    let len;
    let data = new M(null);
    // Deserialize message field [arr1]
    data.arr1 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [arr2]
    data.arr2 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [arr3]
    data.arr3 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [arr4]
    data.arr4 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [arr5]
    data.arr5 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [num]
    data.num = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.arr1.length;
    length += 4 * object.arr2.length;
    length += 4 * object.arr3.length;
    length += 4 * object.arr4.length;
    length += 4 * object.arr5.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolov5_ros_msgs/M';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02c60c121c0de351ab241d16cef9d248';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] arr1
    int32[] arr2
    int32[] arr3
    int32[] arr4
    int32[] arr5
    int32 num
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new M(null);
    if (msg.arr1 !== undefined) {
      resolved.arr1 = msg.arr1;
    }
    else {
      resolved.arr1 = []
    }

    if (msg.arr2 !== undefined) {
      resolved.arr2 = msg.arr2;
    }
    else {
      resolved.arr2 = []
    }

    if (msg.arr3 !== undefined) {
      resolved.arr3 = msg.arr3;
    }
    else {
      resolved.arr3 = []
    }

    if (msg.arr4 !== undefined) {
      resolved.arr4 = msg.arr4;
    }
    else {
      resolved.arr4 = []
    }

    if (msg.arr5 !== undefined) {
      resolved.arr5 = msg.arr5;
    }
    else {
      resolved.arr5 = []
    }

    if (msg.num !== undefined) {
      resolved.num = msg.num;
    }
    else {
      resolved.num = 0
    }

    return resolved;
    }
};

module.exports = M;
