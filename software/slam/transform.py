import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("tf2_frames_alignment")

    # Initialize TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Create the transform message
    transform_msg = geometry_msgs.msg.TransformStamped()
    transform_msg.header.frame_id = "camera"
    transform_msg.child_frame_id = "camera_link"

    # Set the translation and rotation
    transform_msg.transform.translation.x = 0.0  # change as needed
    transform_msg.transform.translation.y = 0.0
    transform_msg.transform.translation.z = 0.0
    transform_msg.transform.rotation.x = 0.0
    transform_msg.transform.rotation.y = 0.0
    transform_msg.transform.rotation.z = 0.0
    transform_msg.transform.rotation.w = 1.0

    # Publish the transform
    tf_broadcaster.sendTransform(transform_msg)

    # Spin the node
    rospy.spin()
