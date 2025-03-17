import rospy
import tf2_ros
import geometry_msgs.msg
import tf

def broadcast_static_transform():
    rospy.init_node('static_tf_broadcaster')

    # 创建 StaticTransformBroadcaster 对象
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # 创建 TransformStamped 对象
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "frame_b"  # 父坐标系
    transform.child_frame_id = "frame_a"   # 子坐标系

    # 设置平移部分
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0

    # 设置旋转部分（四元数）
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)  # 欧拉角转四元数
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]

    # 广播变换关系
    broadcaster.sendTransform(transform)
    rospy.spin()  # 静态广播只需发送一次，然后保持节点运行即可

if __name__ == "__main__":
    broadcast_static_transform()