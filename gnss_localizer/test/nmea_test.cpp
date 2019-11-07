#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>

class NmeaHelper
{
public:
  NmeaHelper() : rcvd_msg()
  {
  }

  void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    rcvd_msg = *msg;
    ROS_ERROR("Got message!");
  }

  geometry_msgs::PoseStamped rcvd_msg;
};

TEST(Nmea2TfPose, ggaTest)
{
  ros::NodeHandle nh;
  NmeaHelper nmeah;
  nmea_msgs::Sentence msg;
  ros::Subscriber sub = nh.subscribe("/test/gnss_pose", 1, &NmeaHelper::pose_callback, &nmeah);
  ros::Publisher pub = nh.advertise<nmea_msgs::Sentence>("/test/nmea_sentence", 1);

  // Give time to set up pub/sub
  ros::WallDuration(0.5).sleep();

  // Send a dummy message, because nodes only respond after they can compute orientation,
  // which requires having seen two different positions
  msg.sentence = "$GPGGA,0,80.000,N,90.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  // Start sending messages which are actually checked

  msg.sentence = "$GPGGA,0,90.000,N,90.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-81305263.475,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(5958827.35862,nmeah.rcvd_msg.pose.position.y);
  ASSERT_FLOAT_EQ(545.4,nmeah.rcvd_msg.pose.position.z);

  msg.sentence = "$GPGGA,0,90.000,N,90.000,W,1,08,0.9,545.4,M,46.9,M,,*47";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-90207689.8924,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(7519179.29268,nmeah.rcvd_msg.pose.position.y);
  ASSERT_FLOAT_EQ(545.4,nmeah.rcvd_msg.pose.position.z);

  msg.sentence = "$GPGGA,0,90.000,S,90.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-81305263.475,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(-13929115.5907,nmeah.rcvd_msg.pose.position.y);
  ASSERT_FLOAT_EQ(545.4,nmeah.rcvd_msg.pose.position.z);

  msg.sentence = "$GPGGA,0,90.000,S,90.000,W,1,08,0.9,545.4,M,46.9,M,,*47";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-90207689.8924,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(-15489467.5247,nmeah.rcvd_msg.pose.position.y);
  ASSERT_FLOAT_EQ(545.4,nmeah.rcvd_msg.pose.position.z);
}

TEST(Nmea2TfPose, rmcTest)
{
  ros::NodeHandle nh;
  NmeaHelper nmeah;
  nmea_msgs::Sentence msg;
  ros::Subscriber sub = nh.subscribe("/test/gnss_pose", 1, &NmeaHelper::pose_callback, &nmeah);
  ros::Publisher pub = nh.advertise<nmea_msgs::Sentence>("/test/nmea_sentence", 1);

  // Give time to set up pub/sub
  ros::WallDuration(0.5).sleep();

  // Send a dummy message, because nodes only respond after they can compute orientation,
  // which requires having seen two different positions
  msg.sentence = "$GPRMC,0,A,80.000,N,90.000,E,022.4,084.4,230394,003.1,W*6A";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  // Start sending messages which are actually checked

  msg.sentence = "$GPRMC,0,A,90.000,N,90.000,E,022.4,084.4,230394,003.1,W*6A";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-81305263.475,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(5958827.35862,nmeah.rcvd_msg.pose.position.y);

  msg.sentence = "$GPRMC,0,A,90.000,N,90.000,W,022.4,084.4,230394,003.1,W*6A";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-90207689.8924,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(7519179.29268,nmeah.rcvd_msg.pose.position.y);

  msg.sentence = "$GPRMC,0,A,90.000,S,90.000,E,022.4,084.4,230394,003.1,W*6A";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-81305263.475,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(-13929115.5907,nmeah.rcvd_msg.pose.position.y);

  msg.sentence = "$GPRMC,0,A,90.000,S,90.000,W,022.4,084.4,230394,003.1,W*6A";
  pub.publish(msg);

  // Give time to publish
  ros::WallDuration(0.5).sleep();

  ros::spinOnce();

  ASSERT_FLOAT_EQ(-90207689.8924,nmeah.rcvd_msg.pose.position.x);
  ASSERT_FLOAT_EQ(-15489467.5247,nmeah.rcvd_msg.pose.position.y);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "nmea_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
