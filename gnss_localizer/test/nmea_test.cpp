#include <cmath>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>

#define DUMMY_NMEA_SENTENCE_0                                                  \
  "$GPGGA,0,80.000,N,90.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
#define DUMMY_NMEA_SENTENCE_1                                                  \
  "$GPGGA,0,81.000,N,90.000,E,1,08,0.9,545.4,M,46.9,M,,*47"

class NmeaHelper {
public:
  NmeaHelper() = default;

  // Initialise the subscriber and publisher. Wait for connection before
  // continuing.
  void init() {
    sub = nh.subscribe("/test/gnss_pose", 1, &NmeaHelper::pose_callback, this);
    pub = nh.advertise<nmea_msgs::Sentence>("/test/nmea_sentence", 1, true);
    auto timeout = wait_for_connection();
    ASSERT_FALSE(timeout);
  }

  // run test case. Send nmea sentence on published, wait for message on
  // subscriber. Assert on the value of the returned message.
  void test_case(std::string nmea_sentence, float expected_x, float expected_y,
                 float expected_z = NAN) {
    reset();

    nmea_msgs::Sentence msg;
    msg.sentence = nmea_sentence.c_str();
    pub.publish(msg);

    auto timeout = wait_for_new_msg();

    ASSERT_FALSE(timeout);
    ASSERT_FLOAT_EQ(expected_x, rcvd_msg.pose.position.x);
    ASSERT_FLOAT_EQ(expected_y, rcvd_msg.pose.position.y);

    if (!std::isnan(expected_z)) {
      ASSERT_FLOAT_EQ(expected_z, rcvd_msg.pose.position.z);
    }
  }

private:
  // The callback invoked when a message is received by the subscriber.
  void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    rcvd_msg = *msg;
    new_msg = true;
  }

  // Reset the new message flag
  void reset() { new_msg = false; }

  // Wait for message to arrive at the subscriber. return true if timed out
  // without receiving any message, return false otherwise
  bool wait_for_new_msg() {
    int loops = static_cast<int>(ceil(timeout_s / delay_s));

    for (int i = 0; i < loops && new_msg == false; ++i) {
      ros::WallDuration(delay_s).sleep();
      ros::spinOnce();
    }

    auto timed_out = !new_msg;
    return timed_out;
  }

  // Send some initial messages to setup the node under test. wait for the first
  // message received on the subscriber before continuing. return true if timed
  // out without receiving any message, return false otherwise.
  bool wait_for_connection() {
    int loops = static_cast<int>(ceil(timeout_s / delay_s));

    nmea_msgs::Sentence msg;
    reset();

    // nmea2tfpose node requires 2 different initial gps readings before being
    // able to output the correct transforms. Before the connection between
    // nodes are established, the messages sent by the publisher are prone to
    // get lost. 2 initial dummy gps sentences are sent alternatively until the
    // node under test responds.
    for (int i = 0; i < loops && new_msg == false; ++i) {
      msg.sentence = i % 2 ? DUMMY_NMEA_SENTENCE_0 : DUMMY_NMEA_SENTENCE_1;
      pub.publish(msg);
      ros::WallDuration(delay_s).sleep();
      ros::spinOnce();
    }

    auto timed_out = !new_msg;
    return timed_out;
  }

  geometry_msgs::PoseStamped rcvd_msg; // message received from subscriber
  std::atomic<bool> new_msg{false}; // flag set when a new message is received
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  const float timeout_s{5}; // timeout in seconds
  const float delay_s{0.1}; // delay between calling spinOnce in wait loop
};

TEST(Nmea2TfPose, ggaTest) {
  NmeaHelper nmeah{};
  nmeah.init();

  nmeah.test_case("$GPGGA,0,90.000,N,90.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
                  -81305263.475, 5958827.35862, 545.4);
  nmeah.test_case("$GPGGA,0,90.000,N,90.000,W,1,08,0.9,545.4,M,46.9,M,,*47",
                  -90207689.8924, 7519179.29268, 545.4);
  nmeah.test_case("$GPGGA,0,90.000,S,90.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
                  -81305263.475, -13929115.5907, 545.4);
  nmeah.test_case("$GPGGA,0,90.000,S,90.000,W,1,08,0.9,545.4,M,46.9,M,,*47",
                  -90207689.8924, -15489467.5247, 545.4);
}

TEST(Nmea2TfPose, rmcTest) {
  NmeaHelper nmeah{};
  nmeah.init();

  nmeah.test_case("$GPRMC,0,A,90.000,N,90.000,E,022.4,084.4,230394,003.1,W*6A",
                  -81305263.475, 5958827.35862);
  nmeah.test_case("$GPRMC,0,A,90.000,N,90.000,W,022.4,084.4,230394,003.1,W*6A",
                  -90207689.8924, 7519179.29268);
  nmeah.test_case("$GPRMC,0,A,90.000,S,90.000,E,022.4,084.4,230394,003.1,W*6A",
                  -81305263.475, -13929115.5907);
  nmeah.test_case("$GPRMC,0,A,90.000,S,90.000,W,022.4,084.4,230394,003.1,W*6A",
                  -90207689.8924, -15489467.5247);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "nmea_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
