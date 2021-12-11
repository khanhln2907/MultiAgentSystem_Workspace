#include <qualisys/QualisysDriver.h>
#include <algorithm>

using namespace std;

namespace qualisys{

double QualisysDriver::deg2rad = M_PI / 180.0;

QualisysDriver::QualisysDriver(const ros::NodeHandle& n):
  nh(n),
  publish_tf(false){
  return;
}

bool QualisysDriver::init() {
  // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
  // of the workspace options
  nh.param("server_address", server_address, string("192.168.254.1"));
  nh.param("server_base_port", base_port, 22222);
  nh.param("publish_tf", publish_tf, false);

  // Connecting to the server
  ROS_INFO_STREAM("Connecting to the Qualisys Motion Tracking system specified at: "
      << server_address << ":" << base_port);

  //if((!port_protocol.Connect((char *)server_address.data(), base_port, 0, 1, 7))
  //		|| (!port_protocol3D.Connect((char *)server_address.data(), base_port, 0, 1, 7)))
if((!port_protocol.Connect((char *)server_address.data(), base_port, 0, 1, 7)) ||
(!port_protocol3D.Connect((char *)server_address.data(), base_port, 0, 1, 7)))
   {
    ROS_FATAL_STREAM("Could not find the Qualisys Motion Tracking system at: "
        << server_address << ":" << base_port);
    return false;
  }
  ROS_INFO_STREAM("Connected to " << server_address << ":" << base_port);

  // Get 6DOF settings
  port_protocol.Read6DOFSettings();

  //Added by Jerry Zhang
  port_protocol3D.Read3DSettings();

  return true;
}

void QualisysDriver::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server "
      << server_address << ":" << base_port);
  port_protocol.StreamFramesStop();
  port_protocol3D.StreamFramesStop();
  port_protocol.Disconnect();
  port_protocol3D.Disconnect();
  return;
}

void QualisysDriver::checkPublishers(const int& body_count) {
  map<string, bool> subject_indicator;

  for (auto it = subject_publishers.begin();
      it != subject_publishers.end(); ++it)
    subject_indicator[it->first] = false;

  // Check publishers for each body
  for(int i = 0; i < body_count; ++i) {
    //std::stringstream name;
    //name << port_protocol.Get6DOFBodyName(i);
    string name(port_protocol.Get6DOFBodyName(i));

    // Create a publisher for the rigid body
    // if it does not have one.
    if (subject_publishers.find(name) ==
          subject_publishers.end())
      subject_publishers[name] =
        nh.advertise<qualisys::Subject>(name, 10);

    subject_indicator[name] = true;
  }

 
  for (auto it = subject_indicator.begin();
      it != subject_indicator.end(); ++it) {
    if (it->second == false)
      subject_publishers.erase(it->first);
  }

  return;
}

    //Added by Jerry Zhang
void QualisysDriver::checkPublishers3D(const int& marker_count) {
  map<string, bool> point_indicator;

  for (auto it = point_publishers.begin();
      it != point_publishers.end(); ++it)
    point_indicator[it->first] = false;

  for(int i = 0; i < marker_count; ++i) {
    //std::stringstream name;
    //name << port_protocol.Get6DOFBodyName(i);
    string name3D(port_protocol3D.Get3DLabelName(i));



    // Create a publisher for the rigid body
    // if it does not have one.
    if (point_publishers.find(name3D) ==
          point_publishers.end())
      point_publishers[name3D] =
        nh.advertise<qualisys::Marker>(name3D, 10);

    point_indicator[name3D] = true;
  }

  for (auto it = point_indicator.begin();
      it != point_indicator.end(); ++it) {
    if (it->second == false)
      point_publishers.erase(it->first);
  }


  return;
}

void QualisysDriver::handlePacketData(CRTPacket* prt_packet) {

  // Number of rigid bodies
  int body_count = prt_packet->Get6DOFEulerBodyCount();

  // Check the publishers for the rigid bodies
  checkPublishers(body_count);

  // Publish data for each rigid body
  for(int i = 0; i < body_count; ++i) {
    float x, y, z, roll, pitch, yaw;
    prt_packet->Get6DOFEulerBody(i, x, y, z, roll, pitch, yaw);

    if(isnan(x) || isnan(y) || isnan(z) ||
        isnan(roll) || isnan(pitch) || isnan(yaw)) {
      ROS_WARN_STREAM_THROTTLE(3, "Rigid-body " << i + 1 << "/"
          << body_count << " not detected");
      continue;
    }

    // ROTATION: GLOBAL (FIXED) X Y Z (R P Y)
    //std::stringstream name;
    //name << port_protocol.Get6DOFBodyName(i);
    string subject_name(port_protocol.Get6DOFBodyName(i));

    // Qualisys sometimes flips 180 degrees around the x axis
    if(roll > 90)
      roll -= 180;
    else if(roll < -90)
      roll += 180;

    // Send transform
    tf::StampedTransform stamped_transform = tf::StampedTransform(
        tf::Transform(
            tf::createQuaternionFromRPY(
                roll * deg2rad, pitch * deg2rad, yaw * deg2rad),
            tf::Vector3(x, y, z) / 1000.),
        ros::Time::now(), "qualisys", subject_name);
    if (publish_tf)
      tf_publisher.sendTransform(stamped_transform);

    // Send Subject msg
    geometry_msgs::TransformStamped geom_stamped_transform;
    tf::transformStampedTFToMsg(stamped_transform,
        geom_stamped_transform);

    qualisys::Subject subject_msg;
    subject_msg.header =
      geom_stamped_transform.header;
    subject_msg.name = subject_name;
    subject_msg.position.x =
        geom_stamped_transform.transform.translation.x;
    subject_msg.position.y =
        geom_stamped_transform.transform.translation.y;
    subject_msg.position.z =
        geom_stamped_transform.transform.translation.z;
    subject_msg.orientation =
        geom_stamped_transform.transform.rotation;
    subject_publishers[subject_name].publish(subject_msg);
  
}
  return;
}

//This function is added by Jerry Zhang
void QualisysDriver::handlePacketData3D(CRTPacket* prt_packet3D) {


  int marker_count = prt_packet3D->Get3DMarkerCount();


  checkPublishers3D(marker_count);

  for(int i = 0; i < marker_count; ++i) {
    float x, y, z;
	prt_packet3D->Get3DMarker(i,x,y,z);
    if(isnan(x) || isnan(y) || isnan(z)) {
      ROS_WARN_STREAM_THROTTLE(3, "Aim-marker " << i + 1 << "/"
          << marker_count << " not detected");
      continue;
    }

    string label_name(port_protocol3D.Get3DLabelName(i));

    qualisys::Marker marker_msg;
    marker_msg.name = label_name;
    marker_msg.position.x = x/1000;
    marker_msg.position.y = y/1000;
    marker_msg.position.z = z/1000;
    marker_msg.occluded = 0;




    point_publishers[label_name].publish(marker_msg);
  }

  return;
}

void QualisysDriver::run() {

  CRTPacket* prt_packet = port_protocol.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol.GetCurrentFrame(CRTProtocol::Component6dEuler);

  if(port_protocol.ReceiveRTPacket(e_type, true)) {

    switch(e_type) {
      // Case 1 - sHeader.nType 0 indicates an error
      case CRTPacket::PacketError:
        ROS_ERROR_STREAM_THROTTLE(
            1, "Error when streaming 6DOF frames: "
            << port_protocol.GetRTPacket()->GetErrorString());
        break;

      // Case 2 - No more data
      case CRTPacket::PacketNoMoreData:
        ROS_WARN_STREAM_THROTTLE(1, "No more data from rigid bodies");
        break;

      // Case 3 - Data received
      case CRTPacket::PacketData:
        handlePacketData(prt_packet);
        break;

      default:
        ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case in receiving 6DOF data");
    }
  }

  return;
}

//This function is added by Jerry Zhang
void QualisysDriver::run3D() {

  CRTPacket* prt_packet3D = port_protocol3D.GetRTPacket();

  CRTPacket::EPacketType e_type3D;

  port_protocol3D.GetCurrentFrame(CRTProtocol::Component3d);


  if(port_protocol3D.ReceiveRTPacket(e_type3D, true)) {

    switch(e_type3D) {
      // Case 1 - sHeader.nType 0 indicates an error
      case CRTPacket::PacketError:
        ROS_ERROR_STREAM_THROTTLE(
            1, "Error when streaming 3D point frames: "
            << port_protocol3D.GetRTPacket()->GetErrorString());
        break;

      // Case 2 - No more data
      case CRTPacket::PacketNoMoreData:
        ROS_WARN_STREAM_THROTTLE(1, "No more data from 3D points");
        break;

      // Case 3 - Data received
      case CRTPacket::PacketData:
        handlePacketData3D(prt_packet3D);

    //    handlePacketData3D(prt_packet);
        break;

      default:
        ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case in receiving 3D points data");
    }
  }

  return;
}

}
