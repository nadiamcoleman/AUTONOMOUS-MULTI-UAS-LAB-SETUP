///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "Client.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <vector>
#include <string.h>

#ifdef WIN32
  #include <conio.h>   // For _kbhit()
  #include <cstdio>   // For getchar()
  #include <windows.h> // For Sleep()
#else
  #include <unistd.h> // For sleep() and usleep
#endif // WIN32

#include <time.h>

// ************* ROS ADDITIONS *************
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
// *****************************************

#define ETH_IP         "128.46.186.232"    // IP Address of Desktop computer running Nexus
//#define MARK_OFFSET   43                  // Vertical Offset (in millimeters) from Pixhawk to Origin Marker
#define MARK_OFFSET    0                  // Vertical Offset (in millimeters) from Pixhawk to Origin Marker
#define PUB_HZ         50                  // Rate at which to publish ROS topic
#define WARN_HZ        2                  // Rate at which to display ROS warnings to user

//#define VICON_TOPIC   "/mavros/mocap/pose"      // ROS Topic name: (/vicon/pose or /mavros/mocap/pose)
#define NUM_DRONES      4                        // Number of intel drones being tracked
#define VICON_TOPIC1   "/intel1/mavros/vision_pose/pose"      // ROS Topic name: (/vicon/pose or /mavros/mocap/pose)
#define VICON_TOPIC2   "/intel2/mavros/vision_pose/pose"      // ROS Topic name: (/vicon/pose or /mavros/mocap/pose)
#define VICON_TOPIC3   "/uvify1/mavros/vision_pose/pose"      // ROS Topic name: (/vicon/pose or /mavros/mocap/pose)
#define VICON_TOPIC4   "/uvify2/mavros/vision_pose/pose"      // ROS Topic name: (/vicon/pose or /mavros/mocap/pose)

using namespace ViconDataStreamSDK::CPP;

#define output_stream if(!LogFile.empty()) ; else std::cout 

namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Kilogram:
        return "Kilogram";
      case Unit::Second:
        return "Second";
      case Unit::Ampere:
        return "Ampere";
      case Unit::Kelvin:
        return "Kelvin";
      case Unit::Mole:
        return "Mole";
      case Unit::Candela:
        return "Candela";
      case Unit::Radian:
        return "Radian";
      case Unit::Steradian:
        return "Steradian";
      case Unit::MeterSquared:
        return "MeterSquared";
      case Unit::MeterCubed:
        return "MeterCubed";
      case Unit::MeterPerSecond:
        return "MeterPerSecond";
      case Unit::MeterPerSecondSquared:
        return "MeterPerSecondSquared";
      case Unit::RadianPerSecond:
        return "RadianPerSecond";
      case Unit::RadianPerSecondSquared:
        return "RadianPerSecondSquared";
      case Unit::Hertz:
        return "Hertz";
      case Unit::Joule:
        return "Joule";
      case Unit::Watt:
        return "Watt";
      case Unit::Pascal:
        return "Pascal";
      case Unit::Lumen:
        return "Lumen";
      case Unit::Lux:
        return "Lux";
      case Unit::Coulomb:
        return "Coulomb";
      case Unit::Ohm:
        return "Ohm";
      case Unit::Farad:
        return "Farad";
      case Unit::Weber:
        return "Weber";
      case Unit::Tesla:
        return "Tesla";
      case Unit::Henry:
        return "Henry";
      case Unit::Siemens:
        return "Siemens";
      case Unit::Becquerel:
        return "Becquerel";
      case Unit::Gray:
        return "Gray";
      case Unit::Sievert:
        return "Sievert";
      case Unit::Katal:
        return "Katal";

      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }
#ifdef WIN32
  bool Hit()
  {
    bool hit = false;
    while( _kbhit() )
    {
      getchar();
      hit = true;
    }
    return hit;
  }
#endif
}

// ROS ADDITIONS


int main( int argc, char* argv[] )
{

  // ************* ROS ADDITIONS *************
  ros::init(argc, argv, "vicon_streamer");
  ros::Time time_keeper = ros::Time();
  ros::NodeHandle n;
  bool tracking_drone1 = false;  // Flag to indicate drone1 is being tracked
  bool tracking_drone2 = false;  // Flag to indicate drone2 is being tracked
  bool tracking_drone3 = false;  // Flag to indicate drone2 is being tracked
  bool tracking_drone4 = false;  // Flag to indicate drone2 is being tracked

  bool occluded_drone1 = false;  // Flag to indicate drone1 is occluded
  bool occluded_drone2 = false;  // Flag to indicate drone2 is occluded
  bool occluded_drone3 = false;  // Flag to indicate drone1 is occluded
  bool occluded_drone4 = false;  // Flag to indicate drone2 is occluded
  double warn_timer = time_keeper.now().toSec();

  // Advertise a vicon pose publisher with queue length = 100
  ros::Publisher vicon_pub1 = n.advertise<geometry_msgs::PoseStamped>(VICON_TOPIC1,10);
  ros::Publisher vicon_pub2 = n.advertise<geometry_msgs::PoseStamped>(VICON_TOPIC2,10);
  ros::Publisher vicon_pub3 = n.advertise<geometry_msgs::PoseStamped>(VICON_TOPIC3,10);
  ros::Publisher vicon_pub4 = n.advertise<geometry_msgs::PoseStamped>(VICON_TOPIC4,10);

  // *****************************************

  // Program options
  //std::string HostName = ETH_IP;
  std::string HostName = "";

  if( argc > 1 )
  {
    HostName = argv[1];  // Take IP address from command line
  }

  // log contains:
  // version number
  // log of framerate over time
  // --multicast
  // kill off internal app
  std::string LogFile = "";
  std::string MulticastAddress = "244.0.0.0:44801";
  bool ConnectToMultiCast = false;
  bool EnableMultiCast = false;
  bool EnableHapticTest = false;
  bool bReadCentroids = false;
  std::vector<std::string> HapticOnList(0);
  for(int a=4; a < argc; ++a)
  {
    std::string arg = argv[a];
    if(arg == "--help")
    {
      //std::cout << argv[0] << " <HostName>: allowed options include:\n  --log_file <LogFile> --enable_multicast <MulticastAddress:Port> --connect_to_multicast <MulticastAddress:Port> --help --enable_haptic_test <DeviceName> --centroids" << std::endl;
      return 0;
    }
    else if (arg=="--log_file")
    {
      if(a < argc)
      {
        LogFile = argv[a+1];
        //std::cout << "Using log file <"<< LogFile << "> ..." << std::endl;
        ++a;
      }
    }
    else if (arg=="--enable_multicast")
    {
      EnableMultiCast = true;
      if(a < argc)
      {
        MulticastAddress = argv[a+1];
        //std::cout << "Enabling multicast address <"<< MulticastAddress << "> ..." << std::endl;
        ++a;
      }
    }
    else if (arg=="--connect_to_multicast")
    {
      ConnectToMultiCast = true;
      if(a < argc)
      {
        MulticastAddress = argv[a+1];
        //std::cout << "connecting to multicast address <"<< MulticastAddress << "> ..." << std::endl;
        ++a;
      }
    }
    else if (arg=="--enable_haptic_test")
    {
      EnableHapticTest = true;
      ++a;
      if ( a < argc )
      {    
        //assuming no haptic device name starts with "--"
        while( a < argc && strncmp( argv[a], "--", 2 ) !=0  )
        {
          HapticOnList.push_back( argv[a] );
          ++a;
        }
      }
    }
    else if( arg=="--centroids" )
    {
      bReadCentroids = true;
    }
    else
    {
      //std::cout << "Failed to understand argument <" << argv[a] << ">...exiting" << std::endl;
      return 1;
    }
  }

  std::ofstream ofs;
  if(!LogFile.empty())
  {
    ofs.open(LogFile.c_str());
    if(!ofs.is_open())//#define DRONE_NAME    "Intel_1"           // Name of drone object in Vicon

    {
      //std::cout << "Could not open log file <" << LogFile << ">...exiting" << std::endl;
      return 1;
    }
  }
  // Make a new client
  Client MyClient;

  for(int i=0; i != 3 && ros::ok() ; ++i) // repeat to check disconnecting doesn't wreck next connect
  {
    // Connect to a server
    //std::cout << "Connecting to " << HostName << " ..." << std::flush;
    while( !MyClient.IsConnected().Connected && ros::ok())
    {
      // Direct connection

      bool ok = false;
      if(ConnectToMultiCast)
      {
        // Multicast connection
        ok = ( MyClient.ConnectToMulticast( HostName, MulticastAddress ).Result == Result::Success );

      }
      else
      {
        ok =( MyClient.Connect( HostName ).Result == Result::Success );
      }
      if(!ok)
      {
        std::cout << "Warning - connect failed..." << std::endl;
      }


      //std::cout << ".";
  #ifdef WIN32
      Sleep( 1000 );
  #else
      sleep(1);
  #endif
    }
    //std::cout << std::endl;

    // Enable some different data types
    MyClient.EnableSegmentData();
    //MyClient.EnableMarkerData();
    //MyClient.EnableUnlabeledMarkerData();
    //MyClient.EnableDeviceData();
    if( bReadCentroids )
    {
      MyClient.EnableCentroidData();
    }

    //std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
    //std::cout << "Marker Data Enabled: "           << Adapt( MyClient.IsMarkerDataEnabled().Enabled )          << std::endl;
    //std::cout << "Unlabeled Marker Data Enabled: " << Adapt( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) << std::endl;
    //std::cout << "Device Data Enabled: "           << Adapt( MyClient.IsDeviceDataEnabled().Enabled )          << std::endl;
    //std::cout << "Centroid Data Enabled: "         << Adapt( MyClient.IsCentroidDataEnabled().Enabled )        << std::endl;

    // Set the streaming mode
    //MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward, 
                             Direction::Left, 
                             Direction::Up ); // Z-up
    // MyClient.SetGlobalUpAxis( Direction::Forward, 
    //                           Direction::Up, 
    //                           Direction::Right ); // Y-up

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    //std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
    //                       << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
    //                       << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    //std::cout << "Version: " << _Output_GetVersion.Major << "." 
    //                         << _Output_GetVersion.Minor << "." 
    //                         << _Output_GetVersion.Point << std::endl;

    if( EnableMultiCast )
    {
      assert( MyClient.IsConnected().Connected );
      MyClient.StartTransmittingMulticast( HostName, MulticastAddress );
    }

    size_t FrameRateWindow = 1000; // frames
    size_t Counter = 0;
    clock_t LastTime = clock();
    // Loop until a key is pressed
  #ifdef WIN32
    while( !Hit() )
  #else
    ros::Rate loop_rate(PUB_HZ);
    while( ros::ok() )
  #endif
    {
      // Get a frame
      //ROS_INFO("Waiting for new frame...\n");
      //output_stream << "Waiting for new frame...";
      while( MyClient.GetFrame().Result != Result::Success )
      {
        // Sleep a little so that we don't lumber the CPU with a busy poll
        #ifdef WIN32
          Sleep( 200 );
        #else
          sleep(1);
        #endif

        //output_stream << ".";
      }
      //output_stream << std::endl;
      if(++Counter == FrameRateWindow)
      {
        clock_t Now = clock();
        double FrameRate = (double)(FrameRateWindow * CLOCKS_PER_SEC) / (double)(Now - LastTime);
        if(!LogFile.empty())
        {
          time_t rawtime;
          struct tm * timeinfo;
          time ( &rawtime );
          timeinfo = localtime ( &rawtime );

          ofs << "Frame rate = " << FrameRate << " at " <<  asctime (timeinfo)<< std::endl;
        }

        LastTime = Now;
        Counter = 0;
      }

      // Get the frame number
      Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      //output_stream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;

      // Reset data validation flags for each new Vicon frame
      tracking_drone1 = false;
      tracking_drone2 = false;
			tracking_drone3 = false;
      tracking_drone4 = false;
      occluded_drone1 = false;
      occluded_drone2 = false;     
			occluded_drone3 = false;
      occluded_drone4 = false;

      if( EnableHapticTest == true )
      {
        for (size_t i = 0; i < HapticOnList.size(); ++ i)
        {
          if( Counter % 2 == 0 )
          {
              Output_SetApexDeviceFeedback Output= MyClient.SetApexDeviceFeedback( HapticOnList[i],  true ); 
              if( Output.Result == Result::Success )
              {
                //output_stream<< "Turn haptic feedback on for device: " << HapticOnList[i]<<std::endl;
              }
              else if( Output.Result == Result::InvalidDeviceName )
              {
                //output_stream<< "Device doesn't exist: "<< HapticOnList[i]<<std::endl;
              }
          }
          if( Counter % 20 == 0 )
          {
              Output_SetApexDeviceFeedback Output = MyClient.SetApexDeviceFeedback( HapticOnList[i],  false); 

              if( Output.Result == Result::Success )
              {
                //output_stream<< "Turn haptic feedback off for device: " << HapticOnList[i]<<std::endl;
              }
          }
        }
      }

      Output_GetFrameRate Rate = MyClient.GetFrameRate();
      //std::cout << "Frame rate: "           << Rate.FrameRateHz          << std::endl;
      // Get the timecode
      Output_GetTimecode _Output_GetTimecode  = MyClient.GetTimecode();

      //output_stream << "Timecode: "
      //          << _Output_GetTimecode.Hours               << "h "
      //          << _Output_GetTimecode.Minutes             << "m " 
      //          << _Output_GetTimecode.Seconds             << "s "
      //          << _Output_GetTimecode.Frames              << "f "
      //          << _Output_GetTimecode.SubFrame            << "sf "
      //          << Adapt( _Output_GetTimecode.FieldFlag ) << " " 
      //          << _Output_GetTimecode.Standard            << " " 
      //          << _Output_GetTimecode.SubFramesPerFrame   << " " 
      //          << _Output_GetTimecode.UserBits            << std::endl << std::endl;

      // Get the latency
      //output_stream << "Latency: " << MyClient.GetLatencyTotal().Total << "s" << std::endl;
      
      for( unsigned int LatencySampleIndex = 0 ; LatencySampleIndex < MyClient.GetLatencySampleCount().Count ; ++LatencySampleIndex )
      {
        std::string SampleName  = MyClient.GetLatencySampleName( LatencySampleIndex ).Name;
        double      SampleValue = MyClient.GetLatencySampleValue( SampleName ).Value;

        //output_stream << "  " << SampleName << " " << SampleValue << "s" << std::endl;
      }
      //output_stream << std::endl;

      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      //output_stream << "Subjects (" << SubjectCount << "):" << std::endl;
      for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
      {
        //output_stream << "  Subject #" << SubjectIndex << std::endl;

        // Get the subject name
        std::string SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;

        // Update drone tracking flags
        if(SubjectName.compare("Intel_1") == 0) { tracking_drone1 = true; }
        if(SubjectName.compare("Intel_2") == 0) { tracking_drone2 = true; }
        if(SubjectName.compare("Uvify_1") == 0) { tracking_drone3 = true; }
        if(SubjectName.compare("Uvify_2") == 0) { tracking_drone4 = true; }

        //output_stream << "    Name: " << SubjectName << std::endl;#define DRONE_NAME    "Intel_1"           // Name of drone object in Vicon


        // Get the root segment
        std::string RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
        //output_stream << "    Root Segment: " << RootSegment << std::endl;

        // Count the number of segments
        unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
        //output_stream << "    Segments (" << SegmentCount << "):" << std::endl;
        for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
        {
          //output_stream << "      Segment #" << SegmentIndex << std::endl;

          // Get the segment name
          std::string SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
          //output_stream << "        Name: " << SegmentName << std::endl;

          // Get the segment parent
          std::string SegmentParentName = MyClient.GetSegmentParentName( SubjectName, SegmentName ).SegmentName;
          //output_stream << "        Parent: " << SegmentParentName << std::endl;

          // Get the segment's children
          unsigned int ChildCount = MyClient.GetSegmentChildCount( SubjectName, SegmentName ).SegmentCount;
          //output_stream << "     Children (" << ChildCount << "):" << std::endl;
          for( unsigned int ChildIndex = 0 ; ChildIndex < ChildCount ; ++ChildIndex )
          {
            std::string ChildName = MyClient.GetSegmentChildName( SubjectName, SegmentName, ChildIndex ).SegmentName;
            //output_stream << "       " << ChildName << std::endl;
          }

          // Get the static segment translation
          Output_GetSegmentStaticTranslation _Output_GetSegmentStaticTranslation = 
            MyClient.GetSegmentStaticTranslation( SubjectName, SegmentName );
         // output_stream << "        Static Translation: (" << _Output_GetSegmentStaticTranslation.Translation[ 0 ]  << ", " 
         //                                              << _Output_GetSegmentStaticTranslation.Translation[ 1 ]  << ", " 
          //                                             << _Output_GetSegmentStaticTranslation.Translation[ 2 ]  << ")" << std::endl;

          // Get the static segment rotation in helical co-ordinates
          Output_GetSegmentStaticRotationHelical _Output_GetSegmentStaticRotationHelical = 
            MyClient.GetSegmentStaticRotationHelical( SubjectName, SegmentName );
          //output_stream << "        Static Rotation Helical: (" << _Output_GetSegmentStaticRotationHelical.Rotation[ 0 ]     << ", " 
           //                                                 << _Output_GetSegmentStaticRotationHelical.Rotation[ 1 ]     << ", " 
           //                                                 << _Output_GetSegmentStaticRotationHelical.Rotation[ 2 ]     << ")" << std::endl;

          // Get the static segment rotation as a matrix
          Output_GetSegmentStaticRotationMatrix _Output_GetSegmentStaticRotationMatrix = 
            MyClient.GetSegmentStaticRotationMatrix( SubjectName, SegmentName );
          //output_stream << "        Static Rotation Matrix: (" << _Output_GetSegmentStaticRotationMatrix.Rotation[ 0 ]     << ", " 
          //                                                 << _Output_GetSegmentStaticRotationMatrix.Rotation[ 1 ]     << ", " 
           //                                                << _Output_GetSegmentStaticRotationMatrix.Rotation[ 2 ]     << ", " 
          //                                                 << _Output_GetSegmentStaticRotationMatrix.Rotation[ 3 ]     << ", " 
          //                                                 << _Output_GetSegmentStaticRotationMatrix.Rotation[ 4 ]     << ", " 
          //                                                 << _Output_GetSegmentStaticRotationMatrix.Rotation[ 5 ]     << ", " 
          //                                                 << _Output_GetSegmentStaticRotationMatrix.Rotation[ 6 ]     << ", " 
          //                                                 << _Output_GetSegmentStaticRotationMatrix.Rotation[ 7 ]     << ", " 
          //                                                 << _Output_GetSegmentStaticRotationMatrix.Rotation[ 8 ]     << ")" << std::endl;

          // Get the static segment rotation in quaternion co-ordinates
          Output_GetSegmentStaticRotationQuaternion _Output_GetSegmentStaticRotationQuaternion = 
            MyClient.GetSegmentStaticRotationQuaternion( SubjectName, SegmentName );
          //output_stream << "        Static Rotation Quaternion: (" << _Output_GetSegmentStaticRotationQuaternion.Rotation[ 0 ]     << ", " 
          //                                                     << _Output_GetSegmentStaticRotationQuaternion.Rotation[ 1 ]     << ", " 
          //                                                     << _Output_GetSegmentStaticRotationQuaternion.Rotation[ 2 ]     << ", " 
          //                                                     << _Output_GetSegmentStaticRotationQuaternion.Rotation[ 3 ]     << ")" << std::endl;

          // Get the static segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentStaticRotationEulerXYZ _Output_GetSegmentStaticRotationEulerXYZ = 
            MyClient.GetSegmentStaticRotationEulerXYZ( SubjectName, SegmentName );
          //output_stream << "        Static Rotation EulerXYZ: (" << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 0 ]     << ", " 
          //                                                   << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 1 ]     << ", " 
           //                                                  << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[ 2 ]     << ")" << std::endl;

          // Get the global segment translation
          Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
            MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
          //output_stream << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[ 0 ]  << ", "
          //                                             << _Output_GetSegmentGlobalTranslation.Translation[ 1 ]  << ", "
          //                                             << _Output_GetSegmentGlobalTranslation.Translation[ 2 ]  << ") "
          //                                             << Adapt( _Output_GetSegmentGlobalTranslation.Occluded ) << std::endl;

          // Get the global segment rotation in helical co-ordinates
          Output_GetSegmentGlobalRotationHelical _Output_GetSegmentGlobalRotationHelical = 
            MyClient.GetSegmentGlobalRotationHelical( SubjectName, SegmentName );
          //output_stream << "        Global Rotation Helical: (" << _Output_GetSegmentGlobalRotationHelical.Rotation[ 0 ]     << ", "
          //                                                  << _Output_GetSegmentGlobalRotationHelical.Rotation[ 1 ]     << ", "
          //                                                  << _Output_GetSegmentGlobalRotationHelical.Rotation[ 2 ]     << ") "
          //                                                  << Adapt( _Output_GetSegmentGlobalRotationHelical.Occluded ) << std::endl;

          // Get the global segment rotation as a matrix
          Output_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix = 
            MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName );
          //output_stream << "        Global Rotation Matrix: (" << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ]     << ", " 
          //                                                 << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ]     << ") " 
          //                                                 << Adapt( _Output_GetSegmentGlobalRotationMatrix.Occluded ) << std::endl;

          // Get the global segment rotation in quaternion co-ordinates
          Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion = 
            MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
          //output_stream << "        Global Rotation Quaternion: (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ]     << ", " 
          //                                                     << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ]     << ", " 
          //                                                     << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ]     << ", " 
          //                                                     << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]     << ") " 
          //                                                     << Adapt( _Output_GetSegmentGlobalRotationQuaternion.Occluded ) << std::endl;

          // Get the global segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ = 
            MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
          //output_stream << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ]     << ", " 
          //                                                   << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ]     << ", " 
          //                                                   << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ]     << ") " 
          //                                                   << Adapt( _Output_GetSegmentGlobalRotationEulerXYZ.Occluded ) << std::endl;

          // ************* ROS ADDITIONS *************
          geometry_msgs::PoseStamped vicon_pose;
          vicon_pose.header.stamp = time_keeper.now();
          vicon_pose.pose.position.x = _Output_GetSegmentGlobalTranslation.Translation[0] / 1000.0;
          vicon_pose.pose.position.y = _Output_GetSegmentGlobalTranslation.Translation[1] / 1000.0;
          vicon_pose.pose.position.z = (_Output_GetSegmentGlobalTranslation.Translation[2] - MARK_OFFSET) / 1000.0;
          vicon_pose.pose.orientation.x = _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
          vicon_pose.pose.orientation.y = _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
          vicon_pose.pose.orientation.z = _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
          vicon_pose.pose.orientation.w = _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
          float m[9];
          // NWU      | 0 1 2 |                       | a b c |
          //    R =   | 3 4 5 |  (row-major order) =  | d e f |
          //     FLU  | 6 7 8 |                       | g h i | 
          for(int i=0;i<9;i++)
          {
            m[i] = (float) _Output_GetSegmentGlobalRotationMatrix.Rotation[i];
          }

          // Obtain Rotation matrix of body_left w.r.t. NED
          // NED      |  1  0  0 |
          //	  R =   |  0 -1  0 |
          //	   NWU  |  0  0 -1 |
          // ** NOTE: Pre-multiplying by above matrix preserves 1st row, negates 2nd and 3rd rows

          // Obtain Rotation matrix of body_right w.r.t. body_left (since body_right is used by ArduCopter)
          // FLU      |  1  0  0 |
          //    R =   |  0 -1  0 |
          //     FRD  |  0  0 -1 |
          // ** NOTE: Post-multiplying by R preserves 1st column, negates 2nd and 3rd columns

          // Final result:
          //  NED        NED       NWU       FLU        |  a -b -c |   |  m[0] -m[1] -m[2] |
          //     R     =    R    *    R    *    R    =  | -d  e  f | = | -m[3]  m[4]  m[5] |
          //      FRD        NWU       FLU       FRD    | -g  h  i |   | -m[6]  m[7]  m[8] |             
          //m[1] = -m[1];
          //m[2] = -m[2];
          //m[3] = -m[3];
          //m[6] = -m[6];

          // Extract matrix elements to be used in building the quaternion
          const float m00 = m[0];
          const float m01 = m[1];
          const float m02 = m[2];
          const float m10 = m[3];
          const float m11 = m[4];
          const float m12 = m[5];
          const float m20 = m[6];
          const float m21 = m[7];
          const float m22 = m[8];

          // Build quaternion from matrix 'm' in order to obtain Euler XYZ angles
          // Based on from_rotation_matrix(...) function in
          //		"ardupilot/libraries/AP_Math/quaternion.cpp"
          float qw = 0;
          float qx = 0;
          float qy = 0;
          float qz = 0;
          float tr = m00 + m11 + m22;
          if (tr > 0) {
            float S = sqrtf(tr+1) * 2;
            qw = 0.25f * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S;
            qz = (m10 - m01) / S;
          } else if ((m00 > m11) && (m00 > m22)) { 
            float S = sqrtf(1.0f + m00 - m11 - m22) * 2;
            qw = (m21 - m12) / S;
            qx = 0.25f * S;
            qy = (m01 + m10) / S; 
            qz = (m02 + m20) / S; 
          } else if (m11 > m22) { 
            float S = sqrtf(1.0f + m11 - m00 - m22) * 2;
            qw = (m02 - m20) / S;//#define DRONE_NAME    "Intel_1"           // Name of drone object in Vicon

            qx = (m01 + m10) / S; 
            qy = 0.25f * S;
            qz = (m12 + m21) / S; 
          } else { 
            float S = sqrtf(1.0f + m22 - m00 - m11) * 2;
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25f * S;
          }
          geometry_msgs::Quaternion qt1 = vicon_pose.pose.orientation;
          geometry_msgs::Point pt = vicon_pose.pose.position;
          //ROS_INFO("Vehicle Pos. mm (XYZ):  %6.1f, %6.1f, %6.1f\n", pt.x, pt.y, pt.z);
          //ROS_INFO("Vicon Quaternion (wxyz): %.4f, %.4f, %.4f, %.4f\n",qt1.w,qt1.x,qt1.y,qt1.z);
          //ROS_INFO("Comp. Quaternion (wxyz): %.4f, %.4f, %.4f, %.4f\n",qw,qx,qy,qz);

          // Convert position to meters
          if(SubjectName.compare("Intel_1") == 0) {
              if(Adapt( _Output_GetSegmentGlobalTranslation.Occluded ).compare("True") == 0) {
                  occluded_drone1 = true;
              }
              vicon_pub1.publish(vicon_pose);
          }
          else if(SubjectName.compare("Intel_2") == 0) {
              if(Adapt( _Output_GetSegmentGlobalTranslation.Occluded ).compare("True") == 0) {
                  occluded_drone2 = true;
              }
              vicon_pub2.publish(vicon_pose);
					}		
          else if(SubjectName.compare("Uvify_1") == 0) {
              if(Adapt( _Output_GetSegmentGlobalTranslation.Occluded ).compare("True") == 0) {
                  occluded_drone3 = true;
              }
              vicon_pub3.publish(vicon_pose);	
					}		
          else if(SubjectName.compare("Uvify_2") == 0) {
              if(Adapt( _Output_GetSegmentGlobalTranslation.Occluded ).compare("True") == 0) {
                  occluded_drone4 = true;
              }
              vicon_pub4.publish(vicon_pose);
          } else {
              // ROS_WARN("No Subject match! NOT publishing pose data to topic");
          }

          // *****************************************

          // Get the local segment translation
          Output_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation = 
            MyClient.GetSegmentLocalTranslation( SubjectName, SegmentName );
          //output_stream << "        Local Translation: (" << _Output_GetSegmentLocalTranslation.Translation[ 0 ]  << ", " 
          //                                            << _Output_GetSegmentLocalTranslation.Translation[ 1 ]  << ", " 
          //                                            << _Output_GetSegmentLocalTranslation.Translation[ 2 ]  << ") " 
           //                                           << Adapt( _Output_GetSegmentLocalTranslation.Occluded ) << std::endl;

          // Get the local segment rotation in helical co-ordinates
          Output_GetSegmentLocalRotationHelical _Output_GetSegmentLocalRotationHelical = 
            MyClient.GetSegmentLocalRotationHelical( SubjectName, SegmentName );
          //output_stream << "        Local Rotation Helical: (" << _Output_GetSegmentLocalRotationHelical.Rotation[ 0 ]     << ", "
          //                                                 << _Output_GetSegmentLocalRotationHelical.Rotation[ 1 ]     << ", "
          //                                                 << _Output_GetSegmentLocalRotationHelical.Rotation[ 2 ]     << ") "
          //                                                 << Adapt( _Output_GetSegmentLocalRotationHelical.Occluded ) << std::endl;

          // Get the local segment rotation as a matrix#define DRONE_NAME    "Intel_1"           // Name of drone object in Vicon

          Output_GetSegmentLocalRotationMatrix _Output_GetSegmentLocalRotationMatrix = 
            MyClient.GetSegmentLocalRotationMatrix( SubjectName, SegmentName );
          //output_stream << "        Local Rotation Matrix: (" << _Output_GetSegmentLocalRotationMatrix.Rotation[ 0 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 1 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 2 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 3 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 4 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 5 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 6 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 7 ]     << ", "
          //                                                << _Output_GetSegmentLocalRotationMatrix.Rotation[ 8 ]     << ") "
          //                                                << Adapt( _Output_GetSegmentLocalRotationMatrix.Occluded ) << std::endl;

          // Get the local segment rotation in quaternion co-ordinates
          Output_GetSegmentLocalRotationQuaternion _Output_GetSegmentLocalRotationQuaternion = 
            MyClient.GetSegmentLocalRotationQuaternion( SubjectName, SegmentName );
          //output_stream << "        Local Rotation Quaternion: (" << _Output_GetSegmentLocalRotationQuaternion.Rotation[ 0 ]     << ", "
           //                                                   << _Output_GetSegmentLocalRotationQuaternion.Rotation[ 1 ]     << ", " 
             //                                                 << _Output_GetSegmentLocalRotationQuaternion.Rotation[ 2 ]     << ", " 
               //                                               << _Output_GetSegmentLocalRotationQuaternion.Rotation[ 3 ]     << ") " 
                 //                                             << Adapt( _Output_GetSegmentLocalRotationQuaternion.Occluded ) << std::endl;

          // Get the local segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentLocalRotationEulerXYZ _Output_GetSegmentLocalRotationEulerXYZ = 
            MyClient.GetSegmentLocalRotationEulerXYZ( SubjectName, SegmentName );
          //output_stream << "        Local Rotation EulerXYZ: (" << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 0 ]     << ", " 
          //                                                  << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 1 ]     << ", " 
          //                                                  << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[ 2 ]     << ") " 
          //                                                  << Adapt( _Output_GetSegmentLocalRotationEulerXYZ.Occluded ) << std::endl;
        }
        #define DRONE_NAME    "Intel_1"           // Name of drone object in Vicon


        // Count the number of markers
        unsigned int MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
        //output_stream << "    Markers (" << MarkerCount << "):" << std::endl;
        for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
        {
          // Get the marker name
          std::string MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

          // Get the marker parent
          std::string MarkerParentName = MyClient.GetMarkerParentName( SubjectName, MarkerName ).SegmentName;

          // Get the global marker translation
          Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
            MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );

          //output_stream << "      Marker #" << MarkerIndex            << ": "
          //                              << MarkerName             << " ("
          //                              << _Output_GetMarkerGlobalTranslation.Translation[ 0 ]  << ", "
          //                              << _Output_GetMarkerGlobalTranslation.Translation[ 1 ]  << ", "
          //                              << _Output_GetMarkerGlobalTranslation.Translation[ 2 ]  << ") "
          //                              << Adapt( _Output_GetMarkerGlobalTranslation.Occluded ) << std::endl;
        }
      }

      // Get the unlabeled markers
      unsigned int UnlabeledMarkerCount = MyClient.GetUnlabeledMarkerCount().MarkerCount;
      //output_stream << "    Unlabeled Markers (" << UnlabeledMarkerCount << "):" << std::endl;
      for( unsigned int UnlabeledMarkerIndex = 0 ; UnlabeledMarkerIndex < UnlabeledMarkerCount ; ++UnlabeledMarkerIndex )
      { 
        // Get the global marker translation
        Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation =
          MyClient.GetUnlabeledMarkerGlobalTranslation( UnlabeledMarkerIndex );

        //output_stream << "      Marker #" << UnlabeledMarkerIndex   << ": ("
        //                              << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ] << ", "
        //                              << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 1 ] << ", "
        //                              << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 2 ] << ")" << std::endl;
      }

      // Count the number of devices
      unsigned int DeviceCount = MyClient.GetDeviceCount().DeviceCount;
      //output_stream << "  Devices (" << DeviceCount << "):" << std::endl;
      for( unsigned int DeviceIndex = 0 ; DeviceIndex < DeviceCount ; ++DeviceIndex )
      {
        //output_stream << "    Device #" << DeviceIndex << ":" << std::endl;

        // Get the device name and type
        Output_GetDeviceName _Output_GetDeviceName = MyClient.GetDeviceName( DeviceIndex );
        //output_stream << "      Name: " << _Output_GetDeviceName.DeviceName << std::endl;
        //output_stream << "      Type: " << Adapt( _Output_GetDeviceName.DeviceType ) << std::endl;

        // Count the number of device outputs
        unsigned int DeviceOutputCount = MyClient.GetDeviceOutputCount( _Output_GetDeviceName.DeviceName ).DeviceOutputCount;
       // output_stream << "      Device Outputs (" << DeviceOutputCount << "):" << std::endl;
        for( unsigned int DeviceOutputIndex = 0 ; DeviceOutputIndex < DeviceOutputCount ; ++DeviceOutputIndex )
        {
          // Get the device output name and unit
          Output_GetDeviceOutputName _Output_GetDeviceOutputName = 
            MyClient.GetDeviceOutputName( _Output_GetDeviceName.DeviceName, DeviceOutputIndex );

          unsigned int DeviceOutputSubsamples = 
                         MyClient.GetDeviceOutputSubsamples( _Output_GetDeviceName.DeviceName, 
                                                             _Output_GetDeviceOutputName.DeviceOutputName ).DeviceOutputSubsamples;

          //output_stream << "      Device Output #" << DeviceOutputIndex << ":" << std::endl;
          //output_stream << "      Samples (" << DeviceOutputSubsamples << "):" << std::endl;

          for( unsigned int DeviceOutputSubsample = 0; DeviceOutputSubsample < DeviceOutputSubsamples; ++DeviceOutputSubsample )
          {
            //output_stream << "        Sample #" << DeviceOutputSubsample << ":" << std::endl;

            // Get the device output value
            Output_GetDeviceOutputValue _Output_GetDeviceOutputValue = 
              MyClient.GetDeviceOutputValue( _Output_GetDeviceName.DeviceName, 
                                             _Output_GetDeviceOutputName.DeviceOutputName, 
                                             DeviceOutputSubsample );

            //output_stream << "          '" << _Output_GetDeviceOutputName.DeviceOutputName          << "' "
            //                               << _Output_GetDeviceOutputValue.Value                    << " " 
            //                               << Adapt( _Output_GetDeviceOutputName.DeviceOutputUnit ) << " " 
             //                              << Adapt( _Output_GetDeviceOutputValue.Occluded )        << std::endl;
          }
        }
      }

      // Output the force plate information.
      unsigned int ForcePlateCount = MyClient.GetForcePlateCount().ForcePlateCount;
      //output_stream << "  Force Plates: (" << ForcePlateCount << ")" << std::endl;

      for( unsigned int ForcePlateIndex = 0 ; ForcePlateIndex < ForcePlateCount ; ++ForcePlateIndex )
      {
        //output_stream << "    Force Plate #" << ForcePlateIndex << ":" << std::endl;

        unsigned int ForcePlateSubsamples = MyClient.GetForcePlateSubsamples( ForcePlateIndex ).ForcePlateSubsamples;

            //output_stream << "    Samples (" << ForcePlateSubsamples << "):" << std::endl;

        for( unsigned int ForcePlateSubsample = 0; ForcePlateSubsample < ForcePlateSubsamples; ++ForcePlateSubsample )
        {
          //output_stream << "      Sample #" << ForcePlateSubsample << ":" << std::endl;

          Output_GetGlobalForceVector _Output_GetForceVector = MyClient.GetGlobalForceVector( ForcePlateIndex, ForcePlateSubsample );
          //output_stream << "        Force (" << _Output_GetForceVector.ForceVector[ 0 ] << ", ";
          //output_stream << _Output_GetForceVector.ForceVector[ 1 ] << ", ";
          //output_stream << _Output_GetForceVector.ForceVector[ 2 ] << ")" << std::endl;

          Output_GetGlobalMomentVector _Output_GetMomentVector = 
                                         MyClient.GetGlobalMomentVector( ForcePlateIndex, ForcePlateSubsample );
          //output_stream << "        Moment (" << _Output_GetMomentVector.MomentVector[ 0 ] << ", ";
          //output_stream << _Output_GetMomentVector.MomentVector[ 1 ] << ", ";
          //output_stream << _Output_GetMomentVector.MomentVector[ 2 ] << ")" << std::endl;

          Output_GetGlobalCentreOfPressure _Output_GetCentreOfPressure = 
                                             MyClient.GetGlobalCentreOfPressure( ForcePlateIndex, ForcePlateSubsample );
          //output_stream << "        CoP (" << _Output_GetCentreOfPressure.CentreOfPressure[ 0 ] << ", ";
          //output_stream << _Output_GetCentreOfPressure.CentreOfPressure[ 1 ] << ", ";
          //output_stream << _Output_GetCentreOfPressure.CentreOfPressure[ 2 ] << ")" << std::endl;
        }
      }

      // Output eye tracker information.
      unsigned int EyeTrackerCount = MyClient.GetEyeTrackerCount().EyeTrackerCount;
      //output_stream << "  Eye Trackers: (" << EyeTrackerCount << ")" << std::endl;

      for( unsigned int EyeTrackerIndex = 0 ; EyeTrackerIndex < EyeTrackerCount ; ++EyeTrackerIndex )
      {
        //output_stream << "    Eye Tracker #" << EyeTrackerIndex << ":" << std::endl;

        Output_GetEyeTrackerGlobalPosition _Output_GetEyeTrackerGlobalPosition = MyClient.GetEyeTrackerGlobalPosition( EyeTrackerIndex );

        //output_stream << "      Position (" << _Output_GetEyeTrackerGlobalPosition.Position[ 0 ] << ", ";
        //output_stream << _Output_GetEyeTrackerGlobalPosition.Position[ 1 ] << ", ";
        //output_stream << _Output_GetEyeTrackerGlobalPosition.Position[ 2 ] << ") ";
        //output_stream << Adapt( _Output_GetEyeTrackerGlobalPosition.Occluded ) << std::endl;

        Output_GetEyeTrackerGlobalGazeVector _Output_GetEyeTrackerGlobalGazeVector = MyClient.GetEyeTrackerGlobalGazeVector( EyeTrackerIndex );

        //output_stream << "      Gaze (" << _Output_GetEyeTrackerGlobalGazeVector.GazeVector[ 0 ] << ", ";
        //output_stream << _Output_GetEyeTrackerGlobalGazeVector.GazeVector[ 1 ] << ", ";
        //output_stream << _Output_GetEyeTrackerGlobalGazeVector.GazeVector[ 2 ] << ") ";
        //output_stream << Adapt( _Output_GetEyeTrackerGlobalGazeVector.Occluded ) << std::endl;
      }

      if( bReadCentroids )
      {
        unsigned int CameraCount = MyClient.GetCameraCount().CameraCount;
        //output_stream << "Cameras(" << CameraCount << "):" << std::endl;

        for( unsigned int CameraIndex = 0; CameraIndex < CameraCount; ++CameraIndex )
        {
          //output_stream << "  Camera #" << CameraIndex << ":" << std::endl;
        
          const std::string CameraName = MyClient.GetCameraName( CameraIndex ).CameraName;
          //output_stream << "    Name: " << CameraName << std::endl;

          /*
          unsigned int CentroidCount = MyClient.GetCentroidCount( CameraName ).CentroidCount;
          //output_stream << "    Centroids(" << CentroidCount << "):" << std::endl;

          for( unsigned int CentroidIndex = 0; CentroidIndex < CentroidCount; ++CentroidIndex )
          {
            //output_stream << "      Centroid #" << CentroidIndex << ":" << std::endl;

            Output_GetCentroidPosition _Output_GetCentroidPosition = MyClient.GetCentroidPosition( CameraName, CentroidIndex );
            //output_stream << "        Position: (" << _Output_GetCentroidPosition.CentroidPosition[0] << ", "
            //                                       << _Output_GetCentroidPosition.CentroidPosition[1] << ")" << std::endl;
            //output_stream << "        Radius: ("    << _Output_GetCentroidPosition.Radius   << ")" << std::endl;
            //output_stream << "        Accuracy: ("  << _Output_GetCentroidPosition.Accuracy << ")" << std::endl;
          }
          */
        }
      }

      // Output tracking/occlusion warnings as needed
      if(time_keeper.now().toSec() - warn_timer > 1.0/WARN_HZ) {
          if(!tracking_drone1) { ROS_WARN("Not tracking Intel_1!"); }
          else if(occluded_drone1) { ROS_WARN("Intel_1 occluded!"); }
          if(!tracking_drone2) { /*ROS_WARN("Not tracking Intel_2!"); */}
          else if(occluded_drone2) { ROS_WARN("Intel_2 occluded!"); }
          if(!tracking_drone3) { ROS_WARN("Not tracking Uvify_1!"); }
          else if(occluded_drone3) { ROS_WARN("Uvify_1 occluded!"); }
          if(!tracking_drone4) { ROS_WARN("Not tracking Uvify_2!"); }
          else if(occluded_drone4) { /*ROS_WARN("Uvify_2 occluded!"); */}
          warn_timer = time_keeper.now().toSec();   // Update warning timer
      }
      loop_rate.sleep();        // Enforce ROS publish loop rate
    }

    if( EnableMultiCast )
    {
      MyClient.StopTransmittingMulticast();
    }
    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    if( bReadCentroids )
    {
      MyClient.DisableCentroidData();
    }

    // Disconnect and dispose
    int t = clock();
    std::cout << " Disconnecting..." << std::endl;
    MyClient.Disconnect();
    int dt = clock() - t;
    double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
    std::cout << " Disconnect time = " << secs << " secs" << std::endl;

  }
}
