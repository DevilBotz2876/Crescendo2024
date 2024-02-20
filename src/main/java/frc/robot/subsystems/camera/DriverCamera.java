// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverCamera extends SubsystemBase {
  private static final VideoMode VIDEO_MODE = new VideoMode(PixelFormat.kMJPEG, 360, 270, 10);
  private final double ratio = VIDEO_MODE.height / VIDEO_MODE.width;
  private static final int COMPRESSION_LEVEL = 35;
  private UsbCamera driveCamera = null;
  private MjpegServer videoSink = null;

  public DriverCamera() {
    // try {

    // } catch (Exception e) {
    //     System.out.println("Driver camera not found");
    // }
  }

  public void setup() {
    try {
      driveCamera = CameraServer.startAutomaticCapture(0);
      driveCamera.setBrightness(20);
      // driveCamera.setResolution(240, 135);
      // driveCamera.setFPS(20);
      driveCamera.setVideoMode(
          VIDEO_MODE.pixelFormat,
          VIDEO_MODE.width,
          (int) (VIDEO_MODE.width * ratio),
          VIDEO_MODE.fps);
      driveCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kAutoManage);

      videoSink = CameraServer.addServer("Drive Camera");

      if (COMPRESSION_LEVEL >= 0) {
        videoSink.setCompression(COMPRESSION_LEVEL);
        videoSink.setDefaultCompression(COMPRESSION_LEVEL);
      }
      videoSink.setSource(driveCamera);

      ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

      VideoSource source = videoSink.getSource();
      driveTab.add("Drive Camera", source).withSize(4, 4).withPosition(7, 0);
    } catch (Exception e) {
      System.out.println("Driver camera not found");
    }
  }

  public void simpleSetup() {
    driveCamera = CameraServer.startAutomaticCapture(0);

    driveCamera.setResolution(240, 135);
    driveCamera.setFPS(20);
    driveCamera.setPixelFormat(PixelFormat.kMJPEG);

    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab
        .add("Driver View", driveCamera)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(5, 0)
        .withSize(4, 4);
  }
}
