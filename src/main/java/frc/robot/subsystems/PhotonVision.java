// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
  
  private final PhotonCamera camera;
  
  private PhotonPipelineResult result;
  private PhotonTrackedTarget bestTarget;
  private boolean hasTargets;
  private boolean lastHeadingPositive;
  private LinearFilter distanceFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
  private double distance = 0;
  private double heading = 0;
  
  /** Creates a new PhotonVision. */
  public PhotonVision() {
    camera = new PhotonCamera("photon1");
    lastHeadingPositive = true;
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
        bestTarget = result.getBestTarget();
        hasTargets = result.hasTargets();

        if (hasTargets) {
            lastHeadingPositive = bestTarget.getYaw() > 0;
        }

        distance = distanceFilter.calculate(calculateDistance());
        SmartDashboard.putNumber("Heading", heading);
        SmartDashboard.putNumber("Distance", distance);
  }

  private double getBestArea() {
      return hasTargets ? bestTarget.getArea() : 0;
  }

  private double getBestPitch() {
      return hasTargets ? bestTarget.getPitch() : 0;
  }

  private double getBestHeading() {
      return hasTargets ? bestTarget.getYaw() : (lastHeadingPositive ? 30 : -30);
  }

  private double getAverageHeading() {
      if (hasTargets) {
          List<PhotonTrackedTarget> targets = result.getTargets();
          double sum = 0;

          for (var target : targets) {
              sum += target.getYaw();
          }

          System.out.println(sum / targets.size());

          return -(sum / targets.size());
      }
      return (lastHeadingPositive ? 30 : -30);
  }

  private double getLeftMostHeading() {
      if (hasTargets) {
          List<PhotonTrackedTarget> targets = result.getTargets();
          double LeftestYaw = targets.get(0).getYaw();

          for (var target : targets) {
              if (target.getYaw() < LeftestYaw) {
                  LeftestYaw = target.getYaw();
              }
          }

          return LeftestYaw;
      }
      return 0;
  }

  public double getHeading() {
      return getAverageHeading();
  }

  public double calculateDistance() {
      if (!camera.getLatestResult().hasTargets()) {
          return 0;
      }

      double angle = Constants.VisionConstants.kCameraMountAngle + getBestPitch();
      return (1 / Math.tan(Conversions.degreesToRadians(angle))) * Constants.VisionConstants.kCameraToTarget;
  }

  public double getDistance() {
      return distance;
  }
}
