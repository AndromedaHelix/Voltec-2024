/**
 * Written by Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import com.andromedalib.vision.LimelightHelpers;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionIOLimelight implements VisionIO {

    AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public VisionIOLimelight() {
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.Results result = LimelightHelpers
                .getLatestResults(VisionConstants.aprilLimeNTName).targetingResults;

        inputs.TA = LimelightHelpers.getTA(VisionConstants.aprilLimeNTName);

        if (!(result.botpose[0] == 0 && result.botpose[1] == 0) &&
                LimelightHelpers.getTA(VisionConstants.aprilLimeNTName) > 0.1) {
            inputs.hasTarget = true;

            inputs.observedPose2d = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.aprilLimeNTName);

            inputs.timestampLatency = Logger.getRealTimestamp()
                    - (result.latency_capture + result.latency_pipeline / 1000.0);
            inputs.targetDistance = computeTagDistance(inputs.observedPose2d);

            try {
                inputs.targetID = (int) LimelightHelpers.getFiducialID(VisionConstants.aprilLimeNTName);
            } catch (Exception e) {
                DriverStation.reportError("[Limelight] Error found while trying to compute target ID", true);
                inputs.targetID = 0;
            }
        } else {
            inputs.observedPose2d = new Pose2d();
            inputs.timestampLatency = 0;
            inputs.hasTarget = false;
            inputs.targetDistance = 0.0;
            inputs.targetID = 0;
        }
    }

    public double computeTagDistance(Pose2d pose) {
        double id = LimelightHelpers.getFiducialID(VisionConstants.aprilLimeNTName);

        if (layout.getTagPose((int) id).isPresent()) {
            return PhotonUtils.getDistanceToPose(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.aprilLimeNTName),
                    layout.getTagPose((int) id).get().toPose2d());

        }

        return 0.0;

    }

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return true;
    }

    @Override
    public void changePipeline(int pipelineNumber) {
        LimelightHelpers.setPipelineIndex(VisionConstants.aprilLimeNTName, pipelineNumber);
    }
}