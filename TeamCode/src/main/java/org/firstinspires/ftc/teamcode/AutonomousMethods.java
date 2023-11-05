package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

public abstract class AutonomousMethods extends LinearOpMode {


    public Attachments robot = new Attachments();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Orientation angles;
    public ElapsedTime runtime = new ElapsedTime();

    public boolean opModeStatus() {
        return opModeIsActive();
    }

    public void initializeRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        robot.initialize(hardwareMap, telemetry, true);
//        robot.setPixelServo(Constants.pixelHold);

    }


    public void dropPixel_toBackdrop(int propLocation, String startPosition, boolean continueAfterPixel) {
        Trajectory spikeMarkTraj = null;
        Trajectory afterSpikeMarkTraj = null;
        Trajectory toBeforeBackBoardTraj = null;
        Trajectory toBackBoardTraj = null;

        Pose2d startPose = null;
        Vector2d pixel = null;
        Vector2d afterPixel = null;
        Vector2d backdrop = null;
        Vector2d beforeBackdrop = null;
        double startTangent = 0;
        double afterPixelAngle = 0;
        double pixelAngle = 0;
        double pixelApproachingTangent = 0;
        double afterPixelStartingTangent = 0;
        double afterPixelEndingTangent = 0;
        double backdropTangent = 0;
        boolean backdropSide = true;


        if (Objects.equals(startPosition, "redLeft")) { // Done

            backdropSide = PoseConstants.redLeft.backdropSide;

            startPose = PoseConstants.redLeft.start;
            startTangent = PoseConstants.redLeft.startingTangent[propLocation - 1];

            pixel = PoseConstants.redLeft.pixel[propLocation - 1];
            pixelAngle = PoseConstants.redLeft.pixelAngle[propLocation - 1];

            pixelApproachingTangent = PoseConstants.redLeft.pixelApproachingTangent[propLocation - 1];
            afterPixelStartingTangent = PoseConstants.redLeft.afterPixelStartingTangent[propLocation - 1];
            afterPixelEndingTangent = PoseConstants.redLeft.afterPixelEndingTangent[propLocation - 1];
            afterPixel = PoseConstants.redLeft.afterPixel;
            afterPixelAngle = PoseConstants.redLeft.afterPixelAngle[propLocation - 1];

            beforeBackdrop = PoseConstants.redLeft.beforeBackdrop;
            backdrop = PoseConstants.redLeft.backdrop[propLocation - 1];
            backdropTangent = PoseConstants.redLeft.backdropTangent;

        } else if (Objects.equals(startPosition, "redRight")) {

            backdropSide = PoseConstants.redRight.backdropSide;

            startPose = PoseConstants.redRight.start;
            startTangent = PoseConstants.redRight.startingTangent[propLocation - 1];

            pixel = PoseConstants.redRight.pixel[propLocation - 1];
            pixelAngle = PoseConstants.redRight.pixelAngle[propLocation - 1];
            pixelApproachingTangent = PoseConstants.redRight.pixelApproachingTangent[propLocation - 1];

            afterPixelStartingTangent = PoseConstants.redRight.afterPixelStartingTangent[propLocation - 1];
            afterPixelEndingTangent = PoseConstants.redRight.afterPixelEndingTangent[propLocation - 1];
            afterPixel = PoseConstants.redRight.afterPixel;
            afterPixelAngle = PoseConstants.redRight.afterPixelAngle[propLocation - 1];

            backdrop = PoseConstants.redRight.backdrop[propLocation - 1];
            backdropTangent = PoseConstants.redRight.backdropTangent;

        } else if (Objects.equals(startPosition, "blueLeft")) {

            backdropSide = PoseConstants.blueLeft.backdropSide;

            startPose = PoseConstants.blueLeft.start;
            startTangent = PoseConstants.blueLeft.startingTangent[propLocation - 1];

            pixel = PoseConstants.blueLeft.pixel[propLocation - 1];
            pixelAngle = PoseConstants.blueLeft.pixelAngle[propLocation - 1];
            pixelApproachingTangent = PoseConstants.blueLeft.pixelApproachingTangent[propLocation - 1];

            afterPixelStartingTangent = PoseConstants.blueLeft.afterPixelStartingTangent[propLocation - 1];
            afterPixelEndingTangent = PoseConstants.blueLeft.afterPixelEndingTangent[propLocation - 1];
            afterPixel = PoseConstants.blueLeft.afterPixel;
            afterPixelAngle = PoseConstants.blueLeft.afterPixelAngle[propLocation - 1];

            backdrop = PoseConstants.blueLeft.backdrop[propLocation - 1];
            backdropTangent = PoseConstants.blueLeft.backdropTangent;

        } else if (Objects.equals(startPosition, "blueRight")) { // Done

            backdropSide = PoseConstants.blueRight.backdropSide;

            startPose = PoseConstants.blueRight.start;
            startTangent = PoseConstants.blueRight.startingTangent[propLocation - 1];

            pixel = PoseConstants.blueRight.pixel[propLocation - 1];
            pixelAngle = PoseConstants.blueRight.pixelAngle[propLocation - 1];
            pixelApproachingTangent = PoseConstants.blueRight.pixelApproachingTangent[propLocation - 1];

            afterPixelStartingTangent = PoseConstants.blueRight.afterPixelStartingTangent[propLocation - 1];
            afterPixelEndingTangent = PoseConstants.blueRight.afterPixelEndingTangent[propLocation - 1];
            afterPixel = PoseConstants.blueRight.afterPixel;
            afterPixelAngle = PoseConstants.blueRight.afterPixelAngle[propLocation - 1];

            beforeBackdrop = PoseConstants.blueRight.beforeBackdrop;
            backdrop = PoseConstants.blueRight.backdrop[propLocation - 1];
            backdropTangent = PoseConstants.blueRight.backdropTangent;

        }

        if (!backdropSide) {

            spikeMarkTraj = robot.trajectoryBuilder(startPose, startTangent)
                    .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                    .build();

            robot.followTrajectory(spikeMarkTraj);

            afterSpikeMarkTraj = robot.trajectoryBuilder(robot.getPoseEstimate(), afterPixelStartingTangent)
                    .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
                    .build();

            robot.setPixelServo(Constants.pixelDrop);
            sleep(1000);
            robot.setPixelServo(Constants.pixelHold);

            if (!continueAfterPixel) {
                return;
            }

            robot.followTrajectory(afterSpikeMarkTraj);

            toBeforeBackBoardTraj = robot.trajectoryBuilder(robot.getPoseEstimate(), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(beforeBackdrop, afterPixelAngle), Math.toRadians(0))
                    .build();

            robot.followTrajectory(toBeforeBackBoardTraj);

            toBackBoardTraj = robot.trajectoryBuilder(robot.getPoseEstimate(), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                    .build();

            robot.followTrajectory(toBackBoardTraj);

        } else {
            spikeMarkTraj = robot.trajectoryBuilder(startPose, startTangent)
                    .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                    .build();

            robot.followTrajectory(spikeMarkTraj);

            afterSpikeMarkTraj = robot.trajectoryBuilder(robot.getPoseEstimate(), afterPixelStartingTangent)
                    .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
                    .build();

            robot.setPixelServo(Constants.pixelDrop);
            sleep(1000);
            robot.setPixelServo(Constants.pixelHold);

            if (!continueAfterPixel) {
                return;
            }

            robot.followTrajectory(afterSpikeMarkTraj);

            toBackBoardTraj = robot.trajectoryBuilder(robot.getPoseEstimate(), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                    .build();

            robot.followTrajectory(toBackBoardTraj);
        }

    }

    public boolean goToAprilTag(int propLocation, String startPosition, VisionPortal visionPortal, AprilTagProcessor aprilTagProcessor) {
        telemetry.addData("Made it: ", "to goToAprilTag");
        telemetry.update();
        final double DESIRED_DISTANCE = Constants.DESIRED_DISTANCE;

        final boolean USE_WEBCAM = true;
        int DESIRED_TAG_ID = propLocation;
        if (startPosition.equals("redLeft") || startPosition.equals("redRight")) {
            DESIRED_TAG_ID += 3;
        }
        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        /* -------------------------------------------- APRIL TAG DETECTION -------------------------------------------- */

        if (USE_WEBCAM) {
            setManualExposure(6, 250, visionPortal);
        }

        telemetry.addData("Finished ", "initialization");
        telemetry.update();
        sleep(500);

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }
        telemetry.addData("targetFound ", targetFound);
        telemetry.update();


        /* -------------------------------------------- MOVEMENT -------------------------------------------- */

        if (targetFound) { //should add timer
            telemetry.addLine("Detected");

            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE) * -1;
            double xError = desiredTag.ftcPose.x * -1;

            telemetry.addData("rangeError ", rangeError);
            telemetry.addData("yawError", xError);
            telemetry.update();

            TrajectorySequence toAprilTag = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                    .forward(rangeError)
                    .strafeRight(xError)
                    .build();
            robot.followTrajectorySequence(toAprilTag);
            return true;

        } else { //does not detect so use roadrunner
            telemetry.addLine("Not Detected");
            return false;

        }

    }

    private void setManualExposure(int exposureMS, int gain, VisionPortal visionPortal) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    public void moveRobotAprilTag(double x, double y, double yaw, DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive, DcMotor rightBackDrive) {
        telemetry.addData("inside moveRobotAprilTag = ", "true");
//        telemetry.update();
        // Calculate wheel powers.
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        sleep(500);
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void roadrunnerToBackdrop(int propLocation, String startPosition) {
        Vector2d toBackDrop = null;

        if (startPosition.equals("blueRight") || startPosition.equals("blueLeft")) {
            toBackDrop = (propLocation == 1) ? PoseConstants.backDropBlueRight.left : ((propLocation == 2) ? PoseConstants.backDropBlueRight.center : PoseConstants.backDropBlueRight.right);
        } else if (startPosition.equals("redRight") || startPosition.equals("redLeft")) {
            toBackDrop = (propLocation == 4) ? PoseConstants.backDropRedRight.left : ((propLocation == 5) ? PoseConstants.backDropRedRight.center : PoseConstants.backDropRedRight.right);
        }

        Pose2d currentPose = robot.getPoseEstimate();
        Trajectory backdropTraj = robot.trajectoryBuilder(currentPose)
                .lineTo(toBackDrop)
                //.splineToSplineHeading(new Pose2d(new Vector2d(46, 29), Math.toRadians(180)), Math.toRadians(180))
                .build();
        robot.followTrajectory(backdropTraj);

    }

    public void toPark(int parkLoc, String startPosition) {
        Vector2d toPark = null;
        double startingTangent = 0;

        if (parkLoc == 3) {
            return;
        }

        if (startPosition.equals("blueRight") || startPosition.equals("blueLeft")) {
            if (parkLoc == 1) {
                toPark = PoseConstants.blueRight.parkRight;
                startingTangent = Math.toRadians(-90);
            } else {
                toPark = PoseConstants.blueRight.parkLeft;
                startingTangent = Math.toRadians(90);
            }
        } else if (startPosition.equals("redRight") || startPosition.equals("redLeft")) {
            if (parkLoc == 1) {
                toPark = PoseConstants.redLeft.parkLeft;
                startingTangent = Math.toRadians(90);
            } else {
                toPark = PoseConstants.redLeft.parkRight;
                startingTangent = Math.toRadians(-90);
            }
        }

        Pose2d currentPose = robot.getPoseEstimate();
        Trajectory parkTraj = robot.trajectoryBuilder(currentPose, startingTangent)
                .splineToSplineHeading(new Pose2d(toPark, Math.toRadians(0)), Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(new Vector2d(46, 29), Math.toRadians(180)), Math.toRadians(180))
                .build();
        robot.followTrajectory(parkTraj);
    }
}