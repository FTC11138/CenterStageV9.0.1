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
import org.firstinspires.ftc.teamcode.PoseConstants;

import java.sql.Time;
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

    public void runAuto(
            int propLocation,
            String startPos,

            Pose2d startPose,
            double startTangent,

            Vector2d pixel,
            double pixelAngle,
            double pixelApproachingTangent,

            double afterPixelStartingTangent,
            double afterPixelEndingTangent,
            Vector2d afterPixel,
            double afterPixelAngle,

            Vector2d beforeBackdrop,
            Vector2d backdrop,
            double backdropTangent,

            Vector2d park,
            double parkAngle,
            double parkStartingTangent,
            double parkEndingTangent
    ) {

        robot.setPoseEstimate(startPose);
        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(startPose)
                        .setTangent(startTangent)
                        .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                        .addDisplacementMarker(() -> {
                            robot.setPixelServo(Constants.pixelDrop);
                            roadrunnerSleep(200);
                            robot.setPixelServo(Constants.pixelHold);
                        })
                        .setTangent(afterPixelStartingTangent)
                        .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
                        .splineToSplineHeading(new Pose2d(beforeBackdrop, afterPixelAngle), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.setClawArmServo(Constants.clawArmUp);
                            robot.setTurnClawServo(Constants.turnClawUp);
                            robot.setLiftMotor(0.5, Constants.liftDropAuto);
                        })
                        .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                        .addDisplacementMarker(() -> {
                            if (goToAprilTag(propLocation, startPos, robot.visionPortal, robot.aprilTagProcessor)) {
                                robot.setLiftMotor(1, Constants.liftDropAuto);
                                roadrunnerSleep(500);
                                robot.setClawServo(Constants.clawOpen);
                                roadrunnerSleep(500);

                                robot.setLiftMotor(1, Constants.liftMin);
                                robot.setClawArmServo(Constants.clawArmLow);
                                robot.setTurnClawServo(Constants.turnClawDown);
                                roadrunnerSleep(300);
                            }
                        })
                        .setTangent(parkStartingTangent)
                        .splineToSplineHeading(new Pose2d(park, parkAngle), parkEndingTangent)
                        .build()
        );
    }

    public void runAuto(
            int propLocation,
            String startPos,

            Pose2d startPose,
            double startTangent,

            Vector2d pixel,
            double pixelAngle,
            double pixelApproachingTangent,

            double afterPixelStartingTangent,
            double afterPixelEndingTangent,
            Vector2d afterPixel,
            double afterPixelAngle,

            Vector2d backdrop,
            double backdropTangent,

            Vector2d park,
            double parkAngle,
            double parkStartingTangent,
            double parkEndingTangent
    ) {

        robot.setPoseEstimate(startPose);
        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(startPose)
                        .addDisplacementMarker(() -> {
                            robot.setClawArmServo(Constants.clawArmUp);
                            robot.setTurnClawServo(Constants.turnClawUp);
                            robot.setLiftMotor(0.5, Constants.liftDropAuto);
                        })
                        .setTangent(startTangent)
                        .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                        .addDisplacementMarker(() -> {
                            robot.setPixelServo(Constants.pixelDrop);
                            roadrunnerSleep(200);
                            robot.setPixelServo(Constants.pixelHold);
                        })
                        .setTangent(afterPixelStartingTangent)
                        .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                        .addDisplacementMarker(() -> {
                            if (goToAprilTag(propLocation, startPos, robot.visionPortal, robot.aprilTagProcessor)) {
                                robot.setLiftMotor(0.8, Constants.liftDropAuto);
                                roadrunnerSleep(500);
                                robot.setClawServo(Constants.clawOpen);
                                roadrunnerSleep(500);

                                robot.setLiftMotor(0.8, Constants.liftMin);
                                robot.setClawArmServo(Constants.clawArmLow);
                                robot.setTurnClawServo(Constants.turnClawDown);
                                roadrunnerSleep(300);
                            }
                        })
                        .setTangent(parkStartingTangent)
                        .splineToSplineHeading(new Pose2d(park, parkAngle), parkEndingTangent)
                        .build()
        );
    }

    /*
     * Sleeps for some amount of milliseconds while updating the roadrunner position
     */
    public void roadrunnerSleep(int milliseconds) {
        long timeStamp = runtime.now(TimeUnit.MILLISECONDS);
        while (runtime.now(TimeUnit.MILLISECONDS) - timeStamp <= milliseconds) {
            robot.update();
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

            Pose2d currentPose = robot.getPoseEstimate();
            robot.followTrajectorySequence(
                    robot.trajectorySequenceBuilder(currentPose)
                            .lineToSplineHeading(calcNewPose(rangeError, xError, currentPose))
                            .build()
            );
            return true;

        } else { //does not detect so use roadrunner
            telemetry.addLine("Not Detected");
            return false;

        }

    }

    private Pose2d calcNewPose(double x, double y, Pose2d currentPose) {
        double newX = currentPose.getX() + x;
        double newY = currentPose.getY() - y;
        return new Pose2d(newX, newY, Math.toRadians(180));
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

}