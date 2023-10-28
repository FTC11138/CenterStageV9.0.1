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

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
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



    public void dropPixel_toBackdrop(int propLocation, String startPosition) {
        Trajectory spikeMarkTraj = null;
        Trajectory afterSpikeMarkTraj = null;
        Trajectory toBackBoardTraj = null;

        Pose2d startPose = null;
        Vector2d pixelVector = null;
        Vector2d afterPixel = null;
        Vector2d backdrop = null;
        double startTangent = 0;
        double afterPixelAngle = 0;
        double pixelAngle = 0;
        double pixelApproachingTangent = 0;
        double afterPixelStartingTangent = 0;
        double afterPixelEndingTangent = 0;
        double backdropTangent = 0;
        boolean backdropSide = false;


        if ( Objects.equals(startPosition, "redLeft") ) {
            startPose = PoseConstants.redLeft.start;
            pixelVector = (propLocation == 1) ? PoseConstants.redLeft.left : ((propLocation == 2) ? PoseConstants.redLeft.center : PoseConstants.redLeft.right);
            pixelAngle = (propLocation == 1) ? PoseConstants.redLeft.spikeMarkAngleLeft : ( (propLocation == 2) ? PoseConstants.redLeft.spikeMarkAngleCenter : PoseConstants.redLeft.spikeMarkAngleRight);
            afterPixelStartingTangent = (propLocation == 1) ? PoseConstants.redLeft.afterSpikeMarkStartingTangentLeft : ( (propLocation == 2) ? PoseConstants.redLeft.afterSpikeMarkStartingTangentCenter : PoseConstants.redLeft.afterSpikeMarkStartingTangentRight);
            afterPixelEndingTangent = PoseConstants.redLeft.afterSpikeMarkEndingTangent;
            afterPixel = PoseConstants.redLeft.afterSpikeMark;
            afterPixelAngle = PoseConstants.redLeft.afterSpikeMarkAngle;
            backdrop = PoseConstants.redLeft.backdrop;
            backdropTangent = PoseConstants.redLeft.backdropTangent;
            backdropSide = PoseConstants.redLeft.backdropSide;
        } else if ( Objects.equals(startPosition, "redRight") ) {
            startPose = PoseConstants.redRight.start;
            pixelVector = (propLocation == 1) ? PoseConstants.redRight.left : ( (propLocation == 2) ? PoseConstants.redRight.center : PoseConstants.redRight.right);
            pixelAngle = (propLocation == 1) ? PoseConstants.redRight.spikeMarkAngleLeft : ( (propLocation == 2) ? PoseConstants.redRight.spikeMarkAngleCenter : PoseConstants.redRight.spikeMarkAngleRight);
            afterPixelStartingTangent = (propLocation == 1) ? PoseConstants.redRight.afterSpikeMarkStartingTangentLeft : ( (propLocation == 2) ? PoseConstants.redRight.afterSpikeMarkStartingTangentCenter : PoseConstants.redRight.afterSpikeMarkStartingTangentRight);
            afterPixelEndingTangent = PoseConstants.redRight.afterSpikeMarkEndingTangent;
            afterPixel = PoseConstants.redRight.afterSpikeMark;
            afterPixelAngle = PoseConstants.redRight.afterSpikeMarkAngle;
            backdrop = PoseConstants.redRight.backdrop;
            backdropTangent = PoseConstants.redRight.backdropTangent;
            backdropSide = PoseConstants.redRight.backdropSide;
        } else if ( Objects.equals(startPosition, "blueLeft") ) {
            startPose = PoseConstants.blueLeft.start;
            pixelVector = (propLocation == 1) ? PoseConstants.blueLeft.left : ( (propLocation == 2) ? PoseConstants.blueLeft.center : PoseConstants.blueLeft.right);
            pixelAngle = (propLocation == 1) ? PoseConstants.blueLeft.spikeMarkAngleLeft : ( (propLocation == 2) ? PoseConstants.blueLeft.spikeMarkAngleCenter : PoseConstants.blueLeft.spikeMarkAngleRight);
            afterPixelStartingTangent = (propLocation == 1) ? PoseConstants.blueLeft.afterSpikeMarkStartingTangentLeft : ( (propLocation == 2) ? PoseConstants.blueLeft.afterSpikeMarkStartingTangentCenter : PoseConstants.blueLeft.afterSpikeMarkStartingTangentRight);
            afterPixelEndingTangent = PoseConstants.blueLeft.afterSpikeMarkEndingTangent;
            afterPixel = PoseConstants.blueLeft.afterSpikeMark;
            afterPixelAngle = PoseConstants.blueLeft.afterSpikeMarkAngle;
            backdrop = PoseConstants.blueLeft.backdrop;
            backdropTangent = PoseConstants.blueLeft.backdropTangent;
            backdropSide = PoseConstants.blueLeft.backdropSide;
        } else if ( Objects.equals(startPosition, "blueRight") ) {
            startPose = PoseConstants.blueRight.start;
            startTangent = PoseConstants.blueRight.startingTangent;
            pixelVector = (propLocation == 1) ? PoseConstants.blueRight.left : ( (propLocation == 2) ? PoseConstants.blueRight.center : PoseConstants.blueRight.right);
            pixelAngle = (propLocation == 1) ? PoseConstants.blueRight.spikeMarkAngleLeft : ( (propLocation == 2) ? PoseConstants.blueRight.spikeMarkAngleCenter : PoseConstants.blueRight.spikeMarkAngleRight);
            pixelApproachingTangent = (propLocation == 1) ? PoseConstants.blueRight.spikeMarkApproachingTangentLeft : ( (propLocation == 2) ? PoseConstants.blueRight.spikeMarkApproachingTangentCenter : PoseConstants.blueRight.spikeMarkApproachingTangentRight);
            afterPixelStartingTangent = (propLocation == 1) ? PoseConstants.blueRight.afterSpikeMarkStartingTangentLeft : ( (propLocation == 2) ? PoseConstants.blueRight.afterSpikeMarkStartingTangentCenter : PoseConstants.blueRight.afterSpikeMarkStartingTangentRight);
            afterPixelEndingTangent = PoseConstants.blueRight.afterSpikeMarkEndingTangent;
            afterPixel = PoseConstants.blueRight.afterSpikeMark;
            afterPixelAngle = PoseConstants.blueRight.afterSpikeMarkAngle;
            backdrop = PoseConstants.blueRight.backdrop;
            backdropTangent = PoseConstants.blueRight.backdropTangent;
            backdropSide = PoseConstants.blueRight.backdropSide;
        }


        if (backdropSide) {

            spikeMarkTraj = robot.trajectoryBuilder(startPose)
                    .splineTo(pixelVector, pixelAngle)
                    .build();

            toBackBoardTraj = robot.trajectoryBuilder(new Pose2d(pixelVector, pixelAngle), afterPixelStartingTangent)
                    .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(0)), backdropTangent)
                    .build();

            robot.followTrajectory(spikeMarkTraj);

//            robot.setPixelServo(Constants.pixelDrop);
            sleep(500);

            robot.followTrajectory(toBackBoardTraj);

        } else {

            spikeMarkTraj = robot.trajectoryBuilder(startPose, startTangent)
                    .splineToSplineHeading(new Pose2d(pixelVector, pixelAngle), pixelApproachingTangent)
                    .build();

            afterSpikeMarkTraj = robot.trajectoryBuilder(new Pose2d(pixelVector, pixelAngle), afterPixelStartingTangent)
                    .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
                    .build();

            toBackBoardTraj = robot.trajectoryBuilder(new Pose2d(afterPixel, afterPixelAngle), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                    .build();

            robot.followTrajectory(spikeMarkTraj);

//            robot.setPixelServo(Constants.pixelDrop);
            sleep(500);

            robot.followTrajectory(afterSpikeMarkTraj);
            robot.followTrajectory(toBackBoardTraj);

        }

    }

    public void goToAprilTag(int propLocation, String startPosition, VisionPortal visionPortal, AprilTagProcessor aprilTagProcessor) {
        telemetry.addData("Made it: ", "to goToAprilTag");
        telemetry.update();
        final double DESIRED_DISTANCE = Constants.DESIRED_DISTANCE;
        final double SPEED_GAIN = Constants.SPEED_GAIN;
        final double STRAFE_GAIN = Constants.STRAFE_GAIN;
        final double TURN_GAIN = Constants.TURN_GAIN;
        final double MAX_AUTO_SPEED = Constants.MAX_AUTO_SPEED;
        final double MAX_AUTO_STRAFE = Constants.MAX_AUTO_STRAFE;
        final double MAX_AUTO_TURN = Constants.MAX_AUTO_TURN;

        final boolean USE_WEBCAM = true;
        int DESIRED_TAG_ID = propLocation;
        //VisionPortal visionPortal = new VisionPortal.Builder().build();
        //AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        AprilTagDetection desiredTag = null;

        boolean targetFound = false;
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;
        //double rangeError = Integer.MAX_VALUE;
        //double headingError = Integer.MIN_VALUE;
        //double yawError = Integer.MAX_VALUE;
        //initAprilTag(visionPortal, aprilTagProcessor);

        DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf"); //leftFront
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rf"); //rightfront_drive
        DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "lr");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rr");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        /* -------------------------------------------- APRIL TAG DETECTION -------------------------------------------- */

        if (USE_WEBCAM) {
            setManualExposure(6, 250, visionPortal);
        }

        telemetry.addData("Finished ", "initialization");
        telemetry.update();
        sleep(2000);
        double rangeError = Integer.MAX_VALUE;
        int i = 0;
        while (rangeError > DESIRED_DISTANCE) {
            telemetry.addData("running while ", i);
            targetFound = false;
            desiredTag  = null;
            drive           = 0;        // Desired forward power/speed (-1 to +1)
            strafe          = 0;        // Desired strafe power/speed (-1 to +1)
            turn            = 0;
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("Inside detection loop", "detecting");
                telemetry.update();
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        telemetry.addData("Detection", detection.id);
                        telemetry.update();
                        sleep(500);
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            telemetry.addData("targetFound ", targetFound);
            telemetry.update();
            sleep(500);


            /* -------------------------------------------- MOVEMENT -------------------------------------------- */
            targetFound = false;

            if (targetFound) { //should add timer
                telemetry.addData("found ", "continuing");
                telemetry.update();
                rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = 0.1 * Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("rangeError ", rangeError);
                telemetry.update();
                sleep(500);


                moveRobotAprilTag(drive, strafe, turn, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
                sleep(10);
            } else { //does not detect
                telemetry.addData("not found", "continuing");
                telemetry.update();

                if (startPosition == "blueRight" || startPosition == "blueLeft") {
                    telemetry.addData("into blueRight ", "driving");
                    telemetry.update();

                    roadrunnerToBackdrop(DESIRED_TAG_ID, startPosition);
                    /**
                    Pose2d currentPose = robot.getPoseEstimate();
                    Trajectory backdropTraj = robot.trajectoryBuilder(currentPose, Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(new Vector2d(46, 29), Math.toRadians(180)), Math.toRadians(180))
                            //.splineToSplineHeading(new Vector2d(45, 30), Math.toRadians(180), )
                            .build();
                    robot.followTrajectory(backdropTraj);**/
                    sleep(2000);
                    break;
                } else {
                    PoseConstants.redRight startPose = new PoseConstants.redRight();
                    Trajectory backdropTraj = robot.trajectoryBuilder(robot.getPoseEstimate())
                            .splineTo(startPose.backdrop, Math.toRadians(0))
                            .build();
                    robot.followTrajectory(backdropTraj);
                    //break;
                }
            }
            i++;
            telemetry.update();
        }
    }

    /**
    private void initAprilTag(VisionPortal visionPortal, AprilTagProcessor aprilTagProcessor) {
        // Create the AprilTag processor by using a builder.
        //aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        // Create the vision portal by using a builder.
        if (true) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTagProcessor)
                    .build();
        }
    }**/

    private void    setManualExposure(int exposureMS, int gain, VisionPortal visionPortal) {
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
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    public void moveRobotAprilTag(double x, double y, double yaw, DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive, DcMotor rightBackDrive) {
        telemetry.addData("inside moveRobotAprilTag = ", "true");
        telemetry.update();
        // Calculate wheel powers.
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

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
        telemetry.addData("leftFrontPower ", leftFrontPower);
        telemetry.addData("rightFrontPower ", rightFrontPower);
        telemetry.addData("leftBackPower ", leftBackPower);
        telemetry.addData("rightBackPower", rightBackPower);
        telemetry.update();
        sleep(500);
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void roadrunnerToBackdrop(int propLocation, String startPosition) {
        Vector2d toBackDrop = null;
        double toBackDropTangent = 0;
        if (startPosition.equals("blueRight") || startPosition.equals("blueLeft")) {
            toBackDrop = (propLocation == 1) ? PoseConstants.backDropBlueRight.left : ( (propLocation == 2) ? PoseConstants.backDropBlueRight.center : PoseConstants.backDropBlueRight.right);
            toBackDropTangent = Math.toRadians(180);
        }
        else if (startPosition.equals("redRight") || startPosition.equals("redLeft")) {
            toBackDrop = (propLocation == 1) ? PoseConstants.backDropRedRight.left : ( (propLocation == 2) ? PoseConstants.backDropRedRight.center : PoseConstants.backDropRedRight.right);
            toBackDropTangent = Math.toRadians(180);
        }

        Pose2d currentPose = robot.getPoseEstimate();
        Trajectory backdropTraj = robot.trajectoryBuilder(currentPose, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(toBackDrop, Math.toRadians(180)), toBackDropTangent)
                //.splineToSplineHeading(new Pose2d(new Vector2d(46, 29), Math.toRadians(180)), Math.toRadians(180))
                .build();
        robot.followTrajectory(backdropTraj);

    /**
        toBackBoardTraj = robot.trajectoryBuilder(new Pose2d(afterPixel, afterPixelAngle), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                .build();**/
    }
}