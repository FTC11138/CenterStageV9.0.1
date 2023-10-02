package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
        robot.setPixelServo(Constants.pixelHold);

    }



    public void dropPixel_toBackdrop(int propLocation, String startPosition) {
        Trajectory spikeMarkTraj = null;
        Trajectory afterSpikeMarkTraj = null;
        Trajectory toBackBoardTraj = null;

        Pose2d startPose = null;
        Vector2d pixelVector = null;
        Vector2d afterPixel = null;
        Vector2d backdrop = null;
        double afterPixelAngle = 0;
        double pixelAngle = 0;
        double afterPixelStartingTangent = 0;
        double afterPixelEndingTangent = 0;
        double backdropTangent = 0;
        boolean backdropSide = false;


        if (startPosition == "redLeft") {
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
        } else if (startPosition == "redRight") {
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
        } else if (startPosition == "blueLeft") {
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
        } else if (startPosition == "blueRight") {
            startPose = PoseConstants.blueRight.start;
            pixelVector = (propLocation == 1) ? PoseConstants.blueRight.left : ( (propLocation == 2) ? PoseConstants.blueRight.center : PoseConstants.blueRight.right);
            pixelAngle = (propLocation == 1) ? PoseConstants.blueRight.spikeMarkAngleLeft : ( (propLocation == 2) ? PoseConstants.blueRight.spikeMarkAngleCenter : PoseConstants.blueRight.spikeMarkAngleRight);
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

            robot.setPixelServo(Constants.pixelDrop);
            sleep(500);

            robot.followTrajectory(toBackBoardTraj);

        } else {

            spikeMarkTraj = robot.trajectoryBuilder(startPose)
                    .splineTo(pixelVector, pixelAngle)
                    .build();

            afterSpikeMarkTraj = robot.trajectoryBuilder(new Pose2d(pixelVector, pixelAngle), afterPixelStartingTangent)
                    .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
                    .build();

            toBackBoardTraj = robot.trajectoryBuilder(new Pose2d(afterPixel, afterPixelAngle), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(0)), backdropTangent)
                    .build();

            robot.followTrajectory(spikeMarkTraj);

            robot.setPixelServo(Constants.pixelDrop);
            sleep(500);

            robot.followTrajectory(afterSpikeMarkTraj);
            robot.followTrajectory(toBackBoardTraj);

        }

    }


}
