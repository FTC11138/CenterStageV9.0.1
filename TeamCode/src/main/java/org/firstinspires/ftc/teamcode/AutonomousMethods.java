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



    public void dropPixel(int propLocation, String startPosition) {
        Trajectory spikeMarkTraj = null;

        Pose2d startPose = null;
        Vector2d pixelVector = null;
        Vector2d afterPixel = null;
        double afterPixelAngle = 0;
        double pixelAngle = 0;


        if (startPosition == "redLeft") {
            startPose = PoseConstants.redLeft.start;
            pixelVector = (propLocation == 1) ? PoseConstants.redLeft.left : ((propLocation == 2) ? PoseConstants.redLeft.center : PoseConstants.redLeft.right);
            pixelAngle = PoseConstants.redLeft.spikeMarkAngle;
            afterPixel = PoseConstants.redLeft.afterSpikeMark;
            afterPixelAngle = PoseConstants.redLeft.afterSpikeMarkAngle;
        } else if (startPosition == "redRight") {
            startPose = PoseConstants.redRight.start;
            pixelVector = (propLocation == 1) ? PoseConstants.redRight.left : ( (propLocation == 2) ? PoseConstants.redRight.center : PoseConstants.redRight.right);
            pixelAngle = PoseConstants.redRight.spikeMarkAngle;
            afterPixel = PoseConstants.redRight.afterSpikeMark;
            afterPixelAngle = PoseConstants.redRight.afterSpikeMarkAngle;
        } else if (startPosition == "blueLeft") {
            startPose = PoseConstants.blueLeft.start;
            pixelVector = (propLocation == 1) ? PoseConstants.blueLeft.left : ( (propLocation == 2) ? PoseConstants.blueLeft.center : PoseConstants.blueLeft.right);
            pixelAngle = PoseConstants.blueLeft.spikeMarkAngle;
            afterPixel = PoseConstants.blueLeft.afterSpikeMark;
            afterPixelAngle = PoseConstants.blueLeft.afterSpikeMarkAngle;
        } else if (startPosition == "blueRight") {
            startPose = PoseConstants.blueRight.start;
            pixelVector = (propLocation == 1) ? PoseConstants.blueRight.left : ( (propLocation == 2) ? PoseConstants.blueRight.center : PoseConstants.blueRight.right);
            pixelAngle = PoseConstants.blueRight.spikeMarkAngle;
            afterPixel = PoseConstants.blueRight.afterSpikeMark;
            afterPixelAngle = PoseConstants.blueRight.afterSpikeMarkAngle;
        }

        spikeMarkTraj = robot.trajectoryBuilder(startPose)
                .splineTo(pixelVector, pixelAngle)
                .addDisplacementMarker(() -> {
                    robot.setPixelServo(Constants.pixelDrop);
                    sleep(500);
                })
                .splineTo(afterPixel, afterPixelAngle)
                .build();

        
        robot.followTrajectory(spikeMarkTraj);

    }


}
