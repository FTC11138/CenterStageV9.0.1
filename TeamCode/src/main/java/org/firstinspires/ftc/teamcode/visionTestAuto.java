package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous(name="visionTestAuto", group="Linear Opmode")
public class visionTestAuto extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        int propLocation;


        TrajectoryVelocityConstraint splineVelConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(Constants.slowerSplineVel),
                new AngularVelocityConstraint(16)
        ));

        //robot.initialize(hardwareMap, telemetry, true);
        robot.initialize(hardwareMap, telemetry, true);
        PoseConstants.blueRight startPose = new PoseConstants.blueRight();
        robot.setPoseEstimate(PoseConstants.blueRight.start);
        propLocation = 2;

        while (!isStarted()) {;
            //propLocation = robot.getPropLocation();
            telemetry.addLine("Location: " + propLocation);
            telemetry.update();
        }

        //propLocation = robot.getPropLocation();
        propLocation = 3;
        //telemetry.addLine("Location: " + propLocation);
        //telemetry.update();

        dropPixel_toBackdrop(propLocation, "blueRight");


        //going to detect point
        /**
        PoseConstants.aprilTag detectPose = new PoseConstants.aprilTag();
        Trajectory detectTraj = robot.trajectoryBuilder(robot.getPoseEstimate())
                .splineTo(detectPose.detect, Math.toRadians(0))
                .build();
        **/

        telemetry.addData("done with dropPixel", "moving on");
        telemetry.update();
        goToAprilTag(propLocation, "blueRight", robot.visionPortal, robot.aprilTagProcessor);
    }
}
