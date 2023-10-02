package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous(name="TestAuto", group="Linear Opmode")
public class TestAuto extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {

        /* -------------------------------------------- INIT -------------------------------------------- */
        int propLocation;


        TrajectoryVelocityConstraint splineVelConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(Constants.slowerSplineVel),
                new AngularVelocityConstraint(16)
        ));

        PoseConstants.blueRight startPose = new PoseConstants.blueRight();
        robot.setPoseEstimate(PoseConstants.blueRight.start);


        // Wait for start button to be pressed
        while (!isStarted()) {
            propLocation = robot.getPropLocation();
            telemetry.addLine("Location: " + propLocation);
            telemetry.update();
        }

        /* -------------------------------------------- START -------------------------------------------- */

        propLocation = robot.getPropLocation();
        telemetry.addLine("Location: " + propLocation);
        telemetry.update();

        dropPixel(propLocation, "blueRight");

//        Trajectory backdropTraj = robot.trajectoryBuilder(robot.getPoseEstimate())
//                .splineTo(startPose.backdrop, Math.toRadians(0))
//                .build();
//
//        robot.followTrajectory(backdropTraj);

    }
}
