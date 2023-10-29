package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.Const;

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
        propLocation = Constants.testPropLoc;

        while (!isStarted()) {;
            //propLocation = robot.getPropLocation();
            telemetry.addLine("Location: " + propLocation);
            telemetry.update();
        }

        //propLocation = robot.getPropLocation();
        //telemetry.addLine("Location: " + propLocation);
        //telemetry.update();

        robot.setClawArmServo(Constants.clawArmDrive);
        robot.setPixelServo(Constants.pixelHold);
        robot.setClawServo(Constants.clawClose);

//        dropPixel_toBackdrop(propLocation, "blueRight");


        //going to detect point
        /**
        PoseConstants.aprilTag detectPose = new PoseConstants.aprilTag();
        Trajectory detectTraj = robot.trajectoryBuilder(robot.getPoseEstimate())
                .splineTo(detectPose.detect, Math.toRadians(0))
                .build();
        **/

        telemetry.addData("done with dropPixel", "moving on");
        telemetry.update();

//        roadrunnerToBackdrop(propLocation, "blueRight");

//        goToAprilTag(propLocation, "blueRight", robot.visionPortal, robot.aprilTagProcessor);


        sleep(3000);

        robot.setPixelServo(Constants.pixelDrop);
        sleep(500);
        robot.setPixelServo(Constants.pixelHold);

        sleep(3000);

        robot.setLiftMotor(1, Constants.liftDropAuto);
        robot.setClawArmServo(Constants.clawArmHigh);
        sleep(2000);
        robot.setClawArmServo(Constants.clawArmUp);
        sleep(1000);
        robot.setClawServo(Constants.clawOpen);
        sleep(500);

        Trajectory toPark = robot.trajectoryBuilder(robot.getPoseEstimate())
                .splineTo(PoseConstants.blueRight.parkRight, Math.toRadians(0))
                .build();

        robot.setLiftMotor(1, Constants.liftLow);
        robot.setClawArmServo(Constants.clawArmDrive);
//        robot.followTrajectory(toPark);
        sleep(5000);


    }
}
