package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.Const;

import java.util.Arrays;

@Autonomous(name="visionTestAuto", group="Linear Opmode")
@Disabled
public class visionTestAuto extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        int propLocation;

        robot.initialize(hardwareMap, telemetry, true);
        robot.setPoseEstimate(PoseConstants.redRight.start);
        propLocation = Constants.testPropLoc;

        while (!isStarted()) {;
            propLocation = robot.getPropLocation();
            telemetry.addLine("Location: " + propLocation);
            telemetry.update();
        }

        telemetry.addLine("Final Location: " + propLocation);
        telemetry.update();

        robot.setClawArmServo(Constants.clawArmLow);
        robot.setClawServo(Constants.clawClose);
        robot.setPixelServo(Constants.pixelHold);

        dropPixel_toBackdrop(propLocation, "redRight", Constants.continueAutoAfterSpikeMark);

        telemetry.addData("done with dropPixel", "moving on");
        telemetry.update();

        goToAprilTag( propLocation, "redRight", robot.visionPortal, robot.aprilTagProcessor);


        sleep(1000);

        robot.setClawArmServo(Constants.clawArmUp);
        robot.setTurnClawServo(Constants.turnClawUp);
        sleep(1000);
        robot.setLiftMotor(1, Constants.liftDropAuto);
        sleep(4000);
        robot.setClawServo(Constants.clawOpen);
        sleep(500);

        robot.setLiftMotor(1, Constants.liftLow);
        robot.setClawArmServo(Constants.clawArmLow);
        sleep(2000);
        robot.setTurnClawServo(Constants.turnClawDown);
        toPark(1, "redRight");
        sleep(5000);


    }
}
