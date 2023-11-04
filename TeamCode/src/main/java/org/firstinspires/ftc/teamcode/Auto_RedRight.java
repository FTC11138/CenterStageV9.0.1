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

@Autonomous(name="Auto_RedRight", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class Auto_RedRight extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        int propLocation = 2;
        String startPos = "redRight";
        boolean targetFound = true;

        robot.initialize(hardwareMap, telemetry, true);
        robot.setPoseEstimate(PoseConstants.redRight.start);

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

        dropPixel_toBackdrop(propLocation, startPos, Constants.continueAutoAfterSpikeMark);
        telemetry.addData("done with dropPixel", "moving on");
        telemetry.update();

        targetFound = goToAprilTag(propLocation, startPos, robot.visionPortal, robot.aprilTagProcessor);

        sleep(1000);

        if (targetFound) {

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

        }

        toPark(Constants.parkLoc, startPos);
        robot.setClawServo(Constants.clawOpen);

    }
}
