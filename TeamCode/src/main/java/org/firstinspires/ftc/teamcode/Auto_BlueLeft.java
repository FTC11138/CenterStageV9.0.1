package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.Const;

import java.util.Arrays;

@Autonomous(name="Auto_BlueLeft", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class Auto_BlueLeft extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        int propLocation = 2;
        int parkLocation = Constants.parkLoc;
        String startPos = "blueLeft";
        boolean targetFound = true;

        robot.initialize(hardwareMap, telemetry, true);
        PoseConstants.blueLeft poses = new PoseConstants.blueLeft();

        while (!isStarted()) {;
            propLocation = robot.getPropLocation();
            telemetry.addLine("Location: " + propLocation);
            telemetry.update();
        }
        int finalPropLocation = propLocation;
        telemetry.addLine("Final Location: " + propLocation);
        telemetry.update();

        robot.setClawArmServo(Constants.clawArmLow);
        robot.setClawServo(Constants.clawClose);
        robot.setPixelServo(Constants.pixelHold);
        robot.setPlaneServo(Constants.planeHold);

        runAuto(
                propLocation,
                startPos,

                poses.start,
                poses.startingTangent[propLocation - 1],

                poses.pixel[propLocation - 1],
                poses.pixelAngle[propLocation - 1],
                poses.pixelApproachingTangent[propLocation - 1],

                poses.backdrop[propLocation - 1],
                poses.backdropTangent,

                poses.park[parkLocation - 1],
                poses.parkAngle[parkLocation - 1],
                poses.parkStartingTangent[parkLocation - 1],
                poses.parkEndingTangent[parkLocation - 1]
        );

        PoseStorage.currentPose = robot.getPoseEstimate();
        robot.setClawArmServo(Constants.clawArmDown);
        robot.setClawServo(Constants.clawOpen);
        sleep(2000);

    }
}
