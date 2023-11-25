package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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


        robot.setClaw1Servo(Constants.clawClose);
        robot.setClaw2Servo(Constants.clawClose);
        robot.setPixelServo(Constants.pixelHold);

        runAuto(
                propLocation,
                startPos,

                poses.start,
                poses.startingTangent[propLocation - 1],

                poses.pixel[propLocation - 1],
                poses.pixelAngle[propLocation - 1],
                poses.pixelApproachingTangent[propLocation - 1],

                poses.afterPixelStartingTangent[propLocation - 1],
                poses.afterPixelEndingTangent[propLocation - 1],
                poses.afterPixel,
                poses.afterPixelAngle[propLocation - 1],

                poses.backdrop[propLocation - 1],
                poses.backdropTangent,

                poses.park[parkLocation - 1],
                poses.parkAngle[parkLocation - 1],
                poses.parkStartingTangent[parkLocation - 1],
                poses.parkEndingTangent[parkLocation - 1]
        );

        PoseStorage.currentPose = robot.getPoseEstimate();
        PoseStorage.fieldCentricOffset = Math.toRadians(-90);
        robot.setClawArmServo(Constants.clawArmDown);
        robot.setClaw1Servo(Constants.clawOpen);
        sleep(2000);

    }
}
