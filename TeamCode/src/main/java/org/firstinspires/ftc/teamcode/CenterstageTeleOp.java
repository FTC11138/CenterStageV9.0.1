package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;

@TeleOp(name = "Tele Op", group = "Iterative Opmode")
public class CenterstageTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();



    private double clawPosition = Constants.clawOpen;
    private double clawArmPosition = Constants.clawArmDown;
    private double planePosition = Constants.planeHold;
    private double pixelPosition = Constants.pixelHold;
    private double liftPower = 0;
    private boolean useLiftPower = true;
    private boolean liftModeUpdate = false;
    private boolean liftUseEnc = true;
    private int targetLiftPosition = Constants.liftLow;
    private int currentLiftPosition = 0;

    private double hangPower1 = 0;
    private double hangPower2 = 0;
    private boolean useHangPower = true;
    private boolean hangModeUpdate = false;
    private boolean hangUseEnc = true;
    private int targetHangPosition = Constants.hangLow;
    private int currentHangPosition1 = 0;
    private int currentHangPosition2 = 0;

    private boolean limits = true;

    private int stage = -1;
    private int stageCounter = 0;

    StandardTrackingWheelLocalizer localizer;
    Pose2d currentPose;
    double xPos;
    double yPos;
    double heading;


    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry, false);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());

        if (Constants.startPos == 1) {
            localizer.setPoseEstimate(PoseConstants.blueLeft.start);
        } else if (Constants.startPos == 2) {
            localizer.setPoseEstimate(PoseConstants.blueRight.start);
        } else if (Constants.startPos == 3) {
            localizer.setPoseEstimate(PoseConstants.redLeft.start);
        } else {
            localizer.setPoseEstimate(PoseConstants.redRight.start);
        }

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {


        localizer.update();
        currentPose = localizer.getPoseEstimate();
        heading = currentPose.getHeading();
        xPos = currentPose.getX();
        yPos = currentPose.getY();

        double joyX = gamepad1.left_stick_x;
        double joyY = -gamepad1.left_stick_y;
        double joyAngle = Math.atan(joyY/joyX);
        double robotAngle = heading;
        double moveAngle = joyAngle - robotAngle;
        double ly = Math.sin(moveAngle) * joyY;
        double lx = Math.cos(moveAngle) * joyX;


//        lx = gamepad1.left_stick_x;
//        ly = -gamepad1.left_stick_y;
        double speedMultiplier = Constants.moveSpeed;
        double rotationMultiplier = Constants.rotSpeed;

        // D-pad
        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = 0.3;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = 0.3;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = 0.3;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = 0.3;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        robot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);


        /* -------------------------------------------- CHANGE -------------------------------------------- */


        if (gamepad2.left_bumper) {
            clawPosition = Constants.clawOpen;
        } else if (gamepad2.right_bumper) {
            clawPosition = Constants.clawClose;
        }

        if (gamepad2.b) {
            clawArmPosition = Constants.clawArmUp;
            //stage = 2;
        } else if (gamepad2.a) {
           clawArmPosition = Constants.clawArmDown;
            //stage = 1;
        }

        if (gamepad1.right_trigger > 0.2|| gamepad2.right_trigger > 0.2) {
            planePosition = Constants.planeRelease;
        }
        else if (gamepad1.left_trigger > 0.2|| gamepad2.left_trigger > 0.2) {
            planePosition = Constants.planeHold;
        }

        if (gamepad1.x) {
            pixelPosition = Constants.pixelHold;
        } else if (gamepad1.y) {
            pixelPosition = Constants.pixelDrop;
        }

        if (gamepad2.dpad_up) {
            useLiftPower = false;
            targetLiftPosition = Constants.liftHigh;
            liftUseEnc = true;
        } else if (gamepad2.dpad_down) {
            useLiftPower = false;
            targetLiftPosition = Constants.liftLow;
            liftUseEnc = true;
        }

        if (gamepad2.y) {
            useHangPower = false;
            targetHangPosition = Constants.hangHigh;
            hangUseEnc = true;
        } else if (gamepad2.x) {
            useHangPower = false;
            targetHangPosition = Constants.hangLow;
            hangUseEnc = true;
        }


        if (gamepad2.dpad_right) {
            useLiftPower = false;
            targetLiftPosition = Constants.liftHigh;
            liftUseEnc = true;
            clawArmPosition = Constants.clawArmUp;
        } else if (gamepad2.dpad_left) {
            useLiftPower = false;
            targetLiftPosition = Constants.liftLow;
            liftUseEnc = true;
            clawArmPosition = Constants.clawArmDown;
        }


        if (gamepad1.right_bumper) {
            limits = true;
        } else if (gamepad1.left_bumper) {
            limits = false;
        }

        if (liftModeUpdate && liftUseEnc) {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftModeUpdate = false;
        }

        if (stage > 0) {
            switch (stage) {
                case 1:
                    // Bring Arm Down
                    if ((clawArmPosition + Constants.clawArmSpeed) < Constants.clawArmDown) {
                        if (clawArmPosition > Constants.clawArmDrive) {
                            clawArmPosition += Constants.clawArmSpeed * Constants.clawArmSlowRatio;
                        } else {
                            clawArmPosition += Constants.clawArmSpeed;
                        }
                    } else {
                        stage = -1;
                    }
                    break;
                case 2:
                    // Bring Arm Up
                    if ((clawArmPosition - Constants.clawArmSpeed) > Constants.clawArmUp) {
                        clawArmPosition -= Constants.clawArmSpeed;
                    } else {
                        stage = -1;
                    }
                    break;
            }
        }

        if (stageCounter > 0) {
            stageCounter--;
        }


        double liftJoystick = -gamepad2.left_stick_y;
        if (liftJoystick > 0.12) {
            liftUseEnc = true;
            // user trying to lift up
            if (currentLiftPosition < Constants.liftMax || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftUpRatio;
            } else {
                liftPower = 0;
            }
        } else if (liftJoystick < -0.12) {
            liftUseEnc = true;
            // user trying to lift down
            if (currentLiftPosition > Constants.liftMin || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftDownRatio;
                if (currentLiftPosition > Constants.liftSlow) {
                    liftPower *= Constants.liftSlowRatio;
                }
            } else {
                liftPower = 0;
            }
        } else if (useLiftPower) {
            liftPower = 0;
        }



        double hangJoystick = -gamepad2.right_stick_y;
        if (hangJoystick > 0.12) {
            hangUseEnc = true;
            // user trying to lift up
            if (currentHangPosition1 > Constants.hangMax || !limits) {
                useHangPower = true;
                hangPower1 = hangJoystick * Constants.hangUpRatio * -1;
            } else {
                hangPower1 = 0;
            }

            if (currentHangPosition2 > Constants.hangMax || !limits) {
                useHangPower = true;
                hangPower2 = hangJoystick * Constants.hangUpRatio * -1;
            } else {
                hangPower2 = 0;
            }

        } else if (hangJoystick < -0.12) {
            hangUseEnc = true;
            // user trying to lift down
            if (currentHangPosition1 < Constants.hangMin || !limits) {
                useHangPower = true;
                hangPower1 = hangJoystick * Constants.hangDownRatio * -1;
            } else {
                hangPower1 = 0;
            }

            if (currentHangPosition2 < Constants.hangMin || !limits) {
                useHangPower = true;
                hangPower2 = hangJoystick * Constants.hangDownRatio * -1;
            } else {
                hangPower2 = 0;
            }

        } else if (useHangPower) {
            hangPower1 = 0;
            hangPower2 = 0;
        }



        double clawArmJoystick = gamepad2.right_stick_x;
        if (clawArmJoystick > 0.2) {
            if ((clawArmPosition - Constants.clawArmSpeed) > Constants.clawArmUp) {
                clawArmPosition -= Constants.clawArmSpeed * clawArmJoystick;
            } else {
                clawArmPosition = Constants.clawArmUp;
            }
        } else if (clawArmJoystick < -0.2) {
            if ((clawArmPosition + Constants.clawArmSpeed) < Constants.clawArmDown) {
                clawArmPosition -= Constants.clawArmSpeed * clawArmJoystick;
            } else {
                clawArmPosition = Constants.clawArmDown;
            }
        }



        currentLiftPosition = robot.getLiftMotorPosition();
        currentHangPosition1 = robot.getHangMotorPosition1();
        currentHangPosition2 = robot.getHangMotorPosition2();



        /* -------------------------------------------- ACTION -------------------------------------------- */

        robot.setClawServo(clawPosition);
        robot.setClawArmServo(clawArmPosition);
        robot.setPlaneServo(planePosition);
        robot.setPixelServo(pixelPosition);

        if (liftModeUpdate && liftUseEnc) {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftModeUpdate = false;
        }

        if (useLiftPower) {
            robot.runLiftMotor(liftPower);
        } else {
            setLiftMotor(targetLiftPosition);
        }

        if (hangModeUpdate && hangUseEnc) {
            robot.hangMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hangMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangModeUpdate = false;
        }

        if (useHangPower) {
            robot.runHangMotor1(hangPower1);
            robot.runHangMotor2(hangPower2);
        } else {
            setHangMotor(targetHangPosition);
        }


        /* -------------------------------------------- TELEMETRY -------------------------------------------- */

        telemetry.addData("X Joy", joyX);
        telemetry.addData("Y Joy", joyY);
        telemetry.addData("Lift Joy", liftJoystick);
        telemetry.addData("Hang Joy", hangJoystick);
        telemetry.addData("Claw Arm Joy", clawArmJoystick);
        telemetry.addLine();
        telemetry.addData("Lift Pos", currentLiftPosition);
        telemetry.addData("Hang Motor 1 Pos", currentHangPosition1);
        telemetry.addData("Hang Motor 2 Pos", currentHangPosition2);
        telemetry.addData("Claw Arm Pos", clawArmPosition);
        telemetry.addData("Plane Pos", planePosition);
        telemetry.addData("Pixel Pos", pixelPosition);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Limits", limits);
        telemetry.update();


    }

    void setLiftMotor(int position) {
        robot.setLiftMotor(1, position);
        liftUseEnc = false;
        liftModeUpdate = true;
    }

//    void setLiftMotor(int position) {
//        //Undefined constants
//        double newPower, powDir;
//        //Initial error
//        double error = -(position - currentLiftPosition) / Constants.liftMax;
//        //Initial Time
//        telemetry.addData("1", "error: " + error);
//        if (Math.abs(error) > (Constants.liftTolerance / -Constants.liftMax)) {
//            //Setting p action
//            newPower = error * Constants.liftkPTele;
//            powDir = Math.signum(error);
//            newPower = Math.min(Math.abs(newPower), 1);
//
//            // Going down
//            if (powDir == 1) {
//                newPower = Math.max(newPower * Constants.liftDownRatio, Constants.liftMinPow);
//                if (currentLiftPosition > Constants.liftSlow) {
//                    newPower *= Constants.liftSlowRatio;
//                }
//            }
//            // Going up
//            else {
//                newPower = Math.min(-newPower, -Constants.liftMinPow - Constants.liftkF * currentLiftPosition / Constants.liftMax);
//            }
//            telemetry.addData("Lift Motor", newPower);
//            robot.runLiftMotor(newPower);
//        } else if (position != 0 && !liftModeUpdate) {
//            robot.setLiftMotor(0.5, position);
//            liftModeUpdate = true;
//            liftUseEnc = false;
//        } else if (!liftModeUpdate){
//            robot.runLiftMotor(0);
//        }
//    }

    void setHangMotor(int position) {
        //Undefined constants
        double newPower, powDir;
        //Initial error
        double error = -(position - currentHangPosition1) / Constants.hangMax;
        //Initial Time
        telemetry.addData("1", "error: " + error);
        if (Math.abs(error) > (Constants.hangTolerance / -Constants.hangMax)) {
            //Setting p action
            newPower = error * Constants.hangkPTele;
            powDir = Math.signum(error);
            newPower = Math.min(Math.abs(newPower), 1);

            // Going down
            if (powDir == 1) {
                newPower = Math.max(newPower * Constants.hangDownRatio, Constants.hangMinPow);
                if (currentLiftPosition > Constants.hangSlow) {
                    newPower *= Constants.hangSlowRatio;
                }
            }
            // Going up
            else {
                newPower = Math.min(-newPower, -Constants.hangMinPow - Constants.hangkF * currentHangPosition1 / Constants.hangMax);
            }
            telemetry.addData("Hang Motor", newPower);
            robot.runHangMotor1(newPower);
            robot.runHangMotor2(newPower);
        } else if (position != 0 && !liftModeUpdate) {
            robot.setHangMotor(0.5, position);
            hangModeUpdate = true;
            hangUseEnc = false;
        } else if (!liftModeUpdate){
            robot.runHangMotor1(0);
            robot.runHangMotor2(0);
        }
    }

}
