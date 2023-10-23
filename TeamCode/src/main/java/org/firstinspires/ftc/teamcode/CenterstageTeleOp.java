package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tele Op", group = "Iterative Opmode")
public class CenterstageTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();



    private double clawPosition = Constants.clawOpen;
    private double clawArmPosition = Constants.clawArmDown;
    private double planePosition = Constants.planeHold;

    private double liftPower = 0;
    private boolean useLiftPower = true;
    private boolean liftModeUpdate = false;
    private boolean liftUseEnc = true;
    private int targetLiftPosition = Constants.liftLow;
    private int currentLiftPosition = robot.getLiftMotorPosition();

    private double hangPower = 0;
    private boolean useHangPower = true;
    private boolean hangModeUpdate = false;
    private boolean hangUseEnc = true;
    private int targetHangPosition = Constants.hangLow;
    private int currentHangPosition = robot.getHangMotorPosition();

    private boolean limits = true;


    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry, false);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
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
        } else if (gamepad2.a) {
            clawArmPosition = Constants.clawArmDown;
        }

        if (gamepad2.x || gamepad1.x) {
            planePosition = Constants.planeRelease;
        }

        if (gamepad2.dpad_up) {
            targetLiftPosition = Constants.liftHigh;
        } else if (gamepad2.dpad_down) {
            targetLiftPosition = Constants.liftLow;
        }

        if (gamepad2.y) {
            targetHangPosition = Constants.hangHigh;
        } else if (gamepad2.x) {
            targetHangPosition = Constants.hangLow;
        }


        if (gamepad2.dpad_left) {
            targetLiftPosition = Constants.liftHigh;
            clawArmPosition = Constants.clawArmUp;
        } else if (gamepad2.dpad_right) {
            targetLiftPosition = Constants.liftLow;
            clawArmPosition = Constants.clawArmDown;
        }


        double liftJoystick = gamepad2.left_stick_y;
        if (liftJoystick < -0.12) {
            liftUseEnc = true;
            // user trying to lift up
            if (currentLiftPosition < Constants.liftMax || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftUpRatio;
            } else {
                liftPower = 0;
            }
        } else if (liftJoystick > 0.12) {
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


        double hangJoystick = gamepad2.right_stick_y;
        if (hangJoystick < -0.12) {
            hangUseEnc = true;
            // user trying to lift up
            if (currentHangPosition < Constants.hangMax || !limits) {
                useHangPower = true;
                hangPower = hangJoystick * Constants.hangUpRatio;
            } else {
                hangPower = 0;
            }
        } else if (hangJoystick > 0.12) {
            hangUseEnc = true;
            // user trying to lift down
            if (currentHangPosition > Constants.hangMin || !limits) {
                useHangPower = true;
                hangPower = hangJoystick * Constants.hangDownRatio;
                if (currentHangPosition > Constants.hangSlow) {
                    hangPower *= Constants.hangSlowRatio;
                }
            } else {
                hangPower = 0;
            }
        } else if (useHangPower) {
            hangPower = 0;
        }


        double clawArmJoystick = -gamepad2.right_stick_y;
        if (clawArmJoystick > 0.2) {
            // User trying to push the clawArm up by pushing the joystick up
            if ((clawArmPosition + Constants.clawArmSpeed) < Constants.clawArmUp) { // making sure servo value doesnt go higher than max
                clawArmPosition += Constants.clawArmSpeed * clawArmJoystick;
            } else {
                clawArmPosition = Constants.clawArmUp;
            }
        } else if (clawArmJoystick < -0.2) {
            // User trying to pull the clawArm down by pushing the joystick down
            if ((clawArmPosition - Constants.clawArmSpeed) > Constants.clawArmDown) { // making sure servo value doesnt go into negative
                clawArmPosition += Constants.clawArmSpeed * clawArmJoystick; // not doing -= because clawArmJoystick is already negative
            } else {
                clawArmPosition = Constants.clawArmDown;
            }
        }



        /* -------------------------------------------- ACTION -------------------------------------------- */
        robot.setClawServo(clawPosition);
        robot.setClawArmServo(clawArmPosition);
        robot.setPlaneServo(planePosition);

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
            robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangModeUpdate = false;
        }

        if (useHangPower) {
            robot.runHangMotor(hangPower);
        } else {
            setHangMotor(targetHangPosition);
        }


    }

    void setLiftMotor(int position) {
        //Undefined constants
        double newPower, powDir;
        //Initial error
        double error = -(position - currentLiftPosition) / Constants.liftMax;
        //Initial Time
        telemetry.addData("1", "error: " + error);
        if (Math.abs(error) > (Constants.liftTolerance / -Constants.liftMax)) {
            //Setting p action
            newPower = error * Constants.liftkPTele;
            powDir = Math.signum(error);
            newPower = Math.min(Math.abs(newPower), 1);

            // Going down
            if (powDir == 1) {
                newPower = Math.max(newPower * Constants.liftDownRatio, Constants.liftMinPow);
                if (currentLiftPosition > Constants.liftSlow) {
                    newPower *= Constants.liftSlowRatio;
                }
            }
            // Going up
            else {
                newPower = Math.min(-newPower, -Constants.liftMinPow - Constants.liftkF * currentLiftPosition / Constants.liftMax);
            }
            telemetry.addData("Lift Motor", newPower);
            robot.runLiftMotor(newPower);
        } else if (position != 0 && !liftModeUpdate) {
            robot.setLiftMotor(0.5, position);
            liftModeUpdate = true;
            liftUseEnc = false;
        } else if (!liftModeUpdate){
            robot.runLiftMotor(0);
        }
    }

    void setHangMotor(int position) {
        //Undefined constants
        double newPower, powDir;
        //Initial error
        double error = -(position - currentHangPosition) / Constants.hangMax;
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
                newPower = Math.min(-newPower, -Constants.hangMinPow - Constants.hangkF * currentHangPosition / Constants.hangMax);
            }
            telemetry.addData("Hang Motor", newPower);
            robot.runHangMotor(newPower);
        } else if (position != 0 && !liftModeUpdate) {
            robot.setHangMotor(0.5, position);
            hangModeUpdate = true;
            hangUseEnc = false;
        } else if (!liftModeUpdate){
            robot.runHangMotor(0);
        }
    }

}
