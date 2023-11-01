package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Attachments extends SampleMecanumDrive {


    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    public Servo clawServo, clawArmServo, planeServo, pixelServo;
    public DcMotorEx liftMotor, hangMotor1, hangMotor2;
    public WebcamName webcam;
    public VisionProcessor visionProcessor;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_, boolean auto) {

        // Random Stuff IDK
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();


        // Initialize Roadrunner
        initializeRoadrunner(hardwareMap);

        telemetry.addLine("Roadrunner Initialized");
        telemetry.update();

        // Motors
        liftMotor = hardwareMap.get(DcMotorEx.class, names.liftMotor);
        hangMotor1 = hardwareMap.get(DcMotorEx.class, names.hangMotor1);
        hangMotor2 = hardwareMap.get(DcMotorEx.class, names.hangMotor2);
        hangMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Motors Initialized");
        telemetry.update();


        // Servos
//        clawServo = hardwareMap.get(Servo.class, names.clawServo);
//        clawArmServo = hardwareMap.get(Servo.class, names.clawArmServo);
        planeServo = hardwareMap.get(Servo.class, names.planeServo);
        pixelServo = hardwareMap.get(Servo.class, names.pixelServo);

        telemetry.addLine("Servos Initialized");
        telemetry.update();


        // Camera
        webcam = hardwareMap.get(WebcamName.class, names.webcam);

        telemetry.addLine("Camera Initialized");
        telemetry.update();

        // Change Drive Motor Modes if not autonomous
        if (!auto) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            visionProcessor = new VisionProcessor();
            // visionPortal = VisionPortal.easyCreateWithDefaults(webcam, visionProcessor);
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
            aprilTagProcessor.setDecimation(2);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .addProcessor(visionProcessor)
                    .build();
        }
    }

    // Run Motors
    public void runLiftMotor(double power) {
        liftMotor.setPower(power);
    }
    public void runHangMotor1(double power) {
        hangMotor1.setPower(power);
    }

    public void runHangMotor2(double power) {
        hangMotor2.setPower(power);
    }



    // Set Motors
    public void setLiftMotor(double power, int position) {
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHangMotor(double power, int position) {
        hangMotor1.setPower(power);
        hangMotor2.setPower(power);
        hangMotor1.setTargetPosition(position);
        hangMotor2.setTargetPosition(position);
        hangMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    // Set Servos
    public void setClawServo (double position) {clawServo.setPosition(position);}
    public void setClawArmServo (double position) {clawArmServo.setPosition(position);}
    public void setPlaneServo (double position) {planeServo.setPosition(position);}
    public void setPixelServo (double position) {pixelServo.setPosition(position);}


    // Get Motor Positions
    public int getLiftMotorPosition() {
        return liftMotor.getCurrentPosition();
    }
    public int getHangMotorPosition1() {
        return hangMotor1.getCurrentPosition();
    }

    public int getHangMotorPosition2() {
        return hangMotor2.getCurrentPosition();
    }

    // Get Servo Positions
    public double getClawPosition() {
        return clawServo.getPosition();
    }
    public double getClawArmPosition() {
        return clawArmServo.getPosition();
    }
    public double getPlanePosition() {
        return planeServo.getPosition();
    }
    public double getPixelPosition() {
        return pixelServo.getPosition();
    }

    // Getting Prop Location
    public int getPropLocation() {
        VisionProcessor.Selected selection = visionProcessor.getSelection();
        if (selection == VisionProcessor.Selected.LEFT) {
            return 1;
        } else if (selection == VisionProcessor.Selected.MIDDLE) {
            return 2;
        } else if (selection == VisionProcessor.Selected.RIGHT) {
            return 3;
        } else {
            return 3;
        }
    }
}