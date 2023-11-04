package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    /* -------------------------------------------- TELE OP CONSTANTS -------------------------------------------- */

    public static int startPos = 1;
    /*
        1: Blue Left
        2: Blue Right
        3: Red Left
        4: Red Right
     */


    /* -------------------------------------------- AUTO CONSTANTS -------------------------------------------- */

    public static int parkLoc = 1;
    public static boolean continueAutoAfterSpikeMark = true;
    public static int testPropLoc = 1;


    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */

    public static double moveSpeed = 1;
    public static double rotSpeed = 1;


    public static double slowerMoveVel = 10;
    public static double slowerSplineVel = 30;


    /* -------------------------------------------- SERVO CONSTANTS -------------------------------------------- */

    // TODO: Tune Values

    public static double pixelHold = 0.5;
    public static double pixelDrop = 1;

    public static double clawClose = 0.6;
    public static double clawOpen = 0.2;

    public static double clawArmDown = 0.9;
    public static double clawArmUp = 0.38;
    public static double clawArmHigh = 0.5;
    public static double clawArmLow = 0.83; // Arm position where it is safe to lower claw
    public static double clawArmDrive = 0.2;
    public static double clawArmSpeed = 0.003;
    public static double clawArmSlowRatio = 0.5;

    public static double turnClawUp = 0.75;
    public static double turnClawDown = 0.27;
    public static double turnClawSpeed = 0.01;

    public static double planeHold = 0.8;
    public static double planeRelease = 0.65;


    /* -------------------------------------------- MOTOR CONSTANTS -------------------------------------------- */


    public static int liftMin = 0;
    public static int liftLow = 100;
    public static int liftDropAuto = 700;
    public static int liftArm = 100;
    public static int liftHigh = 600;
    public static int liftMax = 2000;

    public static double liftUpRatio = 1;
    public static double liftDownRatio = 0.5;
    public static int liftSlow = 100;
    public static double liftSlowRatio = 0.4;

    public static int liftTolerance = 15;
    public static int liftkPTele = 10;
    public static double liftkP = 0.005; // 10
    public static double liftkI = 0;
    public static double liftkD = 0.001;
    public static double liftkF = 0.25;
    public static double liftMinPow = 0.1;




    public static int hangMin = 0;
    public static int hangLow = -1300;
    public static int hangHigh = -3000;
    public static int hangMax = -3100;

    public static double hangUpRatio = 1;
    public static double hangDownRatio = 0.8;
    public static int hangSlow = 100;
    public static double hangSlowRatio = 0.4;

    public static int hangTolerance = 15;
    public static int hangkPTele = 10;
    public static double hangkP = 0.005; // 10
    public static double hangkI = 0;
    public static double hangkD = 0.001;
    public static double hangkF = 0.25;
    public static double hangMinPow = 0.1;


    /* -------------------------------------------- VISION RECTANGLE CONSTANTS -------------------------------------------- */

    public static int rLx = 5;
    public static int rLy = 360;
    public static int rLw = 100;
    public static int rLh = 60;

    public static int rRx = 500;
    public static int rRy = 360;
    public static int rRw = 135;
    public static int rRh = 80;

    public static int rMx = 140;
    public static int rMy = 360;
    public static int rMw = 300;
    public static int rMh = 60;

    /* --------------------------------------------  APRILTAG CONSTANTS ------------------------------------------ */
    public static double DESIRED_DISTANCE = 12;
    public static double SPEED_GAIN  =  0.02;
    public static double STRAFE_GAIN =  0.015;
    public static double TURN_GAIN   =  0.01;
    public static double MAX_AUTO_SPEED = 0.5;
    public static double MAX_AUTO_STRAFE = 0.1;
    public static double MAX_AUTO_TURN  = 0.05;

}
