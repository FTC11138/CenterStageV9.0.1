package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class PoseConstants {

    public static class aprilTag {
        public static Vector2d detect = new Vector2d(2, 12);
    }

    public static Vector2d defaultVector = new Vector2d(0,0);
    public static Pose2d defaultPose = new Pose2d(0,0, 0);

    @Config
    public static class redRight {

        public boolean backdropSide = true;

        public static Pose2d start = new Pose2d(12, -63, Math.toRadians(-90));
        public static double[] startingTangent = {Math.toRadians(45), Math.toRadians(45), Math.toRadians(45)};

        public static Vector2d[] pixel = {new Vector2d(12, -32), new Vector2d(12, -13), new Vector2d(12, -32)};
        public static double[] pixelAngle = {Math.toRadians(0), Math.toRadians(90), Math.toRadians(180)};
        public static double[] pixelApproachingTangent = {Math.toRadians(135), Math.toRadians(135), Math.toRadians(135)};

        public static double[] afterPixelStartingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};
        public static double[] afterPixelEndingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};

        public static Vector2d afterPixel = new Vector2d(12, -7);;
        public static double[] afterPixelAngle = {Math.toRadians(0), Math.toRadians(90), Math.toRadians(180)};

        public static Vector2d[] backdrop = {new Vector2d(36, -34), new Vector2d(36, -36), new Vector2d(36, -38)};
        public static double backdropTangent = Math.toRadians(-90);

        public Vector2d[] park = {new Vector2d(60, -10), new Vector2d(60, -60)};
        public double[] parkStartingTangent ={Math.toRadians(135), Math.toRadians(-135)};
        public double[] parkEndingTangent = {Math.toRadians(0), Math.toRadians(0)};
        public double[] parkAngle = {Math.toRadians(0), Math.toRadians(0)};

    }

    public static class blueLeft {

        public boolean backdropSide = true;

        public static Pose2d start = new Pose2d(12, 63, Math.toRadians(90));
        public static double[] startingTangent = {Math.toRadians(-45), Math.toRadians(-45), Math.toRadians(-45)};

        public static Vector2d[] pixel = {new Vector2d(12, 36), new Vector2d(12, 13), new Vector2d(12, 32)};
        public static double[] pixelAngle = {Math.toRadians(180), Math.toRadians(-90), Math.toRadians(0)};
        public static double[] pixelApproachingTangent = {Math.toRadians(-135), Math.toRadians(-135), Math.toRadians(-135)};

        public static double[] afterPixelStartingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public static double[] afterPixelEndingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public static Vector2d afterPixel = new Vector2d(12, 7);;
        public static double[] afterPixelAngle = {Math.toRadians(180), Math.toRadians(-90), Math.toRadians(0)};

        public Vector2d stack = new Vector2d(-62, 36);
        public double stackAngle = Math.toRadians(180);
        public double stackApproachingTangent = Math.toRadians(180);
        public double stackLeavingTangent = Math.toRadians(0);
        public Vector2d beforeStack = new Vector2d(-36, 12);
        public Vector2d stack2 = new Vector2d(-62, 13);

        public Vector2d beforeBackdrop = new Vector2d(12, 11);
        public static Vector2d[] backdrop = {new Vector2d(36, 38), new Vector2d(36, 36), new Vector2d(36, 34)};
        public double backdropTangent = Math.toRadians(90);

        public Vector2d[] park = {new Vector2d(60, 10), new Vector2d(60, 60)};
        public double[] parkStartingTangent ={Math.toRadians(-135), Math.toRadians(135)};
        public double[] parkEndingTangent = {Math.toRadians(0), Math.toRadians(0)};
        public double[] parkAngle = {Math.toRadians(0), Math.toRadians(0)};

    }

    public static class redLeft {

        public boolean backdropSide = false;

        public Pose2d start = new Pose2d(-36, -63, Math.toRadians(-90));
        public double[] startingTangent = {Math.toRadians(180), Math.toRadians(90), Math.toRadians(135)};

        public Vector2d[] pixel = {new Vector2d(-59, -32), new Vector2d(-36, -36), new Vector2d(-36, -32)};
        public double[] pixelAngle = {Math.toRadians(180), Math.toRadians(-90), Math.toRadians(180)};
        public double[] pixelApproachingTangent = {Math.toRadians(180), Math.toRadians(90), Math.toRadians(45)};

        public double[] afterPixelStartingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};
        public double[] afterPixelEndingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};
        public Vector2d[] afterPixel = {new Vector2d(-56, -11), new Vector2d(-56, -11), new Vector2d(-56, -11)};
        public double[] afterPixelAngle = {Math.toRadians(180), Math.toRadians(180), Math.toRadians(180)};

        public Vector2d stack = new Vector2d(-62, -36);
        public double stackAngle = Math.toRadians(180);
        public double stackApproachingTangent = Math.toRadians(180);
        public double stackLeavingTangent = Math.toRadians(0);
        public Vector2d beforeStack = new Vector2d(-36, -12);
        public Vector2d[] stack2 = {new Vector2d(-61.5, -13.5), new Vector2d(-61.5, -13.5), new Vector2d(-61.5, -14)};

        public Vector2d beforeBackdrop = new Vector2d(12, -11);
        public double[] beforeBackdropAngle = {Math.toRadians(180), Math.toRadians(180), Math.toRadians(180)};
        public Vector2d[] backdrop = {new Vector2d(36, -34), new Vector2d(36, -36), new Vector2d(36, -38)};
        public double backdropTangent = Math.toRadians(-90);

        public Vector2d[] park = {new Vector2d(60, -10), new Vector2d(60, -60)};
        public double[] parkStartingTangent ={Math.toRadians(135), Math.toRadians(-135)};
        public double[] parkEndingTangent = {Math.toRadians(0), Math.toRadians(0)};
        public double[] parkAngle = {Math.toRadians(0), Math.toRadians(0)};

    }

    public static class blueRight {

        public boolean backdropSide = false;

        public Pose2d start = new Pose2d(-36, 63, Math.toRadians(90));
        public double[] startingTangent = {Math.toRadians(-135), Math.toRadians(-90), Math.toRadians(180)};

        public Vector2d[] pixel = {new Vector2d(-38, 32), new Vector2d(-36, 36), new Vector2d(-59, 36)};
        public double[] pixelAngle = {Math.toRadians(180), Math.toRadians(90), Math.toRadians(180)};
        public double[] pixelApproachingTangent = {Math.toRadians(-45), Math.toRadians(-90), Math.toRadians(180)};

        public double[] afterPixelStartingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public double[] afterPixelEndingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public Vector2d[] afterPixel = {new Vector2d(-58, 11), new Vector2d(-58, 11), new Vector2d(-58, 11)};
        public double[] afterPixelAngle = {Math.toRadians(180), Math.toRadians(180), Math.toRadians(180)};

        public Vector2d stack = new Vector2d(-62, 36);
        public double stackAngle = Math.toRadians(180);
        public double stackApproachingTangent = Math.toRadians(180);
        public double stackLeavingTangent = Math.toRadians(0);
        public Vector2d beforeStack = new Vector2d(-36, 12);
        public Vector2d[] stack2 = {new Vector2d(-61.5, 14), new Vector2d(-61.5, 13.5), new Vector2d(-61.5, 13.5)};

        public Vector2d beforeBackdrop = new Vector2d(12, 11);
        public double[] beforeBackdropAngle = {Math.toRadians(-90), Math.toRadians(180), Math.toRadians(90)};
        public Vector2d[] backdrop = {new Vector2d(36, 38), new Vector2d(36, 36), new Vector2d(36, 32)};
        public double backdropTangent = Math.toRadians(90);

        public Vector2d[] park = {new Vector2d(60, 10), new Vector2d(60, 60)};
        public double[] parkStartingTangent ={Math.toRadians(-135), Math.toRadians(135)};
        public double[] parkEndingTangent = {Math.toRadians(0), Math.toRadians(0)};
        public double[] parkAngle = {Math.toRadians(0), Math.toRadians(0)};

    }

    public static class apriltags {
        public static Vector2d one = new Vector2d(63, 42);
        public static Vector2d two = new Vector2d(63, 36);
        public static Vector2d three = new Vector2d(63, 30);

        public static Vector2d four = new Vector2d(63, -30);
        public static Vector2d five = new Vector2d(63, -36);
        public static Vector2d six = new Vector2d(63, -42);
    }

    public static class backDropBlueRight {
        public static Vector2d left = new Vector2d(46, 42);
        public static Vector2d center = new Vector2d(46, 36);
        public static Vector2d right = new Vector2d(46, 24);

    }
    public static class backDropRedRight {

        public static Vector2d left = new Vector2d(46, -30);
        public static Vector2d center = new Vector2d(46, -36);
        public static Vector2d right = new Vector2d(46, -42);

    }
}
