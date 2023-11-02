package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class PoseConstants {

    public static class aprilTag {
        public static Vector2d detect = new Vector2d(2, 12);
    }

    @Config
    public static class redRight {

        public static boolean backdropSide = true;

        public static Pose2d start = new Pose2d(12, -63, Math.toRadians(-90));
        public static double[] startingTangent = {Math.toRadians(45), Math.toRadians(45), Math.toRadians(45)};

        public static Vector2d[] pixel = {new Vector2d(12, -32), new Vector2d(12, -12), new Vector2d(12, -36)}; //1 (13, -32) 3 (12, -32)
        public static double[] pixelAngle = {Math.toRadians(0), Math.toRadians(90), Math.toRadians(180)};
        public static double[] pixelApproachingTangent = {Math.toRadians(135), Math.toRadians(135), Math.toRadians(135)};

        public static double[] afterPixelStartingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};
        public static double[] afterPixelEndingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};

        public static Vector2d afterPixel = new Vector2d(12, -7);;
        public static double[] afterPixelAngle = {Math.toRadians(0), Math.toRadians(90), Math.toRadians(180)};

        public static Vector2d backdrop = new Vector2d(40, -36);
        public static double backdropTangent = Math.toRadians(-90);
        public static Vector2d parkRight = new Vector2d(60, -60);
        public static Vector2d parkLeft = new Vector2d(60, -12);

    }

    @Config
    public static class blueLeft {

        public static boolean backdropSide = true;

        public static Pose2d start = new Pose2d(12, 63, Math.toRadians(90));
        public static double[] startingTangent = {Math.toRadians(-45), Math.toRadians(-45), Math.toRadians(-45)};

        public static Vector2d[] pixel = {new Vector2d(12, 36), new Vector2d(12, 12), new Vector2d(12, 36)}; // 3 (12, 32)
        public static double[] pixelAngle = {Math.toRadians(180), Math.toRadians(-90), Math.toRadians(0)};
        public static double[] pixelApproachingTangent = {Math.toRadians(-135), Math.toRadians(-135), Math.toRadians(-135)};

        public static double[] afterPixelStartingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public static double[] afterPixelEndingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public static Vector2d afterPixel = new Vector2d(12, 7);;
        public static double[] afterPixelAngle = {Math.toRadians(180), Math.toRadians(-90), Math.toRadians(0)};

        public static Vector2d backdrop = new Vector2d(40, 36);
        public static double backdropTangent = Math.toRadians(90);
        public static Vector2d parkLeft = new Vector2d(60, 60);
        public static Vector2d parkRight = new Vector2d(60, 12);

    }

    @Config
    public static class redLeft {

        public static boolean backdropSide = false;

        public static Pose2d start = new Pose2d(-36, -63, Math.toRadians(-90));
        public static double[] startingTangent = {Math.toRadians(135), Math.toRadians(135), Math.toRadians(135)};

        public static Vector2d[] pixel = {new Vector2d(-36, -32), new Vector2d(-36, -12), new Vector2d(-36, -32)};
        public static double[] pixelAngle = {Math.toRadians(0), Math.toRadians(90), Math.toRadians(180)};
        public static double[] pixelApproachingTangent = {Math.toRadians(45), Math.toRadians(45), Math.toRadians(45)};

        public static double[] afterPixelStartingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};
        public static double[] afterPixelEndingTangent = {Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};

        public static Vector2d afterPixel = new Vector2d(-36, -7);;
        public static double[] afterPixelAngle = {Math.toRadians(0), Math.toRadians(90), Math.toRadians(180)};

        public static Vector2d beforeBackdrop = new Vector2d(12, -12);
        public static Vector2d backdrop = new Vector2d(36, -36); //30, 36
        public static Vector2d backdrop6 = new Vector2d(36, -38);
        public static Vector2d backdrop4 = new Vector2d(36, -40);
        public static double backdropTangent = Math.toRadians(-90);
        public static Vector2d parkRight = new Vector2d(60, -60);
        public static Vector2d parkLeft = new Vector2d(60, -12);

    }

    @Config
    public static class blueRight {

        public static boolean backdropSide = false;

        public static Pose2d start = new Pose2d(-36, 63, Math.toRadians(90));
        public static double[] startingTangent = {Math.toRadians(-135), Math.toRadians(-135), Math.toRadians(-135)};

        public static Vector2d[] pixel = {new Vector2d(-37, 38), new Vector2d(-36, 12), new Vector2d(-36, 32)};
        public static double[] pixelAngle = {Math.toRadians(180), Math.toRadians(-90), Math.toRadians(0)};
        public static double[] pixelApproachingTangent = {Math.toRadians(-45), Math.toRadians(-45), Math.toRadians(-45)};

        public static double[] afterPixelStartingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public static double[] afterPixelEndingTangent = {Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)};
        public static Vector2d afterPixel = new Vector2d(-36, 7);;
        public static double[] afterPixelAngle = {Math.toRadians(180), Math.toRadians(-90), Math.toRadians(0)};

        public static Vector2d beforeBackdrop = new Vector2d(12, 12);
        public static Vector2d backdrop = new Vector2d(40, 36);
        public static double backdropTangent = Math.toRadians(90);
        public static Vector2d parkLeft = new Vector2d(60, 60);
        public static Vector2d parkRight = new Vector2d(60, 12);

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
