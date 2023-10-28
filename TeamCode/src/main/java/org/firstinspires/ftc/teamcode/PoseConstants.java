package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Vector;

@Config
public class PoseConstants {

    public static class aprilTag {
        public static Vector2d detect = new Vector2d(2, 12);
    }
    public static class redLeft {

        public static boolean backdropSide = false;

        public static Pose2d start = new Pose2d(-36, -63, Math.toRadians(90));
        public static Vector2d right = new Vector2d(-24, -36);
        public static Vector2d center = new Vector2d(-36, -24);
        public static Vector2d left = new Vector2d(-48, -36);
        public static double spikeMarkAngleLeft = Math.toRadians(180);
        public static double spikeMarkAngleCenter = Math.toRadians(90);
        public static double spikeMarkAngleRight = Math.toRadians(0);

        public static double afterSpikeMarkStartingTangentLeft = Math.toRadians(0);
        public static double afterSpikeMarkStartingTangentCenter = Math.toRadians(90);
        public static double afterSpikeMarkStartingTangentRight = Math.toRadians(180);
        public static double afterSpikeMarkEndingTangent = Math.toRadians(90);

        public static Vector2d afterSpikeMark = new Vector2d(-36, -12);;
        public static double afterSpikeMarkAngle = Math.toRadians(-90);

        public static Vector2d backdrop = new Vector2d(44, -36);
        public static double backdropTangent = Math.toRadians(-90);
        public static Vector2d parkRight = new Vector2d(60, -60);
        public static Vector2d parkLeft = new Vector2d(60, -12);
    }

    public static class redRight {

        public static boolean backdropSide = true;

        public static Pose2d start = new Pose2d(12, -63, Math.toRadians(90));
        public static Vector2d right = new Vector2d(24, -36);
        public static Vector2d center = new Vector2d(12, -24);
        public static Vector2d left = new Vector2d(0, -36);
        public static double spikeMarkAngleLeft = Math.toRadians(180);
        public static double spikeMarkAngleCenter = Math.toRadians(90);
        public static double spikeMarkAngleRight = Math.toRadians(0);

        public static double afterSpikeMarkStartingTangentLeft = Math.toRadians(0);
        public static double afterSpikeMarkStartingTangentCenter = Math.toRadians(90);
        public static double afterSpikeMarkStartingTangentRight = Math.toRadians(0);
        public static double afterSpikeMarkEndingTangent = Math.toRadians(0);

        public static Vector2d afterSpikeMark = new Vector2d(17, -36);;
        public static double afterSpikeMarkAngle = Math.toRadians(0);

        public static Vector2d backdrop = new Vector2d(44, -36);
        public static double backdropTangent = Math.toRadians(0);
        public static Vector2d parkRight = new Vector2d(60, -60);
        public static Vector2d parkLeft = new Vector2d(60, -12);
    }

    public static class blueLeft {

        public static boolean backdropSide = true;

        public static Pose2d start = new Pose2d(12, 63, Math.toRadians(-90));
        public static Vector2d left = new Vector2d(24, 36);
        public static Vector2d center = new Vector2d(12, 24);
        public static Vector2d right = new Vector2d(0, 36);
        public static double spikeMarkAngleLeft = Math.toRadians(0);
        public static double spikeMarkAngleCenter = Math.toRadians(-90);
        public static double spikeMarkAngleRight = Math.toRadians(180);

        public static double afterSpikeMarkStartingTangentLeft = Math.toRadians(0);
        public static double afterSpikeMarkStartingTangentCenter = Math.toRadians(90);
        public static double afterSpikeMarkStartingTangentRight = Math.toRadians(0);
        public static double afterSpikeMarkEndingTangent = Math.toRadians(0);

        public static Vector2d afterSpikeMark = new Vector2d(17, 36);;
        public static double afterSpikeMarkAngle = Math.toRadians(0);

        public static Vector2d backdrop = new Vector2d(44, 36);
        public static double backdropTangent = Math.toRadians(0);
        public static Vector2d parkLeft = new Vector2d(60, 60);
        public static Vector2d parkRight = new Vector2d(60, 12);
    }

    public static class blueRight {

        public static boolean backdropSide = false;

        public static Pose2d start = new Pose2d(-36, 63, Math.toRadians(90));
        public static double startingTangent = Math.toRadians(-90);
        public static Vector2d left = new Vector2d(-24, 36);
        public static Vector2d center = new Vector2d(-36, 24);
        public static Vector2d right = new Vector2d(-48, 36);

        public static double spikeMarkAngleLeft = Math.toRadians(180);
        public static double spikeMarkAngleCenter = Math.toRadians(90);
        public static double spikeMarkAngleRight = Math.toRadians(0);

        public static double spikeMarkApproachingTangentLeft = Math.toRadians(0);
        public static double spikeMarkApproachingTangentCenter = Math.toRadians(-90);
        public static double spikeMarkApproachingTangentRight = Math.toRadians(180);

        public static double afterSpikeMarkStartingTangentLeft = Math.toRadians(180);
        public static double afterSpikeMarkStartingTangentCenter = Math.toRadians(-90);
        public static double afterSpikeMarkStartingTangentRight = Math.toRadians(0);
        public static double afterSpikeMarkEndingTangent = Math.toRadians(-90);

        public static Vector2d afterSpikeMark = new Vector2d(-36, 10);;
        public static double afterSpikeMarkAngle = Math.toRadians(-90);

        public static Vector2d backdrop = new Vector2d(40, 36); //40, 40
        public static double backdropTangent = Math.toRadians(90);
        public static Vector2d parkLeft = new Vector2d(60, 60);
        public static Vector2d parkRight = new Vector2d(60, 12);
    }

    public static class backDropBlueRight {
        public static Vector2d left = new Vector2d(46, 31);
        public static Vector2d center = new Vector2d(46, 29);
        public static Vector2d right = new Vector2d(46, 27);
    }
    public static class backDropRedRight {
        public static Vector2d left = new Vector2d(46, -27);
        public static Vector2d center = new Vector2d(46, -29);
        public static Vector2d right = new Vector2d(46, -31);
    }
}
