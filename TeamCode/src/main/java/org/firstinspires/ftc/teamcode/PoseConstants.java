package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class PoseConstants {


    public static class redLeft {
        public static Pose2d start = new Pose2d(-36, -63, Math.toRadians(90));
        public static Vector2d right = new Vector2d(-24, -36);
        public static Vector2d center = new Vector2d(-36, -24);
        public static Vector2d left = new Vector2d(-48, -36);
        public static double spikeMarkAngle = Math.toRadians(0);

        public static Vector2d afterSpikeMark = new Vector2d(-31, -36);;
        public static double afterSpikeMarkAngle = Math.toRadians(0);

        public static Vector2d backdrop = new Vector2d(48, -36);
        public static Vector2d parkRight = new Vector2d(60, -60);
        public static Vector2d parkLeft = new Vector2d(60, -12);
    }

    public static class redRight {
        public static Pose2d start = new Pose2d(12, -63, Math.toRadians(90));
        public static Vector2d right = new Vector2d(24, -36);
        public static Vector2d center = new Vector2d(36, -24);
        public static Vector2d left = new Vector2d(48, -36);
        public static double spikeMarkAngle = Math.toRadians(0);

        public static Vector2d afterSpikeMark = new Vector2d(17, -36);;
        public static double afterSpikeMarkAngle = Math.toRadians(0);

        public static Vector2d backdrop = new Vector2d(48, -36);
        public static Vector2d parkRight = new Vector2d(60, -60);
        public static Vector2d parkLeft = new Vector2d(60, -12);
    }

    public static class blueLeft {
        public static Pose2d start = new Pose2d(12, 63, Math.toRadians(-90));
        public static Vector2d right = new Vector2d(24, 36);
        public static Vector2d center = new Vector2d(36, 24);
        public static Vector2d left = new Vector2d(48, 36);
        public static double spikeMarkAngle = Math.toRadians(0);

        public static Vector2d afterSpikeMark = new Vector2d(17, 36);;
        public static double afterSpikeMarkAngle = Math.toRadians(0);

        public static Vector2d backdrop = new Vector2d(48, 36);
        public static Vector2d parkLeft = new Vector2d(60, 60);
        public static Vector2d parkRight = new Vector2d(60, 12);
    }

    public static class blueRight {
        public static Pose2d start = new Pose2d(-36, 63, Math.toRadians(-90));
        public static Vector2d right = new Vector2d(-24, 36);
        public static Vector2d center = new Vector2d(-36, 24);
        public static Vector2d left = new Vector2d(-48, 36);
        public static double spikeMarkAngle = Math.toRadians(0);

        public static Vector2d afterSpikeMark = new Vector2d(-31, 36);;
        public static double afterSpikeMarkAngle = Math.toRadians(0);

        public static Vector2d backdrop = new Vector2d(48, 36);
        public static Vector2d parkLeft = new Vector2d(60, 60);
        public static Vector2d parkRight = new Vector2d(60, 12);
    }

}
