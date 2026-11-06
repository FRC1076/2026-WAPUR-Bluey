// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot;

import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class PhysicalConstants {
    public static class VisionConstants {
        public static final AprilTagFieldLayout kAprilTagFieldLayout = new AprilTagFieldLayout(
            List.of(
                new AprilTag(0, new Pose3d())
            ),
            Units.feetToMeters(54),
            Units.feetToMeters(18)
        );

        public static class PhotonVision {
            public static final Vector<N3> kDefaultSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);
            public static final Vector<N3> kDefaultMultiTagStdDevs = VecBuilder.fill(0.75, 0.75, 0.75);
            public static final double kHubTagPriority = 2;

            public static enum PhotonConfig {
                // no cameras
                ;

                public final String name;
                public final Transform3d offset;
                public final Matrix<N3, N1> defaultSingleTagStdDevs;
                public final Matrix<N3, N1> defaultMultiTagStdDevs;
                public final PoseStrategy multiTagPoseStrategy;
                public final PoseStrategy singleTagPoseStrategy;
                private PhotonConfig(
                    String name, 
                    Matrix<N3, N1> defaultSingleTagStdDevs,
                    Matrix<N3, N1> defaultMultiTagStdDevs,
                    PoseStrategy multiTagPoseStrategy,
                    PoseStrategy singleTagPoseStrategy,
                    double xInch, double yInch, double zInch, 
                    double rollDeg, double pitchDeg, double yawDeg
                ) {
                    this.name = name;
                    this.offset = new Transform3d(
                        Units.inchesToMeters(xInch),
                        Units.inchesToMeters(yInch),
                        Units.inchesToMeters(zInch),
                        new Rotation3d(
                            Units.degreesToRadians(rollDeg),
                            Units.degreesToRadians(pitchDeg),
                            Units.degreesToRadians(yawDeg)
                        )
                    );
                    this.multiTagPoseStrategy = multiTagPoseStrategy;
                    this.singleTagPoseStrategy = singleTagPoseStrategy;
                    this.defaultMultiTagStdDevs = defaultMultiTagStdDevs;
                    this.defaultSingleTagStdDevs = defaultSingleTagStdDevs;
                }
            }
        }
    }

    public static class FieldConstants {
        // AprilTag related constants
        public static final int aprilTagCount = VisionConstants.kAprilTagFieldLayout.getTags().size();
        public static final double aprilTagWidth = Units.inchesToMeters(6.5);

        // Field dimensions
        public static final double fieldLength = VisionConstants.kAprilTagFieldLayout.getFieldLength();
        public static final double fieldWidth = VisionConstants.kAprilTagFieldLayout.getFieldWidth();

        /**
         * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
         */
        public static class LinesVertical {
            // TODO: Add useful vertical lines (e.g. alliance zone, center of the field)
        }

        /**
         * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
         *
         * <p>NOTE: The field element start and end are always left to right from the perspective of the
         * alliance station
         */
        public static class LinesHorizontal {
            // TODO: add any useful horizontal lines
        }
    }
}
