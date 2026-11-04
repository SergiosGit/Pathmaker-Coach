package org.firstinspires.ftc.teamcode.configuration;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;


/**
 * Game-specific configuration for FTC competitions.
 * This class handles different game layouts and field elements
 * to make the code portable between different FTC seasons.
 */
@Configurable
public class GameConfig {

    // Field Dimensions
    public static class Field {
        public static double widthInches = 144.0; // 12 feet
        public static double lengthInches = 144.0; // 12 feet
        public static double centerX = 0.0; // Field center is origin (0,0)
        public static double centerY = 0.0; // Field center is origin (0,0)
        public static double halfWidth = widthInches / 2.0; // Distance from center to edge
        public static double halfLength = lengthInches / 2.0; // Distance from center to edge
    }


    // AprilTag configuration stuff
    public static class AprilTags {

        public enum LibraryType {
            CURRENT_GAME, CUSTOM
        }

        // Set whether you want to create a custom library, or use the library from the current FTC game
        // If custom, edit the createCustomTagLibrary method below
        public static LibraryType libraryType = LibraryType.CURRENT_GAME;
        
        public static AprilTagLibrary library = libraryType == LibraryType.CUSTOM? createCustomTagLibrary(): AprilTagGameDatabase.getCurrentGameTagLibrary();
        public static AprilTagLibrary getAprilTagLibrary() { return library; }

        // Create custom tags here
        private static AprilTagLibrary createCustomTagLibrary() {
            AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();

            builder.addTag(
                    0,
                    "Tag1",
                    6,
                    new VectorF(0, 0, 0),
                    DistanceUnit.INCH,
                    new Quaternion(0F, 0F, 0F, 0F, 0)
            );

            return builder.build();
        }
    }
}
