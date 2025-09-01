package org.firstinspires.ftc.teamcode.configuration;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Game-specific configuration for FTC competitions.
 * This class handles different game layouts and field elements
 * to make the code portable between different FTC seasons.
 */
@Config
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


    // Starting Positions for different OpModes
    // change how implemented later probably
    public static class StartingPositions {

        public enum Alliance {
            RED, BLUE
        }

        public enum StartingPosition {
            LEFT, RIGHT
        }

        // Red Alliance starting positions (relative to field center)
        public static double redLeftX = 0; // 18 inches from left edge
        public static double redLeftY = 0; // 18 inches from bottom edge
        public static double redCenterX = Field.centerX; // Center of field
        public static double redCenterY = -Field.halfLength + 18.0; // 18 inches from bottom edge
        public static double redRightX = 0; // 18 inches from right edge
        public static double redRightY = 0; // 18 inches from bottom edge
        
        // Blue Alliance starting positions (relative to field center)
        public static double blueLeftX = -Field.halfWidth + 18.0; // 18 inches from left edge
        public static double blueLeftY = Field.halfLength - 18.0; // 18 inches from top edge
        public static double blueCenterX = Field.centerX; // Center of field
        public static double blueCenterY = Field.halfLength - 18.0; // 18 inches from top edge
        public static double blueRightX = Field.halfWidth - 18.0; // 18 inches from right edge
        public static double blueRightY = Field.halfLength - 18.0; // 18 inches from top edge

        public static double getStartingX(Alliance alliance, StartingPosition position) {
            if (alliance == Alliance.RED) {
                switch (position) {
                    case LEFT: return redLeftX;
                    case RIGHT: return redRightX;
                }
            } else {
                switch (position) {
                    case LEFT: return blueLeftX;
                    case RIGHT: return blueRightX;
                }
            }
            return 0.0;
        }
        
        public static double getStartingY(Alliance alliance, StartingPosition position) {
            if (alliance == Alliance.RED) {
                switch (position) {
                    case LEFT: return redLeftY;
                    case RIGHT: return redRightY;
                }
            } else {
                switch (position) {
                    case LEFT: return blueLeftY;
                    case RIGHT: return blueRightY;
                }
            }
            return 0.0;
        }
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

    /*
    // ===== APRIL TAG POSITIONS =====
    public static class AprilTags {
        // Define April Tag positions for the current game
        // These should be updated for each new FTC season
        public static class TagPosition {
            public final double x, y, z, heading;
            
            public TagPosition(double x, double y, double z, double heading) {
                this.x = x;
                this.y = y;
                this.z = z;
                this.heading = heading;
            }
        }
        
        // Example April Tag positions (update for current game)
        public static final TagPosition[] tagPositions = {
            new TagPosition(0, 0, 0, 0),    // Tag 0
            new TagPosition(0, 0, 0, 0),    // Tag 1
            new TagPosition(0, 0, 0, 0),    // Tag 2
            // Add more tags as needed for the current game
        };
        
        public static TagPosition getTagPosition(int tagId) {
            if (tagId >= 0 && tagId < tagPositions.length) {
                return tagPositions[tagId];
            }
            return new TagPosition(0, 0, 0, 0);
        }
    }

     */
    
    // Positions of various game elements
    // NOT YET CONNECTED TO PATHMAKER IN ANY WAY
    public static class GameElements {
        // Define positions of game elements (cones, pixels, etc.)
        // Update these for each new FTC season
        
        public static class ElementPosition {
            public final double x, y;
            public final String description;
            
            public ElementPosition(double x, double y, String description) {
                this.x = x;
                this.y = y;
                this.description = description;
            }
        }
    }
}
