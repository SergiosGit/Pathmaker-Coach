package org.firstinspires.ftc.teamcode.configuration;

import com.acmerobotics.dashboard.config.Config;

/**
 * Game-specific configuration for FTC competitions.
 * This class handles different game layouts and field elements
 * to make the code portable between different FTC seasons.
 */
@Config
public class GameConfig {
    
    // ===== FIELD DIMENSIONS =====
    public static class Field {
        public static double widthInches = 144.0; // 12 feet
        public static double lengthInches = 144.0; // 12 feet
        public static double centerX = 0.0; // Field center is origin (0,0)
        public static double centerY = 0.0; // Field center is origin (0,0)
        public static double halfWidth = widthInches / 2.0; // Distance from center to edge
        public static double halfLength = lengthInches / 2.0; // Distance from center to edge
    }
    
    // ===== ALLIANCE STATIONS =====
    public static class AllianceStation {
        public static double redStationX = -Field.halfWidth; // Left edge of field
        public static double redStationY = -Field.halfLength; // Bottom edge of field
        public static double blueStationX = Field.halfWidth; // Right edge of field
        public static double blueStationY = -Field.halfLength; // Bottom edge of field
        
        public static double getStationX(RobotConfig.Game.Alliance alliance) {
            return alliance == RobotConfig.Game.Alliance.RED ? redStationX : blueStationX;
        }
        
        public static double getStationY(RobotConfig.Game.Alliance alliance) {
            return alliance == RobotConfig.Game.Alliance.RED ? redStationY : blueStationY;
        }
    }
    
    // ===== STARTING POSITIONS =====
    public static class StartingPositions {
        // Red Alliance starting positions (relative to field center)
        public static double redLeftX = -Field.halfWidth + 18.0; // 18 inches from left edge
        public static double redLeftY = -Field.halfLength + 18.0; // 18 inches from bottom edge
        public static double redCenterX = Field.centerX; // Center of field
        public static double redCenterY = -Field.halfLength + 18.0; // 18 inches from bottom edge
        public static double redRightX = Field.halfWidth - 18.0; // 18 inches from right edge
        public static double redRightY = -Field.halfLength + 18.0; // 18 inches from bottom edge
        
        // Blue Alliance starting positions (relative to field center)
        public static double blueLeftX = -Field.halfWidth + 18.0; // 18 inches from left edge
        public static double blueLeftY = Field.halfLength - 18.0; // 18 inches from top edge
        public static double blueCenterX = Field.centerX; // Center of field
        public static double blueCenterY = Field.halfLength - 18.0; // 18 inches from top edge
        public static double blueRightX = Field.halfWidth - 18.0; // 18 inches from right edge
        public static double blueRightY = Field.halfLength - 18.0; // 18 inches from top edge
        
        public static double getStartingX(RobotConfig.Game.Alliance alliance, RobotConfig.Game.StartingPosition position) {
            if (alliance == RobotConfig.Game.Alliance.RED) {
                switch (position) {
                    case LEFT: return redLeftX;
                    case CENTER: return redCenterX;
                    case RIGHT: return redRightX;
                }
            } else {
                switch (position) {
                    case LEFT: return blueLeftX;
                    case CENTER: return blueCenterX;
                    case RIGHT: return blueRightX;
                }
            }
            return 0.0;
        }
        
        public static double getStartingY(RobotConfig.Game.Alliance alliance, RobotConfig.Game.StartingPosition position) {
            if (alliance == RobotConfig.Game.Alliance.RED) {
                switch (position) {
                    case LEFT: return redLeftY;
                    case CENTER: return redCenterY;
                    case RIGHT: return redRightY;
                }
            } else {
                switch (position) {
                    case LEFT: return blueLeftY;
                    case CENTER: return blueCenterY;
                    case RIGHT: return blueRightY;
                }
            }
            return 0.0;
        }
    }
    
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
    
    // ===== GAME ELEMENTS =====
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
        
        // Example game element positions (relative to field center)
        public static final ElementPosition[] elementPositions = {
            new ElementPosition(Field.centerX, Field.centerY, "Center Element"),
            new ElementPosition(-Field.halfWidth + 24.0, Field.centerY, "Left Side Element"),
            new ElementPosition(Field.halfWidth - 24.0, Field.centerY, "Right Side Element"),
            // Add more elements as needed
        };
    }
    
    // ===== AUTONOMOUS PATHS =====
    public static class AutonomousPaths {
        // Define common autonomous paths that can be reused
        // across different starting positions and alliances
        
        public static class Path {
            public final String name;
            public final double[] waypoints; // [x1, y1, heading1, x2, y2, heading2, ...]
            
            public Path(String name, double[] waypoints) {
                this.name = name;
                this.waypoints = waypoints;
            }
        }
        
        // Example paths (update for current game)
        public static final Path[] predefinedPaths = {
            new Path("Forward", new double[]{0, 24, 0}),
            new Path("Left Turn", new double[]{0, 24, 90}),
            new Path("Right Turn", new double[]{0, 24, -90}),
            // Add more paths as needed
        };
        
        public static Path getPath(String pathName) {
            for (Path path : predefinedPaths) {
                if (path.name.equals(pathName)) {
                    return path;
                }
            }
            return predefinedPaths[0]; // Default to first path
        }
    }
}
