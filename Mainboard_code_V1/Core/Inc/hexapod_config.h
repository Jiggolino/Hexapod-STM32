/**
 * @file hexapod_config.h
 * @brief Hexapod Robot Configuration - All parameters are configurable here
 * @author Converted from Python to C for STM32
 */

#ifndef HEXAPOD_CONFIG_H
#define HEXAPOD_CONFIG_H

/* ============================================
   ROBOT DIMENSIONS (in millimeters)
   ============================================ */

typedef struct {
    float front;    // Front distance of hexagon body
    float side;     // Side distance of hexagon body
    float middle;   // Middle distance from center to vertex
    float coxia;    // First leg segment length
    float femur;    // Second leg segment length
    float tibia;    // Third leg segment length
} HexapodDimensions;

/* Default dimensions */
#define DEFAULT_FRONT       100.0f
#define DEFAULT_SIDE        100.0f
#define DEFAULT_MIDDLE      100.0f
#define DEFAULT_COXIA       100.0f
#define DEFAULT_FEMUR       100.0f
#define DEFAULT_TIBIA       100.0f

/* ============================================
   ANGLE LIMITS (in degrees)
   ============================================ */

#define ALPHA_MAX_ANGLE     90.0f   // Coxia max angle
#define BETA_MAX_ANGLE      90.0f   // Femur max angle
#define GAMMA_MAX_ANGLE     90.0f   // Tibia max angle

/* ============================================
   HEXAPOD PARAMETERS
   ============================================ */

#define LEG_COUNT           6       // Number of legs
#define BODY_VERTICES       6       // Number of body vertices

/* Coxia axis angles for each leg (in degrees) */
#define COXIA_AXIS_LEG0     0.0f    // right-middle
#define COXIA_AXIS_LEG1     45.0f   // right-front
#define COXIA_AXIS_LEG2     135.0f  // left-front
#define COXIA_AXIS_LEG3     180.0f  // left-middle
#define COXIA_AXIS_LEG4     225.0f  // left-back
#define COXIA_AXIS_LEG5     315.0f  // right-back

/* ============================================
   NUMERICAL PRECISION
   ============================================ */

#define EPSILON             1e-6f   // Small value for floating point comparisons
#define ANGLE_TOLERANCE     0.01f   // Degree tolerance for angle comparisons

/* ============================================
   IK SOLVER PARAMETERS
   ============================================ */

#define IK_MAX_ITERATIONS   100
#define IK_CONVERGENCE_DIST 1e-3f   // Distance tolerance for IK

/* ============================================
   DEBUG AND PRINTING
   ============================================ */

#define DEBUG_MODE          0       // 1 to enable debug output
#define PRINT_MODEL_UPDATE  0       // 1 to print on model update

/* ============================================
   FUNCTION-LIKE MACROS FOR CONFIGURATION
   ============================================ */

#define INIT_DIMENSIONS(f, s, m, c, fe, t) { \
    .front = f, .side = s, .middle = m, \
    .coxia = c, .femur = fe, .tibia = t \
}

#define DEFAULT_DIMENSIONS INIT_DIMENSIONS( \
    DEFAULT_FRONT, DEFAULT_SIDE, DEFAULT_MIDDLE, \
    DEFAULT_COXIA, DEFAULT_FEMUR, DEFAULT_TIBIA \
)

#endif /* HEXAPOD_CONFIG_H */
