#ifndef ALGORITHM_H__
#define ALGORITHM_H__

#define CAMERA_ROTATE_MIN					(-45)
#define CAMERA_ROTATE_MAX					(-10)

#define CAMERA_ROTATE_DEFAULT			    (10)

#define SAMPLE_GRID						(0.1f)

#define MAX_RANSIC_LOOP_TIMES			(30)
#define RANSIC_DISTANCE_THRESHOLD		(0.1)

#define X_VALID_MIN						(0)
#define X_VALID_MAX						(8)
#define Y_VALID_MIN						(-4)
#define Y_VALID_MAX						(4)
#define Z_VALID_MIN						(-4)
#define Z_VALID_MAX						(4)



#define GRID_COE						(1)
#define PIC_GRID						(SAMPLE_GRID * GRID_COE)
#define MAP_ROW_NUM						(80)  //(X_VALID_MAX - X_VALID_MIN) / (SAMPLE_GRID * GRID_COE)
#define MAP_COL_NUM						(80)  //(Y_VALID_MAX - Y_VALID_MIN) / (SAMPLE_GRID * GRID_COE)

#define CAR_HEIGHT_MAX					(2)
#define CAR_WITH_MAX					(0.8)

#define POINT_NUM_LIMITED				(500000)

// map relative
#define POINT_ONCE_ACCU					(50)
#define MAP_VALID_ROW_NUM				(40)
#define MAP_VALID_COL_NUM				(40)

//path relative macros
#define ROW_PATH_FIND_GAP				(4)
#define COL_PATH_FIND_GAP				(4)
#define IN_LINE_THRESHOLD				(15)

#define PCD_FILE_DIR					"/home/nick/data/point/"


//car control macros
#define CAR_DRIVE_DIR					(1)	//1 for right and 0 for left

//blind aera(the bigger abs(SLOPE_POS) is, the more narrow of the FOV is)
#define X_BLIND_DIS						(0.7f)
#define SLOPE_POS						(1.0f)
#define SLOPE_NEG						(-1.0f)



//debug macros
#define DEBUG_TIME_PRINT				(1)
#define PLOT_PATH						(1)
#define FIGURE_DEBUG					(1)



#endif
