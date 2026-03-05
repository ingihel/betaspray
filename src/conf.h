#pragma once

/* Camera intrinsic calibration — OV5640, 320x240
 * Calibrated with calibrate_camera.py, RMS reprojection error: 0.7909 px */

#define CALIB_WIDTH   320
#define CALIB_HEIGHT  240
#define CALIB_FX      143.755937f
#define CALIB_FY      141.176552f
#define CALIB_CX      156.018487f
#define CALIB_CY      122.142135f
#define CALIB_K1      -0.237602f
#define CALIB_K2       0.238395f
#define CALIB_P1       0.007217f
#define CALIB_P2       0.001089f
#define CALIB_K3      -0.113216f

/* Scaling K for a different resolution:
 *   scale    = new_width / CALIB_WIDTH
 *   fx_new   = CALIB_FX * scale
 *   fy_new   = CALIB_FY * scale
 *   cx_new   = CALIB_CX * scale
 *   cy_new   = CALIB_CY * scale
 *   dist coefficients (K1–K3, P1–P2) stay the same */
