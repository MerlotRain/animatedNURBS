#include "bezierinterpolator.h"
#include <math.h>

// Casteljau algorithm (interpolating Bezier curve) parameters.
static const unsigned curveRecursionLimit;
static const double curveCollinearityEpsilon;
static const double curveAngleToleranceEpsilon;
static const double AngleTolerance;
static const double CuspLimit;

void InterpolateBezier(double x1, double y1, double x2, double y2, double x3,
                       double y3, double x4, double y4,
                       GArray *interpolatedPoints, unsigned level) {
  if (level > curveRecursionLimit) {
    return;
  }

  // Calculate all the mid-points of the line segments
  double x12 = (x1 + x2) / 2;
  double y12 = (y1 + y2) / 2;
  double x23 = (x2 + x3) / 2;
  double y23 = (y2 + y3) / 2;
  double x34 = (x3 + x4) / 2;
  double y34 = (y3 + y4) / 2;
  double x123 = (x12 + x23) / 2;
  double y123 = (y12 + y23) / 2;
  double x234 = (x23 + x34) / 2;
  double y234 = (y23 + y34) / 2;
  double x1234 = (x123 + x234) / 2;
  double y1234 = (y123 + y234) / 2;

  if (level > 0) {
    // Enforce subdivision first time

    // Try to approximate the full cubic curve by a single straight line
    double dx = x4 - x1;
    double dy = y4 - y1;

    double d2 = fabs(((x2 - x4) * dy - (y2 - y4) * dx));
    double d3 = fabs(((x3 - x4) * dy - (y3 - y4) * dx));

    double da1, da2;

    if (d2 > curveCollinearityEpsilon && d3 > curveCollinearityEpsilon) {
      // Regular care
      if ((d2 + d3) * (d2 + d3) <= DistanceTolerance * (dx * dx + dy * dy)) {
        // If the curvature doesn't exceed the distance_tolerance value
        // we tend to finish subdivisions.
        if (AngleTolerance < curveAngleToleranceEpsilon) {
          Point2D p = {.x = x1234, .y = y1234};
          g_array_append_val(interpolatedPoints, p);
          return;
        }

        // Angle & Cusp Condition
        double a23 = atan2(y3 - y2, x3 - x2);
        da1 = fabs(a23 - atan2(y2 - y1, x2 - x1));
        da2 = fabs(atan2(y4 - y3, x4 - x3) - a23);
        if (da1 >= M_PI)
          da1 = 2 * M_PI - da1;
        if (da2 >= M_PI)
          da2 = 2 * M_PI - da2;

        if (da1 + da2 < AngleTolerance) {
          Point2D p = {.x = x1234, .y = y1234};
          g_array_append_val(interpolatedPoints, p);
          // Finally we can stop the recursion
          return;
        }

        if (CuspLimit != 0.0) {
          if (da1 > CuspLimit) {
            Point2D p = {.x = x2, .y = y2};
            g_array_append_val(interpolatedPoints, p);
            return;
          }

          if (da2 > CuspLimit) {
            Point2D p = {.x = x3, .y = y3};
            g_array_append_val(interpolatedPoints, p);
            return;
          }
        }
      }
    } else {
      if (d2 > curveCollinearityEpsilon) {
        // p1,p3,p4 are collinear, p2 is considerable
        if (d2 * d2 <= DistanceTolerance * (dx * dx + dy * dy)) {
          if (AngleTolerance < curveAngleToleranceEpsilon) {
            Point2D p = {.x = x1234, .y = y1234};
            g_array_append_val(interpolatedPoints, p);
            return;
          }

          // Angle Condition
          da1 = fabs(atan2(y3 - y2, x3 - x2) - atan2(y2 - y1, x2 - x1));
          if (da1 >= M_PI)
            da1 = 2 * M_PI - da1;

          if (da1 < AngleTolerance) {
            Point2D p1 = {.x = x2, .y = y2};
            g_array_append_val(interpolatedPoints, p1);
            Point2D p2 = {.x = x3, .y = y3};
            g_array_append_val(interpolatedPoints, p2);
            return;
          }

          if (CuspLimit != 0.0) {
            if (da1 > CuspLimit) {
              Point2D p = {.x = x2, .y = y2};
              g_array_append_val(interpolatedPoints, p);
              return;
            }
          }
        }
      } else if (d3 > curveCollinearityEpsilon) {
        // p1,p2,p4 are collinear, p3 is considerable
        if (d3 * d3 <= DistanceTolerance * (dx * dx + dy * dy)) {
          if (AngleTolerance < curveAngleToleranceEpsilon) {
            Point2D p = {.x = x1234, .y = y1234};
            g_array_append_val(interpolatedPoints, p);
            return;
          }

          // Angle Condition
          da1 = fabs(atan2(y4 - y3, x4 - x3) - atan2(y3 - y2, x3 - x2));
          if (da1 >= M_PI)
            da1 = 2 * M_PI - da1;

          if (da1 < AngleTolerance) {
            Point2D p1 = {.x = x2, .y = y2};
            g_array_append_val(interpolatedPoints, p1);
            Point2D p2 = {.x = x3, .y = y3};
            g_array_append_val(interpolatedPoints, p2);
            return;
          }

          if (CuspLimit != 0.0) {
            if (da1 > CuspLimit) {
              Point2D p = {.x = x3, .y = y3};
              g_array_append_val(interpolatedPoints, p);
              return;
            }
          }
        }
      } else {
        // Collinear case
        dx = x1234 - (x1 + x4) / 2;
        dy = y1234 - (y1 + y4) / 2;
        if (dx * dx + dy * dy <= DistanceTolerance) {
          Point2D p = {.x = x1234, .y = y1234};
          g_array_append_val(interpolatedPoints, p);
          return;
        }
      }
    }
  }

  // Continue subdivision
  InterpolateBezier(x1, y1, x12, y12, x123, y123, x1234, y1234,
                    interpolatedPoints, level + 1);
  InterpolateBezier(x1234, y1234, x234, y234, x34, y34, x4, y4,
                    interpolatedPoints, level + 1);
}