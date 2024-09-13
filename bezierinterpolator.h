#ifndef BEZIERINTERPOLATOR_H
#define BEZIERINTERPOLATOR_H

#include <glib.h>

typedef struct {
    double x;
    double y;
} Point2D;

typedef struct {
    GArray *points;
} Polygon2D;

void InterpolateBezier(double x1, double y1, double x2, double y2, double x3,
                       double y3, double x4, double y4,
                       GArray *interpolatedPoints, unsigned level);


double DistanceTolerance;

#endif