#ifndef LINE_SCANNER_HPP
#define LINE_SCANNER_HPP

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

struct LineScanner;
typedef struct LineScanner LineScanner;

struct LineScanner{
    uint16_t num_points;
    uint16_t scan_cycle;
    float *angle;
    float *distance;
    uint16_t index;
    uint16_t cycle_cnt;
    void (*event_callback)(LineScanner *linescanner, float x1, float y1, float x2, float y2);
    float thickness;
    float threshold;
};

void LineScanner_Init(LineScanner *linescanner, uint16_t num_points, uint16_t scan_cycle, float thickness, float threshold, void (*event_callback)(LineScanner *linescanner, float x1, float y1, float x2, float y2));
void LineScanner_scan(LineScanner *linescanner, float angle, float distance);
void LineScanner_Deinit(LineScanner *linescanner);

#endif //LINE_SCANNER_HPP