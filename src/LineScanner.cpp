#include "ld06/LineScanner.hpp"

void LineScanner_Init(LineScanner *linescanner, uint16_t num_points, uint16_t scan_cycle, float thickness, float threshold, void (*event_callback)(LineScanner *linescanner, float x1, float y1, float x2, float y2)) {
    linescanner->num_points = num_points;
    linescanner->scan_cycle = scan_cycle;
    linescanner->index = 0;
    linescanner->cycle_cnt = 0;
    linescanner->thickness = thickness;
    linescanner->threshold = threshold;
    linescanner->event_callback = event_callback;

    linescanner->angle = (float *)malloc(sizeof(float) * num_points);
    if (linescanner->angle == NULL) {
        // メモリ確保失敗時の処理
        return;
    }

    linescanner->distance = (float *)malloc(sizeof(float) * num_points);
    if (linescanner->distance == NULL) {
        free(linescanner->angle);
        return;
    }
}

void LineScanner_scan(LineScanner *linescanner, float angle, float distance){
    linescanner->angle[linescanner->index] = angle;
    linescanner->distance[linescanner->index] = distance;
    linescanner->cycle_cnt ++;
    linescanner->cycle_cnt %= linescanner->scan_cycle;
    linescanner->index ++;
    linescanner->index %= linescanner->num_points;
    if(linescanner->cycle_cnt == 0){
        float angle1 = linescanner->angle[linescanner->index];
        float distance1 = linescanner->distance[linescanner->index];
        float angle2 = angle;
        float distance2 = distance;

        float x1 = distance1 * cos(angle1);
        float y1 = distance1 * sin(angle1);
        float x2 = distance2 * cos(angle2);
        float y2 = distance2 * sin(angle2);

        //a*x + b*y + c == 0
        float line_a = y2 - y1;
        float line_b = x1 - x2;
        float line_c = x2 * y1 - x1 * y2;

        float abs_vec = sqrt(line_a * line_a + line_b * line_b);
        line_a /= abs_vec;
        line_b /= abs_vec;
        line_c /= abs_vec;

        //r == k / cos(theta - theta0)
        float k = -line_c;
        float theta0 = atan2(line_b, line_a);

        uint16_t hit = 0;
        for (size_t i = 0; i < linescanner->num_points; i++)
        {
            float angle_ = linescanner->angle[i];
            float distance_ = linescanner->distance[i];

            if(distance_ > 0 && fabs(cos(angle_  - theta0)) > 1e-6){
                float tmp = 1 / cos(angle_ - theta0);
                float r1 = tmp * (k - linescanner->thickness / 2);
                float r2 = tmp * (k + linescanner->thickness / 2);
                if((r1 < distance_ && distance_ < r2) || (r2 < distance_ && distance_ < r1)){
                    hit ++;
                }
            }
        }
        if(hit >= linescanner->num_points * linescanner->threshold){
            linescanner->event_callback(linescanner, x1, y1, x2, y2);
        }
    }
}

void LineScanner_Deinit(LineScanner *linescanner){
    free(linescanner->angle);
    free(linescanner->distance);
}
