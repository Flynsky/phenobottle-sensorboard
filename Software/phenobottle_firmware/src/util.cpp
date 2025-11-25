#include <stdint.h>
#include <Arduino.h>
#include "util.h"

/**
 * Plots a buffer of float values in a terminal-like format.
 * The plot is scaled to fit within a specified height and width.
 * It calculates the min, max, average, and standard deviation of the data.
 *
 * @param buffer Pointer to the float buffer to be plotted.
 * @param length Length of the buffer.
 * @param stream Pointer to the Stream object for output (e.g., Serial).
 */
void plotBuffer(float* buffer, uint32_t length, Stream* stream) {
    // fixed terminal dimensions
    const int PLOT_HEIGHT = 15;
    const int TERMINAL_WIDTH = 80;
    const int NUM_DATA_COLUMNS = TERMINAL_WIDTH - 1;

    // Unicode box-drawing characters
    static const char* VERT = "\u2502";
    static const char* HORIZ = "\u2500";
    static const char* CORNER = "\u2514";
    static const char* SPACE = " ";

    if (length == 0) {
        return;
    }

    float global_min = buffer[0];
    float global_max = buffer[0];
    float global_avg = buffer[0];
    for (uint32_t i = 1; i < length; i++) {
        if (buffer[i] < global_min) global_min = buffer[i];
        if (buffer[i] > global_max) global_max = buffer[i];
        global_avg += buffer[i];
    }
    global_avg /= length;

    // stddev = sqrt(sum((x_i - avg)^2) / N)
    float global_stddev = 0.0f;
    for (uint32_t i = 0; i < length; i++) {
        global_stddev += (buffer[i] - global_avg) * (buffer[i] - global_avg);
    }
    global_stddev = sqrt(global_stddev / length);

    if (global_max - global_min < 1e-9) {
        global_min -= 1.0f;
        global_max += 1.0f;
    }

    // we split the data into NUM_DATA_COLUMNS buckets
    // each bucket will have a top and bottom row (min-max within the time-range of the bucket)
    // the top row is the row where the max value of the bucket is plotted
    // the bottom row is the row where the min value of the bucket is plotted
    // this ensures we don't undersample and still get a good representation
    struct Bucket {
        int top_row;
        int bottom_row;
    };
    Bucket buckets[NUM_DATA_COLUMNS];

    // for every bucket
    for (int bucket = 0; bucket < NUM_DATA_COLUMNS; bucket++) {
        // get start and end indices for the bucket
        uint32_t start = (uint32_t)(((uint64_t)bucket * length) / NUM_DATA_COLUMNS);
        uint32_t end = (uint32_t)(((uint64_t)(bucket+1) * length) / NUM_DATA_COLUMNS);

        // ensure we don't go out of bounds
        if (end > length) end = length;

        if (start >= end) {
            // if the bucket is empty, we set both rows to the middle of the plot
            buckets[bucket].top_row = PLOT_HEIGHT / 2;
            buckets[bucket].bottom_row = PLOT_HEIGHT / 2;
        } else {
            float min_val = buffer[start];
            float max_val = buffer[start];

            // find the min and max in the bucket
            for (uint32_t i = start+1; i < end; i++) {
                if (buffer[i] < min_val) min_val = buffer[i];
                if (buffer[i] > max_val) max_val = buffer[i];
            }

            // map the min and max values to the plot rows
            int top_row = static_cast<int>(0.5f + (PLOT_HEIGHT-1) * (global_max - max_val) / (global_max - global_min));
            int bottom_row = static_cast<int>(0.5f + (PLOT_HEIGHT-1) * (global_max - min_val) / (global_max - global_min));

            // ensure the rows are within bounds
            if (top_row < 0) top_row = 0;
            else if (top_row >= PLOT_HEIGHT) top_row = PLOT_HEIGHT - 1;
            if (bottom_row < 0) bottom_row = 0;
            else if (bottom_row >= PLOT_HEIGHT) bottom_row = PLOT_HEIGHT - 1;

            // ensure top_row is always above bottom_row
            if (top_row > bottom_row) {
                int temp = top_row;
                top_row = bottom_row;
                bottom_row = temp;
            }

            // store the rows in the bucket
            buckets[bucket].top_row = top_row;
            buckets[bucket].bottom_row = bottom_row;
        }
    }

    // for every line (row) in the plot
    for (int row_idx = 0; row_idx <= PLOT_HEIGHT; row_idx++) {
        // at the start of the line
        if (row_idx < PLOT_HEIGHT) {
            // if we are above the last row, print the vertical line
            stream->printf("%s", VERT);
        } else {
            // if we are at the bottom row, print the corner
            stream->printf("%s", CORNER);
        }

        // for every column in the line
        for (int col = 1; col < TERMINAL_WIDTH; col++) {
            if (row_idx == PLOT_HEIGHT) {
                // if we are at the last row, print the horizontal line
                stream->printf("%s", HORIZ);
            } else {
                // get the bucket index for the column
                int bucket_index = col - 1;
                Bucket& b = buckets[bucket_index];
                // if the current row is between the top and bottom row of the bucket, print the vertical line
                if (row_idx >= b.top_row && row_idx <= b.bottom_row) {
                    stream->printf("%s", VERT);
                } else {
                    // otherwise, print a space
                    stream->printf("%s", SPACE);
                }
            }
        }
        // done
        stream->printf("\n");
    }

    // print statistics
    stream->printf("Max: %.2f, Min: %.2f\n", global_max, global_min);
    stream->printf("Avg: %.2f\n", global_avg);
    stream->printf("StdDev: %.2f\n", global_stddev);
    stream->printf("Data points: %d\n", length);
}
