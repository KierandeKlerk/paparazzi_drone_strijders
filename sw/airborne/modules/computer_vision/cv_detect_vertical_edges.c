//
// Created by joep on 21-3-23.
//

#include "cv_detect_vertical_edges.h"

/**
 * @file modules/computer_vision/cv_detect_vertical_edges.h
 * uses Sobel to detect vertical edges in a region (bottom/top) of the image
 */

//copied from cv_detect_color_objects.c from tinka branch and altered first file name
#include "modules/computer_vision/cv_detect_vertical_edges.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include <std.h> //joep: alterd this one to <> instead of ""
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE //joep: unsure if this needs to stay like this
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef VERTICAL_EDGE_DETECTOR_FPS1
#define VERTICAL_EDGE_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif


//define constants here commented out was what detect_color_object had

//original begin
// Filter Settings
//uint8_t cod_lum_min1 = 0;
//uint8_t cod_lum_max1 = 0;
//uint8_t cod_cb_min1 = 0;
//uint8_t cod_cb_max1 = 0;
//uint8_t cod_cr_min1 = 0;
//uint8_t cod_cr_max1 = 0;
//uint8_t minHue1 = 0;
//uint8_t maxHue1 = 0;
//uint8_t minSat1 = 0;
//uint8_t amount_of_pixels1 = 0;
//bool cod_draw1 = false;
//
//int32_t x_c, y_c;
//
//original end

// define global variables
struct edge_object_t {
    int32_t left_pixel;
    int32_t right_pixel;
    uint32_t row;
    bool updated;
};
//bottom edges with fields see above
struct edge_object_t global_edge_objects[2]; // joep: this makes two intances of struct edge_object_t namely global_edge_objects[0] and global_edge_objects[1]

// Function
//Tinka: the input to the function is kept the same + our 4 own variables
uint32_t find_vertical_edges(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw, uint8_t amount_of_pixels);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter) // joep:filter is always 1 at this point see object_detector1
{
    //Tinka: also here our variables are added and the old ones are kept for consistency
    uint8_t minHue, maxHue;
    uint8_t minSat;
    uint8_t amount_of_pixels;

    uint8_t lum_min, lum_max;
    uint8_t cb_min, cb_max;
    uint8_t cr_min, cr_max;
    bool draw;

    switch (filter){
        //Tinka: there used to be 2 cases, but I only kept the first
        case 1:
            //Tinka: also here our variables are added and the old ones are kept for consistency
            minHue = minHue1;
            maxHue = maxHue1;
            minSat = minSat1;
            amount_of_pixels = amount_of_pixels1;

            //Tinka: old variabels
            lum_min = cod_lum_min1;
            lum_max = cod_lum_max1;
            cb_min = cod_cb_min1;
            cb_max = cod_cb_max1;
            cr_min = cod_cr_min1;
            cr_max = cod_cr_max1;
            draw = cod_draw1;
            break;
        default:
            return img;
    };


    // Filter and find centroid
    //Tinka: our variables are added to the next line where we call the 'find_object_centroid' function
    uint32_t quality = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, minHue, maxHue, minSat, amount_of_pixels);
    //Tinka: commented out the print statement because they're no longer relevant TODO: add nice new ones. USEFUL!
    //VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
    //VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
    //hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

    //Tinka: here we add our found variables to the filter. I kept the old filter names to prevent errors, though this might be nice to change sometime
    //Tinka: also commented out old unused filter variables because I lost track of where they're send of to
    pthread_mutex_lock(&mutex);
    global_filters[filter-1].color_count = quality;
    //global_filters[filter-1].x_c = left_pixel;
    //global_filters[filter-1].y_c = right_pixel;
    global_filters[filter-1].updated = true;
    pthread_mutex_unlock(&mutex);

    return img;
}

//this calls joeps function this is also done in original orange avoid reason is to probably switch between diffrent modes with the parmeter that is passed
struct image_t *object_detector_vertical_edges1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector_vertical_edges1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
    return object_detector(img, 1);
}

//init
void vertical_edge_detector_init(void)
{
    memset(global_edge_objects, 0, 2*sizeof(struct edge_object_t)); //joep: I think this creates space for the two structs
    pthread_mutex_init(&mutex, NULL);
#ifdef VERTICAL_EDGE_DETECTOR_CAMERA1
    #ifdef
    // TODO add own variables from xml file
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1 // TODO look up how this works if it needs to be drawn
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, VERTICAL_EDGE_DETECTOR_FPS1, 0);
#endif
}



/**
 * resize an image .
 * takes range of columns and rows to maintain
 * @param[in] *input The input image
 * @param[in] columns the columns to copy
 * @param[in] rows the rows to copy
 * @param[out] *output The output image\
 * most is direct copy from image.c
 */
void crop_and_greyscale_image(struct image_t *input, struct image_t *output)
{
    //buffer
    uint8_t *source = input->buf; //pointer to the input_image buffer
    uint8_t *dest = output->buf; // pointer to the output_image buffer might change this to a simple array

    // Copy the creation timestamp (stays the same)
    output->ts = input->ts;
    output->eulers = input->eulers;
    output->pprz_ts = input->pprz_ts;

    // input - output sizes !WIDTH should be equal
    uint16_t height_in = input->h;
    uint16_t width_in = input->w;
    uint16_t height_out = output->h;
    uint16_t width_out = output->w;

    // bottem rows to copy
    uint16_t rows[2];
    rows[0] = height_in - height_out; //first row to use !height out should be smaller than height in
    rows[1] = height_in;  //last row to use

    // UYVY 2 bytes per pixel 2 * width bytes per row skipping x rows is 2 * width * x, bytes
    // as it is stored as UYVY we need to skip 1 byte more to get to Y value
    source +=  2 * width_in * rows[0] + 1;

    for (int y = 0; y < height_out; y++) {
        for (int x = 0; x < width_out; x++) {

            // if y x outside skip to the next that is in range
            *dest++ = *source;    // Y
            source += 2; // skip a byte to keep
        }
    }

}

void horizontal_convolution(struct image_t *input, struct image_t *output, int8_t kernel_horizontal[3]){
    // buffers should contain arrays only holding Y value per pixel

    // input - output sizes
    uint16_t height_in = input->h;
    uint16_t width_in = input->w;

    for (int y = 0; y < height_in; y++){
        for (int x = 0; x < width_in; x++) {

            // if y x outside skip to the next that is in range
            *dest++ = *source;    // Y
            source += 2; // skip a byte to keep
        }
    };

}

void seperableconvolution(struct image_t *input, struct image_t *output, int8_t kernel_horizontal[3], int8_t, kernel_vertical[3])
{
    struct image_t intermediate_img;


}

/*
 * find_vertical_edges*
 * The code now calculates a list of the found edges containing pixel positions in columns from the left
 * and distance (for now row position from the bottom up) this code assumes images are in the right orientation
 * where row number signifies degrees up-down from the drone and columns left-right !THIS HAS NOT BEEN CONFIRMED!
 */

uint32_t find_vertical_edges(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw, uint8_t amount_of_pixels)
{
    //Variables required in function
    u_int8 rows[2];
    u_int8 cols[2];
    struct image_t grayscale;
    struct image_t sobel;

    //make this tune able? in other words make it a setting
    rows[0] = 200; // first row to include
    rows[1] = 240; // last row to use
    cols[0] = 0;   // equivalent for columns
    cols[1] = 520;


    //crop image and greyscale it saved in grayscale
    crop_and_grayscale_image(img, grayscale, rows, cols)
}

