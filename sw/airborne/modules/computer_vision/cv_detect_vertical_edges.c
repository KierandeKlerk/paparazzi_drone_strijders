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
 * Convert an image to grayscale.
 * Depending on the output type the U/V bytes are removed
 * @param[in] *input The input image (Needs to be YUV422)
 * @param[out] *output The output image
 * joep: taken from image.c no idea how to import correctly so i just coppied it
 */
void image_to_grayscale(struct image_t *input, struct image_t *output)
{
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;
    source++;

    // Copy the creation timestamp (stays the same)
    output->ts = input->ts;
    output->eulers = input->eulers;
    output->pprz_ts = input->pprz_ts;

    // Copy the pixels
    int height = output->h;
    int width = output->w;
    if (output->type == IMAGE_YUV422) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++){
                *dest++ = 127;  // U / V
                *dest++ = *source;    // Y
                source += 2;
            }
        }
    } else {
        for (int y = 0; y < height * width; y++) {
            *dest++ = *source++;    // Y
            source++;
        }
    }
}
/**
 * resize an image .
 * takes range of columns and rows to maintain
 * @param[in] *input The input image
 * @param[in] columns the columns to copy
 * @param[in] rows the rows to copy
 * @param[out] *output The output image
 */
void crop_and_greyscale_image(struct image_t *input, struct image_t *output, uint16_t columns[2], uint16_t rows[2])
{
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;
    source++; // Joep skips the first byte in de image as this it is stored UYVYUY....

    // Copy the creation timestamp (stays the same)
    output->ts = input->ts;
    output->eulers = input->eulers;
    output->pprz_ts = input->pprz_ts;

    // Copy the pixels
    int height = output->h;
    int width = output->w;

//    for (int y = 0; y < height * width; y++) {
//        *dest++ = *source++;    // Y
//        source++;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // TODO add if statement to only add rows and columns in range of parameters
            // if y x outside skip to the next that is in range
            *dest++ = *source;    // Y
            source += 2;
    }

}


uint8_t horizontalkernelconvolution(uint8_t greyscale_img, int8_t horizontal_kernel,int8_t vertical_kernel
uint8_t image_size, uint8_t kernel_size)
{
    uint8_t output_size =
}

uint8_t verticalkernelconvolution(uint8_t greyscale_img, int8_t kernel)
{}

/*
 * find_vertical_edges*
 * The code now calculates a list of the found edges containing pixel positions in columns from the left
 * and distance (for now row position from the bottom up) this code assumes images are in the right orientation
 * where row number signifies degrees up-down from the drone and columns left-right !THIS HAS NOT BEEN CONFIRMED!
 */

uint32_t find_vertical_edges(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw, uint8_t amount_of_pixels)
{
    //Variables required in function
    int row;
    int col;
    struct image_t grayscale;
    struct image_t sobel;

    //crop image only y values (greyscale)





}

