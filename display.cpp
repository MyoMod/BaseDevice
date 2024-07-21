/**
 * @file display.cpp
 * @author Leon Farchau (leon2225)
 * @brief
 * @version 0.1
 * @date 03.01.2024
 *
 * @copyright Copyright (c) 2024
 *
 */

/***********************************************************************************
 * INCLUDES
 ***********************************************************************************/
#include "display.hpp"
#include "pico_unicorn.hpp"

/***********************************************************************************
 * DEFINES
 ***********************************************************************************/

/***********************************************************************************
 * PROTOTYPES
 ***********************************************************************************/

/***********************************************************************************
 * VARIABLES
 * *********************************************************************************/
pimoroni::PicoUnicorn g_display;
float g_brightness;
uint32_t g_colors[7];
uint32_t defaultColors[] = {
    0x00ff00,
    0xffff00,
    0xff00ff,
    0x00ffff,
    0xff0000,
    0x0000ff,
    0xffffff};

/***********************************************************************************
 * FUNCTIONS
 * *********************************************************************************/
void display_init(float brightness, std::span<uint32_t, 7>* colors)
{
    g_display.init();
    g_brightness = brightness;

    display_updateColors(defaultColors);
}

void display_updateColors(std::span<uint32_t, 7> colors)
{
    for (uint32_t i = 0; i < 7; i++)
    {
        g_colors[i] = colors[i];
    }
}

void display_clearBar(uint32_t barIndex)
{
    for (uint32_t x = 0; x < 16; x++)
    {
        g_display.set_pixel(x, barIndex, 0, 0, 0);
    }
}

void display_drawBar(uint32_t barIndex, uint32_t value)
{
    assert(barIndex <= 7);

    // map 0-100 to 0-16
    uint32_t barHeight = value * 16 / 100;
    uint32_t color = g_colors[barIndex];
    uint8_t r = ((color >> 16) & 0xff) * g_brightness;
    uint8_t g = ((color >> 8) & 0xff) * g_brightness;
    uint8_t b = (color & 0xff) * g_brightness;

    for (uint32_t x = 0; x < barHeight; x++)
    {
        g_display.set_pixel(x, barIndex, r, g, b);
    }

    // clear the rest of the bar
    for (uint32_t x = barHeight; x < 16; x++)
    {
        g_display.set_pixel(x, barIndex, 0, 0, 0);
    }
}
