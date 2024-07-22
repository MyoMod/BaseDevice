#ifndef DISPLAY_HPP
#define DISPLAY_HPP

/************************************************************
 *  INCLUDES
 ************************************************************/

#include <stdint.h>
#include <span>

/************************************************************
 *  DEFINES
 ************************************************************/

/************************************************************
 * Type definitions
 * **********************************************************/



/************************************************************
 * Functions
 * **********************************************************/

void display_init(float brightness = 0.7f, std::span<uint32_t, 7> *colors = nullptr);
void display_updateColors(std::span<uint32_t, 7> colors);
void display_drawBar(uint32_t barIndex, uint32_t value);
void display_clearBar(uint32_t barIndex);

#endif // DISPLAY_HPP
