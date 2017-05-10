/*
 * @file ColorSpaceProvider.cpp
 * @author Li Shu
 */
#include "ColorConvertProvider.h"

#include "../../../Representations/Perception/ImagePreprocessing/ColorConvert.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"

MAKE_MODULE(ColorConvertProvider, perception);
void ColorConvertProvider::update(ColorConvert & colorConvert)
{
	colorConvert.dummy = 0;
}
