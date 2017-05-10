/**
 * @author Li Shu
 */

#pragma once
#include "Tools/Module/Module.h"
#include "Representations/Perception/ImagePreprocessing/ColorConvert.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(ColorConvertProvider,
{,
  REQUIRES(FieldDimensions),
  PROVIDES(ColorConvert),
});

class ColorConvertProvider : public ColorConvertProviderBase
{
	void update(ColorConvert& colorConvert);
};

