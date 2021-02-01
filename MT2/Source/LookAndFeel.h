/**
 * \file LookAndFeel.h
 */

#ifndef ATKJUCECOMPONENTS_JUCE_LOOKANDFEEL
#define ATKJUCECOMPONENTS_JUCE_LOOKANDFEEL

#include <AppConfig.h>

#include <juce_gui_basics/juce_gui_basics.h>

namespace ATK
{
namespace juce
{
/// Draws a rotary slider, starting from the left to the right
class SimpleSliderLookAndFeel: public ::juce::LookAndFeel_V2
{
public:
  static SimpleSliderLookAndFeel& get_instance();

  void drawRotarySlider(::juce::Graphics& g,
      int x,
      int y,
      int width,
      int height,
      float sliderPos,
      const float rotaryStartAngle,
      const float rotaryEndAngle,
      ::juce::Slider& slider) override final;
};

/// Draws a rotary slider, starting from the middle to either side
class DualSliderLookAndFeel: public ::juce::LookAndFeel_V2
{
public:
  static DualSliderLookAndFeel& get_instance();

  void drawRotarySlider(::juce::Graphics& g,
      int x,
      int y,
      int width,
      int height,
      float sliderPos,
      const float rotaryStartAngle,
      const float rotaryEndAngle,
      ::juce::Slider& slider) override final;
};
} // namespace juce
} // namespace ATK

#endif
