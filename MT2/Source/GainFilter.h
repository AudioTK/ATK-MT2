/**
 * \file DryWetFilter.h
 */

#ifndef ATKJUCECOMPONENTS_TOOLS_DRYWETFILTER
#define ATKJUCECOMPONENTS_TOOLS_DRYWETFILTER

#include <AppConfig.h>

#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_gui_basics/juce_gui_basics.h>

namespace ATK
{
namespace juce
{
class DryWetFilterComponent: public ::juce::Component
{
public:
  DryWetFilterComponent(::juce::AudioProcessorValueTreeState& paramState, const std::string& name);
  ~DryWetFilterComponent();

  //==============================================================================
  void paint(::juce::Graphics&) override;
  void resized() override;
  void set_color(::juce::Colour color);

private:
  ::juce::Slider levelSlider;
  ::juce::Label levelLabel;
  ::juce::Colour color;

  std::unique_ptr<::juce::AudioProcessorValueTreeState::SliderAttachment> drywetAtt;
};
} // namespace juce
} // namespace ATK

#endif
