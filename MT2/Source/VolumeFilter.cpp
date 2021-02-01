/**
 * \file VolumeFilter.cpp
 */

#include "VolumeFilter.h"

#include "../JUCE/LookAndFeel.h"

namespace ATK
{
namespace juce
{
VolumeFilterComponent::VolumeFilterComponent(::juce::AudioProcessorValueTreeState& paramState,
    const std::string& name,
    const std::string& display)
  : levelSlider(::juce::Slider::SliderStyle::Rotary, ::juce::Slider::TextEntryBoxPosition::TextBoxBelow)
  , color(::juce::Colour(36, 36, 36))
{
  addAndMakeVisible(levelSlider);
  volumeAtt.reset(new ::juce::AudioProcessorValueTreeState::SliderAttachment(paramState, name, levelSlider));
  levelSlider.setTextValueSuffix(" dB");
  levelSlider.setColour(::juce::Slider::rotarySliderFillColourId, ::juce::Colours::orange);
  levelSlider.setLookAndFeel(&SimpleSliderLookAndFeel::get_instance());

  addAndMakeVisible(levelLabel);
  levelLabel.setText(display, ::juce::NotificationType::dontSendNotification);
  levelLabel.setJustificationType(::juce::Justification::centred);

  // Make sure that before the constructor has finished, you've set the
  // editor's size to whatever you need it to be.
  setSize(150, 150);
}

VolumeFilterComponent::~VolumeFilterComponent()
{
}

void VolumeFilterComponent::paint(::juce::Graphics& g)
{
  g.setColour(color);
  g.fillRoundedRectangle(5, 5, getWidth() - 10, getHeight() - 10, 10);
}

void VolumeFilterComponent::resized()
{
  levelLabel.setBoundsRelative(0, 0.05, 1, 0.1);
  levelSlider.setBoundsRelative(0.1, 0.2, 0.8, 0.7);
}
} // namespace juce
} // namespace ATK
