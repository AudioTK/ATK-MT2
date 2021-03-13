/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"
#include "PluginProcessor.h"

/*#include <ATKJUCEComponents/EQ/FrequencySelector.h>
#include <ATKJUCEComponents/Tools/DryWetFilter.h>
#include <ATKJUCEComponents/Tools/VolumeFilter.h>*/

//==============================================================================
/**
 */
class MTBAudioProcessorEditor: public juce::AudioProcessorEditor
{
public:
  MTBAudioProcessorEditor(MTBAudioProcessor&, juce::AudioProcessorValueTreeState& paramState);
  ~MTBAudioProcessorEditor() override;

  //==============================================================================
  void paint(juce::Graphics&) override;
  void resized() override;

private:
  // This reference is provided as a quick way for your editor to
  // access the processor object that created it.
  MTBAudioProcessor& processor;
  juce::AudioProcessorValueTreeState& paramState;

  /*ATK::juce::ImageLookAndFeel bigKnob;
  ATK::juce::ImageLookAndFeel smallKnob;

  ATK::juce::SliderComponent drive;
  ATK::juce::SliderComponent tone;
  ATK::juce::SliderComponent level;*/

  juce::Image bckgndImage;

  /*ATK::juce::DryWetFilterComponent distLevel;
  ATK::juce::VolumeFilterComponent lowLevel;
  ATK::juce::VolumeFilterComponent highLevel;
  ATK::juce::VolumeFilterComponent midLevel;
  ATK::juce::FrequencySelectorComponent midFreq;*/

  JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MTBAudioProcessorEditor)
};
