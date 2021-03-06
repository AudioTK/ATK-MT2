/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"
#include "PluginProcessor.h"

#include <ATKJUCEComponents/EQ/FrequencySelector.h>
#include <ATKJUCEComponents/Tools/DryWetFilter.h>
#include <ATKJUCEComponents/Tools/VolumeFilter.h>

//==============================================================================
/**
 */
class MTBAudioProcessorEditor: public AudioProcessorEditor
{
public:
  MTBAudioProcessorEditor(MTBAudioProcessor& p, AudioProcessorValueTreeState& paramState);
  ~MTBAudioProcessorEditor();

  //==============================================================================
  void paint(Graphics&) override;
  void resized() override;

private:
  // This reference is provided as a quick way for your editor to
  // access the processor object that created it.
  MTBAudioProcessor& processor;
  AudioProcessorValueTreeState& paramState;

  ATK::juce::DryWetFilterComponent distLevel;
  ATK::juce::VolumeFilterComponent lowLevel;
  ATK::juce::VolumeFilterComponent highLevel;
  ATK::juce::VolumeFilterComponent midLevel;
  ATK::juce::FrequencySelectorComponent midFreq;
};
