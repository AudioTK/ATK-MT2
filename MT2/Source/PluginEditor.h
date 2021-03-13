/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"
#include "PluginProcessor.h"

#include <ATKJUCEComponents/JUCE/ImageLookAndFeel.h>
#include <ATKJUCEComponents/JUCE/Slider.h>

//==============================================================================
/**
 */
class MT2AudioProcessorEditor: public juce::AudioProcessorEditor
{
public:
  MT2AudioProcessorEditor(MT2AudioProcessor& p, juce::AudioProcessorValueTreeState& paramState);
  ~MT2AudioProcessorEditor();

  //==============================================================================
  void paint(juce::Graphics&) override;
  void resized() override;

private:
  // This reference is provided as a quick way for your editor to
  // access the processor object that created it.
  MT2AudioProcessor& processor;
  juce::AudioProcessorValueTreeState& paramState;

  ATK::juce::ImageLookAndFeel knob;
  juce::Image bckgndImage;

  ATK::juce::SliderComponent distLevel;
  ATK::juce::SliderComponent lowLevel;
  ATK::juce::SliderComponent highLevel;
  ATK::juce::SliderComponent midLevel;
  ATK::juce::SliderComponent midFreq;
};
