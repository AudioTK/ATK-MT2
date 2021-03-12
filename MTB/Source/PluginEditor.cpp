/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginEditor.h"
#include "PluginProcessor.h"

//==============================================================================
MTBAudioProcessorEditor::MTBAudioProcessorEditor(MTBAudioProcessor& p, juce::AudioProcessorValueTreeState& paramState)
  : AudioProcessorEditor(&p)
  , processor(p)
  , paramState(paramState)
  , distLevel(paramState, "distLevel", "Distortion")
  , lowLevel(paramState, "lowLevel", "Low Level")
  , highLevel(paramState, "highLevel", "High Level")
  , midLevel(paramState, "midLevel", "Mid Level")
  , midFreq(paramState, "midFreq", "Mid Freq")

{
  addAndMakeVisible(distLevel);
  addAndMakeVisible(lowLevel);
  addAndMakeVisible(highLevel);
  addAndMakeVisible(midLevel);
  addAndMakeVisible(midFreq);

  // Make sure that before the constructor has finished, you've set the
  // editor's size to whatever you need it to be.
  setSize(800, 400);
}

MTBAudioProcessorEditor::~MTBAudioProcessorEditor() = default;

void MTBAudioProcessorEditor::paint(juce::Graphics& g)
{
  // (Our component is opaque, so we must completely fill the background with a solid colour)
  g.fillAll(getLookAndFeel().findColour(juce::ResizableWindow::backgroundColourId));
  g.setFont(juce::Font("Times New Roman", 30.0f, juce::Font::bold | juce::Font::italic));
  g.setColour(juce::Colours::whitesmoke);
  g.drawText("ATK MTB", 20, 10, 200, 30, juce::Justification::verticallyCentred);
}

void MTBAudioProcessorEditor::resized()
{
  distLevel.setBoundsRelative(0. / 5, 1. / 4, 1. / 5, 3. / 4);
  lowLevel.setBoundsRelative(1. / 5, 1. / 4, 1. / 5, 3. / 4);
  highLevel.setBoundsRelative(2. / 5, 1. / 4, 1. / 5, 3. / 4);
  midLevel.setBoundsRelative(3. / 5, 1. / 4, 1. / 5, 3. / 4);
  midFreq.setBoundsRelative(4. / 5, 1. / 4, 1. / 5, 3. / 4);
}
