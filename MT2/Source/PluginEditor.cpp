/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginEditor.h"
#include "PluginProcessor.h"

//==============================================================================
MT2AudioProcessorEditor::MT2AudioProcessorEditor(MT2AudioProcessor& p, juce::AudioProcessorValueTreeState& paramState)
  : juce::AudioProcessorEditor(&p)
  , processor(p)
  , paramState(paramState)
  , knob(juce::ImageFileFormat::loadFrom(BinaryData::KNB_Pitt_L_png, BinaryData::KNB_Pitt_L_pngSize), 55, 55, 101)
  , distLevel(paramState, "distLevel", "Distortion", &knob)
  , lowLevel(paramState, "lowLevel", "Low Level", &knob)
  , highLevel(paramState, "highLevel", "High Level", &knob)
  , midLevel(paramState, "midLevel", "Mid Level", &knob)
  , midFreq(paramState, "midFreq", "Mid Freq", &knob)

{
  addAndMakeVisible(distLevel);
  addAndMakeVisible(lowLevel);
  addAndMakeVisible(highLevel);
  addAndMakeVisible(midLevel);
  addAndMakeVisible(midFreq);

  bckgndImage = juce::ImageFileFormat::loadFrom(BinaryData::background_jpg, BinaryData::background_jpgSize);

  // Make sure that before the constructor has finished, you've set the
  // editor's size to whatever you need it to be.
  setSize(300, 200);
}

MT2AudioProcessorEditor::~MT2AudioProcessorEditor() = default;

void MT2AudioProcessorEditor::paint(juce::Graphics& g)
{
  g.drawImageAt(bckgndImage, 0, 0);
  g.setFont(juce::Font("Times New Roman", 30.0f, juce::Font::bold | juce::Font::italic));
  g.setColour(juce::Colours::whitesmoke);
  g.drawText("MT2", 20, 10, 200, 30, juce::Justification::verticallyCentred);
  g.setFont(juce::Font("Times New Roman", 12.0f, juce::Font::bold));
  g.drawText(
      "Dist", 20, 130, 50, 30, juce::Justification::horizontallyCentred | juce::Justification::verticallyCentred);
  g.drawText("Low", 120, 90, 50, 30, juce::Justification::horizontallyCentred | juce::Justification::verticallyCentred);
  g.drawText(
      "High", 120, 170, 50, 30, juce::Justification::horizontallyCentred | juce::Justification::verticallyCentred);
  g.drawText("Mid", 220, 90, 50, 30, juce::Justification::horizontallyCentred | juce::Justification::verticallyCentred);
  g.drawText(
      "Freq", 220, 170, 50, 30, juce::Justification::horizontallyCentred | juce::Justification::verticallyCentred);
}

void MT2AudioProcessorEditor::resized()
{
  distLevel.setBounds(20, 80, 55, 55);
  lowLevel.setBounds(120, 40, 55, 55);
  highLevel.setBounds(120, 120, 55, 55);
  midLevel.setBounds(220, 40, 55, 55);
  midFreq.setBounds(220, 120, 55, 55);
}
