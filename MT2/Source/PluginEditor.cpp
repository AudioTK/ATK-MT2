/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginEditor.h"
#include "PluginProcessor.h"

//==============================================================================
MT2AudioProcessorEditor::MT2AudioProcessorEditor(MT2AudioProcessor& p, AudioProcessorValueTreeState& paramState)
  : AudioProcessorEditor(&p), processor(p), paramState(paramState) //, drywet(paramState, "drywet")

{
  //  addAndMakeVisible(drywet);

  // Make sure that before the constructor has finished, you've set the
  // editor's size to whatever you need it to be.
  setSize(1000, 200);
}

MT2AudioProcessorEditor::~MT2AudioProcessorEditor()
{
}

void MT2AudioProcessorEditor::paint(Graphics& g)
{
  // (Our component is opaque, so we must completely fill the background with a solid colour)
  g.fillAll(getLookAndFeel().findColour(ResizableWindow::backgroundColourId));
  g.setFont(Font("Times New Roman", 30.0f, Font::bold | Font::italic));
  g.setColour(Colours::whitesmoke);
  g.drawText("Auto Swell", 20, 10, 400, 30, Justification::verticallyCentred);
}

void MT2AudioProcessorEditor::resized()
{
  //  drywet.setBoundsRelative(7. / 8, 1. / 4, 1. / 8, 3. / 4);
}
