/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
MT2AudioProcessor::MT2AudioProcessor()
  :
#ifndef JucePlugin_PreferredChannelConfigurations
  AudioProcessor(BusesProperties()
#  if !JucePlugin_IsMidiEffect
#    if !JucePlugin_IsSynth
                     .withInput("Input", AudioChannelSet::stereo(), true)
#    endif
                     .withOutput("Output", AudioChannelSet::stereo(), true)
#  endif
          )
  ,
#endif
  inFilter(nullptr, 1, 0, false)
  , outFilter(nullptr, 1, 0, false)
  , parameters(*this,
        nullptr,
        juce::Identifier("ATKMT2"),
        std::make_unique<juce::AudioParameterFloat>("gain", "Gain", 0.0f, 1.0f, .5f))
{
  highPassFilter->set_input_port(0, &inFilter, 0);
  oversamplingFilter.set_input_port(0, highPassFilter.get(), 0);
  preDistortionToneShapingFilter->set_input_port(0, &oversamplingFilter, 0);
  bandPassFilter->set_input_port(0, preDistortionToneShapingFilter.get(), 0);
  distLevelFilter->set_input_port(0, bandPassFilter.get(), 0);
  postDistrotionToneShapingFilter->set_input_port(0, distLevelFilter.get(), 0);
  lowpassFilter.set_input_port(0, postDistrotionToneShapingFilter.get(), 0);
  decimationFilter.set_input_port(0, &lowpassFilter, 0);
  lowHighToneControlFilter->set_input_port(0, &decimationFilter, 0);
  sweepableMidToneControlFilter.set_input_port(0, lowHighToneControlFilter.get(), 0);
  outFilter.set_input_port(0, &sweepableMidToneControlFilter, 0);

  lowpassFilter.set_cut_frequency(20000);
  lowpassFilter.set_order(6);
}

MT2AudioProcessor::~MT2AudioProcessor() = default;

//==============================================================================
const String MT2AudioProcessor::getName() const
{
  return JucePlugin_Name;
}

bool MT2AudioProcessor::acceptsMidi() const
{
#if JucePlugin_WantsMidiInput
  return true;
#else
  return false;
#endif
}

bool MT2AudioProcessor::producesMidi() const
{
#if JucePlugin_ProducesMidiOutput
  return true;
#else
  return false;
#endif
}

double MT2AudioProcessor::getTailLengthSeconds() const
{
  return 0.0;
}

int MT2AudioProcessor::getNumPrograms()
{
  return 2;
}

int MT2AudioProcessor::getCurrentProgram()
{
  return lastParameterSet;
}

void MT2AudioProcessor::setCurrentProgram(int index)
{
  /*  if(index != lastParameterSet)
    {
      lastParameterSet = index;
      if(index == 0)
      {
        const char* preset0 = "<MT2><PARAM id=\"power\" value=\"10\" /><PARAM id=\"attack\" value=\"10\" /><PARAM
    id=\"release\" value=\"10\" /> <PARAM id=\"threshold\" value=\"0\" /><PARAM id=\"slope\" value=\"2\" /><PARAM
    id=\"softness\" value=\"-2\" /><PARAM id=\"makeup\" value=\"0\" /><PARAM id=\"drywet\" value=\"100\" /></MT2>";
        XmlDocument doc(preset0);

        auto el = doc.getDocumentElement();
        parameters.state = ValueTree::fromXml(*el);
      }
      else if (index == 1)
      {
        const char* preset1 = "<MT2><PARAM id=\"power\" value=\"10\" /><PARAM id=\"attack\" value=\"10\" /><PARAM
    id=\"release\" value=\"10\" /> <PARAM id=\"threshold\" value=\"0\" /><PARAM id=\"slope\" value=\"2\" /><PARAM
    id=\"softness\" value=\"-2\" /><PARAM id=\"makeup\" value=\"0\" /><PARAM id=\"drywet\" value=\"50\" /></MT2>";
        XmlDocument doc(preset1);

        auto el = doc.getDocumentElement();
        parameters.state = ValueTree::fromXml(*el);
      }
    }*/
}

const String MT2AudioProcessor::getProgramName(int index)
{
  /*  if(index == 0)
    {
      return "Serial Compression";
    }
    else if(index == 1)
    {
      return "Parallel Compression";
    }*/
  return {};
}

void MT2AudioProcessor::changeProgramName(int index, const String& newName)
{
}

//==============================================================================
void MT2AudioProcessor::prepareToPlay(double dbSampleRate, int samplesPerBlock)
{
  sampleRate = std::lround(dbSampleRate);

  if(sampleRate != inFilter.get_output_sampling_rate())
  {
    inFilter.set_input_sampling_rate(sampleRate);
    inFilter.set_output_sampling_rate(sampleRate);
    highPassFilter->set_input_sampling_rate(sampleRate);
    highPassFilter->set_output_sampling_rate(sampleRate);
    oversamplingFilter.set_input_sampling_rate(sampleRate);
    oversamplingFilter.set_output_sampling_rate(sampleRate * OVERSAMPLING);
    preDistortionToneShapingFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    preDistortionToneShapingFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    bandPassFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    bandPassFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    distLevelFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    distLevelFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    postDistrotionToneShapingFilter->set_input_sampling_rate(sampleRate * OVERSAMPLING);
    postDistrotionToneShapingFilter->set_output_sampling_rate(sampleRate * OVERSAMPLING);
    lowpassFilter.set_input_sampling_rate(sampleRate * OVERSAMPLING);
    lowpassFilter.set_output_sampling_rate(sampleRate * OVERSAMPLING);
    decimationFilter.set_input_sampling_rate(sampleRate * OVERSAMPLING);
    decimationFilter.set_output_sampling_rate(sampleRate);
    lowHighToneControlFilter->set_input_sampling_rate(sampleRate);
    lowHighToneControlFilter->set_output_sampling_rate(sampleRate);
    sweepableMidToneControlFilter.set_input_sampling_rate(sampleRate);
    sweepableMidToneControlFilter.set_output_sampling_rate(sampleRate);
    outFilter.set_input_sampling_rate(sampleRate);
    outFilter.set_output_sampling_rate(sampleRate);
  }
  outFilter.dryrun(samplesPerBlock);
}

void MT2AudioProcessor::releaseResources()
{
  // When playback stops, you can use this as an opportunity to free up any
  // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool MT2AudioProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const
{
#  if JucePlugin_IsMidiEffect
  ignoreUnused(layouts);
  return true;
#  else
  // This is the place where you check if the layout is supported.
  if(layouts.getMainOutputChannelSet() != AudioChannelSet::stereo())
    return false;

    // This checks if the input layout matches the output layout
#    if !JucePlugin_IsSynth
  if(layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
    return false;
#    endif

  return true;
#  endif
}
#endif

void MT2AudioProcessor::processBlock(AudioSampleBuffer& buffer, MidiBuffer& midiMessages)
{
  /*  if(*parameters.getRawParameterValue ("drywet") != old_drywet)
    {
      old_drywet = *parameters.getRawParameterValue ("drywet");
      drywetFilter.set_dry(old_drywet / 100);
    }*/

  const int totalNumInputChannels = getTotalNumInputChannels();
  const int totalNumOutputChannels = getTotalNumOutputChannels();

  assert(totalNumInputChannels == totalNumOutputChannels);
  assert(totalNumOutputChannels == 1);

  inFilter.set_pointer(buffer.getReadPointer(0), buffer.getNumSamples());
  outFilter.set_pointer(buffer.getWritePointer(0), buffer.getNumSamples());

  outFilter.process(buffer.getNumSamples());
}

//==============================================================================
bool MT2AudioProcessor::hasEditor() const
{
  return true; // (change this to false if you choose to not supply an editor)
}

AudioProcessorEditor* MT2AudioProcessor::createEditor()
{
  return new MT2AudioProcessorEditor(*this, parameters);
}

//==============================================================================
void MT2AudioProcessor::getStateInformation(MemoryBlock& destData)
{
  auto state = parameters.copyState();
  std::unique_ptr<juce::XmlElement> xml(state.createXml());
  xml->setAttribute("version", "0");
  copyXmlToBinary(*xml, destData);
}

void MT2AudioProcessor::setStateInformation(const void* data, int sizeInBytes)
{
  std::unique_ptr<juce::XmlElement> xmlState(getXmlFromBinary(data, sizeInBytes));

  if(xmlState.get() != nullptr)
  {
    if(xmlState->hasTagName(parameters.state.getType()))
    {
      if(xmlState->getStringAttribute("version") == "0")
      {
        parameters.replaceState(juce::ValueTree::fromXml(*xmlState));
      }
    }
  }
}

//==============================================================================
// This creates new instances of the plugin..
AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
  return new MT2AudioProcessor();
}
