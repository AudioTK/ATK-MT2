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
{
  outFilter.set_input_port(0, &inFilter, 0);

  parameters.createAndAddParameter("drywet", "Dry/Wet", "", NormalisableRange<float>(0, 100), 100, nullptr, nullptr);

  parameters.state = ValueTree(Identifier("MT2"));
}

MT2AudioProcessor::~MT2AudioProcessor()
{
}

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
  MemoryOutputStream store(destData, true);
  store.writeInt(0); // version ID
  auto str = parameters.state.toXmlString();
  store.writeString(str);
}

void MT2AudioProcessor::setStateInformation(const void* data, int sizeInBytes)
{
  MemoryInputStream store(data, static_cast<size_t>(sizeInBytes), false);
  int version = store.readInt(); // version ID
  std::unique_ptr<::juce::XmlElement> xml(::juce::XmlDocument::parse(store.readString()));
  if(xml)
  {
    parameters.state = ValueTree::fromXml(*xml);
  }
}

//==============================================================================
// This creates new instances of the plugin..
AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
  return new MT2AudioProcessor();
}
