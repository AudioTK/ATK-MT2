/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include <atk_core/atk_core.h>
#include <atk_eq/atk_eq.h>
#include <atk_tools/atk_tools.h>

#include <memory>

//==============================================================================
/**
 */

class MTBAudioProcessor: public juce::AudioProcessor
{
public:
  //==============================================================================
  MTBAudioProcessor();
  ~MTBAudioProcessor() override;

  //==============================================================================
  void prepareToPlay(double sampleRate, int samplesPerBlock) override;
  void releaseResources() override;

#ifndef JucePlugin_PreferredChannelConfigurations
  bool isBusesLayoutSupported(const BusesLayout& layouts) const override;
#endif

  void processBlock(juce::AudioSampleBuffer&, juce::MidiBuffer&) override;

  //==============================================================================
  juce::AudioProcessorEditor* createEditor() override;
  bool hasEditor() const override;

  //==============================================================================
  const juce::String getName() const override;

  bool isMidiEffect() const override;
  bool acceptsMidi() const override;
  bool producesMidi() const override;
  double getTailLengthSeconds() const override;

  //==============================================================================
  int getNumPrograms() override;
  int getCurrentProgram() override;
  void setCurrentProgram(int index) override;
  const juce::String getProgramName(int index) override;
  void changeProgramName(int index, const juce::String& newName) override;

  //==============================================================================
  void getStateInformation(juce::MemoryBlock& destData) override;
  void setStateInformation(const void* data, int sizeInBytes) override;

private:
  static constexpr int OVERSAMPLING = 8;
  //==============================================================================
  JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MTBAudioProcessor)

  ATK::InPointerFilter<float> inFilter;
  std::unique_ptr<ATK::ModellerFilter<double>> highPassFilter;
  ATK::OversamplingFilter<double, ATK::Oversampling6points5order_8<double>> oversamplingFilter;
  std::unique_ptr<ATK::ModellerFilter<double>> preDistortionToneShapingFilter;
  std::unique_ptr<ATK::ModellerFilter<double>> bandPassFilter;
  std::unique_ptr<ATK::ModellerFilter<double>> distLevelFilter;
  std::unique_ptr<ATK::ModellerFilter<double>> distFilter;
  std::unique_ptr<ATK::ModellerFilter<double>> postDistortionToneShapingFilter;
  ATK::IIRFilter<ATK::ButterworthLowPassCoefficients<double>> lowpassFilter;
  ATK::DecimationFilter<double> decimationFilter;
  ATK::IIRFilter<ATK::ButterworthHighPassCoefficients<double>> DCFilter;
  ATK::SecondOrderSVFFilter<ATK::SecondOrderSVFBellCoefficients<double>> lowToneControlFilter;
  ATK::SecondOrderSVFFilter<ATK::SecondOrderSVFHighShelfCoefficients<double>> highToneControlFilter;
  ATK::SecondOrderSVFFilter<ATK::SecondOrderSVFBellCoefficients<double>> sweepableMidToneControlFilter;
  ATK::OutPointerFilter<float> outFilter;

  juce::AudioProcessorValueTreeState parameters;
  long sampleRate;
  int lastParameterSet;

  float old_distLevel{1000};
  float old_lowLevel{100};
  float old_highLevel{100};
  float old_midLevel{100};
  float old_midFreq{1};
  float old_lowQ{0};
  float old_highQ{0};
  float old_midQ{0};
};
