/*
  ==============================================================================

    This file was auto-generated!

    It contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"

#include <atk_core/atk_core.h>
#include <atk_eq/atk_eq.h>
#include <atk_tools/atk_tools.h>

#include <memory>

//==============================================================================
/**
 */
class MT2AudioProcessor: public AudioProcessor
{
public:
  //==============================================================================
  MT2AudioProcessor();
  ~MT2AudioProcessor();

  //==============================================================================
  void prepareToPlay(double sampleRate, int samplesPerBlock) override;
  void releaseResources() override;

#ifndef JucePlugin_PreferredChannelConfigurations
  bool isBusesLayoutSupported(const BusesLayout& layouts) const override;
#endif

  void processBlock(AudioSampleBuffer&, MidiBuffer&) override;

  //==============================================================================
  AudioProcessorEditor* createEditor() override;
  bool hasEditor() const override;

  //==============================================================================
  const String getName() const override;

  bool acceptsMidi() const override;
  bool producesMidi() const override;
  double getTailLengthSeconds() const override;

  //==============================================================================
  int getNumPrograms() override;
  int getCurrentProgram() override;
  void setCurrentProgram(int index) override;
  const String getProgramName(int index) override;
  void changeProgramName(int index, const String& newName) override;

  //==============================================================================
  void getStateInformation(MemoryBlock& destData) override;
  void setStateInformation(const void* data, int sizeInBytes) override;

private:
  static constexpr int OVERSAMPLING = 8;

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
  std::unique_ptr<ATK::ModellerFilter<double>> lowHighToneControlFilter;
  ATK::SecondOrderSVFFilter<ATK::SecondOrderSVFBellCoefficients<double>> sweepableMidToneControlFilter;
  ATK::OutPointerFilter<float> outFilter;

  AudioProcessorValueTreeState parameters;
  long sampleRate;
  int lastParameterSet;

  //  float old_drywet{1};
};
