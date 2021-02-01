/**
 * \file LookAndFeel.cpp
 */

#include "LookAndFeel.h"

namespace ATK
{
namespace juce
{
SimpleSliderLookAndFeel& SimpleSliderLookAndFeel::get_instance()
{
  static SimpleSliderLookAndFeel instance;
  return instance;
}

void SimpleSliderLookAndFeel::drawRotarySlider(::juce::Graphics& g,
    int x,
    int y,
    int width,
    int height,
    float sliderPos,
    const float rotaryStartAngle,
    const float rotaryEndAngle,
    ::juce::Slider& slider)
{
  const float radius = std::min(width / 2, height / 2) - 2.0f;
  const float centreX = x + width * 0.5f;
  const float centreY = y + height * 0.5f;
  const float rx = centreX - radius;
  const float ry = centreY - radius;
  const float rw = radius * 2.0f;
  const float angle = rotaryStartAngle + sliderPos * (rotaryEndAngle - rotaryStartAngle);
  const bool isMouseOver = slider.isMouseOverOrDragging() && slider.isEnabled();

  if(radius > 12.0f)
  {
    if(slider.isEnabled())
      g.setColour(slider.findColour(::juce::Slider::rotarySliderFillColourId).withAlpha(isMouseOver ? 1.0f : 0.7f));
    else
      g.setColour(::juce::Colour(0x80808080));

    const float thickness = .5f;

    {
      ::juce::Path filledArc;
      filledArc.addPieSegment(rx, ry, rw, rw, rotaryStartAngle, angle, thickness);
      g.fillPath(filledArc);
    }

    if(slider.isEnabled())
      g.setColour(slider.findColour(::juce::Slider::rotarySliderOutlineColourId));
    else
      g.setColour(::juce::Colour(0x80808080));

    ::juce::Path outlineArc;
    outlineArc.addPieSegment(rx, ry, rw, rw, rotaryStartAngle, rotaryEndAngle, thickness);
    outlineArc.closeSubPath();

    g.strokePath(outlineArc, ::juce::PathStrokeType(slider.isEnabled() ? (isMouseOver ? 2.0f : 1.2f) : 0.3f));
  }
  else
  {
    if(slider.isEnabled())
      g.setColour(slider.findColour(::juce::Slider::rotarySliderFillColourId).withAlpha(isMouseOver ? 1.0f : 0.7f));
    else
      g.setColour(::juce::Colour(0x80808080));

    ::juce::Path p;
    p.addEllipse(-0.4f * rw, -0.4f * rw, rw * 0.8f, rw * 0.8f);
    ::juce::PathStrokeType(rw * 0.1f).createStrokedPath(p, p);

    p.addLineSegment(::juce::Line<float>(0.0f, 0.0f, 0.0f, -radius), rw * 0.2f);

    g.fillPath(p, ::juce::AffineTransform::rotation(angle).translated(centreX, centreY));
  }
}

DualSliderLookAndFeel& DualSliderLookAndFeel::get_instance()
{
  static DualSliderLookAndFeel instance;
  return instance;
}

void DualSliderLookAndFeel::drawRotarySlider(::juce::Graphics& g,
    int x,
    int y,
    int width,
    int height,
    float sliderPos,
    const float rotaryStartAngle,
    const float rotaryEndAngle,
    ::juce::Slider& slider)
{
  const float radius = std::min(width / 2, height / 2) - 2.0f;
  const float centreX = x + width * 0.5f;
  const float centreY = y + height * 0.5f;
  const float rx = centreX - radius;
  const float ry = centreY - radius;
  const float rw = radius * 2.0f;
  const float angle = rotaryStartAngle + sliderPos * (rotaryEndAngle - rotaryStartAngle);
  const bool isMouseOver = slider.isMouseOverOrDragging() && slider.isEnabled();

  if(radius > 12.0f)
  {
    if(slider.isEnabled())
      g.setColour(slider.findColour(::juce::Slider::rotarySliderFillColourId).withAlpha(isMouseOver ? 1.0f : 0.7f));
    else
      g.setColour(::juce::Colour(0x80808080));

    const float thickness = .5f;

    {
      ::juce::Path filledArc;
      filledArc.addPieSegment(rx, ry, rw, rw, (rotaryEndAngle + rotaryStartAngle) / 2, angle, thickness);
      g.fillPath(filledArc);
    }

    if(slider.isEnabled())
      g.setColour(slider.findColour(::juce::Slider::rotarySliderOutlineColourId));
    else
      g.setColour(::juce::Colour(0x80808080));

    ::juce::Path outlineArc;
    outlineArc.addPieSegment(rx, ry, rw, rw, rotaryStartAngle, rotaryEndAngle, thickness);
    outlineArc.closeSubPath();

    g.strokePath(outlineArc, ::juce::PathStrokeType(slider.isEnabled() ? (isMouseOver ? 2.0f : 1.2f) : 0.3f));
  }
  else
  {
    if(slider.isEnabled())
      g.setColour(slider.findColour(::juce::Slider::rotarySliderFillColourId).withAlpha(isMouseOver ? 1.0f : 0.7f));
    else
      g.setColour(::juce::Colour(0x80808080));

    ::juce::Path p;
    p.addEllipse(-0.4f * rw, -0.4f * rw, rw * 0.8f, rw * 0.8f);
    ::juce::PathStrokeType(rw * 0.1f).createStrokedPath(p, p);

    p.addLineSegment(::juce::Line<float>(0.0f, 0.0f, 0.0f, -radius), rw * 0.2f);

    g.fillPath(p, ::juce::AffineTransform::rotation(angle).translated(centreX, centreY));
  }
}
} // namespace juce
} // namespace ATK
