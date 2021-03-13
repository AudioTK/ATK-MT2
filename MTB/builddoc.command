#! /bin/sh

#shell script to automate Project build, code-signing and packaging on macos

BASEDIR=$(dirname $0)
cd $BASEDIR

#call python script to update version numbers
./update_version.py

#---------------------------------------------------------------------------------------------------------

#variables
PLUGIN_NAME=`echo | grep "define JucePlugin_Name " JuceLibraryCode/JucePluginDefines.h`
PLUGIN_NAME=${PLUGIN_NAME/\#/}
PLUGIN_NAME=${PLUGIN_NAME/define/}
PLUGIN_NAME=${PLUGIN_NAME/JucePlugin_Name/}
PLUGIN_NAME=${PLUGIN_NAME//\"}
PLUGIN_NAME=$(echo "${PLUGIN_NAME}" | tr -d '[:space:]')

# work out the paths to the binaries

asciidoctor -r asciidoctor-pdf -b pdf manual/manual.adoc -o manual/${PLUGIN_NAME}_manual.pdf

echo "done"
