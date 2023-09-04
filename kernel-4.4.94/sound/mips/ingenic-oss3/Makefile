#
# Makefile for OSS3
#

# TODO: this is ugly, I know
ccflags-y += -I$(PWD)/sound/mips/ingenic-oss3/include
#ccflags-y += -I$(PWD)/sound/mips/ingenic-oss3/inner_codecs

ingenic-audio-objs := board/t31_platform.o inner_codecs/codec.o host/audio_dmic.o host/audio_aic.o audio_dsp.o audio_debug.o

obj-$(CONFIG_SND_INGENIC_OSS3_AUDIO) += ingenic-audio.o

