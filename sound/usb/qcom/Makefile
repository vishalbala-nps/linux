snd-usb-audio-qmi-y := usb_audio_qmi_v01.o qc_audio_offload.o
obj-$(CONFIG_SND_USB_AUDIO_QMI) += snd-usb-audio-qmi.o

ifneq ($(CONFIG_SND_USB_QC_OFFLOAD_MIXER),)
snd-usb-audio-qmi-y += mixer_usb_offload.o
endif
