#ifndef EMU68_USB_AUDIO_POC_H
#define EMU68_USB_AUDIO_POC_H

#ifdef __cplusplus
extern "C" {
#endif

void emu68_usb_audio_init(void);
void emu68_usb_audio_cpu1_worker(void) __attribute__((noreturn));
void emu68_usb_audio_cpu3_worker(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif
