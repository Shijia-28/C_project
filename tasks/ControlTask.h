#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ControlTaskInit(void);
void MainTask(void);
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);
void PlotTask(void);

#ifdef __cplusplus
}
#endif

#endif
