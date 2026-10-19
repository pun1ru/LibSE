#include "diy.h"

#include <string.h>

void DIY_Init(DIY_t *diy, const DIYConfig_t *config)
{
    memset(diy, 0, sizeof(*diy));
    diy->model = config->model;
    memcpy(diy->joint, config->joint, sizeof(diy->joint));
    diy->state = config->init_state;
}
