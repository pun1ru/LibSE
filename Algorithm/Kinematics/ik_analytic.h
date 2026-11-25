#ifndef IK_ANALYTIC_H
#define IK_ANALYTIC_H

#include <stdbool.h>
#include <stdint.h>

#include "dh.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IK_ANALYTIC_JOINTS       7U
#define IK_ANALYTIC_BRANCH_COUNT 8U

/* S-R-S seven-axis geometry extracted from the configured modified-DH table. */
typedef struct {
    double shoulder_m[3];
    double upper_arm_m;
    double elbow_offset_m;
    double forearm_m;
    double tool_m;
    double theta_offset_rad[IK_ANALYTIC_JOINTS];
} IkAnalyticSrsParam_t;

typedef struct {
    bool valid;
    int8_t shoulder;
    int8_t elbow;
    int8_t wrist;
} IkAnalyticBranch_t;

typedef enum {
    IK_ANALYTIC_OK = 0,
    IK_ANALYTIC_OUT_OF_REACH,
    IK_ANALYTIC_REF_DEGENERATE,
    IK_ANALYTIC_BAD_PARAM
} IkAnalyticStatus_t;

typedef struct {
    IkAnalyticStatus_t status;
    float q_deg[IK_ANALYTIC_JOINTS];
    uint8_t shoulder_singular;
    uint8_t wrist_singular;
} IkAnalyticResult_t;

bool IkAnalytic_MakeSrsParam(const DH_Param_t table[IK_ANALYTIC_JOINTS],
                             IkAnalyticSrsParam_t *out);
bool IkAnalytic_ComputeBranch(const DH_Param_t table[IK_ANALYTIC_JOINTS],
                              const float q_deg[IK_ANALYTIC_JOINTS],
                              IkAnalyticBranch_t *out);
uint8_t IkAnalytic_BranchIndex(const IkAnalyticBranch_t *branch);
void IkAnalytic_BranchFromIndex(uint8_t index, IkAnalyticBranch_t *branch);

/* target is frame 7 in the base frame, with rotations relative to the configured
 * DH zero and translations in metres. This function does not apply joint limits. */
void IkAnalytic_Solve(const IkAnalyticSrsParam_t *param,
                      const float target[4][4],
                      const IkAnalyticBranch_t *branch,
                      float psi_deg,
                      IkAnalyticResult_t *out);

void IkAnalytic_SolveAllBranches(const IkAnalyticSrsParam_t *param,
                                 const float target[4][4],
                                 float psi_deg,
                                 IkAnalyticResult_t out[IK_ANALYTIC_BRANCH_COUNT]);

/* Current arm angle psi relative to the reference arm plane. */
bool IkAnalytic_ComputeArmAngle(const DH_Param_t table[IK_ANALYTIC_JOINTS],
                                const IkAnalyticSrsParam_t *param,
                                const float q_deg[IK_ANALYTIC_JOINTS],
                                float *psi_deg);

#ifdef __cplusplus
}
#endif

#endif
