
#include <Kinematic.h>

void DiffDrive_IK_Init(DiffDriveKinematic *kine, float r, float b)
{
    kine->a0 = 1.0 / r;
    kine->a1 = 0.5 * b / r;
    kine->left_wheel = 0.0;
    kine->right_wheel = 0.0;
}

void DiffDrive_IK_Compute(DiffDriveKinematic *kine, double vx, double wz)
{
    kine->left_wheel = (kine->a0 * vx) - (kine->a1 * wz);
    kine->right_wheel = (kine->a0 * vx) + (kine->a1 * wz);
}